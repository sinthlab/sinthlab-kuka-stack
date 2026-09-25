"""Per-trial data recorder -- one CSV + one JSON sidecar per trial, for every experiment.

Design and the full data dictionary: README.md section 7, Data Collected.

Used by the apple pluck, perturb and maze orchestrators. What it guarantees:

  * THE CABINET CLOCK IS RECORDED. LBRState carries the cabinet's own timestamp (fri_s / fri_ns), not
    just time.time() sampled inside a Python callback. The cabinet stamp is an exact, jitter-free
    10 ms grid (measured 2026-09-22: <0.1 ppm drift, no discontinuities) and is what trial data is
    aligned to the Blackrock NSP with.
  * FK IS DONE HERE, not read from TF, so the EE pose and the joint values in a row come from the
    SAME sample on the SAME clock rather than two differently-stamped pipelines.
  * EVENTS LIVE IN THE FILE. The snap, the checkpoint rewards, the cues -- in the CSV and, precisely
    timed, in the sidecar.
  * WRITES INCREMENTALLY, so a crash or Ctrl-C loses at most the last fraction of a second.

It is a class the orchestrator owns, not a separate node: events are direct calls with no topic hop or
serialisation to mis-time, and `robot_description` does not have to be plumbed to a second process.
Recording shares the orchestrator's executor, which is fine -- appending ~46 floats to a list is
nothing against a 10 ms budget.

Usage from an orchestrator:

    self.recorder = TrialRecorder(self, experiment="apple_pluck",
                                  extra_header=["disp_m"],
                                  extra_fn=lambda T: [self.monitor.current_disp()])
    ...
    self.recorder.start(trial_index=self.trial_count, baseline=None)
    self.recorder.mark("armed")
    self.recorder.mark("snap", disp)
    self.recorder.stop_and_save()
"""
from __future__ import annotations

import csv
import json
import os
import subprocess
import time
from typing import Callable, Optional, Sequence

import numpy as np
from rclpy.node import Node as rclpyNode

from lbr_fri_idl.msg import LBRState

# Core schema, in file order. Experiment columns are appended after these -- never substituted, so
# one loader reads every experiment. See README.md section 7, Data Collected §4 and §11.
CORE_HEADER = (
    ["t", "t_wall", "t_ros", "fri_s", "fri_ns"]
    + ["x", "y", "z", "qx", "qy", "qz", "qw"]
    + [f"meas_A{i}" for i in range(1, 8)]
    + [f"cmd_A{i}" for i in range(1, 8)]
    + [f"ext_A{i}" for i in range(1, 8)]
    + ["tracking", "session", "quality", "safety", "drive", "control"]
    + ["event", "event_arg"]
)
SCHEMA_VERSION = 1


def _git_describe(start: str) -> dict:
    """Commit SHA + dirty flag of the tree that produced the file. Never raises."""
    try:
        run = lambda *a: subprocess.run(a, cwd=start, capture_output=True, text=True, timeout=5)
        sha = run("git", "rev-parse", "--short", "HEAD").stdout.strip()
        dirty = bool(run("git", "status", "--porcelain").stdout.strip())
        return {"sha": sha or None, "dirty": dirty}
    except Exception:
        return {"sha": None, "dirty": None}


def _quat_from_matrix(m: np.ndarray) -> tuple:
    """Rotation matrix -> (x, y, z, w). Local so the recorder does not depend on scipy."""
    r = m[0:3, 0:3]
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0.0:
        s = 0.5 / np.sqrt(tr + 1.0)
        return ((r[2, 1] - r[1, 2]) * s, (r[0, 2] - r[2, 0]) * s,
                (r[1, 0] - r[0, 1]) * s, 0.25 / s)
    i = int(np.argmax([r[0, 0], r[1, 1], r[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * np.sqrt(max(1e-12, 1.0 + r[i, i] - r[j, j] - r[k, k]))
    q = [0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    return (q[0], q[1], q[2], (r[k, j] - r[j, k]) / s)


class TrialRecorder:
    def __init__(self, node: rclpyNode, *, experiment: str,
                 save_dir: str = "~/lbr-stack/src/sinthlab-kuka-stack/analysis",
                 extra_header: Optional[Sequence[str]] = None,
                 extra_fn: Optional[Callable[[np.ndarray], list]] = None,
                 session_id: Optional[str] = None,
                 on_event: Optional[Callable[[str, object], None]] = None) -> None:
        self._node = node
        self._experiment = experiment
        # One folder per launch, not a flat dump. `run_name` comes from a ROS param when the launch
        # file sets one; otherwise the node name, which is already experiment-specific.
        run = None
        if node.has_parameter("run_name"):
            run = str(node.get_parameter("run_name").value or "").strip() or None
        if not run:
            run = node.get_name().replace("_orchestrator", "")
        self._save_dir = os.path.join(os.path.expanduser(save_dir),
                                      f"expt_{run}_{time.strftime('%Y%m%d_%H%M%S')}")
        self._extra_header = list(extra_header) if extra_header else []
        self._extra_fn = extra_fn
        self._session_id = session_id or time.strftime("%Y%m%d_%H%M%S")
        # Called synchronously inside mark(), BEFORE anything else, so a hardware sync pulse leaves
        # at the same instant the event is logged. When the USB DIO arrives, pass a function that
        # pulses it: TrialRecorder(..., on_event=lambda tok, arg: dio.pulse(EVENT_CODE[tok])).
        # Emitting from here rather than from the orchestrator beside the mark() call is deliberate:
        # the pulse and the record cannot drift apart in a later edit if there is only one call site.
        self._on_event = on_event

        # FK, same construction the move actions use, so the pose is defined identically.
        import optas
        desc = str(node.get_parameter("robot_description").value) \
            if node.has_parameter("robot_description") else ""
        self._fk = optas.RobotModel(urdf_string=desc).get_link_transform_function(
            link="lbr_link_ee", base_link="lbr_link_0", numpy_output=True)

        self._active = False
        self._fh = None
        self._writer = None
        self._path: Optional[str] = None
        self._t0: Optional[float] = None
        self._rows = 0
        # A LIST, not one slot. Several marks routinely land between two 10 ms samples -- snap,
        # cue_snap and freeze all fire in the same callback -- and a single slot silently kept only
        # the last of them. They are joined with "|" into the one CSV cell they share.
        self._pending: list = []
        self._events: list = []
        self._last_msg: Optional[LBRState] = None
        self._sidecar: dict = {}
        self._subject = None
        if node.has_parameter("subject"):
            self._subject = node.get_parameter("subject").value

        node.create_subscription(LBRState, "lbr_state", self._on_state, 1)

    # ------------------------------------------------------------------ control

    def start(self, trial_index: int = 0, **sidecar) -> None:
        """Open a new file. `sidecar` holds anything constant for the trial (baseline,
        perturbation, maze_geometry) -- it is merged into the JSON verbatim."""
        if self._active:
            # Do NOT silently ignore this: that would leave the previous trial's file open and the new
            # trial appending to it -- two trials in one CSV, and no sidecar for either.
            self._node.get_logger().warn("TrialRecorder: start() while still recording; "
                                         "closing the previous trial first")
            self.stop_and_save()
        os.makedirs(self._save_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self._path = os.path.join(self._save_dir, f"robot_trajectory_{stamp}.csv")
        self._fh = open(self._path, "w", newline="")
        self._writer = csv.writer(self._fh)
        self._writer.writerow(CORE_HEADER + self._extra_header)
        self._t0 = None
        self._rows = 0
        self._pending = []
        self._events = []
        self._sidecar = {
            "schema_version": SCHEMA_VERSION,
            "experiment": self._experiment,
            "trial_index": trial_index,
            "session_id": self._session_id,
            "subject": self._subject,
            "clock_sync": {"start": self._clock_triple(), "end": None},
            "fri": self._fri_snapshot(),
            "git": _git_describe(os.path.dirname(os.path.abspath(__file__))),
            "params": self._param_dump(),
        }
        self._sidecar.update(sidecar)
        self._active = True
        # Write the sidecar NOW, not only at stop. A trial that is interrupted -- Ctrl-C, a crash,
        # an abort -- still leaves a CSV, and without this it is an orphan with no metadata and no
        # matching filename. `partial` stays true until the trial completes, so an interrupted run
        # identifies itself instead of looking like a finished one with events missing.
        self._sidecar["partial"] = True
        self._write_sidecar()
        self._node.get_logger().info(
            f"TrialRecorder: recording {self._experiment} trial {trial_index} -> "
            f"{os.path.basename(self._path)}")

    def mark(self, event: str, arg: Optional[float] = None) -> None:
        """Log an event: fire the sync pulse, record the exact moment, tag the next CSV sample.

        The CSV column is quantised to the sample grid -- the token lands on the next row, so its
        cabinet timestamp is 0-10 ms late. That is fine for reading the file but not for lining up
        against a TTL, so the PRECISE moment goes into the sidecar's event log as well. Use the CSV
        column to find the event, the sidecar to time it."""
        if not self._active:
            return
        if self._on_event is not None:
            try:
                self._on_event(event, arg)      # sync pulse first: nothing above should delay it
            except Exception as exc:
                self._node.get_logger().warn(f"TrialRecorder sync hook failed on '{event}': {exc}")
        m = self._last_msg
        self._events.append({
            "event": event, "arg": arg, "row": self._rows,
            "t": (time.time() - self._t0) if self._t0 is not None else 0.0,
            "t_wall": time.time(),
            "t_ros": self._node.get_clock().now().nanoseconds * 1e-9,
            "fri_s": int(m.time_stamp_sec) if m else None,
            "fri_ns": int(m.time_stamp_nano_sec) if m else None,
        })
        self._pending.append((event, arg))
        self._write_sidecar()   # ~10 small writes a trial; keeps the sidecar current if we are killed

    def stop_and_save(self) -> Optional[str]:
        if not self._active:
            return None
        # trial_end is marked and stop_and_save() called in the same breath, so without this the
        # final event never reaches a row -- the file closes before the next sample arrives.
        if self._pending and self._last_msg is not None:
            try:
                self._on_state(self._last_msg)
            except Exception:
                pass
        self._active = False
        self._sidecar["clock_sync"]["end"] = self._clock_triple()
        self._sidecar["partial"] = False
        self._sidecar["samples"] = self._rows
        path = self._path
        try:
            if self._fh:
                self._fh.close()
            self._write_sidecar()
            self._node.get_logger().info(
                f"TrialRecorder: {self._rows} samples -> {os.path.basename(path)} (+ .meta.json)")
        except Exception as exc:
            self._node.get_logger().error(f"TrialRecorder failed to close cleanly: {exc}")
        finally:
            self._fh, self._writer = None, None
        return path

    # ------------------------------------------------------------------ sampling

    def _on_state(self, msg: LBRState) -> None:
        self._last_msg = msg
        if not self._active or self._writer is None:
            return
        try:
            wall = time.time()
            if self._t0 is None:
                self._t0 = wall
            ros = self._node.get_clock().now().nanoseconds * 1e-9
            q = np.asarray(msg.measured_joint_position, dtype=float)
            T = self._fk(q)
            qx, qy, qz, qw = _quat_from_matrix(T)

            ev = "|".join(e for e, _ in self._pending)
            arg = "|".join("" if a is None else str(a) for _, a in self._pending)
            self._pending = []

            row = [f"{wall - self._t0:.4f}", f"{wall:.6f}", f"{ros:.6f}",
                   msg.time_stamp_sec, msg.time_stamp_nano_sec,
                   f"{T[0,3]:.6f}", f"{T[1,3]:.6f}", f"{T[2,3]:.6f}",
                   f"{qx:.6f}", f"{qy:.6f}", f"{qz:.6f}", f"{qw:.6f}"]
            row += [f"{v:.6f}" for v in msg.measured_joint_position]
            row += [f"{v:.6f}" for v in msg.commanded_joint_position]
            row += [f"{v:.3f}" for v in msg.external_torque]
            row += [f"{msg.tracking_performance:.4f}", msg.session_state, msg.connection_quality,
                    msg.safety_state, msg.drive_state, msg.control_mode, ev,
                    arg]
            if self._extra_fn is not None:
                row += list(self._extra_fn(T))

            self._writer.writerow(row)
            self._rows += 1
            # Flush on event rows so a crash cannot lose the thing that mattered.
            if ev and self._fh:
                self._fh.flush()
        except Exception:
            pass  # never take the node down over a recording failure

    def _write_sidecar(self) -> None:
        """(Re)write the sidecar beside the CSV. Always the same stem as self._path, so the two can
        never disagree -- they did once, because the sidecar was only written at stop and an
        interrupted trial left a CSV with no partner."""
        if not self._path:
            return
        try:
            self._sidecar["events"] = self._events
            with open(self._path[:-4] + ".meta.json", "w") as f:
                json.dump(self._sidecar, f, indent=2, default=str)
        except Exception as exc:
            self._node.get_logger().warn(f"TrialRecorder: sidecar write failed: {exc}")

    # ------------------------------------------------------------------ metadata

    def _clock_triple(self) -> dict:
        m = self._last_msg
        return {
            "t_wall": time.time(),
            "t_ros": self._node.get_clock().now().nanoseconds * 1e-9,
            "fri_s": int(m.time_stamp_sec) if m else None,
            "fri_ns": int(m.time_stamp_nano_sec) if m else None,
        }

    def _fri_snapshot(self) -> dict:
        m = self._last_msg
        if m is None:
            return {}
        return {"sample_time_s": float(m.sample_time), "session_state": int(m.session_state),
                "control_mode": int(m.control_mode),
                "measured_joint_rad": [float(v) for v in m.measured_joint_position],
                "commanded_joint_rad": [float(v) for v in m.commanded_joint_position]}

    def _param_dump(self) -> dict:
        """Full resolved parameter set. The orchestrators run with
        automatically_declare_parameters_from_overrides=True, so this is the whole experiment
        config with no extra plumbing."""
        out = {}
        try:
            for name in self._node._parameters:  # noqa: SLF001 - no public 'list all' in rclpy
                if name in ("robot_description", "use_sim_time"):
                    continue          # the URDF is enormous and sim_time is noise
                out[name] = self._node.get_parameter(name).value
        except Exception:
            pass
        return out
