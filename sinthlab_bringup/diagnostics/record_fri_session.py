#!/usr/bin/env python3
"""Record what the robot and ROS were doing when a run goes wrong -- FRI session, timing, joints.

Start it in a second terminal BEFORE launching the experiment, reproduce the problem, then Ctrl-C:

    ros2 run sinthlab_bringup record_fri_session.py              # robot namespace /lbr
    ros2 run sinthlab_bringup record_fri_session.py --out ~/fri_debug/run1

It listens to <ns>/lbr_state (every robot state the driver publishes) and /rosout (every log line),
and prints ONE line per notable event as it happens, with the time since the recorder started:

  * FRI session, connection-quality, safety, drive and control-mode changes. The cabinet leaving
    COMMANDING_ACTIVE is the usual start of an "Overrun detected" cascade.
  * gaps in the state stream longer than --gap-factor x the FRI sample time (late or dropped frames)
  * controller_manager "Overrun" warnings, with the loop time they report
  * any WARN/ERROR from any node, and trial milestones from the orchestrator
  * the commanded joint anchor jumping more than --jump-deg in one sample (what a jerk looks like)
  * the measured arm lagging the commanded anchor by more than --lag-deg
  * external joint torque above --torque-nm
  * A6 within --a6-deg of 0 (the wrist singularity)

On exit it writes fri_session.csv (one row per state sample), events.txt and summary.txt into --out,
and prints the summary: stream rate and gaps, overruns and the worst loop time, session changes, the
closest A6 came to 0, the largest commanded jump, lag and external torque.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import re
import sys
import time

# KUKA FRI SDK enum values (friClientIf.h). Raw numbers are printed alongside every name, so a
# mismatch with your SDK version is visible rather than misleading.
SESSION = {0: "IDLE", 1: "MONITORING_WAIT", 2: "MONITORING_READY", 3: "COMMANDING_WAIT", 4: "COMMANDING_ACTIVE"}
QUALITY = {0: "POOR", 1: "FAIR", 2: "GOOD", 3: "EXCELLENT"}
SAFETY = {0: "NORMAL_OPERATION", 1: "SAFETY_STOP_LEVEL_0", 2: "SAFETY_STOP_LEVEL_1", 3: "SAFETY_STOP_LEVEL_2"}
DRIVE = {0: "OFF", 1: "TRANSITIONING", 2: "ACTIVE"}
CONTROL = {0: "POSITION_CONTROL_MODE", 1: "CART_IMP_CONTROL_MODE", 2: "JOINT_IMP_CONTROL_MODE", 3: "NO_CONTROL"}

LOG_WARN, LOG_ERROR = 30, 40
ORCHESTRATOR_HINTS = ("orchestrator",)
MILESTONES = ("STARTING TRIAL", "TRIAL", "reached target joints", "physically arrive", "Threshold reached",
              "Displacement monitor activated", "Captured baseline", "FreezeAtPose", "GOAL", "Checkpoint",
              "Quiet window complete", "Perturbation")
OVERRUN_LOOP = re.compile(r"loop took ([0-9.]+) ms(?: \(missed cycles : (\d+)\))?")


def _name(table, value):
    return f"{table.get(int(value), 'UNKNOWN')}({int(value)})"


class SessionAnalyzer:
    """Turns state samples and log lines into events + a summary. No ROS in here, so it is testable."""

    STATE_FIELDS = (("session_state", "session", SESSION), ("connection_quality", "connection quality", QUALITY),
                    ("safety_state", "safety", SAFETY), ("drive_state", "drives", DRIVE),
                    ("control_mode", "control mode", CONTROL))

    def __init__(self, *, jump_deg=1.0, lag_deg=8.0, torque_nm=5.0, a6_deg=12.0, gap_factor=2.5):
        self.jump_deg, self.lag_deg, self.torque_nm, self.a6_deg, self.gap_factor = jump_deg, lag_deg, torque_nm, a6_deg, gap_factor
        self.events = []
        self._prev = None
        self._prev_t = None
        self._prev_fri = None
        self._active = {}                  # condition key -> True while it holds (so it prints on entry only)
        self.samples = 0
        self.t_first = self.t_last = None
        self.dts = []
        self.gaps = 0
        self.fri_gaps = 0
        self.overruns = 0
        self.worst_loop_ms = 0.0
        self._overruns_since_print = 0
        self._last_overrun_print = -1e9
        self.transitions = []
        self.min_a6 = (math.inf, None)
        self.max_jump = (0.0, None, None)
        self.max_lag = (0.0, None, None)
        self.max_torque = [0.0] * 7

    # ------------------------------------------------------------------ helpers
    def _event(self, t, text):
        line = f"{t:9.3f}s  {text}"
        self.events.append(line)
        return line

    def _enter(self, key, holds):
        """True only when `holds` becomes true (edge), so a persisting condition prints once."""
        was = self._active.get(key, False)
        self._active[key] = holds
        return holds and not was

    # ------------------------------------------------------------------ inputs
    def on_state(self, t, msg):
        out = []
        self.samples += 1
        self.t_first = t if self.t_first is None else self.t_first
        self.t_last = t
        nominal = float(msg.sample_time) if msg.sample_time and msg.sample_time > 0 else 0.01

        if self._prev_t is not None:
            dt = t - self._prev_t
            self.dts.append(dt)
            if dt > self.gap_factor * nominal:
                self.gaps += 1
                out.append(self._event(t, f"GAP  no robot state for {dt * 1000:.0f} ms (FRI sample time {nominal * 1000:.0f} ms)"))
        fri = msg.time_stamp_sec + msg.time_stamp_nano_sec * 1e-9
        if self._prev_fri is not None and fri > 0 and fri - self._prev_fri > self.gap_factor * nominal:
            self.fri_gaps += 1
            out.append(self._event(t, f"FRI  robot-side timestamps skipped {(fri - self._prev_fri) * 1000:.0f} ms"))

        for field, label, table in self.STATE_FIELDS:
            new = int(getattr(msg, field))
            if self._prev is None or new != int(getattr(self._prev, field)):
                old = "(start)" if self._prev is None else _name(table, getattr(self._prev, field))
                self.transitions.append((t, label, old, _name(table, new)))
                out.append(self._event(t, f"STATE {label}: {old} -> {_name(table, new)}"))

        cmd, meas, ext = list(msg.commanded_joint_position), list(msg.measured_joint_position), list(msg.external_torque)
        if self._prev is not None:
            pc = list(self._prev.commanded_joint_position)
            steps = [abs(a - b) for a, b in zip(cmd, pc) if not (math.isnan(a) or math.isnan(b))]
            if len(steps) == 7:
                i = max(range(7), key=lambda k: steps[k]); deg = math.degrees(steps[i])
                if deg > self.max_jump[0]:
                    self.max_jump = (deg, i, t)
                if self._enter("jump", deg > self.jump_deg):
                    out.append(self._event(t, f"JUMP commanded A{i + 1} moved {deg:.2f} deg in one sample (limit {self.jump_deg} deg)"))
        if not any(math.isnan(v) for v in cmd + meas):
            lags = [abs(a - b) for a, b in zip(cmd, meas)]
            i = max(range(7), key=lambda k: lags[k]); deg = math.degrees(lags[i])
            if deg > self.max_lag[0]:
                self.max_lag = (deg, i, t)
            if self._enter("lag", deg > self.lag_deg):
                out.append(self._event(t, f"LAG  measured A{i + 1} is {deg:.1f} deg from the commanded anchor"))
        if not math.isnan(meas[5]):
            a6 = math.degrees(meas[5])
            if abs(a6) < abs(self.min_a6[0]):
                self.min_a6 = (a6, t)
            if self._enter("a6", abs(a6) < self.a6_deg):
                out.append(self._event(t, f"WRIST A6 at {a6:.1f} deg -- within {self.a6_deg} deg of the wrist singularity"))
        for i, v in enumerate(ext):
            if not math.isnan(v):
                self.max_torque[i] = max(self.max_torque[i], abs(v))
                if self._enter(f"tau{i}", abs(v) > self.torque_nm):
                    out.append(self._event(t, f"TORQUE external torque on A{i + 1} is {v:+.1f} Nm (limit {self.torque_nm} Nm)"))

        self._prev, self._prev_t, self._prev_fri = msg, t, fri
        return out

    def on_log(self, t, level, name, text):
        out = []
        m = OVERRUN_LOOP.search(text) if "verrun" in text else None
        if "verrun" in text:
            if m:
                self.overruns += 1
                self.worst_loop_ms = max(self.worst_loop_ms, float(m.group(1)))
                self._overruns_since_print += 1
                if t - self._last_overrun_print >= 1.0:          # overruns can arrive at 100 Hz
                    extra = f" (+{self._overruns_since_print - 1} more in the last second)" if self._overruns_since_print > 1 else ""
                    cycles = f", missed cycles {m.group(2)}" if m.group(2) else ""
                    out.append(self._event(t, f"OVERRUN controller loop took {float(m.group(1)):.1f} ms{cycles}{extra}"))
                    self._last_overrun_print, self._overruns_since_print = t, 0
            return out                                           # "Overrun might occur ..." detail lines: counted above only
        if level >= LOG_WARN:
            tag = "ERROR" if level >= LOG_ERROR else "WARN "
            out.append(self._event(t, f"{tag} [{name}] {text[:220]}"))
        elif any(h in name for h in ORCHESTRATOR_HINTS) and any(k in text for k in MILESTONES):
            out.append(self._event(t, f"STEP [{name}] {text[:220]}"))
        return out

    # ------------------------------------------------------------------ report
    def summary(self):
        lines = ["", "================ FRI session summary ================"]
        if not self.samples:
            lines.append("No robot state received -- is the robot launched, and is --ns right?")
            return "\n".join(lines)
        dur = (self.t_last - self.t_first) or 1e-9
        d = sorted(self.dts) or [0.0]
        pct = lambda p: d[min(len(d) - 1, int(p * (len(d) - 1)))] * 1000
        lines += [
            f"state samples   {self.samples} over {dur:.1f} s = {self.samples / dur:.0f} Hz",
            f"sample spacing  median {pct(0.5):.1f} ms, 99th pct {pct(0.99):.1f} ms, worst {d[-1] * 1000:.1f} ms",
            f"stream gaps     {self.gaps} (ROS side), {self.fri_gaps} (robot timestamps)",
            f"overruns        {self.overruns}" + (f", worst loop {self.worst_loop_ms:.1f} ms" if self.overruns else ""),
            f"state changes   {len(self.transitions)}:",
        ]
        lines += [f"   {t:9.3f}s  {label}: {old} -> {new}" for t, label, old, new in self.transitions]
        if self.min_a6[1] is not None:
            lines.append(f"closest A6 to 0 {self.min_a6[0]:.1f} deg at {self.min_a6[1]:.3f} s")
        if self.max_jump[1] is not None:
            lines.append(f"largest jump    {self.max_jump[0]:.2f} deg on A{self.max_jump[1] + 1} at {self.max_jump[2]:.3f} s")
        if self.max_lag[1] is not None:
            lines.append(f"largest lag     {self.max_lag[0]:.1f} deg on A{self.max_lag[1] + 1} at {self.max_lag[2]:.3f} s")
        lines.append("max ext torque  " + "  ".join(f"A{i + 1} {v:.1f}" for i, v in enumerate(self.max_torque)) + " Nm")
        lines.append(f"events          {len(self.events)} (see events.txt)")
        return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ns", default="lbr", help="robot namespace (default: lbr)")
    ap.add_argument("--out", default=None, help="output directory (default: ./fri_debug_<date>_<time>)")
    ap.add_argument("--jump-deg", type=float, default=1.0, help="commanded joint step per sample that counts as a jump")
    ap.add_argument("--lag-deg", type=float, default=8.0, help="measured-vs-commanded joint error to report")
    ap.add_argument("--torque-nm", type=float, default=5.0, help="external joint torque to report")
    ap.add_argument("--a6-deg", type=float, default=12.0, help="report A6 within this many degrees of 0")
    ap.add_argument("--gap-factor", type=float, default=2.5, help="report state gaps longer than this x sample time")
    args, ros_args = ap.parse_known_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from lbr_fri_idl.msg import LBRState
    from rcl_interfaces.msg import Log

    out_dir = os.path.expanduser(args.out or time.strftime("fri_debug_%Y%m%d_%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    analyzer = SessionAnalyzer(jump_deg=args.jump_deg, lag_deg=args.lag_deg, torque_nm=args.torque_nm,
                               a6_deg=args.a6_deg, gap_factor=args.gap_factor)
    t0 = time.monotonic()
    csv_file = open(os.path.join(out_dir, "fri_session.csv"), "w", newline="")
    writer = csv.writer(csv_file)
    joints = [f"A{i}" for i in range(1, 8)]
    writer.writerow(["t_s", "sample_time_s", "session", "quality", "safety", "drive", "control", "tracking"]
                    + [f"cmd_{j}_deg" for j in joints] + [f"meas_{j}_deg" for j in joints] + [f"ext_{j}_Nm" for j in joints])

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = Node("fri_session_recorder")
    own = node.get_fully_qualified_name().lstrip("/")

    def on_state(msg):
        t = time.monotonic() - t0
        for line in analyzer.on_state(t, msg):
            print(line, flush=True)
        writer.writerow([f"{t:.4f}", msg.sample_time, msg.session_state, msg.connection_quality, msg.safety_state,
                         msg.drive_state, msg.control_mode, f"{msg.tracking_performance:.3f}"]
                        + [f"{math.degrees(v):.3f}" for v in msg.commanded_joint_position]
                        + [f"{math.degrees(v):.3f}" for v in msg.measured_joint_position]
                        + [f"{v:.2f}" for v in msg.external_torque])

    def on_log(msg):
        if msg.name.endswith(own) or msg.name == "fri_session_recorder":
            return
        for line in analyzer.on_log(time.monotonic() - t0, msg.level, msg.name, msg.msg):
            print(line, flush=True)

    ns = "/" + args.ns.strip("/") if args.ns.strip("/") else ""
    node.create_subscription(LBRState, f"{ns}/lbr_state", on_state, QoSProfile(depth=200))
    node.create_subscription(Log, "/rosout", on_log, QoSProfile(depth=1000))
    print(f"Recording {ns}/lbr_state and /rosout into {out_dir}  -- reproduce the problem, then Ctrl-C.", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        csv_file.close()
        summary = analyzer.summary()
        with open(os.path.join(out_dir, "events.txt"), "w") as f:
            f.write("\n".join(analyzer.events) + "\n")
        with open(os.path.join(out_dir, "summary.txt"), "w") as f:
            f.write(summary + "\n")
        print(summary)
        print(f"\nSaved: {out_dir}/fri_session.csv, events.txt, summary.txt")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
