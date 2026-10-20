#!/usr/bin/env python3
"""Reach calibration, manual: where can the monkey reach and pull the apple?

The apple-pluck "pluck" trial, re-arranged so the experimenter chooses the position and the arm stays
there between attempts:

    [first: park, then wait move_to_park.hold_sec]
    move to the TEST POSE -> quiet window -> [go cue] -> ARMED (wait for the pull)
      -> threshold: snap beep, freeze for force_release_shutdown_delay_sec -> back to the test pose
      -> next attempt (with any new test pose set meanwhile) ...

  * The test pose is move_to_start.target_joint_position, a LIVE parameter. The dashboard's "Robot
    position" panel sets it; the arm moves there at the start of the next attempt. A pose outside
    `limits` is refused when it is set, so the arm never starts towards it.
  * ~/end_attempt ends an armed attempt (he cannot reach it): the arm returns to the test pose and the
    next attempt starts, at a new test pose if one was set.
  * Pause / Stop end the attempt at once, park the arm out of reach, and hold.
  * Safety, on top of the cabinet's own: the runaway monitor (reach_safety: too far / too fast from
    the test pose), the apple leaving `limits` + pull_margin_m, and the hand force above
    limits.max_force_n. Any of them aborts the attempt, parks the arm and pauses the run.

Publishes <ns>/reach_calib_pose (std_msgs/String, JSON, 10 Hz) for the dashboard: measured joints, the
apple position, the test pose, the hand force. Every attempt is recorded as a trial (experiment
"reach_calib") and as one row of reach_calib_attempts.csv in the run folder.
"""

import csv
import json
import math
import os
import time

import numpy as np
import optas
import rclpy
from rclpy.node import Node as rclpyNode
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from std_srvs.srv import Trigger
from lbr_fri_idl.msg import LBRState

from sinthlab_bringup.actions.move_to_position_joint_space import MoveToPositionJointSpace
from sinthlab_bringup.actions.park_move import ParkMove
from sinthlab_bringup.actions.cartesian_impedance_displacement_monitor import CartesianImpedanceDisplacementMonitor
from sinthlab_bringup.actions.safety_stop_monitor import SafetyStopMonitor
from sinthlab_bringup.actions.audio_cue import AudioCue
from sinthlab_bringup.actions.visual_cue import VisualCue
from sinthlab_bringup.actions.wait_action import WaitAction
from sinthlab_bringup.actions.freeze_at_pose import FreezeAtPoseAction
from sinthlab_bringup.actions.resistance_softener import ResistanceSoftener
from sinthlab_bringup.actions.trial_recorder import TrialRecorder
from sinthlab_bringup.helpers.node_utils import get_required_param, get_launch_param
from sinthlab_bringup.helpers.experiment_control import ExperimentControl
from sinthlab_bringup.helpers.hand_force import HandForce
from sinthlab_bringup.helpers.smartpad_link import SmartPadLink

START = "move_to_start.target_joint_position"
POSE_TOPIC = "reach_calib_pose"
POSE_HZ = 10.0
GUARD_HZ = 100.0
FORCE_TRIP_SAMPLES = 5           # 50 ms above limits.max_force_n at GUARD_HZ
FORCE_JACOBIAN_DAMPING = 0.02    # as the grasp detector: the arm is still and well conditioned here
ATTEMPTS_CSV = "reach_calib_attempts.csv"
# States in which the monkey may be touching the apple: the box and force limits are watched.
GUARDED = ("quiet", "armed", "snapped")


class ReachCalibOrchestratorNode(rclpyNode):

    def __init__(self) -> None:
        super().__init__("reach_calib_orchestrator", automatically_declare_parameters_from_overrides=True)
        AudioCue.warmup(self)
        VisualCue.warmup(self)

        self.trial_count = 0
        self._state = "starting"       # starting | parking | at_park | moving | quiet | armed | snapped | returning | holding
        self._stopping = False
        self._in_trial = False
        self._returning = False
        self._message = ""
        self._reset_attempt()
        self._q_meas = None
        self._tau_ext = None

        # Kinematics: the same model the move actions and the recorder use.
        desc = str(get_launch_param(self, "robot_description", ""))
        self._fk = optas.RobotModel(urdf_string=desc).get_link_transform_function(
            link="lbr_link_ee", base_link="lbr_link_0", numpy_output=True)
        self._hand = HandForce(desc, "lbr_link_ee", FORCE_JACOBIAN_DAMPING)
        self._apple_offset = np.array(get_required_param(self, "apple_offset_m"), dtype=float)
        self._read_limits()
        self._set_test_pose([float(v) for v in get_required_param(self, START)])
        problem = self._check_target(self._test_pose_deg)
        if problem:
            raise ValueError(f"{START}: {problem}")

        # Actions -- the apple-pluck set, plus restricted plane's runaway monitor.
        self.move_to_start = MoveToPositionJointSpace(self, param_prefix="move_to_start",
                                                      on_complete=self.on_move_complete)
        self.quiet_window = WaitAction(self, duration_sec=float(get_required_param(self, "quiet_window_sec")),
                                       on_complete=self.on_quiet_window_complete, name="quiet_window")
        self.reach_timeout = WaitAction(self, duration_sec=1.0, on_complete=self.on_reach_timeout,
                                        name="reach_timeout")
        self.audio_cue = AudioCue(self, param_prefix="audio_cue_play", on_complete=self.on_audio_complete,
                                  on_finished=lambda secs: self.recorder.mark("cue_audio_end", round(secs, 4)))
        self.audio_cue_snap = AudioCue(self, param_prefix="audio_cue_snap", on_complete=lambda: None)
        self.visual_cue = VisualCue(self, label="play", on_complete=lambda: None)
        self.visual_cue_snap = VisualCue(self, label="snap", on_complete=lambda: None)
        self.monitor = CartesianImpedanceDisplacementMonitor(
            self, param_prefix="apple_pluck_impedance_control_displacement",
            on_complete=self.on_monitor_complete, on_snap=self.on_monitor_snap, on_armed=self.on_monitor_armed)
        self.freeze_hold = FreezeAtPoseAction(self)
        self.resistance = ResistanceSoftener(self, param_prefix="resistance")
        self.safety = SafetyStopMonitor(self, param_prefix="reach_safety", on_trip=self.on_safety_trip)
        self.move_park = ParkMove(self, on_complete=self.on_park_complete)
        self._read_live()

        # Dashboard / CLI control, as every experiment (helpers/experiment_control.py).
        self.control = ExperimentControl(self, "reach_calib")
        self.control.on_reload(self._reload_live)
        self.control.on_stop_request(self._stop_now)
        self.control.on_pause_request(self._on_pause_request)
        # Added AFTER ExperimentControl, so rclpy runs it FIRST: a test pose outside the limits is
        # rejected before ExperimentControl queues it for the next attempt.
        self.add_on_set_parameters_callback(self._validate_params)

        self.recorder = TrialRecorder(
            self, experiment="reach_calib",
            extra_header=["disp_m"],
            extra_fn=lambda _T: [f"{self.monitor.current_disp():.6f}"],
            on_event=self.control.on_event,
        )
        VisualCue.set_result_sink(lambda lbl, ok, ms: self.recorder.mark("cue_visual_ack", round(ms, 1)))
        self.control.apply_immediately({
            "move_to_start.speed_scale": self.move_to_start.set_speed_scale,
            "move_to_park.speed_scale": self.move_park.set_speed_scale,
        }, log=self.recorder.note_change)
        for move in (self.move_to_start, self.move_park):
            move.report_moves_to(self.recorder.note_move)

        # Inputs and outputs for the dashboard.
        self.create_subscription(LBRState, "lbr_state", self._on_state, 1)
        self.create_service(Trigger, "~/end_attempt", self._srv_end_attempt)
        self._pose_pub = self.create_publisher(String, POSE_TOPIC, 10)
        self.create_timer(1.0 / POSE_HZ, self._publish_pose)
        self.create_timer(1.0 / GUARD_HZ, self._guard_step)

        self.get_logger().info(
            f"=== REACH CALIBRATION (manual) INITIALIZED: test pose {self._test_pose_deg} deg, "
            f"apple ~{self._fmt_xyz(self._test_apple)} m ===")
        self._state = "parking"
        self.control.on_event("parking")
        self.move_park.start_first(lambda: self.control.begin_trial(self._from_park))

    # ------------------------------------------------------------------ geometry and limits

    def _apple_xyz(self, q_rad) -> np.ndarray:
        """The apple centre in lbr_link_0 [m]: the flange plus apple_offset_m in the flange frame."""
        T = np.asarray(self._fk(np.asarray(q_rad, dtype=float)), dtype=float)
        return T[0:3, 3] + T[0:3, 0:3] @ self._apple_offset

    @staticmethod
    def _fmt_xyz(p) -> str:
        return "(" + ", ".join(f"{v:.3f}" for v in p) + ")"

    def _read_limits(self) -> None:
        g = lambda n: [float(v) for v in get_required_param(self, "limits." + n)]   # noqa: E731
        self._jmin, self._jmax = np.array(g("joint_min_deg")), np.array(g("joint_max_deg"))
        self._box_min, self._box_max = np.array(g("apple_box_min_m")), np.array(g("apple_box_max_m"))
        if self._jmin.shape != (7,) or self._jmax.shape != (7,):
            raise ValueError("limits.joint_min_deg and limits.joint_max_deg need 7 values each (A1..A7)")
        if self._box_min.shape != (3,) or self._box_max.shape != (3,):
            raise ValueError("limits.apple_box_min_m and limits.apple_box_max_m need 3 values each (x, y, z)")
        if np.any(self._jmin >= self._jmax) or np.any(self._box_min >= self._box_max):
            raise ValueError("limits: every minimum must be below its maximum")
        self._pull_margin = float(get_required_param(self, "limits.pull_margin_m"))
        self._max_force = float(get_required_param(self, "limits.max_force_n"))

    def _outside_box(self, p, margin: float):
        """None if the point p is inside the apple box grown by `margin`, else which side it left."""
        for i, axis in enumerate("xyz"):
            lo, hi = self._box_min[i] - margin, self._box_max[i] + margin
            if p[i] < lo:
                return f"{axis} = {p[i]:.3f} m < {lo:.3f} m"
            if p[i] > hi:
                return f"{axis} = {p[i]:.3f} m > {hi:.3f} m"
        return None

    def _check_target(self, q_deg):
        """None if q_deg is an allowed test pose, else the reason."""
        try:
            q = np.array([float(v) for v in q_deg])
        except (TypeError, ValueError):
            return "needs 7 joint angles in degrees"
        if q.shape != (7,):
            return "needs 7 joint angles in degrees"
        for i in range(7):
            if not self._jmin[i] <= q[i] <= self._jmax[i]:
                return (f"A{i + 1} = {q[i]:g} deg is outside the allowed range "
                        f"[{self._jmin[i]:g}, {self._jmax[i]:g}] deg (limits.joint_min_deg / joint_max_deg)")
        p = self._apple_xyz(np.radians(q))
        side = self._outside_box(p, 0.0)
        if side:
            return (f"the apple would be at {self._fmt_xyz(p)} m, outside the allowed box: {side} "
                    f"(limits.apple_box_min_m / apple_box_max_m)")
        return None

    def _validate_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name == START and self.has_parameter(START):
                problem = self._check_target(list(p.value))
                if problem:
                    self.get_logger().warn(f"New test pose refused: {problem}")
                    return SetParametersResult(successful=False, reason=f"test pose refused: {problem}")
        return SetParametersResult(successful=True)

    def _set_test_pose(self, q_deg) -> None:
        self._test_pose_deg = [float(v) for v in q_deg]
        self._test_apple = self._apple_xyz(np.radians(self._test_pose_deg))

    # ------------------------------------------------------------------ robot state, force, guards

    def _on_state(self, msg: LBRState) -> None:
        q = np.array(msg.measured_joint_position.tolist(), dtype=float)
        tau = np.array(msg.external_torque.tolist(), dtype=float)
        if q.shape == (7,) and np.all(np.isfinite(q)):
            self._q_meas = q
            self._tau_ext = tau

    def _raw_force(self) -> np.ndarray:
        if self._q_meas is None or self._tau_ext is None:
            return np.zeros(3)
        return self._hand.force(self._tau_ext, self._q_meas)

    def _guard_step(self) -> None:
        """The box and force limits, while the monkey may be touching the apple."""
        if self._state not in GUARDED or self._q_meas is None:
            return
        p = self._apple_xyz(self._q_meas)
        side = self._outside_box(p, self._pull_margin)
        if side:
            self._abort(f"the apple left the allowed box by more than {self._pull_margin:g} m ({side})")
            return
        d = self.monitor.current_disp()
        if math.isfinite(d):
            self._max_disp = max(self._max_disp, d)
        if self._t_armed is None:
            return
        f = float(np.linalg.norm(self._raw_force() - self._force_tare))
        self._force_now = f
        self._peak_force = max(self._peak_force, f)
        if self._max_force > 0.0:
            self._force_over = self._force_over + 1 if f > self._max_force else 0
            if self._force_over >= FORCE_TRIP_SAMPLES:
                self._abort(f"hand force {f:.0f} N above limits.max_force_n ({self._max_force:g} N)")

    def on_safety_trip(self, reason: str) -> None:
        self._abort(f"runaway monitor: {reason}")

    def _abort(self, reason: str) -> None:
        """Safety: end the attempt, park out of reach, and pause until the experimenter resumes."""
        if self._state in ("starting", "parking", "at_park"):
            return
        self.get_logger().error(f"SAFETY ABORT ({reason}): parking and pausing. Resume from the dashboard to continue.")
        self._message = f"SAFETY ABORT: {reason}"
        self._outcome = "safety_trip"
        self.recorder.mark("safety_trip")
        self._stop_actions()
        self._log_attempt(note=reason)
        self.control._paused = True   # noqa: SLF001 -- hold at park once there, exactly as a Pause would
        self._park_now()

    # ------------------------------------------------------------------ the attempt

    def _reset_attempt(self) -> None:
        self._outcome = None
        self._t_armed = None
        self._time_to_snap = None
        self._peak_force = 0.0
        self._force_now = None
        self._force_tare = np.zeros(3)
        self._force_over = 0
        self._max_disp = 0.0

    def _from_park(self) -> None:
        """At park (first move, after a pause): wait hold_sec, then the next attempt."""
        self._state = "at_park"
        self.move_park.wait_then(self.start_trial)

    def start_trial(self) -> None:
        self._stopping = False
        self._returning = False
        self._message = ""
        self._reset_attempt()
        self.trial_count += 1
        self._set_test_pose(get_required_param(self, START))   # live: the pose this attempt uses
        self.get_logger().info(f"--- ATTEMPT {self.trial_count}: test pose {self._test_pose_deg} deg, "
                               f"apple ~{self._fmt_xyz(self._test_apple)} m ---")
        self.recorder.start(trial_index=self.trial_count, threshold_m=self.monitor.threshold_m(),
                            test_pose_deg=self._test_pose_deg,
                            test_apple_xyz_m=[round(float(v), 4) for v in self._test_apple])
        self._in_trial = True
        self.recorder.annotate(resistance=self.resistance.describe())
        self._check_resistance_base()
        self.recorder.mark("trial_start", self.trial_count)
        self._state = "moving"
        self.move_to_start.start()

    def on_move_complete(self) -> None:
        if self._returning:                 # back at the test pose after the attempt
            self._returning = False
            self._finish_trial()
            return
        if self._stopping:
            return
        self.get_logger().info("At the test pose. Quiet window...")
        self.recorder.mark("at_start")
        self._state = "quiet"
        self.safety.start()
        self.quiet_window.start()

    def on_quiet_window_complete(self) -> None:
        if self._state != "quiet":
            return
        self.recorder.mark("quiet_end")
        if self._go_cue:
            self.audio_cue.start()           # arms in on_audio_complete
            self.visual_cue.start()
            self.recorder.mark("cue_go")
        else:
            self._arm()

    def on_audio_complete(self) -> None:
        if self._stopping or self._state != "quiet":
            return
        self._arm()

    def _arm(self) -> None:
        self.get_logger().info("Attempt armed: waiting for the pull (End attempt if he cannot reach it).")
        self._state = "armed"
        self.monitor.start()
        if self._reach_timeout > 0.0:
            self.reach_timeout.set_duration(self._reach_timeout)
            self.reach_timeout.start()

    def on_monitor_armed(self) -> None:
        self.recorder.mark("armed")
        self._t_armed = time.time()
        self._force_tare = self._raw_force()    # the resting force: tool-load model error, a hand resting
        self._force_over = 0
        self.resistance.start()
        self.recorder.annotate(baseline=self.monitor.baseline_xyz())

    def on_monitor_snap(self) -> None:
        self.reach_timeout.stop()
        self._outcome = "pulled"
        if self._t_armed is not None:
            self._time_to_snap = time.time() - self._t_armed
        self.get_logger().info("Threshold reached: snap beep, holding, then back to the test pose.")
        self.recorder.mark("snap", round(self.monitor.current_disp(), 6))
        self.audio_cue_snap.start()
        self.visual_cue_snap.start()
        self.recorder.mark("cue_snap")
        self.resistance.stop()
        self.freeze_hold.start()
        self.recorder.mark("freeze")
        self._state = "snapped"

    def on_monitor_complete(self) -> None:
        # The dwell after the snap is over: back to the test pose at once.
        self.freeze_hold.stop()
        self.resistance.stop()
        self._return_to_test_pose()

    def on_reach_timeout(self) -> None:
        if self._state == "armed":
            self._end_attempt("timeout", f"no pull within {self._reach_timeout:g} s")

    def _end_attempt(self, token: str, why: str) -> None:
        """End the attempt without a pull: back to the test pose, then the next attempt."""
        self.get_logger().info(f"Attempt {self.trial_count} ended: {why}. Returning to the test pose.")
        self._outcome = self._outcome or token
        self.recorder.mark(token)
        self.reach_timeout.stop()
        self.quiet_window.stop()
        self.monitor.stop()
        self.freeze_hold.stop()
        self.resistance.stop()
        self._return_to_test_pose()

    def _return_to_test_pose(self) -> None:
        self.safety.stop()
        self.recorder.mark("recover_start")
        self._state = "returning"
        self._returning = True
        self.move_to_start.start()          # the same pose; the next attempt may then change it

    def _finish_trial(self) -> None:
        self._close_trial()
        if self.control.pause_requested():
            self.get_logger().info("Pause requested: parking (out of reach) and holding there.")
            self._state = "parking"
            self.control.on_event("parking")
            self.move_park.start()           # on_park_complete -> begin_trial holds while paused
        else:
            self._state = "holding"
            self.control.begin_trial(self.start_trial)   # applies a new test pose, if one was set

    def _close_trial(self) -> None:
        """Log and save the attempt (once)."""
        if not self._in_trial:
            return
        outcome = self._outcome or "none"
        self.get_logger().info(f"--- ATTEMPT {self.trial_count} COMPLETE: {outcome} ---")
        if outcome != "safety_trip" and not self._stopping:
            self._log_attempt()
        self.recorder.annotate(outcome=outcome, time_to_snap_s=self._time_to_snap,
                               peak_force_n=round(self._peak_force, 2), max_disp_m=round(self._max_disp, 4))
        self.recorder.mark("trial_end", self.trial_count)
        self.recorder.stop_and_save()
        self._in_trial = False

    # ------------------------------------------------------------------ park, pause, stop

    def _stop_actions(self) -> None:
        self.reach_timeout.stop()
        self.quiet_window.stop()
        self.monitor.stop()
        self.safety.stop()
        self.freeze_hold.stop()
        self.resistance.stop()
        self.move_to_start.cancel()
        self._returning = False

    def _park_now(self) -> None:
        self._state = "parking"
        self.recorder.mark("recover_start")
        self.move_park.start()

    def on_park_complete(self) -> None:
        self._close_trial()                  # a stop or safety abort parks from inside the attempt
        self._state = "at_park"
        self.control.begin_trial(self._from_park)    # holds here while paused

    def _on_pause_request(self) -> None:
        # An armed attempt has no natural end: end it now, so the pause takes effect at once.
        if self._state in ("quiet", "armed"):
            self._end_attempt("paused", "pause requested")

    def _stop_now(self) -> None:
        """Stop now (the dashboard's Stop, or ~/stop): end the attempt at once and park."""
        if self._stopping:
            return
        if self.move_park.cancel_hold():          # at park, waiting before the next attempt: just hold
            self.control.begin_trial(self._from_park)
            return
        if self.move_park.active():
            return                                # already on the way to park
        self._stopping = True
        self.recorder.mark("stop_requested")
        if self._in_trial:
            self._outcome = self._outcome or "stopped"
            self._log_attempt()
        self._stop_actions()
        self._park_now()

    def _srv_end_attempt(self, request, response):
        if self._state in ("quiet", "armed"):
            self._end_attempt("attempt_ended", "ended from the dashboard")
            response.success = True
            response.message = "attempt ended: the arm returns to the test pose, then the next attempt starts"
        elif self._state == "at_park":
            response.success = False
            response.message = "the arm is at park (paused): press Resume to go to the test pose"
        else:
            response.success = False
            response.message = (f"nothing to end right now (the arm is {self._state}); "
                                f"a new test pose still applies from the next attempt")
        return response

    # ------------------------------------------------------------------ live parameters

    def _read_live(self) -> None:
        self._go_cue = bool(get_required_param(self, "go_cue.enabled"))
        self._reach_timeout = max(0.0, float(get_required_param(self, "reach_timeout_sec")))

    def _reload_live(self) -> None:
        """Re-read the live parameters (helpers/live_params.py). Runs between attempts only."""
        for cue in (self.audio_cue, self.audio_cue_snap, self.visual_cue, self.visual_cue_snap):
            cue.reload()
        self.monitor.reload()
        self.resistance.reload()
        self.quiet_window.set_duration(float(get_required_param(self, "quiet_window_sec")))
        self.move_to_start.reload()          # speed limits and the TEST POSE
        self.move_park.reload()
        self._read_live()

    def _check_resistance_base(self) -> None:
        """As apple pluck: the resistance base must equal the SmartPad's stiffness."""
        k = SmartPadLink.get(self.get_logger()).translational_stiffness()
        base = self.resistance.describe()["base_stiffness_n_per_m"]
        if k is not None and abs(k - base) > 0.5:
            self.get_logger().warn(
                f"resistance.base_stiffness_n_per_m is {base:g} N/m but the SmartPad app reports {k:g} N/m: "
                f"every pull feels {k / base:.2g}x the slider's stiffness.")

    # ------------------------------------------------------------------ outputs

    def _publish_pose(self) -> None:
        q = self._q_meas
        d = self.monitor.current_disp()
        msg = {
            "t": time.time(),
            "trial": self.trial_count,
            "state": self._state,
            "joints_deg": [round(math.degrees(v), 2) for v in q] if q is not None else None,
            "apple_m": [round(float(v), 4) for v in self._apple_xyz(q)] if q is not None else None,
            "test_pose_deg": self._test_pose_deg,
            "test_apple_m": [round(float(v), 4) for v in self._test_apple],
            "disp_m": round(d, 4) if math.isfinite(d) else None,
            "force_n": (round(self._force_now, 1)
                        if self._force_now is not None and self._state in ("armed", "snapped") else None),
            "peak_force_n": round(self._peak_force, 1),
            "message": self._message,
            "limits": {"joint_min_deg": self._jmin.tolist(), "joint_max_deg": self._jmax.tolist(),
                       "apple_box_min_m": self._box_min.tolist(), "apple_box_max_m": self._box_max.tolist()},
        }
        self._pose_pub.publish(String(data=json.dumps(msg)))

    def _log_attempt(self, note: str = "") -> None:
        """One row per attempt in reach_calib_attempts.csv, beside the trial files."""
        folder = self.recorder._save_dir    # noqa: SLF001 -- the run folder the trials go to
        path = os.path.join(folder, ATTEMPTS_CSV)
        try:
            os.makedirs(folder, exist_ok=True)
            new = not os.path.exists(path)
            with open(path, "a", newline="") as fh:
                w = csv.writer(fh)
                if new:
                    w.writerow(["time", "attempt", "outcome"] + [f"A{i}_deg" for i in range(1, 8)]
                               + ["apple_x_m", "apple_y_m", "apple_z_m", "time_to_snap_s",
                                  "peak_force_n", "max_disp_m", "note"])
                w.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), self.trial_count, self._outcome or "none"]
                           + [f"{v:.2f}" for v in self._test_pose_deg]
                           + [f"{v:.4f}" for v in self._test_apple]
                           + [f"{self._time_to_snap:.3f}" if self._time_to_snap is not None else "",
                              f"{self._peak_force:.1f}", f"{self._max_disp:.4f}", note])
        except Exception as exc:
            self.get_logger().warn(f"could not write {ATTEMPTS_CSV}: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReachCalibOrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
