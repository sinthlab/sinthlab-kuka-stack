#!/usr/bin/env python3

import rclpy
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_position_joint_space import MoveToPositionJointSpace
from sinthlab_bringup.actions.park_move import ParkMove
from sinthlab_bringup.actions.switch_controller import SwitchControllerAction
from sinthlab_bringup.actions.move_in_maze import MoveInMazeAction
from sinthlab_bringup.actions.travel_monitor import TravelMonitor
from sinthlab_bringup.actions.safety_stop_monitor import SafetyStopMonitor
from sinthlab_bringup.actions.force_release_waiter import ForceReleaseWaiter
from sinthlab_bringup.actions.audio_cue import AudioCue
from sinthlab_bringup.actions.visual_cue import VisualCue
from sinthlab_bringup.actions.ring_state_cue import RingStateCue
from sinthlab_bringup.actions.wait_action import WaitAction
from sinthlab_bringup.actions.trial_recorder import TrialRecorder
from sinthlab_bringup.helpers.node_utils import get_required_param
from sinthlab_bringup.helpers.experiment_control import ExperimentControl

JOINT_CTRL = "lbr_joint_position_command_controller"
CLIK_CTRL = "kuka_clik_controller"
_PLANE_AXES = {"x": ("y", "z"), "y": ("x", "z"), "z": ("x", "y")}   # restricted axis -> (a, b)


class RailTrainingOrchestratorNode(rclpyNode):
    """Pre-training: one straight rail through the start -- out to a threshold and back.

    One leg of the maze. The same orchestrator runs MOVE VERTICAL and MOVE HORIZONTAL; which one is
    set by the YAML (`experiment`, the rail in `virtual_fixtures`, `travel_task.axis`).

    [first: move_park, slowly, from wherever the arm is]
    move_to_start (JOINT) -> switch to CLIK -> rail on (anchors while the quiet window runs)
    -> go cue (ring: travel quarters GREEN) -> travel monitor + timeout + safety
    -> |travel| >= threshold: ring RED (+ threshold tone)
    -> back at the start (require_return) -> reward -> release wait -> switch to JOINT -> park.

    Timeout and safety end the trial exactly as in the maze.
    """

    def __init__(self) -> None:
        super().__init__("rail_training_orchestrator", automatically_declare_parameters_from_overrides=True)
        AudioCue.warmup(self)
        VisualCue.warmup(self)
        self.experiment = str(get_required_param(self, "experiment"))   # move_vertical | move_horizontal

        self.trial_count = 0
        self._trial_ending = False

        self.move_to_start = MoveToPositionJointSpace(
            self, param_prefix="move_to_start", on_complete=self.on_move_complete)
        self.switch_to_fixture = SwitchControllerAction(
            self, activate=[CLIK_CTRL], deactivate=[JOINT_CTRL],
            on_complete=self.on_switched_to_fixture, name="switch->clik")
        self.rail = MoveInMazeAction(self, param_prefix="")
        self.ring = RingStateCue(self)
        self.quiet_window = WaitAction(
            self, duration_sec=float(get_required_param(self, "quiet_window_sec")),
            on_complete=self.on_quiet_window_complete, name="quiet_window")
        self.go_cue = AudioCue(
            self, param_prefix="audio_cue_play", on_complete=self.on_go_complete,
            on_finished=lambda secs: self.recorder.mark("cue_audio_end", round(secs, 4)))

        self.control = ExperimentControl(self, self.experiment)
        self.control.on_reload(self._reload_live)
        self.control.on_stop_request(self._stop_now)   # Stop now: end the trial, park, hold
        self._stopping = False
        self.recorder = TrialRecorder(
            self, experiment=self.experiment,
            extra_header=self.rail.record_extra_header(), extra_fn=self.rail.record_extra,
            on_event=self.control.on_event)
        VisualCue.set_result_sink(
            lambda lbl, ok, ms: self.recorder.mark("cue_visual_ack", round(ms, 1)))

        self.travel = TravelMonitor(
            self, param_prefix="travel_task", position_provider=self.rail.offset_from_anchor,
            on_threshold=self.on_threshold, on_complete=self.on_success)
        self.safety = SafetyStopMonitor(self, param_prefix="rail_safety", on_trip=self.on_safety_trip)
        self.threshold_cue = AudioCue(self, param_prefix="audio_cue_threshold", on_complete=lambda: None)
        self.reward_cue = AudioCue(self, param_prefix="audio_cue_reward", on_complete=lambda: None)
        self.timeout_cue = AudioCue(self, param_prefix="audio_cue_timeout", on_complete=lambda: None)
        self.timeout = WaitAction(
            self, duration_sec=float(get_required_param(self, "timeout_sec")),
            on_complete=self.on_timeout, name="trial_timeout")
        self.force_release = ForceReleaseWaiter(
            self, param_prefix="force_release", on_complete=self.on_force_released)
        self.switch_to_joint = SwitchControllerAction(
            self, activate=[JOINT_CTRL], deactivate=[CLIK_CTRL],
            on_complete=self.on_switched_to_joint, name="switch->joint")
        self.move_park = ParkMove(self, on_complete=self.on_park_complete)

        self._check_threshold_on_rail()
        # Move speed sliders act at once, from each move's next start (live_params.IMMEDIATE).
        self.control.apply_immediately({
            "move_to_start.speed_scale": self.move_to_start.set_speed_scale,
            "move_to_park.speed_scale": self.move_park.set_speed_scale,
        }, log=self.recorder.note_change)
        # Every move's actual speed (scaled limits, duration) goes into the trial sidecar's `moves`.
        for move in (self.move_to_start, self.move_park):
            move.report_moves_to(self.recorder.note_move)
        self.get_logger().info(f"=== PRE-TRAINING: {self.experiment} INITIALIZED ===")
        # The run's first move: to park, out of reach, from wherever the arm is; then the trial loop.
        self.control.on_event("parking")
        self.move_park.start_first(lambda: self.control.begin_trial(self._next_trial))

    # ------------------------------------------------------------------ trial
    def start_trial(self):
        self._stopping = False
        self.recorder.start(trial_index=self.trial_count + 1,
                            maze_geometry=self.rail.maze_geometry())
        self.recorder.mark("trial_start", self.trial_count + 1)
        self.trial_count += 1
        self._trial_ending = False
        self.get_logger().info(f"--- STARTING TRIAL {self.trial_count} ---")
        self.ring.show("off")
        self.move_to_start.start()

    def on_move_complete(self):
        self.recorder.mark("at_start")
        self.switch_to_fixture.start()

    def on_switched_to_fixture(self):
        if self._stopping:                 # Stop now came during the switch: straight back
            self.switch_to_joint.start()
            return
        # The rail goes on now, not at the go cue, so it has anchored (anchor_settle_sec) before
        # the travel monitor needs its origin. Keep quiet_window_sec >= anchor_settle_sec.
        self.recorder.mark("fixture_active")
        self.rail.start()
        self.quiet_window.start()

    def on_quiet_window_complete(self):
        self.recorder.mark("cue_go")
        self.ring.show("go")
        self.go_cue.start()

    def on_go_complete(self):
        if self._stopping:
            return
        self.recorder.mark("armed")
        self.travel.start()
        self.timeout.start()
        self.safety.start()

    def on_threshold(self, travel_m: float):
        self.get_logger().info(f"Threshold reached: {travel_m:+.3f} m.")
        self.recorder.mark("threshold", round(travel_m, 4))
        self.ring.show("reached")
        self.threshold_cue.start()

    def on_success(self):
        self._end_trial("goal")

    def on_timeout(self):
        self._end_trial("timeout")

    def on_safety_trip(self, reason: str):
        self._end_trial("safety")

    def _end_trial(self, reason: str):
        if self._trial_ending:
            return
        self._trial_ending = True
        self.recorder.mark(reason if reason != "safety" else "safety_trip")
        self.timeout.stop()
        self.travel.stop()
        self.safety.stop()
        self.rail.stop()
        if reason == "safety":
            self.get_logger().error("SAFETY ABORT: parking immediately.")
            self.ring.show("off")
            self.switch_to_joint.start()
            return
        if reason == "goal":
            self.get_logger().info("Out and back done. Reward; waiting for release before reset.")
            self.ring.show("success")
            self.reward_cue.start()
        else:
            self.get_logger().info("Timeout. Waiting for release before reset.")
            self.ring.show("timeout")
            self.timeout_cue.start()
        self.recorder.mark("release_wait")
        self.force_release.start()

    def on_force_released(self):
        self.recorder.mark("released")
        self.ring.show("off")
        self.switch_to_joint.start()

    def on_switched_to_joint(self):
        self.move_park.start()

    def on_park_complete(self):
        self.get_logger().info(f"--- TRIAL {self.trial_count} COMPLETE ---")
        self.recorder.mark("trial_end", self.trial_count)
        self.recorder.stop_and_save()
        self.control.begin_trial(self._next_trial)   # holds at park instead if paused

    # ------------------------------------------------------------------ live parameters
    def _next_trial(self):
        self.move_park.wait_then(self.start_trial)    # wait move_to_park.hold_sec at park, then the next trial

    def _stop_now(self):
        """Stop now (the dashboard's Stop, or ~/stop): end the trial at once and park -- no cue, no
        release wait. The control is paused, so it then holds at park and the launch can end."""
        if self._stopping:
            return
        if self.move_park.cancel_hold():          # at park, waiting for the next trial: just hold
            self.control.begin_trial(self._next_trial)
            return
        if self.move_park.active() or self.switch_to_joint.active():
            return                                # already on the way to park
        self._stopping = True
        self.recorder.mark("stop_requested")
        self._trial_ending = True
        self.quiet_window.stop()
        self.timeout.stop()
        self.travel.stop()
        self.safety.stop()
        self.rail.stop()
        self.force_release.stop()
        self.ring.show("off")
        if self.move_to_start.active():           # still on the joint controller
            self.move_to_start.cancel()
            self.recorder.mark("recover_start")
            self.move_park.start()
        elif not self.switch_to_fixture.active():  # on the CLIK: back to joint, then park
            self.switch_to_joint.start()
        # (mid-switch to the CLIK, on_switched_to_fixture turns back)

    def _reload_live(self):
        for cue in (self.go_cue, self.threshold_cue, self.reward_cue, self.timeout_cue):
            cue.reload()
        self.ring.reload()
        self.quiet_window.set_duration(float(get_required_param(self, "quiet_window_sec")))
        self.move_to_start.reload()
        self.move_park.reload()
        self.timeout.set_duration(float(get_required_param(self, "timeout_sec")))
        self.rail.reload()
        self.travel.reload()
        self._check_threshold_on_rail()

    def _check_threshold_on_rail(self):
        """Warn if the threshold lies beyond the end of the rail -- it could never be reached."""
        geo = self.rail.maze_geometry()
        axis = str(get_required_param(self, "travel_task.axis")).lower()
        a_name, b_name = _PLANE_AXES.get(str(geo["restricted_axis"]).lower(), ("x", "y"))
        if axis not in (a_name, b_name):
            self.get_logger().warn(
                f"travel_task.axis '{axis}' is not one of the rail plane's axes ({a_name}, {b_name}).")
            return
        lo_i, hi_i = (0, 1) if axis == a_name else (2, 3)
        reach = max(max(abs(c[lo_i]), abs(c[hi_i])) for c in geo["corridors"])
        thr = self.travel.threshold_m()
        if thr > reach - 0.005:
            self.get_logger().warn(
                f"travel_task.threshold_m {thr:.3f} m is at or beyond the rail end ({reach:.3f} m): "
                "it can only be reached by pushing into the end stop. Lengthen the rail or lower it.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RailTrainingOrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
