#!/usr/bin/env python3

import rclpy
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_position_joint_space import MoveToPositionJointSpace
from sinthlab_bringup.actions.park_move import ParkMove
from sinthlab_bringup.actions.switch_controller import SwitchControllerAction
from sinthlab_bringup.actions.admittance_move import AdmittanceMoveAction
from sinthlab_bringup.actions.safety_stop_monitor import SafetyStopMonitor
from sinthlab_bringup.actions.force_release_waiter import ForceReleaseWaiter
from sinthlab_bringup.actions.audio_cue import AudioCue
from sinthlab_bringup.actions.visual_cue import VisualCue
from sinthlab_bringup.actions.ring_state_cue import RingStateCue
from sinthlab_bringup.actions.wait_action import WaitAction
from sinthlab_bringup.actions.trial_recorder import TrialRecorder
from sinthlab_bringup.helpers.node_utils import get_required_param, build_node
from sinthlab_bringup.helpers.experiment_control import ExperimentControl

JOINT_CTRL = "lbr_joint_position_command_controller"
CLIK_CTRL = "kuka_clik_controller"


class FreeMoveOrchestratorNode(rclpyNode):
    """Pre-training: the arm in admittance -- it goes wherever it is pushed and stays where it is let go.
    No goal, no reward, no boundary. A trial is one free-movement SESSION that runs until it is ended:
    by Pause or "Stop after trial" on the dashboard (the pause service), or after session_sec if > 0.

    [first: move_park, slowly, from wherever the arm is]
    move_to_start (JOINT) -> switch to CLIK -> hold + measure the resting force (tare) during the
    quiet window -> go cue (ring GREEN) -> admittance + runaway (speed) monitor
    -> session ended (end tone, ring dark): the arm holds where it is -> release wait
    -> switch to JOINT -> park (out of reach) -> holds there (paused) or the launch stops.
    """

    def __init__(self) -> None:
        super().__init__("free_move_orchestrator", automatically_declare_parameters_from_overrides=True)
        AudioCue.warmup(self)
        VisualCue.warmup(self)

        self.trial_count = 0
        self._trial_ending = False

        self.move_to_start = MoveToPositionJointSpace(
            self, param_prefix="move_to_start", on_complete=self.on_move_complete)
        self.switch_to_fixture = SwitchControllerAction(
            self, activate=[CLIK_CTRL], deactivate=[JOINT_CTRL],
            on_complete=self.on_switched_to_fixture, name="switch->clik")
        self.admittance = AdmittanceMoveAction(self, param_prefix="free_move")
        self.ring = RingStateCue(self)
        self.quiet_window = WaitAction(
            self, duration_sec=float(get_required_param(self, "quiet_window_sec")),
            on_complete=self.on_quiet_window_complete, name="quiet_window")
        self.go_cue = AudioCue(
            self, param_prefix="audio_cue_play", on_complete=self.on_go_complete,
            on_finished=lambda secs: self.recorder.mark("cue_audio_end", round(secs, 4)))

        self.control = ExperimentControl(self, "free_move")
        # A live start / park pose whose move would take the end effector below the floor is refused.
        self.control.add_validator(lambda n, v, c: self.move_park.check_live(n, v, c))
        self.control.on_reload(self._reload_live)
        self.control.on_stop_request(self._stop_now)   # Stop now: end the trial, park, hold
        self._stopping = False
        # The session has no natural end: a pause request (Pause, or the dashboard's "Stop after trial")
        # ends it now, so the arm parks and the pause / stop can take effect there.
        self.control.on_pause_request(self.on_end_requested)
        self._session_live = False
        self.recorder = TrialRecorder(
            self, experiment="free_move",
            extra_header=self.admittance.record_extra_header(), extra_fn=self.admittance.record_extra,
            on_event=self.control.on_event)
        VisualCue.set_result_sink(
            lambda lbl, ok, ms: self.recorder.mark("cue_visual_ack", round(ms, 1)))

        self.safety = SafetyStopMonitor(self, param_prefix="free_move_safety", on_trip=self.on_safety_trip)
        self.end_cue = AudioCue(self, param_prefix="audio_cue_end", on_complete=lambda: None)
        self.session = WaitAction(          # only used when session_sec > 0
            self, duration_sec=max(float(get_required_param(self, "session_sec")), 1.0),
            on_complete=self.on_session_over, name="session")
        self._session_sec = float(get_required_param(self, "session_sec"))
        self.force_release = ForceReleaseWaiter(
            self, param_prefix="force_release", on_complete=self.on_force_released)
        self.switch_to_joint = SwitchControllerAction(
            self, activate=[JOINT_CTRL], deactivate=[CLIK_CTRL],
            on_complete=self.on_switched_to_joint, name="switch->joint")
        self.move_park = ParkMove(self, on_complete=self.on_park_complete)

        # Move speed sliders act at once, from each move's next start (live_params.IMMEDIATE).
        self.control.apply_immediately({
            "move_to_start.speed_scale": self.move_to_start.set_speed_scale,
            "move_to_park.speed_scale": self.move_park.set_speed_scale,
        }, log=self.recorder.note_change)
        # Every move's actual speed (scaled limits, duration) goes into the trial sidecar's `moves`.
        for move in (self.move_to_start, self.move_park):
            move.report_moves_to(self.recorder.note_move)
        self.get_logger().info("=== PRE-TRAINING: FREE MOVE INITIALIZED ===")
        # The run's first move: to park, out of reach, from wherever the arm is; then the trial loop.
        self.control.on_event("parking")
        self.move_park.start_first(lambda: self.control.begin_trial(self._next_trial))

    def start_trial(self):
        self._stopping = False
        self.recorder.start(trial_index=self.trial_count + 1)
        self.recorder.mark("trial_start", self.trial_count + 1)
        self.trial_count += 1
        self._trial_ending = False
        self.get_logger().info(f"--- STARTING SESSION {self.trial_count} ---")
        self.ring.show("off")
        self.move_to_start.start()

    def on_move_complete(self):
        self.recorder.mark("at_start")
        self.switch_to_fixture.start()

    def on_switched_to_fixture(self):
        if self._stopping:                 # Stop now came during the switch: straight back
            self.switch_to_joint.start()
            return
        # Hold still and measure the resting force while the quiet window runs; the arm only
        # starts following the hand at the go cue. Keep quiet_window_sec >= free_move.tare_sec.
        self.recorder.mark("fixture_active")
        self.admittance.start()
        self.quiet_window.start()

    def on_quiet_window_complete(self):
        self.recorder.mark("cue_go")
        self.ring.show("go")
        self.go_cue.start()

    def on_go_complete(self):
        if self._stopping:
            return
        self.recorder.mark("armed")
        self.admittance.enable_motion()
        self.safety.start()
        self._session_live = True
        if self._session_sec > 0:
            self.session.start()
            self.get_logger().info(f"Free move: the arm follows the hand for {self._session_sec:.0f} s.")
        else:
            self.get_logger().info("Free move: the arm follows the hand until Pause / Stop after trial.")
        if self.control.pause_requested():      # asked to stop before the session even began
            self.on_end_requested()

    def on_session_over(self):
        self._end_trial("session_end")

    def on_end_requested(self):
        """Pause / End session. A live session ends now; one still starting ends as it goes live. It ends
        the way Stop now does -- straight to park, no end cue or release wait -- and, being paused, holds
        there (Resume starts the next session)."""
        if self._session_live:
            self.get_logger().info("End requested: ending the session and parking.")
            self._stop_now(mark="session_end")

    def on_safety_trip(self, reason: str):
        self._end_trial("safety")

    def _end_trial(self, reason: str):
        if self._trial_ending:
            return
        self._trial_ending = True
        self._session_live = False
        self.recorder.mark(reason if reason != "safety" else "safety_trip")
        self.session.stop()
        self.safety.stop()
        self.admittance.stop()          # holds where it is
        self.ring.show("off")
        if reason == "safety":
            self.get_logger().error("SAFETY ABORT: parking immediately.")
            self.switch_to_joint.start()
            return
        self.get_logger().info("Session over. The arm holds; waiting for release before reset.")
        self.end_cue.start()
        self.recorder.mark("release_wait")
        self.force_release.start()

    def on_force_released(self):
        self.recorder.mark("released")
        self.switch_to_joint.start()

    def on_switched_to_joint(self):
        self.move_park.start()

    def on_park_complete(self):
        self.get_logger().info(f"--- SESSION {self.trial_count} COMPLETE ---")
        self.recorder.mark("trial_end", self.trial_count)
        self.recorder.stop_and_save()
        self.control.begin_trial(self._next_trial)   # holds at park instead if paused

    def _next_trial(self):
        self.move_park.wait_then(self.start_trial)    # wait move_to_park.hold_sec at park, then the next trial

    def _stop_now(self, mark: str = "stop_requested"):
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
        self.recorder.mark(mark)
        self._trial_ending = True
        self._session_live = False
        self.quiet_window.stop()
        self.session.stop()
        self.safety.stop()
        self.admittance.stop()
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
        for cue in (self.go_cue, self.end_cue):
            cue.reload()
        self.ring.reload()
        self.quiet_window.set_duration(float(get_required_param(self, "quiet_window_sec")))
        self.move_to_start.reload()
        self.move_park.reload()
        self._session_sec = float(get_required_param(self, "session_sec"))
        self.session.set_duration(max(self._session_sec, 1.0))
        self.admittance.reload()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = build_node(FreeMoveOrchestratorNode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
