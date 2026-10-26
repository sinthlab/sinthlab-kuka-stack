#!/usr/bin/env python3

import rclpy
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_position_joint_space import MoveToPositionJointSpace
from sinthlab_bringup.actions.park_move import ParkMove
from sinthlab_bringup.actions.switch_controller import SwitchControllerAction
from sinthlab_bringup.actions.cartesian_impedance_displacement_monitor import CartesianImpedanceDisplacementMonitor
from sinthlab_bringup.actions.safety_stop_monitor import SafetyStopMonitor
from sinthlab_bringup.actions.audio_cue import AudioCue
from sinthlab_bringup.actions.visual_cue import VisualCue
from sinthlab_bringup.actions.wait_action import WaitAction
from sinthlab_bringup.actions.move_restricted_on_a_plane import MoveRestrictedOnAPlaneAction
from sinthlab_bringup.actions.trial_recorder import TrialRecorder
from sinthlab_bringup.helpers.node_utils import get_required_param
from sinthlab_bringup.helpers.experiment_control import ExperimentControl

JOINT_CTRL = "lbr_joint_position_command_controller"
CLIK_CTRL = "kuka_clik_controller"


class RestrictedPlaneOrchestratorNode(rclpyNode):
    """Restricted-plane trial loop, composed entirely of actions:

    [first: move_park, slowly, from wherever the arm is]
    move_to_start (JOINT) -> switch to CLIK -> quiet_window -> audio_cue
    -> (restricted_plane + monitor) -> snap (stop fixtures + cue)
    -> switch to JOINT -> move_park (JOINT, out of reach) -> repeat.

    The start and park moves run on the joint-position controller so the arm reaches the EXACT
    configuration (deterministic posture, like apple-pluck). The Cartesian virtual fixture needs
    the CLIK, so we switch to it once the arm is at start, and switch back before parking.
    """

    def __init__(self):
        super().__init__(
            "restricted_plane_orchestrator",
            automatically_declare_parameters_from_overrides=True,
        )
        AudioCue.warmup(self)  # wake the WSL2 audio driver so the first cue isn't delayed
        VisualCue.warmup(self)  # logs which visual-cue trigger is active (Wi-Fi test or wire)

        self.trial_count = 0

        # Actions that make up the trial.
        self.move_to_start = MoveToPositionJointSpace(
            self, param_prefix="move_to_start", on_complete=self.on_move_complete
        )
        self.switch_to_fixture = SwitchControllerAction(
            self, activate=[CLIK_CTRL], deactivate=[JOINT_CTRL],
            on_complete=self.on_switched_to_fixture, name="switch->clik",
        )
        self.quiet_window = WaitAction(
            self, duration_sec=float(get_required_param(self, "quiet_window_sec")),
            on_complete=self.on_quiet_window_complete, name="quiet_window"
        )
        self.audio_cue = AudioCue(
            self, param_prefix="audio_cue_play", on_complete=self.on_audio_complete,
            on_finished=lambda secs: self.recorder.mark("cue_audio_end", round(secs, 4)),
        )
        self.audio_cue_snap = AudioCue(
            self, param_prefix="audio_cue_snap", on_complete=lambda: None
        )
        # Audio and visual cues fire together. The visual cue sends TIMING only --
        # what the ring shows is configured on the board itself. It is a no-op when
        # `visual_cue.enabled` is false, so the experiment runs unchanged before the
        # ring is wired.
        self.visual_cue = VisualCue(
            self, label="play", on_complete=lambda: None
        )
        self.visual_cue_snap = VisualCue(
            self, label="snap", on_complete=lambda: None
        )
        self.monitor = CartesianImpedanceDisplacementMonitor(
            self, param_prefix="apple_pluck_impedance_control_displacement",
            on_complete=self.on_monitor_complete, on_snap=self.on_monitor_snap,
        )
        self.restricted_plane = MoveRestrictedOnAPlaneAction(self, param_prefix="")
        self.safety = SafetyStopMonitor(self, param_prefix="plane_safety", on_trip=self.on_safety_trip)
        self.switch_to_joint = SwitchControllerAction(
            self, activate=[JOINT_CTRL], deactivate=[CLIK_CTRL],
            on_complete=self.on_switched_to_joint, name="switch->joint",
        )
        self.move_park = ParkMove(self, on_complete=self.on_park_complete)

        # Dashboard / CLI control: status topic, pause between trials, live parameters, NSP codes.
        # See helpers/experiment_control.py; which parameters are live is in helpers/live_params.py.
        self.control = ExperimentControl(self, "restricted_plane")
        # A live start / park pose whose move would take the end effector below the floor is refused.
        self.control.add_validator(lambda n, v, c: self.move_park.check_live(n, v, c))
        self.control.on_reload(self._reload_live)
        self.control.on_stop_request(self._stop_now)   # Stop now: end the trial, park, hold
        self._stopping = False
        # Trial data -- see README.md section 7, Data Collected. disp_m is the pull the snap is judged on.
        self.recorder = TrialRecorder(
            self, experiment="restricted_plane",
            extra_header=["disp_m"],
            extra_fn=lambda _T: [f"{self.monitor.current_disp():.6f}"],
            on_event=self.control.on_event,
        )
        VisualCue.set_result_sink(
            lambda lbl, ok, ms: self.recorder.mark("cue_visual_ack", round(ms, 1)))

        # Move speed sliders act at once, from each move's next start (live_params.IMMEDIATE).
        self.control.apply_immediately({
            "move_to_start.speed_scale": self.move_to_start.set_speed_scale,
            "move_to_park.speed_scale": self.move_park.set_speed_scale,
        }, log=self.recorder.note_change)
        # Every move's actual speed (scaled limits, duration) goes into the trial sidecar's `moves`.
        for move in (self.move_to_start, self.move_park):
            move.report_moves_to(self.recorder.note_move)
        self.get_logger().info("=== MULTI-TRIAL RESTRICTED PLANE EXPERIMENT INITIALIZED ===")
        # The run's first move: to park, out of reach, from wherever the arm is; then the trial loop.
        self.control.on_event("parking")
        self.move_park.start_first(lambda: self.control.begin_trial(self._next_trial))

    def start_trial(self):
        self._stopping = False
        self.trial_count += 1
        self.get_logger().info(f"--- STARTING TRIAL {self.trial_count} ---")
        self.recorder.start(trial_index=self.trial_count, threshold_m=self.monitor.threshold_m(),
                            fixture={"profile": self.restricted_plane.active_profile,
                                     **self.restricted_plane.profile_config})
        self.recorder.mark("trial_start", self.trial_count)
        self.move_to_start.start()  # joint controller is active at the start of every trial

    def on_move_complete(self):
        self.recorder.mark("at_start")
        self.get_logger().info("Arm at exact start posture. Switching to CLIK for the fixture...")
        self.switch_to_fixture.start()

    def on_switched_to_fixture(self):
        if self._stopping:                 # Stop now came during the switch: straight back
            self.switch_to_joint.start()
            return
        self.get_logger().info("On CLIK. Waiting for a quiet window...")
        self.quiet_window.start()

    def on_quiet_window_complete(self):
        self.get_logger().info("Quiet window complete. Sounding audio cue.")
        self.recorder.mark("cue_go")
        self.audio_cue.start()
        self.visual_cue.start()

    def on_audio_complete(self):
        if self._stopping:
            return
        self.get_logger().info("Audio cue played. Virtual fixtures + displacement monitor + safety active.")
        self.recorder.mark("armed")
        self.restricted_plane.start()
        self.monitor.start()
        self.safety.start()

    def on_monitor_snap(self):
        self.get_logger().info("Threshold reached! Disabling virtual fixtures and playing snap cue.")
        self.recorder.mark("snap")
        self.restricted_plane.stop()
        self.audio_cue_snap.start()
        self.visual_cue_snap.start()

    def on_safety_trip(self, reason: str):
        # Runaway (e.g. a gravity-driven fall on the free pull axis): abort straight to park.
        self.get_logger().error("SAFETY ABORT: parking immediately.")
        self.recorder.mark("safety_trip")
        self.safety.stop()
        self.monitor.stop()
        self.restricted_plane.stop()
        self.switch_to_joint.start()

    def on_monitor_complete(self):
        self.get_logger().info("User released tension. Switching to joint controller to park.")
        self.safety.stop()
        self.restricted_plane.stop()  # idempotent; ensure the fixture isn't still streaming
        self.switch_to_joint.start()

    def on_switched_to_joint(self):
        self.recorder.mark("recover_start")
        self.move_park.start()

    def on_park_complete(self):
        self.get_logger().info(f"--- TRIAL {self.trial_count} COMPLETE ---")
        self.recorder.mark("trial_end", self.trial_count)
        self.recorder.stop_and_save()
        self.control.begin_trial(self._next_trial)   # holds at park instead if paused

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
        self.quiet_window.stop()
        self.monitor.stop()
        self.safety.stop()
        self.restricted_plane.stop()
        if self.move_to_start.active():           # still on the joint controller
            self.move_to_start.cancel()
            self.recorder.mark("recover_start")
            self.move_park.start()
        elif not self.switch_to_fixture.active():  # on the CLIK: back to joint, then park
            self.switch_to_joint.start()
        # (mid-switch to the CLIK, on_switched_to_fixture turns back)

    def _reload_live(self):
        """Re-read the live parameters (helpers/live_params.py). Runs between trials only."""
        for cue in (self.audio_cue, self.audio_cue_snap, self.visual_cue, self.visual_cue_snap):
            cue.reload()
        self.monitor.reload()
        self.quiet_window.set_duration(float(get_required_param(self, "quiet_window_sec")))
        self.move_to_start.reload()      # speed limits
        self.move_park.reload()


def main(args=None):
    rclpy.init(args=args)
    node = RestrictedPlaneOrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
