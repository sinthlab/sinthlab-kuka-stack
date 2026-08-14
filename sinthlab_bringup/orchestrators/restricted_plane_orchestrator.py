#!/usr/bin/env python3

import rclpy
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_frame import MoveToFrameAction
from sinthlab_bringup.actions.switch_controller import SwitchControllerAction
from sinthlab_bringup.actions.cartesian_impedance_displacement_monitor import CartesianImpedanceDisplacementMonitor
from sinthlab_bringup.actions.safety_stop_monitor import SafetyStopMonitor
from sinthlab_bringup.actions.audio_cue import AudioCue
from sinthlab_bringup.actions.wait_action import WaitAction
from sinthlab_bringup.actions.move_restricted_on_a_plane import MoveRestrictedOnAPlaneAction

# TORQUE mode: joint-space impedance for the exact-posture start/recover moves; Cartesian impedance
# (streamed target_frame) for the fixture. The orchestrator switches between the two.
MOVE_CTRL = "joint_impedance_controller"
FIXTURE_CTRL = "cartesian_impedance_controller"


class RestrictedPlaneOrchestratorNode(rclpyNode):
    """Restricted-plane trial loop, composed entirely of actions:

    move_to_start -> switch to Cartesian impedance -> quiet_window -> audio_cue
    -> (restricted_plane + monitor) -> snap (stop fixtures + cue)
    -> switch to joint impedance -> move_recover -> repeat.

    The start/recover moves run on the joint-impedance controller so the arm reaches the start pose
    firmly. The Cartesian virtual fixture runs on the cartesian_impedance_controller, so we switch to
    it once the arm is at start, and switch back before recovering.
    """

    def __init__(self):
        super().__init__(
            "restricted_plane_orchestrator",
            automatically_declare_parameters_from_overrides=True,
        )
        AudioCue.warmup(self)  # wake the WSL2 audio driver so the first cue isn't delayed

        self.trial_count = 0

        # Actions that make up the trial.
        self.move_to_start = MoveToFrameAction(
            self, param_prefix="move_to_start", on_complete=self.on_move_complete,
            target_controller=MOVE_CTRL,
        )
        self.switch_to_fixture = SwitchControllerAction(
            self, activate=[FIXTURE_CTRL], deactivate=[MOVE_CTRL],
            on_complete=self.on_switched_to_fixture, name="switch->impedance",
        )
        self.quiet_window = WaitAction(
            self, duration_sec=2.0, on_complete=self.on_quiet_window_complete, name="quiet_window"
        )
        self.audio_cue = AudioCue(
            self, param_prefix="audio_cue_play", on_complete=self.on_audio_complete
        )
        self.audio_cue_snap = AudioCue(
            self, param_prefix="audio_cue_snap", on_complete=lambda: None
        )
        self.monitor = CartesianImpedanceDisplacementMonitor(
            self, param_prefix="apple_pluck_impedance_control_displacement",
            on_complete=self.on_monitor_complete, on_snap=self.on_monitor_snap,
        )
        self.restricted_plane = MoveRestrictedOnAPlaneAction(self, param_prefix="")
        self.safety = SafetyStopMonitor(self, param_prefix="plane_safety", on_trip=self.on_safety_trip)
        self.switch_to_joint = SwitchControllerAction(
            self, activate=[MOVE_CTRL], deactivate=[FIXTURE_CTRL],
            on_complete=self.on_switched_to_joint, name="switch->joint_impedance",
        )
        self.move_recover = MoveToFrameAction(
            self, param_prefix="move_to_start_recover", on_complete=self.on_recover_complete,
            target_controller=MOVE_CTRL,
        )

        self.get_logger().info("=== MULTI-TRIAL RESTRICTED PLANE EXPERIMENT INITIALIZED ===")
        self.start_trial()

    def start_trial(self):
        self.trial_count += 1
        self.get_logger().info(f"--- STARTING TRIAL {self.trial_count} ---")
        self.move_to_start.start()  # joint controller is active at the start of every trial

    def on_move_complete(self):
        self.get_logger().info("Arm at exact start posture. Switching to the impedance controller for the fixture...")
        self.switch_to_fixture.start()

    def on_switched_to_fixture(self):
        self.get_logger().info("On the impedance controller. Waiting for a quiet window...")
        self.quiet_window.start()

    def on_quiet_window_complete(self):
        self.get_logger().info("Quiet window complete. Sounding audio cue.")
        self.audio_cue.start()

    def on_audio_complete(self):
        self.get_logger().info("Audio cue played. Virtual fixtures + displacement monitor + safety active.")
        self.restricted_plane.start()
        self.monitor.start()
        self.safety.start()

    def on_monitor_snap(self):
        self.get_logger().info("Threshold reached! Disabling virtual fixtures and playing snap cue.")
        self.restricted_plane.stop()
        self.audio_cue_snap.start()

    def on_safety_trip(self, reason: str):
        # Runaway (e.g. a gravity-driven fall on the free pull axis): abort straight to recovery.
        self.get_logger().error("SAFETY ABORT: recovering to the start posture immediately.")
        self.safety.stop()
        self.monitor.stop()
        self.restricted_plane.stop()
        self.switch_to_joint.start()

    def on_monitor_complete(self):
        self.get_logger().info("User released tension. Switching to joint controller to recover.")
        self.safety.stop()
        self.restricted_plane.stop()  # idempotent; ensure the fixture isn't still streaming
        self.switch_to_joint.start()

    def on_switched_to_joint(self):
        self.move_recover.start()

    def on_recover_complete(self):
        self.get_logger().info(f"--- TRIAL {self.trial_count} COMPLETE ---")
        self.start_trial()


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
