#!/usr/bin/env python3

import rclpy
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_position_joint_space import MoveToPositionJointSpace
from sinthlab_bringup.actions.cartesian_impedance_displacement_monitor import CartesianImpedanceDisplacementMonitor
from sinthlab_bringup.actions.audio_cue import AudioCue
from sinthlab_bringup.actions.wait_action import WaitAction
from sinthlab_bringup.actions.spring_release import SpringReleaseAction


class ApplePluckOrchestratorNode(rclpyNode):
    """Apple-pluck trial loop, composed entirely of actions:

    move_to_start -> quiet_window -> audio_cue -> monitor -> (snap cue) -> move_recover -> repeat.
    """

    def __init__(self) -> None:
        super().__init__(
            "apple_pluck_orchestrator",
            automatically_declare_parameters_from_overrides=True,
        )
        AudioCue.warmup(self)  # wake the WSL2 audio driver so the first cue isn't delayed

        self.trial_count = 0

        # Actions that make up the trial.
        self.move_to_start = MoveToPositionJointSpace(
            self, param_prefix="move_to_start", on_complete=self.on_move_complete
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
        # At threshold, release the cabinet spring so the arm "gives" (apple breaks off) instead of
        # pulling back. The give lasts the monitor's force_release_shutdown_delay_sec, then recover.
        self.spring_release = SpringReleaseAction(self)
        self.move_recover = MoveToPositionJointSpace(
            self, param_prefix="move_to_start_recover", on_complete=self.on_recover_complete
        )

        self.get_logger().info("=== AUTOMATED MULTI-TRIAL EXPERIMENT INITIALIZED ===")
        self.start_trial()

    def start_trial(self):
        self.trial_count += 1
        self.get_logger().info(f"--- STARTING TRIAL {self.trial_count} ---")
        self.move_to_start.start()

    def on_move_complete(self):
        self.get_logger().info("Arm returned to start. Waiting for a quiet window...")
        self.quiet_window.start()

    def on_quiet_window_complete(self):
        self.get_logger().info("Quiet window complete. Sounding audio cue.")
        self.audio_cue.start()

    def on_audio_complete(self):
        self.get_logger().info("Audio cue played. Initiating Displacement Monitor.")
        self.monitor.start()

    def on_monitor_snap(self):
        self.get_logger().info("Threshold reached — releasing spring (apple breaks off).")
        self.audio_cue_snap.start()
        self.spring_release.start()  # spring goes slack; arm gives in for the dwell window

    def on_monitor_complete(self):
        # Dwell elapsed: stop the release and return to start for the next trial.
        self.get_logger().info("Apple plucked! Returning to start.")
        self.spring_release.stop()
        self.move_recover.start()

    def on_recover_complete(self):
        self.get_logger().info(f"--- TRIAL {self.trial_count} COMPLETE ---")
        self.start_trial()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ApplePluckOrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
