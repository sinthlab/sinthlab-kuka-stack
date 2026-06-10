#!/usr/bin/env python3

import rclpy
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_position_joint_space import MoveToPositionJointSpace
from sinthlab_bringup.actions.perturb_initial_position import PerturbInitialPosition
from sinthlab_bringup.actions.cartesian_impedance_displacement_monitor import CartesianImpedanceDisplacementMonitor
from sinthlab_bringup.actions.audio_cue import AudioCue
from sinthlab_bringup.actions.wait_action import WaitAction
from sinthlab_bringup.actions.freeze_at_pose import FreezeAtPoseAction
from sinthlab_bringup.helpers.common_threshold import get_required_param


class PerturbOrchestratorNode(rclpyNode):
    """Perturbation trial loop, composed entirely of actions:

    move_to_start -> quiet_window -> audio_cue -> perturb_delay -> perturb -> monitor
    -> (snap cue) -> move_recover -> repeat.
    """

    def __init__(self) -> None:
        super().__init__(
            "perturb_orchestrator",
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
        # Delay after the audio cue before the perturbation move begins [s] (from config).
        self.perturb_delay = WaitAction(
            self,
            duration_sec=float(get_required_param(self, "perturb_start.start_delay_sec")),
            on_complete=self.start_perturb,
            name="perturb_delay",
        )
        self.perturb = PerturbInitialPosition(
            self, param_prefix="perturb_start", on_complete=self.on_perturb_complete
        )
        self.monitor = CartesianImpedanceDisplacementMonitor(
            self, param_prefix="apple_pluck_impedance_control_displacement",
            on_complete=self.on_monitor_complete, on_snap=self.on_monitor_snap,
            on_armed=self.on_monitor_armed,
        )
        # "Pull now" cue — sounds when the monitor locks its baseline (post-perturbation, settled),
        # telling the operator the perturbation is done and it's time to pull the end effector.
        self.audio_cue_pull = AudioCue(
            self, param_prefix="audio_cue_pull", on_complete=lambda: None
        )
        self.audio_cue_snap = AudioCue(
            self, param_prefix="audio_cue_snap", on_complete=lambda: None
        )
        # At threshold, freeze the equilibrium on the arm's current pose so it stops pulling back
        # but stays supported (not limp). Held for the dwell, then recover.
        self.freeze_hold = FreezeAtPoseAction(self)
        self.move_recover = MoveToPositionJointSpace(
            self, param_prefix="move_to_start_recover", on_complete=self.on_recover_complete
        )

        self.get_logger().info("=== AUTOMATED MULTI-TRIAL PERTURB EXPERIMENT INITIALIZED ===")
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
        self.get_logger().info("Audio cue played. Waiting before perturbation...")
        self.perturb_delay.start()

    def start_perturb(self):
        self.get_logger().info("Initiating Perturbation Move.")
        self.perturb.start()

    def on_perturb_complete(self):
        self.get_logger().info("Perturbation complete. Initiating Displacement Monitor.")
        self.monitor.start()

    def on_monitor_armed(self):
        # Baseline locked after the post-perturbation settle — cue the operator to start pulling.
        self.get_logger().info("Monitor armed. Cue: start pulling the end effector.")
        self.audio_cue_pull.start()

    def on_monitor_snap(self):
        self.get_logger().info("Threshold reached — freezing the arm at its current pose (pull released).")
        self.audio_cue_snap.start()
        self.freeze_hold.start()  # equilibrium moves onto the arm and holds there for the dwell

    def on_monitor_complete(self):
        # Dwell elapsed: stop holding and return to start for the next trial.
        self.get_logger().info("Perturbation trial done. Returning to start.")
        self.freeze_hold.stop()
        self.move_recover.start()

    def on_recover_complete(self):
        self.get_logger().info(f"--- TRIAL {self.trial_count} COMPLETE ---")
        self.start_trial()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerturbOrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
