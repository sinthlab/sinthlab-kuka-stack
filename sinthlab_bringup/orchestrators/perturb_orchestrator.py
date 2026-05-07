#!/usr/bin/env python3


import rclpy
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_position import MoveToPositionAction
from sinthlab_bringup.actions.cartesian_impedance_displacement_monitor import CartesianImpedanceDisplacementMonitor
from sinthlab_bringup.actions.audio_cue import AudioCue
from sinthlab_bringup.actions.force_torque_bias import ForceTorqueBias

class PerturbOrchestratorNode(rclpyNode):
    def __init__(self) -> None:
        super().__init__(
            "perturb_orchestrator",
            automatically_declare_parameters_from_overrides=True,
        )
        self.trial_count = 0

        # Initialize the state machine components
        self.move_to_start = MoveToPositionAction(
            self,
            param_prefix="move_to_start",
            on_complete=self.on_move_complete
        )
        
        self.calibrator = ForceTorqueBias(
            self,
            param_prefix="force_torque_bias_calibrator",
            on_complete=self.on_calib_complete
        )

        self.audio_cue = AudioCue(
            self,
            param_prefix="audio_cue_play",
            on_complete=self.on_audio_complete
        )

        self.perturb = MoveToPositionAction(
            self,
            param_prefix="perturb_start",
            on_complete=self.on_perturb_complete
        )

        self.monitor = CartesianImpedanceDisplacementMonitor(
            self,
            param_prefix="apple_pluck_impedance_control_displacement",
            on_complete=self.on_monitor_complete
        )

        self.move_recover = MoveToPositionAction(
            self,
            param_prefix="move_to_start_recover",
            on_complete=self.on_recover_complete
        )

        # Perturbation specific variables
        import time
        self._start_delay_sec = 1.5

        # Kick off Trial 1!
        self.get_logger().info("=== AUTOMATED MULTI-TRIAL PERTURB EXPERIMENT INITIALIZED ===")
        self.start_trial()

    def start_trial(self):
        self.trial_count += 1
        self.get_logger().info(f"--- STARTING TRIAL {self.trial_count} ---")
        self.move_to_start.start()

    def on_move_complete(self):
        self.get_logger().info("Arm returned to start. Initiating Force-Torque Bias Calibration.")
        self.calibrator.start()

    def on_calib_complete(self, bias):
        self.get_logger().info("Calibration complete. Sounding audio cue.")
        self.audio_cue.start()

    def on_audio_complete(self):
        self.get_logger().info(f"Audio cue played. Waiting {self._start_delay_sec}s before perturbation...")
        self.create_timer(self._start_delay_sec, self.start_perturb)

    def start_perturb(self):
        if hasattr(self, '_timer_handle') and self._timer_handle is not None:
             self._timer_handle.cancel()
             self._timer_handle = None
        self.get_logger().info("Initiating Perturbation Move.")
        self.perturb.start()

    def on_perturb_complete(self):
        self.get_logger().info("Perturbation complete. Initiating Displacement Monitor.")
        self.monitor.start()

    def on_monitor_complete(self):
        # Snap has fully completed! The KUKA arm is free to recoil.
        self.get_logger().info("Apple plucked! Waiting for physical recoil.")
        self.move_recover.start()

    def on_recover_complete(self):
        self.get_logger().info(f"--- TRIAL {self.trial_count} COMPLETE ---")
        # Restart the cycle immediately!
        self.start_trial()

    def create_timer(self, timer_period_sec, callback):
        self._timer_handle = super().create_timer(timer_period_sec, callback)


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