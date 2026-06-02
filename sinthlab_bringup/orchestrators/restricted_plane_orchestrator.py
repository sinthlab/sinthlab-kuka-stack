#!/usr/bin/env python3

import rclpy
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_position import MoveToPositionAction
from sinthlab_bringup.actions.cartesian_impedance_displacement_monitor import CartesianImpedanceDisplacementMonitor
from sinthlab_bringup.actions.audio_cue import AudioCue
from sinthlab_bringup.actions.move_restricted_on_a_plane import MoveRestrictedOnAPlaneAction

class RestrictedPlaneOrchestratorNode(rclpyNode):
    def __init__(self):
        super().__init__('restricted_plane_orchestrator', automatically_declare_parameters_from_overrides=True)

        # Audio driver warmup for Windows/WSL2
        import subprocess
        try:
            subprocess.Popen(["powershell.exe", "-NoProfile", "-Command", "[console]::Beep(37, 10)"])
        except Exception:
            pass

        self.trial_count = 0

        self.move_to_start = MoveToPositionAction(
            self,
            param_prefix="move_to_start",
            on_complete=self.on_move_complete
        )

        self.monitor = CartesianImpedanceDisplacementMonitor(
            self,
            param_prefix="apple_pluck_impedance_control_displacement",
            on_complete=self.on_monitor_complete,
            on_snap=self.on_monitor_snap
        )

        self.audio_cue = AudioCue(
            self,
            param_prefix="audio_cue_play",
            on_complete=self.on_audio_complete
        )

        self.audio_cue_snap = AudioCue(
            self,
            param_prefix="audio_cue_snap",
            on_complete=lambda: None
        )

        self.restricted_plane = MoveRestrictedOnAPlaneAction(
            self,
            param_prefix=""
        )

        self.move_recover = MoveToPositionAction(
            self,
            param_prefix="move_to_start_recover",
            on_complete=self.on_recover_complete
        )
        
        self.get_logger().info("=== MULTI-TRIAL RESTRICTED PLANE EXPERIMENT INITIALIZED ===")
        self.start_trial()

    def start_trial(self):
        self.trial_count += 1
        self.get_logger().info(f"--- STARTING TRIAL {self.trial_count} ---")
        self.move_to_start.start()

    def on_move_complete(self):
        self.get_logger().info("Arm returned to start. Waiting for a quiet window...")
        self._quiet_timer = self.create_timer(2.0, self.on_quiet_window_complete)

    def on_quiet_window_complete(self):
        if hasattr(self, '_quiet_timer') and self._quiet_timer is not None:
             self._quiet_timer.cancel()
             self._quiet_timer = None
        self.get_logger().info("Quiet window complete. Sounding audio cue.")
        self.audio_cue.start()

    def on_audio_complete(self):
        self.get_logger().info("Audio cue played. Virtual fixtures and displacement monitor activated.")
        self.restricted_plane.start()
        self.monitor.start()

    def on_monitor_snap(self):
        self.get_logger().info("Threshold reached! Disabling virtual fixtures and playing snap cue.")
        self.restricted_plane.stop()
        self.audio_cue_snap.start()

    def on_monitor_complete(self):
        self.get_logger().info("User released tension. Waiting for physical recoil before resetting.")
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