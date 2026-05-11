#!/usr/bin/env python3

import rclpy
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_position import MoveToPositionAction
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

        self.move_to_start = MoveToPositionAction(
            self,
            param_prefix="move_to_start",
            on_complete=self.on_move_complete
        )

        self.audio_cue = AudioCue(
            self,
            param_prefix="audio_cue_play",
            on_complete=self.on_audio_complete
        )

        self.restricted_plane = MoveRestrictedOnAPlaneAction(
            self,
            param_prefix=""
        )
        
        self.get_logger().info("=== RESTRICTED PLANE ORCHESTRATOR INITIALIZED ===")
        self.get_logger().info("Starting sequential execution. The virtual fixtures will become active after the initial move to start and audio cue.")
        self.start_sequence()

    def start_sequence(self):
        self.get_logger().info("--- STAGE 1: MOVING TO START ---")
        self.move_to_start.start()

    def on_move_complete(self):
        self.get_logger().info("--- STAGE 2: AUDIO CUE ---")
        self.audio_cue.start()

    def on_audio_complete(self):
        self.get_logger().info("--- STAGE 3: VIRTUAL FIXTURE ACTIVE ---")
        self.get_logger().info("The bounds are now permanently enforced in a continuous control loop.")
        self.restricted_plane.start()

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