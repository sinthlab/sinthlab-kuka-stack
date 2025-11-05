#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from actions.cartesian_impedance_displacement_monitor import CartesianImpedanceDisplacementMonitor
from helpers.common_threshold import get_required_param, DoneGate, create_transient_bool_publisher

class ApplePluckImpedanceControlDisplacementNode(Node):
    """ROS 2 wrapper hosting the reusable displacement monitor action.
    1) Node initializes and runs the CartesianImpedanceDisplacementMonitor action
       after verifying that the move-to-start action has completed.
    2) Publishes a force-release done event, once the action completes.
    """

    def __init__(self) -> None:
        super().__init__(
            "apple_pluck_impedance_control_displacement",
            automatically_declare_parameters_from_overrides=True,
        )

        self._done_topic = str(get_required_param(self, "done_topic"))
        # Done gating: wait for move_to_start completion
        self._done_gate = DoneGate(self, self._done_topic)
        
        self._release_done_topic = str(get_required_param(self, "force_release_done_topic"))
        self._release_pub = create_transient_bool_publisher(self, self._release_done_topic)

        self._action = CartesianImpedanceDisplacementMonitor(
            self, to_start=self._to_start_action, 
            on_complete=self._on_action_complete
        )
    
    def _to_start_action(self) -> bool:
        return self._done_gate and self._done_gate.done
    
    def _on_action_complete(self) -> None:
        self._release_pub.publish(Bool(data=True))      

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ApplePluckImpedanceControlDisplacementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()