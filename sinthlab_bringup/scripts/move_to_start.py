#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from helpers.common_threshold import create_transient_bool_publisher
from actions.move_to_position import MoveToPositionAction


class MoveToStartNode(Node):
    """
    ROS 2 wrapper that hosts the reusable move-to-pos action
    1) The node initializes and runs the MoveToPositionAction 
    to move the robot to a predefined start position.
    2) Once the action completes, it publishes a 'done' signal 
    on a transient-local Bool topic.
    """

    def __init__(self) -> None:
        super().__init__(
            "move_to_start",
            automatically_declare_parameters_from_overrides=True,
        )

        self._pub_done = create_transient_bool_publisher(self, "move_to_start/done")
        self._done_published = False

        self._action = MoveToPositionAction(self, on_complete=self._on_action_complete)

    def _on_action_complete(self) -> None:
        if self._done_published:
            return
        self._done_published = True
        # Publish done signal before shutting down, so downstream nodes can proceed
        try:
            self._pub_done.publish(Bool(data=True))
            self.get_logger().info("Move-to-start reached target; published done=true.")
        except Exception:
            self.get_logger().warn("Failed to publish move_to_start/done; proceeding to shutdown.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveToStartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()