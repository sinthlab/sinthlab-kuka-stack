#!/usr/bin/env python3

import rclpy
import time
from typing import Optional

from rclpy.node import Node as rclpyNode
from std_msgs.msg import Bool

from helpers.common_threshold import get_required_param, DoneGate, create_transient_bool_publisher
from actions.move_to_position import MoveToPositionAction


class MoveToStartNode(rclpyNode):
    """
    ROS 2 wrapper that hosts the reusable move-to-pos action to move the arm to a desired start position.
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
        
        self._move_done_topic = str(get_required_param(self, "move_done_topic"))
        self._move_done = create_transient_bool_publisher(self, self._move_done_topic)
        self._done_published = False
        
        # Flag to control if we are waiting for a gating event before starting motion.
        self.start_action_needed = bool(get_required_param(self, "start_action_needed"))

        self._start_gate_topic: Optional[str] = None
        self._start_gate: Optional[DoneGate] = None
        if self.start_action_needed:
            self._start_gate_topic = str(get_required_param(self, "start_gate_topic"))
            self._start_gate = DoneGate(self, self._start_gate_topic)

        self._action = MoveToPositionAction(
            self,
            to_start=self._to_start_action,
            on_complete=self._on_action_complete,
        )
    
    def _to_start_action(self) -> bool:
        if not self.start_action_needed:
            return True
        if self._start_gate is None:
            return False
        return self._start_gate.done

    def _on_action_complete(self) -> None:
        if self._done_published:
            return
        self._done_published = True
        # Publish done signal before shutting down, so downstream nodes can proceed
        try:
            self._move_done.publish(Bool(data=True))
            self.get_logger().info(f"Move-to-start reached target; published done=true in {self._move_done_topic}.")
            # Allow some time for subscribers to latch onto the message
            time.sleep(get_required_param(self, "subscriber_latch_delay_sec"))
        except Exception:
            self.get_logger().warn(f"Failed to publish to {self._move_done_topic}; proceeding to shutdown.")
        self._shutdown()

    def _shutdown(self) -> None:
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


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