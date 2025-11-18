#!/usr/bin/env python3

import rclpy
from rclpy.node import Node as rclpyNode
from std_msgs.msg import Bool
import time

from helpers.common_threshold import DoneGate, create_transient_bool_publisher, get_required_param
from actions.admittance_controller import AdmittanceControlAction


class AdmittanceControlNode(rclpyNode):
    """ROS 2 wrapper that hosts the admittance control action."""

    def __init__(self) -> None:
        super().__init__(
            "admittance_control",
            automatically_declare_parameters_from_overrides=True,
        )

        self._move_done_topic = str(get_required_param(self, "move_done_topic"))
        self._move_done_gate = DoneGate(self, self._move_done_topic)

        self._admittance_in_action_topic = str(get_required_param(self, "admittance_start_topic"))
        self._admittance_in_action = create_transient_bool_publisher(self, self._admittance_in_action_topic)
        self._admittance_in_action_published = False

        self._force_release_topic = str(get_required_param(self, "force_release_done_topic"))
        self._force_release_gate = DoneGate(self, self._force_release_topic)
        self._hold_ready_topic = str(get_required_param(self, "hold_ready_topic"))
        self._hold_ready_gate = DoneGate(self, self._hold_ready_topic)

        self._action = AdmittanceControlAction(self, to_start=self._to_start_action, in_action=self._in_action, on_complete=self._on_action_complete)
    
    def _to_start_action(self) -> bool:
        return self._move_done_gate and self._move_done_gate.done
	
    def _in_action(self) -> None:
        if self._admittance_in_action_published:
            return
        self._admittance_in_action_published = True
        try:
            self._admittance_in_action.publish(Bool(data=True))
            self.get_logger().info(f"Admittance control action has started; published done=true in {self._admittance_in_action_topic}.")
            # Allow some time for subscribers to latch onto the message
            time.sleep(get_required_param(self, "subscriber_latch_delay_sec"))
        except Exception:
            self.get_logger().warn(f"Failed to publish to {self._admittance_in_action_topic};")
    
    def _on_action_complete(self) -> None:
        if self._hold_ready_gate.done:
            self.get_logger().info(
                f"Detected displacement hold ready on '{self._hold_ready_topic}'; stopping admittance control."
            )
            self._shutdown()
            return

        if not self._force_release_gate.done:
            return
        self.get_logger().info(
            f"Detected force release on '{self._force_release_topic}'; stopping admittance control."
        )
        time.sleep(get_required_param(self, "subscriber_latch_delay_sec"))
        self._shutdown()

    def _shutdown(self) -> None:
        try:
            self.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
	
def main(args=None) -> None:
	rclpy.init(args=args)
	node = AdmittanceControlNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	node.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()
