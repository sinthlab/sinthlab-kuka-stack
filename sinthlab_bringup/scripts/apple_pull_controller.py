#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import JointState


class ApplePullController(Node):
    def __init__(self):
        super().__init__('apple_pull_controller')

        # Parameters
        self.declare_parameter('robot_name', 'lbr')
        self.declare_parameter('pull_force', 5.0)  # Newtons along -Z of apple frame
        self.declare_parameter('effort_threshold', 15.0)  # Stop when any joint effort exceeds
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('use_torque', True)  # Apply torque instead of force for visible rotation

        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.pull_force = float(self.get_parameter('pull_force').value)
        # Threshold to stop applying force. If <= 0, threshold is disabled.
        self.threshold = float(self.get_parameter('effort_threshold').value)
        self.rate = float(self.get_parameter('publish_rate').value)
        self.use_torque = bool(self.get_parameter('use_torque').value)

        # Topic exposed via ros_gz_bridge for entity wrench
        # Assumes a bridge is created for /<robot_name>/apple_link/wrench geometry_msgs/Wrench -> gz.msgs.Wrench
        wrench_topic = f'/{self.robot_name}/apple_link/wrench'
        self.pub_wrench = self.create_publisher(Wrench, wrench_topic, 10)
        self.js_sub = self.create_subscription(JointState, f'/{self.robot_name}/joint_states', self.on_joint_states, 10)

        self.timer = self.create_timer(1.0 / max(self.rate, 1.0), self.publish_wrench)
        self.stop = False

    def on_joint_states(self, msg: JointState):
        # Stop when any effort magnitude exceeds threshold (if enabled)
        max_e = 0.0
        for e in msg.effort:
            if self.threshold > 0.0 and abs(e) >= self.threshold:
                if not self.stop:
                    self.get_logger().info(f'Threshold reached: |effort|={abs(e):.2f} >= {self.threshold:.2f}. Stopping pull.')
                self.stop = True
                return
            max_e = max(max_e, abs(e))
        # No visualization state here; visualization handled by separate node

    def publish_wrench(self):
        if self.stop:
            return
        w = Wrench()
        # For visible motion with a passive joint, applying a torque about the joint axis is effective.
        if self.use_torque:
            # Rotate about +Z (matches apple_yaw_joint axis)
            w.torque.z = -abs(self.pull_force)
        else:
            # Pull along -Z of apple frame to emulate downward tug while apple faces down
            w.force.z = -abs(self.pull_force)
        self.pub_wrench.publish(w)




def main():
    rclpy.init()
    node = ApplePullController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
