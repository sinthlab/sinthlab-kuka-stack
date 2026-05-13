#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lbr_fri_msgs.msg import LBRState
import numpy as np
import optas
import csv
import time
import os

class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')
        self.declare_parameter('robot_description', '')
        self.declare_parameter('state_topic', '/lbr/state')
        
        state_topic = self.get_parameter('state_topic').value
        robot_description = self.get_parameter('robot_description').value
        
        if not robot_description:
            self.get_logger().error("robot_description parameter is required!")
            
        self.robot = optas.RobotModel(urdf_string=robot_description)
        self.fk_func = self.robot.get_link_transform_function(
            link="lbr_link_ee", base_link="lbr_link_0", numpy_output=True
        )
        
        self.trajectory_data = []
        self.start_time = None
        
        self.sub = self.create_subscription(LBRState, state_topic, self.state_callback, 10)
        self.get_logger().info(f"Recording trajectory on {state_topic}... Press Ctrl+C to stop and save.")

    def state_callback(self, msg: LBRState):
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time() - self.start_time
        joints = np.array(msg.measured_joint_position)
        
        # Calculate FK
        transform = self.fk_func(joints)
        x = transform[0, 3]
        y = transform[1, 3]
        z = transform[2, 3]
        
        self.trajectory_data.append([current_time, x, y, z])

    def save_data(self):
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"robot_trajectory_{timestamp}.csv"
        filepath = os.path.join(os.path.dirname(__file__), filename)
        
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["time", "x", "y", "z"])
            writer.writerows(self.trajectory_data)
        self.get_logger().info(f"Saved {len(self.trajectory_data)} points to {filepath}")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, saving trajectory...")
    finally:
        node.save_data()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
