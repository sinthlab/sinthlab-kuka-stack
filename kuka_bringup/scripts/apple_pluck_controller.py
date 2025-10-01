#!/usr/bin/env python3

"""
apple_pluck_controller.py

Applies a gradually increasing pulling force to an apple link attached to the robot's end effector in Gazebo.
When a wrist joint torque threshold is exceeded, the apple is "plucked" by requesting removal of the fixed joint
that attaches it. A manual release service is also provided.

 Assumptions:
 - A ros_gz world exposes:
     /world/<world>/apply_link_wrench   (ros_gz_interfaces/ApplyLinkWrench)
     /world/<world>/delete_entity OR /world/<world>/remove (ros_gz_interfaces/DeleteEntity)
 - The apple is attached via a joint named <robot_name>::apple_fixed_joint
 - Apple link: <robot_name>::apple_link
 - End-effector link: <robot_name>::<robot_name>_link_ee
 - JointState topic: /<robot_name>/joint_states

Parameters:
    world_name (string)
    robot_name (string)
    pull_force_max (double)
    pull_ramp_rate (double)  N/s
    pull_direction_world (double[3])
    release_torque_threshold (double)
    apply_duration_s (double)
    wrist_joint_name (string, default A7)

Service:
    <ns>/release_apple (std_srvs/Trigger)
"""
from dataclasses import dataclass
import math
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from ros_gz_interfaces.srv import ApplyLinkWrench, DeleteEntity

@dataclass
class Config:
    world_name: str = "empty"  # can be overridden; script will try fallbacks if services not found
    robot_name: str = "lbr"
    apple_link_name: str = "apple_link"
    ee_link_suffix: str = "_link_ee"
    wrist_joint_name: str = "A7"
    pull_force_max: float = 15.0
    pull_ramp_rate: float = 3.0
    pull_direction_world: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    release_torque_threshold: float = 8.0
    apply_duration_s: float = 0.02
    period_s: float = 0.01


class ApplePluckController(Node):
    def __init__(self):
        super().__init__("apple_pluck_controller")
        self.cfg = Config()

        # Parameters
        self.cfg.world_name = self.declare_parameter("world_name", self.cfg.world_name).get_parameter_value().string_value
        self.cfg.robot_name = self.declare_parameter("robot_name", self.cfg.robot_name).get_parameter_value().string_value
        self.cfg.pull_force_max = float(self.declare_parameter("pull_force_max", self.cfg.pull_force_max).value)
        self.cfg.pull_ramp_rate = float(self.declare_parameter("pull_ramp_rate", self.cfg.pull_ramp_rate).value)
        self.cfg.release_torque_threshold = float(self.declare_parameter("release_torque_threshold", self.cfg.release_torque_threshold).value)
        self.cfg.apply_duration_s = float(self.declare_parameter("apply_duration_s", self.cfg.apply_duration_s).value)
        self.cfg.wrist_joint_name = self.declare_parameter("wrist_joint_name", self.cfg.wrist_joint_name).get_parameter_value().string_value
        pull_dir = self.declare_parameter("pull_direction_world", list(self.cfg.pull_direction_world)).value
        if len(pull_dir) == 3:
            self.cfg.pull_direction_world = tuple(float(v) for v in pull_dir)  # type: ignore

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        # Direct service names (no fallback scanning)
        self.apply_service = f"/world/{self.cfg.world_name}/apply_link_wrench"
        self.delete_service = f"/world/{self.cfg.world_name}/delete_entity"
        self.delete_service_fallback = f"/world/{self.cfg.world_name}/remove"
        self.apply_cli = self.create_client(ApplyLinkWrench, self.apply_service)
        self.delete_cli = self.create_client(DeleteEntity, self.delete_service)
        self.delete_cli_fb = self.create_client(DeleteEntity, self.delete_service_fallback)

        # Subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState,
            f"/{self.cfg.robot_name}/joint_states",
            self.on_joint_state,
            qos,
        )

        # Manual release service
        self.release_srv = self.create_service(Trigger, "release_apple", self.on_release)

        # Timer
        self.timer = self.create_timer(self.cfg.period_s, self.on_timer)

        self._t_start = self.get_clock().now()
        self._last_js = JointState()
        self._released = False
        self._remove_requested = False
        self._ready = False
        self._last_js_time = None
        self._last_force_time = None

        self.get_logger().info(f"ApplePluckController world={self.cfg.world_name} robot={self.cfg.robot_name} threshold={self.cfg.release_torque_threshold}Nm")
        self.get_logger().info(f"Expect services: {self.apply_service}, {self.delete_service} (fallback {self.delete_service_fallback})")

    def on_joint_state(self, msg: JointState):
        self._last_js = msg
        self._last_js_time = self.get_clock().now()
        # Mark ready once at least one joint effort entry exists (controllers running)
        if not self._ready and msg.effort:
            self._ready = True
            self.get_logger().info("Joint states received; controller activity detected. Force application will begin.")

    def on_release(self, req, resp):
        self._released = True
        resp.success = True
        resp.message = "Released manually"
        self.get_logger().warn("Apple released via service call.")
        return resp

    def wrist_torque(self) -> float:
        try:
            if self.cfg.wrist_joint_name in self._last_js.name:
                idx = self._last_js.name.index(self.cfg.wrist_joint_name)
            else:
                idx = len(self._last_js.effort) - 1
            if 0 <= idx < len(self._last_js.effort):
                return abs(self._last_js.effort[idx])
        except Exception:
            pass
        return 0.0

    def on_timer(self):
        # Initial readiness checks (avoid spamming Gazebo before controllers & services up)
        if not self._ready:
            # Provide periodic status
            if (self._t_start.nanoseconds % int(5e9)) < 1e7:  # coarse gating using start time remainder
                self.get_logger().debug("Waiting for joint states / controllers before applying forces...")
            return

        # Post-release: request joint removal once
        if self._released:
            if not self._remove_requested:
                target = None
                if self.delete_cli.service_is_ready():
                    target = self.delete_cli
                elif self.delete_cli_fb.service_is_ready():
                    target = self.delete_cli_fb
                if target:
                    req = DeleteEntity.Request()
                    req.name = f"{self.cfg.robot_name}::apple_fixed_joint"
                    target.call_async(req)
                    self._remove_requested = True
                    self.get_logger().info("Requested deletion of apple_fixed_joint.")
            return

        tau = self.wrist_torque()
        if tau >= self.cfg.release_torque_threshold:
            self._released = True
            self.get_logger().warn(
                f"Release threshold reached tau={tau:.2f} >= {self.cfg.release_torque_threshold:.2f} Nm; releasing apple."
            )
            return

        dt = (self.get_clock().now() - self._t_start).nanoseconds * 1e-9
        dirx, diry, dirz = self.cfg.pull_direction_world
        norm = math.sqrt(dirx * dirx + diry * diry + dirz * dirz)
        if norm < 1e-9:
            return
        dirx, diry, dirz = dirx / norm, diry / norm, dirz / norm
        pull_mag = min(self.cfg.pull_force_max, self.cfg.pull_ramp_rate * dt)
        fx, fy, fz = dirx * pull_mag, diry * pull_mag, dirz * pull_mag

        if not self.apply_cli.service_is_ready():
            if self._last_force_time is None:
                self.get_logger().warn("Waiting for apply_link_wrench service...")
            return

        # Apply pulling wrench on apple link
        req_pull = ApplyLinkWrench.Request()
        req_pull.link_name = f"{self.cfg.robot_name}::{self.cfg.apple_link_name}"
        req_pull.reference_frame = "world"
        req_pull.wrench.force.x = fx
        req_pull.wrench.force.y = fy
        req_pull.wrench.force.z = fz
        req_pull.wrench.torque.x = 0.0
        req_pull.wrench.torque.y = 0.0
        req_pull.wrench.torque.z = 0.0
        req_pull.duration.sec = int(self.cfg.apply_duration_s)
        req_pull.duration.nanosec = int((self.cfg.apply_duration_s - int(self.cfg.apply_duration_s)) * 1e9)
        self.apply_cli.call_async(req_pull)

        # Opposing wrench on EE link
        req_resist = ApplyLinkWrench.Request()
        req_resist.link_name = f"{self.cfg.robot_name}::{self.cfg.robot_name}{self.cfg.ee_link_suffix}"
        req_resist.reference_frame = "world"
        req_resist.wrench.force.x = -fx
        req_resist.wrench.force.y = -fy
        req_resist.wrench.force.z = -fz
        req_resist.wrench.torque.x = 0.0
        req_resist.wrench.torque.y = 0.0
        req_resist.wrench.torque.z = 0.0
        req_resist.duration = req_pull.duration
        self.apply_cli.call_async(req_resist)
        self._last_force_time = self.get_clock().now()


def main():
    rclpy.init()
    node = ApplePluckController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
