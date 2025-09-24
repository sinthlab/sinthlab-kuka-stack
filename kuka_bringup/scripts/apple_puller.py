#!/usr/bin/env python3

import math
import threading
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Wrench
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from ros_gz_interfaces.srv import ApplyLinkWrench, Entity

# This node simulates an external pulling force applied to the apple link in Gazebo
# via /world/*/apply_link_wrench, while the robot applies a resistive (damping) wrench.
# When a measured torque/effort at the wrist exceeds a threshold, it triggers a release.


@dataclass
class Config:
    world_name: str = 'empty'
    apple_model_name: str = 'lbr'  # we spawn the robot with -name <robot_name>, apple is attached, we target the EE link
    apple_link_name: str = 'apple_link'
    ee_link_name: str = 'lbr_link_ee'
    pull_force_max: float = 15.0  # N along +Z world by default
    pull_ramp_rate: float = 3.0   # N/s
    pull_direction_world: tuple = (0.0, 0.0, 1.0)
    damping_gain: float = 2.0  # N/(rad/s) proxy via wrist vel sign
    hold_force: float = 8.0    # N robot can resist up to
    release_torque_threshold: float = 8.0  # Nm threshold at wrist joint A7
    period_s: float = 0.01
    apply_duration_s: float = 0.02  # duration for each wrench request in gz


class ApplePuller(Node):
    def __init__(self):
        super().__init__('apple_puller')
        self.cfg = Config()

        # QoS profiles
        qos_fast = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        # Declare overrideable parameters
        self.cfg.world_name = self.declare_parameter('world_name', self.cfg.world_name).get_parameter_value().string_value or self.cfg.world_name
        self.cfg.apple_model_name = self.declare_parameter('robot_name', self.cfg.apple_model_name).get_parameter_value().string_value or self.cfg.apple_model_name
        self.cfg.pull_force_max = float(self.declare_parameter('pull_force_max', self.cfg.pull_force_max).value)
        self.cfg.pull_ramp_rate = float(self.declare_parameter('pull_ramp_rate', self.cfg.pull_ramp_rate).value)
        self.cfg.hold_force = float(self.declare_parameter('hold_force', self.cfg.hold_force).value)
        self.cfg.release_torque_threshold = float(self.declare_parameter('release_torque_threshold', self.cfg.release_torque_threshold).value)
        # Derive EE link from robot_name
        self.cfg.ee_link_name = f"{self.cfg.apple_model_name}_link_ee"

        # Gazebo ApplyLinkWrench service client (bridged by parameter_bridge)
        self.apply_srv_name = f"/world/{self.cfg.world_name}/apply_link_wrench"
        self.apply_cli = self.create_client(ApplyLinkWrench, self.apply_srv_name)
        # Remove entity service client to delete the fixed joint on release
        self.remove_srv_name = f"/world/{self.cfg.world_name}/remove"
        self.remove_cli = self.create_client(Entity, self.remove_srv_name)

        # Subscribe to joint states to monitor efforts/velocities under robot namespace
        self.joint_state_sub = self.create_subscription(
            JointState,
            f"/{self.cfg.apple_model_name}/joint_states",
            self.on_joint_state,
            qos_fast,
        )

        # Optional: a service to release manually
        self.release_srv = self.create_service(Trigger, 'release_apple', self.on_release)

        self.timer = self.create_timer(self.cfg.period_s, self.on_timer)

        self._last_joint_state = JointState()
        self._released = False
        self._did_remove = False
        self._t0 = self.get_clock().now()
        self.get_logger().info(
            f"ApplePuller started. Using services: apply={self.apply_srv_name}, remove={self.remove_srv_name}; robot={self.cfg.apple_model_name}"
        )

    def on_joint_state(self, msg: JointState):
        self._last_joint_state = msg

    def on_release(self, request, response):
        self._released = True
        response.success = True
        response.message = 'Released by service call.'
        self.get_logger().warn('Apple released via service call.')
        return response

    def current_wrist_torque(self) -> float:
        try:
            # Assume A7 is last joint
            if 'A7' in self._last_joint_state.name:
                idx = self._last_joint_state.name.index('A7')
            else:
                idx = len(self._last_joint_state.effort) - 1
            return abs(self._last_joint_state.effort[idx]) if idx >= 0 and idx < len(self._last_joint_state.effort) else 0.0
        except Exception:
            return 0.0

    def on_timer(self):
        if self._released:
            # Ensure we remove the joint once
            if not self._did_remove and self.remove_cli.service_is_ready():
                rem = Entity.Request()
                rem.name = f"{self.cfg.apple_model_name}::apple_fixed_joint"
                try:
                    rem.type = 7  # JOINT
                except Exception:
                    pass
                try:
                    rem.recursive = False
                except Exception:
                    pass
                self.remove_cli.call_async(rem)
                self._did_remove = True
                self.get_logger().info("Requested removal of apple_fixed_joint (release).")
            return

        # Check release condition
        tau = self.current_wrist_torque()
        if tau >= self.cfg.release_torque_threshold:
            self._released = True
            self.get_logger().warn(f"Release threshold reached: tau={tau:.2f} Nm >= {self.cfg.release_torque_threshold} Nm. Releasing apple.")
            # Removal is handled at the top of this callback on next tick
            return

        # Time since start
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9

        # Compute pulling force along world direction (ramped)
        dirx, diry, dirz = self.cfg.pull_direction_world
        norm = math.sqrt(dirx*dirx + diry*diry + dirz*dirz)
        if norm < 1e-6:
            return
        dirx, diry, dirz = dirx/norm, diry/norm, dirz/norm
        pull_mag = min(self.cfg.pull_force_max, self.cfg.pull_ramp_rate * t)
        fx = dirx * pull_mag
        fy = diry * pull_mag
        fz = dirz * pull_mag

        # Damping term from wrist joint velocity as a crude proxy
        v = 0.0
        if self._last_joint_state and self._last_joint_state.velocity:
            try:
                if 'A7' in self._last_joint_state.name:
                    idx = self._last_joint_state.name.index('A7')
                else:
                    idx = len(self._last_joint_state.velocity) - 1
                v = self._last_joint_state.velocity[idx]
            except Exception:
                v = 0.0
        # Resistive force from the arm at EE: constant hold + damping (opposite to pull dir)
        hold = min(self.cfg.hold_force, pull_mag)
        damp = self.cfg.damping_gain * v
        resist_mag = max(0.0, hold + abs(damp))

        # Build service request: pull apple outward
        if not self.apply_cli.service_is_ready():
            self.apply_cli.wait_for_service(timeout_sec=0.0)
            if not self.apply_cli.service_is_ready():
                return

        req = ApplyLinkWrench.Request()
        # Common definition used by ros_gz_interfaces: link_name and reference_frame are strings
        # Note: some versions use scoped names like 'model::link'
        req.link_name = f"{self.cfg.apple_model_name}::{self.cfg.apple_link_name}"
        req.reference_frame = "world"
        req.wrench.force.x = fx
        req.wrench.force.y = fy
        req.wrench.force.z = fz
        req.wrench.torque.x = 0.0
        req.wrench.torque.y = 0.0
        req.wrench.torque.z = 0.0
        req.duration.sec = int(self.cfg.apply_duration_s)
        req.duration.nanosec = int((self.cfg.apply_duration_s - int(self.cfg.apply_duration_s)) * 1e9)

        self.apply_cli.call_async(req)

        # Apply opposing wrench to EE to simulate robot resisting
        req2 = ApplyLinkWrench.Request()
        req2.link_name = f"{self.cfg.apple_model_name}::{self.cfg.ee_link_name}"
        req2.reference_frame = "world"
        req2.wrench.force.x = -dirx * resist_mag
        req2.wrench.force.y = -diry * resist_mag
        req2.wrench.force.z = -dirz * resist_mag
        req2.wrench.torque.x = 0.0
        req2.wrench.torque.y = 0.0
        req2.wrench.torque.z = 0.0
        req2.duration.sec = req.duration.sec
        req2.duration.nanosec = req.duration.nanosec
        self.apply_cli.call_async(req2)

        # Normal path continues without removal; removal handled at beginning when released


def main():
    rclpy.init()
    node = ApplePuller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
