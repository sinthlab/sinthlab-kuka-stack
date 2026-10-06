#!/usr/bin/env python3
"""Admittance free move -- the arm goes wherever it is pushed and stays wherever it is let go.

HOW IT WORKS ON THIS STACK. The cabinet runs Cartesian impedance (LbrImpedanceControlServer) and ROS
streams the spring's EQUILIBRIUM to kuka_clik_controller, which does the IK. This action OWNS that
equilibrium and moves it at a velocity proportional to the hand's force:

    v = (F_ext - tare) / damping        (threshold, filtered, speed-capped)
    x_eq += v * dt                      (leashed to the arm)

The cabinet spring then pulls the arm after the equilibrium. Push harder -> it moves faster; let go
-> it stops where it is and holds (the spring still holds it there). There is no goal and no wall:
`damping_ns_per_m` is the only resistance, and lower is lighter.

WHY NOT FOLLOW THE MEASURED POSE. Setting the equilibrium to where the arm IS gives zero spring
force, so nothing holds the arm against the gravity-compensation residual and it sinks -- that is
the collapse the threshold "give-in" once caused (see freeze_at_pose.py). Here the equilibrium is
integrated from FORCE, never copied from the arm, so an untouched arm stays put. Upstream
lbr_ros2_control/AdmittanceController integrates from the measured joints and assumes a stiff
position-controlled arm, which is why it is not used here.

WHAT KEEPS IT SAFE (none of these is a boundary on where the arm may go)
  * leash_m        the equilibrium never runs more than this ahead of the arm, so the spring force is
                   bounded by K * leash_m whatever the force estimate says. At the arm's reach or a
                   joint limit (where the CLIK stops) the arm simply stops there.
  * max_speed_mps  cap on how fast the equilibrium moves.
  * accel_limit_mps2  the equilibrium's speed may only change this fast. The force estimate is noisy
                   and crosses the threshold on and off, and v = F / damping passes all of that
                   straight through as jolts; limiting the acceleration turns them into smooth
                   speed changes. 0 = off.
  * tare           the force at rest (the gravity-model residual, ~5 N here) is measured during
                   tare_sec at the start of each trial and subtracted, so a resting arm does not creep.
  * deadband_n     a THRESHOLD: a (tared) force below it moves nothing, so estimator noise cannot
                   creep the arm. Above it the WHOLE force counts -- it is not subtracted from every
                   push -- with a short ramp (one dead band wide) so the start is not a jolt.
Orientation is held at the start orientation (rotational stiffness on the SmartPad).

THE FORCE. `force_torque_broadcaster/wrench` is lbr's EstimatedWrenchInterface: the external joint
torques mapped through the Jacobian (with its own per-axis dead band, iiwa7_hardware_controllers.yaml)
and published in the EE frame (lbr_link_ee). It is the force the hand applies to the tool. Rotated
here into the base frame with the measured EE orientation.
"""
from __future__ import annotations

import time
from typing import Optional

import numpy as np
from geometry_msgs.msg import PoseStamped, WrenchStamped
from lbr_fri_idl.msg import LBRState
from rclpy.node import Node as rclpyNode

import optas

from sinthlab_bringup.helpers.common_threshold import DebugTicker, get_optional_param, get_required_param


class AdmittanceMoveAction:
    def __init__(self, node: rclpyNode, *, param_prefix: str = "free_move") -> None:
        self._node = node
        self._p = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix
        log = node.get_logger()

        self.base_link = str(get_required_param(node, "base_link"))
        self.ee_link = str(get_required_param(node, "end_effector_link"))
        state_topic = str(get_required_param(node, "state_topic"))
        robot_name = node.get_namespace().strip("/")
        cmd_topic = (f"/{robot_name}/kuka_clik_controller/target_frame" if robot_name
                     else "/kuka_clik_controller/target_frame")

        robot_description = str(node.get_parameter("robot_description").value) \
            if node.has_parameter("robot_description") else ""
        self._fk = optas.RobotModel(urdf_string=robot_description).get_link_transform_function(
            link=self.ee_link, base_link=self.base_link, numpy_output=True)

        self._leash = float(get_required_param(node, self._p + "leash_m"))
        self._tare_max = float(get_optional_param(node, self._p + "tare_max_n", 10.0))
        self._dbg = DebugTicker(float(get_optional_param(node, self._p + "debug_log_rate_hz", 2.0)))
        self.reload()

        self._active = False
        self._moving = False
        self._anchor: Optional[np.ndarray] = None       # start pose (4x4), captured ONCE per trial
        self._eq: Optional[np.ndarray] = None           # commanded equilibrium position (3,)
        self._p_meas: Optional[np.ndarray] = None
        self._f_msg: Optional[np.ndarray] = None        # latest wrench, EE frame
        self._f_filt = np.zeros(3)
        self._f_used = np.zeros(3)
        self._v = np.zeros(3)                           # equilibrium velocity (accel-limited)
        self._tare = np.zeros(3)
        self._tare_samples = []
        self._t_prev: Optional[float] = None

        node.create_subscription(LBRState, state_topic, self._on_state, 1)
        node.create_subscription(WrenchStamped, "force_torque_broadcaster/wrench", self._on_wrench, 1)
        self._pub = node.create_publisher(PoseStamped, cmd_topic, 1)
        log.info(f"Admittance free move: no boundary, leash {self._leash} m")

    # ------------------------------------------------------------------ parameters
    def reload(self) -> None:
        """Re-read the live feel parameters. Called at a trial boundary."""
        n, p = self._node, self._p
        self._damping = max(1.0, float(get_required_param(n, p + "damping_ns_per_m")))
        self._deadband = max(0.0, float(get_required_param(n, p + "deadband_n")))
        self._max_speed = max(0.0, float(get_required_param(n, p + "max_speed_mps")))
        self._accel = max(0.0, float(get_optional_param(n, p + "accel_limit_mps2", 0.0)))
        self._tau = max(0.0, float(get_optional_param(n, p + "force_filter_tau_sec", 0.05)))
        self._tare_sec = max(0.0, float(get_optional_param(n, p + "tare_sec", 1.0)))
        self._debug = bool(get_optional_param(n, p + "debug_log_enabled", False))
        n.get_logger().info(
            f"Free move feel: damping {self._damping:g} N s/m, threshold {self._deadband:g} N, "
            f"max speed {self._max_speed:g} m/s, accel limit "
            f"{f'{self._accel:g} m/s^2' if self._accel > 0 else 'off'}")

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> None:
        """Hold the start pose and measure the resting force (tare). Motion starts at enable_motion()."""
        self._active, self._moving = True, False
        self._anchor = self._eq = None
        self._tare_samples = []
        self._f_filt[:] = 0.0
        self._v[:] = 0.0
        self._t_prev = None
        self._t_start = time.monotonic()

    def enable_motion(self) -> None:
        """Finish the tare and let the arm move. Call at the go cue."""
        if self._tare_samples:
            tare = np.mean(self._tare_samples, axis=0)
            if np.linalg.norm(tare) <= self._tare_max:
                self._tare = tare
            else:   # someone was pushing during the tare: keep the previous one rather than learn the push
                self._node.get_logger().warn(
                    f"Free move: resting force {np.round(tare, 1)} N exceeds tare_max_n "
                    f"({self._tare_max:g} N) -- was the arm being touched? Keeping the previous tare "
                    f"{np.round(self._tare, 1)} N.")
        self._node.get_logger().info(f"Free move: tare {np.round(self._tare, 2)} N (base frame); moving.")
        self._moving = True

    def stop(self) -> None:
        """Stop moving. The CLIK keeps the last equilibrium, so the arm holds where it is."""
        self._active = self._moving = False

    # ------------------------------------------------------------------ outputs
    def offset_from_anchor(self) -> Optional[np.ndarray]:
        if not self._active or self._anchor is None or self._p_meas is None:
            return None
        return self._p_meas - self._anchor[0:3, 3]

    def record_extra_header(self):
        return ["eq_dx", "eq_dy", "eq_dz", "f_adm_x", "f_adm_y", "f_adm_z"]

    def record_extra(self, measured_T):
        if self._anchor is None or self._eq is None:
            return [float("nan")] * 6
        d = self._eq - self._anchor[0:3, 3]
        return [round(float(v), 5) for v in d] + [round(float(v), 3) for v in self._f_used]

    # ------------------------------------------------------------------ loop
    def _on_wrench(self, msg: WrenchStamped) -> None:
        f = msg.wrench.force
        self._f_msg = np.array([f.x, f.y, f.z], dtype=float)

    def _threshold(self, f: np.ndarray) -> np.ndarray:
        """Zero below the dead band; the FULL force from twice the dead band; a linear ramp between."""
        mag = float(np.linalg.norm(f))
        if self._deadband <= 0.0:
            return f
        if mag <= self._deadband:
            return np.zeros(3)
        return f * min(1.0, (mag - self._deadband) / self._deadband)

    def _on_state(self, msg: LBRState) -> None:
        q = np.array(msg.measured_joint_position, dtype=float)
        if not np.all(np.isfinite(q)):
            return
        T = self._fk(q)
        self._p_meas = T[0:3, 3].copy()
        if not self._active:
            return
        if self._anchor is None:
            # Anchor on the COMMANDED pose where there is one: the arm rests a few mm below it under
            # the gravity residual, and anchoring on the measured pose would lower the hold by that much.
            q_cmd = np.array(msg.commanded_joint_position, dtype=float)
            use_cmd = np.all(np.isfinite(q_cmd)) and np.any(q_cmd != 0.0)
            self._anchor = self._fk(q_cmd) if use_cmd else T.copy()
            self._eq = self._anchor[0:3, 3].copy()
            self._node.get_logger().info(f"Free move anchored at EE {np.round(self._eq, 3)}")

        now = time.monotonic()
        dt = 0.0 if self._t_prev is None else min(max(now - self._t_prev, 0.0), 0.05)
        self._t_prev = now
        f_base = T[0:3, 0:3] @ self._f_msg if self._f_msg is not None else np.zeros(3)

        if not self._moving:
            if now - self._t_start <= max(self._tare_sec, 0.1) or not self._tare_samples:
                self._tare_samples.append(f_base)
            self._publish(self._eq)
            return

        # Force -> velocity.
        f = f_base - self._tare
        k = 1.0 if self._tau <= 0.0 or dt <= 0.0 else dt / (self._tau + dt)
        self._f_filt += k * (f - self._f_filt)
        self._f_used = self._threshold(self._f_filt)
        v = self._f_used / self._damping
        speed = float(np.linalg.norm(v))
        if speed > self._max_speed > 0.0:
            v *= self._max_speed / speed
        if self._accel > 0.0 and dt > 0.0:                # smooth: the speed changes at most accel * dt
            dv = v - self._v
            step = float(np.linalg.norm(dv))
            if step > self._accel * dt:
                v = self._v + dv * (self._accel * dt / step)

        eq = self._eq + v * dt
        lead = eq - self._p_meas                          # leash: never far ahead of the arm
        dist = float(np.linalg.norm(lead))
        if dist > self._leash:
            eq = self._p_meas + lead * (self._leash / dist)
        # Remember the speed actually applied (after the leash), so a held-back equilibrium does not
        # build up speed it would release as a lurch.
        self._v = (eq - self._eq) / dt if dt > 0.0 else v
        self._eq = eq
        self._publish(eq)

        if self._debug and self._dbg.tick(max(dt, 1e-3)):
            self._node.get_logger().info(
                f"free move: F={np.round(self._f_used, 1)} N  v={np.round(v, 3)} m/s  "
                f"eq-start={np.round(eq - self._anchor[0:3, 3], 3)} m")

    def _publish(self, xyz: np.ndarray) -> None:
        from scipy.spatial.transform import Rotation as R
        cmd = PoseStamped()
        cmd.header.frame_id = self.base_link
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        cmd.pose.position.x, cmd.pose.position.y, cmd.pose.position.z = (float(v) for v in xyz)
        qx, qy, qz, qw = R.from_matrix(self._anchor[0:3, 0:3]).as_quat()
        cmd.pose.orientation.x, cmd.pose.orientation.y = float(qx), float(qy)
        cmd.pose.orientation.z, cmd.pose.orientation.w = float(qz), float(qw)
        self._pub.publish(cmd)
