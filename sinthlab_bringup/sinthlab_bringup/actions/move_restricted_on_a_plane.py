#!/usr/bin/env python3
from __future__ import annotations

from typing import Optional

import numpy as np
import optas  # fast kinematics
from geometry_msgs.msg import PoseStamped
from lbr_fri_idl.msg import LBRState
from rclpy.node import Node as rclpyNode
from scipy.spatial.transform import Rotation

from sinthlab_bringup.helpers.floor import Floor
from sinthlab_bringup.helpers.node_utils import get_launch_param, get_required_param

_AXES = {"x": 0, "y": 1, "z": 2}

# Keys each fixture geometry needs under virtual_fixtures.<profile>. A cartesian_box's bounds
# (z_min, x_min, x_max) are each optional instead: a bound that is absent leaves that side open.
_PROFILE_KEYS = {
    "plane": ("restricted_axis",),
    "maze": ("restricted_axis",),
    "cylinder": ("radius", "center_x", "center_y"),
    "sinusoid": ("amplitude", "spatial_freq", "pull_axis", "osc_axis"),
    "cartesian_box": (),
}
_BOX_BOUNDS = ("z_min", "x_min", "x_max")


class MoveRestrictedOnAPlaneAction:
    """Virtual fixture: streams the measured EE pose, projected onto the allowed geometry, to the
    CLIK controller as the cabinet's Cartesian-impedance equilibrium (FRI position mode).

    Inside the allowed region the projection is the measured pose itself, so the spring is slack and
    the arm moves freely; off it the equilibrium stays on the geometry and the cabinet's 1 kHz spring
    pulls the arm back -- a virtual wall. The geometry is anchored on the start pose each trial.
    """

    def __init__(self, node: rclpyNode, *, param_prefix: str = "") -> None:
        self._node = node
        self._param_prefix = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix
        p = self._param_prefix

        # POSITION mode: stream the fixture equilibrium to the CLIK, which IKs it to joint positions
        # for the cabinet's Cartesian impedance. SOFT-INSIDE does not come from low stiffness: inside
        # the allowed region the projection returns the measured pose, so the spring error -- and hence
        # the force -- is ~zero whatever K is. High K only bites at the walls. What breaks that is
        # TRACKING LAG: if the equilibrium cannot keep up with the hand, the error (and the force) grows
        # with speed. Hence kuka_clik_controller.max_linear_velocity and max_target_step_m must exceed
        # real hand speed -- see config/iiwa7_hardware_controllers.yaml.
        robot_name = node.get_namespace().strip("/")
        cmd_topic = f"/{robot_name}/kuka_clik_controller/target_frame" if robot_name else "/kuka_clik_controller/target_frame"
        self.ee_link = str(get_required_param(node, p + "end_effector_link"))
        self.base_link = str(get_required_param(node, p + "base_link"))
        self._state_sub = node.create_subscription(
            LBRState, str(get_required_param(node, p + "state_topic")), self._state_cb, 1)
        self._cmd_pub = node.create_publisher(PoseStamped, cmd_topic, 1)

        self.robot = optas.RobotModel(urdf_string=str(get_launch_param(node, "robot_description", "")))
        self._fk_func = self.robot.get_link_transform_function(
            link=self.ee_link, base_link=self.base_link, numpy_output=True)
        self._floor = Floor.from_node(node)     # every published equilibrium stays above it

        fx = p + "virtual_fixtures."
        # Cap on how far the commanded equilibrium may move per state tick [m] -- a jerk guard so a
        # sudden push/spike can't make the target leap (the CLIK also clamps velocity downstream).
        self.max_target_step_m = float(get_required_param(node, fx + "max_target_step_m"))
        # WHERE THE FIXTURE ANCHORS ITS ORIGIN. Under cabinet impedance a FREE axis carries no
        # restoring force, so the arm settles a few cm below the commanded start (~4 cm, bounded).
        # Anchoring on the commanded pose then puts a maze that far ABOVE the arm.
        #   anchor_settle_sec  : wait this long after start() before capturing the origin [s]
        #   anchor_on_measured : capture from the MEASURED joints instead of the commanded ones
        self._anchor_settle_sec = float(get_required_param(node, fx + "anchor_settle_sec"))
        self._anchor_on_measured = bool(get_required_param(node, fx + "anchor_on_measured"))

        # Which fixture geometry is active. Required, never defaulted: a fixture that fails to load
        # degrades into "publish the measured pose unchanged" -- an unconstrained arm that feels like a
        # working fixture until you reach a wall -- so it has to be an error, not a warning.
        self.active_profile = str(get_required_param(node, p + "virtual_fixture_profile"))
        self.profile_config = self._load_profile(fx + self.active_profile + ".")
        node.get_logger().info(
            f"Virtual fixture ACTIVE: profile='{self.active_profile}' config={self.profile_config}")

        self._active = False
        self.last_measured_joints = np.zeros(self.robot.ndof)
        self._q_cmd_at_start: Optional[np.ndarray] = None   # commanded joints when the fixture started
        self._initial_transform: Optional[np.ndarray] = None  # the anchor; None until anchored
        self._published_xyz: Optional[np.ndarray] = None      # last commanded translation (step clamp)
        self._anchor_elapsed = 0.0

    def _load_profile(self, prefix: str) -> dict:
        node = self._node
        ptype = str(get_required_param(node, prefix + "type"))
        if ptype not in _PROFILE_KEYS:
            raise ValueError(f"{prefix}type must be one of {sorted(_PROFILE_KEYS)}, got {ptype!r}")
        cfg = {"type": ptype}
        for key in _PROFILE_KEYS[ptype]:
            cfg[key] = get_required_param(node, prefix + key)
        if ptype == "cartesian_box":
            cfg.update({k: float(node.get_parameter(prefix + k).value)
                        for k in _BOX_BOUNDS if node.has_parameter(prefix + k)})
        for key in ("restricted_axis", "pull_axis", "osc_axis"):
            if key in cfg:
                cfg[key] = str(cfg[key]).lower()
                if cfg[key] not in _AXES:
                    raise ValueError(f"{prefix}{key} must be x, y or z, got {cfg[key]!r}")
        if ptype == "sinusoid" and cfg["pull_axis"] == cfg["osc_axis"]:
            raise ValueError(f"{prefix}pull_axis and osc_axis must differ")
        return cfg

    def anchor_position(self):
        """Base-frame EE position the fixture is anchored on, or None until it has anchored this trial.

        Anything else placed relative to the start -- the maze checkpoints and goal -- must use THIS
        origin rather than capturing its own. With anchor_settle_sec the fixture anchors seconds after
        starting, once the arm has sagged (~4 cm), so an earlier capture sits that far off the rails.
        """
        if self._initial_transform is None:
            return None
        return np.array(self._initial_transform[0:3, 3], dtype=float)

    def offset_from_anchor(self):
        """Base-frame offset of the MEASURED EE from the anchor [m], or None until anchored. The
        offset is measured from the same origin as the fixture."""
        if not self._active or self._initial_transform is None:
            return None
        return self._fk_func(self.last_measured_joints)[0:3, 3] - self._initial_transform[0:3, 3]

    def start(self) -> None:
        if self._active:
            self._node.get_logger().warn("MoveRestrictedOnAPlaneAction is already active.")
            return
        # Re-anchor on wherever the recovery move left the arm; a stale anchor would jump the
        # equilibrium and cause velocity faults.
        self._q_cmd_at_start = None
        self._initial_transform = None
        self._published_xyz = None
        self._anchor_elapsed = 0.0
        self._active = True
        self._node.get_logger().info("Restricted Plane Action started, applying boundary IK.")

    def stop(self) -> None:
        self._active = False
        self._node.get_logger().info("Restricted Plane Action stopped.")

    def apply_surface_constraints(self, transform: np.ndarray) -> np.ndarray:
        """Project a 4x4 EE transform onto the active geometry (in place) and return it."""
        cfg = self.profile_config
        ptype = cfg["type"]
        pos = transform[0:3, 3].copy()

        if ptype == "cartesian_box":
            if "z_min" in cfg:
                pos[2] = max(pos[2], cfg["z_min"])
            if "x_min" in cfg:
                pos[0] = max(pos[0], cfg["x_min"])
            if "x_max" in cfg:
                pos[0] = min(pos[0], cfg["x_max"])
        elif ptype == "cylinder":
            d = pos[0:2] - np.array([cfg["center_x"], cfg["center_y"]], dtype=float)
            dist = float(np.linalg.norm(d))
            if dist > cfg["radius"]:
                pos[0:2] -= d * (1.0 - cfg["radius"] / dist)
        elif ptype == "plane":
            # Lock the restricted axis at the start; the other two stay free.
            start = self._initial_transform[0:3, 3]
            i = _AXES[cfg["restricted_axis"]]
            pos[i] = start[i]
        elif ptype == "sinusoid":
            # A 1D rail through the start: osc = start + amplitude * sin(spatial_freq * pull travel),
            # and the third axis pinned. Always projected onto it (no threshold).
            start = self._initial_transform[0:3, 3]
            ip, io = _AXES[cfg["pull_axis"]], _AXES[cfg["osc_axis"]]
            pin = 3 - ip - io
            pos[pin] = start[pin]
            pos[io] = start[io] + float(cfg["amplitude"]) * np.sin(
                float(cfg["spatial_freq"]) * (pos[ip] - start[ip]))

        transform[0:3, 3] = pos
        return transform

    def _state_cb(self, msg: LBRState):
        """Runs at the state rate (~100 Hz): project the measured pose, step-clamp it, publish."""
        if not self._active:
            return
        q_measured = np.array(msg.measured_joint_position)
        # Drop non-finite states (lbr sets them to NaN once FRI leaves COMMANDING_ACTIVE): FK on NaN
        # yields a NaN pose/quaternion, which faults the impedance controller and shows up as
        # TF_NAN_INPUT / TF_DENORMALIZED_QUATERNION downstream.
        if not np.all(np.isfinite(q_measured)):
            return
        self.last_measured_joints = q_measured

        if self._q_cmd_at_start is None:
            q_cmd = np.array(msg.commanded_joint_position)
            bad = np.isnan(q_cmd).any() or np.all(q_cmd == 0)
            self._q_cmd_at_start = q_measured.copy() if bad else q_cmd

        if self._initial_transform is None:
            if self._anchor_elapsed < self._anchor_settle_sec:
                # Hold off: let the arm finish settling so the origin is where it RESTS, and keep the
                # equilibrium on the measured pose meanwhile so we do not fight the settle.
                self._anchor_elapsed += 1.0 / 100.0
                self._publish_pose(self._fk_func(q_measured))
                return
            src = q_measured if self._anchor_on_measured else self._q_cmd_at_start
            self._initial_transform = self._fk_func(src)
            self._published_xyz = self._initial_transform[0:3, 3].copy()
            self._node.get_logger().info(
                f"Fixture anchored at start pose "
                f"({'measured' if self._anchor_on_measured else 'commanded'}"
                f"{f', after {self._anchor_settle_sec:.1f}s settle' if self._anchor_settle_sec > 0 else ''})"
                f" EE={np.round(self._initial_transform[0:3, 3], 3)}")

        # The measured pose projected onto the fixture, orientation held at the start orientation.
        target = self.apply_surface_constraints(self._fk_func(q_measured))
        target[0:3, 0:3] = self._initial_transform[0:3, 0:3]

        # Jerk guard: bound how far the commanded equilibrium may jump in one tick.
        xyz = target[0:3, 3].copy()
        if self.max_target_step_m > 0.0:
            delta = xyz - self._published_xyz
            dist = float(np.linalg.norm(delta))
            if dist > self.max_target_step_m:
                xyz = self._published_xyz + delta * (self.max_target_step_m / dist)
        self._published_xyz = xyz
        target[0:3, 3] = xyz
        self._publish_pose(target)

    def _publish_pose(self, target_pose: np.ndarray) -> None:
        cmd = PoseStamped()
        cmd.header.frame_id = self.base_link
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        # The floor, as a soft wall: the equilibrium never goes below it, so pushing further down
        # stretches the cabinet spring (helpers/floor.py).
        xyz = self._floor.clamp_xyz(target_pose[0:3, 3], target_pose[0:3, 0:3])
        cmd.pose.position.x, cmd.pose.position.y, cmd.pose.position.z = (float(v) for v in xyz)
        qx, qy, qz, qw = Rotation.from_matrix(target_pose[0:3, 0:3]).as_quat()
        cmd.pose.orientation.x, cmd.pose.orientation.y = float(qx), float(qy)
        cmd.pose.orientation.z, cmd.pose.orientation.w = float(qz), float(qw)
        self._cmd_pub.publish(cmd)
