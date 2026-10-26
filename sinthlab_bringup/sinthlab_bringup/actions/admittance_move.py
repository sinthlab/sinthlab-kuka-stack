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
the collapse a threshold "give-in" causes. Here the equilibrium is
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
  * deadband_n     a THRESHOLD to START: a (tared) force below it moves a resting arm nothing, so
                   force-reading noise cannot creep it. Above it the WHOLE force counts -- it is not
                   subtracted from every push -- with a short ramp (one dead band wide) so the start is
                   not a jolt. Once MOVING, a force between HOLD_FRACTION of it and it still counts, for
                   up to HOLD_SEC at a time: a hand moving the arm at a comfortable speed pushes only
                   ~4-5 N (damping 30), and its own wobble of +-2 N kept dipping under the threshold for
                   a moment and stopping the arm -- stop-go in ~0.3 s bursts (recording 26 Oct, 15:56;
                   most dips under 0.15 s). The time limit keeps a hand left resting on the apple, or
                   reading noise after a let-go, from creeping the arm.

LETTING GO: THE ARM GLIDES TO A STOP AND HOLDS. A hand usually lets go while the arm is still moving
fast. The equilibrium, which runs 1-2 cm ahead of the arm, then brakes at accel_limit_mps2 and stops a
few cm on; the arm coasts past it and the cabinet spring pulls it back -- slowly, against the joints'
friction, so it creeps back over up to a second: the "spring pulling back" instead of a stable hold
(free-move recording: 5-11 mm past the equilibrium after let-gos at 0.2-0.3 m/s). So on release (the
force under the threshold for RELEASE_SEC) the equilibrium's speed is zeroed and, for up to SETTLE_SEC,
it glides onto where the arm IS plus its resting offset (equilibrium - arm at rest, measured at the
tare: the sag under the gravity residual) with time constant GLIDE_TAU_SEC. The spring then neither
pulls back nor drags forward, the arm glides to a stop, and the equilibrium freezes where it stopped
(arm still and the equilibrium on it, or SETTLE_SEC up).
  Not a snap, and not on a flicker: while pushed the equilibrium leads the arm by 1-2 cm, and setting it
onto the arm in one tick moved the spring's pull by that much at once -- a jerk, every time a lifting
force dipped under the threshold for a moment (free-move recording 26 Oct: 5 of 12 pushes under
0.15 s, equilibrium jumps of 7-13 mm). A dip shorter than RELEASE_SEC only slows the equilibrium, under
the acceleration limit, as before.
Orientation is held at the start orientation (rotational stiffness on the SmartPad).

ON A RAIL (rail_axis x | y | z, Move Vertical / Move Horizontal): the same, along one base axis through
the start. Only the hand's force ALONG the rail counts, the equilibrium only moves along it, between
rail_min_m and rail_max_m from the start, and across it stays on the rail -- so the cabinet spring
holds the arm on the rail. rail_axis none = free in every direction (Free Move).

THE FORCE. Computed here from the arm's external joint torques (LBRState.external_torque) through the
base-frame Jacobian of lbr_link_ee:  F = pinv(J)^T tau_ext, with a damped pseudo-inverse
(jacobian_damping) and this action's own ROUND threshold (deadband_n) -- the same in every direction, so
a diagonal push moves along a straight line.

Why not lbr's EstimatedWrenchInterface (`force_torque_broadcaster/wrench`)? It applies its dead band per
axis (2 N on each of X, Y and Z), so a diagonal push, split across axes that switch on and off
independently, snaps between them: bumpy on diagonals. That topic is unchanged for everything else
(e.g. the release check).

The two settings trade accuracy for stability. The external torques also carry the arm's own model
errors (gravity, dynamics, the tool's load data -- determine it whenever the tool changes), and the
elbow amplifies them most: 1 Nm of error on A4 reads as 4.3 N at damping 0.02, 3.4 N at 0.1, 2.0 N at
0.2. At 0.02 that drove a sustained ~1 Hz oscillation about A4, in Z. Damping 0.1 with a 2.5 N round
threshold is smooth and stable on this arm; it reads a push at 78-96 % of its strength, its direction
bent by at most 6 deg -- a constant small bend for a pose, never a snap between axes.
"""
from __future__ import annotations

import time
from typing import Optional

import numpy as np
from geometry_msgs.msg import PoseStamped
from lbr_fri_idl.msg import LBRState
from rclpy.node import Node as rclpyNode

import optas

from sinthlab_bringup.helpers.node_utils import DebugTicker, get_required_param, get_launch_param
from sinthlab_bringup.helpers.floor import Floor
from sinthlab_bringup.helpers.hand_force import HandForce

HOLD_FRACTION = 0.5     # once moving, the force counts down to this fraction of deadband_n (resting noise ~1.25 N p90)
HOLD_SEC = 0.3          # ...for at most this long at a time below deadband_n
RELEASE_SEC = 0.1       # the force must stay under that this long to count as letting go
GLIDE_TAU_SEC = 0.08    # the equilibrium glides onto the arm with this time constant, never in one step
SETTLE_SEC = 0.3        # after letting go: the longest the equilibrium follows the arm while it glides to a stop
SETTLED_MPS = 0.01      # ...or until the arm is slower than this, with the equilibrium on it


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

        robot_description = str(get_launch_param(node, "robot_description", ""))
        model = optas.RobotModel(urdf_string=robot_description)
        self._fk = model.get_link_transform_function(
            link=self.ee_link, base_link=self.base_link, numpy_output=True)
        self._jac_damping = float(get_required_param(node, self._p + "jacobian_damping"))
        self._hand = HandForce(robot_description, self.ee_link, self._jac_damping)

        self._leash = float(get_required_param(node, self._p + "leash_m"))
        axis = str(get_required_param(node, self._p + "rail_axis")).strip().lower()
        if axis not in ("none", "x", "y", "z"):
            raise ValueError(f"{self._p}rail_axis must be none, x, y or z, got {axis!r}")
        self._rail = None if axis == "none" else (
            np.eye(3)["xyz".index(axis)],
            float(get_required_param(node, self._p + "rail_min_m")),
            float(get_required_param(node, self._p + "rail_max_m")))
        self._floor = Floor.from_node(node)     # the equilibrium never goes below it (a soft wall)
        self._tare_max = float(get_required_param(node, self._p + "tare_max_n"))
        self._dbg = DebugTicker(float(get_required_param(node, self._p + "debug_log_rate_hz")))
        self.reload()

        self._active = False
        self._moving = False
        self._anchor: Optional[np.ndarray] = None       # start pose (4x4), captured ONCE per trial
        self._eq: Optional[np.ndarray] = None           # commanded equilibrium position (3,)
        self._p_meas: Optional[np.ndarray] = None
        self._f_filt = np.zeros(3)
        self._f_used = np.zeros(3)
        self._v = np.zeros(3)                           # equilibrium velocity (accel-limited)
        self._tare = np.zeros(3)
        self._tare_samples = []
        self._rest_offset = np.zeros(3)                 # equilibrium - arm at rest (gravity sag), from the tare
        self._pushing = False
        self._quiet = 0.0                               # how long the force has been under the threshold
        self._low_t = 0.0                               # how long a moving hand has pushed under deadband_n
        self._settle_until = 0.0                        # gliding to a stop after a let-go until this time
        self._p_prev: Optional[np.ndarray] = None
        self._t_prev: Optional[float] = None

        node.create_subscription(LBRState, state_topic, self._on_state, 1)
        self._pub = node.create_publisher(PoseStamped, cmd_topic, 1)
        log.info(f"Admittance free move: {'no boundary' if self._rail is None else f'on a {axis} rail, {self._rail[1]:+.2f} to {self._rail[2]:+.2f} m'}, leash {self._leash} m, force from the joint torques "
                 f"(Jacobian damping {self._jac_damping:g})")

    # ------------------------------------------------------------------ parameters
    def reload(self) -> None:
        """Re-read the live feel parameters. Called at a trial boundary."""
        n, p = self._node, self._p
        self._damping = max(1.0, float(get_required_param(n, p + "damping_ns_per_m")))
        self._deadband = max(0.0, float(get_required_param(n, p + "deadband_n")))
        self._max_speed = max(0.0, float(get_required_param(n, p + "max_speed_mps")))
        self._accel = max(0.0, float(get_required_param(n, p + "accel_limit_mps2")))
        self._tau = max(0.0, float(get_required_param(n, p + "force_filter_tau_sec")))
        self._tare_sec = max(0.0, float(get_required_param(n, p + "tare_sec")))
        self._debug = bool(get_required_param(n, p + "debug_log_enabled"))
        n.get_logger().info(
            f"Free move feel: damping {self._damping:g} N s/m, threshold {self._deadband:g} N, "
            f"max speed {self._max_speed:g} m/s, accel limit "
            f"{f'{self._accel:g} m/s^2' if self._accel > 0 else 'off'}, "
            f"debug log {'ON' if self._debug else 'off'}")

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> None:
        """Hold the start pose and measure the resting force (tare). Motion starts at enable_motion()."""
        self._active, self._moving = True, False
        self._anchor = self._eq = None
        self._tare_samples = []
        self._f_filt[:] = 0.0
        self._v[:] = 0.0
        self._t_prev = None
        self._p_prev = None
        self._pushing = False
        self._quiet = 0.0
        self._low_t = 0.0
        self._settle_until = 0.0
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
        if self._eq is not None and self._p_meas is not None:
            off = self._eq - self._p_meas                 # the resting sag under the gravity residual
            n = float(np.linalg.norm(off))
            self._rest_offset = off if n <= self._leash else off * (self._leash / n)
        self._node.get_logger().info(f"Free move: tare {np.round(self._tare, 2)} N (base frame), resting offset "
                                     f"{np.round(self._rest_offset * 1000, 1)} mm; moving.")
        self._pushing = False
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
    def _hand_force(self, msg: LBRState, q: np.ndarray, T: np.ndarray) -> np.ndarray:
        """The force the hand applies to the tool, base frame [N] (helpers/hand_force.py)."""
        self._tau_ext = np.array(msg.external_torque, dtype=float)     # for the debug line
        return self._hand.force(self._tau_ext, q)

    def _threshold(self, f: np.ndarray) -> np.ndarray:
        """At rest: zero below the dead band, the FULL force from twice the dead band, a linear ramp
        between. Moving: the full force, and down to HOLD_FRACTION of the dead band for up to HOLD_SEC
        at a time (class doc)."""
        mag = float(np.linalg.norm(f))
        if self._deadband <= 0.0:
            return f
        if self._pushing and (mag >= self._deadband or
                              (mag >= HOLD_FRACTION * self._deadband and self._low_t < HOLD_SEC)):
            return f
        if mag <= self._deadband:
            return np.zeros(3)
        return f * min(1.0, (mag - self._deadband) / self._deadband)

    def _on_state(self, msg: LBRState) -> None:
        q = np.array(msg.measured_joint_position, dtype=float)
        if not np.all(np.isfinite(q)):
            return
        T = self._fk(q)
        self._q_last = q
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
        f_base = self._hand_force(msg, q, T)

        if not self._moving:
            if now - self._t_start <= max(self._tare_sec, 0.1) or not self._tare_samples:
                self._tare_samples.append(f_base)
            self._publish(self._eq)
            return

        # Force -> velocity.
        f = f_base - self._tare
        if self._rail is not None:                        # on a rail only the force along it counts
            f = self._rail[0] * float(f @ self._rail[0])
        k = 1.0 if self._tau <= 0.0 or dt <= 0.0 else dt / (self._tau + dt)
        self._f_filt += k * (f - self._f_filt)
        low = self._pushing and float(np.linalg.norm(self._f_filt)) < self._deadband
        self._low_t = self._low_t + dt if low else 0.0      # how long a moving hand has been under it
        self._f_used = self._threshold(self._f_filt)

        # Let go: glide to a stop, then hold (class doc, "letting go").
        arm_speed = (float(np.linalg.norm(self._p_meas - self._p_prev)) / dt
                     if self._p_prev is not None and dt > 0.0 else 0.0)
        self._p_prev = self._p_meas.copy()
        # Released = the force under the threshold for RELEASE_SEC. (Half the threshold was tried: after a
        # let-go the reading hovers at 1-2.5 N, so it fired late on half the let-gos, after the run-on.)
        self._quiet = self._quiet + dt if self._pushing and not np.any(self._f_used) else 0.0
        if self._pushing and self._quiet >= RELEASE_SEC:
            self._pushing = False
            self._v[:] = 0.0
            self._settle_until = now + SETTLE_SEC
        if not self._pushing:
            if not np.any(self._f_used):
                if now < self._settle_until:
                    target = self._floor.clamp_xyz(self._on_rail(self._p_meas + self._rest_offset),
                                                   self._anchor[0:3, 0:3])
                    self._eq = self._eq + min(1.0, dt / GLIDE_TAU_SEC) * (target - self._eq)
                    if arm_speed < SETTLED_MPS and float(np.linalg.norm(target - self._eq)) < 1e-3:
                        self._settle_until = 0.0
                self._publish(self._eq)
                return
            self._pushing = True
            self._settle_until = 0.0

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
        # The rail, then the floor: clamp the integrated equilibrium itself, so a push past a rail end or
        # down at the floor winds nothing up and the arm comes away from it at once.
        eq = self._floor.clamp_xyz(self._on_rail(eq), self._anchor[0:3, 0:3])
        # Remember the speed actually applied (after the leash), so a held-back equilibrium does not
        # build up speed it would release as a lurch.
        self._v = (eq - self._eq) / dt if dt > 0.0 else v
        self._eq = eq
        self._publish(eq)

        if self._debug and self._dbg.tick(max(dt, 1e-3)):
            self._node.get_logger().info(
                f"free move: F={np.round(self._f_used, 1)} N  v={np.round(v, 3)} m/s  "
                + f"tau_ext={np.round(getattr(self, '_tau_ext', np.zeros(7)), 2)} Nm  "
                + f"q4={np.degrees(self._q_last[3]):.1f} deg  "
                f"eq-start={np.round(eq - self._anchor[0:3, 3], 3)} m")

    def _on_rail(self, xyz: np.ndarray) -> np.ndarray:
        """xyz moved onto the rail -- the nearest point of the line through the start, within its ends."""
        if self._rail is None:
            return xyz
        u, lo, hi = self._rail
        a = self._anchor[0:3, 3]
        return a + u * float(np.clip((np.asarray(xyz, dtype=float) - a) @ u, lo, hi))

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
