#!/usr/bin/env python3
"""Resistance of the pull -- felt stiffness below the SmartPad profile, chosen per trial -- and the
release at the threshold, when the apple is "plucked".

The cabinet's stiffness is fixed when the Sunrise app starts (SmartPad profile); FRI streams only the
spring's EQUILIBRIUM, never its stiffness. So ROS softens the spring the one way it can: during the
pull the equilibrium follows a fraction f of the arm's movement, and the spring is stretched by only
(1 - f) of it:

    q_cmd = q_hold + f * (q_measured - q_rest)        felt stiffness = base * (1 - f)
    f     = 1 - stiffness_n_per_m / base

  base    the stiffness the SmartPad app REPORTS it is running (helpers/smartpad_link.py), re-read at
          every arm; resistance.base_stiffness_n_per_m only stands in until a report has arrived

  q_rest  the arm where it rests when the monitor arms (its settled pose, sag included), so the
          resting sag is unchanged and only the PULL is softened
  q_hold  the equilibrium being commanded at that moment (the start, or the perturbed pose)

It works in joint space on the joint-position controller apple pluck and perturb already use -- no IK.
For pulls of a few cm, f * (q - q_rest) moves the end effector by ~f times the pull, so the felt
stiffness is ~base * (1 - f) in every direction. Orientation and the elbow soften by the same fraction.
During the pull the felt stiffness never goes below MIN_FELT_STIFFNESS (50 N/m); with f = 0
(stiffness == base) nothing is published and the arm behaves exactly as with the bare profile.

RELEASE -- THE APPLE IS PLUCKED (release(), at the threshold). f ramps to 1 over RELEASE_RAMP_SEC: the
equilibrium follows the arm completely, plus the resting offset (q_hold - q_rest), so the hand feels
no pull-back at all and the arm still carries its own weight. Once the arm has been still for
STILL_SEC the equilibrium holds there (settled()), so it cannot creep under the gravity residual.

THREE WALLS. The equilibrium never follows the arm
  * below the floor (helpers/floor.py), nor
  * toward a straight elbow: |A4| under `elbow_min_deg`, or past 0 to the other side, nor
  * within JOINT_MARGIN_DEG of any joint's limit. A hard pull after the snap dragged the elbow (A4)
    to -109 deg at ~45 deg/s with the equilibrium following, into the cabinet's axis-range monitor
    (+-112 deg on A2/A4/A6, README section 2), which stopped the arm (recording 26 Oct, 16:52).
A command that would is replaced by the last one that did not, so from there the full base spring
holds. Why the elbow: a pull toward the monkey STRAIGHTENS it, and a straight elbow can fold back
either way. Folded the wrong way (A4 > 0) it stays there -- the cabinet holds only the apple's pose --
and every later move swings and jerks. At Uniform 100 the base spring is no wall (1 N per cm); at
Uniform 400 with a felt stiffness of 100 the pull feels the same and the walls hold 4 N per cm.

Streams from start() (the monitor arms) to stop() (the move to park).
"""
from __future__ import annotations

from typing import Optional

import numpy as np
import optas
from lbr_fri_idl.msg import LBRJointPositionCommand, LBRState
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.helpers.floor import Floor
from sinthlab_bringup.helpers.node_utils import get_launch_param, get_required_param
from sinthlab_bringup.helpers.live_params import IIWA7_LIMITS_DEG, MIN_FELT_STIFFNESS
from sinthlab_bringup.helpers.smartpad_link import SmartPadLink

RELEASE_RAMP_SEC = 0.3     # the pull-back fades to nothing over this, so the release is felt, not a jolt
STILL_SEC = 0.3            # released arm this long below STILL_RAD_S -> settled: the equilibrium holds there
STILL_RAD_S = 0.05         # [rad/s] fastest joint (~3 deg/s)
A4 = 3                     # the elbow
JOINT_MARGIN_DEG = 14.0    # never follow a joint closer than this to its limit: A2/A4/A6 stop at +-105 deg
_JOINT_WALL = np.radians(np.asarray(IIWA7_LIMITS_DEG) - JOINT_MARGIN_DEG)


class ResistanceSoftener:
    def __init__(self, node: rclpyNode, *, param_prefix: str = "resistance", update_rate: int = 100) -> None:
        self._node = node
        self._p = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix
        self._dt = 1.0 / float(update_rate)
        robot_name = node.get_namespace().strip("/")
        ns = f"/{robot_name}" if robot_name else ""
        self._pub = node.create_publisher(LBRJointPositionCommand, f"{ns}/command/lbr_joint_position_command", 1)
        node.create_subscription(LBRState, "lbr_state", self._on_state, 1)
        self._measured: Optional[np.ndarray] = None
        self._prev: Optional[np.ndarray] = None
        self._commanded: Optional[np.ndarray] = None
        self._active = False
        self._q_rest: Optional[np.ndarray] = None
        self._q_hold: Optional[np.ndarray] = None
        self._q_last_ok: Optional[np.ndarray] = None    # the last command inside both walls
        self._released = False
        self._release_t = 0.0
        self._still_t = 0.0
        self._settled = False
        self._floor = Floor.from_node(node)
        self._elbow_min = np.radians(float(get_required_param(node, self._p + "elbow_min_deg")))
        self._fk = optas.RobotModel(urdf_string=str(get_launch_param(node, "robot_description", ""))
                                    ).get_global_link_transform_function("lbr_link_ee", numpy_output=True)
        self.reload()
        node.create_timer(self._dt, self._step)

    def reload(self) -> None:
        """Re-read the felt stiffness. Called at a trial boundary (live parameter)."""
        self._k = float(get_required_param(self._node, self._p + "stiffness_n_per_m"))
        self._base = None
        self._refresh_base()

    def _refresh_base(self) -> None:
        """The base is what the SmartPad app reports it runs; the YAML value only until it reports."""
        n = self._node
        reported = SmartPadLink.get(n.get_logger()).translational_stiffness()
        base = reported if reported is not None else float(get_required_param(n, self._p + "base_stiffness_n_per_m"))
        if base == self._base:
            return
        self._base = base
        k = min(max(self._k, MIN_FELT_STIFFNESS), base)     # 50 N/m .. the base
        self._follow = 1.0 - k / base
        n.get_logger().info(
            f"Resistance: {self.felt_stiffness():g} N/m felt (base {base:g} N/m "
            f"{'reported by the SmartPad app' if reported is not None else 'from the YAML: no SmartPad report yet'}, "
            f"equilibrium follows {100 * self._follow:.0f} % of the pull); elbow wall "
            f"{np.degrees(self._elbow_min):g} deg from straight")

    def felt_stiffness(self) -> float:
        return self._base * (1.0 - self._follow)

    def follow_fraction(self) -> float:
        return self._follow

    def describe(self) -> dict:
        """For the trial sidecar: what this trial actually ran with."""
        return {"base_stiffness_n_per_m": self._base, "felt_stiffness_n_per_m": self.felt_stiffness(),
                "follow_fraction": self._follow, "elbow_min_deg": round(float(np.degrees(self._elbow_min)), 3)}

    def start(self) -> None:
        """The monitor has armed: from here the pull is softened."""
        self._refresh_base()
        self._released = self._settled = False
        if self._measured is None:
            return
        self._q_rest = self._measured.copy()
        self._q_hold = self._commanded.copy() if self._commanded is not None else self._measured.copy()
        self._q_last_ok = self._q_hold.copy()
        self._active = self._follow > 0.0

    def release(self) -> None:
        """The threshold: from here the equilibrium follows the arm completely (see the module doc)."""
        if self._q_rest is None:
            return
        self._released, self._release_t = True, 0.0
        self._settled, self._still_t = False, 0.0
        self._active = True

    def settled(self) -> bool:
        """Released, and the arm has come to rest: the equilibrium holds where it stopped."""
        return self._settled

    def stop(self) -> None:
        self._active = self._released = self._settled = False

    def elbow_deg(self) -> Optional[float]:
        """The measured elbow (A4) [deg]."""
        return None if self._measured is None else float(np.degrees(self._measured[A4]))

    def elbow_bent(self) -> bool:
        """The measured elbow bends the commanded way and is at least near the wall -- safe to park."""
        if self._measured is None or self._commanded is None:
            return True
        q, ref = self._measured[A4], self._commanded[A4]
        return np.sign(q) == np.sign(ref) and abs(q) >= self._elbow_min - np.radians(5.0)

    def elbow_flipped(self) -> bool:
        """The measured elbow bends the opposite way from the commanded one (see the module doc)."""
        if self._measured is None or self._commanded is None:
            return False
        q, ref = self._measured[A4], self._commanded[A4]
        return abs(q) > np.radians(5.0) and np.sign(q) != np.sign(ref)

    def _on_state(self, msg: LBRState) -> None:
        qm = np.array(msg.measured_joint_position, dtype=float)
        qc = np.array(msg.commanded_joint_position, dtype=float)
        if np.all(np.isfinite(qm)):
            self._prev, self._measured = self._measured, qm
        if np.all(np.isfinite(qc)) and np.any(qc != 0.0):
            self._commanded = qc

    def command_for(self, q_measured: np.ndarray) -> np.ndarray:
        f = self._follow
        if self._released:      # ramp to following completely
            f += (1.0 - f) * min(1.0, self._release_t / RELEASE_RAMP_SEC)
        return self._q_hold + f * (q_measured - self._q_rest)

    def _inside_walls(self, q: np.ndarray) -> bool:
        a4, ref = q[A4], self._q_hold[A4]
        if np.sign(a4) != np.sign(ref) or abs(a4) < self._elbow_min:
            return False
        if np.any(np.abs(q) > _JOINT_WALL):
            return False
        return self._floor.clearance(self._fk(q)) >= 0.0

    def _step(self) -> None:
        if not self._active or self._measured is None:
            return
        if self._released:
            self._release_t += self._dt
            speed = (float(np.max(np.abs(self._measured - self._prev))) / self._dt
                     if self._prev is not None else 0.0)
            self._still_t = 0.0 if speed > STILL_RAD_S else self._still_t + self._dt
            if self._release_t >= RELEASE_RAMP_SEC and self._still_t >= STILL_SEC:
                self._settled = True
            elif self._settled and speed > 2 * STILL_RAD_S:
                self._settled = False   # the hand moves it again before the park: follow again
        if self._settled:
            q = self._q_last_ok         # at rest: hold here, so the arm cannot creep
        else:
            q = self.command_for(self._measured)
            # Outside a wall (the floor, or the elbow toward straight): follow no further.
            if self._inside_walls(q) or not self._inside_walls(self._q_last_ok):
                self._q_last_ok = q
            else:
                q = self._q_last_ok
        cmd = LBRJointPositionCommand()
        cmd.joint_position = [float(v) for v in q]
        self._pub.publish(cmd)
