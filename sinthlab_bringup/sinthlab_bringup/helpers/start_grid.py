"""The random_start paradigm's start positions: offsets in a plane through the start, solved to joints.

Each position is `[a, b]` metres from the start pose's end effector, in one plane of the base frame
(the same planes as the perturbation, perturb_start.polar_plane):

    frontal     base Y-Z, the plane FACING the monkey   a = +Y (the monkey's right), b = +Z (up)
    horizontal  base X-Y                                a = +X (toward the monkey),  b = +Y
    sagittal    base X-Z                                a = +X (toward the monkey),  b = +Z

Each is solved by 6-D IK (position AND orientation) seeded at the start posture, so the apple is only
translated -- it keeps pointing at the monkey with the same tilt -- and the redundancy is pulled toward
the start posture. A position is refused if the IK does not reach it, it lands within MIN_MARGIN_DEG
of a joint limit, the arm is nearly singular there, or the elbow (A4) bends the other way from the
start (every move between them would pass a straight elbow).

Used by the apple-pluck orchestrator (at start-up, and again if the start pose changes), and by
diagnostics/solve_start_grid.py and check_park.py.
"""
from __future__ import annotations

import itertools
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

PLANES = {"frontal": (1, 2), "horizontal": (0, 1), "sagittal": (0, 2)}   # base axes of (a, b)
MIN_MARGIN_DEG, MIN_SINGULAR = 10.0, 0.10
# Free Move's box (box_problem): every corner, edge middle and face centre must keep the elbow at least
# BOX_MIN_ELBOW_DEG bent -- the jerk of 23 Nov began at A4 ~23 deg, on the way to a straight elbow -- with
# the same joint margin, and stay this far from singular (a box reaches nearer the edge of the
# workspace than a few random-start positions: the default box's tightest corner is at 0.088).
BOX_MIN_ELBOW_DEG, BOX_MIN_SINGULAR = 30.0, 0.08


class StartGrid:
    def __init__(self, robot, ee_link: str = "lbr_link_ee") -> None:
        """robot: an optas.RobotModel of the iiwa7."""
        self._fk = robot.get_global_link_transform_function(ee_link, numpy_output=True)
        self._jac = robot.get_global_link_geometric_jacobian_function(link=ee_link, numpy_output=True)
        self._lo = robot.lower_actuated_joint_limits.toarray().ravel()
        self._hi = robot.upper_actuated_joint_limits.toarray().ravel()

    def offset_xyz(self, plane: str, ab) -> np.ndarray:
        if plane not in PLANES:
            raise ValueError(f"random_start.plane must be one of {list(PLANES)}, got {plane!r}")
        a, b = (float(v) for v in ab)
        off = np.zeros(3)
        ia, ib = PLANES[plane]
        off[ia], off[ib] = a, b
        return off

    def ik(self, q_seed: np.ndarray, pos: np.ndarray, R_target: np.ndarray) -> Tuple[np.ndarray, float]:
        """Damped least squares on position + orientation, null space pulled toward the seed."""
        q = np.array(q_seed, dtype=float)
        err = np.full(6, np.inf)
        for _ in range(200):
            T = self._fk(q)
            err = np.r_[pos - T[:3, 3], Rotation.from_matrix(R_target @ T[:3, :3].T).as_rotvec()]
            if np.linalg.norm(err) < 1e-9:
                break
            J = np.asarray(self._jac(q), dtype=float)
            Jp = J.T @ np.linalg.inv(J @ J.T + 1e-6 * np.eye(6))
            q = q + Jp @ err + 0.2 * (np.eye(7) - Jp @ J) @ (q_seed - q)
        return q, float(np.linalg.norm(err))

    def margin_deg(self, q) -> float:
        return float(np.degrees(np.min(np.minimum(q - self._lo, self._hi - q))))

    def sigma_min(self, q) -> float:
        return float(np.linalg.svd(np.asarray(self._jac(q), dtype=float), compute_uv=False)[-1])

    def box_problem(self, start_deg, box_min, box_max) -> Optional[str]:
        """None if the arm can hold the start's orientation everywhere in the box (offsets from the start's
        end effector, base frame [x, y, z]) with the elbow bent, away from the joint limits and singular
        postures -- checked at its corners, edge middles and face centres -- else the reason."""
        q0 = np.radians(np.asarray(start_deg, dtype=float))
        T0 = self._fk(q0)
        lo, hi = np.asarray(box_min, dtype=float), np.asarray(box_max, dtype=float)
        levels = [sorted({float(l), 0.0, float(h)}) for l, h in zip(lo, hi)]
        for off in (np.array(p) for p in itertools.product(*levels)):
            q, err = self.ik(q0, T0[:3, 3] + off, T0[:3, :3])
            where = f"the box point {np.round(off, 3).tolist()} m from the start"
            if err > 1e-4:
                return f"{where} is out of reach"
            if (q[3] > 0) != (q0[3] > 0) or abs(np.degrees(q[3])) < BOX_MIN_ELBOW_DEG:
                return (f"{where} straightens the elbow to A4 {np.degrees(q[3]):.0f} deg "
                        f"(needs {BOX_MIN_ELBOW_DEG:g} deg or more)")
            if self.margin_deg(q) < MIN_MARGIN_DEG:
                return f"{where} is {self.margin_deg(q):.1f} deg from a joint limit (needs {MIN_MARGIN_DEG:g})"
            if self.sigma_min(q) < BOX_MIN_SINGULAR:
                return f"{where} is nearly singular (sigma_min {self.sigma_min(q):.3f} < {BOX_MIN_SINGULAR})"
        return None

    def solve(self, start_deg, plane: str, offsets: Dict[str, List[float]]
              ) -> Dict[str, Tuple[List[float], Optional[str]]]:
        """name -> (joint angles A1..A7 [deg], None if usable else the reason)."""
        q0 = np.radians(np.asarray(start_deg, dtype=float))
        T0 = self._fk(q0)
        out = {}
        for name, ab in offsets.items():
            off = self.offset_xyz(plane, ab)
            q, err = self.ik(q0, T0[:3, 3] + off, T0[:3, :3])
            problem = None
            if err > 1e-4:
                problem = "out of reach" + (f" ({err * 1000:.1f} mm short)" if err < 0.05 else "")
            elif (q[3] > 0) != (q0[3] > 0):
                problem = "the elbow (A4) would bend the other way from the start"
            elif self.margin_deg(q) < MIN_MARGIN_DEG:
                problem = f"{self.margin_deg(q):.1f} deg from a joint limit (needs {MIN_MARGIN_DEG:g})"
            elif self.sigma_min(q) < MIN_SINGULAR:
                problem = f"nearly singular (sigma_min {self.sigma_min(q):.3f} < {MIN_SINGULAR})"
            out[name] = ([round(float(v), 2) + 0.0 for v in np.degrees(q)], problem)
        return out
