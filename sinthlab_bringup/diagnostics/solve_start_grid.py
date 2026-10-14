#!/usr/bin/env python3
"""Solve the 9 apple-pluck start positions of the random_start paradigm, and check them.

The normal start pose (apple_pluck_impedance.yaml move_to_start) is the origin C. The other 8 lie on a
circle of radius `--spacing` around it, in the plane FACING the monkey (base Y-Z). Compass as the monkey sees it -- it faces the apple from +X:

    NW  N  NE        N = up (+Z)          E = the monkey's right (+Y)
     W  C  E         S = down (-Z)        W = the monkey's left (-Y)
    SW  S  SE        diagonals at 45 deg, also `spacing` from C

Each position is solved by full 6-D IK (position AND orientation), so the apple keeps pointing at the
monkey with the same tilt: it is translated, never turned. The redundancy is pulled toward the start
posture. Prints the YAML block to paste under `random_start.positions`, with per-position checks:
distance from the nearest joint limit, the smallest singular value of the Jacobian (0 = singular),
and the same margin after a further 10 cm pull toward the monkey.

    export IIWA7_URDF=/tmp/iiwa7.urdf     # xacro .../lbr_description/urdf/iiwa7/iiwa7.xacro > it
    ros2 run sinthlab_bringup solve_start_grid.py --spacing 0.05
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

# (name, angle in the facing plane from E toward N [deg]); C is the origin itself.
COMPASS = [("C", None), ("N", 90), ("NE", 45), ("E", 0), ("SE", -45),
           ("S", -90), ("SW", -135), ("W", 180), ("NW", 135)]
MIN_MARGIN_DEG, MIN_SINGULAR = 10.0, 0.10


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--spacing", type=float, required=True, help="distance of each position from C [m]")
    ap.add_argument("--params", default=os.path.join(here, "..", "config", "apple_pluck_impedance.yaml"))
    a = ap.parse_args()

    urdf = os.environ.get("IIWA7_URDF", "")
    if not urdf or not os.path.exists(urdf):
        print("Set IIWA7_URDF to the iiwa7 URDF:\n"
              "  xacro $(ros2 pkg prefix lbr_description)/share/lbr_description/urdf/iiwa7/iiwa7.xacro "
              "> /tmp/iiwa7.urdf && export IIWA7_URDF=/tmp/iiwa7.urdf")
        return 2
    import optas

    robot = optas.RobotModel(urdf_filename=urdf)
    fk = robot.get_global_link_transform_function("lbr_link_ee", numpy_output=True)
    lo = robot.lower_actuated_joint_limits.toarray().ravel()
    hi = robot.upper_actuated_joint_limits.toarray().ravel()
    params = yaml.safe_load(open(a.params))["/**"]["ros__parameters"]
    q_origin = np.radians(np.array(params["move_to_start"]["target_joint_position"], dtype=float))
    T_origin = fk(q_origin)

    def jacobian(q, eps=1e-6):
        T, J = fk(q), np.zeros((6, 7))
        for i in range(7):
            d = q.copy()
            d[i] += eps
            Td = fk(d)
            J[:3, i] = (Td[:3, 3] - T[:3, 3]) / eps
            J[3:, i] = Rotation.from_matrix(Td[:3, :3] @ T[:3, :3].T).as_rotvec() / eps
        return J

    def ik(pos, seed):
        """Damped least squares on position + orientation; null space pulled toward the start posture."""
        q = seed.copy()
        for _ in range(500):
            T = fk(q)
            err = np.r_[pos - T[:3, 3], Rotation.from_matrix(T_origin[:3, :3] @ T[:3, :3].T).as_rotvec()]
            if np.linalg.norm(err) < 1e-9:
                break
            J = jacobian(q)
            Jp = J.T @ np.linalg.inv(J @ J.T + 1e-6 * np.eye(6))
            q = q + Jp @ err + 0.2 * (np.eye(7) - Jp @ J) @ (q_origin - q)
        return q, float(np.linalg.norm(err))

    def margin(q):
        return float(np.degrees(np.min(np.minimum(q - lo, hi - q))))

    # A pull toward the monkey: along the apple's axis, flattened to horizontal.
    pull = T_origin[:3, 2] * np.array([1.0, 0.0, 1.0])
    pull[2] = 0.0
    pull /= np.linalg.norm(pull)
    origin = T_origin[:3, 3]

    ok, rows = True, []
    for name, ang in COMPASS:
        off = np.zeros(3) if ang is None else a.spacing * np.array(
            [0.0, np.cos(np.radians(ang)), np.sin(np.radians(ang))])
        q, err = ik(origin + off, q_origin)
        q_pull, _ = ik(origin + off + 0.10 * pull, q)
        m, m_pull = margin(q), margin(q_pull)
        sv = float(np.linalg.svd(jacobian(q), compute_uv=False)[-1])
        good = err < 1e-5 and min(m, m_pull) >= MIN_MARGIN_DEG and sv >= MIN_SINGULAR
        ok &= good
        rows.append((name, np.round(np.degrees(q), 1), m, sv, m_pull, good))

    print(f"# spacing {a.spacing} m; origin C (move_to_start) at EE {np.round(origin, 3).tolist()} m")
    print(f"{'pos':>4} {'margin':>7} {'sigma_min':>9} {'margin after 10 cm pull':>24}")
    for name, _, m, sv, m_pull, good in rows:
        print(f"{name:>4} {m:6.1f}d {sv:9.3f} {m_pull:23.1f}d  {'ok' if good else 'TOO CLOSE'}")
    print("\n      positions:  # A1..A7 [deg] each; C = move_to_start.target_joint_position")
    for name, q, *_ in rows:
        print(f"        {name}: {[float(v) + 0.0 for v in q]}")
    if not ok:
        print(f"\nAt least one position is within {MIN_MARGIN_DEG:.0f} deg of a joint limit or near-singular "
              f"(sigma_min < {MIN_SINGULAR}): use a smaller spacing.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
