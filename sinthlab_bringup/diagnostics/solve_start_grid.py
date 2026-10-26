#!/usr/bin/env python3
"""Check the apple-pluck random_start positions before a session: solve them as the run will, and show
how much room each leaves.

`random_start.positions` are [a, b] metres from the start pose (move_to_start) in `random_start.plane`
-- see apple_pluck_impedance.yaml and helpers/start_grid.py. The run solves them itself when it starts;
this prints the same solution with per-position checks: distance from the nearest joint limit, the
smallest singular value of the Jacobian (0 = singular), and the same margin after a further 10 cm pull
toward the monkey. Exit status 1 if any position would stop the run or leaves under 10 deg after the pull.

    export IIWA7_URDF=/tmp/iiwa7.urdf     # xacro .../lbr_description/urdf/iiwa7/iiwa7.xacro > it
    ros2 run sinthlab_bringup solve_start_grid.py --params <the YAML>
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import yaml


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", default=os.path.join(here, "..", "config", "apple_pluck_impedance.yaml"))
    a = ap.parse_args()

    urdf = os.environ.get("IIWA7_URDF", "")
    if not urdf or not os.path.exists(urdf):
        print("Set IIWA7_URDF to the iiwa7 URDF:\n"
              "  xacro $(ros2 pkg prefix lbr_description)/share/lbr_description/urdf/iiwa7/iiwa7.xacro "
              "> /tmp/iiwa7.urdf && export IIWA7_URDF=/tmp/iiwa7.urdf")
        return 2
    import optas
    from sinthlab_bringup.helpers.start_grid import MIN_MARGIN_DEG, StartGrid

    grid = StartGrid(optas.RobotModel(urdf_filename=urdf))
    params = yaml.safe_load(open(a.params))["/**"]["ros__parameters"]
    start = params["move_to_start"]["target_joint_position"]
    plane = params["random_start"].get("plane", "frontal")
    offsets = params["random_start"]["positions"]
    solved = grid.solve(start, plane, offsets)

    # A pull toward the monkey: along the apple's axis, flattened to horizontal.
    T0 = grid._fk(np.radians(start))
    pull = T0[:3, 2] * np.array([1.0, 1.0, 0.0])
    pull /= np.linalg.norm(pull)

    ok = True
    print(f"# {plane} plane around the start {start} (EE {np.round(T0[:3, 3], 3).tolist()} m)")
    print(f"{'pos':>4} {'[a, b] m':>18} {'margin':>7} {'sigma_min':>9} {'after 10 cm pull':>17}")
    for name, (deg, why) in solved.items():
        q = np.radians(deg)
        T = grid._fk(q)
        q_pull, _ = grid.ik(q, T[:3, 3] + 0.10 * pull, T[:3, :3])
        m_pull = grid.margin_deg(q_pull)
        good = why is None and m_pull >= MIN_MARGIN_DEG
        ok &= good
        print(f"{name:>4} {str(offsets[name]):>18} {grid.margin_deg(q):6.1f}d {grid.sigma_min(q):9.3f} "
              f"{m_pull:16.1f}d  {'ok' if good else (why or 'TOO CLOSE after the pull')}")
        print(f"       -> {deg}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
