#!/usr/bin/env python3
"""Check an experiment's park pose (move_to_park) against its start, before running it.

The arm moves park <-> start in joint space every trial. This checks, for the YAML given:

  * the elbow (A4) bends the same way at park and start -- otherwise every such move passes A4 = 0,
    a straight, singular elbow;
  * along the whole joint path park <-> start (and park <-> each random_start position, for apple
    pluck), the arm stays well conditioned (smallest Jacobian singular value >= 0.10) and >= 10 deg
    from every joint limit;
  * where the apple ends up at park, against where it starts.

    export IIWA7_URDF=/tmp/iiwa7.urdf     # xacro .../lbr_description/urdf/iiwa7/iiwa7.xacro > it
    ros2 run sinthlab_bringup check_park.py --params config/maze_params.yaml
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import yaml

MIN_SIGMA, MIN_MARGIN_DEG = 0.10, 10.0
APPLE_M = 0.212          # apple centre along the tool axis from the flange [m]


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

    robot = optas.RobotModel(urdf_filename=urdf)
    fk = robot.get_global_link_transform_function("lbr_link_ee", numpy_output=True)
    jac = robot.get_global_link_geometric_jacobian_function(link="lbr_link_ee", numpy_output=True)
    lo = robot.lower_actuated_joint_limits.toarray().ravel()
    hi = robot.upper_actuated_joint_limits.toarray().ravel()

    def sigma(q):
        return float(np.linalg.svd(np.array(jac(q)), compute_uv=False)[-1])

    def margin(q):
        return float(np.degrees(np.min(np.minimum(q - lo, hi - q))))

    def apple(q):
        T = fk(q)
        return T[:3, 3] + APPLE_M * T[:3, 2]

    p = yaml.safe_load(open(a.params))["/**"]["ros__parameters"]
    park = np.radians(np.array(p["move_to_park"]["target_joint_position"], dtype=float))
    starts = {"start": p["move_to_start"]["target_joint_position"]}
    starts.update({f"random_start {k}": v for k, v in p.get("random_start", {}).get("positions", {}).items()})

    ok = True
    print(f"park  {np.degrees(park).round(1).tolist()}: apple at {apple(park).round(3).tolist()} m, "
          f"sigma_min {sigma(park):.3f}, {margin(park):.1f} deg from the nearest limit")
    for name, deg in starts.items():
        q = np.radians(np.array(deg, dtype=float))
        same = (q[3] > 0) == (park[3] > 0)
        path = [park + t * (q - park) for t in np.linspace(0, 1, 101)]
        s, m = min(sigma(x) for x in path), min(margin(x) for x in path)
        good = same and s >= MIN_SIGMA and m >= MIN_MARGIN_DEG
        ok &= good
        why = "" if good else ("  <- elbow bends the other way" if not same else "  <- too close to singular / a limit")
        print(f"  <-> {name:16s} apple at {apple(q).round(3).tolist()} m; path sigma_min {s:.3f}, "
              f"margin {m:5.1f} deg  {'ok' if good else 'BAD'}{why}")
    if not ok:
        print(f"\nAt least one park <-> start path fails (needs sigma_min >= {MIN_SIGMA}, >= {MIN_MARGIN_DEG:.0f} deg "
              f"from every limit, and A4 on the same side).")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
