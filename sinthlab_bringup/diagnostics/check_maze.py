#!/usr/bin/env python3
"""Validate a maze definition BEFORE running it on hardware.

The maze is pure configuration -- you change it by editing `maze_params.yaml`, never by editing code.
But a maze can be wrong in ways that only show up as odd behaviour on the arm (a rail the robot cannot
physically reach, a checkpoint floating off the rails, a section walled off from the start). This
script checks all of that offline.

    ros2 run sinthlab_bringup check_maze.py
    ros2 run sinthlab_bringup check_maze.py --profile free_plane

Structural checks always run. Kinematic checks (reachability + how hard each point is to move
sideways at) additionally need `optas` and the iiwa7 URDF; they are skipped with a note if missing.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import yaml

OK, BAD = "  [ok] ", "  [--] "
EPS = 1e-6


class _DupKeyLoader(yaml.SafeLoader):
    """SafeLoader that records keys defined twice in the SAME mapping.

    YAML silently keeps the LAST of a duplicated key, so a stale leftover entry parses cleanly and
    every other check still passes -- while the file says one thing and ROS loads another. This has
    already bitten once here: an old 3-element `checkpoint_x` sat above a new 4-element one, and
    nothing complained. Detect it explicitly.
    """

    duplicates: list = []


def _dup_aware_mapping(loader, node, deep=False):
    seen = set()
    for key_node, _ in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in seen:
            _DupKeyLoader.duplicates.append((key, key_node.start_mark.line + 1))
        seen.add(key)
    return yaml.SafeLoader.construct_mapping(loader, node, deep)


_DupKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _dup_aware_mapping
)


def load_checked(path):
    """Parse `path`, returning (document, [(key, line), ...]) for keys duplicated in one mapping."""
    _DupKeyLoader.duplicates = []
    with open(path) as f:
        doc = yaml.load(f, _DupKeyLoader)
    return doc, list(_DupKeyLoader.duplicates)


def load(params_path, profile):
    with open(params_path) as f:
        doc = yaml.safe_load(f)
    p = doc["/**"]["ros__parameters"]
    vf = p["virtual_fixtures"]
    if profile not in vf:
        sys.exit(f"profile {profile!r} not found; available: "
                 f"{[k for k, v in vf.items() if isinstance(v, dict)]}")
    return p, vf[profile]


def rails_of(cfg):
    keys = ("corridor_a_min", "corridor_a_max", "corridor_b_min", "corridor_b_max")
    arrs = [cfg.get(k, []) for k in keys]
    if not arrs[0] or len({len(a) for a in arrs}) != 1:
        sys.exit(f"corridor_* arrays must be non-empty and the same length, got "
                 f"{[len(a) for a in arrs]}")
    return [tuple(float(a[i]) for a in arrs) for i in range(len(arrs[0]))]


def seg_dist(r, s):
    """Distance between two axis-aligned rectangles/segments (0 if they touch or overlap)."""
    da = max(0.0, r[0] - s[1], s[0] - r[1])
    db = max(0.0, r[2] - s[3], s[2] - r[3])
    return math.hypot(da, db)


def point_dist(p, r):
    ca = min(max(p[0], r[0]), r[1])
    cb = min(max(p[1], r[2]), r[3])
    return math.hypot(p[0] - ca, p[1] - cb)


def main() -> int:
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--params", default=os.path.join(here, "..", "config", "maze_params.yaml"))
    ap.add_argument("--profile", default=None, help="default: whatever virtual_fixture_profile selects")
    ap.add_argument("--tol", type=float, default=0.005, help="on-rail tolerance [m] for waypoints")
    a = ap.parse_args()

    params_path = os.path.abspath(a.params)
    doc, dups = load_checked(params_path)
    top = doc["/**"]["ros__parameters"]
    profile = a.profile or top["virtual_fixture_profile"]
    p, cfg = load(params_path, profile)
    rails = rails_of(cfg)
    print(f"maze check: {params_path}\n            profile {profile!r}, {len(rails)} rails\n")
    fails = 0

    print("structure")
    # ---- 0. duplicate keys -- checked FIRST, because a duplicate silently changes what loads ---
    if dups:
        fails += 1
        for key, line in dups:
            print(BAD + f"key {key!r} is defined TWICE in the same block (second at line {line}). "
                        f"YAML keeps the LAST one, so the file does not say what ROS loads.")
    else:
        print(OK + "no duplicate keys")

    # ---- 1. every rail must be LINEAR (a degenerate rectangle) -------------------------------
    boxes = [i for i, r in enumerate(rails) if (r[1] - r[0]) > EPS and (r[3] - r[2]) > EPS]
    if boxes:
        fails += 1
        print(BAD + f"rails {boxes} have width in BOTH axes -- they are boxes, not lines. "
                    f"A horizontal leg needs b_min == b_max; a vertical leg needs a_min == a_max.")
    else:
        print(OK + "all rails are linear (degenerate in one axis)")

    # ---- 2. start must lie ON a rail ---------------------------------------------------------
    d0 = min(point_dist((0.0, 0.0), r) for r in rails)
    if d0 > a.tol:
        fails += 1
        print(BAD + f"START (0,0) is {d0*1000:.0f} mm off the nearest rail -- the arm begins pinned "
                    f"against a wall. Add a rail through the origin.")
    else:
        print(OK + "START (0,0) lies on a rail")

    # ---- 3. connectivity: every rail reachable from the one holding START ---------------------
    start = min(range(len(rails)), key=lambda i: point_dist((0.0, 0.0), rails[i]))
    seen, stack = {start}, [start]
    while stack:
        i = stack.pop()
        for j in range(len(rails)):
            if j not in seen and seg_dist(rails[i], rails[j]) <= EPS:
                seen.add(j); stack.append(j)
    orphans = sorted(set(range(len(rails))) - seen)
    if orphans:
        fails += 1
        print(BAD + f"rails {orphans} are NOT connected to the start -- unreachable, the arm can "
                    f"never enter them. Segments must touch (share an endpoint) to form a junction.")
    else:
        print(OK + f"all {len(rails)} rails connected to the start")

    # ---- 4. checkpoints and goal must lie on rails --------------------------------------------
    cm = p["checkpoint_monitor"]
    n = len(cm["checkpoint_y"])
    if len({len(cm[k]) for k in ("checkpoint_x", "checkpoint_y", "checkpoint_z", "checkpoint_radius")}) != 1:
        fails += 1
        print(BAD + "checkpoint_x/y/z/radius must all be the same length")
    wps = [(f"CP{i+1}", (cm["checkpoint_y"][i], cm["checkpoint_z"][i])) for i in range(n)]
    wps.append(("GOAL", (cm["goal_y"], cm["goal_z"])))
    off = [(nm, min(point_dist(w, r) for r in rails)) for nm, w in wps]
    bad = [(nm, d) for nm, d in off if d > a.tol]
    if bad:
        fails += 1
        for nm, d in bad:
            print(BAD + f"{nm} is {d*1000:.0f} mm off every rail -- it can never be triggered.")
    else:
        print(OK + f"all {len(wps)} waypoints (checkpoints + goal) lie on rails")

    # ---- 5. safety envelope must contain the maze ---------------------------------------------
    far = max(math.hypot(x, y) for r in rails
              for x, y in ((r[0], r[2]), (r[0], r[3]), (r[1], r[2]), (r[1], r[3])))
    lim = p["maze_safety"]["max_displacement_m"]
    if far >= lim:
        fails += 1
        print(BAD + f"maze reaches {far:.2f} m from the anchor but maze_safety.max_displacement_m is "
                    f"{lim:.2f} -- the safety stop would fire mid-trial. Raise it above {far:.2f}.")
    else:
        print(OK + f"maze reaches {far:.2f} m; safety trips at {lim:.2f} m")

    # ---- 6. CLIK nullspace posture must match move_to_start ------------------------------------
    ns_path = os.path.join(os.path.dirname(params_path), "clik_nullspace_maze.yaml")
    if os.path.exists(ns_path):
        with open(ns_path) as f:
            ns = yaml.safe_load(f)["/**/kuka_clik_controller"]["ros__parameters"]["nullspace_desired_configuration"]
        tgt = p["move_to_start"]["target_joint_position"]
        if [round(v, 4) for v in ns] != [round(v, 4) for v in tgt]:
            fails += 1
            print(BAD + f"clik_nullspace_maze.yaml {ns} != move_to_start {tgt} -- the CLIK would hold "
                        f"the right EE pose in the wrong arm posture.")
        else:
            print(OK + "CLIK nullspace posture matches move_to_start")

    # ---- 7. kinematics: is every rail physically reachable? ------------------------------------
    print("\nkinematics")
    try:
        import numpy as np
        import optas
        from scipy.spatial.transform import Rotation as R
    except Exception as e:
        print(f"  [--] skipped (needs optas/scipy/numpy): {e}")
        print(f"\n{'FAILED' if fails else 'PASSED'} ({fails} problem(s))")
        return 1 if fails else 0

    urdf = os.environ.get("IIWA7_URDF", "")
    if not urdf or not os.path.exists(urdf):
        print("  [--] skipped: set IIWA7_URDF to a generated iiwa7 URDF to enable reach checks, e.g.")
        print("       xacro $(ros2 pkg prefix lbr_description)/share/lbr_description/urdf/iiwa7/"
              "iiwa7.xacro > /tmp/iiwa7.urdf && export IIWA7_URDF=/tmp/iiwa7.urdf")
        print(f"\n{'FAILED' if fails else 'PASSED'} ({fails} problem(s))")
        return 1 if fails else 0

    rm = optas.RobotModel(urdf_filename=urdf)
    fk = rm.get_link_transform_function(link="lbr_link_ee", base_link="lbr_link_0", numpy_output=True)
    Jf = rm.get_link_geometric_jacobian_function(link="lbr_link_ee", base_link="lbr_link_0", numpy_output=True)
    lim_j = np.deg2rad([170, 120, 170, 120, 170, 120, 175])
    q0 = np.deg2rad(np.array(p["move_to_start"]["target_joint_position"], float))
    T0 = fk(q0); R0 = T0[:3, :3]
    sag = float(p["virtual_fixtures"].get("anchor_settle_sec", 0) and 0.04)
    P0 = T0[:3, 3] + np.array([0, 0, -sag])
    locked = cfg.get("restricted_axis", "z").lower()

    def to_xyz(pt):
        if locked == "x":   return P0 + np.array([0, pt[0], pt[1]])
        if locked == "y":   return P0 + np.array([pt[0], 0, pt[1]])
        return P0 + np.array([pt[0], pt[1], 0])

    def _ik_from(tp, seed, iters=400):
        q = seed.copy()
        for _ in range(iters):
            T = fk(q); ep = tp - T[:3, 3]
            eo = R.from_matrix(R0 @ T[:3, :3].T).as_rotvec()
            if np.linalg.norm(ep) < 2e-3 and np.linalg.norm(eo) < np.deg2rad(1):
                return q
            J = Jf(q)
            q = np.clip(q + 0.4 * (J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(6),
                                                         np.concatenate([ep, eo]))), -lim_j, lim_j)
        ok = np.linalg.norm(tp - fk(q)[:3, 3]) < 2e-3 and np.all(np.abs(q) < lim_j - np.deg2rad(3))
        return q if ok else None

    def ik(tp):
        """Damped-least-squares IK, retried from several seeds.

        A SINGLE seed gives false negatives: the solver is a local method on a redundant arm, so it can
        stall short of a target that is perfectly reachable from a different arm configuration. That
        happened here -- a point on the goal row was reported unreachable and then solved first try from
        another seed. Reporting a good maze as broken is worse than taking a few extra iterations.
        """
        rng = np.random.default_rng(0)
        for seed in [q0] + [np.clip(q0 + rng.uniform(-0.6, 0.6, 7), -lim_j, lim_j) for _ in range(12)]:
            q = _ik_from(tp, seed)
            if q is not None:
                return q
        return None

    pts, unreachable, costs = set(), [], []
    for r in rails:
        steps = max(3, int(max(r[1] - r[0], r[3] - r[2]) / 0.05) + 1)
        for t in [i / (steps - 1) for i in range(steps)]:
            pts.add((round(r[0] + t * (r[1] - r[0]), 4), round(r[2] + t * (r[3] - r[2]), 4)))
    for pt in sorted(pts):
        q = ik(to_xyz(pt))
        if q is None:
            unreachable.append(pt)
        else:
            costs.append(float(np.linalg.norm(np.linalg.pinv(Jf(q)) @ np.array([0, 1, 0, 0, 0, 0.0]))))
    if unreachable:
        fails += 1
        print(BAD + f"{len(unreachable)}/{len(pts)} rail points NOT reachable holding the tool "
                    f"orientation, e.g. {unreachable[:4]}")
        print("       Move them inside the envelope (measured a in [-0.40,+0.40], b in [-0.35,+0.20],")
        print("       TRAPEZOIDAL -- much narrower near the top).")
    else:
        print(OK + f"all {len(pts)} sampled rail points reachable holding the tool orientation")
    if costs:
        mx = max(costs)
        msg = OK if mx < 12 else BAD
        if mx >= 12:
            fails += 1
        print(msg + f"sideways-motion cost ||qdot||: median {sorted(costs)[len(costs)//2]:.1f}, "
                    f"max {mx:.1f} (over ~12 feels like treacle -- try a different start posture)")

    print(f"\n{'FAILED' if fails else 'PASSED'} ({fails} problem(s))")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
