"""The floor: a height above the ground the end effector must never go below, in every experiment.

The robot stands on the ground, so the base frame's z IS the height above the ground. What has to
stay above the floor is one point of the end effector, chosen with `floor.reference`:

    apple_bottom   the lowest point of the apple ball
    base_edge      the lowest point of the Ø188 base (tiers + cover). With the base horizontal (the
                   apple pointing up) that is its bottom face; tilted, it is the edge of its rim.

Both follow the tool's orientation, from the end-effector geometry below (end_effector_design/).
lbr_link_ee is the robot flange; the tool axis is its z.

The floor is enforced three ways (README, "The floor"):
  * a soft wall: wherever ROS moves the spring's equilibrium (the CLIK experiments' fixtures and Free
    Move, the apple pluck softener), it never puts the reference point below the floor;
  * a trip: SafetyStopMonitor aborts the trial and parks if the MEASURED reference point gets more
    than `floor.trip_margin_m` below the floor -- the spring alone cannot stop a hard push;
  * pose checks: the start and park poses, the joint-space path between them, the random_start
    positions and each perturbation are checked against it before the arm moves there.

Plain numpy, no ROS: the dashboard uses it too (start height).
"""
from __future__ import annotations

from typing import Callable, Optional

import numpy as np

# End-effector geometry, along the tool axis from the robot flange [m] (end_effector_design/README.md).
APPLE_CENTRE_M = 0.212
APPLE_RADIUS_M = 0.0225         # Ø45 ball
BASE_RADIUS_M = 0.094           # Ø188 tiers and cover
BASE_FROM_M, BASE_TO_M = 0.016, 0.090   # the base stack: on top of the 16 mm flange plate, to the cover top

REFERENCES = ("apple_bottom", "base_edge")


def reference_drop(R: np.ndarray, reference: str) -> float:
    """Height of the reference point minus the flange's, for flange orientation R (3x3, base frame)."""
    az = float(np.clip(R[2, 2], -1.0, 1.0))         # vertical component of the tool axis
    if reference == "apple_bottom":
        return APPLE_CENTRE_M * az - APPLE_RADIUS_M
    if reference == "base_edge":
        # A tilted cylinder's lowest point lies on the rim of one of its two end faces.
        return min(BASE_FROM_M * az, BASE_TO_M * az) - BASE_RADIUS_M * float(np.sqrt(1.0 - az * az))
    raise ValueError(f"floor.reference must be one of {REFERENCES}, got {reference!r}")


def reference_z(T: np.ndarray, reference: str) -> float:
    """Height above the ground of the reference point, for the flange pose T (4x4, base frame)."""
    return float(T[2, 3]) + reference_drop(T[:3, :3], reference)


class Floor:
    def __init__(self, height_m: float, reference: str, trip_margin_m: float = 0.03) -> None:
        reference_drop(np.eye(3), reference)            # validates the name
        self.height = float(height_m)
        self.reference = reference
        self.trip_margin = max(0.0, float(trip_margin_m))

    @classmethod
    def from_node(cls, node) -> "Floor":
        from sinthlab_bringup.helpers.node_utils import get_required_param
        return cls(float(get_required_param(node, "floor.height_m")),
                   str(get_required_param(node, "floor.reference")).strip().lower(),
                   float(get_required_param(node, "floor.trip_margin_m")))

    def describe(self) -> str:
        return f"{self.reference.replace('_', ' ')} >= {self.height:.3f} m above the ground"

    def clearance(self, T: np.ndarray) -> float:
        """How far the reference point is above the floor [m]; negative = below it."""
        return reference_z(T, self.reference) - self.height

    def clamp_xyz(self, xyz, R: np.ndarray) -> np.ndarray:
        """The flange position `xyz`, raised just enough that the reference point is not below the
        floor, for orientation R. The soft wall: applied to the equilibrium, never to the arm."""
        out = np.array(xyz, dtype=float)
        out[2] = max(out[2], self.height - reference_drop(R, self.reference))
        return out

    def check_joint_path(self, fk: Callable, q_from, q_to, name: str, steps: int = 40) -> Optional[str]:
        """None if a joint-space move q_from -> q_to [rad] keeps the reference point above the floor
        all the way, else the reason. fk(q) -> 4x4 flange pose."""
        q_from, q_to = np.asarray(q_from, dtype=float), np.asarray(q_to, dtype=float)
        worst = min(self.clearance(fk(q_from + t * (q_to - q_from))) for t in np.linspace(0.0, 1.0, steps + 1))
        if worst < 0.0:
            return (f"{name} takes the {self.reference.replace('_', ' ')} {-worst * 100:.1f} cm below the "
                    f"floor ({self.height:.3f} m)")
        return None


ROOM_M = 0.01           # the task's lowest point must clear the floor by at least this [m]
_START, _PARK = "move_to_start.target_joint_position", "move_to_park.target_joint_position"
_THRESHOLD = "apple_pluck_impedance_control_displacement.cartesian_displacement_threshold_m"


def setup_problem(params: dict, fk: Callable, grid=None) -> Optional[str]:
    """None if an experiment's configuration -- flat parameter names, as in the YAML -- leaves the floor
    clear of everything the arm is meant to do, else the reason. The dashboard runs it on every edit that
    could change the answer and at Start, so a floor in the way is refused before anything moves:
      * the start and park poses and the joint-space move between them;
      * apple pluck: a pull straight DOWN to the threshold, from the start, every random-start position
        (needs `grid`, a StartGrid) and the perturbed start, must stay ROOM_M above the floor -- the
        floor must never be what ends a pull;
      * maze and Move Vertical: the lowest rail point must be ROOM_M above the floor."""
    floor = Floor(params["floor.height_m"], params["floor.reference"], params.get("floor.trip_margin_m", 0.03))
    what = floor.reference.replace("_", " ")
    start, park = params.get(_START), params.get(_PARK)
    if start is None:
        return None
    q0 = np.radians(np.asarray(start, dtype=float))
    T0 = fk(q0)
    z0 = reference_z(T0, floor.reference)
    if park is not None:
        problem = floor.check_joint_path(fk, np.radians(np.asarray(park, dtype=float)), q0,
                                         "the move between park and start")
        if problem:
            return problem

    def room(lowest: float, task: str) -> Optional[str]:
        if lowest - ROOM_M >= floor.height:
            return None
        return (f"the floor ({floor.height:.3f} m) is in the way of {task}: the {what} goes down to "
                f"{lowest:.3f} m there, so the floor must be at or below {lowest - ROOM_M:.3f} m")

    threshold = params.get(_THRESHOLD)
    if threshold is not None and "paradigm" in params:          # apple pluck (restricted plane has no paradigm)
        paradigm = str(params.get("paradigm", "pluck"))
        starts = {"the start": z0}
        if paradigm == "random_start" and grid is not None:
            plane = str(params.get("random_start.plane", "frontal"))
            prefix = "random_start.positions."
            offsets = {k[len(prefix):]: v for k, v in params.items() if k.startswith(prefix)}
            for name, (q, why) in grid.solve(start, plane, offsets).items():
                if why is None:
                    starts[f"random-start position {name}"] = reference_z(fk(np.radians(q)), floor.reference)
                    problem = park is not None and floor.check_joint_path(
                        fk, np.radians(np.asarray(park, dtype=float)), np.radians(q), f"the move from park to {name}")
                    if problem:
                        return problem
        if paradigm in ("perturb", "perturb_bump"):
            r = float(params.get("perturb_start.polar_r_m", 0.0))
            th = np.radians(float(params.get("perturb_start.polar_theta_deg", 0.0)))
            if str(params.get("perturb_start.polar_plane", "frontal")) in ("frontal", "sagittal"):
                starts["the perturbed start"] = z0 + r * float(np.sin(th))
        name, z = min(starts.items(), key=lambda kv: kv[1])
        return room(z - float(threshold), f"a {float(threshold) * 100:.0f} cm pull straight down from {name}")

    if str(params.get("free_move.rail_axis", "none")) == "z":      # Move Vertical: the rail's bottom end
        return room(z0 + float(params["free_move.rail_min_m"]), "the bottom of the rail")
    profile = params.get("virtual_fixture_profile")                # maze
    if profile is not None:
        pre = f"virtual_fixtures.{profile}."
        b_min = params.get(pre + "corridor_b_min")
        if (b_min and params.get(pre + "corridor_frame") == "relative"
                and params.get(pre + "restricted_axis") in ("x", "y")):     # b is base Z
            return room(z0 + min(float(v) for v in b_min), "the lowest rail")
    return None
