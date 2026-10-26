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
