#!/usr/bin/env python3
from __future__ import annotations

from typing import Callable, Optional
import numpy as np

from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_restricted_on_a_plane import MoveRestrictedOnAPlaneAction


class MoveInMazeAction(MoveRestrictedOnAPlaneAction):
    """Planar maze = a MoveRestrictedOnAPlaneAction specialised with corridors.

    The maze IS a restricted-plane fixture (lock one axis + orientation to the start plane) plus a
    corridor projection in the freed 2D plane, so it reuses the base class wholesale: the LBRState
    subscription, optas FK, the state-rate equilibrium loop, the step clamp, and the publish to
    kuka_clik_controller/target_frame all come from the base. This subclass adds only two things:

      1. it loads the corridor rectangles (in __init__), and
      2. it overrides apply_surface_constraints to lock the out-of-plane axis + orientation to the
         start pose (exactly like the base 'plane' profile) and then clamp the in-plane (a, b) point
         to the union of corridors.

    Everything else -- and the "stream the constrained equilibrium; the cabinet impedance provides
    the compliance and the soft/firm walls" contract -- is inherited unchanged.
    """

    def __init__(self, node: rclpyNode, *, param_prefix: str = "", on_complete: Optional[Callable[[], None]] = None) -> None:
        # Base wires up sub/pub/FK/recorder and loads the profile 'type' + restricted_axis + step clamp.
        super().__init__(node, param_prefix=param_prefix, on_complete=on_complete)

        if self.profile_config.get("type") != "maze":
            raise ValueError(
                f"MoveInMazeAction requires a 'maze' profile, got type='{self.profile_config.get('type')}' "
                f"for profile '{self.active_profile}'. Use MoveRestrictedOnAPlaneAction for plane/sinusoid."
            )

        prefix = self._param_prefix + "virtual_fixtures." + f"{self.active_profile}."

        # Corridors = parallel arrays of axis-aligned rectangles in the two in-plane (a, b) axes (the
        # non-restricted axes). Free inside any corridor; clamped to the nearest edge outside it (the
        # cabinet impedance then turns "outside" into a soft/firm wall).
        amn = list(node.get_parameter(prefix + "corridor_a_min").value) if node.has_parameter(prefix + "corridor_a_min") else []
        amx = list(node.get_parameter(prefix + "corridor_a_max").value) if node.has_parameter(prefix + "corridor_a_max") else []
        bmn = list(node.get_parameter(prefix + "corridor_b_min").value) if node.has_parameter(prefix + "corridor_b_min") else []
        bmx = list(node.get_parameter(prefix + "corridor_b_max").value) if node.has_parameter(prefix + "corridor_b_max") else []
        if len(amn) == 0 or not (len(amn) == len(amx) == len(bmn) == len(bmx)):
            # Empty corridors => projection passes the point straight through => the same silent
            # no-fixture failure as a missing profile. Refuse to run.
            raise ValueError(
                "maze profile needs equal-length, non-empty corridor_a_min/a_max/b_min/b_max arrays"
            )
        self._maze_corridors = [
            (float(amn[i]), float(amx[i]), float(bmn[i]), float(bmx[i])) for i in range(len(amn))
        ]
        # "relative" (default): corridor coords are OFFSETS from the start EE, so the maze origin is
        # wherever the arm starts. "absolute": corridors are raw base-frame coords.
        self._corridor_relative = True
        if node.has_parameter(prefix + "corridor_frame"):
            self._corridor_relative = (
                str(node.get_parameter(prefix + "corridor_frame").value).lower() != "absolute"
            )

        node.get_logger().info(f"Maze fixture: {len(self._maze_corridors)} corridors loaded.")

    def apply_surface_constraints(self, transform: np.ndarray) -> tuple[np.ndarray, bool]:
        """Lock the out-of-plane axis + orientation to the start pose (like the base 'plane' profile),
        then clamp the in-plane point to the union of corridor rectangles."""
        x = transform[0, 3]
        y = transform[1, 3]
        z = transform[2, 3]
        restricted = True

        if self._initial_transform is not None:
            locked = self.profile_config.get("restricted_axis", "z").lower()
            # Corridor origin = the start EE on the two in-plane (free) axes, so RELATIVE corridors are
            # centred on wherever the arm started (0.0 for absolute).
            si = self._initial_transform
            if locked == "x":
                x = si[0, 3]
                a0, b0 = (si[1, 3], si[2, 3]) if self._corridor_relative else (0.0, 0.0)
                y, z = self._project_to_corridors(y, z, a0, b0)
            elif locked == "y":
                y = si[1, 3]
                a0, b0 = (si[0, 3], si[2, 3]) if self._corridor_relative else (0.0, 0.0)
                x, z = self._project_to_corridors(x, z, a0, b0)
            else:  # "z" (default): maze lives in the X-Y plane
                z = si[2, 3]
                a0, b0 = (si[0, 3], si[1, 3]) if self._corridor_relative else (0.0, 0.0)
                x, y = self._project_to_corridors(x, y, a0, b0)
            # Lock orientation so the end effector doesn't twist
            transform[0:3, 0:3] = si[0:3, 0:3]

        transform[0, 3] = x
        transform[1, 3] = y
        transform[2, 3] = z
        return transform, restricted

    def _project_to_corridors(self, pa: float, pb: float,
                              a0: float = 0.0, b0: float = 0.0) -> tuple[float, float]:
        """Project an in-plane point onto the union of allowed corridor rectangles.

        Bounds are relative to the origin (a0, b0) -- the start EE for a relative maze, or (0, 0) for
        an absolute one. Inside any corridor -> the point is returned unchanged (free motion). Outside
        all corridors -> clamped to the nearest corridor boundary. Coords stay in the SAME (absolute)
        frame as the input.
        """
        corridors = self._maze_corridors
        if not corridors:
            return pa, pb
        qa, qb = pa - a0, pb - b0  # into corridor-local (origin-relative) coordinates
        for (amn, amx, bmn, bmx) in corridors:
            if amn <= qa <= amx and bmn <= qb <= bmx:
                return pa, pb  # inside a corridor: free motion
        best = (qa, qb)
        best_d = float("inf")
        for (amn, amx, bmn, bmx) in corridors:
            ca = min(max(qa, amn), amx)
            cb = min(max(qb, bmn), bmx)
            d = (qa - ca) ** 2 + (qb - cb) ** 2
            if d < best_d:
                best_d = d
                best = (ca, cb)
        return best[0] + a0, best[1] + b0  # back to absolute
