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
    kuka_clik_controller/target_frame all come from the base. This subclass adds two things:

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

        # Periodic maze-position logging (see _record_extra). Reuses the fixture's own debug flags if
        # present so it can be switched off without touching code.
        # How far off a rail still counts as "on" it, for logging only (the projection is unaffected).
        self._on_rail_tol = 0.015
        self._dbg = None
        self._log_dt = 1.0 / 100.0
        try:
            from sinthlab_bringup.helpers.common_threshold import DebugTicker
            rate = 2.0
            if node.has_parameter(self._param_prefix + "maze_safety.debug_log_rate_hz"):
                rate = float(node.get_parameter(self._param_prefix + "maze_safety.debug_log_rate_hz").value)
            self._dbg = DebugTicker(rate)
        except Exception:
            self._dbg = None

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

    # ---------------- trajectory logging (see analysis/robot_trajectory_*.csv) ----------------
    # The absolute EE pose alone is hard to reason about: what you actually want to know is "where am I
    # IN THE MAZE, and which corridor am I in". These add maze-relative columns so a run can be plotted
    # straight on top of the corridor rectangles from maze_params.yaml.
    def _record_extra_header(self):
        return ["rel_a", "rel_b", "corridor", "off_rail", "rail_dist"]

    def _maze_coords(self, measured_T):
        """Return (a, b, corridor_index, clamped) for a measured EE transform.

        a, b are the in-plane offsets from the maze origin (the anchored start), in the same frame the
        corridors are written in. corridor_index is the first corridor containing the point, or -1 when
        the arm is OUTSIDE every corridor (i.e. being held against a wall). clamped mirrors that as 0/1.
        """
        if self._initial_transform is None:
            return 0.0, 0.0, -1, 1
        si = self._initial_transform
        locked = self.profile_config.get("restricted_axis", "z").lower()
        if locked == "x":
            a, b = measured_T[1, 3], measured_T[2, 3]
            a0, b0 = (si[1, 3], si[2, 3]) if self._corridor_relative else (0.0, 0.0)
        elif locked == "y":
            a, b = measured_T[0, 3], measured_T[2, 3]
            a0, b0 = (si[0, 3], si[2, 3]) if self._corridor_relative else (0.0, 0.0)
        else:
            a, b = measured_T[0, 3], measured_T[1, 3]
            a0, b0 = (si[0, 3], si[1, 3]) if self._corridor_relative else (0.0, 0.0)
        qa, qb = a - a0, b - b0
        # Report the NEAREST segment and how far off it we are, rather than a strict inside/outside
        # test. With zero-width (linear) corridors a strict test can never pass -- the arm is only ever
        # exactly on a rail by coincidence -- so it would always read "outside" and tell you nothing.
        idx, best = -1, float("inf")
        for i, (amn, amx, bmn, bmx) in enumerate(self._maze_corridors):
            ca = min(max(qa, amn), amx)
            cb = min(max(qb, bmn), bmx)
            d = ((qa - ca) ** 2 + (qb - cb) ** 2) ** 0.5
            if d < best:
                best, idx = d, i
        # "on the rail" if within this distance of it; beyond that the spring is actively pulling back.
        on_rail = best <= self._on_rail_tol
        return qa, qb, (idx if on_rail else -1), (0 if on_rail else 1), best, idx

    def _record_extra(self, measured_T):
        qa, qb, idx, clamped, dist, nearest = self._maze_coords(measured_T)
        # Live view of progress. Without this you cannot tell "I am inside C1 and need to move further"
        # from "I am pinned against a wall" -- they feel similar through a compliant arm.
        if self._dbg is not None and self._dbg.tick(self._log_dt):
            where = (f"on rail C{nearest + 1}" if idx >= 0
                     else f"OFF rail (nearest C{nearest + 1}, {dist * 1000:.0f} mm away, being pulled back)")
            self._node.get_logger().info(
                f"maze: a={qa:+.3f} b={qb:+.3f} -> {where}"
            )
        return [round(qa, 5), round(qb, 5), idx, clamped, round(dist, 5)]

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
