#!/usr/bin/env python3
from __future__ import annotations

import time
import numpy as np

from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_restricted_on_a_plane import MoveRestrictedOnAPlaneAction
from sinthlab_bringup.helpers import maze_path
from sinthlab_bringup.helpers.node_utils import DebugTicker, get_required_param


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

    def __init__(self, node: rclpyNode, *, param_prefix: str = "") -> None:
        # Base wires up sub/pub/FK and loads the profile 'type' + restricted_axis + step clamp.
        super().__init__(node, param_prefix=param_prefix)

        if self.profile_config["type"] != "maze":
            raise ValueError(
                f"MoveInMazeAction requires a 'maze' profile, got type='{self.profile_config['type']}' "
                f"for profile '{self.active_profile}'. Use MoveRestrictedOnAPlaneAction for plane/sinusoid."
            )

        prefix = self._param_prefix + "virtual_fixtures." + f"{self.active_profile}."

        # TWO KINDS OF MAZE, both a "maze" profile:
        #   rails  corridor_a_min/a_max/b_min/b_max -- axis-aligned rails meeting at junctions and forks
        #   shape  path_a/path_b (+ path_closed) -- points joined by straight sides at any angle, usually
        #          a closed loop (the "0", the fish, the duck)
        # Either way corner_radius_m rounds every corner (helpers/maze_path.py).
        self._shape_points = None
        self._shape_closed = False
        if node.has_parameter(prefix + "path_a"):
            pa_ = [float(v) for v in get_required_param(node, prefix + "path_a")]
            pb_ = [float(v) for v in get_required_param(node, prefix + "path_b")]
            if len(pa_) != len(pb_):
                raise ValueError("maze shape needs equal-length path_a / path_b arrays")
            self._shape_points = list(zip(pa_, pb_))
            self._shape_closed = bool(get_required_param(node, prefix + "path_closed"))
            # Each side, as a degenerate box, so maze_geometry and the plots still list "corridors".
            pts = self._shape_points
            sides = [(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts) if self._shape_closed else len(pts) - 1)]
            amn, amx = [min(p0[0], p1[0]) for p0, p1 in sides], [max(p0[0], p1[0]) for p0, p1 in sides]
            bmn, bmx = [min(p0[1], p1[1]) for p0, p1 in sides], [max(p0[1], p1[1]) for p0, p1 in sides]
        else:
            amn, amx, bmn, bmx = (list(get_required_param(node, prefix + k))
                                  for k in ("corridor_a_min", "corridor_a_max", "corridor_b_min", "corridor_b_max"))
        # Corridors = parallel arrays of axis-aligned rectangles in the two in-plane (a, b) axes (the
        # non-restricted axes). Free inside any corridor; clamped to the nearest edge outside it (the
        # cabinet impedance then turns "outside" into a soft/firm wall).
        if len(amn) == 0 or not (len(amn) == len(amx) == len(bmn) == len(bmx)):
            # Empty corridors => projection passes the point straight through => the same silent
            # no-fixture failure as a missing profile. Refuse to run.
            raise ValueError(
                "maze profile needs equal-length, non-empty corridor_a_min/a_max/b_min/b_max arrays"
            )
        self._maze_corridors = [
            (float(amn[i]), float(amx[i]), float(bmn[i]), float(bmx[i])) for i in range(len(amn))
        ]
        # "relative": corridor coords are OFFSETS from the start EE, so the maze origin is wherever the
        # arm starts. "absolute": corridors are raw base-frame coords.
        self._corridor_relative = str(get_required_param(node, prefix + "corridor_frame")).lower() != "absolute"

        # ROUNDED CORNERS. The path the arm is held to is the rails with every corner and fork turned
        # into a quarter arc of corner_radius_m (helpers/maze_path.py), so the pull turns smoothly
        # through a corner instead of swinging 90 degrees in one tick. 0 = the bare rails.
        self._corner_radius = float(get_required_param(node, prefix + "corner_radius_m"))
        if self._shape_points is not None:
            self._path = maze_path.build_shape(self._shape_points, self._shape_closed, self._corner_radius)
        else:
            self._path = maze_path.build_path(self._maze_corridors, self._corner_radius)   # raises if it does not fit

        # PATH ADJACENCY. Two pieces of the path are neighbours if they touch -- a junction, or where
        # an arc meets its rail. Precomputed once so the per-tick projection can be restricted to the
        # piece we are ON and the pieces it actually connects to.
        self._rail_neighbours = maze_path.neighbours(self._path)
        # The rail the arm is currently travelling. Latched: see _project_to_corridors.
        self._cur_rail = None

        # EASE OF MOVING ALONG A RAIL. The equilibrium is the arm's own position projected onto the rail,
        # so along the rail the spring force is zero -- EXCEPT for tracking lag: the equilibrium reaches
        # the arm a few tens of ms late (state stream, IK, FRI), and the spring pulls back by
        # K * speed * lag, felt as drag that grows with speed. rail_lead_sec aims the equilibrium that
        # far AHEAD of the arm along the rail (from its filtered in-plane velocity):
        #   > 0  assist -- cancels the lag, so moving along a rail feels lighter
        #   = 0  off (the equilibrium is exactly the projected arm position)
        #   < 0  deliberate drag -- the equilibrium trails the arm, so faster = harder
        # Only the along-rail part acts: the led point is projected onto the rail the arm is ACTUALLY
        # on, so it can never push across a wall or pick a different rail. Live (helpers/live_params.py).
        self._rail_lead_sec = 0.0
        self._vel_tau = 0.03          # velocity low-pass time constant [s]
        self._lead_cap_m = 0.02       # never lead or trail by more than this [m]
        self._prev_q = None           # (t, qa, qb) of the previous tick
        self._vel = (0.0, 0.0)
        self._read_rail_lead(node)

        # How far off a rail still counts as "on" it, for the trial data only (the projection is unaffected).
        self._on_rail_tol = 0.015
        # Periodic log of where the arm is in the maze (see record_extra): virtual_fixtures.debug_log_*.
        fx = self._param_prefix + "virtual_fixtures."
        self._debug_log_enabled = bool(get_required_param(node, fx + "debug_log_enabled"))
        self._dbg = DebugTicker(float(get_required_param(node, fx + "debug_log_rate_hz")))
        self._log_dt = 1.0 / 100.0

        if self._shape_points is not None:
            what = f"{'closed ' if self._shape_closed else ''}shape of {len(self._shape_points)} points"
        else:
            what = f"{len(self._maze_corridors)} rails"
        node.get_logger().info(f"Maze fixture '{self.active_profile}': {what}, corners rounded to "
                               f"{self._corner_radius * 1000:.0f} mm.")

    def start(self) -> None:
        # Forget which rail we were on; the next trial re-latches from wherever the arm actually is.
        self._cur_rail = None
        self._prev_q = None
        self._vel = (0.0, 0.0)
        super().start()

    def _read_rail_lead(self, node: rclpyNode) -> None:
        self._rail_lead_sec = float(get_required_param(node, self._param_prefix + "virtual_fixtures.rail_lead_sec"))

    def reload(self) -> None:
        """Re-read the live fixture parameters (rail_lead_sec). Called at a trial boundary."""
        self._read_rail_lead(self._node)
        self._node.get_logger().info(f"Maze fixture: rail_lead_sec = {self._rail_lead_sec:+.3f} s")

    def _update_velocity(self, qa: float, qb: float) -> tuple[float, float]:
        """Low-passed in-plane velocity of the arm [m/s], from successive measured positions."""
        now = time.monotonic()
        if self._prev_q is not None:
            dt = now - self._prev_q[0]
            if 1e-4 < dt < 0.1:
                va, vb = (qa - self._prev_q[1]) / dt, (qb - self._prev_q[2]) / dt
                k = dt / (self._vel_tau + dt)
                self._vel = (self._vel[0] + k * (va - self._vel[0]), self._vel[1] + k * (vb - self._vel[1]))
            else:
                self._vel = (0.0, 0.0)          # a gap in the stream: do not extrapolate across it
        self._prev_q = (now, qa, qb)
        return self._vel

    def apply_surface_constraints(self, transform: np.ndarray) -> np.ndarray:
        """Lock the out-of-plane axis + orientation to the start pose (like the base 'plane' profile),
        then clamp the in-plane point to the union of corridor rectangles."""
        x = transform[0, 3]
        y = transform[1, 3]
        z = transform[2, 3]

        if self._initial_transform is not None:
            locked = self.profile_config["restricted_axis"]
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
            else:  # "z": maze lives in the X-Y plane
                z = si[2, 3]
                a0, b0 = (si[0, 3], si[1, 3]) if self._corridor_relative else (0.0, 0.0)
                x, y = self._project_to_corridors(x, y, a0, b0)
            # Lock orientation so the end effector doesn't twist
            transform[0:3, 0:3] = si[0:3, 0:3]

        transform[0, 3] = x
        transform[1, 3] = y
        transform[2, 3] = z
        return transform

    # ---------------- trial-data columns (the orchestrator's TrialRecorder extra_header/extra_fn) -------
    # The absolute EE pose alone is hard to reason about: what you actually want to know is "where am I
    # IN THE MAZE, and which corridor am I in". These add maze-relative columns so a run can be plotted
    # straight on top of the corridor rectangles from maze_params.yaml.
    @staticmethod
    def record_extra_header():
        # rail_nearest is the sixth: `corridor` reads -1 off-rail, so rail_nearest (worked out BEFORE
        # the on-rail test) is what says WHICH corridor the arm has been pushed off.
        return ["rel_a", "rel_b", "corridor", "off_rail", "rail_dist", "rail_nearest"]

    def maze_geometry(self) -> dict:
        """Corridors, checkpoints and goal AS THEY ARE NOW, for the trial sidecar. Without this an
        old run silently plots against whatever maze_params.yaml says today."""
        return {
            "restricted_axis": self.profile_config["restricted_axis"],
            "corridor_relative": bool(self._corridor_relative),
            "on_rail_tol_m": float(self._on_rail_tol),
            "corner_radius_m": float(self._corner_radius),
            "profile": self.active_profile,
            "shape": ([list(map(float, p)) for p in self._shape_points] if self._shape_points else None),
            "shape_closed": bool(self._shape_closed),
            "corridors": [list(map(float, c)) for c in self._maze_corridors],
        }

    # ---------------- the dashboard's live maze view (helpers/maze_view.py) -------------------------------
    def plane_axes(self):
        """Base-frame indices of the two in-plane axes (a, b)."""
        return {"x": (1, 2), "y": (0, 2), "z": (0, 1)}[self.profile_config["restricted_axis"]]

    def view_geometry(self, step: float = 0.005) -> dict:
        """The path as drawn: one polyline per piece (straight or arc), in maze (a, b) coordinates."""
        return {
            "profile": self.active_profile,
            "restricted_axis": self.profile_config["restricted_axis"],
            "relative": bool(self._corridor_relative),
            "on_rail_tol_m": float(self._on_rail_tol),
            "corner_radius_m": float(self._corner_radius),
            "closed": bool(self._shape_closed),
            "pieces": [[[round(a, 4), round(b, 4)] for a, b in maze_path.sample([e], step)] for e in self._path],
        }

    def view_state(self):
        """Where the arm is in the maze now, or None before the fixture has first anchored. After a trial
        the last anchor is kept, so the arm stays in view through the release wait and the park."""
        q = self.latest_joints
        if self._initial_transform is None or q is None:
            return None
        ia, ib = self.plane_axes()
        o = self._initial_transform[0:3, 3] if self._corridor_relative else np.zeros(3)
        p = self._fk_func(q)[0:3, 3]
        qa, qb = float(p[ia] - o[ia]), float(p[ib] - o[ib])
        _, (_, _, d2) = maze_path.nearest(self._path, qa, qb)
        state = {"ee": [round(qa, 4), round(qb, 4)], "dist_m": round(d2 ** 0.5, 4), "active": bool(self._active)}
        if self._active and self._published_xyz is not None:     # where the spring is pulling the arm
            t = self._published_xyz
            state["target"] = [round(float(t[ia] - o[ia]), 4), round(float(t[ib] - o[ib]), 4)]
        return state

    def _maze_coords(self, measured_T):
        """Return (a, b, corridor_index, clamped) for a measured EE transform.

        a, b are the in-plane offsets from the maze origin (the anchored start), in the same frame the
        corridors are written in. corridor_index is the first corridor containing the point, or -1 when
        the arm is OUTSIDE every corridor (i.e. being held against a wall). clamped mirrors that as 0/1.
        """
        if self._initial_transform is None:
            # Not anchored yet. The recorder runs from trial START, so this branch is hit for
            # every sample of the approach -- it MUST return the same arity as the anchored path or
            # the unpack below raises, and the recorder swallows it, losing the approach silently.
            return 0.0, 0.0, -1, 1, 0.0, -1
        si = self._initial_transform
        locked = self.profile_config["restricted_axis"]
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
        # Report the NEAREST piece of the path and how far off it we are, rather than a strict
        # inside/outside test. With zero-width (linear) corridors a strict test can never pass -- the
        # arm is only ever exactly on a rail by coincidence -- so it would always read "outside" and
        # tell you nothing. The distance is to the rounded path the spring actually pulls toward; an
        # arc reports the rail it leads into.
        el, (_, _, d2) = maze_path.nearest(self._path, qa, qb)
        best, idx = d2 ** 0.5, self._path[el].rail
        # "on the rail" if within this distance of it; beyond that the spring is actively pulling back.
        on_rail = best <= self._on_rail_tol
        return qa, qb, (idx if on_rail else -1), (0 if on_rail else 1), best, idx

    def record_extra(self, measured_T):
        qa, qb, idx, clamped, dist, nearest = self._maze_coords(measured_T)
        # Live view of progress. Without this you cannot tell "I am inside C1 and need to move further"
        # from "I am pinned against a wall" -- they feel similar through a compliant arm.
        if self._debug_log_enabled and self._dbg.tick(self._log_dt):
            where = (f"on rail C{nearest + 1}" if idx >= 0
                     else f"OFF rail (nearest C{nearest + 1}, {dist * 1000:.0f} mm away, being pulled back)")
            self._node.get_logger().info(
                f"maze: a={qa:+.3f} b={qb:+.3f} -> {where}"
            )
        return [round(qa, 5), round(qb, 5), idx, clamped, round(dist, 5), nearest]

    def _project_to_corridors(self, pa: float, pb: float,
                              a0: float = 0.0, b0: float = 0.0) -> tuple[float, float]:
        """Project the point onto the piece of path the arm is CURRENTLY on, or one it connects to.

        The path is the rails with rounded corners (helpers/maze_path.py); "rail" below means any
        piece of it, a straight rail or a corner arc.

        WHY NOT SIMPLY THE NEAREST RAIL: that let the arm pass through walls. Push hard enough off
        rail A and some unrelated rail B becomes the closest one, so the equilibrium jumps to B and the
        cabinet spring drags the arm across the maze -- exactly the "it went off rail" behaviour. The
        rails are a graph, and travel must follow the graph.

        So the rail is LATCHED: each tick we only consider the current rail and its neighbours (rails
        that physically touch it, i.e. real junctions). Switching therefore happens only where two rails
        actually meet. At the end of a rail with nowhere to go the projection stays clamped to the
        endpoint, and the spring holds the arm there -- a wall you cannot walk through, only push
        against.
        """
        path = self._path
        if not path:
            return pa, pb
        qa, qb = pa - a0, pb - b0  # into corridor-local (origin-relative) coordinates

        # First tick of a trial: latch onto whichever piece of the path we are actually on.
        if self._cur_rail is None:
            self._cur_rail = maze_path.nearest(path, qa, qb)[0]

        # Candidates = the piece we are on + the pieces it connects to. Never the whole maze.
        candidates = [self._cur_rail] + sorted(self._rail_neighbours[self._cur_rail])
        best_i, best = maze_path.nearest(path, qa, qb, candidates)
        self._cur_rail = best_i

        # Lead / trail along the chosen piece (see rail_lead_sec in __init__). The piece is chosen from
        # the ACTUAL position above; only the projected point moves, and only along that piece.
        va, vb = self._update_velocity(qa, qb)
        if self._rail_lead_sec != 0.0:
            da, db = self._rail_lead_sec * va, self._rail_lead_sec * vb
            n = (da * da + db * db) ** 0.5
            if n > self._lead_cap_m:
                da, db = da * self._lead_cap_m / n, db * self._lead_cap_m / n
            best = path[best_i].closest(qa + da, qb + db)
        return best[0] + a0, best[1] + b0  # back to absolute
