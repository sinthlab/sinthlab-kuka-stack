"""Reach Map: how far a monkey reaches in each direction, measured while he moves the arm freely.

No ROS, so it can be unit-tested (sinthlab_bringup/test/test_reach_map.py). Used twice:

  * ON THE ARM (actions/long_move.py, long_move.reward `new_best`): BestTracker rewards a movement that
    takes the hand farther in its direction than any movement before it this run.
  * AFTER (or during) THE RUN (the dashboard's Reach Map view): estimate() turns the recorded movements into
    a reach per direction, and proposal() into the four reach values of the monkey's profile.

GEOMETRY. Everything is in the FRONTAL PLANE through the start, as offsets from the start [m]: a = base Y
(right), b = base Z (up) -- the plane every current task's choices lie in. Directions are angles, 0 deg =
right, 90 = up, 180 = left, 270 = down (rail_shapes' convention), split into `n` equal SECTORS centred on
0, 360/n, ... deg (12 sectors: every 30 deg, so right, up, left and down each have their own).

A MOVEMENT is one stretch of motion between two rests (helpers/long_moves.MovementSegmenter); its REACH
POINT is the farthest it got from the start. Only OUTWARD movements count (their reach point at least
OUTWARD_M farther from the start than where they began): the arm stays where it is let go, so a movement
back toward the middle starts at the last reach point and would count it twice. A sector's

    comfortable reach  = the `percentile` (80 by default) of the reach points of the movements that ended
                         up in it: how far he goes on most reaches that way
    farthest reach     = the largest of them
    wall               = where the box (the arm's limit) stops the hand that way. A comfortable reach
                         within WALL_MARGIN_M of it is ARM-LIMITED: he would have gone farther.

The profile's four values are the comfortable reach of the right, up, left and down sectors. The map starts
where the vertical tasks do, so down and up are on their rail; left and right are measured 5 cm below the
horizontal tasks' rail (proposal()).
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from sinthlab_bringup.helpers.long_moves import MovementSegmenter

WALL_MARGIN_M = 0.01      # a reach this close to the box counts as stopped by the arm, not the monkey
OUTWARD_M = 0.02          # a movement counts as a reach only if it ends up at least this much farther out than it began
MIN_MOVEMENTS = 5         # fewer movements in a sector: no estimate there
DIRECTIONS = {"right": 0.0, "up": 90.0, "left": 180.0, "down": 270.0}


def sector_of(a: float, b: float, n: int) -> int:
    """The sector (0..n-1) of the direction of (a, b); sector k is centred on k * 360 / n deg."""
    deg = math.degrees(math.atan2(b, a)) % 360.0
    return int(round(deg / (360.0 / n))) % n


def sector_angle(k: int, n: int) -> float:
    return (k * 360.0 / n) % 360.0


class BestTracker:
    """The New-best reward: a movement pays when its reach point is farther, in its sector, than every
    earlier one by at least margin_m (and at least min_m from the start). Kept for the whole run."""

    def __init__(self, n: int = 12, margin_m: float = 0.01, min_m: float = 0.03) -> None:
        self.configure(n, margin_m, min_m)

    def configure(self, n: int, margin_m: float, min_m: float) -> None:
        n = int(n)
        if n < 4 or n % 4:
            raise ValueError(f"reach_map.sectors must be a multiple of 4 (so right, up, left and down each have one), got {n}")
        if getattr(self, "n", None) != n:
            self.best = [0.0] * n            # a new sector layout starts over
        self.n, self.margin, self.min_m = n, float(margin_m), float(min_m)

    def check(self, a: float, b: float, commit: bool = True) -> Tuple[bool, int, float, float]:
        """(new best, sector, its distance, the sector's previous best). With commit, a new best is kept as
        the best (without, call keep() once it is rewarded)."""
        k, d = sector_of(a, b, self.n), math.hypot(a, b)
        prev = self.best[k]
        if d < self.min_m or d < prev + self.margin:
            return False, k, d, prev
        if commit:
            self.best[k] = d
        return True, k, d, prev

    def keep(self, k: int, d: float) -> None:
        self.best[k] = max(self.best[k], d)

    def summary(self) -> str:
        """For the log: every sector's best so far, in cm."""
        return " ".join(f"{sector_angle(k, self.n):.0f}:{v * 100:.0f}" for k, v in enumerate(self.best) if v > 0)


# ---------------------------------------------------------------- after the run
def reach_points(t: Sequence[float], pts: Sequence[Sequence[float]], segmenter: Optional[MovementSegmenter] = None
                 ) -> List[Tuple[float, float]]:
    """The reach point (a, b) of every movement in one recording. `pts` are (x, y, z) offsets from the start
    (base frame); a movement's reach point is the farthest from the start, in the frontal plane, of the
    samples from its onset to its end. Movements that do not go outward (OUTWARD_M) are left out."""
    seg = segmenter or MovementSegmenter()
    seg.reset()
    out: List[Tuple[float, float]] = []
    far: Optional[Tuple[float, float]] = None
    origin = 0.0                                  # how far from the start the movement began
    for ti, p in zip(t, pts):
        a, b = float(p[1]), float(p[2])
        kind, _ = seg.update(float(ti), p)
        if seg.active:
            if far is None:
                origin = math.hypot(a, b)
            if far is None or math.hypot(a, b) > math.hypot(*far):
                far = (a, b)
        if kind == "end" and far is not None and math.hypot(*far) - origin >= OUTWARD_M:
            out.append(far)
        if not seg.active:
            far = None
    return out


def wall_distance(deg: float, box: Optional[Tuple[float, float, float, float]]) -> Optional[float]:
    """How far from the start the box (a_min, a_max, b_min, b_max) stops the hand along a direction."""
    if box is None:
        return None
    a_min, a_max, b_min, b_max = box
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    ts = []
    if c > 1e-9:
        ts.append(a_max / c)
    elif c < -1e-9:
        ts.append(a_min / c)
    if s > 1e-9:
        ts.append(b_max / s)
    elif s < -1e-9:
        ts.append(b_min / s)
    return min(ts) if ts else None


def estimate(points: Iterable[Tuple[float, float]], n: int = 12, percentile: float = 80.0,
             box: Optional[Tuple[float, float, float, float]] = None, min_movements: int = MIN_MOVEMENTS) -> List[dict]:
    """One row per sector: angle, n (movements), comfortable and farthest reach [m] (None with too few
    movements), the box's wall that way, and whether the arm, not the monkey, limited it."""
    by: List[List[float]] = [[] for _ in range(n)]
    for a, b in points:
        by[sector_of(a, b, n)].append(math.hypot(a, b))
    rows = []
    for k, d in enumerate(by):
        deg = sector_angle(k, n)
        wall = wall_distance(deg, box)
        enough = len(d) >= min_movements
        comf = float(np.percentile(d, percentile)) if enough else None
        rows.append({"deg": deg, "n": len(d), "comfortable_m": None if comf is None else round(comf, 4),
                     "farthest_m": round(max(d), 4) if d else None, "wall_m": None if wall is None else round(wall, 4),
                     "arm_limited": bool(comf is not None and wall is not None and comf >= wall - WALL_MARGIN_M)})
    return rows


def proposal(rows: List[dict], horizontal_dz: float = 0.0) -> Dict[str, dict]:
    """The profile's four reach values: each the comfortable reach of its own direction's sector (right,
    up, left, down). Each: {value (m, or None), arm_limited, why}.

    Left and right are measured at the map's start height, horizontal_dz below the horizontal tasks' rail
    (5 cm): their rail runs near the top of the arm's workspace, where the box cuts every reach short, so
    the reach there cannot be measured -- a few cm of height changes the sideways reach little."""
    n = len(rows)
    out: Dict[str, dict] = {}
    for name, deg in DIRECTIONS.items():
        r = rows[sector_of(math.cos(math.radians(deg)), math.sin(math.radians(deg)), n)]
        why = "" if r["comfortable_m"] is not None else f"fewer than {MIN_MOVEMENTS} movements {name}"
        if not why and name in ("left", "right") and abs(horizontal_dz) > 1e-6:
            why = f"measured {abs(horizontal_dz) * 100:.0f} cm {'below' if horizontal_dz > 0 else 'above'} the horizontal tasks' rail"
        out[f"reach.{name}_m"] = {"value": r["comfortable_m"], "arm_limited": r["arm_limited"], "why": why}
    return out


def load_trial(path) -> Tuple[List[float], List[Tuple[float, float, float]]]:
    """(t, offsets from the start) of one recorded trial (robot_trajectory_*.csv), from the go cue on. The
    start is where the end effector was at `armed` (the go cue's end: still at the start pose)."""
    import csv
    t, pos, start, armed = [], [], None, False
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            try:
                p = (float(row["x"]), float(row["y"]), float(row["z"]))
                ti = float(row["t"])
            except (KeyError, TypeError, ValueError):
                continue
            if not armed:
                if row.get("event") == "armed":
                    armed, start = True, p
                else:
                    continue
            t.append(ti)
            pos.append((p[0] - start[0], p[1] - start[1], p[2] - start[2]))
    return t, pos
