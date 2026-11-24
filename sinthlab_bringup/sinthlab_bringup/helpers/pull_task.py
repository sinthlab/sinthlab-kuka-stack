"""Cued Pull: the trial logic -- no ROS, unit-tested.

The task (actions/cued_pull.py on the free-move node): he grips the apple and pulls it, free in 3-D inside
the box (free_move.rail_axis none; or frontal, held to the plane facing him). Only the movement's part in
the plane facing him is scored -- depth (toward or away from him) is free. A pull ENDS when that part has
gone pull.distance_m from where the pull started (a circle around it is crossed). Three modes, live, switchable between trials:

    shaping   any direction: every crossing is rewarded (the whole ring lights at the go cue)
    jump      cued: one quarter of the ring lights, the direction to pull. On a fraction of trials
              (perturb.probability) the apple JUMPS perturb.jump_m before he has gripped it, a set delay
              after the go cue; he then pulls from where it landed. The other trials are standard.
    bump      cued, as jump; on perturbed trials, once he has pulled perturb.bump_at_m along the cued
              direction, the apple is BUMPED perturb.bump_m across it (perpendicular by default), and he
              must correct to finish inside the target.

A cued pull is correct when the crossing point lies within pull.cone_deg of the cued direction; a
crossing outside it ends the trial without reward (pull_wrong). The cone is checked at the crossing
only, so a path may curve -- the bump's correction is the point.

Directions are named from the monkey's side, in the frontal plane: right (+Y), up (+Z), left, down.
Angles are degrees in that plane, 0 = right, 90 = up. A perturbation's angle is relative to the cued
direction: 90 = a quarter turn counter-clockwise as he sees it (right -> up), 270 = clockwise.

THE SCHEDULE (PullSchedule) draws trials in shuffled blocks: every direction pull.block_trials times; of
those, round(probability x block_trials) are perturbed, cycling through the perturbation angles, so
directions, perturbed trials and angles stay balanced however long the run. A perturbed trial never
follows more than perturb.max_in_row perturbed trials, so each one stays unexpected. Any change of mode or
settings starts a new block.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

MODES = ("shaping", "jump", "bump")
SHAPES = ("none", "frontal")      # free_move.rail_axis: free in 3-D (the default), or held to the frontal plane
CUED = ("jump", "bump")
# Frontal-plane unit vectors (base Y, base Z) and the ring quarter that lights for each direction.
DIRECTIONS: Dict[str, Tuple[float, float]] = {"right": (1.0, 0.0), "up": (0.0, 1.0),
                                              "left": (-1.0, 0.0), "down": (0.0, -1.0)}
QUARTER = {"up": 0, "right": 1, "down": 2, "left": 3}
BOX_MARGIN_M = 0.005      # every point a trial can need stays this far inside the box


def unit(direction: str) -> np.ndarray:
    try:
        return np.array(DIRECTIONS[direction])
    except KeyError:
        raise ValueError(f"unknown direction {direction!r}: use {list(DIRECTIONS)}") from None


def rotate(u: np.ndarray, deg: float) -> np.ndarray:
    """u turned by deg, counter-clockwise as the monkey sees the frontal plane (right -> up)."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([c * u[0] - s * u[1], s * u[0] + c * u[1]])


def angle_of(d: Sequence[float]) -> float:
    """The direction of a frontal-plane vector [deg], 0 = right, 90 = up, in [0, 360)."""
    return math.degrees(math.atan2(d[1], d[0])) % 360.0


def signed_error(d: Sequence[float], direction: str) -> float:
    """How far the vector d points from the named direction [deg], -180..180, + = counter-clockwise."""
    e = angle_of(d) - angle_of(unit(direction))
    return (e + 180.0) % 360.0 - 180.0


def frontal(offset3: Sequence[float]) -> np.ndarray:
    """A base-frame offset (x, y, z) -> its frontal-plane part (y, z)."""
    return np.array([float(offset3[1]), float(offset3[2])])


def to_base(d2: Sequence[float]) -> np.ndarray:
    """A frontal-plane vector (y, z) -> a base-frame offset (0, y, z)."""
    return np.array([0.0, float(d2[0]), float(d2[1])])


def crossing(d: Sequence[float], distance_m: float, direction: Optional[str], cone_deg: float):
    """The pull's outcome once it has gone distance_m: None before; else (correct, angle, error) --
    shaping (direction None): always correct; cued: correct within cone_deg of the direction."""
    if float(np.hypot(d[0], d[1])) < distance_m:
        return None
    ang = round(angle_of(d), 1)
    if direction is None:
        return True, ang, None
    err = round(signed_error(d, direction), 1)
    return abs(err) <= cone_deg, ang, err


def min_jerk(s: float) -> float:
    """Minimum-jerk position profile, 0..1 over s in 0..1."""
    s = min(max(s, 0.0), 1.0)
    return s ** 3 * (10.0 - 15.0 * s + 6.0 * s * s)


def min_jerk_peak_speed(size_m: float, duration_sec: float) -> float:
    return 1.875 * size_m / duration_sec


# ---------------------------------------------------------------- settings and their checks
@dataclass
class PullSettings:
    mode: str = "shaping"
    directions: List[str] = field(default_factory=lambda: ["right", "left", "down", "up"])
    distance_m: float = 0.05
    cone_deg: float = 15.0
    block_trials: int = 6
    probability: float = 0.33
    max_in_row: int = 1
    jump_m: float = 0.015
    jump_sec: float = 0.1
    jump_angles_deg: List[float] = field(default_factory=lambda: [90.0, 270.0])
    jump_delay_min_sec: float = 0.1
    jump_delay_max_sec: float = 0.3
    bump_m: float = 0.015
    bump_sec: float = 0.1
    bump_angles_deg: List[float] = field(default_factory=lambda: [90.0, 270.0])
    bump_at_m: float = 0.025

    def angles(self) -> List[float]:
        """This mode's perturbation angles (none in shaping)."""
        return list(self.jump_angles_deg if self.mode == "jump" else
                    self.bump_angles_deg if self.mode == "bump" else [])

    def size(self) -> float:
        return self.jump_m if self.mode == "jump" else self.bump_m if self.mode == "bump" else 0.0


def settings_problem(s: PullSettings) -> Optional[str]:
    """None if the settings make sense together, else the reason (geometry is box_problem's)."""
    if s.mode not in MODES:
        return f"pull.mode must be one of {list(MODES)}, got {s.mode!r}"
    if not s.directions:
        return "pull.directions needs at least one direction"
    bad = [d for d in s.directions if d not in DIRECTIONS]
    if bad:
        return f"pull.directions: unknown {bad}; use {list(DIRECTIONS)}"
    if len(set(s.directions)) != len(s.directions):
        return "pull.directions lists a direction twice"
    if not 0.0 < s.cone_deg < 45.0:
        return "pull.cone_deg must be between 0 and 45 (wider and two directions' targets overlap)"
    if not 0.0 <= s.probability <= 1.0:
        return "perturb.probability must be 0-1"
    if s.block_trials < 1:
        return "pull.block_trials must be at least 1"
    if s.jump_delay_min_sec > s.jump_delay_max_sec:
        return "perturb.jump_delay_min_sec must be <= perturb.jump_delay_max_sec"
    if not 0.0 < s.bump_at_m < s.distance_m - 0.01:
        return (f"perturb.bump_at_m ({s.bump_at_m:g}) must be positive and at least 1 cm short of "
                f"pull.distance_m ({s.distance_m:g}), so there is room to correct")
    for name, m, sec in (("jump", s.jump_m, s.jump_sec), ("bump", s.bump_m, s.bump_sec)):
        if m > 0.0 and sec <= 0.0:
            return f"perturb.{name}_sec must be positive"
    if s.mode in CUED and s.probability > 0.0 and not s.angles():
        return f"perturb.{s.mode}_angles_deg needs at least one angle"
    return None


def needed_points(s: PullSettings) -> List[np.ndarray]:
    """Every frontal-plane offset from the start a trial under these settings can need the arm at:
    each direction's end, from the start and after every jump; and where every bump leaves it."""
    pts = []
    for name in s.directions:
        u = unit(name)
        pts.append(s.distance_m * u)
        for a in s.jump_angles_deg:
            pts.append(s.jump_m * rotate(u, a) + s.distance_m * u)
        for a in s.bump_angles_deg:
            pts.append(s.bump_at_m * u + s.bump_m * rotate(u, a))
    return pts


def box_problem(s: PullSettings, lo3: Sequence[float], hi3: Sequence[float]) -> Optional[str]:
    """None if every point needed_points() lists is inside the box (offsets from the start, base frame),
    with BOX_MARGIN_M to spare; else which one is not. Every mode is checked, so switching mode mid-run
    can never ask for a pull the box cuts off."""
    lo, hi = frontal(lo3) + BOX_MARGIN_M, frontal(hi3) - BOX_MARGIN_M
    for p in needed_points(s):
        if np.any(p < lo - 1e-9) or np.any(p > hi + 1e-9):
            where = ", ".join(f"{abs(v) * 100:.1f} cm {name[int(v < 0)]}" for v, name in
                              ((p[0], ("right", "left")), (p[1], ("up", "down"))) if abs(v) > 1e-6)
            return (f"a pull needs the apple {where} from the start, outside the box (free_move.box, "
                    f"{BOX_MARGIN_M * 1000:.0f} mm margin): shorten pull.distance_m or the perturbations, "
                    f"or drop a direction")
    return None


# The parameters (config/cued_pull.yaml) and how they map onto PullSettings.
_FLOATS = {"pull.distance_m": "distance_m", "pull.cone_deg": "cone_deg", "perturb.probability": "probability",
           "perturb.jump_m": "jump_m", "perturb.jump_sec": "jump_sec",
           "perturb.jump_delay_min_sec": "jump_delay_min_sec", "perturb.jump_delay_max_sec": "jump_delay_max_sec",
           "perturb.bump_m": "bump_m", "perturb.bump_sec": "bump_sec", "perturb.bump_at_m": "bump_at_m"}
_INTS = {"pull.block_trials": "block_trials", "perturb.max_in_row": "max_in_row"}
_LISTS = {"perturb.jump_angles_deg": "jump_angles_deg", "perturb.bump_angles_deg": "bump_angles_deg"}
PARAMS = ("pull.mode", "pull.directions") + tuple(_FLOATS) + tuple(_INTS) + tuple(_LISTS)


def settings_from(get: Callable[[str], object]) -> PullSettings:
    """PullSettings from parameter values (get(name) -> value)."""
    s = PullSettings(mode=str(get("pull.mode")).strip().lower(),
                        directions=[str(d).strip().lower() for d in (get("pull.directions") or [])])
    for name, attr in _FLOATS.items():
        setattr(s, attr, float(get(name)))
    for name, attr in _INTS.items():
        setattr(s, attr, int(get(name)))
    for name, attr in _LISTS.items():
        setattr(s, attr, [float(a) for a in (get(name) or [])])
    return s


def params_problem(values: Dict[str, object]) -> Optional[str]:
    """None if a full set of Cued Pull parameter values (flat names, the box included) makes sense together
    and fits the box; else the reason. What the dashboard checks before a run or a live change; the node
    checks the same (actions/cued_pull.py)."""
    if "pull.mode" not in values:
        return None
    from sinthlab_bringup.helpers.free_move_box import box_bounds
    try:
        s = settings_from(lambda n: values[n])
        lo, hi = box_bounds(lambda n: values[n])
    except (KeyError, TypeError, ValueError) as exc:
        return f"Cued Pull settings: {exc}"
    if str(values.get("free_move.rail_axis", "none")) not in SHAPES:
        return f"Cued Pull needs free_move.rail_axis {' or '.join(SHAPES)}"
    lo_d, hi_d = float(values.get("pull.cue_delay_min_sec", 0.0)), float(values.get("pull.cue_delay_max_sec", 0.0))
    if lo_d > hi_d:
        return "pull.cue_delay_min_sec must be <= pull.cue_delay_max_sec"
    return settings_problem(s) or box_problem(s, lo, hi)


# ---------------------------------------------------------------- the per-trial plan
@dataclass
class TrialPlan:
    mode: str
    direction: Optional[str]          # None in shaping (any direction)
    perturb: Optional[str] = None     # "jump" | "bump" | None
    angle_deg: Optional[float] = None
    size_m: float = 0.0
    block: int = 0

    def offset(self) -> Optional[np.ndarray]:
        """The perturbation as a frontal-plane offset (y, z) [m], or None."""
        if self.perturb is None or self.direction is None:
            return None
        return self.size_m * rotate(unit(self.direction), float(self.angle_deg))

    def record(self) -> dict:
        off = self.offset()
        return {"mode": self.mode, "direction": self.direction, "perturb": self.perturb,
                "angle_deg": self.angle_deg, "size_m": self.size_m, "block": self.block,
                "offset_yz_m": None if off is None else [round(float(v), 4) for v in off]}


class PullSchedule:
    def __init__(self, settings: PullSettings, rng: Optional[random.Random] = None) -> None:
        self._rng = rng or random.Random()
        self._s = settings
        self._queue: List[TrialPlan] = []
        self._key = None
        self.block = 0
        self._in_row = 0              # perturbed trials in a row, so far

    def configure(self, settings: PullSettings) -> None:
        self._s = settings

    def _signature(self):
        s = self._s
        return (s.mode, tuple(s.directions), s.block_trials, s.probability, s.max_in_row,
                tuple(s.angles()), s.size())

    def _new_block(self) -> None:
        s = self._s
        self.block += 1
        self._key = self._signature()
        if s.mode not in CUED:
            self._queue = [TrialPlan(s.mode, None, block=self.block)]
            return
        n_pert = int(round(s.probability * s.block_trials)) if s.angles() else 0
        trials: List[TrialPlan] = []
        k = 0
        angles = s.angles()
        for name in s.directions:
            for i in range(s.block_trials):
                if i < n_pert:
                    a = angles[k % len(angles)]
                    k += 1
                    trials.append(TrialPlan(s.mode, name, s.mode, float(a), s.size(), self.block))
                else:
                    trials.append(TrialPlan(s.mode, name, block=self.block))
        self._queue = self._shuffled(trials)

    def _shuffled(self, trials: List[TrialPlan]) -> List[TrialPlan]:
        """A random order in which no more than max_in_row perturbed trials run together, counting the end of
        the last block: the standard trials are shuffled, and the perturbed ones dropped into the gaps
        between them, at most max_in_row to a gap. If there are too many perturbed trials for that, the cap
        is raised as little as needed."""
        cap = max(0, int(self._s.max_in_row))
        pert = [t for t in trials if t.perturb]
        std = [t for t in trials if not t.perturb]
        self._rng.shuffle(pert)
        self._rng.shuffle(std)
        if cap == 0 or not pert:
            out = pert + std
            self._rng.shuffle(out)
            return out
        gaps = len(std) + 1
        caps = [cap] * gaps
        caps[0] = max(0, cap - self._in_row)        # the last block may have ended on perturbed trials
        while sum(caps) < len(pert):
            caps = [c + 1 for c in caps]
        slots = [g for g in range(gaps) for _ in range(caps[g])]
        counts = [0] * gaps
        for g in self._rng.sample(slots, len(pert)):
            counts[g] += 1
        out, it = [], iter(pert)
        for g in range(gaps):
            out += [next(it) for _ in range(counts[g])]
            if g < len(std):
                out.append(std[g])
        return out

    def next(self) -> TrialPlan:
        """This trial's plan. A changed mode or setting starts a new block."""
        if not self._queue or self._signature() != self._key:
            self._new_block()
        plan = self._queue.pop(0)
        if self._s.mode not in CUED:
            self._queue = []          # shaping: a fresh one each trial
        self._in_row = self._in_row + 1 if plan.perturb else 0
        return plan

    def remaining(self) -> int:
        return len(self._queue)
