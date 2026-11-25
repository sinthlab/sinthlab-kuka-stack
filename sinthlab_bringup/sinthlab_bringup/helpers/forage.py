"""Foraging: the trial logic -- no ROS, unit-tested.

The task (actions/forage.py on the free-move node; design: robot tasks doc, section 2.2): ONE ENVIRONMENT
PER SESSION, so a new one each day. He moves the apple inside a felt arena -- free in the plane facing
him (free_move.rail_axis frontal) or in 3-D (none) -- to find hidden reward ZONES. Each zone pays once per
trial after his hand has stayed in it forage.dwell_sec; the trial ends when every zone has paid
(`forage_done`) or at the time limit (forage.time_limit_sec). Every trial starts at the same place, the
start pose, near the arena's centre.

THE ENVIRONMENT: the arena's shape (free_move.box.shape: rectangle or ellipse -- an ellipsoid in 3-D)
and size (free_move.box), and the zones: forage.n_zones circles (spheres in 3-D) of forage.zone_radius_m,
placed at random from a seed -- inside the arena by forage.wall_margin_m, at least forage.start_clear_m
from the start and forage.min_gap_m from each other (edge to edge). forage.seed 0 = today's environment:
a seed from the date and the monkey (day_seed), so a restart the same day gets the same zones, the next
day new ones, and the dashboard can draw them. Any other seed repeats that environment.

THE WARM-ZONE CUE (warm.*): the ring lights in warm colour while his hand is within the WARM RADIUS of a
zone he has not emptied this trial -- a threshold cue, not warmer / colder. warm.mode:
    schedule  trials 1..shrink_trials: the warm radius shrinks from start_factor x the zone radius to the
              zone radius; then trials ..+fade_trials: the cue's brightness falls to 0; then off.
    always    full radius x start_factor, full brightness, every trial (the training version)
    off       never
PROBE TRIALS (probe.*): from trial probe.after_trial on, a probe with probability probe.probability:
no cue and no rewards, forage.probe_sec long (outcome `probe_end`); where he searches, from memory.
"""
from __future__ import annotations

import datetime as _dt
import math
import random
import zlib
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

SHAPES = ("rectangle", "ellipse")
SPACES = ("frontal", "none")            # free_move.rail_axis: the plane facing him, or 3-D
WARM_MODES = ("schedule", "always", "off")
PLANE_DIMS = (1, 2)                     # base Y (right) and Z (up): the plane facing him
SPACE_DIMS = (0, 1, 2)


def dims_for(space: str) -> Tuple[int, ...]:
    return PLANE_DIMS if space == "frontal" else SPACE_DIMS


def day_seed(subject: str, date: Optional[_dt.date] = None) -> int:
    """Today's environment: the same for one monkey all day, new each day (YYYYMMDD x 1000 + monkey)."""
    d = date or _dt.date.today()
    return int(d.strftime("%Y%m%d")) * 1000 + zlib.crc32(str(subject or "").encode()) % 1000


# ---------------------------------------------------------------- the arena
def inside(p: Sequence[float], shape: str, lo: Sequence[float], hi: Sequence[float], dims, margin: float = 0.0) -> bool:
    """Is the point p (offset from the start, base frame) inside the arena by at least `margin`? For the
    ellipse the margin shrinks every semi-axis (close enough for zones well inside the walls)."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    p = np.asarray(p, float)
    if shape == "ellipse":
        c, r = (lo + hi) / 2, (hi - lo) / 2 - margin
        if any(r[i] <= 0 for i in dims):
            return False
        return sum(((p[i] - c[i]) / r[i]) ** 2 for i in dims) <= 1.0
    return all(lo[i] + margin <= p[i] <= hi[i] - margin for i in dims)


def clamp_ellipse(d: np.ndarray, lo, hi, dims) -> np.ndarray:
    """d (an offset from the start) kept inside the ellipse / ellipsoid inscribed in the box: pulled in
    toward the centre where it lies outside -- the arena's felt wall."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c, r = (lo + hi) / 2, (hi - lo) / 2
    q = sum(((d[i] - c[i]) / r[i]) ** 2 for i in dims if r[i] > 0)
    if q <= 1.0:
        return d
    out = np.array(d, dtype=float)
    k = 1.0 / math.sqrt(q)
    for i in dims:
        out[i] = c[i] + (d[i] - c[i]) * k
    return out


# ---------------------------------------------------------------- settings
@dataclass
class ForageSettings:
    space: str = "frontal"
    shape: str = "rectangle"
    lo: List[float] = field(default_factory=lambda: [-0.05, -0.15, -0.07])
    hi: List[float] = field(default_factory=lambda: [0.03, 0.15, 0.06])
    n_zones: int = 1
    zone_radius_m: float = 0.03
    min_gap_m: float = 0.02
    wall_margin_m: float = 0.005
    start_clear_m: float = 0.03
    seed: int = 0
    dwell_sec: float = 0.0
    probe_sec: float = 20.0
    warm_mode: str = "schedule"
    warm_start_factor: float = 2.5
    warm_shrink_trials: int = 5
    warm_fade_trials: int = 5
    probe_probability: float = 0.0
    probe_after_trial: int = 10


_FLOATS = {"forage.zone_radius_m": "zone_radius_m", "forage.min_gap_m": "min_gap_m",
           "forage.wall_margin_m": "wall_margin_m", "forage.start_clear_m": "start_clear_m",
           "forage.dwell_sec": "dwell_sec", "forage.probe_sec": "probe_sec",
           "warm.start_factor": "warm_start_factor", "probe.probability": "probe_probability"}
_INTS = {"forage.n_zones": "n_zones", "forage.seed": "seed", "warm.shrink_trials": "warm_shrink_trials",
         "warm.fade_trials": "warm_fade_trials", "probe.after_trial": "probe_after_trial"}
BOX = tuple(f"free_move.box.{a}_{e}_m" for e in ("min", "max") for a in "xyz")
PARAMS = ("free_move.rail_axis", "free_move.box.shape", "warm.mode") + tuple(_FLOATS) + tuple(_INTS)


def settings_from(get: Callable[[str], object]) -> ForageSettings:
    """ForageSettings from parameter values (get(name) -> value). The box must be custom (its six values)."""
    s = ForageSettings(space=str(get("free_move.rail_axis")).strip().lower(),
                       shape=str(get("free_move.box.shape")).strip().lower(),
                       warm_mode=str(get("warm.mode")).strip().lower())
    s.lo = [float(get(f"free_move.box.{a}_min_m")) for a in "xyz"]
    s.hi = [float(get(f"free_move.box.{a}_max_m")) for a in "xyz"]
    for name, attr in _FLOATS.items():
        setattr(s, attr, float(get(name)))
    for name, attr in _INTS.items():
        setattr(s, attr, int(get(name)))
    return s


def settings_problem(s: ForageSettings) -> Optional[str]:
    if s.space not in SPACES:
        return f"foraging needs free_move.rail_axis {' or '.join(SPACES)} (the plane facing him, or 3-D)"
    if s.shape not in SHAPES:
        return f"free_move.box.shape must be one of {list(SHAPES)}, got {s.shape!r}"
    if s.warm_mode not in WARM_MODES:
        return f"warm.mode must be one of {list(WARM_MODES)}, got {s.warm_mode!r}"
    if s.n_zones < 1:
        return "forage.n_zones must be at least 1"
    if s.zone_radius_m <= 0:
        return "forage.zone_radius_m must be positive"
    if s.warm_start_factor < 1.0:
        return "warm.start_factor must be at least 1 (the warm zone is never smaller than the zone)"
    if not 0.0 <= s.probe_probability <= 1.0:
        return "probe.probability must be 0-1"
    if s.dwell_sec < 0 or s.probe_sec <= 0:
        return "forage.dwell_sec must not be negative and forage.probe_sec must be positive"
    return None


def zones_for(s: ForageSettings, seed: int, tries: int = 20000) -> List[np.ndarray]:
    """The environment's zone centres (offsets from the start) for this seed; raises ValueError with the
    reason if they cannot all be placed."""
    dims = dims_for(s.space)
    rng = random.Random(seed)
    lo, hi = np.asarray(s.lo, float), np.asarray(s.hi, float)
    r = s.zone_radius_m
    zones: List[np.ndarray] = []
    for _ in range(tries):
        if len(zones) == s.n_zones:
            break
        p = np.zeros(3)
        for i in dims:
            p[i] = rng.uniform(lo[i], hi[i])
        if not inside(p, s.shape, lo, hi, dims, r + s.wall_margin_m):
            continue
        if np.linalg.norm(p[list(dims)]) < r + s.start_clear_m:
            continue
        if any(np.linalg.norm((p - z)[list(dims)]) < 2 * r + s.min_gap_m for z in zones):
            continue
        zones.append(p)
    if len(zones) < s.n_zones:
        raise ValueError(f"{s.n_zones} zone(s) of {r * 100:g} cm do not fit in this {s.shape} arena with the "
                         f"margins asked for: use fewer or smaller zones, smaller gaps, or a bigger arena")
    return zones


def params_problem(values: Dict[str, object], subject: str = "") -> Optional[str]:
    """None if a full set of foraging parameter values makes sense and today's zones fit; else the reason.
    What the dashboard checks before Start (the node checks the same)."""
    if "forage.n_zones" not in values:
        return None
    if str(values.get("free_move.box.preset", "custom")) != "custom":
        return "foraging's arena is the custom box (free_move.box.preset custom and its six values)"
    try:
        s = settings_from(lambda n: values[n])
    except (KeyError, TypeError, ValueError) as exc:
        return f"foraging settings: {exc}"
    problem = settings_problem(s)
    if problem:
        return problem
    try:
        zones_for(s, s.seed or day_seed(subject))
    except ValueError as exc:
        return str(exc)
    return None


# ---------------------------------------------------------------- the trial
def warm_state(s: ForageSettings, trial: int) -> Tuple[float, float]:
    """(warm radius as a factor of the zone radius, cue brightness 0-1) for trial 1, 2, ... of the session."""
    if s.warm_mode == "off":
        return 1.0, 0.0
    if s.warm_mode == "always":
        return s.warm_start_factor, 1.0
    n_s, n_f = max(0, s.warm_shrink_trials), max(0, s.warm_fade_trials)
    if trial <= n_s:
        k = (trial - 1) / max(1, n_s - 1) if n_s > 1 else 1.0
        return round(s.warm_start_factor + (1.0 - s.warm_start_factor) * k, 3), 1.0
    if trial <= n_s + n_f:
        return 1.0, round(1.0 - (trial - n_s) / (n_f + 1), 3)
    return 1.0, 0.0


@dataclass
class TrialPlan:
    trial: int
    seed: int
    zones: List[np.ndarray]
    warm_factor: float
    warm_brightness: float
    probe: bool

    def record(self, s: ForageSettings) -> dict:
        return {"seed": self.seed, "space": s.space, "shape": s.shape, "zone_radius_m": s.zone_radius_m,
                "zones": [[round(float(v), 4) for v in z] for z in self.zones], "trial": self.trial,
                "warm_factor": self.warm_factor, "warm_brightness": self.warm_brightness,
                "probe": self.probe, "dwell_sec": s.dwell_sec}


def plan_trial(s: ForageSettings, trial: int, seed: int, zones, rng: random.Random) -> TrialPlan:
    probe = trial >= s.probe_after_trial and s.probe_probability > 0 and rng.random() < s.probe_probability
    factor, bright = warm_state(s, trial)
    if probe:
        bright = 0.0
    return TrialPlan(trial, seed, list(zones), factor, bright, probe)


class ZoneTracker:
    """One trial's zones: which he is in, dwell, which have paid, revisits, whether the warm cue is on."""

    def __init__(self, s: ForageSettings, plan: TrialPlan) -> None:
        self._s, self.plan = s, plan
        self._dims = list(dims_for(s.space))
        self.found: List[int] = []                      # in the order they paid
        self._inside: Optional[int] = None
        self._since: Optional[float] = None
        self.revisits = 0

    def _dist(self, p, z) -> float:
        return float(np.linalg.norm((np.asarray(p, float) - z)[self._dims]))

    def warm(self, p) -> bool:
        """Is the hand within the warm radius of a zone not yet emptied this trial (and the cue on)?"""
        if self.plan.warm_brightness <= 0:
            return False
        r = self._s.zone_radius_m * self.plan.warm_factor
        return any(self._dist(p, z) <= r for i, z in enumerate(self.plan.zones) if i not in self.found)

    def step(self, p, now: float) -> List[Tuple[str, int]]:
        """Advance with the hand at p (offset from the start) at time `now` [s]; the events this caused:
        ("enter", i), ("zone", i) when zone i pays (after the dwell), ("revisit", i) entering an emptied one."""
        r = self._s.zone_radius_m
        here = next((i for i, z in enumerate(self.plan.zones) if self._dist(p, z) <= r), None)
        out = []
        if here != self._inside:
            self._inside, self._since = here, now
            if here is not None:
                out.append(("revisit", here) if here in self.found else ("enter", here))
                if here in self.found:
                    self.revisits += 1
        if here is not None and here not in self.found and not self.plan.probe \
                and now - self._since >= self._s.dwell_sec:
            self.found.append(here)
            out.append(("zone", here))
        return out

    def done(self) -> bool:
        return len(self.found) == len(self.plan.zones)
