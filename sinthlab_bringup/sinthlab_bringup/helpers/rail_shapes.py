"""Shapes the free-move admittance can hold the arm to, beyond one straight rail -- no ROS, unit-tested.

actions/admittance_move.py moves the cabinet spring's equilibrium from the hand's force, then clamps it to
the shape set by free_move.rail_axis:

    none        free in every direction (Free Move)
    x | y | z   one straight rail along that base axis through the start (Move Horizontal: y)
    frontal     free in the FRONTAL plane through the start -- base Y (right) and Z (up), the plane facing
                the monkey; no movement toward or away from him. The colour rule's "No rail" option.
    rays        an ASTERISK of straight rails from the start, in the frontal plane, free_move.ray_length_m
                long: free_move.ray_layout of them (2-8), EVENLY SPACED, so no two choices are closer than
                any others (layout_degs below). The N-choice task.

Offsets here are from the start (the anchor), in the base frame, in metres.
"""
from __future__ import annotations

from typing import Optional, Sequence, Tuple

import numpy as np

SHAPES = ("none", "x", "y", "z", "frontal", "rays", "switch")   # switch: helpers/switch_rails.py
FRONTAL_NORMAL = np.array([1.0, 0.0, 0.0])     # base X: toward the monkey

# How many choices: the N-Choice card's tabs (free_move.ray_layout, per run), to work up from 2 over sessions.
LAYOUT_COUNTS = (2, 3, 4, 5, 6, 7, 8)


def layout_degs(n) -> list:
    """The directions [deg] of n EVENLY SPACED rails, 360/n apart, so no two choices are closer than any
    others. Always mirror-symmetric left to right: an even number starts at right (2 = left/right, 4 = the
    cross, 8 = every compass direction), an odd number starts straight down (3 = down, up-right, up-left)."""
    try:
        n = int(str(n).strip())
    except ValueError:
        n = 0
    if n not in LAYOUT_COUNTS:
        raise ValueError(f"free_move.ray_layout must be one of {list(LAYOUT_COUNTS)} (how many choices), got {n!r}")
    first = 0.0 if n % 2 == 0 else 270.0
    return [round((first + k * 360.0 / n) % 360.0, 6) for k in range(n)]


def frontal_dir(deg: float) -> np.ndarray:
    """Unit vector in the frontal plane: 0 deg = right (+Y), 90 = up (+Z), as the monkey sees it."""
    r = np.radians(float(deg))
    return np.array([0.0, np.cos(r), np.sin(r)])


def ray_dirs(degs: Sequence[float]) -> np.ndarray:
    if len(degs) < 2:
        raise ValueError("an asterisk needs at least 2 rays")
    d = np.array([frontal_dir(a) for a in degs])
    for i in range(len(d)):
        for j in range(i + 1, len(d)):
            if float(d[i] @ d[j]) > np.cos(np.radians(10.0)):
                raise ValueError(f"rays {degs[i]} and {degs[j]} deg are less than 10 deg apart")
    return d


def on_plane(offset: np.ndarray, normal: np.ndarray = FRONTAL_NORMAL) -> np.ndarray:
    """The offset with its component along `normal` removed."""
    offset = np.asarray(offset, dtype=float)
    return offset - normal * float(offset @ normal)


def on_rays(offset: np.ndarray, dirs: np.ndarray, length: float) -> Tuple[np.ndarray, int, float]:
    """The nearest point of the asterisk to `offset`: (that point, which ray, how far along it [m]).

    Each ray is the segment from the start to `length` along its direction. Near the start every ray is
    close, so a push in some direction moves the point along the ray closest to that direction; once out
    along a ray, the point stays on it (a sideways push only slides it back toward the start)."""
    offset = np.asarray(offset, dtype=float)
    t = np.clip(dirs @ offset, 0.0, float(length))             # how far along each ray
    pts = dirs * t[:, None]
    i = int(np.argmin(np.linalg.norm(pts - offset, axis=1)))
    return pts[i], i, float(t[i])


def rays_problem(grid, start_deg, degs: Sequence[float], length: float) -> Optional[str]:
    """Check every ray with the same sampled posture guard as a straight rail."""
    if not np.isfinite(length) or length <= 0:
        return "rail length must be positive and finite"
    for a in degs:
        problem = grid.directed_rail_problem(start_deg, frontal_dir(a), 0.0, length)
        if problem:
            return f"{direction_name(a)}: {problem}"
    return None


def direction_name(deg: float) -> str:
    """right, up-right, up, ... for the 8 compass angles; else the angle, e.g. "30deg" (0 = right, 90 = up)."""
    names = {0: "right", 45: "up-right", 90: "up", 135: "up-left", 180: "left", 225: "down-left",
             270: "down", 315: "down-right"}
    a = int(round(float(deg))) % 360
    return names.get(a, f"{a}deg")
