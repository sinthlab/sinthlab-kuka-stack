"""Shapes the free-move admittance can hold the arm to, beyond one straight rail -- no ROS, unit-tested.

actions/admittance_move.py moves the cabinet spring's equilibrium from the hand's force, then clamps it to
the shape set by free_move.rail_axis:

    none        free in every direction (Free Move)
    x | y | z   one straight rail along that base axis through the start (Move Horizontal: y)
    frontal     free in the FRONTAL plane through the start -- base Y (right) and Z (up), the plane facing
                the monkey; no movement toward or away from him. The colour rule's "No rail" option.
    rays        an ASTERISK of straight rails from the start, in the frontal plane: free_move.rays_deg
                (0 = right, 90 = up, 180 = left, 270 = down) and free_move.ray_length_m. The N-choice task.

Offsets here are from the start (the anchor), in the base frame, in metres.
"""
from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np

SHAPES = ("none", "x", "y", "z", "frontal", "rays")
FRONTAL_NORMAL = np.array([1.0, 0.0, 0.0])     # base X: toward the monkey


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


def direction_name(deg: float) -> str:
    """right, up-right, up, ... for the 8 compass angles; else e.g. "deg30"."""
    names = {0: "right", 45: "up-right", 90: "up", 135: "up-left", 180: "left", 225: "down-left",
             270: "down", 315: "down-right"}
    a = int(round(float(deg))) % 360
    return names.get(a, f"deg{a}")
