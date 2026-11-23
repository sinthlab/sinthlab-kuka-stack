"""Free Move's box: where the equilibrium may go. Its ORIGIN IS THE START POSITION (the end effector at
the start pose): every bound is an offset from it [m], in the base frame -- x toward the subject, y
sideways, z up. Plain numpy, no ROS: the dashboard uses it too.

    free_move.box.preset   a named box below (each checked safe at the pre-training start), or custom
    free_move.box.x_min_m / x_max_m, y_min_m / y_max_m, z_min_m / z_max_m
                           the bounds when preset is custom: each min <= 0 <= max (the start inside it)
"""
from __future__ import annotations

from typing import Callable, Dict, Tuple

import numpy as np

# name -> ((x_min, y_min, z_min), (x_max, y_max, z_max)) [m] from the start. The arm reaches only ~3 cm
# up from the pre-training start but 12 cm down, and a corner combines directions (forward + sideways
# straightens the elbow, back + down takes A6 to its limit), so a longer X costs Y or Z.
PRESETS: Dict[str, Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = {
    "standard": ((-0.05, -0.18, -0.12), (0.03, 0.18, 0.01)),   # wide and deep: 8 x 36 x 13 cm
    "more_x":   ((-0.08, -0.14, -0.08), (0.06, 0.14, 0.01)),   # 14 x 28 x 9 cm
    "most_x":   ((-0.10, -0.10, -0.04), (0.08, 0.10, 0.00)),   # 18 x 20 x 4 cm
    "centred":  ((-0.04, -0.12, -0.02), (0.04, 0.12, 0.02)),   # 8 x 24 x 4 cm, the start in the middle
}
CUSTOM = "custom"
CHOICES = tuple(PRESETS) + (CUSTOM,)
_KEYS = ("x_min_m", "y_min_m", "z_min_m", "x_max_m", "y_max_m", "z_max_m")


def box_bounds(get: Callable[[str], object], prefix: str = "free_move.box.") -> Tuple[np.ndarray, np.ndarray]:
    """(min, max) offsets from the start [m], [x, y, z]. `get(name)` returns a parameter's value by its
    full name. Raises ValueError on a bad setting."""
    preset = str(get(prefix + "preset")).strip().lower()
    if preset in PRESETS:
        lo, hi = PRESETS[preset]
        return np.array(lo, dtype=float), np.array(hi, dtype=float)
    if preset != CUSTOM:
        raise ValueError(f"{prefix}preset must be one of {CHOICES}, got {preset!r}")
    v = [float(get(prefix + k)) for k in _KEYS]
    lo, hi = np.array(v[:3]), np.array(v[3:])
    if np.any(lo > 0.0) or np.any(hi < 0.0):
        raise ValueError(f"{prefix}*_min_m must be <= 0 and *_max_m >= 0 (the start, the box's origin, inside "
                         f"it), got min {lo.tolist()} / max {hi.tolist()}")
    return lo, hi
