"""Cued Reaches, Rails (switchable): one rail at a time, horizontal or vertical, switched between trials with
no restart -- no ROS, unit-tested.

Move Horizontal and Move Vertical start from different poses, and the CLIK reads its redundancy posture
(which must equal the start pose) once, at launch; so changing rail there means a new launch. The
switchable variant (experiment move_rails) runs both rails from ONE start pose -- Move Vertical's, which
N-Choice and Cued Reaches Free also use -- so only the rail changes:

    free_move.rail_axis: switch     the rail is rails.axis, and its ends come from rails.*
    rails.axis                      horizontal | vertical (live, from the next trial)
    rails.left_m, rails.right_m     the horizontal rail's ends, from the start [m] (per-run)
    rails.down_m, rails.up_m        the vertical rail's (per-run)

Both rails are checked against the arm's posture limits at start-up, so a switch never asks for one that
was not. The ends are named as on the fixed rails (left / right, down / up), and the ring quarters come
from centre_out.quarter_<end> for all four.
"""
from __future__ import annotations

from typing import Callable, Dict, Optional, Tuple

AXES = {"horizontal": "y", "vertical": "z"}          # rails.axis -> the rail's base axis
ENDS = {"y": ("left", "right"), "z": ("down", "up")}  # (rail_min end, rail_max end), as actions/centre_out.END_NAMES
LENGTHS = {"y": ("rails.left_m", "rails.right_m"), "z": ("rails.down_m", "rails.up_m")}
PARAMS = ("rails.axis",) + LENGTHS["y"] + LENGTHS["z"]
SWITCH = "switch"                                    # free_move.rail_axis for this variant


def base_axis(name: str) -> str:
    """"horizontal" -> "y", "vertical" -> "z"; raises ValueError for anything else."""
    key = str(name).strip().lower()
    if key not in AXES:
        raise ValueError(f"rails.axis must be one of {list(AXES)}, got {name!r}")
    return AXES[key]


def rail(axis: str, get: Callable[[str], object]) -> Tuple[float, float]:
    """(rail_min, rail_max) [m from the start] of the base axis "y" or "z", from rails.*: (-left, +right) or
    (-down, +up)."""
    lo_name, hi_name = LENGTHS[axis]
    return -abs(float(get(lo_name))), abs(float(get(hi_name)))


def current(get: Callable[[str], object]) -> Tuple[str, float, float]:
    """(base axis, rail_min, rail_max) of the rail in force (rails.axis)."""
    axis = base_axis(get("rails.axis"))
    return (axis,) + rail(axis, get)


def effective(values: Dict[str, object]) -> Optional[Tuple[str, float, float]]:
    """For the dashboard: (base axis, rail_min, rail_max) of whatever rail a task's values put in force --
    a fixed rail (rail_axis y / z) or this variant's -- or None (no rail)."""
    axis = str(values.get("free_move.rail_axis", "none")).strip().lower()
    try:
        if axis == SWITCH:
            return current(lambda n: values[n])
        if axis in ("x", "y", "z"):
            return axis, float(values["free_move.rail_min_m"]), float(values["free_move.rail_max_m"])
    except (KeyError, TypeError, ValueError):
        return None
    return None
