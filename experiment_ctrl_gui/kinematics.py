"""The start pose from a height: the dashboard's "start height" field.

Given a height above the ground for the floor's reference point (floor.reference: apple bottom or the
base's lowest edge), solve the start pose that puts it there: the end effector is moved straight up or
down from the current start -- same horizontal position, same orientation -- by 6-D IK seeded at the
current start (helpers/start_grid.py). The result is refused if it is out of reach, near a joint limit
or singular, bends the elbow the other way, or if it -- or the move to it from park -- goes below the
floor.

Needs the iiwa7 URDF (built with xacro) and optas, so ROS must be sourced (run_gui.sh does). Without
them, available() is False and the dashboard hides the field with the reason.
"""
from __future__ import annotations

import os
import subprocess
import tempfile
from typing import Optional, Tuple

import numpy as np

import experiments as ex  # noqa: F401  (puts sinthlab_bringup on sys.path)
from sinthlab_bringup.helpers.floor import Floor, reference_z, setup_problem

_MODEL = None
_ERROR: Optional[str] = None


def _load():
    global _MODEL, _ERROR
    if _MODEL is not None or _ERROR is not None:
        return
    try:
        import optas
        from sinthlab_bringup.helpers.start_grid import StartGrid
        prefix = subprocess.run(["ros2", "pkg", "prefix", "lbr_description"], capture_output=True,
                                text=True, check=True, timeout=20).stdout.strip()
        urdf = os.path.join(tempfile.gettempdir(), "experiment_ctrl_gui_iiwa7.urdf")
        with open(urdf, "w") as f:
            subprocess.run(["xacro", os.path.join(prefix, "share/lbr_description/urdf/iiwa7/iiwa7.xacro")],
                           stdout=f, check=True, timeout=60)
        robot = optas.RobotModel(urdf_filename=urdf)
        _MODEL = (robot.get_global_link_transform_function("lbr_link_ee", numpy_output=True), StartGrid(robot))
    except Exception as exc:        # no ROS / optas / xacro: the field is hidden, nothing else breaks
        _ERROR = f"start height unavailable: {exc}"


def available() -> Tuple[bool, Optional[str]]:
    _load()
    return _MODEL is not None, _ERROR


def height_of(start_deg, reference: str) -> Optional[float]:
    """Height above the ground of the reference point at the start pose [m], or None without a model."""
    _load()
    if _MODEL is None:
        return None
    return reference_z(_MODEL[0](np.radians(np.asarray(start_deg, dtype=float))), reference)


# Edits that can move something relative to the floor: the dashboard re-checks the setup after them.
FLOOR_RELEVANT = ("floor.", "move_to_start.target_joint_position", "move_to_park.target_joint_position",
                  "apple_pluck_impedance_control_displacement.cartesian_displacement_threshold_m", "paradigm",
                  "random_start.", "perturb_start.polar_", "virtual_fixture", "free_move.rail_")


def floor_problem(current: dict) -> Optional[str]:
    """None if the configuration leaves the floor clear of the task (helpers/floor.setup_problem), else
    the reason. None also when there is no robot model (no ROS): the run still checks at start-up."""
    _load()
    if _MODEL is None or "floor.height_m" not in current:
        return None
    return setup_problem(current, _MODEL[0], _MODEL[1])


def start_for_height(height_m: float, current: dict) -> Tuple[list, str]:
    """(start pose A1..A7 [deg], a one-line report) that puts the floor's reference point at height_m.
    `current` is the experiment's parameter set in effect (dashboard edits included). Raises ValueError
    with the reason if it cannot be used."""
    _load()
    if _MODEL is None:
        raise ValueError(_ERROR or "start height unavailable")
    fk, grid = _MODEL
    floor = Floor(current["floor.height_m"], current["floor.reference"], current["floor.trip_margin_m"])
    start = current[ex.START_POSE]
    now = reference_z(fk(np.radians(np.asarray(start, dtype=float))), floor.reference)
    if height_m < floor.height:
        raise ValueError(f"{height_m:.3f} m is below the floor ({floor.height:.3f} m)")
    # Frontal plane: b is +Z, so [0, dz] moves the end effector straight up or down, orientation held.
    q, why = grid.solve(start, "frontal", {"start": [0.0, float(height_m - now)]})["start"]
    if why:
        raise ValueError(f"no usable start pose at {height_m:.3f} m: {why}")
    problem = setup_problem({**current, ex.START_POSE: q}, fk, grid)
    if problem:
        raise ValueError(problem)
    T = fk(np.radians(q))
    return q, (f"{floor.reference.replace('_', ' ')} at {reference_z(T, floor.reference):.3f} m "
               f"(was {now:.3f} m); flange at ({T[0, 3]:.3f}, {T[1, 3]:.3f}, {T[2, 3]:.3f}) m, "
               f"{grid.margin_deg(np.radians(q)):.0f} deg from the nearest joint limit")
