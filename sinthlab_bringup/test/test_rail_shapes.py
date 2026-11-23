"""Shapes the free-move admittance holds the arm to (helpers/rail_shapes.py), and the admittance's clamp
to them (actions/admittance_move.py).

    python3 -m pytest sinthlab_bringup/test
"""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import test_centre_out_monitor as stubs  # noqa: E402,F401  (ROS stand-ins)
from sinthlab_bringup.helpers import rail_shapes as rs  # noqa: E402
from sinthlab_bringup.actions.admittance_move import AdmittanceMoveAction  # noqa: E402

ASTERISK = [0, 45, 90, 135, 180, 225, 270, 315]


def test_frontal_directions():
    assert np.allclose(rs.frontal_dir(0), [0, 1, 0])        # right = +Y
    assert np.allclose(rs.frontal_dir(90), [0, 0, 1])       # up = +Z
    assert rs.direction_name(225) == "down-left" and rs.direction_name(30) == "30deg"


def test_rays_too_close_are_refused():
    with pytest.raises(ValueError):
        rs.ray_dirs([0, 5])
    with pytest.raises(ValueError):
        rs.ray_dirs([0])


def test_on_plane_drops_toward_and_away():
    assert np.allclose(rs.on_plane([0.05, 0.02, -0.03]), [0, 0.02, -0.03])


def test_a_push_slides_along_the_nearest_ray_and_stays_on_it():
    d = rs.ray_dirs(ASTERISK)
    p, i, t = rs.on_rays([0, 0.03, 0.005], d, 0.07)          # mostly right
    assert ASTERISK[i] == 0 and t == pytest.approx(0.03) and np.allclose(p, [0, 0.03, 0])
    p, i, t = rs.on_rays([0, 0.05, 0.02], d, 0.07)           # out on the right ray, pushed up a little
    assert ASTERISK[i] == 0                                 # still the right ray
    p, i, t = rs.on_rays(0.2 * rs.frontal_dir(135), d, 0.07)  # far up-left: clamped to the ray's end
    assert ASTERISK[i] == 135 and t == pytest.approx(0.07)


def test_the_start_is_on_every_ray():
    p, i, t = rs.on_rays([0, 0, 0], rs.ray_dirs(ASTERISK), 0.07)
    assert t == 0 and np.allclose(p, 0)


def _admittance(shape, degs=None, length=0.0):
    adm = AdmittanceMoveAction.__new__(AdmittanceMoveAction)
    adm._rail, adm._open, adm._shape = None, None, shape
    adm._plane_n = rs.FRONTAL_NORMAL if shape in ("frontal", "rays") else None
    adm._rays = rs.ray_dirs(degs) if degs else None
    adm._ray_degs, adm._ray_len = degs or [], length
    adm._anchor = np.eye(4)
    adm._anchor[0:3, 3] = [0.38, 0.01, 1.03]
    return adm


def test_admittance_clamps_to_the_frontal_plane():
    adm = _admittance("frontal")
    a = adm._anchor[0:3, 3]
    assert np.allclose(adm._on_rail(a + [0.04, 0.1, -0.05]), a + [0, 0.1, -0.05])


def test_admittance_clamps_to_the_asterisk():
    adm = _admittance("rays", ASTERISK, 0.07)
    a = adm._anchor[0:3, 3]
    out = adm._on_rail(a + [0.03, 0.0, -0.2])                # down, too far, and toward the monkey
    assert np.allclose(out, a + [0, 0, -0.07])
    assert adm.rays() == (ASTERISK, 0.07)


# ---------------------------------------------------------------- the floor check before the asterisk starts
def test_floor_check_covers_the_asterisks_lowest_end():
    from sinthlab_bringup.helpers import floor

    def fk(q):                                             # stand-in: the reference point at 1.00 m
        T = np.eye(4)
        T[2, 3] = 1.0 - floor.reference_drop(np.eye(3), "apple_bottom")
        return T

    params = {"floor.height_m": 0.88, "floor.reference": "apple_bottom", "floor.trip_margin_m": 0.03,
              "move_to_start.target_joint_position": [0, 0, 0, 0, 0, 0, 0],
              "free_move.rail_axis": "rays", "free_move.ray_layout": "8", "free_move.ray_length_m": 0.07}
    assert floor.setup_problem(params, fk) is None                         # lowest end at 0.93 m
    params["free_move.ray_length_m"] = 0.2
    problem = floor.setup_problem(params, fk)                              # ...at 0.80 m: below the floor
    assert problem and "the lowest rail's end" in problem
    params["free_move.ray_layout"] = "2"                                    # left / right: nothing goes down
    assert floor.setup_problem(params, fk) is None


def test_no_rail_is_the_frontal_plane_inside_the_box():
    adm = _admittance("frontal")
    adm._box = (np.array([-0.05, -0.18, -0.12]), np.array([0.03, 0.18, 0.01]))     # Navin's standard box
    adm._floor = type("NoFloor", (), {"clamp_xyz": staticmethod(lambda xyz, R: xyz)})()
    a = adm._anchor[0:3, 3]
    out = adm._bound(a + [0.02, 0.25, 0.06])          # toward him, too far right, too far up
    assert np.allclose(out, a + [0.0, 0.18, 0.01])    # in the plane, held at the box's right and top faces


def test_the_asterisk_has_no_box():
    adm = _admittance("rays", ASTERISK, 0.07)
    adm._box = None
    adm._floor = type("NoFloor", (), {"clamp_xyz": staticmethod(lambda xyz, R: xyz)})()
    a = adm._anchor[0:3, 3]
    assert np.allclose(adm._bound(a + [0, 0, 0.2]), a + [0, 0, 0.07])


# ---------------------------------------------------------------- every rail reachable before the asterisk runs
class FakeGrid:
    """StartGrid.solve stand-in: anything higher than `up_limit` above the start is out of reach."""
    def __init__(self, up_limit):
        self.up_limit, self.asked = up_limit, {}

    def solve(self, start, plane, offsets):
        assert plane == "frontal"
        self.asked.update(offsets)
        return {n: ([0] * 7, "out of reach" if ab[1] > self.up_limit else None) for n, ab in offsets.items()}


def test_rays_problem_checks_every_rails_end_and_middle():
    g = FakeGrid(up_limit=1.0)
    assert rs.rays_problem(g, [0] * 7, rs.layout_degs(4), 0.07) is None
    assert len(g.asked) == 8                                                 # 4 ends + 4 middles
    assert g.asked["the up rail's end"] == pytest.approx([0.0, 0.07])
    problem = rs.rays_problem(FakeGrid(up_limit=0.05), [0] * 7, rs.layout_degs(4), 0.07)
    assert problem == "the up rail's end (7 cm rails) is out of reach"
    assert rs.rays_problem(FakeGrid(up_limit=0.05), [0] * 7, rs.layout_degs(2), 0.07) is None   # left / right only


def test_dashboard_check_refuses_an_unreachable_asterisk():
    from sinthlab_bringup.helpers import floor

    def fk(q):
        T = np.eye(4)
        T[2, 3] = 1.0 - floor.reference_drop(np.eye(3), "apple_bottom")
        return T

    params = {"floor.height_m": 0.88, "floor.reference": "apple_bottom", "floor.trip_margin_m": 0.03,
              "move_to_start.target_joint_position": [0] * 7, "free_move.rail_axis": "rays",
              "free_move.ray_layout": "3", "free_move.ray_length_m": 0.07}
    problem = floor.setup_problem(params, fk, FakeGrid(up_limit=0.02))      # 3 = down, 30 deg, 150 deg: up 3.5 cm
    assert problem and problem.startswith("N-choice:") and "30deg" in problem
    assert floor.setup_problem(params, fk, FakeGrid(up_limit=0.05)) is None
