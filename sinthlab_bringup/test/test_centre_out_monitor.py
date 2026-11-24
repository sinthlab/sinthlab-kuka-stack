"""The centre-out monitor (actions/centre_out.py) and the admittance's per-trial rail clamp
(actions/admittance_move.py), driven by a fake node and a fake arm.

Runs with or without ROS: where rclpy, optas or the message packages are missing (a laptop), minimal
stand-ins are installed for the import only -- nothing here talks to ROS.

    python3 -m pytest sinthlab_bringup/test
"""
import importlib
import sys
import types
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))


def _stub(name, **attrs):
    if name in sys.modules:
        return
    try:
        importlib.import_module(name)
        return
    except ImportError:
        pass
    mod = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(mod, k, v)
    sys.modules[name] = mod


_stub("rclpy")
_stub("rclpy.node", Node=object)
_stub("optas")
_stub("geometry_msgs")
_stub("geometry_msgs.msg", PoseStamped=object)
_stub("lbr_fri_idl")
_stub("lbr_fri_idl.msg", LBRState=object)

from sinthlab_bringup.actions.admittance_move import AdmittanceMoveAction  # noqa: E402
from sinthlab_bringup.actions.centre_out import CentreOutMonitor  # noqa: E402


class _Param:
    def __init__(self, value):
        self.value = value


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(("info", m))

    def warn(self, m):
        self.lines.append(("warn", m))

    def error(self, m):
        self.lines.append(("error", m))


class FakeNode:
    def __init__(self, params):
        self.params = dict(params)
        self.log = _Log()
        self.timers = []

    def has_parameter(self, n):
        return n in self.params

    def get_parameter(self, n):
        return _Param(self.params[n])

    def get_logger(self):
        return self.log

    def create_timer(self, period, fn):
        self.timers.append(fn)


class FakeArm:
    """The admittance as the monitor sees it: a rail and where the arm is along it."""

    def __init__(self, axis, ends):
        self.axis, self.ends, self.x = axis, ends, 0.0

    def rail_axis(self):
        return self.axis

    def rail_ends(self):
        return self.ends

    def rail_position(self):
        return self.x


def horizontal(**over):
    params = {"centre_out.enabled": True, "centre_out.target": "either", "centre_out.block_other_end": True,
              "centre_out.max_repeats": 3, "centre_out.end_tol_m": 0.02,
              "centre_out.quarter_left": 3, "centre_out.quarter_right": 1}
    params.update({"centre_out." + k: v for k, v in over.items()})
    node, arm = FakeNode(params), FakeArm("y", (-0.15, 0.15))
    return node, arm, CentreOutMonitor(node, arm)


def run_to(node, arm, mon, x):
    """Move the arm to x along the rail and tick the monitor; the outcome, or None."""
    got = []
    mon.start(lambda end, rewarded: got.append((end, rewarded)))
    arm.x = x
    for tick in node.timers:
        tick()
    return got[0] if got else None


# ---------------------------------------------------------------- one-direction trials
def test_left_only_lights_left_and_closes_right():
    node, arm, mon = horizontal(target="left")
    assert mon.pick_target() == "left"
    assert mon.quarters() == [3]
    assert mon.open_span() == (-0.15, 0.0)
    assert mon.closed_end() == "right"
    assert run_to(node, arm, mon, -0.14) == ("left", True)


def test_unblocked_wrong_end_is_not_rewarded():
    node, arm, mon = horizontal(target="left", block_other_end=False)
    mon.pick_target()
    assert mon.open_span() is None and mon.closed_end() is None
    assert run_to(node, arm, mon, 0.14) == ("right", False)


def test_either_keeps_the_whole_rail_and_rewards_both():
    node, arm, mon = horizontal(target="either")
    assert mon.pick_target() == "either"
    assert mon.quarters() == [0, 1, 2, 3]          # the whole ring: no direction cued
    assert mon.open_span() is None
    assert run_to(node, arm, mon, 0.15) == ("right", True)


def test_either_turns_the_whole_ring_red_and_one_end_modes_their_quarter():
    node, arm, mon = horizontal(target="either")
    mon.pick_target()
    assert mon.reached_quarters("left") == [0, 1, 2, 3] == mon.reached_quarters("right")
    node, arm, mon = horizontal(target="cued")
    goal = mon.pick_target()
    assert mon.quarters() == [mon.quarter_of(goal)]
    assert mon.reached_quarters("left") == [3] and mon.reached_quarters("right") == [1]
    node, arm, mon = horizontal(target="right")
    mon.pick_target()
    assert mon.quarters() == [1] and mon.reached_quarters("right") == [1]


def test_no_outcome_short_of_an_end():
    node, arm, mon = horizontal(target="right")
    mon.pick_target()
    assert run_to(node, arm, mon, 0.10) is None


# ---------------------------------------------------------------- live switching
def test_live_switch_between_trials():
    node, arm, mon = horizontal(target="left")
    assert mon.pick_target() == "left"
    mon.stop()
    node.params["centre_out.target"] = "right"
    mon.reload()                                   # the trial boundary
    assert mon.pick_target() == "right"
    assert mon.open_span() == (0.0, 0.15)


def test_validator_refuses_an_end_this_rail_lacks():
    node, arm, mon = horizontal()
    assert mon.check_live("centre_out.target", "up", {}) is not None
    assert mon.check_live("centre_out.target", "right", {}) is None
    assert mon.check_live("centre_out.end_tol_m", 0.01, {}) is None    # not its business


def test_bad_value_at_reload_keeps_the_previous_mode():
    node, arm, mon = horizontal(target="left")
    node.params["centre_out.target"] = "up"       # slipped past the validator
    mon.reload()
    assert mon.pick_target() == "left"
    assert any(level == "error" for level, _ in node.log.lines)


def test_bad_value_at_start_up_raises():
    with pytest.raises(ValueError):
        horizontal(target="up")


# ---------------------------------------------------------------- the admittance clamp
def _admittance_on_rail(lo=-0.15, hi=0.15):
    adm = AdmittanceMoveAction.__new__(AdmittanceMoveAction)    # no ROS: only the rail clamp
    adm._node = FakeNode({})
    adm._rail = (np.array([0.0, 1.0, 0.0]), lo, hi)
    adm._anchor = np.eye(4)
    adm._anchor[0:3, 3] = [0.38, 0.01, 1.03]
    adm._open = None
    return adm


def test_clamp_closes_the_other_half():
    adm = _admittance_on_rail()
    a = adm._anchor[0:3, 3]
    adm.set_open_span((-0.15, 0.0))                              # left only
    assert adm._on_rail(a + [0, 0.10, 0])[1] == pytest.approx(a[1])          # a push right goes nowhere
    assert adm._on_rail(a + [0, -0.10, 0])[1] == pytest.approx(a[1] - 0.10)  # left moves freely
    assert adm._on_rail(a + [0, -0.30, 0])[1] == pytest.approx(a[1] - 0.15)  # to the rail end
    assert adm.rail_ends() == (-0.15, 0.15)                     # the monitor still sees the whole rail
    adm.set_open_span(None)
    assert adm._on_rail(a + [0, 0.10, 0])[1] == pytest.approx(a[1] + 0.10)


def test_clamp_keeps_within_the_rail():
    adm = _admittance_on_rail()
    adm.set_open_span((-0.5, 0.0))                               # wider than the rail: trimmed to it
    assert adm._open == (-0.15, 0.0)
    with pytest.raises(ValueError):
        adm.set_open_span((0.2, 0.3))                            # outside it entirely
