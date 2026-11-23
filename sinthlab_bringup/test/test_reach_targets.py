"""Centre-out goals and one-direction trials (helpers/reach_targets.py), and how the dashboard offers them.

No ROS needed:  python3 -m pytest sinthlab_bringup/test
"""
import random
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))
sys.path.insert(0, str(ROOT / "experiment_ctrl_gui"))

from sinthlab_bringup.helpers import live_params  # noqa: E402
from sinthlab_bringup.helpers.reach_targets import (  # noqa: E402
    TargetPicker, check_mode, modes_for, open_span)

H = ("left", "right")       # Move Horizontal's ends (rail_min_m, rail_max_m)
V = ("down", "up")          # Move Vertical's
H_RAIL = (-0.15, 0.15)
V_RAIL = (-0.07, 0.07)


def longest_run(seq):
    best = run = 0
    prev = None
    for x in seq:
        run = run + 1 if x == prev else 1
        prev = x
        best = max(best, run)
    return best


# ---------------------------------------------------------------- modes
def test_modes_per_rail():
    assert modes_for(H) == ("either", "cued", "left", "right")
    assert modes_for(V) == ("either", "cued", "down", "up")
    assert check_mode("left", H) is None
    assert check_mode("left", V) is not None        # no left end on the vertical rail
    assert check_mode("sideways", H) is not None


def test_invalid_mode_raises_and_keeps_the_old_one():
    p = TargetPicker(H)
    p.set_mode("left")
    with pytest.raises(ValueError):
        p.set_mode("up")
    assert p.mode == "left"


# ---------------------------------------------------------------- picking
def test_either_and_one_end_only():
    p = TargetPicker(H, random.Random(0))
    assert {p.pick() for _ in range(50)} == {"either"}
    p.set_mode("left")
    assert {p.pick() for _ in range(50)} == {"left"}
    p.set_mode("right")
    assert {p.pick() for _ in range(50)} == {"right"}


@pytest.mark.parametrize("cap", [1, 2, 3, 5])
def test_cued_never_repeats_more_than_the_cap(cap):
    p = TargetPicker(V, random.Random(cap))
    p.set_mode("cued", max_repeats=cap)
    seq = [p.pick() for _ in range(5000)]
    assert longest_run(seq) <= cap
    assert set(seq) == set(V)
    assert 0.4 < seq.count("up") / len(seq) < 0.6


def test_cued_without_a_cap_has_long_runs():
    p = TargetPicker(H, random.Random(1))
    p.set_mode("cued", max_repeats=0)
    assert longest_run([p.pick() for _ in range(5000)]) > 5


def test_cap_of_one_alternates():
    p = TargetPicker(H, random.Random(2))
    p.set_mode("cued", max_repeats=1)
    seq = [p.pick() for _ in range(20)]
    assert all(a != b for a, b in zip(seq, seq[1:]))


def test_changing_mode_restarts_the_run_count():
    p = TargetPicker(H, random.Random(3))
    p.set_mode("cued", max_repeats=1)
    first = p.pick()
    p.set_mode(first)                  # one-direction trials to the same end...
    for _ in range(5):
        assert p.pick() == first
    p.set_mode("cued", max_repeats=1)  # ...do not count toward the cued run afterwards
    assert p._run_len == 0 and p._run_end is None


def test_same_mode_keeps_the_run_count():
    p = TargetPicker(H, random.Random(4))
    p.set_mode("cued", max_repeats=2)
    seq = []
    for _ in range(200):               # a live reload between every trial, same values
        p.set_mode("cued", max_repeats=2)
        seq.append(p.pick())
    assert longest_run(seq) <= 2


# ---------------------------------------------------------------- the open part of the rail
def test_open_span_closes_the_other_half():
    assert open_span("left", H, H_RAIL, True) == (-0.15, 0.0)
    assert open_span("right", H, H_RAIL, True) == (0.0, 0.15)
    assert open_span("down", V, V_RAIL, True) == (-0.07, 0.0)
    assert open_span("up", V, V_RAIL, True) == (0.0, 0.07)


def test_open_span_whole_rail():
    assert open_span("either", H, H_RAIL, True) is None      # a free choice: both ends open
    assert open_span("left", H, H_RAIL, False) is None       # blocking off


def test_open_span_stays_on_a_rail_not_through_the_start():
    lo, hi = open_span("up", V, (0.02, 0.10), True)
    assert 0.02 <= lo <= hi <= 0.10


# ---------------------------------------------------------------- the dashboard and the node agree
def test_live_registry_accepts_the_new_settings():
    for mode in ("either", "cued", "left", "right", "down", "up"):
        assert live_params.check_value("centre_out.target", mode) is None
    assert live_params.check_value("centre_out.target", "sideways") is not None
    assert live_params.check_value("centre_out.max_repeats", 3) is None
    assert live_params.check_value("centre_out.max_repeats", 50) is not None
    for name in ("centre_out.target", "centre_out.block_other_end", "centre_out.max_repeats"):
        assert live_params.is_live("move_horizontal", name)
        assert live_params.is_live("move_vertical", name)
    assert live_params.paradigm_of("centre_out.max_repeats") == ("cued",)


def test_dashboard_tabs_match_each_rail():
    experiments = pytest.importorskip("experiments")
    tabs = {k: [m for m, _ in experiments.BY_KEY[k].paradigms] for k in ("move_horizontal", "move_vertical")}
    assert tuple(tabs["move_horizontal"]) == modes_for(H)
    assert tuple(tabs["move_vertical"]) == modes_for(V)
