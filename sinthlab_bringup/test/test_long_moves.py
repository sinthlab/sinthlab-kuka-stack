"""Long-movement reward (helpers/long_moves.py, actions/long_move.py).

    python3 -m pytest sinthlab_bringup/test
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import test_centre_out_monitor as stubs  # noqa: E402,F401  (installs the ROS stand-ins where ROS is missing)
from sinthlab_bringup.helpers import live_params  # noqa: E402
from sinthlab_bringup.helpers.long_moves import LongMoveRule, Movement, MovementSegmenter  # noqa: E402
from sinthlab_bringup.actions.long_move import LongMoveMonitor  # noqa: E402

DT = 0.02   # the monitor's 50 Hz


def reach(seg, t0, start, end, speed=0.2, rest_after=0.6, wobble=0.0):
    """Feed a minimum-jerk movement start -> end, then a rest; returns the events and the end time."""
    start, end = np.asarray(start, float), np.asarray(end, float)
    dur = max(float(np.linalg.norm(end - start)) / speed * 1.875, 0.1)    # min-jerk peak = 1.875 x mean
    events, t = [], t0
    for k in range(int(dur / DT) + 1):
        s = min(1.0, k * DT / dur)
        s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
        side = wobble * math.sin(math.pi * s) * np.array([0.0, 0.0, 1.0])
        events.append(seg.update(t, start + s * (end - start) + side))
        t += DT
    for _ in range(int(rest_after / DT)):
        events.append(seg.update(t, end))
        t += DT
    return [e for e in events if e[0]], t


def bob(seg, t0, at, secs=3.0, amp=0.007, hz=1.6):
    """A resting hand: the arm bobs +-amp at hz."""
    at, events, t = np.asarray(at, float), [], t0
    for _ in range(int(secs / DT)):
        events.append(seg.update(t, at + np.array([0, 0, amp * math.sin(2 * math.pi * hz * (t - t0))])))
        t += DT
    return [e for e in events if e[0]], t


# ---------------------------------------------------------------- segmenting
def test_one_reach_is_one_movement_of_the_right_length():
    seg = MovementSegmenter()
    events, _ = reach(seg, 0.0, [0, 0, 0], [0, 0.15, 0])
    assert [k for k, _ in events] == ["start", "end"]
    m = events[-1][1]
    assert m.length_m == pytest.approx(0.15, abs=0.005)
    assert m.straightness > 0.97


def test_a_resting_hands_bob_is_not_a_movement():
    seg = MovementSegmenter()
    events, _ = bob(seg, 0.0, [0, 0, 0])
    assert events == []


def test_two_reaches_with_a_pause_are_two_movements():
    seg = MovementSegmenter()
    e1, t = reach(seg, 0.0, [0, 0, 0], [0, 0.1, 0])
    e2, _ = reach(seg, t, [0, 0.1, 0], [0, 0.1, -0.08])
    ends = [m for k, m in e1 + e2 if k == "end"]
    assert len(ends) == 2
    assert ends[1].length_m == pytest.approx(0.08, abs=0.005)    # measured from where the arm rested


def test_a_curved_path_is_less_straight():
    seg = MovementSegmenter()
    events, _ = reach(seg, 0.0, [0, 0, 0], [0, 0.15, 0], wobble=0.06)
    m = events[-1][1]
    assert m.length_m == pytest.approx(0.15, abs=0.005) and m.straightness < 0.9


def test_stop_speed_above_start_speed_is_refused():
    with pytest.raises(ValueError):
        MovementSegmenter(start_speed_mps=0.02, stop_speed_mps=0.03)


# ---------------------------------------------------------------- the reward rule
def mv(length, path=None, t_end=10.0):
    return Movement(t_end - 1.0, t_end, length, path or length, 0.2)


def test_rule_length_straightness_and_gap():
    r = LongMoveRule(min_m=0.10, min_straightness=0.8, min_gap_sec=1.0)
    assert not r.check(mv(0.08))[0]
    assert not r.check(mv(0.12, path=0.2))[0]                 # straightness 0.6
    assert r.check(mv(0.12, t_end=10.0))[0]
    assert not r.check(mv(0.15, t_end=10.5))[0]               # 0.5 s after the last reward
    assert r.check(mv(0.15, t_end=11.2))[0]


def test_straightness_zero_is_not_checked():
    assert LongMoveRule(min_m=0.10, min_straightness=0.0).check(mv(0.12, path=1.0))[0]


# ---------------------------------------------------------------- the monitor on a fake arm
class Arm:
    def __init__(self):
        self.p = np.zeros(3)

    def offset_from_anchor(self):
        return self.p


def monitor(**over):
    params = {"long_move.reward": "each_move", "long_move.min_m": 0.10, "long_move.min_straightness": 0.0,
              "long_move.min_gap_sec": 1.0, "long_move.start_speed_mps": 0.03, "long_move.stop_speed_mps": 0.02,
              "long_move.stop_sec": 0.3, "long_move.confirm_m": 0.02}
    params.update({"long_move." + k: v for k, v in over.items()})
    node, arm = stubs.FakeNode(params), Arm()
    return node, arm, LongMoveMonitor(node, arm)


def drive(node, arm, mon, path, clock):
    """Move the fake arm along `path` (positions), one 50 Hz tick each, on a fake clock."""
    for p in path:
        arm.p = np.asarray(p, float)
        clock[0] += DT
        for tick in node.timers:
            tick()


def straight(a, b, secs=1.0, rest=0.6):
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = int(secs / DT)
    s = [(lambda u: 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5)(k / n) for k in range(n + 1)]
    return [a + x * (b - a) for x in s] + [b] * int(rest / DT)


@pytest.fixture
def clock(monkeypatch):
    c = [100.0]
    import sinthlab_bringup.actions.long_move as lm
    monkeypatch.setattr(lm.time, "monotonic", lambda: c[0])
    return c


def test_monitor_reports_and_rewards_long_moves_only(clock):
    node, arm, mon = monitor()
    got = []
    mon.start(lambda tok, arg: got.append((tok, arg)))
    drive(node, arm, mon, [[0, 0, 0]] * 10, clock)
    drive(node, arm, mon, straight([0, 0, 0], [0, 0.06, 0]), clock)          # short: no reward
    drive(node, arm, mon, straight([0, 0.06, 0], [0, 0.06, -0.12]), clock)   # long: reward
    toks = [t for t, _ in got]
    assert toks == ["move_start", "move_end", "move_start", "move_end", "long_move"]
    assert got[-1][1] == pytest.approx(0.12, abs=0.005)
    assert mon.rewards == 1


def test_monitor_off_in_session_mode_and_after_stop(clock):
    node, arm, mon = monitor(reward="session")
    assert not mon.enabled
    node, arm, mon = monitor()
    got = []
    mon.start(lambda tok, arg: got.append(tok))
    mon.stop()
    drive(node, arm, mon, straight([0, 0, 0], [0, 0.15, 0]), clock)
    assert got == []


def test_live_change_applies_at_reload(clock):
    node, arm, mon = monitor()
    node.params["long_move.min_m"] = 0.20
    mon.reload()
    got = []
    mon.start(lambda tok, arg: got.append(tok))
    drive(node, arm, mon, straight([0, 0, 0], [0, 0.15, 0]), clock)
    assert "long_move" not in got


def test_validator_refuses_stop_above_start(clock):
    node, arm, mon = monitor()
    ctx = {"long_move.start_speed_mps": 0.03}
    assert mon.check_live("long_move.stop_speed_mps", 0.05, {**ctx, "long_move.stop_speed_mps": 0.05})
    assert mon.check_live("long_move.stop_speed_mps", 0.01, {**ctx, "long_move.stop_speed_mps": 0.01}) is None


# ---------------------------------------------------------------- the dashboard and the node agree
def test_live_registry():
    for name in ("long_move.reward", "long_move.min_m", "long_move.min_straightness"):
        assert live_params.is_live("free_move", name)
    assert live_params.check_value("long_move.reward", "each_move") is None
    assert live_params.check_value("long_move.reward", "sometimes") is not None
    assert live_params.check_value("long_move.min_m", 0.5) is not None


def test_free_move_card_has_the_tabs():
    experiments = pytest.importorskip("experiments")
    e = experiments.BY_KEY["free_move"]
    assert e.paradigm_param == "long_move.reward"
    assert [k for k, _ in e.paradigms] == ["session", "each_move"]
