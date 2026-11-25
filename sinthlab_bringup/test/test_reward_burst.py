"""Rewards of several pulses (reward_pulse.count / gap_ms): helpers/experiment_control.py sends them, and
trials.csv counts them.

    python3 -m pytest sinthlab_bringup/test
"""
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from test_centre_out_monitor import _stub  # noqa: E402  (ROS stand-ins where ROS is missing)

_stub("rclpy.qos", DurabilityPolicy=object, QoSProfile=object, ReliabilityPolicy=object)
_stub("rcl_interfaces")
_stub("rcl_interfaces.msg", SetParametersResult=object)
_stub("std_msgs")
_stub("std_msgs.msg", String=object)
_stub("std_srvs")
_stub("std_srvs.srv", Trigger=object)

from sinthlab_bringup.helpers import live_params, outcome_tables as ot  # noqa: E402
from sinthlab_bringup.helpers.experiment_control import ExperimentControl  # noqa: E402


class FakePulser:
    def __init__(self):
        self.sent = []

    def pulse(self, output, ms, token):
        self.sent.append((time.monotonic(), output, ms, token))


class Log:
    def info(self, *a, **k):
        pass


def control(reward):
    c = ExperimentControl.__new__(ExperimentControl)     # no ROS: only on_event
    c._nsp, c._nsp_enabled, c._log = FakePulser(), False, Log()
    c._reward, c._reward_events, c._trial = reward, ("reach",), 0
    c._sequence_rewards = False
    c._set_phase = lambda phase, arg=None: None
    return c


def test_a_single_pulse_reward():
    c = control((4, 120, 1, 300))
    assert c.on_event("reach", "left") == {"output": 4, "ms": 120, "count": 1, "gap_ms": 300}
    assert [s[1:3] for s in c._nsp.sent] == [(4, 120)]
    assert c.on_event("reach_wrong", "left") is None


def test_a_burst_is_spaced_by_width_plus_gap():
    c = control((4, 50, 3, 100))
    t0 = time.monotonic()
    r = c.on_event("reach", "left")
    assert r == {"output": 4, "ms": 50, "count": 3, "gap_ms": 100}
    time.sleep(0.45)
    times = [s[0] - t0 for s in c._nsp.sent]
    assert len(times) == 3 and all(s[2] == 50 for s in c._nsp.sent)
    assert times[1] == pytest.approx(0.15, abs=0.04) and times[2] == pytest.approx(0.30, abs=0.04)


def test_checkpoint_and_goal_pulses_do_not_overlap(monkeypatch):
    c = control((4, 500, 1, 300))
    c._reward_ready_at = 0.0
    c._reward_timers = []
    c.set_reward_events(("checkpoint", "goal"), sequence=True)
    now = [10.0]
    monkeypatch.setattr(time, "monotonic", lambda: now[0])
    scheduled = []

    class Timer:
        def __init__(self, delay, callback, args):
            self.delay, self.callback, self.args = delay, callback, args

        def start(self):
            scheduled.append(self)

        def is_alive(self):
            return True

    monkeypatch.setattr("threading.Timer", Timer)
    c.on_event("checkpoint", 0)
    now[0] += 0.2
    second = c.on_event("checkpoint", 1)
    goal = c.on_event("goal")
    assert [s[-1] for s in c._nsp.sent] == ["reward on checkpoint"]
    assert second["delay_ms"] == 600.0
    assert goal["delay_ms"] == 1400.0
    assert [timer.delay for timer in scheduled] == pytest.approx([0.6, 1.4])
    for timer in scheduled:
        timer.callback(*timer.args)
    assert [s[-1] for s in c._nsp.sent] == [
        "reward on checkpoint", "reward on checkpoint", "reward on goal",
    ]


def test_trials_csv_counts_every_pulse():
    meta = {"experiment": "move_horizontal", "trial_index": 1,
            "events": [{"event": "reach", "arg": "left", "t_wall": 1.0}],
            "rewards": [{"output": 4, "ms": 50, "count": 3, "gap_ms": 300}]}
    row = ot.trial_row(meta)
    assert (row["n_rewards"], row["reward_pulses"], row["reward_ms"]) == (1, 3, 150)
    old = dict(meta, rewards=[{"output": 4, "ms": 500}])                    # recorded before bursts existed
    assert (ot.trial_row(old)["reward_pulses"], ot.trial_row(old)["reward_ms"]) == (1, 500)


def test_burst_settings_are_live_with_limits():
    assert live_params.is_live("colour_rule", "reward_pulse.count")
    assert live_params.is_live("maze", "reward_pulse.gap_ms")
    assert live_params.check_value("reward_pulse.count", 3) is None
    assert live_params.check_value("reward_pulse.count", 0) is not None
    assert live_params.check_value("reward_pulse.gap_ms", 50) is not None


def test_every_config_has_the_burst_settings():
    import yaml
    for f in sorted((ROOT / "sinthlab_bringup" / "config").glob("*.yaml")):
        doc = yaml.safe_load(f.read_text()) or {}
        for node in doc.values():
            rp = ((node or {}).get("ros__parameters") or {}).get("reward_pulse")
            if rp is not None:
                assert rp.get("count") == 1 and rp.get("gap_ms") == 300, f.name
