"""The quiet-window staircase (helpers/quiet_ramp.py), and ExperimentControl running it from trial events.

    python3 -m pytest sinthlab_bringup/test
"""
import sys
import types
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from sinthlab_bringup.helpers import live_params as lp  # noqa: E402
from sinthlab_bringup.helpers import quiet_ramp as qr  # noqa: E402

ON = qr.RampSettings(enabled=True, step_sec=0.1, after_trials=3, back_sec=0.1, min_sec=0.3, max_sec=0.8)
CONFIGS = sorted(p for p in (ROOT / "sinthlab_bringup" / "config").glob("*.yaml") if "move_to_park:" in p.read_text())


def run(trials, start=0.5, s=ON):
    """trials: 'g' reached the go, 'e' early touch, '-' neither. The window after each."""
    w, streak, out = start, 0, []
    for t in trials:
        w, streak, _ = qr.step(w, t == "g", t == "e", streak, s)
        out.append(w)
    return out


def test_off_changes_nothing():
    assert run("ggggggeee", s=qr.RampSettings()) == [0.5] * 9


def test_three_good_trials_add_a_step():
    assert run("gggggg") == [0.5, 0.5, 0.6, 0.6, 0.6, 0.7]


def test_an_early_touch_steps_back_and_restarts_the_streak():
    assert run("ggeggg") == [0.5, 0.5, 0.4, 0.4, 0.4, 0.5]


def test_other_trials_neither_count_nor_break_the_streak():
    assert run("g-g-g") == [0.5, 0.5, 0.5, 0.5, 0.6]


def test_stays_within_min_and_max():
    assert run("eeeee")[-1] == 0.3
    assert run("g" * 30)[-1] == 0.8
    assert run("gggg", start=0.8) == [0.8] * 4
    # min above max is read as the range between them, not an error
    assert run("e", s=qr.RampSettings(enabled=True, min_sec=1.0, max_sec=0.5), start=0.55) == [0.5]


@pytest.mark.parametrize("path", CONFIGS, ids=lambda p: p.name)
def test_every_task_starts_short_with_the_staircase_off(path):
    p = next(iter(yaml.safe_load(path.read_text()).values()))["ros__parameters"]
    assert p["quiet_window_sec"] == 0.5
    s = qr.settings_from(lambda n: p["quiet_ramp"][n.split(".")[1]])
    assert s.enabled is False and s.min_sec <= p["quiet_window_sec"] <= s.max_sec
    for name in qr.PARAMS:
        assert lp.check_value(f"quiet_ramp.{name}", p["quiet_ramp"][name]) is None


# ---------------------------------------------------------------- ExperimentControl runs it
import test_reward_burst  # noqa: E402,F401  (ROS stand-ins for experiment_control)
from sinthlab_bringup.helpers.experiment_control import ExperimentControl  # noqa: E402


class _Result:
    successful, reason = True, ""


class _Node:
    def __init__(self, **params):
        self.params, self.set = params, []

    def has_parameter(self, n):
        return n in self.params

    def get_parameter(self, n):
        return types.SimpleNamespace(value=self.params[n])

    def set_parameters(self, ps):
        for p in ps:
            self.params[p.name] = p.value
            self.set.append((p.name, p.value))
        return [_Result()]


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    warn = info


def control(enabled=True):
    sys.modules.setdefault("rclpy.parameter", types.ModuleType("rclpy.parameter"))
    if not hasattr(sys.modules["rclpy.parameter"], "Parameter"):
        sys.modules["rclpy.parameter"].Parameter = lambda name, value: types.SimpleNamespace(name=name, value=value)
    c = ExperimentControl.__new__(ExperimentControl)        # no ROS: only on_event
    c._nsp_enabled, c._reward, c._trial, c._log = False, None, 0, _Log()
    c._set_phase = lambda phase, arg=None: None
    c._node = _Node(**{"quiet_window_sec": 0.5, **{f"quiet_ramp.{k}": v for k, v in {
        "enabled": enabled, "step_sec": 0.1, "after_trials": 2, "back_sec": 0.1, "min_sec": 0.3, "max_sec": 2.0}.items()}})
    return c


def trial(c, n, *events):
    for token in ("trial_start", *events, "trial_end"):
        c.on_event(token, n if token in ("trial_start", "trial_end") else None)


def test_control_steps_the_live_quiet_window_after_each_trial():
    c = control()
    trial(c, 1, "cue_preview", "cue_go", "reach")
    assert c._node.set == []
    trial(c, 2, "cue_preview", "cue_go", "reach")
    assert c._node.set == [("quiet_window_sec", 0.6)]           # two in a row reached the go
    trial(c, 3, "cue_preview", "early_touch")
    assert c._node.set[-1] == ("quiet_window_sec", 0.5)
    trial(c, 4, "stop_requested")                               # neither: nothing
    assert len(c._node.set) == 2
    assert any("staircase" in line for line in c._log.lines)


def test_control_leaves_the_window_alone_when_off_or_absent():
    c = control(enabled=False)
    for n in range(1, 6):
        trial(c, n, "cue_go")
    assert c._node.set == []
    c = control()
    del c._node.params["quiet_ramp.enabled"]                    # an older YAML without the block
    for n in range(1, 6):
        trial(c, n, "cue_go")
    assert c._node.set == []
