"""Colour rule with unsignaled reversals: helpers/rule_schedule.py, its place in the centre-out monitor
(actions/centre_out.py), and its columns in trials.csv / sessions.csv.

    python3 -m pytest sinthlab_bringup/test
"""
import json
import random
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import test_centre_out_monitor as stubs  # noqa: E402  (ROS stand-ins where ROS is missing)
from sinthlab_bringup.helpers import live_params, outcome_tables as ot  # noqa: E402
from sinthlab_bringup.helpers.rule_schedule import ReversalSchedule  # noqa: E402

H = ("left", "right")


def sched(**kw):
    args = dict(window=10, criterion_pct=80, min_trials=10, max_repeats=3, rng=random.Random(0))
    args.update(kw)
    return ReversalSchedule(H, "right", **args)


# ---------------------------------------------------------------- the mapping
def test_colours_mean_ends_and_flip():
    s = sched()
    assert (s.end_for("red"), s.end_for("blue")) == ("right", "left")
    assert s.mapping() == "red=right,blue=left"
    for _ in range(10):
        s.record(True)
    assert (s.end_for("red"), s.end_for("blue")) == ("left", "right")
    assert (s.block, s.reversals, s.choices_in_block) == (2, 1, 0)


def test_red_end_must_be_on_this_rail():
    with pytest.raises(ValueError):
        ReversalSchedule(("down", "up"), "right")


def test_colour_draws_are_capped():
    s = sched(max_repeats=2)
    seq = [s.draw_colour() for _ in range(2000)]
    run = best = 0
    for a, b in zip(seq, seq[1:]):
        run = run + 1 if a == b else 0
        best = max(best, run + 1)
    assert best <= 2 and set(seq) == {"red", "blue"}


# ---------------------------------------------------------------- the criterion
def test_no_reversal_before_min_trials_even_if_perfect():
    s = sched(window=5, min_trials=12)
    assert not any(s.record(True) for _ in range(11))
    assert s.record(True)                                  # the 12th choice


def test_criterion_is_over_the_window_not_the_block():
    s = sched(window=10, criterion_pct=80, min_trials=10)
    for _ in range(10):
        s.record(False)                                    # a bad start...
    assert s.block == 1
    flips = [s.record(True) for _ in range(8)]             # ...then 8 right: window = 8/10
    assert flips[-1] and s.block == 2


def test_no_choice_is_ignored():
    s = sched(window=4, min_trials=4)
    for c in (True, None, True, None, True):
        assert not s.record(c)
    assert s.choices_in_block == 3 and s.record(True)


def test_reversal_is_announced_once():
    s = sched(window=2, min_trials=2)
    s.record(True)
    s.record(True)
    assert s.take_reversal() and not s.take_reversal()


def test_live_settings_keep_the_block():
    s = sched()
    for _ in range(5):
        s.record(True)
    s.configure(window=20, criterion_pct=90, min_trials=30, max_repeats=3)
    assert (s.block, s.choices_in_block, len(s._recent)) == (1, 5, 5)


def test_a_learner_reverses_repeatedly():
    rng = random.Random(1)
    s = sched(window=20, criterion_pct=80, min_trials=20)
    for _ in range(600):
        s.draw_colour()
        p = min(0.95, 0.3 + 0.04 * s.choices_in_block)    # learns within each block
        s.record(rng.random() < p)
    assert s.reversals >= 5


# ---------------------------------------------------------------- the monitor in rule mode
@pytest.fixture
def rule_params():
    """A Move Horizontal monitor whose config has the rule block (criterion: 4 choices, 75 %)."""
    def make(**over):
        params = {"centre_out.enabled": True, "centre_out.target": "either", "centre_out.block_other_end": True,
                  "centre_out.max_repeats": 3, "centre_out.end_tol_m": 0.02,
                  "centre_out.quarter_left": 3, "centre_out.quarter_right": 1,
                  "rule.red_end": "right", "rule.window": 4, "rule.criterion_pct": 75.0,
                  "rule.min_trials": 4, "rule.max_repeats": 3}
        params.update({"centre_out." + k: v for k, v in over.items()})
        node, arm = stubs.FakeNode(params), stubs.FakeArm("y", (-0.15, 0.15))
        return node, arm, stubs.CentreOutMonitor(node, arm)
    return make


def test_rule_mode_lights_the_cue_and_keeps_the_rail_open(rule_params):
    node, arm, mon = rule_params(target="rule")
    target = mon.pick_target()
    colour = mon.cue_colour()
    assert target == ("right" if colour == "red" else "left")
    state, quarters = mon.ring_cue()
    assert state == f"rule_{colour}" and quarters == [0, 1, 2, 3]
    assert mon.open_span() is None and mon.closed_end() is None
    assert mon.rule_state()["mapping"] == "red=right,blue=left"


def test_rule_mode_reverses_after_criterion_and_marks_it_once(rule_params):
    node, arm, mon = rule_params(target="rule")
    for _ in range(4):                                     # four correct choices: criterion
        target = mon.pick_target()
        assert stubs.run_to(node, arm, mon, -0.14 if target == "left" else 0.14)[1] is True
        mon.stop()
    assert mon.take_reversal() == "red=left,blue=right"
    assert mon.take_reversal() is None
    mon.pick_target()
    assert mon.rule_state()["block"] == 2


def test_entering_the_rule_restarts_it(rule_params):
    node, arm, mon = rule_params(target="rule")
    for _ in range(4):
        t = mon.pick_target()
        stubs.run_to(node, arm, mon, -0.14 if t == "left" else 0.14)
        mon.stop()
    node.params["centre_out.target"] = "either"
    mon.reload()
    node.params["centre_out.target"] = "rule"
    mon.reload()
    mon.pick_target()
    assert mon.rule_state()["block"] == 1


def test_validator_refuses_a_red_end_this_rail_lacks(rule_params):
    node, arm, mon = rule_params(target="rule")
    assert mon.check_live("rule.red_end", "up", {}) is not None
    assert mon.check_live("rule.red_end", "left", {}) is None


def test_rule_needs_its_block():
    node, arm, mon = stubs.horizontal()                    # no rule block in this config
    assert mon.check_live("centre_out.target", "rule", {}) == "this experiment's YAML has no rule block"


def test_live_registry():
    assert live_params.check_value("centre_out.target", "rule") is None
    assert live_params.is_live("colour_rule", "rule.window")
    assert live_params.check_value("rule.criterion_pct", 40) is not None
    assert live_params.paradigm_of("rule.window") is None             # Rail and No rail alike
    assert live_params.paradigm_of("centre_out.free_reach_m") == ("frontal",)
    assert live_params.check_value("free_move.rail_axis", "frontal") is None


def test_colour_rule_is_its_own_task():
    experiments = pytest.importorskip("experiments")
    e = experiments.BY_KEY["colour_rule"]
    assert e.group == "experiments" and e.paradigm_param == "free_move.rail_axis"
    assert [k for k, _ in e.paradigms] == ["y", "frontal"]
    for key in ("move_horizontal", "move_vertical"):            # no longer a training tab
        assert "rule" not in [k for k, _ in experiments.BY_KEY[key].paradigms]
    assert "centre_out.target" in e.hidden                    # fixed to the rule, not shown
    assert not live_params.is_live("colour_rule", "centre_out.target")
    assert live_params.is_live("colour_rule", "centre_out.free_reach_m")
    assert live_params.is_live("colour_rule", "rule.criterion_pct")


def test_colour_rule_config_and_training_configs():
    import yaml
    cfg = ROOT / "sinthlab_bringup" / "config"
    d = yaml.safe_load((cfg / "colour_rule.yaml").read_text())["/**"]["ros__parameters"]
    assert d["experiment"] == "colour_rule" and d["centre_out"]["target"] == "rule"
    assert set(d["rule"]) == {"red_end", "window", "criterion_pct", "min_trials", "max_repeats"}
    assert {"rule_red", "rule_blue"} <= set(d["ring_cue"]["colours"])
    for f in ("pretrain_move_horizontal.yaml", "pretrain_move_vertical.yaml"):
        assert "rule" not in yaml.safe_load((cfg / f).read_text())["/**"]["ros__parameters"]


# ---------------------------------------------------------------- No rail: a choice in the frontal plane
class FreeArm:
    """The admittance with rail_axis frontal, as the monitor sees it."""
    def __init__(self):
        self.p = [0.0, 0.0, 0.0]

    def shape(self):
        return "frontal"

    def rail_axis(self):
        return None

    def rail_ends(self):
        return None

    def rest_offset_from_anchor(self):
        import numpy as np
        return np.array(self.p)


def test_no_rail_choice_is_reaching_far_enough_left_or_right(rule_params):
    node, _, _ = rule_params(target="rule")
    node.params["centre_out.free_reach_m"] = 0.12
    arm = FreeArm()
    mon = stubs.CentreOutMonitor(node, arm)
    assert mon.enabled and mon.open_span() is None
    target = mon.pick_target()
    got = []
    mon.start(lambda end, ok: got.append((end, ok)))
    arm.p = [0.0, 0.10, -0.05]                             # right and down, not far enough
    for tick in node.timers:
        tick()
    assert got == []
    arm.p = [0.0, 0.125, 0.03]                             # past 0.12 m to the right
    for tick in node.timers:
        tick()
    assert got == [("right", target == "right")]


# ---------------------------------------------------------------- trials.csv / sessions.csv
def rule_trial(k, colour, mapping, block, chose, target, reversal=False, choices=0):
    events = [{"event": "trial_start", "arg": k, "t_wall": 1e9 + 10 * k}, {"event": "cue_go", "t_wall": 1e9 + 10 * k + 3}]
    if reversal:
        events.append({"event": "reversal", "arg": mapping, "t_wall": 1e9 + 10 * k + 3})
    events += [{"event": "cue_colour", "arg": colour, "t_wall": 1e9 + 10 * k + 3},
               {"event": "target", "arg": target, "t_wall": 1e9 + 10 * k + 3},
               {"event": "reach" if chose == target else "reach_wrong", "arg": chose, "t_wall": 1e9 + 10 * k + 4},
               {"event": "trial_end", "arg": k, "t_wall": 1e9 + 10 * k + 4}]
    return {"experiment": "colour_rule", "trial_index": k, "subject": "M1",
            "params": {"centre_out.target": "rule", "free_move.rail_axis": "frontal"}, "events": events,
            "rule": {"mapping": mapping, "block": block, "choices_in_block": choices, "window_pct": None}}


def test_rule_columns_and_session_summary(tmp_path):
    a, b = "red=right,blue=left", "red=left,blue=right"
    metas = [rule_trial(1, "red", a, 1, "right", "right"), rule_trial(2, "blue", a, 1, "left", "left", choices=1),
             rule_trial(3, "red", b, 2, "right", "left", reversal=True), rule_trial(4, "red", b, 2, "left", "left", choices=1)]
    row = ot.trial_row(metas[2])
    assert (row["cue_colour"], row["mapping"], row["block"], row["reversal"], row["correct"]) == ("red", b, 2, 1, 0)
    assert row["paradigm"] == "no rail"
    assert ot.trial_row(metas[0])["reversal"] == 0
    run = tmp_path / "expt_x"
    run.mkdir()
    for m in metas:                                        # the sidecars, then their trials.csv
        (run / f"t{m['trial_index']}.meta.json").write_text(json.dumps(m))
        ot.append_trial(run, m)
    sess = ot.session_row(run)
    assert (sess["n_reversals"], sess["choices_to_criterion"], sess["pct_correct"]) == (1, 2.0, 75.0)
    assert sess["n_choices"] == 4
