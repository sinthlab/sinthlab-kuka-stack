"""N-choice with depleting rewards: helpers/bandit_schedule.py, actions/n_choice.py on a fake asterisk, its
trials.csv / sessions.csv columns, and its card.

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

import test_centre_out_monitor as stubs  # noqa: E402  (ROS stand-ins)
from sinthlab_bringup.helpers import live_params, outcome_tables as ot  # noqa: E402
from sinthlab_bringup.helpers.bandit_schedule import DepletingBandit  # noqa: E402
from sinthlab_bringup.actions.n_choice import NChoiceMonitor  # noqa: E402

DIRS = ["right", "up", "left", "down"]


# ---------------------------------------------------------------- the schedule
def test_choosing_depletes_and_the_rest_recover():
    b = DepletingBandit(DIRS, p_max=0.9, p_min=0.1, deplete=0.5, recover=0.5, rng=random.Random(0))
    b.choose("up")
    assert b.probabilities()["up"] == pytest.approx(0.5)          # halfway from 0.9 to 0.1
    assert b.probabilities()["left"] == pytest.approx(0.9)        # already at p_max
    b.choose("left")
    assert b.probabilities()["up"] == pytest.approx(0.7)          # halfway back up from 0.5
    assert b.best() == ["right", "down"]


def test_repeated_choice_falls_to_p_min_and_recovers_to_p_max():
    b = DepletingBandit(DIRS, p_max=0.8, p_min=0.2, deplete=0.3, recover=0.2, rng=random.Random(1))
    for _ in range(60):
        b.choose("right")
    assert b.probabilities()["right"] == pytest.approx(0.2, abs=1e-6)
    for _ in range(60):
        b.choose("left")
    assert b.probabilities()["right"] == pytest.approx(0.8, abs=1e-4)


def test_reward_follows_the_probability():
    b = DepletingBandit(DIRS, p_max=0.7, p_min=0.7, deplete=0.0, recover=0.0, rng=random.Random(2))
    paid = sum(b.choose("down") for _ in range(4000))
    assert 0.66 < paid / 4000 < 0.74


def test_random_start_and_live_range():
    b = DepletingBandit(DIRS, p_max=0.9, p_min=0.1, start="random", rng=random.Random(3))
    assert all(0.1 <= p <= 0.9 for p in b.p) and len(set(b.p)) == 4
    b.configure(p_max=0.5, p_min=0.3, deplete=0.3, recover=0.1)
    assert all(0.3 <= p <= 0.5 for p in b.p)
    with pytest.raises(ValueError):
        b.configure(p_max=0.2, p_min=0.5, deplete=0.3, recover=0.1)
    with pytest.raises(ValueError):
        DepletingBandit(DIRS, start="half")


# ---------------------------------------------------------------- the monitor on a fake asterisk
class Asterisk:
    def __init__(self, degs=(0, 90, 180, 270), length=0.07):
        self.degs, self.length, self.pos = list(degs), length, (0, 0.0)

    def rays(self):
        return self.degs, self.length

    def ray_position(self):
        return self.pos


def monitor(**over):
    params = {"n_choice.end_tol_m": 0.01, "n_choice.p_max": 1.0, "n_choice.p_min": 0.0,
              "n_choice.deplete": 1.0, "n_choice.recover": 1.0, "n_choice.start": "full"}
    params.update({"n_choice." + k: v for k, v in over.items()})
    node, ast = stubs.FakeNode(params), Asterisk()
    return node, ast, NChoiceMonitor(node, ast)


def tick(node):
    for t in node.timers:
        t()


def test_a_choice_is_reaching_a_rails_end():
    node, ast, mon = monitor()
    assert mon.names == ["right", "up", "left", "down"]
    got = []
    mon.start(lambda d, ok: got.append((d, ok)))
    ast.pos = (2, 0.05)                                    # out along left, not at the end
    tick(node)
    assert got == []
    ast.pos = (2, 0.065)                                   # within end_tol of the end
    tick(node)
    assert got == [("left", True)]                         # p was 1.0
    tick(node)
    assert len(got) == 1                                   # once per trial


def test_state_before_the_choice_and_depletion_after():
    node, ast, mon = monitor()
    assert mon.state()["p"] == {"right": 1.0, "up": 1.0, "left": 1.0, "down": 1.0}
    mon.start(lambda d, ok: None)
    ast.pos = (1, 0.07)
    tick(node)
    assert mon.state()["p"]["up"] == 0.0 and mon.state()["best"] == ["right", "left", "down"]


def test_needs_an_asterisk():
    node = stubs.FakeNode({"n_choice.p_max": 0.9})
    with pytest.raises(ValueError):
        NChoiceMonitor(node, Asterisk(degs=[]))


def test_validator():
    node, ast, mon = monitor(p_max=0.9, p_min=0.1)
    assert mon.check_live("n_choice.p_min", 0.95, {"n_choice.p_min": 0.95}) is not None
    assert mon.check_live("n_choice.p_min", 0.2, {"n_choice.p_min": 0.2}) is None


# ---------------------------------------------------------------- trials.csv / sessions.csv
def trial(k, choice, paid, p, best):
    ev = lambda n, dt, a=None: {"event": n, "arg": a, "t_wall": 1e9 + 10 * k + dt}   # noqa: E731
    return {"experiment": "n_choice", "trial_index": k, "subject": "M1", "params": {},
            "n_choice": {"p": p, "best": best},
            "events": [ev("trial_start", 0, k), ev("cue_go", 3), ev("choice" if paid else "choice_unrewarded", 5, choice),
                       ev("trial_end", 5, k)],
            "rewards": [{"ms": 500}] if paid else []}


def test_n_choice_columns_and_session(tmp_path):
    p1 = {"right": 0.9, "up": 0.9, "left": 0.9, "down": 0.9}
    p2 = {"right": 0.66, "up": 0.9, "left": 0.9, "down": 0.9}
    metas = [trial(1, "right", True, p1, ["right", "up", "left", "down"]),
             trial(2, "right", False, p2, ["up", "left", "down"]),
             trial(3, "up", True, p2, ["up", "left", "down"])]
    row = ot.trial_row(metas[1])
    assert (row["choice"], row["rewarded"], row["p_chosen"], row["chose_best"], row["success"]) == ("right", 0, 0.66, 0, 1)
    assert row["p_values"].startswith("right=0.66; up=0.9")
    run = tmp_path / "expt_n"
    run.mkdir()
    for m in metas:
        (run / f"t{m['trial_index']}.meta.json").write_text(json.dumps(m))
        ot.append_trial(run, m)
    s = ot.session_row(run)
    assert (s["n_choices"], s["reward_rate"], s["pct_chose_best"], s["pct_switch"]) == (3, 66.7, 66.7, 50.0)
    assert s["choice_counts"] == "right=2; up=1"


# ---------------------------------------------------------------- the card, the config, the registry
def test_card_config_and_registry():
    experiments = pytest.importorskip("experiments")
    e = experiments.BY_KEY["n_choice"]
    assert e.group == "experiments" and e.launch_file == "iiwa7_n_choice.launch.py"
    import yaml
    d = yaml.safe_load((ROOT / "sinthlab_bringup/config/n_choice.yaml").read_text())["/**"]["ros__parameters"]
    assert d["experiment"] == "n_choice" and d["free_move"]["rail_axis"] == "rays"
    assert d["free_move"]["ray_layout"] == "2" and "centre_out" not in d     # start with 2 choices
    assert e.paradigm_param == "free_move.ray_layout"
    assert [k for k, _ in e.paradigms] == ["2", "3", "4", "5", "6", "7", "8"]
    assert not live_params.is_live("n_choice", "free_move.ray_layout")      # per session
    assert live_params.is_live("n_choice", "n_choice.deplete")
    assert not live_params.is_live("n_choice", "free_move.ray_length_m")     # the asterisk is per-run
    assert live_params.check_value("n_choice.start", "random") is None


def test_layouts_are_evenly_spaced_and_symmetric():
    from sinthlab_bringup.helpers.rail_shapes import LAYOUT_COUNTS, direction_name, layout_degs
    import numpy as np
    for n in LAYOUT_COUNTS:
        degs = layout_degs(n)
        gaps = np.diff(sorted(degs) + [sorted(degs)[0] + 360])
        assert len(degs) == n and np.allclose(gaps, 360 / n)          # no two closer than any others
        mirrored = sorted(round((180 - a) % 360, 3) for a in degs)  # left-right mirror image
        assert np.allclose(mirrored, sorted(round(a, 3) for a in degs))
    names = lambda n: [direction_name(a) for a in layout_degs(n)]   # noqa: E731
    assert names(2) == ["right", "left"]
    assert names(3) == ["down", "30deg", "150deg"]                  # down, up-right, up-left
    assert names(4) == ["right", "up", "left", "down"]
    assert len(set(names(8))) == 8
    assert layout_degs("3") == layout_degs(3)                       # a run file may carry it unquoted
    for bad in (1, 9, "custom"):
        with pytest.raises(ValueError):
            layout_degs(bad)


def test_monitor_names_follow_the_layout():
    node = stubs.FakeNode({"n_choice.end_tol_m": 0.01, "n_choice.p_max": 0.9, "n_choice.p_min": 0.1,
                           "n_choice.deplete": 0.3, "n_choice.recover": 0.1, "n_choice.start": "full"})
    mon = NChoiceMonitor(node, Asterisk(degs=[180, 270, 0]))
    assert mon.names == ["left", "down", "right"] and set(mon.state()["p"]) == {"left", "down", "right"}
