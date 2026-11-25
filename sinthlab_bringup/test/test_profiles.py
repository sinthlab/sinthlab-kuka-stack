"""Monkey profiles (experiment_ctrl_gui/profiles.py): the settings that belong to the animal, applied to every
free-move task under today's edits; the reach as rail ends; Save to profile; the change log.

    python3 -m pytest sinthlab_bringup/test
"""
import json
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))
sys.path.insert(0, str(ROOT / "experiment_ctrl_gui"))

experiments = pytest.importorskip("experiments")
profiles = pytest.importorskip("profiles")
from params import ExperimentParams  # noqa: E402

FREE_MOVE_TASKS = [e for e in experiments.EXPERIMENTS if profiles.applies_to(e)]


def ex_labels():
    return experiments.LABELS


@pytest.fixture
def data(tmp_path, monkeypatch):
    monkeypatch.setattr(experiments, "DATA", tmp_path)
    return tmp_path


def prof(**changes):
    vals = profiles.defaults()
    vals.update(changes)
    return {"subject": "M1", "version": 1, "updated": "", "values": vals, "reward_by_task": {}}


def test_layers_cover_the_free_move_tasks_only():
    assert {e.key for e in FREE_MOVE_TASKS} == {
        "free_move", "restricted_move_horizontal", "restricted_move_vertical", "move_horizontal", "move_vertical",
        "move_rails", "colour_rule", "n_choice", "reach_map", "move_free", "forage"}
    for e in experiments.EXPERIMENTS:
        p = ExperimentParams(e)
        assert set(e.task_settings) <= set(p.defaults), e.key            # every task setting exists
        if profiles.applies_to(e):
            assert e.task_settings, e.key
            for f in profiles.FIELDS:                                     # every one-to-one field exists
                assert not f.param or f.param in p.defaults, (e.key, f.param)
            layers = {r["name"]: r["layer"] for r in p.describe()}
            assert not {n for n in e.task_settings if layers[n] != "task"}
        else:                                                         # Navin's: a Task tab, no profile
            assert e.task_settings and set(e.task_settings) <= set(ex_labels())


def test_the_default_profile_changes_no_task():
    """A new profile is the tasks' own defaults: applying it changes nothing anywhere."""
    for e in FREE_MOVE_TASKS:
        p = ExperimentParams(e)
        p.profile = prof()
        assert p.current() == p.defaults, e.key
        assert p.launch_overrides() == {}, e.key                          # so no run file either


def test_feel_and_pacing_carry_to_every_task():
    for e in FREE_MOVE_TASKS:
        p = ExperimentParams(e)
        p.profile = prof(**{"feel.damping_ns_per_m": 70.0, "pacing.response_timeout_sec": 20.0, "reward.pulse_ms": 311})
        cur = p.current()
        assert cur["free_move.damping_ns_per_m"] == 70.0, e.key
        assert cur["response_timeout_sec"] == 20.0 and cur["reward_pulse.pulse_ms"] == 311


def test_reach_sets_the_rail_ends():
    p_ = prof(**{"reach.left_m": 0.10, "reach.right_m": 0.12, "reach.down_m": 0.06, "reach.up_m": 0.05})
    want = {"restricted_move_horizontal": (-0.10, 0.12), "move_horizontal": (-0.10, 0.12), "colour_rule": (-0.10, 0.12),
            "restricted_move_vertical": (-0.06, 0.05), "move_vertical": (-0.06, 0.05)}
    for key, (lo, hi) in want.items():
        p = ExperimentParams(experiments.BY_KEY[key])
        p.profile = p_
        cur = p.current()
        assert (cur["free_move.rail_min_m"], cur["free_move.rail_max_m"]) == (lo, hi), key
    for key in ("free_move", "n_choice"):                                # no rail ends: nothing to set
        p = ExperimentParams(experiments.BY_KEY[key])
        p.profile = p_
        assert "free_move.rail_min_m" not in p.current()


def test_a_reach_beyond_the_task_limit_is_capped_with_a_note():
    p = ExperimentParams(experiments.BY_KEY["move_horizontal"])
    p.profile = prof(**{"reach.right_m": 0.30})
    over, notes, _ = p.overlay()
    lim = experiments.live_params.limits_for("free_move.rail_max_m")
    assert over["free_move.rail_max_m"] == lim[1]
    p.profile = prof(**{"reach.right_m": 0.29})
    over, notes, _ = p.overlay()
    assert over["free_move.rail_max_m"] == 0.29 and not notes


def test_choice_distances_stay_task_settings_with_a_note_past_the_reach():
    p = ExperimentParams(experiments.BY_KEY["n_choice"])
    p.profile = prof(**{"reach.up_m": 0.04})
    p.set_edit("free_move.ray_layout", "4")                               # right, up, left, down
    over, notes, _ = p.overlay()
    assert "free_move.ray_length_m" not in over                           # the asterisk keeps its length
    assert "up" in notes["free_move.ray_length_m"]
    p.set_edit("free_move.ray_layout", "2")                               # left and right only: within reach
    assert "free_move.ray_length_m" not in p.overlay()[1]
    c = ExperimentParams(experiments.BY_KEY["colour_rule"])
    c.profile = prof(**{"reach.left_m": 0.10})
    assert c.current()["centre_out.free_reach_m"] == 0.12 and "centre_out.free_reach_m" in c.overlay()[1]


def test_todays_edit_wins_and_an_edit_back_to_the_profile_is_no_edit():
    p = ExperimentParams(experiments.BY_KEY["colour_rule"])
    p.profile = prof(**{"feel.damping_ns_per_m": 70.0})
    p.set_edit("free_move.damping_ns_per_m", 80.0)
    assert p.current()["free_move.damping_ns_per_m"] == 80.0
    p.set_edit("free_move.damping_ns_per_m", 70.0)
    assert p.edits == {}
    p.set_edit("free_move.damping_ns_per_m", 50.0)                         # the YAML's value is now an edit
    assert p.edits == {"free_move.damping_ns_per_m": 50.0}


def test_the_run_file_has_the_profile_and_names_it(tmp_path, monkeypatch):
    import params as params_mod
    monkeypatch.setattr(params_mod, "RUNS_DIR", tmp_path)
    p = ExperimentParams(experiments.BY_KEY["move_vertical"])
    p.profile = prof(**{"feel.deadband_n": 4.0, "reach.up_m": 0.05})
    path = Path(p.launch_overrides()["params_file"])
    text = path.read_text()
    assert "# Profile M1 v1:" in text
    root = yaml.safe_load(text)["/**"]["ros__parameters"]
    assert root["free_move"]["deadband_n"] == 4.0 and root["free_move"]["rail_max_m"] == 0.05


def test_navins_experiments_ignore_the_profile():
    p = ExperimentParams(experiments.BY_KEY["apple_pluck"])
    p.profile = prof(**{"reward.pulse_ms": 311})
    assert p.current() == p.defaults and p.profile_tag() == ""


def test_save_versions_and_logs_every_change(data):
    a = profiles.save("M1", {}, "profile editor", note="created")
    assert a["version"] == 1 and a["values"] == profiles.defaults()
    b = profiles.save("M1", {"feel.damping_ns_per_m": "65"}, "Free Move")      # from the browser: a string
    assert b["version"] == 2 and b["values"]["feel.damping_ns_per_m"] == 65.0
    assert profiles.save("M1", {"feel.damping_ns_per_m": 65.0}, "Free Move")["version"] == 2   # no change: no version
    c = profiles.save("M1", {}, "Colour Rule", {"colour_rule": {"reward.pulse_ms": 372}})
    assert c["version"] == 3 and c["reward_by_task"] == {"colour_rule": {"reward.pulse_ms": 372}}
    log = profiles.changes("M1")
    assert [r["version"] for r in log] == [3, 2, 1]
    assert log[1]["changes"] == {"feel.damping_ns_per_m": [50.0, 65.0]} and log[1]["by"] == "Free Move"
    d = profiles.save("M1", {}, "editor", {"colour_rule": {"reward.pulse_ms": None}})      # back to the profile's
    assert d["reward_by_task"] == {}
    assert profiles.names() == ["M1"]
    doc = yaml.safe_load((data / "subjects" / "M1.yaml").read_text())
    assert doc["feel"]["damping_ns_per_m"] == 65.0 and doc["version"] == 4


def test_save_refuses_bad_values(data):
    with pytest.raises(ValueError):
        profiles.save("M1", {"feel.damping_ns_per_m": 1.0}, "x")              # below the limit
    with pytest.raises(ValueError):
        profiles.save("M1", {"reach.up_m": 0.5}, "x")
    with pytest.raises(KeyError):
        profiles.save("M1", {"feel.nonsense": 1}, "x")
    with pytest.raises(ValueError):
        profiles.save("M1", {}, "x", {"colour_rule": {"feel.deadband_n": 3.0}})   # only the reward per task
    assert not (data / "subjects" / "M1.yaml").exists()


def test_a_task_reward_override_applies_to_that_task_only(data):
    saved = profiles.save("M1", {"reward.pulse_ms": 311}, "x", {"colour_rule": {"reward.pulse_ms": 372}})
    for key, want in (("colour_rule", 372), ("n_choice", 311), ("move_horizontal", 311)):
        p = ExperimentParams(experiments.BY_KEY[key])
        p.profile = saved
        assert p.current()["reward_pulse.pulse_ms"] == want, key


def test_diff_from_a_tuned_task_and_the_reach_back_from_a_rail():
    e = experiments.BY_KEY["move_vertical"]
    p = ExperimentParams(e)
    p.profile = prof()
    vals = {**p.current(), "free_move.damping_ns_per_m": 60.0, "free_move.rail_max_m": 0.05}
    rows = {r["field"]: r for r in profiles.diff(p.profile, e, vals)}
    assert set(rows) == {"feel.damping_ns_per_m", "reach.up_m"}
    assert rows["reach.up_m"]["task"] == 0.05 and rows["reach.up_m"]["profile"] == 0.07
    # The Colour Rule on No rail does not write its (unused) rail ends back as the reach.
    c = experiments.BY_KEY["colour_rule"]
    cp = ExperimentParams(c)
    vals = {**cp.defaults, "free_move.rail_axis": "frontal", "free_move.rail_min_m": -0.05}
    assert not profiles.diff(prof(), c, vals)


def test_a_profile_file_missing_a_new_field_gets_the_default(data):
    (data / "subjects").mkdir()
    (data / "subjects" / "M2.yaml").write_text(yaml.safe_dump({"subject": "M2", "version": 4, "feel": {"deadband_n": 3.0}}))
    got = profiles.load("M2")
    assert got["values"]["feel.deadband_n"] == 3.0 and got["values"]["reward.count"] == 1
    assert "reward.count" in got["missing"] and got["version"] == 4
    assert json.dumps(got)                                                   # the dashboard sends it as JSON
