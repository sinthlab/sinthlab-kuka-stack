"""Behavioural outcome tables (helpers/outcome_tables.py), the recorder's part in them, the rebuild script
and the dashboard's subject check.

    python3 -m pytest sinthlab_bringup/test
"""
import csv
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import test_centre_out_monitor as stubs  # noqa: E402  (ROS stand-ins where ROS is missing)
from sinthlab_bringup.helpers import outcome_tables as ot  # noqa: E402

T0 = 1_795_400_000.0


def ev(name, dt, arg=None):
    return {"event": name, "arg": arg, "t_wall": T0 + dt}


def centre_out(trial, events, params=None, rewards=(), exp="move_horizontal", subject="M1"):
    return {"experiment": exp, "trial_index": trial, "session_id": "20261123_100000", "subject": subject,
            "git": {"sha": "abc1234", "dirty": False}, "partial": False,
            "params": {"centre_out.target": "left", "centre_out.end_tol_m": 0.02, "centre_out.block_other_end": True,
                       "centre_out.max_repeats": 3, "reward_pulse.pulse_ms": 500, **(params or {})},
            "events": events, "rewards": [{"output": 4, "ms": 500, "event": "reach"} for _ in rewards]}


def good_reach(trial, dt0=0.0):
    return centre_out(trial, [ev("trial_start", dt0, trial), ev("cue_go", dt0 + 4), ev("closed_end", dt0 + 4, "right"),
                              ev("target", dt0 + 4, "left"), ev("reach", dt0 + 5.2, "left"), ev("trial_end", dt0 + 5.2, trial)],
                      rewards=[1])


# ---------------------------------------------------------------- one trial's row
def test_common_and_centre_out_columns():
    row = ot.trial_row(good_reach(3), "expt_x")
    assert row["run"] == "expt_x" and row["subject"] == "M1" and row["trial"] == 3
    assert row["paradigm"] == "left"                       # the card's paradigm: centre_out.target
    assert (row["outcome"], row["success"]) == ("reach", 1)
    assert row["outcome_latency_s"] == pytest.approx(1.2)
    assert (row["n_rewards"], row["reward_ms"]) == (1, 500)
    assert (row["target"], row["closed_end"], row["chosen_end"], row["correct"]) == ("left", "right", "left", 1)
    assert list(row)[:len(ot.COMMON)] == list(ot.COMMON)


def test_wrong_end_and_aborted_trials():
    wrong = ot.trial_row(centre_out(1, [ev("trial_start", 0), ev("cue_go", 4), ev("target", 4, "left"),
                                        ev("reach_wrong", 6, "right"), ev("trial_end", 6)]))
    assert (wrong["success"], wrong["correct"], wrong["chosen_end"], wrong["n_rewards"]) == (0, 0, "right", 0)
    stopped = ot.trial_row(centre_out(2, [ev("trial_start", 0), ev("stop_requested", 2), ev("trial_end", 2)]))
    assert (stopped["outcome"], stopped["success"], stopped["correct"]) == ("stop_requested", "", "")


def test_an_experiment_without_its_own_tables_gets_the_common_columns():
    row = ot.trial_row({"experiment": "maze", "trial_index": 1, "events": [ev("trial_start", 0), ev("goal", 30)]})
    assert list(row) == list(ot.COMMON) and row["outcome"] == "goal" and row["success"] == 1


def test_long_moves_movements_table():
    meta = {"experiment": "free_move", "trial_index": 1,
            "params": {"long_move.reward": "each_move", "long_move.min_m": 0.1},
            "events": [ev("trial_start", 0), ev("cue_go", 3), ev("move_start", 4),
                       ev("move_end", 5, "length=0.150;straightness=0.95;duration=1.10;peak_speed=0.250"),
                       ev("long_move", 5, 0.15), ev("move_start", 7),
                       ev("move_end", 8, "length=0.040;straightness=0.80;duration=0.60;peak_speed=0.100"),
                       ev("session_end", 60), ev("trial_end", 60)]}
    row = ot.trial_row(meta)
    assert (row["n_movements"], row["n_long_moves"], row["max_length_m"]) == (2, 1, 0.15)
    mv = ot.unit_rows(meta, "r")["movements"]
    assert [m["rewarded"] for m in mv] == [1, 0] and mv[1]["length_m"] == "0.040"


# ---------------------------------------------------------------- files
def write_run(folder: Path, metas):
    folder.mkdir(parents=True)
    for m in metas:
        (folder / f"robot_trajectory_{m['trial_index']:03d}.meta.json").write_text(json.dumps(m))


def read(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def test_append_writes_the_header_once(tmp_path):
    for k in (1, 2):
        ot.append_trial(tmp_path, good_reach(k))
    rows = read(tmp_path / "trials.csv")
    assert [r["trial"] for r in rows] == ["1", "2"]


def test_a_changed_header_starts_a_new_file(tmp_path):
    (tmp_path / "trials.csv").write_text("old,columns\n1,2\n")
    ot.append_trial(tmp_path, good_reach(1))
    assert (tmp_path / "trials.csv").read_text() == "old,columns\n1,2\n"
    assert len(list(tmp_path.glob("trials_*.csv"))) == 1


def test_rebuild_and_session_row(tmp_path):
    run = tmp_path / "expt_iiwa7_pretrain_move_horizontal_20261123_100000"
    metas = [good_reach(1, 0), good_reach(2, 10),
             centre_out(3, [ev("trial_start", 20), ev("cue_go", 24), ev("target", 24, "right"),
                            ev("reach_wrong", 25, "left"), ev("trial_end", 25)],
                        params={"centre_out.target": "cued", "centre_out.block_other_end": False})]
    write_run(run, metas)
    assert ot.rebuild_run(run) == 3
    assert (run / "tables.md").exists()
    row = ot.update_sessions(run, tmp_path / "sessions.csv")
    assert (row["n_trials"], row["n_success"], row["success_pct"]) == (3, 2, 66.7)
    assert row["paradigms"] == "left | cued" and row["subject"] == "M1"
    assert (row["n_reaches"], row["pct_correct"]) == (3, 66.7)
    assert "centre_out.target: left -> cued" in row["changed"]
    assert "centre_out.target=left" in row["settings"]
    # Adding the same run again replaces its row; another run adds one.
    ot.update_sessions(run, tmp_path / "sessions.csv")
    other = tmp_path / "expt_iiwa7_maze_20261124_090000"
    write_run(other, [{"experiment": "maze", "trial_index": 1, "subject": "M2",
                       "events": [ev("trial_start", 86400), ev("goal", 86430), ev("trial_end", 86430)]}])
    ot.rebuild_run(other)
    ot.update_sessions(other, tmp_path / "sessions.csv")
    rows = read(tmp_path / "sessions.csv")
    assert [r["subject"] for r in rows] == ["M1", "M2"]
    assert rows[1]["pct_correct"] == ""                    # a centre-out column, blank for the maze


def test_rebuild_script(tmp_path, capsys):
    sys.path.insert(0, str(ROOT / "analysis"))
    import outcome_tables as cli
    write_run(tmp_path / "expt_a_20261123_100000", [good_reach(1)])
    assert cli.main(["--data-dir", str(tmp_path)]) == 0
    assert (tmp_path / "expt_a_20261123_100000" / "trials.csv").exists()
    assert len(read(tmp_path / "sessions.csv")) == 1


def test_data_dir_follows_the_environment(monkeypatch, tmp_path):
    monkeypatch.setenv(ot.DATA_DIR_ENV, str(tmp_path))
    assert ot.data_dir() == tmp_path
    monkeypatch.delenv(ot.DATA_DIR_ENV)
    assert str(ot.data_dir()).endswith("lbr-stack/src/sinthlab-kuka-stack/analysis")


# ---------------------------------------------------------------- the recorder
class _Clock:
    def now(self):
        class T:
            nanoseconds = 0
        return T()


def test_recorder_keeps_the_commanded_reward_and_writes_trials_csv(tmp_path):
    from sinthlab_bringup.actions.trial_recorder import TrialRecorder
    rec = TrialRecorder.__new__(TrialRecorder)       # no ROS: only mark() and stop_and_save()
    node = stubs.FakeNode({})
    node.get_clock = lambda: _Clock()
    rec._node, rec._experiment, rec._save_dir = node, "move_horizontal", str(tmp_path)
    rec._on_event = lambda tok, arg: {"output": 4, "ms": 350} if tok == "reach" else None
    rec._active, rec._t0, rec._rows, rec._pending, rec._events = True, None, 0, [], []
    rec._last_msg, rec._fh, rec._writer, rec._docs_written = None, None, None, False
    rec._path = str(tmp_path / "robot_trajectory_1.csv")
    rec._sidecar = {"experiment": "move_horizontal", "trial_index": 1, "clock_sync": {"start": None, "end": None},
                    "params": {"centre_out.target": "either"}}
    rec._clock_triple = lambda: None
    for tok, arg in (("trial_start", 1), ("cue_go", None), ("target", "either"), ("reach", "left"), ("trial_end", 1)):
        rec.mark(tok, arg)
    rec.stop_and_save()
    side = json.loads((tmp_path / "robot_trajectory_1.meta.json").read_text())
    assert side["rewards"][0]["ms"] == 350 and side["rewards"][0]["event"] == "reach"
    row = read(tmp_path / "trials.csv")[0]
    assert (row["outcome"], row["n_rewards"], row["reward_ms"], row["chosen_end"]) == ("reach", "1", "350", "left")
    assert (tmp_path / "tables.md").exists()


# ---------------------------------------------------------------- the dashboard's subject check
def test_subject_ids():
    sys.path.insert(0, str(ROOT / "experiment_ctrl_gui"))
    server = pytest.importorskip("server")
    assert server.check_subject(" M1 ") == "M1"
    assert server.check_subject("test") == "test"
    for bad in ("", "  ", "M 1", "-M1", "x" * 40, "M1;rm"):
        with pytest.raises(ValueError):
            server.check_subject(bad)


# ---------------------------------------------------------------- today vs recent sessions (dashboard view E)
def test_summarize_takes_typed_rows_as_the_demo_builds_them():
    rows = [ot.trial_row(good_reach(1)), ot.trial_row(good_reach(2, 10))]        # ints, not CSV strings
    row = ot.summarize("demo_run", rows, {"reward_pulse.pulse_ms": 500, "centre_out.target": "left"},
                       {"reward_pulse.pulse_ms": 350, "centre_out.target": "left"})
    assert (row["n_trials"], row["n_success"], row["success_pct"], row["n_rewards"]) == (2, 2, 100.0, 2)
    assert "reward_pulse.pulse_ms=500" in row["settings"]
    assert row["changed"] == "reward_pulse.pulse_ms: 500 -> 350"
    assert ot.summarize("empty", [], {}, {}) is None


def test_settings_for_adds_the_common_ones_once():
    s = ot.settings_for("colour_rule")
    assert s[0] == "free_move.rail_axis" and "quiet_window_sec" in s and s.count("reward_pulse.pulse_ms") == 1
    assert "reward_pulse.pulse_ms" in ot.settings_for("maze")                    # no class of its own


def test_parse_settings_round_trips():
    row = ot.summarize("r", [ot.trial_row(good_reach(1))],
                       {"centre_out.target": "left", "reward_pulse.pulse_ms": 500, "quiet_window_sec": 2.0}, {})
    assert ot.parse_settings(row["settings"]) == {"centre_out.target": "left", "reward_pulse.pulse_ms": "500",
                                                  "quiet_window_sec": "2.0"}
    assert ot.parse_settings("") == {}


def test_history_filters_by_experiment_and_subject_oldest_first(tmp_path):
    path = tmp_path / "sessions.csv"
    for k, (subj, exp, ms) in enumerate([("M1", "colour_rule", 400), ("M2", "colour_rule", 300),
                                         ("M1", "n_choice", 200), ("M1", "colour_rule", 450),
                                         ("M1", "colour_rule", 500), ("M1", "colour_rule", 550)]):
        ot.upsert_session({"run": f"r{k}", "date": f"2026-11-{10 + k}", "start": "09:00", "subject": subj,
                           "experiment": exp, "settings": f"reward_pulse.pulse_ms={ms}"}, path)
    h = ot.history("colour_rule", "M1", 3, path)
    assert [x["run"] for x in h] == ["r3", "r4", "r5"]
    assert [x["settings_dict"]["reward_pulse.pulse_ms"] for x in h] == ["450", "500", "550"]
    assert len(ot.history("colour_rule", "", 0, path)) == 5                    # every subject, every session
    assert ot.history("maze", "M1", 3, path) == []


def test_validate_recording_knows_every_experiment():
    """The dashboard's Validate recording checks each experiment's columns: none may be unknown."""
    sys.path.insert(0, str(ROOT / "analysis"))
    import validate_recording as vr
    experiments = pytest.importorskip("experiments")
    for key in experiments.BY_KEY:
        assert key in vr.EXTRA, f"validate_recording.py has no column layout for {key}"
    assert vr.EXTRA["n_choice"] == ["eq_dx", "eq_dy", "eq_dz", "f_adm_x", "f_adm_y", "f_adm_z"]
