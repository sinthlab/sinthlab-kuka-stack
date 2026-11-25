"""The Reward calibration page's server side (experiment_ctrl_gui/calibration.py), with a simulated cabinet.

    python3 -m pytest sinthlab_bringup/test
"""
import json
import shutil
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))
sys.path.insert(0, str(ROOT / "experiment_ctrl_gui"))

pytest.importorskip("experiments")
cal = pytest.importorskip("calibration")
rv = pytest.importorskip("reward_volume")
REAL = rv.FOLDER


def make(tmp_path, busy=None, copy=True):
    folder = tmp_path / "reward_calibrations"
    if copy:
        shutil.copytree(REAL, folder)
    else:
        folder.mkdir()
    log = []
    c = cal.Calibrator(True, busy or (lambda: None), lambda text, kind: log.append(text), folder, sleep_scale=0.0)
    return c, folder, log


def wait_phase(c, phases=("weigh",), timeout=5.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if (c.job is None and None in phases) or (c.job and c.job["phase"] in phases):
            return
        time.sleep(0.01)
    raise AssertionError(f"job stuck at {c.job}")


def test_state_lists_the_committed_set_as_in_use(tmp_path):
    c, _, _ = make(tmp_path)
    st = c.state()
    assert st["active"] == "2026-11-23_gravity_reservoir"
    s = st["sets"][0]
    assert [p["burst"] for p in s["profiles"]] == [1, 2, 3]
    assert {w["name"] for w in s["sweeps"]} >= {"single", "burst2", "burst3"}
    json.dumps(st, allow_nan=False)          # the page parses it: no NaN anywhere


def test_a_whole_sweep_then_the_fit_switches_the_dashboard(tmp_path):
    c, folder, _ = make(tmp_path)
    r = c.new_sweep("", "Pressure Reservoir", 1, 0, [100, 200, 300], reps=2, blanks=1, rewards=10)
    assert r["set"].endswith("_pressure_reservoir") and r["name"] == "single"
    # A set with only a sweep in progress is passed over: the dashboard keeps the last fitted set.
    assert rv.calibration(folder)["set"] == "2026-11-23_gravity_reservoir"
    with pytest.raises(ValueError, match="already has"):
        c.new_sweep(r["set"], "", 1, 0, [100, 200], 1, 0, 10)
    true_ul = lambda w: 2.0 * w + 500.0                       # noqa: E731
    while True:
        sw = next(s for s in c.sets() if s["name"] == r["set"])["sweeps"][0]
        if sw["next"] is None:
            break
        c.batch("", 4, r["set"], "single")
        wait_phase(c)
        w = c.job["width_ms"]
        c.weigh(0.0 if w == 0 else true_ul(w) * 10 / 1000.0 * (1.01 if sw["n_ok"] % 2 else 0.99))
        assert c.job is None and c.last["ok"]
    with pytest.raises(ValueError, match="every batch"):
        c.batch("", 4, r["set"], "single")
    out = c.fit(r["set"], "single", "interp")
    assert out["profile"]["valid_width_ms"] == [100, 300]
    # The new set is the newest: the dashboard now uses it, and it lacks the 2- and 3-pulse sizes.
    assert rv.calibration(folder)["set"] == r["set"]
    assert any("2 or 3 pulses" in n for n in out["notes"]), out["notes"]
    assert rv.volume_ml(rv.calibration(folder), 200, 1)["ml"] == pytest.approx(0.9, abs=0.01)
    rows = (folder / r["set"] / "single.csv").read_text()
    # The CLI can read the same files: it resumes a dashboard sweep.
    assert rows.splitlines()[0].split(",") == cal.rc.FIELDS


def test_a_check_compares_with_the_profile_in_use_and_is_logged(tmp_path):
    c, folder, _ = make(tmp_path)
    c.check("", 4, 1, 200, 10)
    wait_phase(c)
    assert c.job["expected_g"] == pytest.approx(10.6, abs=0.1)        # 200 ms = 1.06 mL
    c.weigh(10.6)
    assert c.last["ok"] and abs(c.last["off_pct"]) < 1
    c.check("", 4, 1, 200, 10)
    wait_phase(c)
    c.weigh(9.0)
    assert not c.last["ok"] and c.last["off_pct"] < -10
    checks = c.sets()[0]["checks"]
    assert len(checks) == 2 and checks[0]["note"].startswith("check vs")
    assert (folder / "2026-11-23_gravity_reservoir" / "checks.csv").exists()


def test_a_check_needs_a_calibrated_size(tmp_path):
    c, _, _ = make(tmp_path)
    with pytest.raises(ValueError, match="4 pulse"):
        c.check("", 4, 4, 30, 10)


def test_no_dispensing_while_an_experiment_runs(tmp_path):
    c, _, _ = make(tmp_path, busy=lambda: "maze")
    with pytest.raises(RuntimeError, match="maze is running"):
        c.prime("", 4, 200, 10)
    assert c.job is None


def test_one_job_at_a_time_and_weigh_or_discard_to_move_on(tmp_path):
    c, folder, _ = make(tmp_path)
    r = c.new_sweep("", "x", 2, 300, [30, 60], 1, 0, 5)
    c.batch("", 4, r["set"], "burst2")
    wait_phase(c)
    with pytest.raises(RuntimeError, match="weigh it"):
        c.prime("", 4, 200, 10)
    c.weigh(discard=True)
    row = cal.rc.read_rows(folder / r["set"] / "burst2.csv")[0]
    assert row["status"] == "discarded" and row["burst"] == "2" and row["burst_gap_ms"] == "300"
    assert next(s for s in c.sets() if s["name"] == r["set"])["sweeps"][0]["n_ok"] == 0


def test_stop_ends_the_batch_as_aborted_and_sets_the_outputs_low(tmp_path):
    c, folder, _ = make(tmp_path, copy=False)
    c._scale = 1.0                                          # real waits, so there is time to stop
    sent = []
    fake = c._fake.call
    c._fake.call = lambda cmd: (sent.append(cmd), fake(cmd))[1]
    r = c.new_sweep("", "x", 1, 0, [100, 200], 1, 0, 20, spacing=1.0)
    c.batch("", 4, r["set"], "single")
    time.sleep(0.2)
    c.stop()
    wait_phase(c, (None,))
    assert "ALLOFF" in sent and not c.last["ok"]
    row = cal.rc.read_rows(folder / r["set"] / "single.csv")[0]
    assert row["status"] == "aborted"


def test_prime_records_nothing(tmp_path):
    c, folder, _ = make(tmp_path, copy=False)
    c.prime("", 4, 200, 3)
    wait_phase(c, (None,))
    assert c.last["ok"] and "Primed: 3" in c.last["text"] and not any(folder.iterdir())


def test_inputs_are_checked(tmp_path):
    c, _, _ = make(tmp_path, copy=False)
    with pytest.raises(ValueError, match="2 widths"):
        c.new_sweep("", "x", 1, 0, [100], 1, 0, 10)
    with pytest.raises(ValueError, match="setup"):
        c.new_sweep("", "", 1, 0, [100, 200], 1, 0, 10)
    with pytest.raises(ValueError, match="unknown sweep"):
        c.batch("", 4, "../etc", "single")
    with pytest.raises(ValueError, match="too short"):
        c.new_sweep("", "x", 1, 0, [100, 900], 1, 0, 10, spacing=1.0)


def test_spacing_leaves_the_valve_closed():
    assert cal.spacing_for(400, 1, 0) == 1.0
    assert cal.spacing_for(900, 1, 0) == 1.5
    assert cal.spacing_for(50, 3, 300) == 1.5               # 0.75 s burst + 0.3 s closed
