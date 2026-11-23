"""Reward volume (experiment_ctrl_gui/reward_volume.py): the rig's valve calibration -> mL per reward.

    python3 -m pytest sinthlab_bringup/test
"""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))
sys.path.insert(0, str(ROOT / "experiment_ctrl_gui"))

pytest.importorskip("experiments")
rv = pytest.importorskip("reward_volume")


@pytest.fixture(scope="module")
def cal():
    c = rv.calibration()
    assert c is not None, "no calibration under analysis/reward_calibrations/"
    return c


def test_the_committed_calibration_has_one_two_and_three_pulses(cal):
    assert sorted(p["burst"] for p in cal["profiles"]) == [1, 2, 3]
    assert cal["set"].startswith("2026-11-23")


def test_volumes_at_the_calibrated_widths(cal):
    # 2026-11-23, gravity reservoir: 1 pulse 246 / 311 / 372 ms = 1.2 / 1.4 / 1.6 mL; 2 x 78 ms and 3 x 32 ms = 1.2 mL.
    for ms, n, ml in ((246, 1, 1.2), (311, 1, 1.4), (372, 1, 1.6), (78, 2, 1.2), (32, 3, 1.2), (100, 1, 0.78)):
        got = rv.volume_ml(cal, ms, n, 100)
        assert got["status"] == "calibrated" and got["ml"] == pytest.approx(ml, abs=0.01), (ms, n)


def test_outside_the_widths_is_extrapolated_and_flagged(cal):
    got = rv.volume_ml(cal, 500, 1)
    assert got["status"] == "extrapolated" and "100-400" in got["note"]
    assert got["ml"] == pytest.approx((3.0414 * 500 + 464.5) / 1000, abs=0.01)
    assert rv.volume_ml(cal, 300, 4)["status"] == "none"                  # no 4-pulse calibration
    assert "100 ms gap" in rv.volume_ml(cal, 50, 2, 300)["note"]           # the YAMLs' 300 ms gap


def test_width_for_a_volume_round_trips(cal):
    for ml, n in ((1.2, 1), (1.0, 2), (1.3, 3)):
        w = rv.width_for_ml(cal, ml, n)
        assert w["ml"] == pytest.approx(ml, abs=0.01), (ml, n)
    with pytest.raises(ValueError):
        rv.width_for_ml(cal, 0.2, 1)                                       # less than any opening gives
    with pytest.raises(ValueError):
        rv.width_for_ml(cal, 1.0, 5)


def test_the_newest_set_wins(tmp_path):
    for name, slope in (("2026-01-01_old", 1.0), ("2026-12-01_pressure", 2.0)):
        d = tmp_path / name
        d.mkdir()
        (d / "single.json").write_text(json.dumps({"burst": 1, "points": [[100, 500], [200, 700]], "valid_width_ms": [100, 200],
                                                   "slope_ul_per_ms": slope, "intercept_ul": 300}))
    assert rv.calibration(tmp_path)["set"] == "2026-12-01_pressure"
    assert rv.calibration(tmp_path / "missing") is None
