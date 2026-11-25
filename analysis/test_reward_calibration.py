"""Tests for reward_calibration.py: the plan, the dispensing loop, the fit and its inverse.

    python3 -m pytest analysis/test_reward_calibration.py
"""
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import reward_calibration as rc  # noqa: E402


def rows_from(volume, widths, reps=3, pulses=50, density=1.0, noise=0.0, seed=0):
    """Sweep rows as the CSV holds them, for a valve giving volume(w) uL per pulse."""
    rng = random.Random(seed)
    rows = []
    for w in widths:
        for _ in range(reps):
            each = volume(w) * (1 + rng.gauss(0, noise))
            rows.append({"width_ms": str(w), "pulses_confirmed": str(pulses), "status": "ok",
                         "net_mass_g": str(each * pulses * density / 1000)})
    return rows


LINE = lambda w: 0.8 * w - 6.0          # uL per pulse: a valve that opens after ~7.5 ms
WIDTHS = [20, 40, 60, 100, 150, 200, 300]


# ---------------------------------------------------------------- plan
def test_plan_has_every_batch_shuffled_with_a_blank_first():
    plan = rc.make_plan(WIDTHS, reps=3, blanks=2, seed=4)
    widths = [b["width_ms"] for b in plan]
    assert len(plan) == 23 and widths[0] == 0 and widths.count(0) == 2
    assert sorted(w for w in widths if w) == sorted(WIDTHS * 3)
    assert [b["batch"] for b in plan] == list(range(1, 24))
    assert widths != [0] + sorted(WIDTHS * 3)                   # shuffled
    assert plan == rc.make_plan(WIDTHS, reps=3, blanks=2, seed=4)   # reproducible, so a sweep can resume


def test_spacing_must_let_the_valve_close():
    rc.check_widths([300], spacing=1.0)
    with pytest.raises(ValueError):
        rc.check_widths([800], spacing=1.0)                     # 0.8 s open leaves 0.2 s closed
    with pytest.raises(ValueError):
        rc.check_widths([5], spacing=1.0)                       # below the cabinet's 10 ms


# ---------------------------------------------------------------- dispensing
def test_dispense_sends_each_pulse_once():
    sent = []
    got, err = rc.dispense(lambda c: (sent.append(c), (True, "OK " + c, 1.0))[1], 4, 80, 5, 0.0,
                           sleep=lambda s: None)
    assert (got, err) == (5, None) and sent == ["PULSE 4 80"] * 5


def test_dispense_stops_at_an_unconfirmed_pulse_and_never_retries():
    sent = []

    def call(cmd):
        sent.append(cmd)
        return (True, "OK", 1.0) if len(sent) < 3 else (False, "no answer", 1000.0)

    got, err = rc.dispense(call, 4, 80, 10, 0.0, sleep=lambda s: None)
    assert got == 2 and err == "no answer" and len(sent) == 3


def test_blank_sends_nothing():
    slept = []
    got, err = rc.dispense(lambda c: pytest.fail("a blank must not pulse"), 4, 0, 10, 1.0, sleep=slept.append)
    assert (got, err) == (0, None) and slept == [10.0]


# ---------------------------------------------------------------- fit
def test_linear_fit_recovers_the_valve():
    p = rc.fit_profile(rows_from(LINE, WIDTHS, noise=0.01), density=1.0)
    assert p["slope_ul_per_ms"] == pytest.approx(0.8, rel=0.02)
    assert p["intercept_ul"] == pytest.approx(-6.0, abs=1.5)
    assert p["fit"]["r2"] > 0.99
    assert p["valid_width_ms"] == [20, 300]


def test_density_scales_the_volume():
    p = rc.fit_profile(rows_from(LINE, WIDTHS, density=1.05), density=1.05)
    assert rc.volume_for(p, 100) == pytest.approx(74.0, rel=1e-6)


def test_aborted_discarded_and_blank_batches_are_left_out():
    rows = rows_from(LINE, WIDTHS)
    rows.append({"width_ms": "100", "pulses_confirmed": "3", "status": "aborted", "net_mass_g": ""})
    rows.append({"width_ms": "100", "pulses_confirmed": "50", "status": "discarded", "net_mass_g": ""})
    rows.append({"width_ms": "0", "pulses_confirmed": "0", "status": "ok", "net_mass_g": "0.01"})
    p = rc.fit_profile(rows, density=1.0)
    assert p["fit"]["n_batches"] == 21 and p["blanks_g"] == [0.01]


def test_variable_narrow_widths_are_excluded_from_the_valid_range():
    rows = rows_from(LINE, WIDTHS[1:], noise=0.01)
    rows += [{"width_ms": "20", "pulses_confirmed": "50", "status": "ok", "net_mass_g": str(m)}
             for m in (0.2, 0.6, 0.9)]                          # 20 ms: drops some pulses
    p = rc.fit_profile(rows, density=1.0)
    assert p["excluded_widths_ms"] == [20] and p["valid_width_ms"][0] == 40


def test_too_few_widths_is_refused():
    with pytest.raises(ValueError):
        rc.fit_profile(rows_from(LINE, [100]), density=1.0)


# ---------------------------------------------------------------- inverse
def test_width_for_a_volume():
    p = rc.fit_profile(rows_from(LINE, WIDTHS), density=1.0)
    w, got = rc.width_for(p, 100.0)
    assert abs(w - 106 / 0.8) <= 0.5 + 1e-6 and got == pytest.approx(100.0, abs=0.5)   # whole ms
    assert rc.width_for(p, 0) == (0, 0.0)                       # no reward: no pulse


def test_volume_outside_the_calibration_is_refused():
    p = rc.fit_profile(rows_from(LINE, WIDTHS), density=1.0)
    with pytest.raises(ValueError):
        rc.width_for(p, 500.0)                                  # > 234 uL at 300 ms
    w, _ = rc.width_for(p, 500.0, allow_outside=True)
    assert abs(w - 506 / 0.8) <= 0.5 + 1e-6


def test_interp_follows_a_curved_valve():
    curve = lambda w: 40 * (1 - 2.718 ** (-w / 150)) * 3     # saturating
    lin = rc.fit_profile(rows_from(curve, WIDTHS), density=1.0, model="linear")
    itp = rc.fit_profile(rows_from(curve, WIDTHS), density=1.0, model="interp")
    assert max(abs(t["linear_resid_pct"]) for t in lin["per_width"]) > 5   # the line misses it
    w, got = rc.width_for(itp, curve(150))
    assert abs(w - 150) <= 1


def test_interp_refuses_means_that_do_not_rise():
    rows = rows_from(LINE, [40, 60, 100])
    rows += rows_from(lambda w: 30.0, [150])                    # 150 ms gives less than 100 ms
    with pytest.raises(ValueError):
        rc.fit_profile(rows, density=1.0, model="interp")


# ---------------------------------------------------------------- the CSV round trip
def test_csv_round_trip(tmp_path):
    out = tmp_path / "sweep.csv"
    for r in rows_from(LINE, WIDTHS):
        rc.append_row(out, {**r, "batch": 1})
    p = rc.fit_profile(rc.read_rows(out), density=1.0)
    assert p["slope_ul_per_ms"] == pytest.approx(0.8, rel=1e-6)
