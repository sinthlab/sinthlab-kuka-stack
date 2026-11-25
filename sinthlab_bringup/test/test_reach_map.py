"""Reach Map (helpers/reach_map.py): the New-best reward, the reach per direction and the profile's values.

    python3 -m pytest sinthlab_bringup/test
"""
import csv
import math
import random
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))

from sinthlab_bringup.helpers import reach_map as rm  # noqa: E402

BOX = (-0.18, 0.18, -0.10, 0.06)          # a_min, a_max (Y), b_min, b_max (Z) from the start


def test_sectors_centre_on_right_up_left_down():
    n = 12
    assert [rm.sector_of(*xy, n) for xy in [(1, 0), (0, 1), (-1, 0), (0, -1)]] == [0, 3, 6, 9]
    assert rm.sector_of(math.cos(math.radians(14)), math.sin(math.radians(14)), n) == 0
    assert rm.sector_of(math.cos(math.radians(16)), math.sin(math.radians(16)), n) == 1
    assert rm.sector_of(math.cos(math.radians(-14)), math.sin(math.radians(-14)), n) == 0
    with pytest.raises(ValueError):
        rm.BestTracker(n=10)                                   # right / up / left / down must have a sector each


def test_new_best_pays_only_past_the_last_best_by_the_margin():
    tr = rm.BestTracker(n=12, margin_m=0.01, min_m=0.03)
    assert not tr.check(0.02, 0.0)[0]                          # too close to the start
    assert tr.check(0.05, 0.0)[0]
    assert not tr.check(0.055, 0.0)[0]                         # better, but by less than the margin
    assert tr.check(0.07, 0.0)[0]
    assert tr.check(0.0, 0.04)[0]                              # another direction keeps its own best
    assert not tr.check(0.06, 0.001)[0]                        # right again: below its best (0.07)
    tr.configure(12, 0.02, 0.03)                               # a live margin change keeps the bests
    assert tr.best[0] == pytest.approx(0.07)
    tr.configure(8, 0.02, 0.03)                                # a new layout starts over
    assert tr.best == [0.0] * 8


def _reach_trace(targets, dt=0.01, seed=0):
    """A hand that goes out to each target and back toward the middle, resting between: (t, (x, y, z))."""
    rng = random.Random(seed)
    t, pts, cur = 0.0, [], (0.0, 0.0)
    out_t, out_p = [], []

    def hold(p, secs):
        nonlocal t
        for _ in range(int(secs / dt)):
            out_t.append(t); out_p.append((0.0, p[0] + rng.gauss(0, 0.00005), p[1] + rng.gauss(0, 0.00005))); t += dt

    def go(a, b, secs=0.6):
        nonlocal t, cur
        steps = int(secs / dt)
        for i in range(1, steps + 1):
            f = 0.5 - 0.5 * math.cos(math.pi * i / steps)
            out_t.append(t); out_p.append((0.0, cur[0] + f * (a - cur[0]), cur[1] + f * (b - cur[1]))); t += dt
        cur = (a, b)

    hold(cur, 1.0)
    for a, b in targets:
        go(a, b); hold(cur, 0.6)
        go(a * 0.2, b * 0.2); hold(cur, 0.6)
    return out_t, out_p


def test_reach_points_are_each_movements_farthest_point():
    t, p = _reach_trace([(0.12, 0.0), (0.0, -0.08), (-0.10, 0.02)])
    pts = rm.reach_points(t, p)
    # Only the outward movements: each return toward the middle starts at the reach point and is left out.
    assert [tuple(round(v, 2) for v in q) for q in pts] == [(0.12, 0.0), (0.0, -0.08), (-0.1, 0.02)]


def test_estimate_uses_the_percentile_and_flags_the_arm():
    pts = [(d, 0.0) for d in (0.08, 0.09, 0.10, 0.11, 0.12)]                         # right
    pts += [(0.0, d) for d in (0.055, 0.056, 0.058, 0.059, 0.06)]                    # up, against the box (0.06)
    pts += [(-0.10, 0.0)] * 2                                                        # left: too few
    rows = {r["deg"]: r for r in rm.estimate(pts, 12, 80, BOX)}
    assert rows[0]["comfortable_m"] == pytest.approx(0.112) and rows[0]["farthest_m"] == 0.12
    assert not rows[0]["arm_limited"] and rows[0]["wall_m"] == 0.18
    assert rows[90]["arm_limited"]
    assert rows[180]["comfortable_m"] is None and rows[180]["n"] == 2


def _ring(radius_fn, n=12, k=6):
    pts = []
    for s in range(n):
        deg = s * 360 / n
        for j in range(k):
            r = radius_fn(deg) * (0.9 + 0.1 * j / (k - 1))      # the top 20 %: near radius_fn(deg)
            pts.append((r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg))))
    return pts


def test_proposal_at_the_start_height_is_the_four_directions():
    reach = {0: 0.14, 90: 0.05, 180: 0.12, 270: 0.09}
    rows = rm.estimate(_ring(lambda d: reach.get(d, 0.08)), 12, 100, BOX)
    got = rm.proposal(rows, horizontal_dz=0.0)
    assert {k: v["value"] for k, v in got.items()} == pytest.approx(
        {"reach.right_m": 0.14, "reach.up_m": 0.05, "reach.left_m": 0.12, "reach.down_m": 0.09})


def test_proposal_notes_where_left_and_right_were_measured():
    rows = rm.estimate(_ring(lambda d: 0.10), 12, 100, BOX)
    got = rm.proposal(rows, horizontal_dz=0.05)
    assert got["reach.left_m"]["value"] == pytest.approx(0.10) and "5 cm below" in got["reach.left_m"]["why"]
    assert got["reach.up_m"]["arm_limited"] and not got["reach.up_m"]["why"]     # the box (6 cm) stops 10 cm
    few = rm.proposal(rm.estimate(_ring(lambda d: 0.10, k=3), 12, 100, BOX), 0.05)
    assert few["reach.down_m"]["value"] is None and "fewer than" in few["reach.down_m"]["why"]


def test_load_trial_measures_from_the_start_at_the_go_cue(tmp_path):
    f = tmp_path / "robot_trajectory_1.csv"
    with f.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "x", "y", "z", "event"])
        w.writerow([0.0, 0.5, 0.0, 0.3, ""])                    # on the way to the start: ignored
        w.writerow([1.0, 0.6, 0.1, 0.4, "armed"])
        w.writerow([1.01, 0.6, 0.15, 0.38, ""])
    t, p = rm.load_trial(f)
    assert t == [1.0, 1.01] and p[1] == pytest.approx((0.0, 0.05, -0.02))


def test_a_new_best_that_is_not_paid_is_not_kept():
    tr = rm.BestTracker(n=12, margin_m=0.01, min_m=0.03)
    new, k, d, prev = tr.check(0.08, 0.0, commit=False)
    assert new and tr.best[k] == 0.0                           # e.g. within the gap: not kept...
    assert tr.check(0.08, 0.0, commit=False)[0]                # ...so the same reach still counts next time
    tr.keep(k, d)
    assert not tr.check(0.085, 0.0)[0]
