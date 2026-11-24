"""Foraging: helpers/forage.py (today's environment, the arena, the warm-zone cue, the zones), actions/forage.py
on a fake arm and ring, the admittance's elliptical arena, its trials.csv columns and its card.

    python3 -m pytest sinthlab_bringup/test
"""
import datetime as dt
import json
import random
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(ROOT / "experiment_ctrl_gui"))

import test_centre_out_monitor as stubs  # noqa: E402  (ROS stand-ins)
from sinthlab_bringup.helpers import forage as fg, live_params, outcome_tables as ot  # noqa: E402
from sinthlab_bringup.actions import forage as fa  # noqa: E402
from sinthlab_bringup.actions.admittance_move import AdmittanceMoveAction  # noqa: E402

YAML = ROOT / "sinthlab_bringup/config/forage.yaml"


def yaml_params():
    out = {}

    def flat(node, prefix=""):
        for k, v in node.items():
            if isinstance(v, dict):
                flat(v, prefix + k + ".")
            else:
                out[prefix + k] = v
    flat(yaml.safe_load(YAML.read_text())["/**"]["ros__parameters"])
    return out


def settings(**over):
    p = {**yaml_params(), **over}
    return fg.settings_from(lambda n: p[n])


# ---------------------------------------------------------------- today's environment
def test_the_day_seed_is_one_environment_per_monkey_per_day():
    d = dt.date(2026, 11, 25)
    assert fg.day_seed("M1", d) == fg.day_seed("M1", d)
    assert fg.day_seed("M1", d) != fg.day_seed("M1", d + dt.timedelta(days=1))
    assert fg.day_seed("M1", d) != fg.day_seed("M2", d)


def test_zones_are_reproducible_and_keep_their_distances():
    s = settings(**{"forage.n_zones": 3, "forage.zone_radius_m": 0.02})
    a, b = fg.zones_for(s, 1234), fg.zones_for(s, 1234)
    assert all(np.allclose(x, y) for x, y in zip(a, b)) and len(a) == 3
    assert not np.allclose(a[0], fg.zones_for(s, 99)[0])
    for i, z in enumerate(a):
        assert z[0] == 0.0                                                           # the plane: no depth
        assert fg.inside(z, "rectangle", s.lo, s.hi, fg.PLANE_DIMS, s.zone_radius_m + s.wall_margin_m)
        assert np.linalg.norm(z[1:]) >= s.zone_radius_m + s.start_clear_m
        for w in a[i + 1:]:
            assert np.linalg.norm((z - w)[1:]) >= 2 * s.zone_radius_m + s.min_gap_m


def test_three_d_and_ellipse_arenas():
    s = settings(**{"free_move.rail_axis": "none", "forage.zone_radius_m": 0.025})
    z = fg.zones_for(s, 7)[0]
    assert fg.inside(z, "rectangle", s.lo, s.hi, fg.SPACE_DIMS, 0.025)
    s = settings(**{"free_move.box.shape": "ellipse"})
    z = fg.zones_for(s, 7)[0]
    assert fg.inside(z, "ellipse", s.lo, s.hi, fg.PLANE_DIMS, s.zone_radius_m + s.wall_margin_m)


def test_zones_that_cannot_fit_are_refused_with_the_reason():
    with pytest.raises(ValueError, match="do not fit"):
        fg.zones_for(settings(**{"forage.n_zones": 6, "forage.zone_radius_m": 0.05}), 1)
    p = yaml_params()
    assert fg.params_problem(p, "M1") is None                                       # the defaults fit
    assert "do not fit" in fg.params_problem({**p, "forage.n_zones": 6, "forage.zone_radius_m": 0.05}, "M1")
    assert "rail_axis" in fg.params_problem({**p, "free_move.rail_axis": "y"}, "M1")
    assert fg.params_problem({"free_move.damping_ns_per_m": 50}) is None          # not foraging


def test_the_ellipse_wall():
    lo, hi = np.array([-0.05, -0.15, -0.07]), np.array([0.03, 0.15, 0.06])
    d = fg.clamp_ellipse(np.array([0.0, 0.15, 0.06]), lo, hi, fg.PLANE_DIMS)     # the box's corner: outside
    c, r = (lo + hi) / 2, (hi - lo) / 2
    assert sum(((d[i] - c[i]) / r[i]) ** 2 for i in fg.PLANE_DIMS) == pytest.approx(1.0)
    inner = np.array([0.0, 0.02, 0.0])
    assert np.allclose(fg.clamp_ellipse(inner, lo, hi, fg.PLANE_DIMS), inner)


def test_the_admittance_holds_the_equilibrium_in_the_elliptical_arena():
    adm = AdmittanceMoveAction.__new__(AdmittanceMoveAction)
    adm._rail, adm._open, adm._shape, adm._rays = None, None, "frontal", None
    adm._plane_n = np.array([1.0, 0.0, 0.0])
    adm._anchor = np.eye(4)
    adm._box = (np.array([-0.05, -0.15, -0.07]), np.array([0.03, 0.15, 0.06]))
    adm._box_shape = "ellipse"
    adm._floor = type("NoFloor", (), {"clamp_xyz": staticmethod(lambda xyz, R: xyz)})()
    out = adm._bound(np.array([0.02, 0.15, 0.06]))                               # toward him and to the corner
    assert out[0] == 0.0 and fg.inside(out, "ellipse", *adm._box, fg.PLANE_DIMS, -1e-9)


# ---------------------------------------------------------------- the warm-zone cue and probes
def test_the_warm_cue_shrinks_then_fades_then_is_off():
    s = settings()
    assert fg.warm_state(s, 1) == (2.5, 1.0) and fg.warm_state(s, 5) == (1.0, 1.0)
    assert fg.warm_state(s, 6)[1] == pytest.approx(5 / 6, abs=1e-3) and fg.warm_state(s, 10)[1] > 0
    assert fg.warm_state(s, 11) == (1.0, 0.0)
    s.warm_mode = "always"
    assert fg.warm_state(s, 40) == (2.5, 1.0)
    s.warm_mode = "off"
    assert fg.warm_state(s, 1)[1] == 0.0


def test_probes_only_after_their_trial_and_without_the_cue():
    s = settings(**{"probe.probability": 1.0, "probe.after_trial": 3, "warm.mode": "always"})
    rng = random.Random(0)
    plans = [fg.plan_trial(s, t, 1, [], rng) for t in (1, 2, 3)]
    assert [p.probe for p in plans] == [False, False, True] and plans[2].warm_brightness == 0.0


# ---------------------------------------------------------------- one trial's zones
def test_dwell_pays_once_revisits_count_and_all_found_ends():
    s = settings(**{"forage.n_zones": 2, "forage.dwell_sec": 0.3, "forage.zone_radius_m": 0.02})
    zones = [np.array([0, 0.08, 0.0]), np.array([0, -0.08, 0.0])]
    t = fg.ZoneTracker(s, fg.TrialPlan(1, 1, zones, 2.5, 1.0, False))
    assert t.warm([0, 0.08 + 0.04, 0]) and not t.warm([0, 0.0, 0])               # within 2.5 x 2 cm of zone 0
    assert t.step([0, 0.08, 0], 0.0) == [("enter", 0)] and t.step([0, 0.08, 0], 0.2) == []
    assert t.step([0, 0.08, 0], 0.31) == [("zone", 0)]                           # after the dwell
    assert not t.warm([0, 0.09, 0])                                              # an emptied zone: no cue
    t.step([0, 0, 0], 1.0)
    assert t.step([0, 0.08, 0], 1.1) == [("revisit", 0)] and t.revisits == 1
    t.step([0, -0.08, 0], 2.0)
    t.step([0, -0.08, 0], 2.4)
    assert t.done() and t.found == [0, 1]


def test_a_probe_pays_nothing():
    s = settings()
    t = fg.ZoneTracker(s, fg.TrialPlan(12, 1, [np.array([0, 0.08, 0.0])], 1.0, 0.0, True))
    assert t.step([0, 0.08, 0], 0.0) == [("enter", 0)] and t.step([0, 0.08, 0], 5.0) == [] and not t.done()


# ---------------------------------------------------------------- the monitor on a fake arm and ring
class Clock:
    def __init__(self):
        self.t = 50.0

    def monotonic(self):
        return self.t


class FakeArm:
    def __init__(self, shape="frontal"):
        self.pos, self.shape_ = np.zeros(3), shape
        p = yaml_params()
        self.box_ = (np.array([p[f"free_move.box.{a}_min_m"] for a in "xyz"]),
                     np.array([p[f"free_move.box.{a}_max_m"] for a in "xyz"]))

    def shape(self):
        return self.shape_

    def box(self):
        return self.box_

    def box_shape(self):
        return "rectangle"

    def rest_offset_from_anchor(self):
        return self.pos.copy()


class FakeRing:
    def __init__(self):
        self.shown, self._state = [], None

    def show(self, state, quarters=None, *, label=None, scale=1.0):
        self._state = state
        self.shown.append((state, round(scale, 3)))

    def reward(self, *, label=None):
        self.show("reward")

    def state(self):
        return self._state


def monitor(monkeypatch, **over):
    clock = Clock()
    monkeypatch.setattr(fa, "time", clock)
    params = {**yaml_params(), "subject": "M1", **over}
    node, arm, ring, marks = stubs.FakeNode(params), FakeArm(), FakeRing(), []
    mon = fa.ForageMonitor(node, arm, ring, lambda e, a=None: marks.append((e, a)))
    return node, arm, ring, mon, marks, clock


def tick(node):
    for t in node.timers:
        t()


def test_a_search_with_the_warm_cue_a_reward_flash_and_the_end(monkeypatch):
    node, arm, ring, mon, marks, clock = monitor(monkeypatch, **{"forage.n_zones": 2, "forage.zone_radius_m": 0.02})
    assert mon.seed == fg.day_seed("M1") and len(mon.zones) == 2
    mon.pick(1)
    got, paid = [], []
    mon.start(paid.append, got.append)
    z0, z1 = mon.zones
    arm.pos = z0 + np.array([0, 0.03, 0.0])                    # near zone 0: inside the warm radius (5 cm)
    tick(node)
    assert ring.shown[-1] == ("warm", 1.0) and marks[-1][0] == "warm_on"
    arm.pos = z0.copy()
    tick(node)
    assert paid == [0] and ("zone", 0) in marks and ring.state() == "reward"     # the reward flash
    clock.t += 1.2
    arm.pos = np.zeros(3)
    tick(node)
    assert ring.state() == "off"                                                  # the flash is over
    arm.pos = z1.copy()
    tick(node)
    assert got == ["forage_done"] and paid == [0, 1]
    r = mon.result()
    assert r["found"] == [0, 1] and r["n_found"] == 2 and r["path_m"] > 0


def test_a_probe_ends_on_its_clock(monkeypatch):
    node, arm, ring, mon, marks, clock = monitor(monkeypatch, **{"probe.probability": 1.0, "probe.after_trial": 1})
    mon.pick(1)
    got = []
    mon.start(lambda i: None, got.append)
    arm.pos = mon.zones[0].copy()
    tick(node)
    assert not got and ("zone_enter", 0) in marks and not any(e == "zone" for e, _ in marks)
    clock.t += 20.5
    tick(node)
    assert got == ["probe_end"]


def test_needs_the_arena(monkeypatch):
    arm = FakeArm(shape="y")
    with pytest.raises(ValueError):
        fa.ForageMonitor(stubs.FakeNode({**yaml_params(), "subject": "M1"}), arm, FakeRing(), lambda *a: None)


# ---------------------------------------------------------------- trials.csv / sessions.csv
def sidecar(k, outcome, found, times, probe=False):
    ev = lambda n, t, a=None: {"event": n, "arg": a, "t_wall": 1e9 + 100 * k + t}   # noqa: E731
    evs = [ev("trial_start", 0, k), ev("cue_go", 3)] + [ev("zone", 3 + t, i) for i, t in zip(found, times)]
    evs += [ev(outcome, 3 + (times[-1] if times else 20)), ev("trial_end", 30, k)]
    return {"experiment": "forage", "trial_index": k, "params": {"free_move.rail_axis": "frontal"},
            "forage": {"seed": 5, "space": "frontal", "shape": "ellipse", "zones": [[0, 0.08, 0], [0, -0.08, 0]],
                       "warm_factor": 1.0, "warm_brightness": 0.5, "probe": probe, "dwell_sec": 0.2},
            "forage_result": {"found": found, "n_found": len(found), "n_zones": 2, "t_zone_s": times,
                              "revisits": 1, "path_m": 0.4, "warm_on_s": 1.5}, "events": evs}


def test_columns_and_session(tmp_path):
    metas = [sidecar(1, "forage_done", [1, 0], [9.0, 14.0]), sidecar(2, "response_timeout", [0], [8.0]),
             sidecar(3, "forage_done", [0, 1], [3.0, 5.0]), sidecar(4, "probe_end", [], [], probe=True)]
    row = ot.trial_row(metas[0])
    assert (row["paradigm"], row["arena"], row["n_found"], row["all_found"], row["found_order"]) == \
        ("plane", "ellipse", 2, 1, "1 0")
    assert (row["t_first_s"], row["t_all_s"], row["success"], row["revisits"]) == (9.0, 14.0, 1, 1)
    probe = ot.trial_row(metas[3])
    assert (probe["probe"], probe["all_found"], probe["success"]) == (1, "", "")
    run = tmp_path / "expt_f"
    run.mkdir()
    for m in metas:
        (run / f"t{m['trial_index']}.meta.json").write_text(json.dumps(m))
        ot.append_trial(run, m)
    s = ot.session_row(run)
    assert (s["n_searches"], s["pct_all_found"], str(s["seed"])) == (3, 66.7, "5")


# ---------------------------------------------------------------- the card and the live settings
def test_card_and_live_settings():
    experiments = pytest.importorskip("experiments")
    e = experiments.BY_KEY["forage"]
    assert e.group == "experiments" and e.paradigm_param == "free_move.rail_axis"
    assert [k for k, _ in e.paradigms] == ["frontal", "none"]
    for name in ("forage.dwell_sec", "forage.time_limit_sec", "warm.mode", "warm.fade_trials", "probe.probability"):
        assert live_params.is_live("forage", name), name
    for name in ("forage.seed", "forage.n_zones", "forage.zone_radius_m", "free_move.box.shape"):
        assert not live_params.is_live("forage", name), name               # the environment is the session's
    assert live_params.check_value("free_move.box.shape", "ellipse") is None
    assert live_params.check_value("warm.mode", "sometimes") is not None
