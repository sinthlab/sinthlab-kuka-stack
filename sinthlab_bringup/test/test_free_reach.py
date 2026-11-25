"""Cued Reaches, free (3-D): helpers/free_reach.py (scoring, schedule, geometry), actions/free_reach.py on a fake arm,
AdmittanceMoveAction.displace(), its trials.csv columns and its card.

    python3 -m pytest sinthlab_bringup/test
"""
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
from sinthlab_bringup.helpers import live_params, outcome_tables as ot, free_reach as pt  # noqa: E402
from sinthlab_bringup.actions import free_reach as fra  # noqa: E402
from sinthlab_bringup.actions.admittance_move import AdmittanceMoveAction  # noqa: E402

YAML = ROOT / "sinthlab_bringup/config/pretrain_move_free.yaml"


def yaml_params():
    d = yaml.safe_load(YAML.read_text())["/**"]["ros__parameters"]
    out = {}

    def flat(node, prefix=""):
        for k, v in node.items():
            if isinstance(v, dict):
                flat(v, prefix + k + ".")
            else:
                out[prefix + k] = v
    flat(d)
    return out


def yaml_box():
    p = yaml_params()
    return (np.array([p[f"free_move.box.{a}_min_m"] for a in "xyz"]),
            np.array([p[f"free_move.box.{a}_max_m"] for a in "xyz"]))


# ---------------------------------------------------------------- scoring
def test_directions_and_rotation():
    assert np.allclose(pt.rotate(pt.unit("right"), 90), pt.unit("up"))          # counter-clockwise as he sees it
    assert np.allclose(pt.rotate(pt.unit("down"), 270), pt.unit("left"))
    assert pt.QUARTER == {"up": 0, "right": 1, "down": 2, "left": 3}             # the ring's quarters


def test_a_pull_ends_at_its_distance_and_is_scored_against_the_cone():
    assert pt.crossing([0.03, 0.0], 0.05, "right", 15) is None                   # not there yet
    assert pt.crossing([0.0, -0.06], 0.05, None, 15) == (True, 270.0, None)      # no cue: any direction pays
    ok, ang, err = pt.crossing([0.05, 0.01], 0.05, "right", 15)                  # 11 deg above right
    assert ok and err == pytest.approx(11.3, abs=0.1)
    ok, _, err = pt.crossing([0.05, -0.02], 0.05, "right", 15)                   # 22 deg below: outside
    assert not ok and err < 0
    ok, _, err = pt.crossing([-0.01, -0.05], 0.05, "down", 15)                   # left of down = clockwise
    assert ok and err < 0


def test_an_uncorrected_bump_misses_the_target():
    s = pt.ReachSettings()
    d = s.bump_extent_m() * pt.rotate(pt.unit("right"), 90) + s.distance_m * pt.unit("right")   # bumped, carried on straight
    assert not pt.crossing(d, s.distance_m, "right", s.cone_deg)[0]


# ---------------------------------------------------------------- the schedule
def test_blocks_balance_directions_perturbations_and_angles():
    s = pt.ReachSettings(cue="cued", perturb="bump")
    sched = pt.ReachSchedule(s, random.Random(1))
    plans = [sched.next() for _ in range(4 * s.block_trials)]
    assert {p.block for p in plans} == {1}
    for d in s.directions:
        mine = [p for p in plans if p.direction == d]
        assert len(mine) == 6 and sum(p.perturb == "bump" for p in mine) == 2      # round(0.33 x 6)
    angles = [p.angle_deg for p in plans if p.perturb]
    assert angles.count(90.0) == angles.count(270.0) == 4


@pytest.mark.parametrize("p", [0.33, 0.5])
def test_perturbed_trials_never_run_together(p):
    sched = pt.ReachSchedule(pt.ReachSettings(cue="cued", perturb="jump", probability=p), random.Random(2))
    plans = [sched.next() for _ in range(240)]                                   # ten blocks, across their edges
    flags = [x.perturb is not None for x in plans]
    assert not any(a and b for a, b in zip(flags, flags[1:]))
    assert sum(flags) == round(p * 6) * 4 * 10                                    # still balanced


def test_too_many_perturbed_trials_raise_the_cap_as_little_as_needed():
    sched = pt.ReachSchedule(pt.ReachSettings(cue="cued", perturb="bump", probability=0.83), random.Random(5))   # 5 of 6
    flags = [sched.next().perturb is not None for _ in range(24)]
    runs, run = [], 0
    for f in flags:
        run = run + 1 if f else 0
        runs.append(run)
    assert sum(flags) == 20 and max(runs) <= 5


def test_a_change_starts_a_new_block_and_either_has_no_cue():
    s = pt.ReachSettings(cue="cued", perturb="jump")
    sched = pt.ReachSchedule(s, random.Random(3))
    sched.next()
    s.cue = "either"
    p = sched.next()
    assert (p.direction, p.perturb, p.block) == (None, None, 2)
    assert sched.next().block == 2                                               # no cue: one block until a change
    s.cue, s.perturb, s.probability = "cued", "bump", 0.0
    assert all(sched.next().perturb is None for _ in range(24))                  # cued pulls only


# ---------------------------------------------------------------- settings and the box
def test_settings_checks():
    assert pt.settings_problem(pt.ReachSettings()) is None
    assert "bump_at_m" in pt.settings_problem(pt.ReachSettings(bump_at_m=0.045))  # no room left to correct
    assert "unknown" in pt.settings_problem(pt.ReachSettings(directions=["sideways"]))
    assert pt.settings_problem(pt.ReachSettings(cue="pluck"))
    assert pt.settings_problem(pt.ReachSettings(perturb="shove"))
    assert pt.ReachSettings(cue="either", perturb="bump").kind() is None     # nothing to perturb against


def test_the_default_pulls_fit_the_box_and_longer_ones_need_fewer_directions():
    lo, hi = yaml_box()
    s = fra.settings_from(lambda n: yaml_params()[n])
    assert pt.settings_problem(s) is None and pt.box_problem(s, lo, hi) is None
    s.distance_m = 0.07
    assert "outside the box" in pt.box_problem(s, lo, hi)                        # up runs out of room
    s.directions = ["right", "left", "down"]
    s.distance_m = 0.06
    assert pt.box_problem(s, lo, hi) is None
    s.directions, s.distance_m = ["right", "left"], 0.12
    assert pt.box_problem(s, lo, hi) is None


# ---------------------------------------------------------------- the monitor on a fake arm
class Clock:
    def __init__(self):
        self.t = 100.0

    def monotonic(self):
        return self.t


class FakeArm:
    def __init__(self):
        self.pos, self.force, self.disp, self.box_ = np.zeros(3), 0.0, None, yaml_box()
        self.vel = np.zeros(3)

    def shape(self):
        return "none"

    def box(self):
        return self.box_

    def rest_offset_from_anchor(self):
        return self.pos.copy()

    def displacing(self):
        return self.disp is not None and self.disp[2] == "displace"

    def pushing(self):
        return self.disp is not None and self.disp[2] == "push"

    def perturbing(self):
        return self.disp is not None

    def hand_force_n(self):
        return self.force

    def arm_velocity(self):
        return self.vel

    def displace(self, off, sec, on_done=None):
        self.disp = (np.asarray(off), on_done, "displace", sec)
        return True

    def push(self, force, sec, on_done=None):
        self.disp = (np.asarray(force), on_done, "push", sec)
        return True

    def finish(self, damping=50.0):
        """The jump lands; the bump, not resisted, moves the apple force x duration / damping."""
        vec, on_done, kind, sec = self.disp
        self.pos = self.pos + (vec if kind == "displace" else vec * sec / damping)
        self.disp = None
        on_done()


def monitor(monkeypatch, **over):
    clock = Clock()
    monkeypatch.setattr(fra, "time", clock)
    params = {**yaml_params(), **over}
    node, arm, marks = stubs.FakeNode(params), FakeArm(), []
    mon = fra.FreeReachMonitor(node, arm, lambda e, a=None: marks.append((e, a)), rng=random.Random(4))
    return node, arm, mon, marks, clock


def tick(node):
    for t in node.timers:
        t()


def run_trial(node, arm, mon, clock, plan):
    got = []
    mon.plan = plan
    mon.start(lambda ok, ang: got.append((ok, ang)))
    return got


def test_no_cue_rewards_any_reach(monkeypatch):
    node, arm, mon, marks, clock = monitor(monkeypatch)
    got = run_trial(node, arm, mon, clock, pt.TrialPlan("either", None))
    assert mon.quarters() == [0, 1, 2, 3] and mon.target() == "any"
    arm.pos = np.array([0.0, -0.03, -0.045])                  # down-left, 5.4 cm
    tick(node)
    assert got == [(True, pytest.approx(236.3, abs=0.1))]
    assert [e for e, _ in marks] == ["grip"]                  # moved > grip.move_m counts as the grip


def test_a_bump_comes_after_the_grip_and_the_target_stays(monkeypatch):
    node, arm, mon, marks, clock = monitor(monkeypatch, **{"free_reach.cue": "cued", "perturb.kind": "bump"})
    plan = pt.TrialPlan("cued", "right", "bump", 90.0, force_n=5.0)
    got = run_trial(node, arm, mon, clock, plan)
    assert mon.quarters() == [1]
    arm.force = 5.0
    tick(node)
    clock.t += 0.05
    tick(node)
    assert marks[-1][0] == "grip"
    arm.pos = np.array([0.0, 0.026, 0.0])                     # past bump_at_m along right
    tick(node)
    assert marks[-1] == ("bump", 90.0) and arm.pushing() and not arm.displacing()
    assert np.allclose(arm.disp[0], [0, 0, 5.0])              # a 5 N push straight up: perpendicular to right
    assert arm.disp[3] == pytest.approx(0.15) and mon.result()["force_n"] == [0.0, 0.0, 5.0]
    tick(node)
    assert not got                                            # nothing is judged mid-bump
    arm.finish()                                              # not resisted: 5 N x 0.15 s / 50 = 1.5 cm up
    assert marks[-1][0] == "bump_end" and np.allclose(arm.pos, [0, 0.026, 0.015])
    arm.pos = np.array([0.0, 0.05, 0.015])                    # carried on without correcting: 16.7 deg off
    tick(node)
    assert got and got[0][0] is False
    assert mon.result()["perturbed"] == 1 and mon.result()["error_deg"] == pytest.approx(16.7, abs=0.1)


def test_a_jump_before_the_grip_moves_where_the_pull_starts(monkeypatch):
    node, arm, mon, marks, clock = monitor(monkeypatch, **{"free_reach.cue": "cued", "perturb.kind": "jump"})
    got = run_trial(node, arm, mon, clock, pt.TrialPlan("cued", "down", "jump", 270.0, 0.015))
    clock.t += 0.35                                            # past the latest jump delay
    tick(node)
    assert marks[-1] == ("jump", 270.0)
    assert np.allclose(arm.disp[0], [0, -0.015, 0])            # clockwise from down = left (-Y)
    arm.finish()
    tick(node)
    assert "grip" not in [e for e, _ in marks]                 # the jump itself is not his grip
    arm.pos = arm.pos + [0, 0, -0.05]                          # straight down from where it landed
    tick(node)
    assert got == [(True, 270.0)]


def test_no_jump_once_he_has_gripped(monkeypatch):
    node, arm, mon, marks, clock = monitor(monkeypatch, **{"free_reach.cue": "cued", "perturb.kind": "jump"})
    run_trial(node, arm, mon, clock, pt.TrialPlan("cued", "left", "jump", 90.0, 0.015))
    arm.pos = np.array([0.0, -0.01, 0.0])                      # already moving it
    tick(node)
    tick(node)
    assert [e for e, _ in marks] == ["grip", "jump_skipped"]
    clock.t += 1.0
    tick(node)
    assert not arm.displacing() and mon.result()["perturbed"] == 0


def test_live_changes_are_checked_against_the_box(monkeypatch):
    node, arm, mon, marks, clock = monitor(monkeypatch)
    assert mon.check_live("free_reach.distance_m", 0.09, {}) is not None
    assert mon.check_live("free_reach.distance_m", 0.04, {}) is None
    assert mon.check_live("perturb.kind", "bump", {}) is None
    assert mon.check_live("free_reach.cue", "cued", {}) is None
    assert mon.check_live("free_reach.cue_delay_min_sec", 2.0, {}) is not None        # above the max (0.3)
    assert mon.check_live("free_reach.cue_delay_max_sec", 2.0, {}) is None


def test_depth_is_free_and_not_scored(monkeypatch):
    node, arm, mon, marks, clock = monitor(monkeypatch, **{"free_reach.cue": "cued", "perturb.kind": "jump", "perturb.probability": 0.0})
    got = run_trial(node, arm, mon, clock, pt.TrialPlan("cued", "right"))
    arm.pos = np.array([0.04, 0.0, 0.0])                       # pushed 4 cm toward himself only
    tick(node)
    assert [e for e, _ in marks] == ["grip"] and not got       # a grip, but no pull yet
    arm.pos = np.array([-0.03, 0.05, 0.005])                   # then away from himself while pulling right
    tick(node)
    assert got == [(True, pytest.approx(5.7, abs=0.1))]        # scored on its sideways part only


def test_3d_or_frontal_only():
    p = yaml_params()
    assert p["free_move.rail_axis"] == "none"
    assert pt.params_problem({**p, "free_move.rail_axis": "frontal"}) is None
    assert "rail_axis" in pt.params_problem({**p, "free_move.rail_axis": "y"})


def test_needs_a_box(monkeypatch):
    arm = FakeArm()
    arm.shape = lambda: "y"
    with pytest.raises(ValueError):
        fra.FreeReachMonitor(stubs.FakeNode(yaml_params()), arm, lambda *a: None)


# ---------------------------------------------------------------- the admittance's displacement
def test_displace_drives_the_equilibrium_then_hands_back(monkeypatch):
    from sinthlab_bringup.actions import admittance_move as am
    clock = Clock()
    monkeypatch.setattr(am.time, "monotonic", clock.monotonic)
    adm = AdmittanceMoveAction.__new__(AdmittanceMoveAction)
    adm._rail, adm._open, adm._shape, adm._rays, adm._box = None, None, "frontal", None, yaml_box()
    adm._plane_n = np.array([1.0, 0.0, 0.0])
    adm._anchor = np.eye(4)
    adm._anchor[0:3, 3] = [0.38, 0.01, 0.98]
    adm._floor = type("NoFloor", (), {"clamp_xyz": staticmethod(lambda xyz, R: xyz)})()
    adm._eq, adm._moving, adm._disp = adm._anchor[0:3, 3].copy(), True, None
    adm._v, adm._f_filt = np.ones(3), np.ones(3)
    published, done = [], []
    adm._publish = published.append
    assert adm.displace([0.0, 0.0, -0.015], 0.1, on_done=lambda: done.append(1))
    assert not adm.displace([0.0, 0.01, 0.0], 0.1)                 # one at a time
    clock.t += 0.05
    adm._displace_tick(clock.t)
    assert published[-1][2] == pytest.approx(0.98 - 0.0075)        # halfway, on the minimum-jerk path
    clock.t += 0.06
    adm._displace_tick(clock.t)
    assert published[-1][2] == pytest.approx(0.965) and done == [1] and not adm.displacing()
    assert not np.any(adm._v)                                      # no speed carried over


def test_min_jerk_profile():
    assert pt.min_jerk(0) == 0 and pt.min_jerk(1) == 1 and pt.min_jerk(0.5) == pytest.approx(0.5)
    assert pt.min_jerk_peak_speed(0.015, 0.1) == pytest.approx(0.281, abs=0.001)


# ---------------------------------------------------------------- trials.csv / sessions.csv
def sidecar(k, plan, outcome, events=(), err=None):
    ev = lambda n, dt, a=None: {"event": n, "arg": a, "t_wall": 1e9 + 10 * k + dt}   # noqa: E731
    evs = [ev("trial_start", 0, k), ev("cue_go", 3)] + [ev(n, dt, a) for n, dt, a in events]
    if outcome:
        evs.append(ev(outcome[0], 4, outcome[1]))
    evs.append(ev("trial_end", 4.1, k))
    return {"experiment": "move_free", "trial_index": k, "subject": "M1", "params": {"free_reach.cue": plan["cue"]},
            "free_reach": plan, "free_reach_result": {"error_deg": err}, "events": evs}


def test_columns_and_session(tmp_path):
    std = {"cue": "cued", "direction": "right", "perturb": None, "angle_deg": None, "size_m": 0.0, "block": 1,
           "cue_delay_sec": 0.3}
    bump = {**std, "perturb": "bump", "angle_deg": 90.0, "size_m": 0.015}
    metas = [sidecar(1, std, ("reach", 2.0), [("grip", 3.25, 4.0)], err=2.0),
             sidecar(2, bump, ("reach_wrong", 17.0), [("grip", 3.3, 4.0), ("bump", 3.5, 90.0), ("bump_end", 3.6, None)],
                     err=17.0),
             sidecar(3, bump, ("reach", 5.0), [("grip", 3.2, 4.0), ("bump", 3.45, 90.0), ("bump_end", 3.55, None)],
                     err=5.0)]
    row = ot.trial_row(metas[1])
    assert (row["paradigm"], row["target"], row["perturbation"], row["perturbed"], row["correct"]) == \
        ("cued", "right", "bump", 1, 0)
    assert (row["grip_s"], row["perturb_s"], row["error_deg"], row["success"]) == (0.3, 0.5, 17.0, 0)
    run = tmp_path / "expt_p"
    run.mkdir()
    for m in metas:
        (run / f"t{m['trial_index']}.meta.json").write_text(json.dumps(m))
        ot.append_trial(run, m)
    s = ot.session_row(run)
    assert (s["n_reaches"], s["pct_correct"], s["pct_correct_standard"], s["pct_correct_perturbed"]) == \
        (3, 66.7, 100.0, 50.0)
    assert s["median_grip_s"] == 0.25


# ---------------------------------------------------------------- the card and the live settings
def test_card_and_live_settings():
    experiments = pytest.importorskip("experiments")
    e = experiments.BY_KEY["move_free"]
    assert e.group == "experiments" and e.card == "cued_reach" and e.paradigm_param == "free_reach.cue"
    assert [k for k, _ in e.paradigms] == ["either", "cued"]
    assert e.clik_nullspace == "clik_nullspace_pretrain_vertical.yaml"
    for name in yaml_params():
        if name.split(".")[0] in ("free_reach", "perturb", "grip"):
            assert live_params.is_live("move_free", name), name                # every task setting, mid-run
    assert not live_params.is_live("move_free", "free_move.box.z_max_m")       # the box is per-run
    assert live_params.paradigm_of("perturb.bump_at_m") == ("cued",)
    assert live_params.check_value("perturb.kind", "jump") is None
    assert live_params.check_value("free_reach.cue", "cued") is None
    assert live_params.check_value("perturb.jump_sec", 0.01) is not None        # too fast
    start = yaml_params()["move_to_start.target_joint_position"]
    nchoice = yaml.safe_load((ROOT / "sinthlab_bringup/config/n_choice.yaml").read_text())["/**"]["ros__parameters"]
    assert start == nchoice["move_to_start"]["target_joint_position"]          # N-Choice's start and posture


def test_the_dashboard_check_matches_the_node():
    p = yaml_params()
    assert pt.params_problem(p) is None
    assert "outside the box" in pt.params_problem({**p, "free_reach.distance_m": 0.09})
    assert pt.params_problem({**p, "free_reach.cue_delay_min_sec": 1.0})          # above the max
    assert pt.params_problem({"free_move.damping_ns_per_m": 50}) is None     # not a free reach: nothing to check


# ---------------------------------------------------------------- the bump's depth component
def test_depth_sign_follows_his_depth_movement():
    rng = random.Random(0)
    assert pt.depth_sign("against", 0.05, 0.0, 0.02, rng) == (-1, "velocity")     # moving toward himself -> away
    assert pt.depth_sign("with", -0.05, 0.0, 0.02, rng) == (-1, "velocity")
    assert pt.depth_sign("against", 0.01, -0.012, 0.02, rng) == (1, "displacement")  # slow: where it has gone
    assert pt.depth_sign("with", 0.0, 0.001, 0.02, rng)[1] == "random"
    assert pt.depth_sign("toward", -0.3, 0.0, 0.02, rng) == (1, "fixed")
    assert pt.depth_room(1, 0.02, -0.05, 0.03) == pytest.approx(0.005)              # 3 cm box face, 5 mm margin
    assert pt.depth_room(-1, -0.05, -0.05, 0.03) == 0.0


def test_bump_blocks_balance_angles_and_depth_rules():
    sched = pt.ReachSchedule(pt.ReachSettings(cue="cued", perturb="bump"), random.Random(6))
    plans = [p for p in (sched.next() for _ in range(24)) if p.perturb]
    combos = sorted((p.angle_deg, p.depth_rule) for p in plans)
    assert len(plans) == 8 and all(combos.count(c) == 2 for c in set(combos)) and len(set(combos)) == 4
    jumps = pt.ReachSchedule(pt.ReachSettings(cue="cued", perturb="jump"), random.Random(6))
    assert all(p.depth_rule is None for p in (jumps.next() for _ in range(24)))     # jumps stay in the plane
    flat = pt.ReachSchedule(pt.ReachSettings(cue="cued", perturb="bump", bump_depth_force_n=0.0), random.Random(6))
    assert all(p.depth_rule is None for p in (flat.next() for _ in range(24)))
    assert "unknown" in pt.settings_problem(pt.ReachSettings(bump_depth_rules=["sideways"]))


def test_a_bump_against_his_depth_movement(monkeypatch):
    node, arm, mon, marks, clock = monitor(monkeypatch, **{"free_reach.cue": "cued", "perturb.kind": "bump"})
    run_trial(node, arm, mon, clock, pt.TrialPlan("cued", "down", "bump", 90.0, depth_rule="against", force_n=5.0))
    arm.pos, arm.vel = np.array([0.01, 0.0, -0.026]), np.array([0.06, 0.0, -0.2])   # pulling down and toward himself
    tick(node)
    tick(node)
    assert marks[-1] == ("bump", 90.0)
    assert np.allclose(arm.disp[0], [-3.0, 5.0, 0.0])          # 3 N away from him, 5 N right of down
    r = mon.result()
    assert (r["depth_dir"], r["depth_force_n"], r["depth_basis"], r["depth_speed_mps"]) == ("away", 3.0, "velocity", 0.06)
    assert r["depth_m"] == pytest.approx(0.009)                 # 3 N x 0.15 s / 50 N s/m, if not resisted


def test_the_depth_is_cut_short_at_the_box(monkeypatch):
    node, arm, mon, marks, clock = monitor(monkeypatch, **{"free_reach.cue": "cued", "perturb.kind": "bump"})
    run_trial(node, arm, mon, clock, pt.TrialPlan("cued", "right", "bump", 270.0, depth_rule="with", force_n=5.0))
    arm.pos, arm.vel = np.array([0.02, 0.026, 0.0]), np.array([0.1, 0.2, 0.0])     # 2 cm toward him: 5 mm of room
    tick(node)
    tick(node)
    # the 3 N would move it 9 mm: scaled to what fits, 5 mm
    assert arm.disp[0][0] == pytest.approx(3.0 * 5 / 9, abs=0.01) and mon.result()["depth_m"] == pytest.approx(0.005)


def test_no_depth_in_the_frontal_plane(monkeypatch):
    node, arm, mon, marks, clock = monitor(monkeypatch, **{"free_reach.cue": "cued", "perturb.kind": "bump"})
    arm.shape = lambda: "frontal"
    run_trial(node, arm, mon, clock, pt.TrialPlan("cued", "right", "bump", 90.0, depth_rule="with", force_n=5.0))
    arm.pos, arm.vel = np.array([0.0, 0.026, 0.0]), np.array([0.1, 0.2, 0.0])
    tick(node)
    tick(node)
    assert arm.disp[0][0] == 0.0 and mon.result()["depth_dir"] is None


def test_depth_columns():
    plan = {"cue": "cued", "direction": "down", "perturb": "bump", "angle_deg": 90.0, "size_m": 0.015, "block": 1,
            "cue_delay_sec": 0.3, "depth_rule": "against"}
    meta = sidecar(1, plan, ("reach", 268.0), [("grip", 3.2, 4.0), ("bump", 3.5, 90.0), ("bump_end", 3.6, None)])
    meta["free_reach_result"].update(depth_dir="away", depth_m=0.01, depth_basis="velocity", depth_speed_mps=0.06)
    row = ot.trial_row(meta)
    assert (row["depth_rule"], row["depth_dir"], row["depth_m"], row["depth_basis"], row["depth_speed_mps"]) == \
        ("against", "away", 0.01, "velocity", 0.06)


# ---------------------------------------------------------------- the admittance's push (the bump), simulated
def push_run(monkeypatch, hand_y, push=None, sec=0.6, rate=100.0):
    """The real admittance loop on a simulated arm that follows its equilibrium with a short lag (the cabinet
    spring), the hand pressing hand_y N along +Y throughout. push = (force_xyz, duration) fired at 0.1 s.
    Returns the equilibrium's y over time [m, from the start], the push's on_done count and the adm."""
    from types import SimpleNamespace
    from sinthlab_bringup.actions import admittance_move as am
    clock = Clock()
    monkeypatch.setattr(am.time, "monotonic", clock.monotonic)
    adm = AdmittanceMoveAction.__new__(AdmittanceMoveAction)
    adm._node, adm._p = stubs.FakeNode(yaml_params()), "free_move."
    adm.reload()
    adm._leash = float(adm._node.get_parameter("free_move.leash_m").value)
    adm._rail, adm._open, adm._shape, adm._rays, adm._box = None, None, "none", None, yaml_box()
    adm._plane_n = None
    adm._anchor = np.eye(4)
    adm._anchor[0:3, 3] = [0.38, 0.01, 0.98]
    adm._floor = type("NoFloor", (), {"clamp_xyz": staticmethod(lambda xyz, R: xyz)})()
    adm._active = adm._moving = True
    adm._eq = adm._anchor[0:3, 3].copy()
    arm = adm._eq.copy()
    for name, v in dict(_f_filt=np.zeros(3), _f_used=np.zeros(3), _v=np.zeros(3), _tare=np.zeros(3),
                        _rest_offset=np.zeros(3), _pushing=False, _v_arm=np.zeros(3), _quiet=0.0, _low_t=0.0,
                        _settle_until=0.0, _p_prev=None, _t_prev=None, _p_go=None, _max_exc=0.0,
                        _f_hand=np.zeros(3), _disp=None, _push=None, _vp_last=np.zeros(3), _debug=False).items():
        setattr(adm, name, v)
    T = np.eye(4)
    adm._fk = lambda q: (T.__setitem__((slice(0, 3), 3), arm), T.copy())[1]
    adm._hand_force = lambda msg, q, T_: np.array([0.0, hand_y, 0.0])
    adm._publish = lambda eq: None
    msg = SimpleNamespace(measured_joint_position=[0.1] * 7, commanded_joint_position=[0.1] * 7)
    done, ys, dt = [], [], 1.0 / rate
    for k in range(int(sec * rate)):
        clock.t += dt
        if push is not None and k == int(0.1 * rate):
            assert adm.push(push[0], push[1], on_done=lambda: done.append(clock.t))
            assert not adm.push(push[0], push[1])                  # one at a time
        adm._on_state(msg)
        arm = arm + (adm._eq - arm) * min(1.0, dt / 0.04)          # the arm lags its equilibrium ~40 ms
        ys.append(float(adm._eq[1] - adm._anchor[1, 3]))
    return np.array(ys), done, adm


def test_a_bump_alone_moves_the_apple_force_times_time_over_damping(monkeypatch):
    ys, done, adm = push_run(monkeypatch, 0.0, push=([0.0, 5.0, 0.0], 0.15))
    # 5 N x 0.15 s / 50 N s/m = 15 mm by the end of the pulse (0.1 s + 0.15 s)
    assert ys[int(0.25 * 100) - 1] == pytest.approx(0.015, abs=0.002)
    # With no hand on the apple at all, the let-go glide then settles the equilibrium where the arm got to
    # (it lags ~40 ms): a little less, and it stays there. Mid-reach his hand is on it (the next test).
    assert 0.010 < ys[-1] <= 0.016 and abs(ys[-1] - ys[-10]) < 1e-4
    assert len(done) == 1 and not adm.pushing()
    v = np.diff(ys) * 100.0
    assert v.max() == pytest.approx(0.1, abs=0.01)                 # a steady 0.1 m/s during it: no build-up


def test_a_bump_adds_the_same_push_whatever_the_hand_does(monkeypatch):
    """Linear on top of the hand's own movement (not through its threshold or light-force damping): the
    difference the bump makes is F x T / damping, pushing with it or against it."""
    for hand in (8.0, -8.0):
        alone, _, _ = push_run(monkeypatch, hand)
        bumped, _, _ = push_run(monkeypatch, hand, push=([0.0, 5.0, 0.0], 0.15))
        assert bumped[-1] - alone[-1] == pytest.approx(0.015, abs=0.002), hand


def test_pushing_back_opposes_a_bump_at_once(monkeypatch):
    still, _, _ = push_run(monkeypatch, 0.0, push=([0.0, 5.0, 0.0], 0.15))
    resisted, _, _ = push_run(monkeypatch, -10.0, push=([0.0, 5.0, 0.0], 0.15), sec=0.3)
    i = int(0.1 * 100) + 15                                         # the end of the pulse
    assert resisted[i] - resisted[int(0.1 * 100)] < 0.0 < still[i] - still[int(0.1 * 100)]
