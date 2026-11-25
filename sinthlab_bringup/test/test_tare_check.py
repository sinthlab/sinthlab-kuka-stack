"""Free Move's tare check (helpers/tare_check.py): a hand resting on the apple during the quiet window must not
become the zero, or the arm creeps off by itself once he lets go (Cued Reaches, horizontal, 25 Nov).

    python3 -m pytest sinthlab_bringup/test
"""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import test_centre_out_monitor as stubs  # noqa: E402  (ROS stand-ins)
from sinthlab_bringup.helpers import live_params as lp, tare_check as tc  # noqa: E402
from sinthlab_bringup.actions.admittance_move import AdmittanceMoveAction  # noqa: E402

ARM = np.array([0.4, -1.2, 4.8])          # the arm's own reading at rest at the start pose [N]
HAND = np.array([0.0, 4.0, 1.5])          # a hand resting on the apple [N]
CONFIGS = sorted(p for p in (ROOT / "sinthlab_bringup" / "config").glob("*.yaml")   # the Free Move node's
                 if "free_move:" in p.read_text() and "force_release:" in p.read_text())
CUED = {"pretrain_move_horizontal.yaml", "pretrain_move_vertical.yaml", "pretrain_move_rails.yaml",
        "pretrain_move_free.yaml"}


def run(raws, tol=2.0, tare_max=10.0):
    history, prev, out = [], np.zeros(3), []
    for raw in raws:
        prev, rec = tc.choose(None if raw is None else np.asarray(raw, float), history, tol, tare_max, prev)
        out.append((prev, rec))
    return out


def test_a_hand_on_the_apple_is_not_learned_once_the_run_has_a_usual_tare():
    out = run([ARM, ARM + 0.2, ARM - 0.1, ARM + HAND, ARM + 0.1])
    assert [rec["kept"] for _, rec in out] == ["own", "own", "own", "usual", "own"]
    used, rec = out[3]
    assert np.linalg.norm(used - ARM) < 0.2 and rec["off_n"] > 4.0
    assert rec["raw_n"] == pytest.approx(list(ARM + HAND), abs=0.01)


def test_the_first_trials_and_tol_zero_use_their_own():
    out = run([ARM + HAND, ARM, ARM])                       # nothing to compare with yet
    assert [rec["kept"] for _, rec in out] == ["own", "own", "own"]
    out = run([ARM, ARM, ARM, ARM + HAND], tol=0.0)        # off: as before
    assert out[-1][1]["kept"] == "own" and np.allclose(out[-1][0], ARM + HAND)


def test_the_median_survives_a_few_hands_and_follows_slow_drift():
    raws = [ARM, ARM + HAND, ARM, ARM + HAND, ARM, ARM]     # a hand on it in a third of the trials
    assert np.linalg.norm(run(raws)[-1][0] - ARM) < 1e-9
    drift = [ARM + [0, 0, 0.15 * k] for k in range(20)]     # 3 N over 20 trials, 0.15 N per trial
    assert all(rec["kept"] == "own" for _, rec in run(drift))


def test_pressing_hard_and_no_samples():
    out = run([ARM, ARM, ARM, ARM + [0, 12.0, 0], None])
    assert out[3][1]["kept"] == "usual" and out[3][1]["over_max"] and np.allclose(out[3][0], ARM)
    assert out[4][1]["kept"] == "previous"
    assert len([r for r in out if r[1]["raw_n"] is not None]) == 4   # the hard press is not in the history


@pytest.mark.parametrize("path", CONFIGS, ids=lambda p: p.name)
def test_every_free_move_task_has_the_check_and_the_new_trial_end_settings(path):
    p = next(iter(yaml.safe_load(path.read_text()).values()))["ros__parameters"]
    assert p["free_move"]["tare_tol_n"] == 2.0 < p["free_move"]["deadband_n"]   # under the threshold that moves it
    assert lp.check_value("free_move.tare_tol_n", 2.0) is None
    fr = p["force_release"]
    assert fr["snatch"] is (path.name in CUED)               # Cued Reaches snatch; the others wait for release
    assert lp.check_value("force_release.snatch_after_sec", fr["snatch_after_sec"]) is None
    off_at = p["ring_cue"].get("off_at")
    if path.name == "forage.yaml":
        assert off_at is None                                # its ring shows the warm-zone cue
    else:
        assert off_at == ("move" if path.name in CUED else "go")
        assert off_at in lp.choices_for("ring_cue.off_at")


# ---------------------------------------------------------------- the real admittance loop
def rail_run(monkeypatch, trials, tol=2.0, sec=1.5, rate=100.0, after_go=(0.0, 0.0, 0.0), on_go=None):
    """The admittance on the horizontal rail through several trials: in each, the quiet window's hold (the
    tare) with the hand force given for it, then the go and `sec` of nobody touching. The arm follows its
    equilibrium with a short lag. Returns how far the arm went in each trial [m] and the adm."""
    from sinthlab_bringup.actions import admittance_move as am
    params = {}

    def flat(node, prefix=""):
        for k, v in node.items():
            if isinstance(v, dict):
                flat(v, prefix + k + ".")
            else:
                params[prefix + k] = v
    flat(yaml.safe_load((ROOT / "sinthlab_bringup/config/pretrain_move_horizontal.yaml").read_text())
         ["/**"]["ros__parameters"])
    params["free_move.tare_tol_n"] = tol
    clock = SimpleNamespace(t=100.0)
    monkeypatch.setattr(am.time, "monotonic", lambda: clock.t)
    adm = AdmittanceMoveAction.__new__(AdmittanceMoveAction)
    adm._node, adm._p = stubs.FakeNode(params), "free_move."
    adm.reload()
    adm._leash, adm._tare_max = params["free_move.leash_m"], params["free_move.tare_max_n"]
    adm._rail, adm._shape, adm._rays, adm._box, adm._plane_n = (np.eye(3)[1], -0.15, 0.15), "y", None, None, None
    adm._floor = type("NoFloor", (), {"clamp_xyz": staticmethod(lambda xyz, R: xyz)})()
    adm._tare, adm._tare_history, adm._tare_rec, adm._rest_offset = np.zeros(3), [], {}, np.zeros(3)
    adm._f_used, adm._debug = np.zeros(3), False
    adm._f_filt, adm._v, adm._v_arm, adm._f_hand = np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)
    start = np.array([0.38, 0.01, 1.03])
    arm = {"p": start.copy()}
    adm._fk = lambda q: (lambda T: (T.__setitem__((slice(0, 3), 3), arm["p"]), T)[1])(np.eye(4))
    hand = {"f": np.zeros(3)}
    adm._hand_force = lambda msg, q, T_: ARM + hand["f"]
    adm._publish = lambda eq: None
    msg = SimpleNamespace(measured_joint_position=[0.1] * 7, commanded_joint_position=[0.0] * 7)
    went, dt = [], 1.0 / rate
    for resting in trials:
        arm["p"] = start.copy()
        adm.start()
        hand["f"] = np.asarray(resting, float)
        for _ in range(int(0.5 * rate)):                     # the quiet window (0.5 s): the tare
            clock.t += dt
            adm._on_state(msg)
        adm.enable_motion()
        if on_go is not None:
            on_go(adm)
        hand["f"] = np.asarray(after_go, float)              # he lets go at the go (or pushes: after_go)
        for _ in range(int(sec * rate)):
            clock.t += dt
            adm._on_state(msg)
            arm["p"] = arm["p"] + (adm._eq - arm["p"]) * min(1.0, dt / 0.04)
        went.append(float(np.linalg.norm(arm["p"] - start)))
        adm.stop()
    return went, adm


def test_a_hand_resting_through_the_quiet_window_no_longer_drives_the_arm_off(monkeypatch):
    trials = [np.zeros(3)] * 3 + [HAND]
    went, adm = rail_run(monkeypatch, trials, tol=0.0)        # the old behaviour
    assert max(went[:3]) < 0.002 and went[3] > 0.04          # it crept off along the rail on its own
    went, adm = rail_run(monkeypatch, trials)
    assert max(went) < 0.002                                 # it stays put
    assert adm.tare_record()["kept"] == "usual"


def test_notify_moved_fires_once_when_he_starts_and_stop_drops_it(monkeypatch):
    """ring_cue.off_at move: the go cue's light goes out once the arm has moved 1 cm from where it was at the go."""
    fired = []
    went, adm = rail_run(monkeypatch, [np.zeros(3)], sec=1.0, after_go=(0.0, 8.0, 0.0),
                         on_go=lambda a: a.notify_moved(0.01, lambda: fired.append(round(a.max_excursion_m(), 3))))
    assert went[0] > 0.05 and len(fired) == 1 and 0.01 <= fired[0] < 0.013
    fired.clear()
    rail_run(monkeypatch, [np.zeros(3)], sec=1.0,                       # nobody touches it: no "start"
             on_go=lambda a: a.notify_moved(0.01, lambda: fired.append(1)))
    assert fired == []
    adm.notify_moved(0.01, lambda: fired.append(1))
    adm.stop()
    assert adm._moved_cb is None
