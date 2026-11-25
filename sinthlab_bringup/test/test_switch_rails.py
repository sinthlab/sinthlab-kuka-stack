"""Cued Reaches, Rails (switchable): helpers/switch_rails.py, the centre-out monitor following a switch, the
admittance changing rail on reload, the profile's reach and the trial table.

    python3 -m pytest sinthlab_bringup/test
"""
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import test_centre_out_monitor as stubs  # noqa: E402  (ROS stand-ins)
from sinthlab_bringup.helpers import switch_rails as sr, live_params  # noqa: E402
from sinthlab_bringup.actions.centre_out import CentreOutMonitor  # noqa: E402

CFG = ROOT / "sinthlab_bringup" / "config" / "pretrain_move_rails.yaml"


def flat(d, prefix=""):
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(flat(v, f"{prefix}{k}."))
        else:
            out[f"{prefix}{k}"] = v
    return out


def params():
    return flat(next(iter(yaml.safe_load(CFG.read_text()).values()))["ros__parameters"])


def test_the_rail_in_force_and_its_ends():
    p = params()
    assert p["free_move.rail_axis"] == "switch" and p["rails.axis"] == "horizontal"
    assert sr.current(p.__getitem__) == ("y", -0.15, 0.15)
    assert sr.current({**p, "rails.axis": "vertical"}.__getitem__) == ("z", -0.07, 0.07)
    with pytest.raises(ValueError):
        sr.base_axis("diagonal")
    # The dashboard's view of any task's rail: a fixed one, this one, or none.
    assert sr.effective({"free_move.rail_axis": "z", "free_move.rail_min_m": -0.07, "free_move.rail_max_m": 0.07}) \
        == ("z", -0.07, 0.07)
    assert sr.effective({**p, "rails.axis": "vertical"}) == ("z", -0.07, 0.07)
    assert sr.effective({"free_move.rail_axis": "none"}) is None


def test_the_switch_is_live_and_the_ends_per_run():
    assert live_params.is_live("move_rails", "rails.axis")
    assert not live_params.is_live("move_rails", "rails.left_m")       # both rails are checked at start-up
    assert not live_params.is_live("move_vertical", "rails.axis")
    assert live_params.check_value("rails.axis", "vertical") is None
    assert live_params.check_value("rails.axis", "diagonal")
    # Same start pose and CLIK posture as Move Vertical: nothing else changes with the rail.
    v = flat(next(iter(yaml.safe_load((CFG.parent / "pretrain_move_vertical.yaml").read_text()).values()))["ros__parameters"])
    assert params()["move_to_start.target_joint_position"] == v["move_to_start.target_joint_position"]


class SwitchArm(stubs.FakeArm):
    """The admittance with a rail that reload() may change (rails.axis), as actions/admittance_move.py."""

    def __init__(self, node):
        self.node = node
        super().__init__(*self._rail())

    def _rail(self):
        axis, lo, hi = sr.current(lambda n: self.node.params[n])
        return axis, (lo, hi)

    def reload(self):
        self.axis, self.ends = self._rail()


def test_the_centre_out_follows_a_switch_from_the_next_trial():
    p = {k: v for k, v in params().items() if k.startswith(("centre_out.", "rails."))}
    node = stubs.FakeNode(p)
    arm = SwitchArm(node)
    mon = CentreOutMonitor(node, arm)
    node.params["centre_out.target"] = "cued"
    mon.reload()
    assert mon.pick_target() in ("left", "right")
    node.params["rails.axis"] = "vertical"                      # set live; applies at the trial boundary
    arm.reload()
    mon.reload()
    assert mon.mode() == "cued" and mon.pick_target() in ("down", "up")
    goal = mon._target
    assert mon.quarters() == [2 if goal == "down" else 0]
    got = []
    mon.start(lambda end, ok: got.append((end, ok)))
    arm.x = -0.065 if goal == "down" else 0.065                 # within end_tol_m of the cued end
    for tick in node.timers:
        tick()
    assert got == [(goal, True)]


def test_the_profile_reach_sets_all_four_ends():
    sys.path.insert(0, str(ROOT / "experiment_ctrl_gui"))
    profiles = pytest.importorskip("profiles")
    ex = pytest.importorskip("experiments")
    exp = ex.BY_KEY["move_rails"]
    reach = {"reach.left_m": 0.12, "reach.right_m": 0.13, "reach.down_m": 0.05, "reach.up_m": 0.06}
    ctx = params()
    assert profiles.reach_values(exp, reach, ctx) == {"rails.left_m": 0.12, "rails.right_m": 0.13,
                                                     "rails.down_m": 0.05, "rails.up_m": 0.06}
    assert profiles.mapped(exp, ctx)["rails.up_m"] == "reach.up_m"
    assert profiles.from_task(exp, ctx)["reach.left_m"] == 0.15


def test_the_trial_table_names_the_rail():
    from sinthlab_bringup.helpers import outcome_tables as ot
    ev = lambda n, dt, a=None: {"event": n, "arg": a, "t_wall": 1.8e9 + dt}   # noqa: E731
    meta = {"experiment": "move_rails", "trial_index": 1,
            "params": {"centre_out.target": "cued", "rails.axis": "vertical"},
            "events": [ev("trial_start", 0, 1), ev("cue_go", 3), ev("target", 3, "up"), ev("reach", 4, "up"),
                       ev("trial_end", 4, 1)]}
    row = ot.trial_row(meta)
    assert (row["rail"], row["chosen_end"], row["correct"]) == ("vertical", "up", 1)
    fixed = ot.trial_row({**meta, "experiment": "move_horizontal",
                          "params": {"centre_out.target": "cued", "free_move.rail_axis": "y"}})
    assert fixed["rail"] == "horizontal"


def test_the_admittance_changes_rail_on_reload():
    from sinthlab_bringup.actions.admittance_move import AdmittanceMoveAction
    adm = AdmittanceMoveAction.__new__(AdmittanceMoveAction)      # without the robot model and posture checks
    adm._node, adm._p, adm._shape = stubs.FakeNode(params()), "free_move.", "switch"
    adm._rail, adm._open = adm._switch_rail(), (-0.15, 0.0)        # a restricted trial left half the rail closed
    assert adm.rail_axis() == "y" and adm.rail_ends() == (-0.15, 0.15)
    adm._node.params["rails.axis"] = "vertical"
    adm.reload()
    assert adm.rail_axis() == "z" and adm.rail_ends() == (-0.07, 0.07) and adm._open is None
    assert np.allclose(adm._rail[0], [0, 0, 1])
    assert any("vertical rail" in m for _, m in adm._node.log.lines)
