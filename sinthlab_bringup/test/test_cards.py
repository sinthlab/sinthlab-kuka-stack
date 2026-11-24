"""The dashboard's pre-training cards: Free Move, Restricted Move (horizontal / vertical) and Cued Reaches
(horizontal / vertical), each experiment with its own config, launch file and settings.

    python3 -m pytest sinthlab_bringup/test
"""
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))
sys.path.insert(0, str(ROOT / "experiment_ctrl_gui"))

experiments = pytest.importorskip("experiments")
from sinthlab_bringup.helpers import live_params  # noqa: E402

CONFIG, LAUNCH = ROOT / "sinthlab_bringup" / "config", ROOT / "sinthlab_bringup" / "launch"


def params(e):
    return yaml.safe_load((CONFIG / e.params_yaml).read_text())["/**"]["ros__parameters"]


def test_pretraining_cards_and_their_variants():
    cards = {}
    for e in experiments.EXPERIMENTS:
        if e.group == "pretraining":
            cards.setdefault(e.card or e.key, []).append((e.variant, e.key))
    assert cards == {
        "free_move": [("", "free_move")],
        "reach_map": [("", "reach_map")],
        "restricted_move": [("Horizontal", "restricted_move_horizontal"), ("Vertical", "restricted_move_vertical")],
    }
    # Anything organised in trials sits under Experiments, Cued Reaches included.
    assert {e.key for e in experiments.EXPERIMENTS if e.card == "cued_reach"} == {"move_horizontal", "move_vertical"}
    assert all(e.group == "experiments" for e in experiments.EXPERIMENTS if e.card == "cued_reach")
    assert set(experiments.CARDS) == {"restricted_move", "cued_reach"}


def test_every_experiment_has_its_own_config_and_launch_file():
    yamls = [e.params_yaml for e in experiments.EXPERIMENTS]
    assert len(yamls) == len(set(yamls))                     # nothing shared between experiments
    for e in experiments.EXPERIMENTS:
        assert (CONFIG / e.params_yaml).exists() and (LAUNCH / e.launch_file).exists(), e.key
        launch = (LAUNCH / e.launch_file).read_text()
        assert f'"{e.params_yaml}"' in launch, e.key
        if e.node == "free_move_orchestrator":
            assert params(e)["experiment"] == e.key, e.key   # the node records the dashboard's key


def test_restricted_move_is_a_rail_without_a_reach():
    for key, axis in (("restricted_move_horizontal", "y"), ("restricted_move_vertical", "z")):
        e = experiments.BY_KEY[key]
        p = params(e)
        assert p["free_move"]["rail_axis"] == axis and "centre_out" not in p and not e.paradigms
        assert live_params.is_live(key, "session_sec")
    # each rail keeps its own start: vertical 5 cm lower, with its own CLIK posture
    assert params(experiments.BY_KEY["restricted_move_vertical"])["move_to_start"]["target_joint_position"] == \
        params(experiments.BY_KEY["move_vertical"])["move_to_start"]["target_joint_position"]
    assert experiments.BY_KEY["restricted_move_vertical"].clik_nullspace == "clik_nullspace_pretrain_vertical.yaml"


def test_cued_reaches_keep_their_modes():
    for key in ("move_horizontal", "move_vertical"):
        e = experiments.BY_KEY[key]
        assert e.card == "cued_reach" and e.paradigm_param == "centre_out.target"
        assert params(e)["centre_out"]["enabled"] is True


def test_validator_and_live_view_know_every_free_move_task():
    sys.path.insert(0, str(ROOT / "analysis"))
    import validate_recording as vr
    src = (ROOT / "experiment_ctrl_gui" / "live_view.py").read_text()
    for e in experiments.EXPERIMENTS:
        if e.node == "free_move_orchestrator":
            assert e.key in vr.EXTRA and f'"{e.key}"' in src, e.key


def test_live_view_draws_the_new_shapes():
    import numpy as np
    import live_view
    cls = next(v for v in vars(live_view).values() if isinstance(v, type) and hasattr(v, "_free_move"))
    a = np.array([0.38, 0.01, 0.98])
    shapes = []
    cls._free_move(None, {"free_move.rail_axis": "rays", "free_move.ray_layout": "3", "free_move.ray_length_m": 0.07},
                   a, shapes.append)
    assert [s["label"] for s in shapes if s["kind"] == "marker"] == ["down end", "30deg end", "150deg end"]
    shapes = []
    cls._free_move(None, {"free_move.rail_axis": "frontal", "centre_out.free_reach_m": 0.12}, a, shapes.append)
    assert [s["id"] for s in shapes if s["kind"] == "marker"] == ["end:left", "end:right"]
