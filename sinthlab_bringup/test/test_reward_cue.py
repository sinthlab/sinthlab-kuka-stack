"""The reward cue is the same in every task: the whole ring white, a colour no cue uses
(actions/ring_state_cue.py; the reward colours of Navin's tasks in their YAMLs).

    python3 -m pytest sinthlab_bringup/test
"""
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import test_centre_out_monitor as stubs  # noqa: E402  (ROS stand-ins)
from sinthlab_bringup.actions.ring_state_cue import RingStateCue  # noqa: E402

CONFIG = ROOT / "sinthlab_bringup" / "config"
WHITE = [0, 0, 0, 255]
FREE_MOVE = ["pretrain_free_move", "reach_map", "pretrain_restricted_move_horizontal",
             "pretrain_restricted_move_vertical", "pretrain_move_horizontal", "pretrain_move_vertical",
             "pretrain_move_free", "colour_rule", "n_choice"]


def params(name):
    return yaml.safe_load((CONFIG / f"{name}.yaml").read_text())["/**"]["ros__parameters"]


def test_every_free_move_task_has_the_white_reward_and_no_cue_is_white():
    for name in FREE_MOVE:
        colours = params(name)["ring_cue"]["colours"]
        assert colours["reward"] == WHITE and "reached" not in colours, name
        assert all(c != WHITE for k, c in colours.items() if k != "reward"), name


def test_navins_tasks_reward_in_white_and_a_timeout_shows_nothing():
    assert params("apple_pluck_impedance")["visual_cue"]["colours"]["snap"] == WHITE
    assert params("virtual_fixtures_params")["visual_cue"]["colours"]["snap"] == WHITE
    maze = params("maze_params")["visual_cue"]["colours"]
    assert maze["reward"] == maze["goal"] == WHITE and not any(maze["timeout"])


def test_the_reward_cue_is_the_whole_ring():
    p = params("colour_rule")
    flat = {"visual_cue.enabled": True, "ring_cue.quarters": [1, 3]}
    flat.update({f"ring_cue.colours.{k}": v for k, v in p["ring_cue"]["colours"].items()})
    node = stubs.FakeNode(flat)
    ring = RingStateCue(node)
    assert ring.path_for("reward", [0, 1, 2, 3]) == "/segments?factor=4&colors=" + ".".join(["0,0,0,255"] * 4)
