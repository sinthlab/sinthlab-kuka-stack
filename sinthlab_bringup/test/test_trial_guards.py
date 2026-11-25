"""Regression cases for choice deadlines, force release and first-run preflight."""
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

import test_centre_out_monitor as stubs
from sinthlab_bringup.helpers import live_params, smartpad_link
from sinthlab_bringup.helpers.start_grid import StartGrid
from sinthlab_bringup.helpers.stiffness_profiles import SMARTPAD_PROFILES, required_profile

ROOT = Path(__file__).resolve().parents[2]


def test_release_requires_continuous_post_outcome_samples(monkeypatch):
    import geometry_msgs.msg as msg
    monkeypatch.setattr(msg, "WrenchStamped", object, raising=False)
    from sinthlab_bringup.actions import force_release_waiter as release
    node = stubs.FakeNode({"update_rate": 100, "release_force_threshold_newton": 3,
                           "release_duration_sec": 1, "debug_log_enabled": False, "debug_log_rate_hz": 0})
    node.create_subscription = Mock()
    done = Mock()
    waiter = release.ForceReleaseWaiter(node, on_complete=done)
    def sample(t, x=0):
        monkeypatch.setattr(release.time, "monotonic", lambda: t)
        waiter._on_wrench(SimpleNamespace(wrench=SimpleNamespace(force=SimpleNamespace(x=x, y=0, z=0))))
    sample(0)
    waiter.start()
    sample(1)
    sample(3)  # stale low reading cannot count the gap
    for i in range(1, 96):
        sample(3 + i / 100)
    sample(3.96, 5)  # grasp breaks the low interval
    for i in range(100):
        sample(4 + i / 100)
    done.assert_not_called()
    sample(5)
    sample(5.01)
    done.assert_called_once()


def test_full_cabinet_diagonal_and_report_age(monkeypatch):
    link = smartpad_link.SmartPadLink.__new__(smartpad_link.SmartPadLink)
    link._lock = threading.Lock()
    link._last_report = 10
    monkeypatch.setattr(smartpad_link.time, "monotonic", lambda: 11)
    expected = SMARTPAD_PROFILES[required_profile("n_choice")]
    for settings in [None, {}, {"stiffness": SMARTPAD_PROFILES["Uniform 1000"]}]:
        link._settings = settings
        assert link.stiffness_problem(expected)
    link._settings = {"stiffness": expected}
    assert link.stiffness_problem(expected) is None
    monkeypatch.setattr(smartpad_link.time, "monotonic", lambda: 14)
    assert link.stiffness_problem(expected)


def test_geometry_edits_check_both_sides_of_each_constraint():
    rays = {"free_move.rail_axis": "rays", "free_move.ray_length_m": .02, "n_choice.end_tol_m": .015}
    assert live_params.check_value("n_choice.end_tol_m", .02, rays)
    assert live_params.check_value("free_move.ray_length_m", .02, {**rays, "n_choice.end_tol_m": .03})
    plane = {"free_move.rail_axis": "frontal", "free_move.box.preset": "standard", "centre_out.free_reach_m": .12}
    assert live_params.check_value("free_move.box.preset", "most_x", plane)
    assert live_params.check_value("centre_out.free_reach_m", .2, plane)
    assert live_params.choice_geometry_problem(plane) is None


def test_directed_rail_rejects_shallow_elbow_inside_segment():
    grid = StartGrid.__new__(StartGrid)
    grid._fk = lambda q: np.eye(4)
    grid.margin_deg = lambda q: 40
    grid.sigma_min = lambda q: .2
    def ik(q, pos, rot):
        q = q.copy()
        if .019 < pos[1] < .021:
            q[3] = np.radians(25)
        return q, 0
    grid.ik = ik
    assert "straightens the elbow" in grid.directed_rail_problem(
        [0, 0, 0, 65, 0, 0, 0], np.array([0, 1, 0]), 0, .07)


def test_existing_rail_yaml_keeps_both_halves_open():
    node, arm, _ = stubs.horizontal(target="cued")
    del node.params["centre_out.max_repeats"]
    monitor = stubs.CentreOutMonitor(node, arm)
    monitor.pick_target()
    assert monitor.open_span() is None
    assert monitor._picker.max_repeats == 0


@pytest.mark.parametrize("ending,valid", [("choice", True), ("response_timeout", True),
                                          ("smartpad_mismatch", True), ("missing_end", False)])
def test_n_choice_recording_endings(ending, valid, monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "analysis"))
    import validate_recording as validator
    events = ["trial_start", "at_start", "fixture_active"]
    meta = {}
    if ending != "smartpad_mismatch":
        events += ["cue_go", "armed"]
        meta["n_choice"] = {"p": {"right": .9, "left": .9}}
    events += ["choice" if ending == "missing_end" else ending]
    if ending != "missing_end":
        events += ["trial_end"]
    report = validator.Report("sample")
    validator.check_admittance_events(report, "n_choice", events, meta)
    assert (not report.bad) == valid


def _free_move_node_class(monkeypatch):
    """The Free Move orchestrator's class, its hardware actions substituted."""
    import importlib.util
    from unittest.mock import MagicMock
    for module in ("actions.move_to_position_joint_space", "actions.park_move", "actions.switch_controller",
                   "actions.safety_stop_monitor", "actions.force_release_waiter", "actions.audio_cue",
                   "actions.visual_cue", "actions.ring_state_cue", "actions.quiet_gate", "actions.trial_recorder",
                   "helpers.experiment_control"):
        monkeypatch.setitem(sys.modules, "sinthlab_bringup." + module, MagicMock())
    path = ROOT / "sinthlab_bringup/orchestrators/free_move_orchestrator.py"
    spec = importlib.util.spec_from_file_location("trial_timeout_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.FreeMoveOrchestratorNode


def test_snatch_parks_after_the_outcome_cue_without_waiting_for_release(monkeypatch):
    """force_release.snatch: the reward and its cue, snatch_after_sec, then the quick retreat to park."""
    from unittest.mock import call
    cls = _free_move_node_class(monkeypatch)
    node = Mock()
    node._trial_ending = node._stopping = node._ended = False
    node._snatch, node._snatch_sec = True, 0.5
    node.trial_count = 4
    node.centre_out = Mock(enabled=True)
    node.centre_out.mode.return_value = "either"
    node.n_choice = node.long_move = node.free_reach = node.forage = None
    node.control.pays.return_value = True
    for name in ("_choice_on", "_centre_out_on", "_n_choice_on", "_end_trial", "_trial_over", "_mark_end"):
        setattr(node, name, getattr(cls, name).__get__(node))
    cls.on_reach(node, "right", True)
    node.ring.reward.assert_called_once()                 # the reward cue shows during the hold
    node.end_cue.start.assert_called_once()
    node.force_release.start.assert_not_called()
    node.snatch_wait.set_duration.assert_called_once_with(0.5)
    node.snatch_wait.start.assert_called_once()
    node.switch_to_joint.start.assert_not_called()        # not before the hold is up
    assert node.recorder.mark.call_args_list[-1] == call("snatch_wait", 0.5)
    cls.on_snatch(node)
    node.ring.show.assert_called_with("off")
    node.move_park.retreat.assert_called_once()
    node.switch_to_joint.start.assert_called_once()
    assert node.recorder.mark.call_args_list[-1] == call("snatch")
    node._stopping = True                                 # Stop during the hold: _stop_now has it in hand
    node.switch_to_joint.start.reset_mock()
    cls.on_snatch(node)
    node.switch_to_joint.start.assert_not_called()


@pytest.mark.parametrize("task", ["n_choice", "colour_rule", "move_horizontal"])
def test_choice_deadline_holds_until_release(task, monkeypatch):
    """Exercise the orchestrator callbacks, substituting the hardware actions at their boundaries."""
    import importlib.util
    from unittest.mock import MagicMock, call
    for module in ("actions.move_to_position_joint_space", "actions.park_move", "actions.switch_controller",
                   "actions.safety_stop_monitor", "actions.force_release_waiter", "actions.audio_cue",
                   "actions.visual_cue", "actions.ring_state_cue", "actions.quiet_gate", "actions.trial_recorder",
                   "helpers.experiment_control"):
        monkeypatch.setitem(sys.modules, "sinthlab_bringup." + module, MagicMock())
    path = ROOT / "sinthlab_bringup/orchestrators/free_move_orchestrator.py"
    spec = importlib.util.spec_from_file_location("trial_timeout_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    cls = module.FreeMoveOrchestratorNode
    node = Mock()
    node._trial_ending = node._stopping = node._ended = False
    node._snatch = False                     # force_release.snatch off: the release wait
    node.trial_count = 2
    node.centre_out = None if task == "n_choice" else Mock(enabled=True)
    node.n_choice = Mock() if task == "n_choice" else None
    node.long_move = None
    node.response_timeout.duration_sec.return_value = 30
    for name in ("_choice_on", "_centre_out_on", "_n_choice_on", "_end_trial", "_trial_over", "_mark_end"):
        setattr(node, name, getattr(cls, name).__get__(node))
    cls.on_response_timeout(node)
    cls.on_response_timeout(node)  # duplicate notification
    assert node.recorder.mark.call_args_list == [
        call("response_timeout", None), call("trial_end", 2), call("release_wait")]
    node.force_release.start.assert_called_once()
    node.switch_to_joint.start.assert_not_called()
    node.end_cue.start.assert_not_called()
    cls.on_force_released(node)
    node.switch_to_joint.start.assert_called_once()
    cls.on_switched_to_joint(node)
    node.move_park.start.assert_called_once()
