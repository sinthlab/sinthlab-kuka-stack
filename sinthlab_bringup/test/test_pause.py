"""Pause / Resume (helpers/experiment_control.py): the trial ends at once and the arm parks, as for Stop,
but the run goes on -- the next trial waits at park until Resume. Shared by every orchestrator, which only
marks the interrupted trial with control.stop_token() and parks into control.begin_trial().

    python3 -m pytest sinthlab_bringup/test
"""
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import test_reward_burst  # noqa: E402,F401  (ROS stand-ins for experiment_control)
from sinthlab_bringup.helpers.experiment_control import ExperimentControl  # noqa: E402
from sinthlab_bringup.helpers import outcome_tables as ot  # noqa: E402


class Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    warn = error = info


class Orchestrator:
    """What every orchestrator does: its stop hook ends the trial with control.stop_token() and parks; at
    park it calls control.begin_trial(next_trial)."""

    def __init__(self, control):
        self.c, self.marks, self.trials, self.parking = control, [], 0, False
        control.on_stop_request(self.stop_now)

    def next_trial(self):
        self.trials += 1

    def stop_now(self):
        self.marks.append(self.c.stop_token())
        self.parking = True

    def parked(self):
        self.parking = False
        self.c.begin_trial(self.next_trial)


def control():
    c = ExperimentControl.__new__(ExperimentControl)     # no ROS: the trial gate only
    c._log, c._stopped, c._held, c._paused, c._resume_fn = Log(), False, False, False, None
    c._stop_hooks, c._reloaders, c._pending, c._reward_timers, c._reward_wait = [], [], {}, [], None
    c._sequence_rewards, c._reward_ready_at, c._trial = False, 0.0, 3
    c.phases = []
    c._set_phase = lambda phase, arg=None: c.phases.append(phase)
    c._publish = lambda: None
    c._read_nsp = lambda: None
    c._effective = {}
    return c, Orchestrator(c)


def call(fn):
    return fn(None, SimpleNamespace(success=None, message=""))


def test_pause_parks_then_holds_and_resume_starts_the_next_trial():
    c, o = control()
    r = call(c._on_pause)
    assert r.success and o.marks == ["paused"] and o.parking         # the trial ends at once, marked paused
    o.parked()
    assert o.trials == 0 and c._held and c.phases[-1] == "paused"     # held at park: no next trial
    assert call(c._on_pause).message == "already paused at park"
    r = call(c._on_resume)
    assert r.success and o.trials == 1 and not c._held and not c._paused


def test_resume_before_the_arm_has_parked_carries_straight_on():
    c, o = control()
    call(c._on_pause)
    assert "once the arm is parked" in call(c._on_resume).message
    o.parked()
    assert o.trials == 1                                              # no hold: the run just continues


def test_live_changes_made_while_paused_apply_from_the_resumed_trial():
    c, o = control()
    applied = []
    c.on_reload(lambda: applied.append(dict(c._pending)))
    call(c._on_pause)
    o.parked()
    c._pending = {"reward_pulse.pulse_ms": 300}
    call(c._on_resume)
    assert applied == [{"reward_pulse.pulse_ms": 300}] and c._effective == {"reward_pulse.pulse_ms": 300}


def test_stop_while_paused_ends_the_run_and_resume_is_refused():
    c, o = control()
    call(c._on_pause)
    o.parked()
    r = call(c._on_stop)
    assert r.message == "already holding at park" and c._stopped    # parked already: the launch can end
    assert not call(c._on_resume).success and o.trials == 0
    assert c.stop_token() == "stop_requested"


def test_a_paused_trial_is_aborted_in_the_tables():
    meta = {"experiment": "move_free", "trial_index": 1, "events": [
        {"event": "trial_start", "arg": 1, "t_wall": 1.0}, {"event": "cue_go", "t_wall": 2.0},
        {"event": "paused", "t_wall": 3.0}, {"event": "trial_end", "arg": 1, "t_wall": 3.0}]}
    row = ot.trial_row(meta)
    assert row["outcome"] == "paused" and row["success"] == ""
