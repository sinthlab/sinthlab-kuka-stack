"""A pending go flash must not turn off a newer outcome cue."""

from types import SimpleNamespace

from test_centre_out_monitor import FakeNode

from sinthlab_bringup.actions.ring_state_cue import RingStateCue
from sinthlab_bringup.actions.visual_cue import VisualCue


class TimerNode(FakeNode):
    def create_timer(self, period, callback):
        self.timers.append((period, callback))
        return self.timers[-1]

    def destroy_timer(self, timer):
        self.timers.remove(timer)


class Board:
    def __init__(self):
        self.commands = []

    def show(self, path, label):
        self.commands.append((path, label))

    def fire(self, label, colour):
        self.commands.append((colour, label))


def test_go_flash_ends_and_a_new_cue_cancels_the_blackout(monkeypatch):
    board = Board()
    monkeypatch.setattr(VisualCue, "sender", lambda node: board)
    monkeypatch.setattr(VisualCue, "_flash_timer", None)
    node = TimerNode({
        "visual_cue.enabled": True,
        "visual_cue.colours.play": [0, 64, 0, 0],
        "visual_cue.colours.snap": [255, 0, 0, 0],
        "ring_cue.quarters": [0, 1, 2, 3],
        "ring_cue.colours.go": [0, 64, 0, 0],
    })
    ring = RingStateCue(node)

    ring.show("go", flash=True)
    assert board.commands[-1][1] == "ring_go"
    period, turn_off = node.timers[0]
    assert period == 0.30
    turn_off()
    assert board.commands[-1] == ("/off", "ring_off")
    assert node.timers == []

    go = VisualCue(node, label="play", on_complete=None)
    go.start()
    _, stale_turn_off = node.timers[0]
    snap = VisualCue(node, label="snap", on_complete=None)
    snap.start()
    stale_turn_off()
    assert board.commands[-1] == ((255, 0, 0, 0), "snap")
    assert node.timers == []


def test_go_waits_for_both_channels_and_ignores_an_old_reply(monkeypatch):
    from sinthlab_bringup.actions import go_cue

    clock = [0.0]
    monkeypatch.setattr(go_cue.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(VisualCue, "_result_sink", None)
    node = TimerNode({"audio_cue.enabled": True, "visual_cue.enabled": True})
    board = Board()
    monkeypatch.setattr(VisualCue, "sender", lambda node: board)
    monkeypatch.setattr(VisualCue, "_flash_timer", None)
    events, audio, visual = [], [], []

    class Recorder:
        def mark(self, label, value=None):
            events.append((label, value))

    class Audio:
        def enabled(self):
            return True

        def start(self, *, on_started):
            audio.append(on_started)

    gate = go_cue.GoCue(node, Recorder(), lambda: events.append(("armed", None)))
    gate.start(Audio(), visual.append, visual_enabled=True)
    audio[0]()
    gate._poll()
    assert events == [("cue_audio_start", None)]

    clock[0] = 0.26
    gate._poll()
    assert events[-1] == ("go_cue_failed", "visual")
    assert board.commands[-1] == ("/off", "ring_off")

    gate.start(Audio(), visual.append, visual_enabled=True)
    gate._visual_result(visual[0], True, 1.0)
    gate._poll()
    assert events[-1][0] == "go_cue_failed"
    gate._visual_result(visual[1], True, 2.0)
    gate._poll()
    assert events[-1] == ("cue_visual_ack", 2.0)
    audio[1]()
    gate._poll()
    assert events[-1] == ("armed", None)


def test_go_with_both_channels_off_arms_without_waiting(monkeypatch):
    from sinthlab_bringup.actions.go_cue import GoCue

    monkeypatch.setattr(VisualCue, "_result_sink", None)
    node = TimerNode({"audio_cue.enabled": False, "visual_cue.enabled": False})

    class Silent:
        def enabled(self):
            return False

        def start(self, *, on_started):
            pass

    ready = []
    gate = GoCue(node, SimpleNamespace(mark=lambda *args: None), lambda: ready.append(True))
    gate.start(Silent(), lambda label: None, visual_enabled=False)
    assert ready == [True]
