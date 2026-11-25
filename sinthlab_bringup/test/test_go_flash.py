"""Verify the light preview, audio release and early-touch cancellation."""

from types import SimpleNamespace
from unittest.mock import Mock

from test_centre_out_monitor import FakeNode

from sinthlab_bringup.actions import go_cue
from sinthlab_bringup.actions.ring_state_cue import RingStateCue
from sinthlab_bringup.actions.visual_cue import VisualCue


def make_gate(monkeypatch, *, audio_enabled=True, visual_enabled=True):
    clock = [0.0]
    monkeypatch.setattr(go_cue.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(go_cue.AudioCue, "warmup", Mock())
    monkeypatch.setattr(VisualCue, "_result_sink", None)
    commands, events = [], []
    board = SimpleNamespace(show=lambda path, label: commands.append((path, label)))
    monkeypatch.setattr(VisualCue, "sender", staticmethod(lambda node: board))
    node = FakeNode({
        "visual_cue.enabled": visual_enabled,
        "visual_cue.colours.play": [0, 64, 0, 0],
        "ring_cue.quarters": [0, 1, 2, 3],
        "ring_cue.colours.rule_red": [128, 0, 0, 0],
    })
    audio, quiet, ready = Mock(), Mock(), Mock()
    audio.enabled.return_value = audio_enabled
    recorder = SimpleNamespace(mark=lambda *args: events.append(args))
    gate = go_cue.GoCue(node, recorder, quiet, audio, ready)
    return node, gate, audio, quiet, ready, clock, commands, events


def test_go_waits_for_preview_then_waits_for_darkness_and_audio(monkeypatch):
    node, gate, audio, quiet, ready, clock, commands, events = make_gate(monkeypatch)
    ring = RingStateCue(node)
    gate.start(lambda label: ring.show("rule_red", label=label), visual_enabled=True)
    path, preview_label = commands[-1]
    assert path == "/segments?factor=4&colors=" + ".".join(["128,0,0,0"] * 4)
    quiet.continue_until_go.assert_called_once()
    audio.start.assert_not_called()

    clock[0] = 0.10
    gate._visual_result(preview_label, True, 2.0)
    gate._poll()
    clock[0] = 0.399
    gate._poll()
    audio.start.assert_not_called()
    clock[0] = 0.40
    gate._poll()
    path, release_label = commands[-1]
    assert path == "/off"
    assert release_label != preview_label

    audio.start.call_args.kwargs["on_started"]()
    gate._visual_result(preview_label, True, 2.0)  # duplicate reply cannot release movement
    gate._poll()
    ready.assert_not_called()
    gate._visual_result(release_label, True, 2.0)
    gate._poll()
    ready.assert_called_once()
    quiet.stop.assert_called_once()
    assert [event[0] for event in events] == [
        "cue_preview", "cue_visual_ack", "cue_go", "cue_audio_start", "cue_visual_off",
    ]


def test_early_touch_cancels_preview_and_plays_distinct_warning(monkeypatch):
    node, gate, audio, quiet, ready, clock, commands, _ = make_gate(monkeypatch)
    warning = Mock()
    monkeypatch.setattr(go_cue.AudioCue, "play_early_touch", warning)
    gate.start(VisualCue(node, label="play", on_complete=None).preview, visual_enabled=True)
    path, preview_label = commands[-1]
    assert path == "/segments?factor=1&colors=0,64,0,0"
    gate._visual_result(preview_label, True, 2.0)
    gate._poll()
    gate.cancel()
    gate.warn_early_touch()
    clock[0] = 0.31
    gate._poll()
    assert commands[-1] == ("/off", "ring_off")
    warning.assert_called_once_with(node)
    audio.start.assert_not_called()
    ready.assert_not_called()
    quiet.stop.assert_called_once()


def test_missing_preview_does_not_play_go(monkeypatch):
    node, gate, audio, _, ready, clock, commands, events = make_gate(monkeypatch)
    gate.start(VisualCue(node, label="play", on_complete=None).preview, visual_enabled=True)
    clock[0] = 0.26
    gate._poll()
    assert events[-1] == ("go_cue_failed", "visual")
    assert commands[-1] == ("/off", "ring_off")
    audio.start.assert_not_called()
    ready.assert_not_called()


def test_go_with_both_channels_off_arms_without_waiting(monkeypatch):
    _, gate, _, quiet, ready, _, commands, _ = make_gate(monkeypatch, audio_enabled=False, visual_enabled=False)
    gate.start(Mock(), visual_enabled=False)
    assert commands == []
    ready.assert_called_once()
    quiet.stop.assert_called_once()
