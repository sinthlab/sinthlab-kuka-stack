"""Preview the visual cue, then release movement on light-off and the tone.

An early touch (warn_early_touch) gets its own cue: the low tone, and a few quick flashes of the whole ring
(quiet_gate.flash_*: yellow by default, helpers/early_touch_flash.py), ending dark."""

import queue
import time

from sinthlab_bringup.actions.audio_cue import AudioCue
from sinthlab_bringup.actions.visual_cue import VisualCue, colour_param
from sinthlab_bringup.helpers import early_touch_flash


class GoCue:
    WAIT_SEC = 0.25
    PREVIEW_SEC = 0.30

    def __init__(self, node, recorder, quiet_window, audio, on_ready):
        self._node = node
        self._record = recorder.mark
        self._quiet_window = quiet_window
        self._audio = audio
        self._on_ready = on_ready
        self._results = queue.SimpleQueue()
        self._sequence = 0
        self._stage = None
        self._flash = []                # the early-touch flashes still to send: [(monotonic time, on?)]
        self._flash_path = ""
        AudioCue.warmup("early_touch")
        node.create_timer(0.005, self._poll)
        VisualCue.set_result_sink(self._visual_result)

    def start(self, light, *, visual_enabled, preview_sec=None):
        """Light the cue (light(label)), hold it PREVIEW_SEC -- or preview_sec: the free reach's instructed
        delay, the cued direction shown before the go -- then release on light-off and the tone."""
        self._preview_sec = self.PREVIEW_SEC if preview_sec is None else max(0.0, float(preview_sec))
        self._sequence += 1
        self._visual_enabled = visual_enabled
        self._quiet_window.continue_until_go()
        if visual_enabled:
            self._wait_for("preview", {"visual"})
            self._record("cue_preview")
            light(self._label)
        else:
            self._release()
        self._poll()

    def _wait_for(self, stage, needed):
        self._stage = stage
        self._label = f"go_{self._sequence}_{stage}"
        self._needed = needed
        self._deadline = time.monotonic() + self.WAIT_SEC

    def _release(self):
        needed = {"visual"} if self._visual_enabled else set()
        if self._audio.enabled():
            needed.add("audio")
        self._record("cue_go")
        self._wait_for("release", needed)
        label = self._label
        if self._visual_enabled:
            VisualCue.clear_ring(self._node, label=label)
        self._audio.start(on_started=lambda: self._post(label, "audio"))

    def warn_early_touch(self):
        """The early-touch cue: the low tone and the yellow flashes (call it after anything that darkens the
        ring as the trial ends, or that would cut the first flash short)."""
        if self._audio.enabled():
            AudioCue.play_early_touch(self._node)
        node = self._node
        if not (node.has_parameter("visual_cue.enabled") and node.get_parameter("visual_cue.enabled").value):
            return
        rgbw = early_touch_flash.colour(colour_param(node, "quiet_gate.flash_colour"))
        n = node.get_parameter("quiet_gate.flashes").value if node.has_parameter("quiet_gate.flashes") else 0
        sec = node.get_parameter("quiet_gate.flash_sec").value if node.has_parameter("quiet_gate.flash_sec") else 0.15
        steps = early_touch_flash.schedule(n, sec) if rgbw is not None else []
        if not steps:
            return
        now = time.monotonic()
        self._flash_path = early_touch_flash.ring_path(rgbw)
        self._flash = [(now + t, on) for t, on in steps]
        self._step_flash()

    def _step_flash(self):
        now = time.monotonic()
        while self._flash and self._flash[0][0] <= now:
            _, on = self._flash.pop(0)
            sender = VisualCue.sender(self._node)
            if sender is None:
                self._flash = []
                return
            try:
                sender.show(self._flash_path if on else "/off", "early_touch_on" if on else "early_touch_off")
            except Exception as exc:
                self._node.get_logger().warn(f"Early-touch flash not sent: {exc}")
                self._flash = []

    def stop_flash(self):
        """Drop any early-touch flashes still to come (the ring is about to show something else)."""
        self._flash = []

    def cancel(self):
        if self._stage is not None:
            self._stage = None
            self._quiet_window.stop()
            if self._visual_enabled:
                VisualCue.clear_ring(self._node)

    def _post(self, label, kind, ms=None):
        self._results.put((label, kind, ms, time.monotonic()))

    def _visual_result(self, label, ok, ms):
        if ok and label.startswith("go_"):
            self._post(label, "visual", ms)

    def _poll(self):
        if self._flash:
            self._step_flash()
        while True:
            try:
                label, kind, ms, received = self._results.get_nowait()
            except queue.Empty:
                break
            if self._stage is None or label != self._label or received > self._deadline or kind not in self._needed:
                continue
            self._needed.remove(kind)
            event = "cue_audio_start" if kind == "audio" else (
                "cue_visual_ack" if self._stage == "preview" else "cue_visual_off")
            self._record(event, round(ms, 1) if ms is not None else None)
            if self._stage == "preview":
                self._stage = "showing"
                self._deadline = received + self._preview_sec
        if self._stage is None:
            return
        now = time.monotonic()
        if self._stage == "showing":
            if now >= self._deadline:
                self._release()
            return
        if self._stage == "release" and not self._needed and now <= self._deadline:
            self._stage = None
            self._quiet_window.stop()
            self._on_ready()
        elif now >= self._deadline:
            missing = ", ".join(sorted(self._needed)) or "late arming"
            self.cancel()
            self._record("go_cue_failed", missing)
            self._node.get_logger().error(
                "Go cue not confirmed. The arm is staying at the start. Press Stop to end the run.")
