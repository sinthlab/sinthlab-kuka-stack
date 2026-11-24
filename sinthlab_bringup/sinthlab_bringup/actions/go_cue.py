"""Preview the visual cue, then release movement on light-off and the tone."""

import queue
import time

from sinthlab_bringup.actions.audio_cue import AudioCue
from sinthlab_bringup.actions.visual_cue import VisualCue


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
        AudioCue.warmup("early_touch")
        node.create_timer(0.005, self._poll)
        VisualCue.set_result_sink(self._visual_result)

    def start(self, light, *, visual_enabled, preview_sec=None):
        """Light the cue (light(label)), hold it PREVIEW_SEC -- or preview_sec: Cued Pull's instructed
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
        if self._audio.enabled():
            AudioCue.play_early_touch(self._node)

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
