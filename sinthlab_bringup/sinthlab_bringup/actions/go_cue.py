"""Wait for both enabled go cues without blocking the ROS executor."""

import queue
import time

from sinthlab_bringup.actions.visual_cue import VisualCue


class GoCue:
    WAIT_SEC = 0.25

    def __init__(self, node, recorder, on_ready):
        self._node = node
        self._record = recorder.mark
        self._on_ready = on_ready
        self._results = queue.SimpleQueue()
        self._sequence = 0
        self._pending = None
        node.create_timer(0.005, self._poll)
        VisualCue.set_result_sink(self._visual_result)

    def start(self, audio, light, *, visual_enabled):
        self._sequence += 1
        label = f"go_{self._sequence}"
        deadline = time.monotonic() + self.WAIT_SEC
        needed = set()
        if audio.enabled():
            needed.add("audio")
        if visual_enabled:
            needed.add("visual")
        self._pending = (label, deadline, needed, visual_enabled)
        light(label)
        audio.start(on_started=lambda: self._post(label, "audio"))
        self._poll()

    def cancel(self):
        if self._pending is not None:
            visual_enabled = self._pending[3]
            self._pending = None
            if visual_enabled:
                VisualCue.clear_ring(self._node)

    def _post(self, label, kind, ms=None):
        self._results.put((label, kind, ms, time.monotonic()))

    def _visual_result(self, label, ok, ms):
        if ok and label.startswith("go_"):
            self._post(label, "visual", ms)

    def _poll(self):
        pending = self._pending
        while True:
            try:
                label, kind, ms, received = self._results.get_nowait()
            except queue.Empty:
                break
            if pending is None or label != pending[0] or received > pending[1] or kind not in pending[2]:
                continue
            pending[2].discard(kind)
            self._record("cue_visual_ack" if kind == "visual" else "cue_audio_start",
                         round(ms, 1) if ms is not None else None)
        if pending is None:
            return
        _, deadline, needed, _ = pending
        now = time.monotonic()
        if not needed and now <= deadline:
            self._pending = None
            self._on_ready()
        elif now >= deadline:
            self.cancel()
            self._record("go_cue_failed", ", ".join(sorted(needed)) or "late arming")
            self._node.get_logger().error(
                "Go cue not confirmed. The arm is staying at the start. Press Stop to end the run.")
