#!/usr/bin/env python3
"""Visual cue — fires the end effector's NeoPixel ring. Mirrors AudioCue.

However it is fired, the board runs its configured cue: on, then off after its own `duration`.
This action decides WHEN, and can give each cue site its own colour (`visual_cue.colours.<label>`).
`visual_cue.trigger` decides HOW, and the experiments never know which:

  "serial"  The RS-422 link through the media flange. See visual_cue_serial.py.

             this computer ─USB─► StarTech ══ X76 ══ flange ══ tool pins ══► MikroE ──► Metro UART

  "wifi"    The board's own access point. See visual_cue_remote.py.

             this computer ── joined to KUKA_NEOPIXEL ──► GET http://192.168.4.1/cue

Both carry the same things -- a cue, a cue in a colour, and the ring states RingStateCue sends -- and
both report each cue's acknowledgement (the board answers only once the ring is lit). They differ in
timing: the serial link answers in a few ms, every time; the Wi-Fi delay varies from cue to cue.

WHAT THE CUE LOOKS LIKE is set on the board -- brightness, pattern, segments, duration and the
default colour -- over its own Wi-Fi access point; see end_effector_metro_code/README.md. Set from here:
an optional colour per cue site, and the ring's BRIGHTNESS (visual_cue.brightness, live), sent at the
start of every run and on every change (apply_brightness), so it is the task's setting and in its record
rather than whatever the board was last left at. It goes into the board's RAM only (never saved).
"""
from __future__ import annotations

from typing import Callable, Optional, Tuple

from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.visual_cue_remote import RemoteCueTrigger
from sinthlab_bringup.actions.visual_cue_serial import SerialCueTrigger
from sinthlab_bringup.helpers.node_utils import get_required_param

TRIGGERS = ("serial", "wifi")


def colour_param(node: rclpyNode, name: str):
    """A per-site colour, or None. Colours are the one optional part of the visual_cue block: a cue
    site without one deliberately leaves the board's own colour in place."""
    return node.get_parameter(name).value if node.has_parameter(name) else None


class VisualCue:
    """Fires the ring's configured cue when start() is called. See the module doc for the two paths.

    Parameters, under a single shared `visual_cue` block:
        enabled          bool  false makes every visual cue a silent no-op
        trigger          str   "serial" (the RS-422 link) or "wifi" (the board's access point)
        serial_port      str   the StarTech adapter's port, e.g. /dev/ttyUSB0 (serial only)
        remote_board     str   the board's address on its access point, 192.168.4.1 (wifi only)
        colours.<label>  int[] [r, g, b] or [r, g, b, w] (0-255) for the cue site named `label`.
                               Absent = whatever colour the board has at that moment.
        brightness       float 0-0.3, scales the whole ring; 0 or absent = the board's own setting

    `label` names the cue site in log lines.
    """

    # One sender per path per process, shared by every cue site and by RingStateCue, so commands to
    # the board go out one at a time whichever object sends them.
    _senders = {}
    _warned_trigger = None

    @staticmethod
    def warmup(node: rclpyNode) -> None:
        """Mirrors AudioCue.warmup(). Says which trigger is in use and checks the board answers.
        Never raises, never blocks."""
        if not get_required_param(node, "visual_cue.enabled"):
            return
        sender = VisualCue.sender(node)
        if sender is not None:
            sender.check()
        VisualCue.apply_brightness(node)

    _brightness = None      # the brightness last sent, so the many cue sites send a change once

    @classmethod
    def apply_brightness(cls, node: rclpyNode) -> None:
        """Send visual_cue.brightness to the board if it differs from what was last sent. Called at
        start-up and at every reload (a trial boundary). Never raises, never blocks."""
        try:
            if not node.has_parameter("visual_cue.brightness") or not get_required_param(node, "visual_cue.enabled"):
                return
            value = round(float(node.get_parameter("visual_cue.brightness").value), 3)
            if value <= 0.0 or value == cls._brightness:
                return
            sender = cls.sender(node)
            if sender is not None:
                sender.set_brightness(value)
                cls._brightness = value
        except Exception as exc:
            node.get_logger().warn(f"Ring brightness not sent: {exc}")

    # Where board acknowledgements go. The senders are class-level singletons shared by every cue
    # site, so the sink is class-level too; an orchestrator registers once.
    _result_sink = None

    @classmethod
    def set_result_sink(cls, fn) -> None:
        """fn(label, ok, rtt_ms) is called when the board acknowledges a cue.

        The firmware lights the ring before it answers, on either path, so an ack means the LEDs are
        already on. Register this from an orchestrator to get `cue_visual_ack` into the trial record."""
        cls._result_sink = fn

    @classmethod
    def sender(cls, node: rclpyNode):
        """The sender for the trigger `visual_cue.trigger` selects right now, or None (with one warning)
        if it names neither path. Created on first use."""
        trigger = str(get_required_param(node, "visual_cue.trigger")).strip().lower()
        if trigger not in TRIGGERS:
            if cls._warned_trigger != trigger:
                cls._warned_trigger = trigger
                node.get_logger().warn(
                    f"visual_cue.trigger is {trigger!r}; it must be one of {', '.join(TRIGGERS)}. "
                    "Visual cues will NOT fire until it is.")
            return None
        if trigger not in cls._senders:
            sink = lambda lbl, ok, ms: cls._result_sink(lbl, ok, ms) if cls._result_sink else None
            if trigger == "serial":
                port = str(get_required_param(node, "visual_cue.serial_port"))
                cls._senders[trigger] = SerialCueTrigger(node, port, on_result=sink)
            else:
                board = str(get_required_param(node, "visual_cue.remote_board"))
                cls._senders[trigger] = RemoteCueTrigger(node, board, on_result=sink)
        return cls._senders[trigger]

    def __init__(self, node: rclpyNode, *, label: str = "cue",
                 on_complete: Callable[[], None]) -> None:
        self._node = node
        self._label = label
        self._on_complete = on_complete
        self.reload()

    def reload(self) -> None:
        """Re-read enabled / this site's colour. Both are live parameters, and so is the trigger, which
        sender() reads at every cue: an orchestrator calls this at a trial boundary after a change
        (see helpers/live_params.py)."""
        node = self._node
        self._enabled = bool(get_required_param(node, "visual_cue.enabled"))
        self._colour = self._read_colour(node, self._label) if self._enabled else None
        VisualCue.apply_brightness(node)

    @staticmethod
    def _read_colour(node: rclpyNode, label: str) -> Optional[Tuple[int, int, int, int]]:
        """visual_cue.colours.<label> as (r, g, b, w), or None to use the board's configured colour."""
        name = f"visual_cue.colours.{label}"
        raw = colour_param(node, name)
        if raw is None:
            return None
        try:
            vals = [int(v) for v in raw]
        except (TypeError, ValueError):
            vals = []
        if len(vals) not in (3, 4):
            node.get_logger().warn(
                f"{name} must be [r, g, b] or [r, g, b, w] with values 0-255, got {raw!r}. "
                "This cue will use the board's configured colour.")
            return None
        vals += [0] * (4 - len(vals))
        return tuple(max(0, min(255, v)) for v in vals)

    @classmethod
    def clear_ring(cls, node: rclpyNode, *, label="ring_off") -> None:
        sender = cls.sender(node)
        if sender is not None:
            sender.show("/off", label)

    def enabled(self) -> bool:
        return self._enabled

    def preview(self, label: str) -> None:
        """Show a solid cue until the go controller explicitly turns the ring off."""
        sender = self.sender(self._node)
        if self._enabled and sender is not None:
            colour = self._colour if self._colour is not None else (0, 64, 0, 0)
            if any(colour):
                sender.show("/segments?factor=1&colors=" + ",".join(map(str, colour)), label)

    def start(self) -> None:
        if self._enabled:
            try:
                sender = VisualCue.sender(self._node)
                if sender is not None:
                    sender.fire(self._label, self._colour)
            except Exception as exc:    # a missing cue is bad; a stalled experiment is worse
                self._node.get_logger().warn(f"Visual cue '{self._label}' not sent: {exc}")
        self._shutdown()

    def _shutdown(self) -> None:
        if self._on_complete is not None:
            self._on_complete()
