#!/usr/bin/env python3
"""Visual cue — fires the end effector's NeoPixel ring. Mirrors AudioCue.

However it is fired, the board runs its configured cue: on, then off after its own `duration`.
This action only decides WHEN. `visual_cue.remote_test_trigger` decides HOW:

  true   Wi-Fi -- a TEST trigger for demos and recordings. See visual_cue_remote.py.

             this computer ── joined to KUKA_NEOPIXEL ──► GET http://192.168.4.1/cue

  false  The wire -- the experiment trigger. See end_effector_design/README.md.

             switch at robot base ── X76 contacts 1/2 ══ media flange ══ tool pins 9/10 ──► Metro D2

THE WIRE'S SWITCH IS NOT CHOSEN YET

    The cabinet cannot close it: this arm's Media flange Inside electric has no cabinet-driven I/O,
    and the Sunrise project has no generated I/O groups at all. So the switch must be driven from
    this computer -- most likely a USB relay with dry contacts, which is simply a jumper wire the
    computer can open and close.

    Until that hardware exists the wire path is a SAFE NO-OP: start() calls on_complete immediately,
    and one warning is logged per process. While commissioning, short X76 1-2 by hand and watch the
    result with sinthlab_bringup/diagnostics/check_cue_wiring.py. When the switch is chosen,
    implement _close_switch() below; every orchestrator already calls start() at the right moment.

WHAT THE CUE LOOKS LIKE IS NOT SET FROM HERE. Colour, brightness, pattern, segments and duration
live on the board and are set over its own Wi-Fi access point -- see
end_effector_metro_code/README.md.
"""
from __future__ import annotations

from typing import Callable, Optional

from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.visual_cue_remote import DEFAULT_BOARD, RemoteCueTrigger

_NOT_WIRED = (
    "visual_cue.enabled is true, but no trigger switch is wired to this computer yet, so visual "
    "cues will NOT fire. For a demo, set visual_cue.remote_test_trigger: true to fire them over the "
    "board's Wi-Fi; to test the wire, short X76 contacts 1-2 by hand. "
    "See end_effector_design/README.md.")


def optional_param(node: rclpyNode, name: str, default):
    """Parameter with a fallback — the visual_cue block may be absent or partial."""
    if node.has_parameter(name):
        value = node.get_parameter(name).value
        if value is not None:
            return value
    return default


class VisualCue:
    """Fires the ring's configured cue when start() is called. See the module doc for the two paths.

    Parameters, under a single shared `visual_cue` block:
        enabled              bool  false (the default) makes every visual cue a silent no-op
        remote_test_trigger  bool  true: fire over the board's Wi-Fi (demos only)
                                   false (the default): the X76 wire, a no-op until a switch is fitted
        remote_board         str   where the Wi-Fi trigger sends, default 192.168.4.1

    `label` names the cue site in log lines.
    """

    _warned = False                             # one warning per process, not one per cue site per trial
    _remote: Optional[RemoteCueTrigger] = None  # one Wi-Fi sender per process, shared by every cue site

    @staticmethod
    def warmup(node: rclpyNode) -> None:
        """Mirrors AudioCue.warmup(). Says which trigger is in use. Never raises, never blocks."""
        if not optional_param(node, "visual_cue.enabled", False):
            return
        if optional_param(node, "visual_cue.remote_test_trigger", False):
            VisualCue._remote_trigger(node).check()
        else:
            VisualCue._warn_once(node)

    @classmethod
    def _remote_trigger(cls, node: rclpyNode) -> RemoteCueTrigger:
        if cls._remote is None:
            board = str(optional_param(node, "visual_cue.remote_board", DEFAULT_BOARD))
            cls._remote = RemoteCueTrigger(node, board)
        return cls._remote

    @classmethod
    def _warn_once(cls, node: rclpyNode) -> None:
        if not cls._warned:
            cls._warned = True
            node.get_logger().warn(_NOT_WIRED)

    def __init__(self, node: rclpyNode, *, label: str = "cue",
                 on_complete: Callable[[], None]) -> None:
        self._node = node
        self._label = label
        self._on_complete = on_complete
        self._enabled = bool(optional_param(node, "visual_cue.enabled", False))
        self._remote_test = bool(optional_param(node, "visual_cue.remote_test_trigger", False))

    def start(self) -> None:
        if self._enabled:
            if self._remote_test:
                VisualCue._remote_trigger(self._node).fire(self._label)
            else:
                self._close_switch()
        self._shutdown()

    def _close_switch(self) -> None:
        """Short X76 contacts 1 and 2 briefly.   >>> IMPLEMENT WHEN THE SWITCH IS CHOSEN <<<

        The board is in `pulse` mode by default, so the closure only has to outlast its 5 ms
        debounce — about 50 ms is plenty — and the board's own `duration` decides how long the
        ring stays lit. Log the ROS time here: it is the cue's timestamp in the trial record.

        Must never raise and must never block the orchestrator for long. A missing cue is bad;
        a stalled experiment is worse.
        """
        VisualCue._warn_once(self._node)

    def _shutdown(self) -> None:
        if self._on_complete is not None:
            self._on_complete()
