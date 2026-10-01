#!/usr/bin/env python3
"""Quarter-ring STATE cue for the pre-training experiments -- e.g. the up and down quarters green
while the arm may travel, red once the threshold is reached, dark between trials.

Unlike VisualCue (a timed flash the board runs on a trigger), this sets what the ring SHOWS and it
stays that way until the next state. It uses the board's existing `/segments` endpoint, so the
firmware already on the effector needs no change:

    /segments?factor=4&colors=0,255,0,0.0,0,0,0.0,255,0,0.0,0,0,0     quarters 0 and 2 green
    /off                                                               all dark

The ring is 4 quarter arcs of 15 LEDs -- pixels 0-14, 15-29, 30-44, 45-59 in data order -- so
quarter 0..3 below means that arc. WHICH ONE IS "UP" depends on how the ring sits on the flange and
on A7 at the start pose; find it once with `curl "http://192.168.4.1/segments?factor=4&colors=0,255,0,0"`
(lights quarter 0 only) and set `ring_cue.quarters` to match.

Wi-Fi only (`visual_cue.remote_test_trigger: true`): the X76 wire is one bit and cannot carry a
colour pattern. With the wire selected this is a no-op and says so once. `visual_cue.enabled: false`
turns it off too, like every other visual cue.

Parameters (all live):
    ring_cue.quarters        int[]  which quarters light, 0-3
    ring_cue.colours.<state> int[]  [r, g, b] or [r, g, b, w] 0-255 for that state; a state with no
                                    colour, or "off", darkens the ring
"""
from __future__ import annotations

from typing import List, Optional, Tuple

from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.visual_cue import VisualCue, optional_param

_QUARTERS = 4


class RingStateCue:
    _warned = False

    def __init__(self, node: rclpyNode, *, param_prefix: str = "ring_cue") -> None:
        self._node = node
        self._p = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix
        self._state: Optional[str] = None
        self.reload()

    def reload(self) -> None:
        node = self._node
        self._enabled = bool(optional_param(node, "visual_cue.enabled", False))
        self._wifi = bool(optional_param(node, "visual_cue.remote_test_trigger", False))
        raw = optional_param(node, self._p + "quarters", [0, 1, 2, 3])
        try:
            self._quarters = sorted({int(q) for q in raw if 0 <= int(q) < _QUARTERS})
        except (TypeError, ValueError):
            self._quarters = [0, 1, 2, 3]

    def _colour(self, state: str) -> Optional[Tuple[int, ...]]:
        raw = optional_param(self._node, f"{self._p}colours.{state}", None)
        if raw is None:
            return None
        try:
            vals = [max(0, min(255, int(v))) for v in raw]
        except (TypeError, ValueError):
            return None
        if len(vals) not in (3, 4):
            return None
        return tuple(vals + [0] * (4 - len(vals)))

    def path_for(self, state: str) -> str:
        """The board request for `state` (kept separate so it can be checked without a board)."""
        colour = self._colour(state) if state != "off" else None
        if colour is None or not any(colour) or not self._quarters:
            return "/off"
        parts: List[str] = []
        for q in range(_QUARTERS):
            c = colour if q in self._quarters else (0, 0, 0, 0)
            parts.append(",".join(str(v) for v in c))
        return "/segments?factor=4&colors=" + ".".join(parts)

    def show(self, state: str) -> None:
        """Show `state` on the ring until the next show(). Never blocks, never raises."""
        self._state = state
        if not self._enabled:
            return
        if not self._wifi:
            if not RingStateCue._warned:
                RingStateCue._warned = True
                self._node.get_logger().warn(
                    "Quarter-ring cues need the Wi-Fi trigger (visual_cue.remote_test_trigger: true); "
                    "the X76 wire carries one bit, not a pattern. The ring will stay dark.")
            return
        try:
            VisualCue._remote_trigger(self._node).show(self.path_for(state), f"ring_{state}")
        except Exception as exc:
            self._node.get_logger().warn(f"Ring cue '{state}' not sent: {exc}")

    def state(self) -> Optional[str]:
        return self._state
