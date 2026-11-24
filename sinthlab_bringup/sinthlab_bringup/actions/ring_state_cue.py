#!/usr/bin/env python3
"""Quarter-ring STATE cue for the free-move experiments -- e.g. a quarter green for the cue, the whole ring
white for the reward, dark between trials.

THE REWARD CUE is the same in every task: the whole ring in ring_cue.colours.reward (white), shown with
every reward that is actually given (reward()). Nothing else uses white, so it never looks like a cue.

Unlike VisualCue (a timed flash the board runs on a trigger), this sets what the ring SHOWS and it
stays that way until the next state. It is expressed as the board's `/segments` request:

    /segments?factor=4&colors=0,255,0,0.0,0,0,0.0,255,0,0.0,0,0,0     quarters 0 and 2 green
    /off                                                               all dark

The ring is 4 quarter arcs of 15 LEDs -- pixels 0-14, 15-29, 30-44, 45-59 in data order -- so
quarter 0..3 below means that arc. WHICH ONE IS "UP" depends on how the ring sits on the flange and
on A7 at the start pose; find it once with `curl "http://192.168.4.1/segments?factor=4&colors=0,255,0,0"`
(lights quarter 0 only) and set `ring_cue.quarters` to match.

It goes out on whichever path `visual_cue.trigger` selects: over Wi-Fi as the request above, over the
serial link as the equivalent `SEG` / `OFF` command (visual_cue_serial.py). `visual_cue.enabled: false`
turns it off, like every other visual cue.

Parameters (all live):
    ring_cue.quarters        int[]  which quarters light, 0-3
    ring_cue.colours.<state> int[]  [r, g, b] or [r, g, b, w] 0-255 for that state; a state with no
                                    colour, or "off", darkens the ring
"""
from __future__ import annotations

from typing import List, Optional, Tuple

from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.visual_cue import VisualCue, colour_param
from sinthlab_bringup.helpers.node_utils import get_required_param

_QUARTERS = 4


class RingStateCue:
    def __init__(self, node: rclpyNode, *, param_prefix: str = "ring_cue") -> None:
        self._node = node
        self._p = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix
        self._state: Optional[str] = None
        self.reload()

    def reload(self) -> None:
        node = self._node
        self._enabled = bool(get_required_param(node, "visual_cue.enabled"))
        self._quarters = sorted({int(q) for q in get_required_param(node, self._p + "quarters")
                                 if 0 <= int(q) < _QUARTERS})

    def _colour(self, state: str) -> Optional[Tuple[int, ...]]:
        raw = colour_param(self._node, f"{self._p}colours.{state}")
        if raw is None:
            return None
        try:
            vals = [max(0, min(255, int(v))) for v in raw]
        except (TypeError, ValueError):
            return None
        if len(vals) not in (3, 4):
            return None
        return tuple(vals + [0] * (4 - len(vals)))

    def path_for(self, state: str, quarters: Optional[List[int]] = None) -> str:
        """The board request for `state` (kept separate so it can be checked without a board). `quarters`
        lights those instead of ring_cue.quarters (the centre-out task's cued end)."""
        colour = self._colour(state) if state != "off" else None
        lit = self._quarters if quarters is None else sorted({int(q) for q in quarters if 0 <= int(q) < _QUARTERS})
        if colour is None or not any(colour) or not lit:
            return "/off"
        parts: List[str] = []
        for q in range(_QUARTERS):
            c = colour if q in lit else (0, 0, 0, 0)
            parts.append(",".join(str(v) for v in c))
        return "/segments?factor=4&colors=" + ".".join(parts)

    def enabled(self) -> bool:
        return self._enabled

    def show(self, state: str, quarters: Optional[List[int]] = None, *, label: Optional[str] = None) -> None:
        """Show the state on the selected quarters until the next cue."""
        self._state = state
        if not self._enabled:
            return
        try:
            sender = VisualCue.sender(self._node)
            if sender is not None:
                path = self.path_for(state, quarters)
                if label is None:
                    sender.show(path, f"ring_{state}")
                elif path != "/off":
                    sender.show(path, label)
        except Exception as exc:
            self._node.get_logger().warn(f"Ring cue '{state}' not sent: {exc}")

    def reward(self, *, label: Optional[str] = None) -> None:
        """The reward cue: the whole ring in ring_cue.colours.reward, until the next state."""
        self.show("reward", list(range(_QUARTERS)), label=label)

    def state(self) -> Optional[str]:
        return self._state
