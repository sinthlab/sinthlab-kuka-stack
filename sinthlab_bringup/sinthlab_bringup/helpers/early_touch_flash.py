"""The early-touch cue on the ring: a few quick flashes of the whole ring (yellow by default), ending dark.

An early touch (actions/quiet_gate.py: the apple moved or pushed before the go cue) already plays a low
tone (go_cue.GoCue.warn_early_touch). This adds a visual cue that no other event uses, so the animal can
tell "too early" apart from "no reward" (dark) and "reward" (white). Settings, in every task's quiet_gate
block, all live:

    quiet_gate.flash_colour  [r, g, b, w] 0-255, at full value (visual_cue.brightness sets how bright)
    quiet_gate.flashes       how many flashes; 0 = no visual cue
    quiet_gate.flash_sec     each flash is on this long, then off this long [s]

Pure (no ROS), so the timing and the board request are tested on the Mac (test/test_early_touch_flash.py).
"""
from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

QUARTERS = 4


def schedule(flashes: int, flash_sec: float) -> List[Tuple[float, bool]]:
    """[(seconds after the touch, ring on?)]: on, off, on, off ... ending dark. Empty for 0 flashes."""
    n = max(0, int(flashes))
    step = max(0.02, float(flash_sec))
    return [(round(i * step, 4), i % 2 == 0) for i in range(2 * n)]


def colour(raw) -> Optional[Tuple[int, int, int, int]]:
    """[r, g, b] or [r, g, b, w] clamped to 0-255, as RGBW; None if missing, malformed or all dark."""
    if raw is None:
        return None
    try:
        vals = [max(0, min(255, int(v))) for v in raw]
    except (TypeError, ValueError):
        return None
    if len(vals) not in (3, 4) or not any(vals):
        return None
    return tuple(vals + [0] * (4 - len(vals)))


def ring_path(rgbw: Sequence[int]) -> str:
    """The board request lighting the whole ring in one colour (the same /segments request the ring cues
    use, so it goes out over Wi-Fi or, translated to SEG, over the serial link)."""
    part = ",".join(str(int(v)) for v in rgbw)
    return "/segments?factor=4&colors=" + ".".join([part] * QUARTERS)
