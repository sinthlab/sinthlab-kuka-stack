"""The quiet-window staircase: lengthen the wait before the go cue as the animal learns to leave the apple alone.

Training starts with a very short quiet window (quiet_window_sec, 0.5 s by default) and lengthens it. By
hand, change quiet_window_sec on the dashboard (live, from the next trial). Or switch this on
(quiet_ramp.enabled), and ExperimentControl does it after every trial, in every task:

    a trial that reached the go cue (no early touch)  counts toward the streak; after quiet_ramp.after_trials
                                                      of them in a row, the window grows by step_sec
    an early touch                                    the window shrinks by back_sec, the streak restarts
    anything else (Stop, Pause, a fault before the go) changes nothing

within [quiet_ramp.min_sec, quiet_ramp.max_sec]. The new value is set as the live parameter
quiet_window_sec, so it applies from the next trial, the dashboard shows it, and each trial's sidecar
records the window it ran with. Off by default: how fast to ramp is decided with the animal.

Pure (no ROS): tested in test/test_quiet_ramp.py.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Tuple

PARAMS = ("enabled", "step_sec", "after_trials", "back_sec", "min_sec", "max_sec")
GUARD_BASELINE_SEC = 0.2      # quiet_gate.BASELINE_SEC: the window's first stretch is the resting reference


@dataclass
class RampSettings:
    enabled: bool = False
    step_sec: float = 0.1
    after_trials: int = 3
    back_sec: float = 0.1
    min_sec: float = 0.3
    max_sec: float = 2.0


def settings_from(get: Callable[[str], object]) -> RampSettings:
    """From the quiet_ramp.* parameters (get(name) returns a value)."""
    s = RampSettings()
    for name in PARAMS:
        v = get(f"quiet_ramp.{name}")
        setattr(s, name, bool(v) if name == "enabled" else int(v) if name == "after_trials" else float(v))
    return s


def step(current: float, went: bool, early: bool, streak: int, s: RampSettings) -> Tuple[float, int, Optional[str]]:
    """After one trial: (the next quiet window, the streak, why it changed or None)."""
    if not s.enabled:
        return current, 0, None
    lo, hi = min(s.min_sec, s.max_sec), max(s.min_sec, s.max_sec)
    if early:
        new = round(max(lo, current - s.back_sec), 3)
        return new, 0, (f"an early touch: quiet window {new:g} s" if new != current else None)
    if not went:
        return current, streak, None
    streak += 1
    if streak < max(1, s.after_trials):
        return current, streak, None
    new = round(min(hi, current + s.step_sec), 3)
    return new, 0, (f"{streak} trials in a row without an early touch: quiet window {new:g} s"
                    if new != current else None)
