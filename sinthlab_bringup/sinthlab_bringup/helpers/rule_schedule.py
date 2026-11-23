"""Colour rule with unsignaled reversals: which end each cue colour means, and when the mapping flips.

No ROS, so it can be unit-tested (sinthlab_bringup/test/test_rule_schedule.py); actions/centre_out.py runs
it in the centre-out reach's `rule` mode (the rail cards' "Colour rule" tab).

THE TASK. At the go cue the whole ring lights red or blue (drawn at random, never more than max_repeats of
one colour in a row). One colour means one end of the rail, the other colour the other end: on Move
Horizontal, red = right and blue = left to begin with (rule.red_end). The whole rail is open: the trial
is a choice. Reaching the end the colour means is correct and rewarded; the other end is not.

THE REVERSAL. Once the last `window` choices of the current block are at least `criterion_pct` % correct,
and the block has run at least `min_trials` choices, the mapping flips from the next trial on: red now
means the other end. The contingency flips without any signal to the animal -- no tone, no light,
nothing on the ring changes. Only the outcomes tell him. Each block is a learning episode; a session
runs as many as he reaches criterion for.

Only choices count (a reach to either end). A trial with no reach -- the response time-out, an early
touch, a safety stop -- neither helps nor hurts the criterion.
"""
from __future__ import annotations

import random
from collections import deque
from typing import Optional, Sequence

from sinthlab_bringup.helpers.reach_targets import TargetPicker

COLOURS = ("red", "blue")


class ReversalSchedule:
    """The mapping in force, the block, and the criterion (module doc)."""

    def __init__(self, ends: Sequence[str], red_end: str, window: int = 20, criterion_pct: float = 80.0,
                 min_trials: int = 20, max_repeats: int = 3, rng: Optional[random.Random] = None) -> None:
        if len(ends) != 2:
            raise ValueError(f"a rail has two ends, got {tuple(ends)!r}")
        self.ends = tuple(ends)
        self._colours = TargetPicker(COLOURS, rng)
        self.configure(window, criterion_pct, min_trials, max_repeats)
        self.restart(red_end)

    # ------------------------------------------------------------------ settings and state
    def configure(self, window: int, criterion_pct: float, min_trials: int, max_repeats: int) -> None:
        """Live settings: they apply from the next choice, and never reset the block."""
        if window < 1 or not 0 < criterion_pct <= 100 or min_trials < 1:
            raise ValueError("need window >= 1, 0 < criterion_pct <= 100, min_trials >= 1")
        self.window, self.criterion_pct, self.min_trials = int(window), float(criterion_pct), int(min_trials)
        self._colours.set_mode("cued", max_repeats)
        old = list(getattr(self, "_recent", []))
        self._recent = deque(old[-self.window:], maxlen=self.window)

    def restart(self, red_end: str) -> None:
        """Start from block 1 with red meaning `red_end` (at the run's start, or on entering rule mode)."""
        if red_end not in self.ends:
            raise ValueError(f"rule.red_end must be one of {list(self.ends)} on this rail, got {red_end!r}")
        self.red_end = red_end
        self.block = 1
        self.choices_in_block = 0      # choices made in this block
        self.reversals = 0
        self._unannounced = False      # a reversal not yet marked in a trial (take_reversal)
        self._recent = deque(maxlen=self.window)

    # ------------------------------------------------------------------ trials
    def draw_colour(self) -> str:
        """This trial's cue colour."""
        return self._colours.pick()

    def end_for(self, colour: str) -> str:
        """The end that colour means under the current mapping."""
        if colour not in COLOURS:
            raise ValueError(f"cue colour must be one of {COLOURS}, got {colour!r}")
        blue_end = self.ends[1] if self.red_end == self.ends[0] else self.ends[0]
        return self.red_end if colour == "red" else blue_end

    def mapping(self) -> str:
        """The mapping in force, e.g. "red=right,blue=left"."""
        return f"red={self.end_for('red')},blue={self.end_for('blue')}"

    def window_pct(self) -> Optional[float]:
        """% correct over the last `window` choices of this block, or None before any."""
        return 100.0 * sum(self._recent) / len(self._recent) if self._recent else None

    def record(self, correct: Optional[bool]) -> bool:
        """The trial's choice: True correct, False wrong, None no choice (ignored). Returns True if this
        choice met the criterion, so the mapping flips from the next trial."""
        if correct is None:
            return False
        self.choices_in_block += 1
        self._recent.append(1 if correct else 0)
        pct = self.window_pct()
        if (self.choices_in_block >= self.min_trials and len(self._recent) >= self.window
                and pct is not None and pct >= self.criterion_pct):
            self.red_end = self.ends[1] if self.red_end == self.ends[0] else self.ends[0]
            self.block += 1
            self.reversals += 1
            self.choices_in_block = 0
            self._recent.clear()
            self._unannounced = True
            return True
        return False

    def take_reversal(self) -> bool:
        """True once after each reversal: the trial that calls it first (at its go cue) is the first of
        the new block, and marks the `reversal` event in its record. Nothing reaches the animal."""
        was, self._unannounced = self._unannounced, False
        return was

    def state(self) -> dict:
        """For the trial's sidecar (`rule`), recorded before the trial's choice."""
        pct = self.window_pct()
        return {"mapping": self.mapping(), "block": self.block, "choices_in_block": self.choices_in_block,
                "window_pct": None if pct is None else round(pct, 1), "reversals": self.reversals,
                "window": self.window, "criterion_pct": self.criterion_pct, "min_trials": self.min_trials}
