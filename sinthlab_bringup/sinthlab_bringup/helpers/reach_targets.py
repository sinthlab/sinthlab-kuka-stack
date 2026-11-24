"""The centre-out goal of each trial, and which part of the rail is open -- no ROS, so it can be unit-tested.

A rail has two ends, named by where they are (actions/centre_out.END_NAMES): ("down", "up") on Move
Vertical's rail, ("left", "right") on Move Horizontal's, ("min", "max") otherwise; the first is the
rail_min_m end. centre_out.target picks the trial's goal:

    either   both ends are goals                                   (a free choice: the whole ring lit)
    cued     one end per trial, drawn at random, never more than   (interleaved one-direction trials)
             max_repeats of the same end in a row
    <end>    that end on every trial: left / right, down / up      (one-direction trials, one side only)
    rule     the colour rule: a red or blue ring means one end or  (helpers/rule_schedule.py; the whole
             the other, and the mapping reverses unsignaled          rail stays open, it is a choice)

Changed between trials (it is live), so a session can go left only -> right only -> cued -> either.

With block_other_end, a trial with one goal end (cued or <end>) closes the other half of the rail: the
admittance clamps the equilibrium between the centre and the goal end, so the arm can only move toward
the goal -- one allowed direction per trial. Off, the whole rail stays open and reaching the other end
is reach_wrong (no reward).
"""
from __future__ import annotations

import random
from typing import Optional, Sequence, Tuple

BOTH = ("either", "cued")
RULE = "rule"      # the colour rule (helpers/rule_schedule.py): drawn by the ReversalSchedule, not here


def modes_for(ends: Sequence[str]) -> Tuple[str, ...]:
    """Every centre_out.target value valid on a rail with these two ends."""
    return BOTH + tuple(ends) + (RULE,)


def check_mode(mode: str, ends: Sequence[str]) -> Optional[str]:
    """None if `mode` is a valid centre_out.target on this rail, else the reason."""
    valid = modes_for(ends)
    return None if mode in valid else f"centre_out.target must be one of {list(valid)} on this rail, got {mode!r}"


class TargetPicker:
    """Draws each trial's goal for the current mode (module doc)."""

    def __init__(self, ends: Sequence[str], rng: Optional[random.Random] = None) -> None:
        if len(ends) != 2 or ends[0] == ends[1]:
            raise ValueError(f"a rail has two distinct ends, got {tuple(ends)!r}")
        self.ends = tuple(ends)
        self._rng = rng or random.Random()
        self.mode = "either"
        self.max_repeats = 0
        self._run_end: Optional[str] = None     # cued: the end drawn last, and how many times in a row
        self._run_len = 0

    def set_mode(self, mode: str, max_repeats: int = 0) -> None:
        """Set the mode (raises ValueError if it is not valid on this rail) and the repeat cap (0 = none).
        Changing the mode restarts the run count: a new mode is a new schedule."""
        problem = check_mode(mode, self.ends)
        if problem:
            raise ValueError(problem)
        if mode != self.mode:
            self._run_end, self._run_len = None, 0
        self.mode = mode
        self.max_repeats = max(0, int(max_repeats))

    def pick(self) -> str:
        """This trial's goal: an end's name, or "either"."""
        if self.mode == "either":
            return "either"
        if self.mode in self.ends:
            return self.mode
        if self.mode == RULE:
            raise ValueError("the colour rule draws its own goals (helpers/rule_schedule.py)")
        end = self._rng.choice(self.ends)       # cued
        if self.max_repeats and end == self._run_end and self._run_len >= self.max_repeats:
            end = self.other(end)
        self._run_len = self._run_len + 1 if end == self._run_end else 1
        self._run_end = end
        return end

    def other(self, end: str) -> str:
        return self.ends[1] if end == self.ends[0] else self.ends[0]


def open_span(target: str, ends: Sequence[str], rail_ends_m: Tuple[float, float],
              block_other_end: bool) -> Optional[Tuple[float, float]]:
    """The part of the rail (lo, hi) [m from the start] open in this trial, or None for the whole rail.

    With block_other_end and one goal end, the rail runs from the centre (the start, 0 m) to that end
    only. "either" always leaves the whole rail open."""
    if not block_other_end or target not in ends:
        return None        # (the colour rule is a choice: its monitor never asks to close anything)
    lo, hi = rail_ends_m
    centre = min(max(0.0, lo), hi)
    return (lo, centre) if target == ends[0] else (centre, hi)
