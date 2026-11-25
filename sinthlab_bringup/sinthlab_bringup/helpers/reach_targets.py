"""The centre-out goal of each trial, and which part of the rail is open -- no ROS, so it can be unit-tested.

A rail has two ends, named by where they are (actions/centre_out.END_NAMES): ("down", "up") on Move
Vertical's rail, ("left", "right") on Move Horizontal's, ("min", "max") otherwise; the first is the
rail_min_m end. centre_out.target picks the trial's goal:

    either      both ends are goals                                (a free choice: the whole ring lit)
    restricted  RESTRICTED CUED: one end per trial, drawn at random  (the cued end's quarter lit; the arm
                (never more than max_repeats of the same end in a     can only move toward it)
                row), the other half of the rail CLOSED
    retry       CUED, CAN CORRECT: drawn the same way, the whole     (the cued end's quarter lit; the
                rail OPEN; reaching the other end does nothing        first training step with both
                (event wrong_end) and he may go on to the cued end    directions open)
    cued        CUED, ONE CHOICE: drawn the same way, the whole rail (the cued end's quarter lit; a real
                OPEN: only the cued end pays; the other end is        choice)
                reach_wrong, which ends the trial and parks at once
    rule        the colour rule: a red or blue ring means one end or (helpers/rule_schedule.py; the whole
                the other, and the mapping reverses unsignaled         rail stays open, it is a choice)

Changed between trials (it is live), so a session can go restricted cued -> can correct -> one choice -> either.

Restricted cued closes the other half of the rail: the admittance clamps the equilibrium between the
centre and the cued end, so the arm can only move toward it -- the cue always matches the one open
direction. The two cued modes leave the whole rail open: going the wrong way is wrong_end (can correct: the
trial goes on) or reach_wrong (one choice: no reward, the trial ends). outcome() decides which.
"""
from __future__ import annotations

import random
from typing import Optional, Sequence, Tuple

RESTRICTED = "restricted"
RETRY = "retry"                       # cued, can correct: the wrong end does not end the trial
CUED_MODES = ("cued", RESTRICTED, RETRY)     # one end per trial, drawn at random
RULE = "rule"      # the colour rule (helpers/rule_schedule.py): drawn by the ReversalSchedule, not here


def modes_for(ends: Sequence[str]) -> Tuple[str, ...]:
    """Every centre_out.target value (the same on every rail), in the order the dashboard offers them."""
    return ("either", RESTRICTED, RETRY, "cued", RULE)


def outcome(mode: str, target: Optional[str], end: str) -> str:
    """What reaching `end` means this trial: "reward" (a goal end), "retry" (the wrong end, can correct:
    nothing happens, the trial goes on), or "wrong" (the wrong end, one choice: the trial ends unrewarded)."""
    if target == "either" or end == target:
        return "reward"
    return "retry" if mode == RETRY else "wrong"


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
        self._run_end: Optional[str] = None     # cued / restricted: the end drawn last, and how many in a row
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
        if self.mode == RULE:
            raise ValueError("the colour rule draws its own goals (helpers/rule_schedule.py)")
        end = self._rng.choice(self.ends)       # cued, restricted: at random, capped runs
        if self.max_repeats and end == self._run_end and self._run_len >= self.max_repeats:
            end = self.other(end)
        self._run_len = self._run_len + 1 if end == self._run_end else 1
        self._run_end = end
        return end

    def other(self, end: str) -> str:
        return self.ends[1] if end == self.ends[0] else self.ends[0]


def open_span(target: str, ends: Sequence[str], rail_ends_m: Tuple[float, float],
              close_other_half: bool) -> Optional[Tuple[float, float]]:
    """The part of the rail (lo, hi) [m from the start] open in this trial, or None for the whole rail.

    Closing the other half (restricted cued) with one goal end, the rail runs from the centre (the start,
    0 m) to that end only. "either" always leaves the whole rail open."""
    if not close_other_half or target not in ends:
        return None        # (the colour rule is a choice: its monitor never asks to close anything)
    lo, hi = rail_ends_m
    centre = min(max(0.0, lo), hi)
    return (lo, centre) if target == ends[0] else (centre, hi)
