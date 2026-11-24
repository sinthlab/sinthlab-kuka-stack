#!/usr/bin/env python3
"""Centre-out reach on a rail -- the rewarded version of Move Vertical / Move Horizontal.

ONE REACH PER TRIAL. The centre is the start; the two goals are the rail's ends (free_move.rail_min_m
and rail_max_m from the start). At the go cue the ring lights green, and the trial ends at the first end
the arm reaches:

    go cue          the ring lights -- event `target` (an end's name, or "either")
    a goal end      `reach` -> the reward pulse, then the release wait and park, as any trial
    the other end   `reach_wrong` (cued only) -> no reward; the release wait and park
    neither within response_timeout_sec -> `response_timeout`, as in every experiment

    centre_out.target  either -- both ends are goals; the WHOLE ring lights (green at the go cue, red at the
                                 end): no direction is cued, so nothing tells him which way to go
                       cued   -- one end drawn at random per trial (never more than max_repeats of the
                                 same end in a row); only its quarter lights (green, then red when it is
                                 reached) and only it is rewarded
                       <end>  -- that end on every trial (left / right, down / up): one-direction
                                 trials, its quarter lit as in cued -- the step before cued, where the lit
                                 quarter always matches the one open direction. Live, so a session can go
                                 left only -> right only -> cued.
                       rule   -- the colour rule (helpers/rule_schedule.py): the whole ring lights red
                                 or blue, each colour means one end, the whole rail is open, and the
                                 mapping reverses without any signal to the animal once he reaches
                                 criterion (the rule block: red_end, window, criterion_pct, min_trials,
                                 max_repeats). Entering it starts at block 1 with red = rule.red_end.

ONE DIRECTION PER TRIAL. With centre_out.block_other_end, a trial with one goal end (cued or <end>) closes
the other half of the rail (AdmittanceMoveAction.set_open_span): the arm moves only between the centre and
the goal end, so it cannot reach the wrong end. Off, the whole rail is open and the other end is
`reach_wrong`. The goal logic itself is helpers/reach_targets.py (no ROS, unit-tested).

THE ENDS ARE NAMED by where they are, seen from the front of the arm (README, "The ring"): the
rail_min_m / rail_max_m end is down / up on a z rail (Move Vertical), left / right on a y rail (Move
Horizontal). Every reach event carries the end's name. The position is
the arm's own, along the rail (AdmittanceMoveAction.rail_position), not the equilibrium, so a reach
counts only when the arm is really there. It only watches and reports: the motion is the admittance's.

Parameters (all live, read at the next trial):
    centre_out.enabled        bool   off = Move Vertical / Horizontal as plain free movement on the rail
    centre_out.target         str    either | cued | an end's name (left / right, down / up)
    centre_out.block_other_end bool  a trial with one goal end closes the other half of the rail
    centre_out.max_repeats    int    cued: at most this many trials in a row to the same end (0 = no cap)
    centre_out.end_tol_m      float  this close to an end counts as reaching it [m]
    centre_out.quarter_<end>  int    ring quarter (0-3) that lights for that end: quarter_down / quarter_up
                                     (Move Vertical), quarter_left / quarter_right (Move Horizontal)
"""
from __future__ import annotations

from typing import Callable, List, Optional, Tuple

from rclpy.node import Node as rclpyNode

from sinthlab_bringup.helpers.node_utils import get_required_param, get_launch_param
from sinthlab_bringup.helpers.reach_targets import RULE, TargetPicker, check_mode, open_span as rail_open_span
from sinthlab_bringup.helpers.rule_schedule import ReversalSchedule

# The (rail_min_m, rail_max_m) ends' names, by the rail's base axis (an x rail: "min" / "max").
END_NAMES = {"z": ("down", "up"), "y": ("left", "right")}
RATE_HZ = 50.0


class CentreOutMonitor:
    def __init__(self, node: rclpyNode, admittance, *, param_prefix: str = "centre_out") -> None:
        self._node = node
        self._adm = admittance
        self._p = param_prefix + "."
        self._on_end: Optional[Callable[[str, bool], None]] = None
        self._target: Optional[str] = None
        # No rail (free_move.rail_axis frontal, the colour rule's "No rail"): the arm is free in the plane
        # facing the monkey, and a choice is reaching centre_out.free_reach_m to the left or right.
        self._free = getattr(admittance, "shape", lambda: None)() == "frontal"
        axis = "y" if self._free else admittance.rail_axis()
        self._ends = END_NAMES.get(axis, ("min", "max"))   # (rail_min_m end, rail_max_m end)
        self._free_ends: Optional[Tuple[float, float]] = None
        self._picker = TargetPicker(self._ends)
        self._colour: Optional[str] = None
        # The colour rule (centre_out.target "rule"): built from the rule block; restarted on entering it.
        self._rule: Optional[ReversalSchedule] = None
        if node.has_parameter("rule.red_end"):
            self._rule = ReversalSchedule(self._ends, str(get_required_param(node, "rule.red_end")).strip().lower())
        self._first_load = True
        self.reload()
        node.create_timer(1.0 / RATE_HZ, self._tick)

    def reload(self) -> None:
        n, p = self._node, self._p
        if self._free:
            d = float(get_required_param(n, p + "free_reach_m"))
            self._free_ends = (-d, d)
        self.enabled = bool(get_required_param(n, p + "enabled")) and self._ends_m() is not None
        mode = str(get_required_param(n, p + "target")).strip().lower()
        self._block = bool(get_launch_param(n, p + "block_other_end", False))
        max_repeats = int(get_launch_param(n, p + "max_repeats", 0))
        was = None if self._first_load else self._picker.mode
        try:
            if mode == RULE and self._rule is None:
                raise ValueError("centre_out.target rule needs the rule block in this experiment's YAML")
            self._picker.set_mode(mode, max_repeats)
        except ValueError as exc:
            if self._first_load:
                raise
            # A live change is checked before it is accepted (check_live); this only guards a race.
            n.get_logger().error(f"Centre-out: {exc}; keeping {self._picker.mode}.")
            self._picker.set_mode(self._picker.mode, max_repeats)
        self._first_load = False
        if self._rule is not None:
            self._rule.configure(int(get_required_param(n, "rule.window")),
                                 float(get_required_param(n, "rule.criterion_pct")),
                                 int(get_required_param(n, "rule.min_trials")),
                                 int(get_required_param(n, "rule.max_repeats")))
            if self._picker.mode == RULE and was != RULE:     # entering the rule: block 1, rule.red_end
                self._rule.restart(str(get_required_param(n, "rule.red_end")).strip().lower())
        self._end_tol = float(get_required_param(n, p + "end_tol_m"))
        self._quarter = {end: int(get_required_param(n, p + "quarter_" + end)) for end in self._ends}
        if self.enabled:
            lo, hi = self._ends_m()
            mode = self._picker.mode
            r = self._rule
            how = (f"{mode} only" if mode in self._ends else
                   f"cued (at most {self._picker.max_repeats} in a row)" if mode == "cued" and self._picker.max_repeats
                   else f"colour rule, {r.mapping()} (block {r.block}; reverses unsignaled at "
                        f"{r.criterion_pct:g}% of the last {r.window} choices, after at least {r.min_trials})"
                   if mode == RULE else mode)
            n.get_logger().info(
                f"Centre-out: one reach per trial, {how}"
                + (", the other half of the rail closed" if self._block and mode not in ("either", RULE) else "")
                + ("; no rail (free in the frontal plane): a choice is reaching" if self._free else
                   "; an end counts within {:.0f} mm of".format(self._end_tol * 1000))
                + f" the {self._ends[0]} ({lo:+.3f} m) / {self._ends[1]} ({hi:+.3f} m) end.")

    def _ends_m(self) -> Optional[Tuple[float, float]]:
        """Where the two ends are along the choice axis [m from the start]: the rail's, or with no rail
        (+-free_reach_m), the distance left or right that counts as a choice."""
        return self._free_ends if self._free else self._adm.rail_ends()

    def _position(self) -> Optional[float]:
        """Where the arm is along the choice axis (where it would rest) [m from the start]."""
        if not self._free:
            return self._adm.rail_position()
        p = self._adm.rest_offset_from_anchor()
        return None if p is None else float(p[1])          # base +Y: the monkey's right

    def check_live(self, name: str, value, context: dict) -> Optional[str]:
        """ExperimentControl validator: refuse a centre_out.target this rail does not have (e.g. "left" on
        Move Vertical's rail) before it is accepted, instead of at the next trial."""
        if name == "rule.red_end":
            v = str(value).strip().lower()
            return None if v in self._ends else f"rule.red_end must be one of {list(self._ends)} on this rail, got {v!r}"
        if name != self._p + "target":
            return None
        mode = str(value).strip().lower()
        if mode == RULE and self._rule is None:
            return "this experiment's YAML has no rule block"
        return check_mode(mode, self._ends)

    # ------------------------------------------------------------------ the colour rule
    def rule_on(self) -> bool:
        return self._picker.mode == RULE

    def cue_colour(self) -> Optional[str]:
        """This trial's cue colour (the colour rule), else None."""
        return self._colour

    def rule_state(self) -> Optional[dict]:
        """The rule's state for the trial's sidecar (`rule`), before the trial's choice."""
        return self._rule.state() if self.rule_on() else None

    def take_reversal(self) -> Optional[str]:
        """The new mapping, once, on the first trial after a reversal; else None."""
        return self._rule.mapping() if self.rule_on() and self._rule.take_reversal() else None

    def ring_cue(self) -> Tuple[str, List[int]]:
        """What the ring shows at the go cue: (ring state, quarters). The colour rule lights the whole ring
        in the cue colour (ring_cue.colours.rule_red / rule_blue); otherwise quarters(), green."""
        if self.rule_on() and self._colour:
            return f"rule_{self._colour}", [0, 1, 2, 3]
        return "go", self.quarters()

    def open_span(self) -> Optional[Tuple[float, float]]:
        """This trial's open part of the rail (lo, hi) [m from the start], or None for the whole rail."""
        if self._target is None or self._adm.rail_ends() is None or self.rule_on() or self._free:
            return None             # the colour rule is a choice, and no rail has nothing to close
        return rail_open_span(self._target, self._ends, self._adm.rail_ends(), self._block)

    def closed_end(self) -> Optional[str]:
        """The end closed off this trial, or None if the whole rail is open."""
        return None if self.open_span() is None else self._picker.other(self._target)

    def pick_target(self) -> str:
        """The goal for this trial: an end's name (cued, drawn at random; or the one end of a one-direction
        mode; or the end the cue colour means under the colour rule) or "either"."""
        if self.rule_on():
            self._colour = self._rule.draw_colour()
            self._target = self._rule.end_for(self._colour)
        else:
            self._colour = None
            self._target = self._picker.pick()
        return self._target

    def quarter_of(self, end: str) -> Optional[int]:
        """The ring quarter of one end of the rail, or None for an unknown name."""
        return self._quarter.get(end)

    def quarters(self) -> List[int]:
        """The ring quarters to light green for this trial: the goal end's quarter when one end is the goal
        (cued, left / right, down / up); the whole ring for either end, so no direction is cued."""
        if self._target in self._ends:
            return [self._quarter[self._target]]
        return [0, 1, 2, 3]

    def reached_quarters(self, end: str) -> List[int]:
        """The ring quarters to turn red once the arm has reached `end`: that end's quarter in the one-end
        modes, the whole ring for either end (both ends pay: still no direction shown)."""
        if self._target in self._ends and end in self._quarter:
            return [self._quarter[end]]
        return self.quarters()

    def start(self, on_end: Callable[[str, bool], None]) -> None:
        """At the go cue: watch until the arm reaches an end, then call on_end(end, rewarded) once."""
        if self._target is None:
            self.pick_target()
        self._on_end = on_end

    def stop(self) -> None:
        self._on_end = None
        self._target = None
        self._colour = None

    def _tick(self) -> None:
        if self._on_end is None:
            return
        x = self._position()
        if x is None:
            return
        lo, hi = self._ends_m()
        tol = 0.0 if self._free else self._end_tol      # no rail: reaching the distance is the choice
        end = (self._ends[0] if x <= lo + tol else
               self._ends[1] if x >= hi - tol else None)
        if end is None:
            return
        rewarded = self._target == "either" or end == self._target
        on_end, self._on_end = self._on_end, None
        self._node.get_logger().info(
            f"Centre-out: reached the {end} end" + (" -- reward." if rewarded else
                                                    f", the target was {self._target} -- no reward."))
        if self.rule_on() and self._rule.record(rewarded):
            # Logged for the operator only: nothing reaches the animal.
            self._node.get_logger().info(
                f"Colour rule: criterion met -- the mapping reverses, unsignaled, from the next trial: "
                f"{self._rule.mapping()} (block {self._rule.block}).")
        on_end(end, rewarded)
