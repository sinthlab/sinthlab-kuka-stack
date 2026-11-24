#!/usr/bin/env python3
"""Cued Pull: grip the apple and pull it, with jump and bump perturbations (the CIHR grant's Aims 1-2).

The arm is free in the plane facing the monkey (free_move.rail_axis frontal), inside the box. ONE PULL PER
TRIAL. The trial's plan (helpers/pull_task.py) is drawn at the go cue; the ring shows it (the whole ring
in shaping, the cued direction's quarter in jump / bump). The pull ends when the arm has gone
pull.distance_m from where the pull started:

    shaping    any direction -> `pull` (reward)
    jump/bump  within pull.cone_deg of the cued direction -> `pull` (reward); outside it -> `pull_wrong`
    neither within response_timeout_sec -> `response_timeout`, as every trial

THE INSTRUCTED DELAY. The ring shows the trial's cue for a delay drawn per trial from
pull.cue_delay_min_sec .. max_sec before the go (light off + tone); the quiet window's guard runs on
through it, so moving early is an early touch. 0.3 s each is the standard cue flash of every task.

THE GRIP. The first moment the hand's force on the apple (every direction, tared) stays at least
grip.threshold_n for grip.hold_sec, or the arm has been moved grip.move_m: event `grip` (the force, N).
Its time is the reaction time in every mode.

THE JUMP (mode jump, perturbed trials). A delay drawn per trial from perturb.jump_delay_min_sec ..
max_sec after the go cue, if he has not gripped yet, the apple jumps perturb.jump_m at
perturb.jump_angles_deg from the cued direction, in perturb.jump_sec: events `jump` (angle, at its
onset) and `jump_end`. The pull is then measured from where it landed. If he grips first, there is no
jump this trial: `jump_skipped` (the trial is a standard trial; its plan stays in the sidecar).
Without a hand tracker the delay stands in for "while the hand is on its way"; a camera-based
proximity trigger can replace it later.

THE BUMP (mode bump, perturbed trials). Once he has pulled perturb.bump_at_m along the cued direction,
the apple is pushed perturb.bump_m at perturb.bump_angles_deg (90 / 270 = perpendicular) in
perturb.bump_sec: events `bump` (angle, at its onset) and `bump_end`. The target does not move: he has
to correct to finish inside the cone.

Both are AdmittanceMoveAction.displace(): the equilibrium is driven, the hand's force does not move it
meanwhile, and the admittance carries on from there. The pull's outcome is not judged during one.

Each trial's sidecar has `pull` (helpers/pull_task.TrialPlan.record: mode, direction, perturbation and
angle, block) and `pull_result` (delivered perturbation, crossing angle and error). It only watches,
reports and displaces: the motion is the admittance's.

Parameters (all live, from the next trial; the box is per-run): pull.*, perturb.*, grip.* -- see
config/cued_pull.yaml.
"""
from __future__ import annotations

import random
import time
from typing import Callable, Optional

import numpy as np
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.helpers import pull_task as pt
from sinthlab_bringup.helpers.pull_task import PARAMS, settings_from
from sinthlab_bringup.helpers.node_utils import get_required_param

RATE_HZ = 100.0
GRIP_BLANK_SEC = 0.1      # after a jump or bump, the force reading carries the arm's own acceleration: no grip check

class CuedPullMonitor:
    def __init__(self, node: rclpyNode, admittance, mark: Callable[[str, object], None], *,
                 rng: Optional[random.Random] = None) -> None:
        if admittance.shape() != "frontal" or admittance.box() is None:
            raise ValueError("Cued Pull needs free_move.rail_axis: frontal (free in the plane facing the monkey, "
                             "inside the box)")
        self._node, self._adm, self._mark = node, admittance, mark
        self._rng = rng or random.Random()
        self._settings = self._read()
        problem = pt.settings_problem(self._settings) or pt.box_problem(self._settings, *admittance.box())
        if problem:
            raise ValueError(f"Cued Pull: {problem}")
        self._schedule = pt.PullSchedule(self._settings, self._rng)
        self.plan: Optional[pt.TrialPlan] = None
        self._delay = 0.3
        self._on_outcome: Optional[Callable[[bool, float], None]] = None
        self.reload()
        node.create_timer(1.0 / RATE_HZ, self._tick)

    # ------------------------------------------------------------------ parameters
    def _read(self, override: Optional[dict] = None) -> pt.PullSettings:
        over = override or {}
        return settings_from(lambda n: over[n] if n in over else get_required_param(self._node, n))

    def reload(self) -> None:
        n = self._node
        s = self._read()
        problem = pt.settings_problem(s) or pt.box_problem(s, *self._adm.box())
        if problem:          # a live change is checked before it is accepted (check_live); this guards a race
            n.get_logger().error(f"Cued Pull: {problem}; keeping the previous settings.")
        else:
            self._settings = s
        self._schedule.configure(self._settings)
        self._grip_n = float(get_required_param(n, "grip.threshold_n"))
        self._grip_sec = float(get_required_param(n, "grip.hold_sec"))
        self._grip_m = float(get_required_param(n, "grip.move_m"))
        lo = float(get_required_param(n, "pull.cue_delay_min_sec"))
        self._cue_delay = (lo, max(lo, float(get_required_param(n, "pull.cue_delay_max_sec"))))
        s = self._settings
        how = ("shaping: any direction" if s.mode == "shaping" else
               f"cued {', '.join(s.directions)} within {s.cone_deg:g} deg; "
               + (f"{s.probability:.0%} of trials {s.mode}ed {s.size() * 100:g} cm at {s.angles()} deg"
                  if s.probability > 0 and s.angles() else "no perturbations"))
        n.get_logger().info(f"Cued Pull: {how}; a pull ends at {s.distance_m * 100:g} cm.")

    def check_live(self, name: str, value, context: dict) -> Optional[str]:
        """ExperimentControl validator: the settings must make sense together and fit the box."""
        if name in ("pull.cue_delay_min_sec", "pull.cue_delay_max_sec"):
            get = lambda k: value if k == name else context.get(k, get_required_param(self._node, k))  # noqa: E731
            if float(get("pull.cue_delay_min_sec")) > float(get("pull.cue_delay_max_sec")):
                return "pull.cue_delay_min_sec must be <= pull.cue_delay_max_sec"
            return None
        if name not in PARAMS:
            return None
        s = self._read({**{k: v for k, v in context.items() if k in PARAMS}, name: value})
        return pt.settings_problem(s) or pt.box_problem(s, *self._adm.box())

    # ------------------------------------------------------------------ the trial
    def mode(self) -> str:
        return self._settings.mode

    def pick(self) -> pt.TrialPlan:
        """This trial's plan, drawn at the go cue."""
        self.plan = self._schedule.next()
        self._delay = round(self._rng.uniform(*self._cue_delay), 3)
        return self.plan

    def cue_delay(self) -> float:
        """How long the cue shows before the go, this trial [s] (drawn once per trial, at pick())."""
        return self._delay

    def quarters(self):
        """The ring quarters lit at the go cue: the cued direction's, or the whole ring in shaping."""
        if self.plan is None or self.plan.direction is None:
            return [0, 1, 2, 3]
        return [pt.QUARTER[self.plan.direction]]

    def target(self) -> str:
        return "any" if self.plan is None or self.plan.direction is None else self.plan.direction

    def result(self) -> dict:
        return dict(self._result)

    def start(self, on_outcome: Callable[[bool, float], None]) -> None:
        """At the go cue (armed): watch the pull; on_outcome(correct, crossing angle) once."""
        if self.plan is None:
            self.pick()
        s, now = self._settings, time.monotonic()
        self._on_outcome = on_outcome
        self._t0 = now
        self._baseline = np.zeros(3)          # where the pull is measured from (moves with a jump)
        self._gripped = False
        self._grip_since: Optional[float] = None
        self._blank_until = 0.0
        self._fired = False
        self._jump_at = (now + self._rng.uniform(s.jump_delay_min_sec, s.jump_delay_max_sec)
                         if self.plan.perturb == "jump" else None)
        self._result = {"perturbed": 0, "t_grip_s": None, "t_perturb_s": None,
                        "crossing_deg": None, "error_deg": None}

    def stop(self) -> None:
        self._on_outcome = None
        self.plan = None

    # ------------------------------------------------------------------ the loop
    def _done_displacing(self, token: str) -> None:
        self._blank_until = time.monotonic() + GRIP_BLANK_SEC
        self._mark(token, None)

    def _perturb(self, kind: str) -> None:
        plan, s = self.plan, self._settings
        off = pt.to_base(plan.offset())
        sec = s.jump_sec if kind == "jump" else s.bump_sec
        if not self._adm.displace(off, sec, on_done=lambda: self._done_displacing(kind + "_end")):
            self._node.get_logger().warn(f"Cued Pull: the {kind} could not start (the arm is not free to move).")
            return
        if kind == "jump":                    # the pull now starts where the apple lands
            self._baseline = self._baseline + off
        self._fired = True
        self._result.update(perturbed=1, t_perturb_s=round(time.monotonic() - self._t0, 3))
        self._mark(kind, plan.angle_deg)

    def _tick(self) -> None:
        if self._on_outcome is None or self.plan is None:
            return
        p = self._adm.rest_offset_from_anchor()
        if p is None or self._adm.displacing():
            return
        now = time.monotonic()
        d = pt.frontal(p - self._baseline)
        if not self._gripped and now >= self._blank_until:
            held = self._adm.hand_force_n() >= self._grip_n
            self._grip_since = (self._grip_since or now) if held else None
            if (held and now - self._grip_since >= self._grip_sec) or float(np.hypot(*d)) >= self._grip_m:
                self._gripped = True
                self._result["t_grip_s"] = round(now - self._t0, 3)
                self._mark("grip", round(self._adm.hand_force_n(), 2))
        plan, s = self.plan, self._settings
        if plan.perturb == "jump" and not self._fired and self._jump_at is not None:
            if self._gripped:
                self._jump_at = None
                self._mark("jump_skipped", None)
            elif now >= self._jump_at:
                self._perturb("jump")
                return
        if plan.perturb == "bump" and not self._fired and float(d @ pt.unit(plan.direction)) >= s.bump_at_m:
            self._perturb("bump")
            return
        out = pt.crossing(d, s.distance_m, plan.direction, s.cone_deg)
        if out is None:
            return
        correct, ang, err = out
        self._result.update(crossing_deg=ang, error_deg=err)
        on_outcome, self._on_outcome = self._on_outcome, None
        self._node.get_logger().info(
            f"Cued Pull: pulled toward {ang:g} deg" + ("" if err is None else f" ({err:+g} from {plan.direction})")
            + (" -- reward." if correct else " -- outside the target, no reward."))
        on_outcome(correct, ang)
