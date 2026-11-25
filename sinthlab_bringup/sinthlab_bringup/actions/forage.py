#!/usr/bin/env python3
"""Foraging: search a felt arena for hidden reward zones (design: robot tasks doc, section 2.2).

ONE ENVIRONMENT PER SESSION. The arena (free_move.box, its shape free_move.box.shape) and today's zones
(helpers/forage.py: forage.n_zones of forage.zone_radius_m, placed from the seed) are fixed at start-up
and the same on every trial. Each trial starts at the start pose, near the arena's centre:

    go cue -> he searches: the admittance, free in the plane facing him (free_move.rail_axis frontal) or in
              3-D (none), inside the arena's felt walls
    a zone   his hand stays in a zone he has not emptied this trial for forage.dwell_sec -> `zone` (its
             index; the reward pulse, the reward tone, and the reward cue -- the whole ring white -- for
             REWARD_FLASH_SEC). Entering an emptied zone -> `zone_revisit`; entering any zone `zone_enter`.
    all found -> `forage_done`; the reward cue stays until the hand lets go, as at any trial's end
    time limit (forage.time_limit_sec) -> `response_timeout`, the zones found so far having paid
    probe trial -> no cue and no rewards; it ends after forage.probe_sec -> `probe_end`

THE WARM-ZONE CUE. While his hand is within the trial's warm radius of a zone he has not emptied, the ring
shows ring_cue.colours.warm, dimmed to the trial's brightness (helpers/forage.warm_state): events
`warm_on` / `warm_off`. A reward flash takes the ring for its second, then the cue resumes.

Each trial's sidecar has `forage` (seed, arena, zones, warm radius and brightness, probe, dwell) and
`forage_result` (zones in the order found, the time each paid from the go, revisits, path length, time with
the cue on). It watches, reports and drives the ring; the motion is the admittance's.

Parameters: the arena, the zones and the seed are per-run (the environment is the session's); the dwell,
warm.* and probe.* are live, from the next trial. See config/forage.yaml.
"""
from __future__ import annotations

import random
import time
from typing import Callable, Optional

import numpy as np
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.helpers import forage as fg
from sinthlab_bringup.helpers.node_utils import get_launch_param, get_required_param

RATE_HZ = 100.0
REWARD_FLASH_SEC = 1.0          # a zone's reward cue (the whole ring white), then back to the warm cue
ALL = [0, 1, 2, 3]
LIVE = ("forage.time_limit_sec", "forage.dwell_sec", "forage.probe_sec", "warm.mode", "warm.start_factor", "warm.shrink_trials",
        "warm.fade_trials", "probe.probability", "probe.after_trial")


class ForageMonitor:
    def __init__(self, node: rclpyNode, admittance, ring, mark: Callable[[str, object], None]) -> None:
        if admittance.shape() not in fg.SPACES or admittance.box() is None:
            raise ValueError("Foraging needs free_move.rail_axis frontal or none, inside the box (the arena)")
        self._node, self._adm, self._ring, self._mark = node, admittance, ring, mark
        self._s = self._read()
        problem = fg.settings_problem(self._s)
        if problem:
            raise ValueError(f"Foraging: {problem}")
        subject = str(get_launch_param(node, "subject", "") or "")
        self.seed = self._s.seed or fg.day_seed(subject)
        self.zones = fg.zones_for(self._s, self.seed)        # raises with the reason if they do not fit
        self._rng = random.Random(self.seed + 1)               # the probe draws: the same every run of a day
        self.plan: Optional[fg.TrialPlan] = None
        self._on_zone = self._on_done = None
        node.get_logger().info(
            f"Foraging: today's environment, seed {self.seed}{' (from the date and the monkey)' if not self._s.seed else ''}: "
            f"a {self._s.shape} arena {'in the plane facing him' if self._s.space == 'frontal' else 'in 3-D'}, "
            f"{len(self.zones)} zone(s) of {self._s.zone_radius_m * 100:g} cm at "
            + "; ".join(str(np.round(z * 100, 1).tolist()) for z in self.zones) + " cm from the start.")
        node.create_timer(1.0 / RATE_HZ, self._tick)

    # ------------------------------------------------------------------ parameters
    def _read(self, override: Optional[dict] = None) -> fg.ForageSettings:
        over = override or {}
        get = lambda n: over[n] if n in over else get_required_param(self._node, n)  # noqa: E731
        s = fg.settings_from(lambda n: get(n) if not n.startswith("free_move.box.") or n == "free_move.box.shape"
                             else 0.0)
        lo, hi = self._adm.box()                               # the box as the admittance holds it (any preset)
        s.lo, s.hi = [float(v) for v in lo], [float(v) for v in hi]
        s.shape = self._adm.box_shape()
        return s

    def reload(self) -> None:
        """The live settings (dwell, warm cue, probes), from the next trial. The environment stays."""
        s = self._read()
        problem = fg.settings_problem(s)
        if problem:
            self._node.get_logger().error(f"Foraging: {problem}; keeping the previous settings.")
            return
        s.seed, s.n_zones, s.zone_radius_m = self._s.seed, self._s.n_zones, self._s.zone_radius_m
        self._s = s

    def check_live(self, name: str, value, context: dict) -> Optional[str]:
        if name not in LIVE:
            return None
        return fg.settings_problem(self._read({**{k: v for k, v in context.items() if k in LIVE}, name: value}))

    def settings(self) -> fg.ForageSettings:
        return self._s

    # ------------------------------------------------------------------ the trial
    def pick(self, trial: int) -> fg.TrialPlan:
        self.plan = fg.plan_trial(self._s, trial, self.seed, self.zones, self._rng)
        return self.plan

    def start(self, on_zone: Callable[[int], None], on_done: Callable[[str], None]) -> None:
        """At the go cue (armed): watch the search."""
        if self.plan is None:
            self.pick(1)
        now = time.monotonic()
        self._on_zone, self._on_done = on_zone, on_done
        self._tracker = fg.ZoneTracker(self._s, self.plan)
        self._t0, self._prev, self._path = now, None, 0.0
        self._warm, self._warm_since, self._warm_total = False, None, 0.0
        self._flash_until = 0.0
        self._t_zone = {}

    def stop(self) -> None:
        self._on_zone = self._on_done = None
        self.plan = None

    def result(self) -> dict:
        t = self._tracker
        warm = self._warm_total + ((time.monotonic() - self._warm_since) if self._warm and self._warm_since else 0.0)
        return {"found": list(t.found), "n_found": len(t.found), "n_zones": len(t.plan.zones),
                "t_zone_s": [self._t_zone.get(i) for i in t.found], "revisits": t.revisits,
                "path_m": round(self._path, 4), "warm_on_s": round(warm, 3)}

    # ------------------------------------------------------------------ the loop
    def _set_warm(self, on: bool, now: float) -> None:
        if on == self._warm:
            return
        self._warm = on
        if on:
            self._warm_since = now
            self._ring.show("warm", ALL, scale=self.plan.warm_brightness)
        else:
            if self._warm_since is not None:
                self._warm_total += now - self._warm_since
            self._warm_since = None
            self._ring.show("off")
        self._mark("warm_on" if on else "warm_off", None)

    def _tick(self) -> None:
        if self._on_done is None or self.plan is None:
            return
        p = self._adm.rest_offset_from_anchor()
        if p is None:
            return
        now = time.monotonic()
        dims = list(fg.dims_for(self._s.space))
        if self._prev is not None:
            self._path += float(np.linalg.norm((p - self._prev)[dims]))
        self._prev = p.copy()
        for ev, i in self._tracker.step(p, now):
            if ev == "zone":
                self._t_zone[i] = round(now - self._t0, 3)
                if self._warm and self._warm_since is not None:   # the reward cue takes the ring
                    self._warm_total += now - self._warm_since
                self._warm, self._warm_since = False, None
                self._mark("zone", i)                            # the reward pulse (a reward event)
                if self._on_zone is not None:
                    self._on_zone(i)
                if not self._tracker.done():
                    self._ring.reward()
                    self._flash_until = now + REWARD_FLASH_SEC
            else:
                self._mark("zone_revisit" if ev == "revisit" else "zone_enter", i)
        if self._tracker.done():
            on_done, self._on_done = self._on_done, None
            self._node.get_logger().info(f"Foraging: every zone found ({self._t_zone}).")
            on_done("forage_done")
            return
        if self.plan.probe and now - self._t0 >= self._s.probe_sec:
            on_done, self._on_done = self._on_done, None
            on_done("probe_end")
            return
        if now >= self._flash_until:
            if self._flash_until and self._ring.state() == "reward":
                self._ring.show("off")                           # the flash is over
                self._flash_until = 0.0
            self._set_warm(self._tracker.warm(p), now)
