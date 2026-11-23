"""N-choice with depleting rewards: each direction's reward probability, and how it changes with choices.

No ROS, so it can be unit-tested (sinthlab_bringup/test/test_n_choice.py); actions/n_choice.py runs it on
the asterisk of rails (helpers/rail_shapes.py).

Each trial he reaches out along one ray of the asterisk. That direction pays with its current probability
p. Then the probabilities change:

    the direction chosen   depletes: p -> p - deplete * (p - p_min)    (falls toward p_min)
    every other direction  recovers: p -> p + recover * (p_max - p)    (rises toward p_max)

so a direction he keeps choosing gets worse, and one he leaves alone gets better again. Nothing signals
which direction is best: he has to track it from the rewards he gets. With start `full` every direction
begins at p_max; with `random`, each begins anywhere between p_min and p_max.

The probabilities before each choice are kept in the trial's record (for fitting learning models), with
the direction chosen and whether it paid.
"""
from __future__ import annotations

import random
from typing import Dict, List, Optional, Sequence

STARTS = ("full", "random")


class DepletingBandit:
    def __init__(self, names: Sequence[str], p_max: float = 0.9, p_min: float = 0.1, deplete: float = 0.3,
                 recover: float = 0.1, start: str = "full", rng: Optional[random.Random] = None) -> None:
        if len(names) < 2 or len(set(names)) != len(names):
            raise ValueError(f"need 2 or more distinct options, got {list(names)}")
        self.names = list(names)
        self._rng = rng or random.Random()
        self.configure(p_max, p_min, deplete, recover)
        self.restart(start)

    def configure(self, p_max: float, p_min: float, deplete: float, recover: float) -> None:
        """Live settings: they apply from the next choice; the current probabilities are kept, clipped to
        the new range."""
        if not 0.0 <= p_min <= p_max <= 1.0:
            raise ValueError("need 0 <= p_min <= p_max <= 1")
        if not (0.0 <= deplete <= 1.0 and 0.0 <= recover <= 1.0):
            raise ValueError("deplete and recover are fractions, 0-1")
        self.p_max, self.p_min, self.deplete, self.recover = float(p_max), float(p_min), float(deplete), float(recover)
        if hasattr(self, "p"):
            self.p = [min(self.p_max, max(self.p_min, x)) for x in self.p]

    def restart(self, start: str = "full") -> None:
        if start not in STARTS:
            raise ValueError(f"start must be one of {STARTS}, got {start!r}")
        self.p: List[float] = ([self.p_max] * len(self.names) if start == "full" else
                               [self._rng.uniform(self.p_min, self.p_max) for _ in self.names])
        self.choices = 0

    def probabilities(self) -> Dict[str, float]:
        return {n: round(p, 4) for n, p in zip(self.names, self.p)}

    def best(self) -> List[str]:
        """The direction(s) with the highest probability now."""
        top = max(self.p)
        return [n for n, p in zip(self.names, self.p) if abs(p - top) < 1e-9]

    def choose(self, name: str) -> bool:
        """He chose `name`: draw its reward with its current probability, then deplete it and let the others
        recover. Returns whether it paid."""
        i = self.names.index(name)
        rewarded = self._rng.random() < self.p[i]
        for j in range(len(self.p)):
            if j == i:
                self.p[j] -= self.deplete * (self.p[j] - self.p_min)
            else:
                self.p[j] += self.recover * (self.p_max - self.p[j])
        self.choices += 1
        return rewarded
