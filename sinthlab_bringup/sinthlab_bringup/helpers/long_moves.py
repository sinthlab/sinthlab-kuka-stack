"""Long-movement reward for Free Move: split the arm's motion into movements, reward the long ones.

No ROS, so it can be unit-tested (sinthlab_bringup/test/test_long_moves.py); actions/long_move.py runs it
on the arm.

A MOVEMENT is one stretch of motion between two rests:

    onset      the arm's smoothed speed rises above start_speed_mps. Where it last rested is the
               movement's start point.
    confirmed  it has gone confirm_m from that start point. Only then is it reported (`move_start`):
               a resting hand bobs the arm by up to ~7 mm at ~1.6 Hz (README, Free Move), fast
               enough to cross the speed threshold but never this far.
    end        the speed has stayed below stop_speed_mps for stop_sec (the let-go glide is part of
               the movement).

Its LENGTH is the straight-line distance from start point to end point; its STRAIGHTNESS is that length
over the path actually travelled (1 = a straight line). A movement is rewarded when it ends, if it is at
least min_m long, at least min_straightness straight (0 = not checked), and at least min_gap_sec after
the last reward.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np


@dataclass
class Movement:
    t_onset: float          # when the speed first crossed start_speed_mps [s]
    t_end: float            # when it was declared over (stop_sec after it slowed down) [s]
    length_m: float         # straight-line distance, start point to end point
    path_m: float           # distance actually travelled
    peak_speed_mps: float

    @property
    def straightness(self) -> float:
        return self.length_m / self.path_m if self.path_m > 0 else 0.0

    @property
    def duration_sec(self) -> float:
        return self.t_end - self.t_onset

    def summary(self) -> str:
        """The `move_end` event's argument."""
        return (f"length={self.length_m:.3f};straightness={self.straightness:.2f};"
                f"duration={self.duration_sec:.2f};peak_speed={self.peak_speed_mps:.3f}")


class MovementSegmenter:
    """Feed it (time, position) samples; it reports each movement's confirmation and end."""

    def __init__(self, start_speed_mps: float = 0.03, stop_speed_mps: float = 0.02, stop_sec: float = 0.3,
                 confirm_m: float = 0.02, tau_sec: float = 0.05) -> None:
        self.configure(start_speed_mps, stop_speed_mps, stop_sec, confirm_m, tau_sec)
        self.reset()

    def configure(self, start_speed_mps: float, stop_speed_mps: float, stop_sec: float, confirm_m: float,
                  tau_sec: float = 0.05) -> None:
        if not 0 < stop_speed_mps <= start_speed_mps:
            raise ValueError("need 0 < stop_speed_mps <= start_speed_mps")
        self.start_speed, self.stop_speed = float(start_speed_mps), float(stop_speed_mps)
        self.stop_sec, self.confirm_m, self.tau = float(stop_sec), float(confirm_m), float(tau_sec)

    def reset(self) -> None:
        self._t: Optional[float] = None
        self._p: Optional[np.ndarray] = None
        self._v = 0.0
        self._rest: Optional[np.ndarray] = None    # where the arm last rested
        self._moving = False                       # past onset
        self._confirmed = False
        self._onset = 0.0
        self._slow_since: Optional[float] = None   # when the speed last dropped below stop_speed
        self._path = self._peak = 0.0
        self._start: Optional[np.ndarray] = None

    @property
    def moving(self) -> bool:
        return self._confirmed

    @property
    def active(self) -> bool:
        """Past a movement's onset (confirmed or not), until it ends."""
        return self._moving

    def update(self, t: float, p: Sequence[float]) -> Tuple[Optional[str], Optional[Movement]]:
        """("start", None) when a movement is confirmed, ("end", Movement) when a confirmed one ends,
        else (None, None)."""
        p = np.asarray(p, dtype=float)
        if self._t is None or t <= self._t:
            self._t, self._p = t, p
            self._rest = p.copy() if self._rest is None else self._rest
            return None, None
        dt, step = t - self._t, float(np.linalg.norm(p - self._p))
        self._v += min(1.0, dt / (self.tau + dt)) * (step / dt - self._v)
        self._t, self._p = t, p
        if not self._moving:
            if self._v > self.start_speed:
                self._moving, self._confirmed = True, False
                self._onset, self._start = t, self._rest.copy()
                self._path, self._peak = float(np.linalg.norm(p - self._start)), self._v
                self._slow_since = None
            else:
                self._rest = p.copy()
                return None, None
        else:
            self._path += step
        self._peak = max(self._peak, self._v)
        kind = None
        if not self._confirmed and float(np.linalg.norm(p - self._start)) >= self.confirm_m:
            self._confirmed, kind = True, "start"
        if self._v < self.stop_speed:
            if self._slow_since is None:
                self._slow_since = t
            elif t - self._slow_since >= self.stop_sec:
                done = self._confirmed
                m = Movement(self._onset, t, float(np.linalg.norm(p - self._start)), self._path, self._peak)
                self._moving = self._confirmed = False
                self._rest = p.copy()
                return ("end", m) if done else (kind, None)
        else:
            self._slow_since = None
        return kind, None


class LongMoveRule:
    """Whether a finished movement earns the reward (module doc)."""

    def __init__(self, min_m: float = 0.10, min_straightness: float = 0.0, min_gap_sec: float = 1.0) -> None:
        self.configure(min_m, min_straightness, min_gap_sec)
        self.reset()

    def configure(self, min_m: float, min_straightness: float, min_gap_sec: float) -> None:
        self.min_m, self.min_straightness, self.min_gap = float(min_m), float(min_straightness), float(min_gap_sec)

    def reset(self) -> None:
        self._last_reward: Optional[float] = None

    def check_gap(self, m: Movement) -> Tuple[bool, str]:
        """The gap alone (the Reach Map's New-best reward): (ok, why). A reward is counted when granted."""
        if self._last_reward is not None and m.t_end - self._last_reward < self.min_gap:
            return False, f"within {self.min_gap:g} s of the last reward"
        self._last_reward = m.t_end
        return True, ""

    def check(self, m: Movement) -> Tuple[bool, str]:
        """(rewarded, why). A reward is counted toward the gap as soon as it is granted."""
        if m.length_m < self.min_m:
            return False, f"too short ({m.length_m * 100:.1f} < {self.min_m * 100:.1f} cm)"
        if self.min_straightness > 0 and m.straightness < self.min_straightness:
            return False, f"not straight enough ({m.straightness:.2f} < {self.min_straightness:.2f})"
        if self._last_reward is not None and m.t_end - self._last_reward < self.min_gap:
            return False, f"within {self.min_gap:g} s of the last reward"
        self._last_reward = m.t_end
        return True, f"{m.length_m * 100:.1f} cm, straightness {m.straightness:.2f}"
