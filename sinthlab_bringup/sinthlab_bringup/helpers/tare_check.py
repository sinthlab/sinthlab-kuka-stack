"""Free Move's tare, checked against the run's usual one: a hand resting on the apple during the quiet window
must not become the "zero".

The admittance subtracts a tare, the force the arm reads at rest at the start pose (its own model error, a
few N), measured each trial while the arm holds there before the go cue (actions/admittance_move.py). If the
monkey's hand already rests on the apple then, the tare learns the hand too. When he lets go, the arm reads
his missing hand as a push the other way and creeps off by itself: the "moves on its own" seen with the
monkey on the horizontal rail (25 Nov), which human testers, who keep their hands off until the go, never
caused. tare_max_n (10 N) only catches a hand pressing hard.

The start pose is the same every trial, so the arm's own reading there barely changes from trial to trial.
So each trial's tare is compared with the median of the run's last HISTORY tares (this one included; a
median, so a few trials with a hand on it do not move it):

    within tare_tol_n of it   this trial's own tare is used (it follows slow drift)
    farther                   the median is used, and the trial is flagged (a hand on the apple, most likely)

The first MIN_HISTORY - 1 trials of a run have nothing to compare with and use their own. tare_tol_n 0
turns the check off. Pure (no ROS): tested in test/test_tare_check.py.
"""
from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np

HISTORY = 9         # the median of this many most recent tares
MIN_HISTORY = 3     # ...once there are at least this many


def choose(raw: Optional[np.ndarray], history: List[np.ndarray], tol_n: float, tare_max_n: float,
           previous: np.ndarray) -> Tuple[np.ndarray, dict]:
    """The tare to use this trial, and its record for the trial's sidecar. `raw` is this trial's measured
    resting force (None without samples); `history` (updated in place) holds the run's recent tares;
    `previous` is the tare in use until now."""
    previous = np.asarray(previous, dtype=float)
    rec = {"raw_n": None, "used_n": [round(float(v), 2) for v in previous], "usual_n": None, "kept": "previous"}
    if raw is None:
        return previous, rec
    raw = np.asarray(raw, dtype=float)
    rec["raw_n"] = [round(float(v), 2) for v in raw]
    if float(np.linalg.norm(raw)) > tare_max_n:          # pressing hard: not even a sample of the arm's reading
        usual = np.median(np.array(history), axis=0) if history else previous
        rec.update(used_n=[round(float(v), 2) for v in usual], kept="usual" if history else "previous",
                   over_max=True)
        return usual, rec
    history.append(raw)
    del history[:-HISTORY]
    if tol_n <= 0.0 or len(history) < MIN_HISTORY:
        rec.update(used_n=rec["raw_n"], kept="own")
        return raw, rec
    usual = np.median(np.array(history), axis=0)
    rec["usual_n"] = [round(float(v), 2) for v in usual]
    off = float(np.linalg.norm(raw - usual))
    rec["off_n"] = round(off, 2)
    if off > tol_n:
        rec.update(used_n=rec["usual_n"], kept="usual")
        return usual, rec
    rec.update(used_n=rec["raw_n"], kept="own")
    return raw, rec
