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
turns the check off.

THE PRE-TASK CALIBRATION (tare_calibration.enabled, the default): a monkey early in training grabs early, so
the quiet window is a poor time to measure anything. Instead, at Start, before the monkey is rolled up,
the arm goes to the start pose and holds there untouched for tare_calibration.hold_sec; the mean reading
after settle_sec is the tare for the whole run (calibrate()). The reading must be steady: every
STEADY_CHUNK_SEC stretch within tare_calibration.steady_n of the mean, else someone touched it (or the arm
had not settled) and the run pauses at park to try again on Resume. Each trial
still measures its quiet-window reading, for the record only (against_calibration()): it does not change
the tare. The start pose must be the one the trials use: the error in the reading depends on posture, so
a tare taken at park would be wrong at the start.

Pure (no ROS): tested in test/test_tare_check.py.
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


STEADY_CHUNK_SEC = 0.25   # the calibration's reading, averaged over stretches this long, must stay within steady_n
MIN_SAMPLES = 20


def calibrate(samples: List[Tuple[float, np.ndarray]], settle_sec: float, tare_max_n: float,
              steady_n: float = 1.5) -> dict:
    """The pre-task calibration from (seconds since the hold began, force [N]) samples: the mean after
    settle_sec, if steady. Returns {ok, tare_n, spread_n, n, why}."""
    use = [(t, np.asarray(f, dtype=float)) for t, f in samples if t >= settle_sec]
    if len(use) < MIN_SAMPLES:
        return {"ok": False, "tare_n": None, "spread_n": None, "n": len(use),
                "why": f"only {len(use)} readings after {settle_sec:g} s (is the robot state arriving?)"}
    forces = np.array([f for _, f in use])
    mean = forces.mean(axis=0)
    t0 = use[0][0]
    chunks = {}
    for t, f in use:
        chunks.setdefault(int((t - t0) / STEADY_CHUNK_SEC), []).append(f)
    spread = max(float(np.linalg.norm(np.mean(c, axis=0) - mean)) for c in chunks.values())
    rec = {"ok": True, "tare_n": [round(float(v), 2) for v in mean], "spread_n": round(spread, 2),
           "n": len(use), "why": ""}
    if float(np.linalg.norm(mean)) > tare_max_n:
        rec.update(ok=False, why=f"the reading at rest, {float(np.linalg.norm(mean)):.1f} N, is over tare_max_n "
                                 f"({tare_max_n:g} N): was the apple being touched?")
    elif spread > steady_n:
        rec.update(ok=False, why=f"the reading moved by {spread:.1f} N while it should have been still: was the "
                                 "apple touched?")
    return rec


def against_calibration(raw: Optional[np.ndarray], calibrated: np.ndarray, tol_n: float) -> dict:
    """A trial's record when the run is calibrated: the tare is the calibration's; this trial's quiet-window
    reading is kept for the record, with how far it was from it (hand_likely: farther than tol_n)."""
    rec = {"kept": "calibrated", "used_n": [round(float(v), 2) for v in calibrated], "raw_n": None}
    if raw is not None:
        off = float(np.linalg.norm(np.asarray(raw, dtype=float) - calibrated))
        rec.update(raw_n=[round(float(v), 2) for v in raw], off_n=round(off, 2),
                   hand_likely=bool(tol_n > 0.0 and off > tol_n))
    return rec
