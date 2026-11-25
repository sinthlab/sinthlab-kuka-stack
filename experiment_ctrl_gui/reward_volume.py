"""Reward volume: the valve calibration (analysis/reward_calibrations/<newest>/*.json, written by
analysis/reward_calibration.py fit) -> mL per reward for a reward setting, and back.

A reward is reward_pulse.count pulses of reward_pulse.pulse_ms, gap_ms apart. A calibration profile holds
one count (its `burst`), with the volume PER REWARD at each calibrated width. Within the calibrated widths
the volume is interpolated between them; outside, it follows the profile's straight-line fit and is marked
extrapolated. A count with no profile has no estimate. The dashboard does the same sums in the page
(static/app.js, rewardVolume), from calibration() as served by GET /api/reward/calibration.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

import numpy as np

import experiments as ex

FOLDER = ex.ANALYSIS / "reward_calibrations"


def calibration(folder: Optional[Path] = None) -> Optional[dict]:
    """The newest calibration set (folders are named <date>_<setup>, so the newest sorts last), or None."""
    root = Path(folder) if folder else FOLDER
    sets = sorted(p for p in root.glob("*") if p.is_dir() and any(p.glob("*.json"))) if root.is_dir() else []
    if not sets:
        return None
    d = sets[-1]
    profiles = []
    for f in sorted(d.glob("*.json")):
        if f.name.endswith(".plan.json"):
            continue
        p = json.loads(f.read_text())
        if "points" not in p or "valid_width_ms" not in p:
            continue
        profiles.append({"file": f.name, "burst": int(p.get("burst", 1)), "gap_ms": int(p.get("burst_gap_ms") or 0),
                         "points": [[float(w), float(v)] for w, v in p["points"]],
                         "slope_ul_per_ms": float(p["slope_ul_per_ms"]), "intercept_ul": float(p["intercept_ul"]),
                         "valid_width_ms": p["valid_width_ms"], "created": p.get("created", ""),
                         "max_cv_pct": max((r.get("cv_pct", 0.0) for r in p.get("per_width", [])), default=None)})
    return {"set": d.name, "profiles": profiles} if profiles else None


def volume_ml(cal: Optional[dict], pulse_ms: float, count: int = 1, gap_ms: float = 0) -> dict:
    """{ml, status: calibrated | extrapolated | none, note} for one reward."""
    prof = next((p for p in (cal or {}).get("profiles", []) if p["burst"] == int(count)), None)
    if prof is None:
        return {"ml": None, "status": "none",
                "note": f"no calibration for {int(count)} pulse(s)" if cal else "no reward calibration"}
    w, v = zip(*prof["points"])
    lo, hi = prof["valid_width_ms"]
    if lo <= pulse_ms <= hi:
        ul, status, note = float(np.interp(pulse_ms, w, v)), "calibrated", ""
    else:
        ul = prof["slope_ul_per_ms"] * pulse_ms + prof["intercept_ul"]
        status, note = "extrapolated", f"calibrated {lo}-{hi} ms"
    if int(count) > 1 and prof["gap_ms"] and int(gap_ms) != prof["gap_ms"]:
        note = "; ".join(x for x in (note, f"calibrated with a {prof['gap_ms']} ms gap") if x)
    return {"ml": round(max(ul, 0.0) / 1000.0, 3), "status": status, "note": note}


def width_for_ml(cal: Optional[dict], ml: float, count: int = 1) -> dict:
    """{ms, ml (what that width gives), status, note}: the valve time for a volume per reward."""
    prof = next((p for p in (cal or {}).get("profiles", []) if p["burst"] == int(count)), None)
    if prof is None:
        raise ValueError(f"no reward calibration for {int(count)} pulse(s)")
    ul = float(ml) * 1000.0
    w, v = zip(*prof["points"])
    if v[0] <= ul <= v[-1]:
        ms = float(np.interp(ul, v, w))
    else:
        ms = (ul - prof["intercept_ul"]) / prof["slope_ul_per_ms"]
    ms = int(round(ms))
    if ms < 10:
        raise ValueError(f"{ml:g} mL is less than {int(count)} pulse(s) give at the shortest opening")
    got = volume_ml(cal, ms, count, prof["gap_ms"])
    return {"ms": ms, **got}
