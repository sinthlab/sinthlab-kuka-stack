"""The Reach Map view: a Reach Map run's recordings -> the monkey's reach per direction, and the four reach
values for his profile (helpers/reach_map.py does the work; this reads the files and packs it for the page).

Read while the run is going too: the recorder writes each trial's CSV as it goes.
"""
from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Tuple

import experiments as ex
from sinthlab_bringup.helpers import reach_map as rm
from sinthlab_bringup.helpers.free_move_box import box_bounds
from sinthlab_bringup.helpers.long_moves import MovementSegmenter
from sinthlab_bringup.helpers.rail_shapes import direction_name

# How much higher the horizontal tasks start than the Reach Map (the vertical tasks' start): 5 cm by their
# YAMLs (EE ~1.03 vs ~0.98 m). With the arm's model (kinematics.py) it is worked out from the two poses.
HORIZONTAL_DZ_M = 0.05
MAX_POINTS = 2500                # the page's scatter of where the hand went

_cache: Dict[str, Tuple[tuple, dict]] = {}


def horizontal_dz(params: dict) -> Tuple[float, str]:
    """(dz [m], how it was found): the horizontal rail's height above the Reach Map's start."""
    try:
        import kinematics
        import numpy as np
        from params import ExperimentParams
        ok, _ = kinematics.available()
        if ok:
            qh = ExperimentParams(ex.BY_KEY["move_horizontal"]).defaults[ex.START_POSE]
            th, tm = kinematics.ee_pose(np.radians(qh)), kinematics.ee_pose(np.radians(params[ex.START_POSE]))
            if th is not None and tm is not None:
                return float(th[2, 3] - tm[2, 3]), "from the two start poses"
    except Exception:
        pass
    return HORIZONTAL_DZ_M, "the YAMLs' 5 cm (no arm model here)"


def _segmenter(params: dict) -> MovementSegmenter:
    g = lambda k, d: float(params.get(f"long_move.{k}", d))
    return MovementSegmenter(g("start_speed_mps", 0.03), g("stop_speed_mps", 0.02), g("stop_sec", 0.3), g("confirm_m", 0.02))


def compute(folder: Optional[str], params: dict, profile: Optional[dict]) -> dict:
    """The page's data for one run folder (None: no Reach Map run yet)."""
    n = int(params.get("reach_map.sectors", 12))
    pct = float(params.get("reach_map.percentile", 80.0))
    try:
        lo, hi = box_bounds(params.__getitem__)
        box = (float(lo[1]), float(hi[1]), float(lo[2]), float(hi[2]))
    except (KeyError, ValueError):
        box = None
    dz, dz_how = horizontal_dz(params)
    current = {k: profile["values"][k] for k in rm_keys()} if profile else None
    out = {"run": Path(folder).name if folder else None, "sectors": n, "percentile": pct, "box": box,
           "horizontal_dz": round(dz, 4), "horizontal_dz_how": dz_how, "profile_reach": current,
           "points": [], "reaches": [], "rows": _named(rm.estimate([], n, pct, box)), "proposal": rm.proposal(rm.estimate([], n, pct, box), dz),
           "n_movements": 0, "minutes": 0.0}
    if not folder:
        return out
    files = sorted(Path(folder).glob("robot_trajectory_*.csv"))
    key = tuple((f.name, f.stat().st_size) for f in files) + (n, pct, box, dz)
    hit = _cache.get(folder)
    if hit and hit[0] == key:
        data = hit[1]
    else:
        seg = _segmenter(params)
        pts: List[Tuple[float, float]] = []
        reaches: List[Tuple[float, float]] = []
        secs = 0.0
        for f in files:
            t, p = rm.load_trial(f)
            if len(t) < 2:
                continue
            secs += t[-1] - t[0]
            reaches += rm.reach_points(t, p, seg)
            pts += [(q[1], q[2]) for q in p]
        step = max(1, len(pts) // MAX_POINTS)
        data = {"points": [(round(a, 4), round(b, 4)) for a, b in pts[::step]],
                "reaches": [(round(a, 4), round(b, 4)) for a, b in reaches], "minutes": round(secs / 60.0, 1)}
        _cache[folder] = (key, data)
    rows = _named(rm.estimate(data["reaches"], n, pct, box))
    out.update(data, rows=rows, proposal=rm.proposal(rows, dz), n_movements=len(data["reaches"]))
    return out


def _named(rows: List[dict]) -> List[dict]:
    for r in rows:
        r["name"] = direction_name(r["deg"])
    return rows


def rm_keys() -> List[str]:
    return ["reach.left_m", "reach.right_m", "reach.down_m", "reach.up_m"]


def record(data: dict) -> dict:
    """What the profile keeps of the map it was set from (profiles.save, reach_map)."""
    return {"run": data.get("run"), "percentile": data.get("percentile"), "sectors": data.get("sectors"),
            "n_movements": data.get("n_movements"), "horizontal_dz": data.get("horizontal_dz"),
            "comfortable_m": {f"{r['deg']:.0f}": r["comfortable_m"] for r in data.get("rows", [])},
            "farthest_m": {f"{r['deg']:.0f}": r["farthest_m"] for r in data.get("rows", [])},
            "arm_limited": [f"{r['deg']:.0f}" for r in data.get("rows", []) if r["arm_limited"]]}
