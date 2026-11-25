#!/usr/bin/env python3
"""The Reward calibration page: weigh what the juice valve gives and fit mL per reward, from the dashboard.

It runs analysis/reward_calibration.py's own steps (the plan, the dispensing, the CSV, the fit), so a sweep
started here can be resumed with the script and the other way round, and its files are the ones
reward_volume.py reads:

    analysis/reward_calibrations/<date>_<setup>/single.csv  (+ .plan.json)   one sweep per reward size
                                               single.json                  its fit: the profile
                                               burst2.*, burst3.* ...       rewards of 2, 3 ... pulses
                                               checks.csv                   the pre-session checks

The dashboard uses the NEWEST set that has a profile (reward_volume.calibration). Fitting the first profile
into a new set therefore switches the dashboard to it, along with the reward sizes it covers.

One job at a time, in a background thread, so the pulses keep their timing whatever the browser does:
    prime   N openings to fill the line, nothing recorded
    check   N rewards at one width; you weigh them; the volume against the profile in use (checks.csv)
    sweep   the next batch of a sweep; you weigh it; one row in the sweep's CSV
A job dispenses, waits for the last drop (DRAIN_SEC), then (check, sweep) waits for the net mass. Stop sets
every output low between pulses; the batch is then marked aborted and redone later, never repeated blindly.

The page polls GET /api/calib/state. Dispensing is refused while an experiment runs (its rewards would
land in the cup, and these pulses in its NSP recording), and the dashboard refuses Start while a job runs.
"""
from __future__ import annotations

import datetime as dt
import hashlib
import json
import re
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional

import experiments as ex
import reward_volume
import ttl

sys.path.insert(0, str(ex.ANALYSIS))
import reward_calibration as rc  # noqa: E402

DRAIN_SEC = rc.DRAIN_SEC
CHECKS_CSV = "checks.csv"
SET_NAME = re.compile(r"^\d{4}-\d{2}-\d{2}_[a-z0-9][a-z0-9_-]{0,39}$")
SETUP = re.compile(r"^[a-z0-9][a-z0-9_-]{0,39}$")
MAX_UNITS = 200                     # rewards per batch, check or prime
TOLERANCE_PCT = 5.0                 # a check further than this from the profile is flagged
PRIME = {"width_ms": 200, "count": 10}


class Stopped(Exception):
    pass


def profile_name(burst: int) -> str:
    """The file stem for a reward size: single, burst2, burst3 ... (the 2026-11-23 set's names)."""
    return "single" if int(burst) == 1 else f"burst{int(burst)}"


def spacing_for(width_ms: int, burst: int, gap_ms: int) -> float:
    """Seconds from one reward to the next: at least 1 s, and the valve closed MIN_OFF_SEC after each."""
    need = rc.burst_sec(width_ms, burst, gap_ms) + rc.MIN_OFF_SEC
    return max(1.0, round(need * 2 + 0.4999) / 2)       # whole half seconds


def _num(v, name: str, kind=float, lo=None, hi=None):
    try:
        x = kind(v)
    except (TypeError, ValueError):
        raise ValueError(f"{name}: a number, got {v!r}") from None
    if (lo is not None and x < lo) or (hi is not None and x > hi):
        raise ValueError(f"{name}: {lo}-{hi}, got {x:g}")
    return x


class Calibrator:
    def __init__(self, demo: bool, busy: Callable[[], Optional[str]], emit: Callable[[str, str], None],
                 folder: Optional[Path] = None, sleep_scale: float = 1.0) -> None:
        self.demo = demo
        self.folder = Path(folder) if folder else reward_volume.FOLDER
        self._busy, self._emit = busy, emit
        self._fake = ttl.FakeCabinet()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._scale = sleep_scale            # tests run the waits faster
        self.job: Optional[dict] = None
        self.last: Optional[dict] = None     # the last finished job's outcome, for the page

    # ------------------------------------------------------------------ what the page shows
    def running(self) -> Optional[str]:
        """What the valve is doing now (dispensing or waiting for the last drop), else None."""
        j = self.job
        return j["label"] if j and j["phase"] in ("dispensing", "draining") else None

    def state(self) -> dict:
        active = reward_volume.calibration(self.folder)
        return {"active": active["set"] if active else None, "sets": self.sets(), "job": self.job,
                "last": self.last, "busy": self._busy(), "demo": self.demo,
                "defaults": {"cabinet": rc.DEFAULT_CABINET, "output": rc.DEFAULT_OUTPUT, "prime": PRIME,
                             "drain_s": DRAIN_SEC, "tolerance_pct": TOLERANCE_PCT, "today": dt.date.today().isoformat()}}

    def sets(self) -> List[dict]:
        """Every calibration set, newest first: its profiles, sweeps and checks."""
        if not self.folder.is_dir():
            return []
        out = []
        for d in sorted((p for p in self.folder.iterdir() if p.is_dir()), reverse=True):
            profiles, sweeps = [], []
            for f in sorted(d.glob("*.json")):
                if f.name.endswith(".plan.json"):
                    continue
                try:
                    p = json.loads(f.read_text())
                except (OSError, ValueError):
                    continue
                if "points" in p and "valid_width_ms" in p:
                    profiles.append(_profile_summary(f.stem, p))
            for f in sorted(d.glob("*.plan.json")):
                sweeps.append(self._sweep_summary(d, f.name[:-len(".plan.json")]))
            checks = rc.read_rows(d / CHECKS_CSV)[-8:]
            out.append({"name": d.name, "profiles": sorted(profiles, key=lambda p: p["burst"]),
                        "sweeps": sorted(sweeps, key=lambda s: s["burst"]), "checks": list(reversed(checks))})
        return out

    def _sweep_summary(self, d: Path, name: str) -> dict:
        saved = json.loads((d / f"{name}.plan.json").read_text())
        s, plan = saved["settings"], saved["plan"]
        rows = rc.read_rows(d / f"{name}.csv")
        ok = {int(r["batch"]) for r in rows if r.get("status") == "ok"}
        pending = [b for b in plan if b["batch"] not in ok]
        done = [{"batch": int(r["batch"]), "width_ms": int(float(r["width_ms"])), "status": r["status"],
                 "net_mass_g": r.get("net_mass_g", ""), "units": int(r.get("pulses_confirmed") or 0)}
                for r in rows if r.get("batch", "").isdigit()]
        return {"name": name, "burst": int(s.get("burst", 1)), "gap_ms": int(s.get("burst_gap_ms", 300)),
                "settings": s, "total": len(plan), "n_ok": len(ok), "next": pending[0] if pending else None,
                "rows": done, "fitted": (d / f"{name}.json").exists()}

    # ------------------------------------------------------------------ the cabinet
    def _caller(self, addr: str):
        if self.demo:
            return self._fake.call
        ttl.check(addr, "STATUS")
        return lambda cmd: ttl.call(addr, cmd)

    def cabinet(self, addr: str) -> dict:
        ok, text, ms = self._caller(addr)("STATUS")
        return {"ok": ok and text.startswith("OK"), "text": text, "ms": round(ms, 1)}

    # ------------------------------------------------------------------ jobs
    def _claim(self, job: dict) -> None:
        busy = self._busy()
        if busy:
            raise RuntimeError(f"{busy} is running: its rewards would land in the cup, and these pulses in its "
                               f"NSP recording. Stop it first.")
        with self._lock:
            if self.job:
                raise RuntimeError("Finish the current job first" + (" (weigh it, or discard it)"
                                                                     if self.job["phase"] == "weigh" else ""))
            self._stop.clear()
            self.last = None
            self.job = {"phase": "dispensing", "done": 0, "started": dt.datetime.now().isoformat(timespec="seconds"),
                        **job}

    def _sleep(self, s: float) -> None:
        if self._stop.wait(max(0.0, s) * self._scale):
            raise Stopped()

    def _run(self, call, output: int) -> None:
        j = self.job
        t0 = time.monotonic()

        def progress(n):
            j["done"] = n
        try:
            got, err = rc.dispense(call, output, j["width_ms"], j["total"], j["spacing_s"], sleep=self._sleep,
                                   progress=progress, burst=j["burst"], gap_ms=j["gap_ms"])
        except Stopped:
            call("ALLOFF")
            got, err = -1, "stopped; every output set low"
        j["done"] = max(got, j["done"])
        if err:
            self._finish_failed(j, err, got)
            return
        if j["kind"] == "prime":
            self._end({"ok": True, "text": f"Primed: {got} openings of {j['width_ms']} ms."})
            return
        j["phase"] = "draining"
        try:
            self._sleep(DRAIN_SEC)
        except Stopped:
            pass                                  # the pulses all went: weigh what is in the cup anyway
        j["duration_s"] = round(time.monotonic() - t0, 1)
        j["phase"] = "weigh"

    def _finish_failed(self, j: dict, err: str, got: int) -> None:
        if j["kind"] == "sweep":
            rc.append_row(self._csv(j), {**self._row(j), "pulses_confirmed": max(got, 0), "status": "aborted",
                                         "note": err})
        self._end({"ok": False, "text": f"{j['label']}: {err}. Empty the cup"
                   + (" ; the batch is marked aborted and comes round again." if j["kind"] == "sweep" else ".")})

    def _end(self, last: dict) -> None:
        with self._lock:
            self.job, self.last = None, last
        self._emit(f"Reward calibration: {last['text']}", "gui" if last["ok"] else "warn")

    def stop(self) -> dict:
        self._stop.set()
        return {"ok": True}

    def prime(self, addr: str, output: int, width_ms: int, count: int) -> dict:
        width_ms = _num(width_ms, "width [ms]", int, *ttl.WIDTH_MS)
        count = _num(count, "openings", int, 1, MAX_UNITS)
        output = _num(output, "output", int, 1, ttl.CHANNELS)
        call = self._caller(addr)
        self._claim({"kind": "prime", "label": f"Prime {count} × {width_ms} ms", "width_ms": width_ms,
                     "total": count, "burst": 1, "gap_ms": 0, "spacing_s": spacing_for(width_ms, 1, 0)})
        threading.Thread(target=self._run, args=(call, output), daemon=True).start()
        return {"ok": True}

    def check(self, addr: str, output: int, burst: int, width_ms: int, rewards: int) -> dict:
        """N rewards at one width of the profile in use for that reward size, then weigh them."""
        output = _num(output, "output", int, 1, ttl.CHANNELS)
        burst = _num(burst, "pulses per reward", int, 1, 10)
        width_ms = _num(width_ms, "width [ms]", int, *ttl.WIDTH_MS)
        rewards = _num(rewards, "rewards", int, 1, MAX_UNITS)
        active = reward_volume.calibration(self.folder)
        prof = self._profile(active["set"], profile_name(burst)) if active else None
        if prof is None:
            raise ValueError(f"no calibration in use for rewards of {burst} pulse(s): calibrate that size first")
        gap = int(prof.get("burst_gap_ms") or 0) if burst > 1 else 0
        spacing = spacing_for(width_ms, burst, gap or 300)
        rc.check_widths([width_ms], spacing, burst, gap or 300)
        want_ul = rc.volume_for(prof, width_ms)
        call = self._caller(addr)
        self._claim({"kind": "check", "label": f"Check {_reward_text(burst, width_ms, gap)} × {rewards}",
                     "set": active["set"], "name": profile_name(burst), "width_ms": width_ms, "total": rewards,
                     "burst": burst, "gap_ms": gap or 300, "spacing_s": spacing, "output": output,
                     "cabinet": "demo" if self.demo else addr, "want_ul": round(want_ul, 1),
                     "expected_g": round(want_ul * rewards * prof["density_g_per_ml"] / 1000.0, 2),
                     "profile_id": prof["profile_id"]})
        threading.Thread(target=self._run, args=(call, output), daemon=True).start()
        return {"ok": True}

    # ------------------------------------------------------------------ sweeps
    def new_sweep(self, set_name: str, setup: str, burst: int, gap_ms: int, widths, reps: int, blanks: int,
                  rewards: int, spacing: Optional[float] = None, seed: int = 1) -> dict:
        """Write a sweep's plan (no pulses): into an existing set, or a new set <today>_<setup>."""
        burst = _num(burst, "pulses per reward", int, 1, 10)
        gap_ms = _num(gap_ms, "gap [ms]", int, rc.MIN_GAP_MS, 5000) if burst > 1 else 300
        widths = sorted({_num(w, "width [ms]", int, *ttl.WIDTH_MS) for w in (widths or [])})
        if len(widths) < 2:
            raise ValueError("at least 2 widths, to fit a line")
        reps = _num(reps, "batches per width", int, 1, 10)
        blanks = _num(blanks, "blank batches", int, 0, 10)
        rewards = _num(rewards, "rewards per batch", int, 1, MAX_UNITS)
        spacing = (_num(spacing, "spacing [s]", float, 0.3, 30) if spacing not in (None, "")
                   else spacing_for(widths[-1], burst, gap_ms))
        rc.check_widths(widths, spacing, burst, gap_ms)
        if not set_name:
            setup = (setup or "").strip().lower().replace(" ", "_")
            if not SETUP.match(setup):
                raise ValueError("Name the setup (letters, digits, - and _), e.g. pressure_reservoir")
            set_name = f"{dt.date.today().isoformat()}_{setup}"
        if not SET_NAME.match(set_name):
            raise ValueError(f"set {set_name!r}: <date>_<setup>")
        d = self.folder / set_name
        name = profile_name(burst)
        plan_path = d / f"{name}.plan.json"
        if plan_path.exists():
            raise ValueError(f"{set_name} already has a {name} sweep: carry on with it, or start a new set")
        want = {"widths": widths, "reps": reps, "blanks": blanks, "pulses": rewards, "spacing": spacing,
                "seed": seed, "output": rc.DEFAULT_OUTPUT, "burst": burst, "burst_gap_ms": gap_ms}
        plan = rc.make_plan(widths, reps, blanks, seed)
        d.mkdir(parents=True, exist_ok=True)
        plan_path.write_text(json.dumps({"settings": want, "plan": plan}, indent=1))
        self._emit(f"Reward calibration: new sweep {set_name}/{name} ({len(plan)} batches)", "gui")
        return {"ok": True, "set": set_name, "name": name}

    def _plan(self, set_name: str, name: str) -> dict:
        if not SET_NAME.match(set_name or "") or not re.match(r"^[a-z0-9][a-z0-9_-]{0,39}$", name or ""):
            raise ValueError("unknown sweep")
        p = self.folder / set_name / f"{name}.plan.json"
        if not p.exists():
            raise ValueError(f"no sweep {set_name}/{name}")
        return json.loads(p.read_text())

    def _csv(self, j: dict) -> Path:
        d = self.folder / j["set"]
        return d / (CHECKS_CSV if j["kind"] == "check" else f"{j['name']}.csv")

    def batch(self, addr: str, output: int, set_name: str, name: str) -> dict:
        """Dispense the sweep's next batch not yet weighed."""
        saved = self._plan(set_name, name)
        s = saved["settings"]
        output = _num(output, "output", int, 1, ttl.CHANNELS)
        ok = {int(r["batch"]) for r in rc.read_rows(self.folder / set_name / f"{name}.csv") if r.get("status") == "ok"}
        todo = [b for b in saved["plan"] if b["batch"] not in ok]
        if not todo:
            raise ValueError("every batch is weighed: fit it")
        b = todo[0]
        burst, gap = int(s.get("burst", 1)), int(s.get("burst_gap_ms", 300))
        w = int(b["width_ms"])
        call = self._caller(addr)
        what = "blank (no pulse)" if w == 0 else f"{_reward_text(burst, w, gap if burst > 1 else 0)} × {s['pulses']}"
        self._claim({"kind": "sweep", "label": f"Batch {b['batch']}: {what}", "set": set_name, "name": name,
                     "batch": b["batch"], "width_ms": w, "total": int(s["pulses"]), "burst": burst, "gap_ms": gap,
                     "spacing_s": float(s["spacing"]), "output": output, "cabinet": "demo" if self.demo else addr,
                     "left": len(todo), "of": len(saved["plan"])})
        threading.Thread(target=self._run, args=(call, output), daemon=True).start()
        return {"ok": True}

    def _row(self, j: dict) -> dict:
        return {"batch": j.get("batch", "check"), "width_ms": j["width_ms"],
                "pulses_requested": 0 if j["width_ms"] == 0 else j["total"], "pulses_confirmed": j["done"],
                "spacing_s": j["spacing_s"], "started": j["started"], "duration_s": j.get("duration_s", ""),
                "cabinet": j["cabinet"], "output": j["output"], "burst": j["burst"],
                "burst_gap_ms": j["gap_ms"] if j["burst"] > 1 else ""}

    def weigh(self, mass_g=None, discard: bool = False) -> dict:
        """The net mass of what the job gave, or discard it (the batch comes round again)."""
        with self._lock:
            j = self.job
            if not j or j["phase"] != "weigh":
                raise RuntimeError("Nothing is waiting to be weighed")
        if discard:
            if j["kind"] == "sweep":
                rc.append_row(self._csv(j), {**self._row(j), "status": "discarded"})
            self._end({"ok": True, "text": f"{j['label']}: discarded" + (", it comes round again." if j["kind"] == "sweep" else ".")})
            return {"ok": True}
        m = _num(mass_g, "net mass [g]", float, 0, 1000)
        if j["kind"] == "sweep":
            rc.append_row(self._csv(j), {**self._row(j), "net_mass_g": m, "status": "ok"})
            per = f": {m / j['done']:.2f} mL per reward at 1 g/mL" if j["width_ms"] and j["done"] else ""
            self._end({"ok": True, "text": f"{j['label']}: {m:g} g{per}."})
            return {"ok": True}
        # A check: against the profile in use.
        prof = self._profile(j["set"], j["name"])
        each = 1000.0 * m / (prof["density_g_per_ml"] * j["done"])
        off = 100.0 * (each - j["want_ul"]) / j["want_ul"]
        good = abs(off) <= TOLERANCE_PCT
        rc.append_row(self._csv(j), {**self._row(j), "net_mass_g": m, "status": "ok",
                                     "note": f"check vs {j['profile_id']}: {off:+.1f}%"})
        self._end({"ok": good, "check": True, "off_pct": round(off, 1),
                   "text": f"{j['label']}: {each / 1000:.2f} mL per reward, {off:+.1f}% from the calibration"
                           + ("." if good else f": outside ±{TOLERANCE_PCT:g}%. Check the reservoir fill, the line for "
                                               f"air and the spout; recalibrate if it persists.")})
        return {"ok": True}

    # ------------------------------------------------------------------ the fit
    def _profile(self, set_name: str, name: str) -> Optional[dict]:
        f = self.folder / set_name / f"{name}.json"
        return json.loads(f.read_text()) if f.exists() else None

    def fit(self, set_name: str, name: str, model: str = "interp", density=1.0, max_cv=10.0) -> dict:
        self._plan(set_name, name)                     # a known sweep
        density = _num(density, "density [g/mL]", float, 0.5, 2.0)
        max_cv = _num(max_cv, "max CV [%]", float, 1, 50)
        before = reward_volume.calibration(self.folder)
        path = self.folder / set_name / f"{name}.csv"
        rows = rc.read_rows(path)
        sources = [{"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}] if rows else []
        p = rc.fit_profile(rows, density, model, max_cv, sources, "fitted on the dashboard's Reward calibration page")
        (self.folder / set_name / f"{name}.json").write_text(json.dumps(p, indent=1))
        after = reward_volume.calibration(self.folder)
        notes = []
        if after and after["set"] == set_name:
            if before and before["set"] != set_name:
                lost = sorted({q["burst"] for q in before["profiles"]} - {q["burst"] for q in after["profiles"]})
                notes.append(f"The dashboard now uses {set_name}" + (
                    f"; it has no profile for rewards of {' or '.join(map(str, lost))} pulses yet, so those show no mL "
                    f"until you calibrate them here." if lost else "."))
        elif after:
            notes.append(f"The dashboard keeps using {after['set']}, the newest set.")
        self._emit(f"Reward calibration: fitted {set_name}/{name} ({p['profile_id']})", "gui")
        return {"ok": True, "profile": _profile_summary(name, p), "notes": notes}


def _reward_text(burst: int, width_ms: int, gap_ms: int) -> str:
    return f"{width_ms} ms" if burst == 1 else f"{burst} × {width_ms} ms, {gap_ms} ms apart"


def _clean(v):
    """NaN (a width with one batch has no SD) is not JSON: None instead."""
    if isinstance(v, float) and v != v:
        return None
    if isinstance(v, dict):
        return {k: _clean(x) for k, x in v.items()}
    if isinstance(v, list):
        return [_clean(x) for x in v]
    return v


def _profile_summary(name: str, p: dict) -> dict:
    return _clean({"name": name, "burst": int(p.get("burst", 1)), "gap_ms": int(p.get("burst_gap_ms") or 0),
            "model": p.get("model"), "created": p.get("created", ""), "profile_id": p.get("profile_id", ""),
            "points": p["points"], "valid_width_ms": p["valid_width_ms"],
            "per_width": [{k: t.get(k) for k in ("width_ms", "n", "mean_ul", "sd_ul", "cv_pct")}
                          for t in p.get("per_width", [])],
            "excluded_widths_ms": p.get("excluded_widths_ms", []), "r2": (p.get("fit") or {}).get("r2"),
            "slope_ul_per_ms": p.get("slope_ul_per_ms"), "intercept_ul": p.get("intercept_ul"),
            "density": p.get("density_g_per_ml", 1.0), "blanks_g": p.get("blanks_g", [])})
