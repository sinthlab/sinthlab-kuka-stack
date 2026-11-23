#!/usr/bin/env python3
"""Reward calibration: how many microlitres the juice valve gives for how many milliseconds open.

The reward size in every experiment is reward_pulse.pulse_ms, the time the valve stays open (README,
"Reward pulse"). This turns it into a volume. Weigh what the valve gives over batches of pulses at a few
widths, fit volume against width, then ask for the width that gives a volume:

    # 1. Sweep: the cabinet opens the valve N times per batch, you weigh each batch.
    python3 analysis/reward_calibration.py sweep --widths 20 40 60 100 150 200 300 --reps 3 --pulses 50 \\
        --out reward_calib/2026-11-24.csv

    # 2. Fit: volume per pulse against width, written as a profile.
    python3 analysis/reward_calibration.py fit reward_calib/2026-11-24.csv --out reward_calib/profile.json

    # 3. Use: the width for a volume (and the volume a width gives).
    python3 analysis/reward_calibration.py ms --profile reward_calib/profile.json 150 250 400
    python3 analysis/reward_calibration.py ul --profile reward_calib/profile.json 120

    # Before a session: one batch at the width you will use, compared with the profile.
    python3 analysis/reward_calibration.py check --profile reward_calib/profile.json --width 120

    # A reward of several drops (reward_pulse.count > 1): calibrate the burst itself, not one pulse
    # times N -- each opening adds a fixed amount, and the line may not recover between drops.
    python3 analysis/reward_calibration.py sweep --burst 3 --burst-gap-ms 300 --widths 20 30 40 50 \
        --out reward_calib/burst3.csv

BURSTS (--burst N, --burst-gap-ms G): each unit of a batch is a burst of N pulses of the width, G ms of
closed valve between them, as an experiment delivers reward_pulse.count = N, reward_pulse.gap_ms = G.
The batch then holds --pulses bursts, --spacing apart (from the start of one burst to the next), and
everything is per burst: the fit is microlitres per reward against the width of each pulse, and the
profile records N and G. One sweep file holds one burst setting; compare settings with separate files.

THE PULSES go the way an experiment's reward does: "PULSE <output> <ms>" over UDP to the cabinet
(port 30002, sunrise_controller_code/TtlPulser.java), output 4 = SYNC 4 = the reward box's NSP jack.
The cabinet must be running PulseTestServer (it never moves the arm) or LbrImpedanceControlServer, and
no experiment may be running: its pulses would land in the cup, and this script's in its recording.
--demo uses a simulated cabinet, to try the procedure with no hardware.

BEFORE A SWEEP: the arm parked, the session's fluid in the reservoir at its usual fill, the line primed,
FLUSH off, the pedal and hand switch unplugged, a cup on a scale under the spout. Keep the reservoir,
tubing and spout exactly as in a session: the volume depends on all of them.

EACH BATCH: tare the cup on the scale (or empty it and tare), press Enter, the valve opens --pulses
times, --spacing apart; after the last drip, type the net mass in grams. Batches run in shuffled order,
plus --blanks batches with no pulse (drips, evaporation, scale drift). Every batch is appended to the CSV
at once, so a sweep can stop and resume: run the same command again and it continues where it stopped
(the batch order is kept in <out>.plan.json). A pulse the cabinet does not confirm stops the batch, which
is marked aborted and redone later -- a pulse whose outcome is unknown is never repeated blindly.

THE FIT: volume per pulse = 1000 * net mass / (density * pulses), one point per batch. A straight line
with a free intercept (valves give little or nothing below a minimum opening, so the intercept is
usually negative), or with --model interp, straight segments between the per-width means. Widths whose
batches vary by more than --max-cv are reported and left out of the valid range. The profile records the
valid width and volume range; asking for a volume outside it is refused.

Repeat the sweep after changing the valve, tubing, spout, fluid or reservoir, and at least at low, middle
and high reservoir fill once, to see whether the fill matters.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import random
import sys
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "experiment_ctrl_gui"))
import ttl  # noqa: E402  (the dashboard's pulse relay: the same command checks)

DEFAULT_CABINET = "172.31.1.147"      # nsp_sync.cabinet_ip in every experiment YAML
DEFAULT_OUTPUT = 4                    # reward_pulse.output: SYNC 4 -> the reward box's NSP jack
MIN_OFF_SEC = 0.3                     # the valve closed at least this long between pulses
DRAIN_SEC = 3.0                       # default wait after the last pulse for the last drop (--drain)
# pulses_requested / _confirmed count UNITS: single pulses, or with --burst, whole bursts of `burst` pulses.
FIELDS = ["batch", "width_ms", "pulses_requested", "pulses_confirmed", "spacing_s", "net_mass_g",
          "status", "started", "duration_s", "cabinet", "output", "note", "burst", "burst_gap_ms"]
MIN_GAP_MS = 100                      # reward_pulse.gap_ms's lower limit (helpers/live_params.py)

Call = Callable[[str], Tuple[bool, str, float]]   # cmd -> (answered, reply, round trip ms)


# ---------------------------------------------------------------- the plan
def make_plan(widths: Sequence[int], reps: int, blanks: int, seed: int) -> List[dict]:
    """Every batch of the sweep, in the order to run them: reps per width, shuffled, plus blanks
    (width 0, no pulse) spread through it -- the first one first."""
    batches = [int(w) for w in widths for _ in range(reps)]
    rng = random.Random(seed)
    rng.shuffle(batches)
    for k in range(blanks):
        batches.insert(round(k * len(batches) / max(blanks, 1)) + k, 0)
    return [{"batch": i + 1, "width_ms": w} for i, w in enumerate(batches)]


def burst_sec(width_ms: int, burst: int = 1, gap_ms: int = 300) -> float:
    """How long one unit lasts, first opening to last closing [s]."""
    return (burst * width_ms + (burst - 1) * gap_ms) / 1000.0


def check_widths(widths: Sequence[int], spacing: float, burst: int = 1, gap_ms: int = 300) -> None:
    if not 1 <= burst <= 10:
        raise ValueError(f"--burst {burst}: 1-10 pulses per reward (reward_pulse.count)")
    if burst > 1 and gap_ms < MIN_GAP_MS:
        raise ValueError(f"--burst-gap-ms {gap_ms}: at least {MIN_GAP_MS} ms of closed valve between pulses")
    for w in widths:
        lo, hi = ttl.WIDTH_MS
        if not lo <= w <= hi:
            raise ValueError(f"width {w} ms: the cabinet pulses {lo}-{hi} ms")
        need = burst_sec(w, burst, gap_ms) + MIN_OFF_SEC
        if spacing < need:
            # TtlPulser extends a pulse that is still high instead of starting a new one, so two
            # pulses that overlap would come out as one long opening.
            what = f"{w} ms" if burst == 1 else f"bursts of {burst} x {w} ms, {gap_ms} ms apart"
            raise ValueError(f"--spacing {spacing:g} s is too short for {what}: the valve must close for at "
                             f"least {MIN_OFF_SEC:g} s between units (spacing >= {need:g} s)")


# ---------------------------------------------------------------- dispensing
def dispense(call: Call, output: int, width_ms: int, pulses: int, spacing: float,
             sleep: Callable[[float], None] = time.sleep, progress: Callable[[int], None] = lambda n: None,
             burst: int = 1, gap_ms: int = 300) -> Tuple[int, Optional[str]]:
    """Deliver `pulses` units -- single openings, or bursts of `burst` openings gap_ms apart -- spacing
    apart. Returns (units the cabinet confirmed in full, None) or, at the first pulse it does not confirm,
    (complete units before it, the reason). Never retries a pulse."""
    if width_ms == 0:                    # a blank: the same time, no pulse
        sleep(pulses * spacing)
        return 0, None
    t_next = time.monotonic()
    for n in range(pulses):
        wait = t_next - time.monotonic()
        if wait > 0:
            sleep(wait)
        t_next += spacing
        for k in range(burst):
            answered, reply, _ = call(f"PULSE {output} {width_ms}")
            if not (answered and reply.startswith("OK")):
                return n, reply or "no answer"
            if k < burst - 1:
                sleep((width_ms + gap_ms) / 1000.0)
        progress(n + 1)
    return pulses, None


def read_rows(path: Path) -> List[dict]:
    if not path.exists():
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def append_row(path: Path, row: dict) -> None:
    new = not path.exists()
    if not new:
        with path.open(newline="") as f:
            header = next(csv.reader(f), [])
        if header != FIELDS:
            raise ValueError(f"{path} was written by an older version of this script (other columns): "
                             f"use a new --out")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            w.writeheader()
        w.writerow({k: row.get(k, "") for k in FIELDS})


def ask(prompt: str) -> str:
    """input(), with the end of input (Ctrl-D, a closed pipe) read as q: never wait on nothing."""
    try:
        return input(prompt).strip().lower()
    except EOFError:
        print()
        return "q"


def ask_mass(prompt: str) -> Optional[float]:
    """Net mass in g, or None to discard the batch (or at the end of input)."""
    while True:
        s = ask(prompt)
        if s == "q":
            return None
        if s in ("x", "discard"):
            return None
        try:
            m = float(s)
        except ValueError:
            print("  a number in grams (e.g. 4.37), or x to discard this batch")
            continue
        if m < 0:
            print("  negative: was the scale tared with the cup on it? Re-tare and type the net mass.")
            continue
        return m


def run_batches(plan: List[dict], out: Path, call: Call, *, output: int, pulses: int, spacing: float,
                cabinet: str, note: str = "", drain: float = DRAIN_SEC, burst: int = 1, gap_ms: int = 300) -> int:
    """Run every batch of `plan` not yet in `out` with status ok. Returns how many are left."""
    done = {int(r["batch"]) for r in read_rows(out) if r["status"] == "ok"}
    todo = [b for b in plan if b["batch"] not in done]
    if not todo:
        print(f"All {len(plan)} batches are done: fit it with   python3 {Path(__file__).name} fit {out}")
        return 0
    print(f"{len(todo)} of {len(plan)} batches to go; each ~{pulses * spacing + drain:.0f} s.")
    for i, b in enumerate(todo, 1):
        w = b["width_ms"]
        what = ("BLANK (no pulse)" if w == 0 else f"{w} ms x {pulses}" if burst == 1
                else f"{pulses} bursts of {burst} x {w} ms ({gap_ms} ms apart)")
        s = ask(f"\n[{i}/{len(todo)}] batch {b['batch']}: {what}. Tare the cup, then Enter "
                f"(s = skip for now, q = quit): ")
        if s == "q":
            break
        if s == "s":
            continue
        started, t0 = dt.datetime.now().isoformat(timespec="seconds"), time.monotonic()
        try:
            got, err = dispense(call, output, w, pulses, spacing,
                                progress=lambda n: print(f"\r  {n}/{pulses}", end="", flush=True),
                                burst=burst, gap_ms=gap_ms)
        except KeyboardInterrupt:
            call("ALLOFF")
            got, err = -1, "interrupted (Ctrl-C); every output set low"
        print()
        row = {"batch": b["batch"], "width_ms": w, "pulses_requested": 0 if w == 0 else pulses,
               "pulses_confirmed": max(got, 0), "spacing_s": spacing, "started": started,
               "cabinet": cabinet, "output": output, "note": note, "burst": burst, "burst_gap_ms": gap_ms}
        if err:
            row.update(status="aborted", duration_s=round(time.monotonic() - t0, 1), note=f"{note} {err}".strip())
            append_row(out, row)
            print(f"  STOPPED: {err}. The batch is marked aborted and will be redone; empty the cup.")
            if got < 0:
                break
            continue
        time.sleep(drain)
        row["duration_s"] = round(time.monotonic() - t0, 1)
        mass = ask_mass("  net mass [g] (x = discard): ")
        row.update(net_mass_g=mass if mass is not None else "", status="ok" if mass is not None else "discarded")
        append_row(out, row)
        if mass is not None and w:
            print(f"  {1000 * mass / pulses:.1f} uL per {'pulse' if burst == 1 else 'burst'} at density 1 "
                  f"(the fit uses your --density)")
    left = len([b for b in plan if b["batch"] not in {int(r["batch"]) for r in read_rows(out) if r["status"] == "ok"}])
    print(f"\n{left} batch(es) left." + ("" if left else f" Fit it:  python3 {Path(__file__).name} fit {out}"))
    return left


# ---------------------------------------------------------------- the fit
def batch_volumes(rows: Sequence[dict], density: float) -> Tuple[List[Tuple[int, float]], List[float]]:
    """[(width ms, uL per pulse)] for every good pulsed batch, and the blanks' net masses [g]."""
    pts, blanks = [], []
    for r in rows:
        if r.get("status") != "ok" or r.get("net_mass_g", "") == "":
            continue
        w, m = int(float(r["width_ms"])), float(r["net_mass_g"])
        if w == 0:
            blanks.append(m)
            continue
        n = int(r["pulses_confirmed"])
        if n > 0:
            pts.append((w, 1000.0 * m / (density * n)))
    return pts, blanks


def per_width(pts: Sequence[Tuple[int, float]]) -> List[dict]:
    out = []
    for w in sorted({w for w, _ in pts}):
        v = np.array([u for x, u in pts if x == w])
        sd = float(v.std(ddof=1)) if len(v) > 1 else float("nan")
        out.append({"width_ms": w, "n": len(v), "mean_ul": float(v.mean()), "sd_ul": sd,
                    "cv_pct": 100.0 * sd / float(v.mean()) if v.mean() > 0 and len(v) > 1 else float("nan")})
    return out


def _line(pts: Sequence[Tuple[float, float]]) -> Tuple[float, float]:
    x, y = np.array([p[0] for p in pts], float), np.array([p[1] for p in pts], float)
    slope, intercept = np.polyfit(x, y, 1)
    return float(slope), float(intercept)


def fit_profile(rows: Sequence[dict], density: float, model: str = "linear", max_cv: float = 10.0,
                sources: Sequence[dict] = (), notes: str = "") -> dict:
    """The calibration profile from the sweep's rows (module doc, THE FIT)."""
    if model not in ("linear", "interp"):
        raise ValueError("model must be linear or interp")
    # One burst setting per fit: a burst's volume is not N single pulses (each opening adds a fixed amount).
    pulsed = [r for r in rows if r.get("status") == "ok" and str(r.get("width_ms", "0")) not in ("0", "")]
    settings = {(int(r.get("burst") or 1), int(r.get("burst_gap_ms") or 300) if int(r.get("burst") or 1) > 1 else 0)
                for r in pulsed}
    if len(settings) > 1:
        raise ValueError(f"these batches mix burst settings {sorted(settings)} (pulses per reward, gap ms): "
                         f"fit each setting's sweep on its own")
    burst, gap = next(iter(settings)) if settings else (1, 0)
    pts, blanks = batch_volumes(rows, density)
    table = per_width(pts)
    if len(table) < 2:
        raise ValueError("need good batches at 2 or more widths to fit")
    # The valid range: from the narrowest width whose batches agree (CV <= max_cv) up. Below it the valve
    # gives too variable a dose to promise a volume.
    reliable = [t for t in table if not t["cv_pct"] > max_cv]      # a single batch (nan CV) is not refused
    if len(reliable) < 2:
        raise ValueError(f"fewer than 2 widths with CV <= {max_cv:g}%: check the line for air, or add batches")
    w_min = reliable[0]["width_ms"]
    used = [p for p in pts if p[0] >= w_min]
    slope, intercept = _line(used)
    resid = np.array([u - (slope * w + intercept) for w, u in used])
    y = np.array([u for _, u in used])
    r2 = 1.0 - float((resid ** 2).sum()) / float(((y - y.mean()) ** 2).sum()) if len(y) > 2 else float("nan")
    fit_table = [t for t in table if t["width_ms"] >= w_min]
    for t in fit_table:
        pred = slope * t["width_ms"] + intercept
        t["linear_resid_pct"] = 100.0 * (t["mean_ul"] - pred) / pred if pred > 0 else float("nan")
    # Leave one width out: how well the line predicts a width it was not fitted to (interior widths).
    loo = []
    ws = [t["width_ms"] for t in fit_table]
    for t in fit_table[1:-1]:
        s, c = _line([p for p in used if p[0] != t["width_ms"]])
        loo.append(100.0 * (t["mean_ul"] - (s * t["width_ms"] + c)) / t["mean_ul"])
    if model == "interp":
        means = [t["mean_ul"] for t in fit_table]
        if any(b <= a for a, b in zip(means, means[1:])):
            raise ValueError("per-width means do not rise with width, so they cannot be inverted: use --model "
                             "linear, or redo the widths that are out of order")
    v_lo = (slope * ws[0] + intercept) if model == "linear" else fit_table[0]["mean_ul"]
    v_hi = (slope * ws[-1] + intercept) if model == "linear" else fit_table[-1]["mean_ul"]
    created = dt.datetime.now().isoformat(timespec="seconds")
    digest = hashlib.sha256(json.dumps([sorted(pts), density, model, max_cv]).encode()).hexdigest()[:8]
    return {
        "burst": burst,                      # pulses per reward (reward_pulse.count)
        "burst_gap_ms": gap,                 # closed valve between them (reward_pulse.gap_ms); 0 for single pulses
        "profile_id": f"reward-{created[:10]}-{digest}",
        "created": created,
        "model": model,
        "density_g_per_ml": density,
        "slope_ul_per_ms": slope,
        "intercept_ul": intercept,
        "points": [[t["width_ms"], t["mean_ul"]] for t in fit_table],   # the interp model's segments
        "valid_width_ms": [ws[0], ws[-1]],
        "valid_volume_ul": [v_lo, v_hi],
        "max_cv_pct": max_cv,
        "excluded_widths_ms": [t["width_ms"] for t in table if t["width_ms"] < w_min],
        "fit": {"r2": r2, "resid_sd_ul": float(resid.std(ddof=2)) if len(resid) > 2 else float("nan"),
                "leave_one_width_out_err_pct": loo, "n_batches": len(used)},
        "per_width": table,
        "blanks_g": blanks,
        "sources": list(sources),
        "notes": notes,
    }


def volume_for(profile: dict, width_ms: float) -> float:
    """uL per pulse the profile predicts for a width."""
    if profile["model"] == "interp":
        w, v = zip(*profile["points"])
        return float(np.interp(width_ms, w, v))
    return profile["slope_ul_per_ms"] * width_ms + profile["intercept_ul"]


def width_for(profile: dict, volume_ul: float, allow_outside: bool = False) -> Tuple[int, float]:
    """(width in whole ms, the volume that width gives) for a volume. 0 uL -> 0 ms (no pulse). A volume
    outside the profile's valid range raises ValueError unless allow_outside."""
    if volume_ul == 0:
        return 0, 0.0
    lo, hi = profile["valid_volume_ul"]
    if not lo - 1e-9 <= volume_ul <= hi + 1e-9 and not allow_outside:
        raise ValueError(f"{volume_ul:g} uL is outside this profile's calibrated range {lo:.0f}-{hi:.0f} uL "
                         f"({profile['valid_width_ms'][0]}-{profile['valid_width_ms'][1]} ms): extend the sweep")
    if profile["model"] == "interp":
        w, v = zip(*profile["points"])
        exact = float(np.interp(volume_ul, v, w))
    else:
        exact = (volume_ul - profile["intercept_ul"]) / profile["slope_ul_per_ms"]
    width = int(round(exact))
    return width, volume_for(profile, width)


# ---------------------------------------------------------------- commands
def _caller(args) -> Tuple[Call, str]:
    if args.demo:
        fake = ttl.FakeCabinet()
        return fake.call, "demo"
    ttl.check(args.cabinet, "STATUS")
    return (lambda cmd: ttl.call(args.cabinet, cmd)), args.cabinet


def _status_ok(call: Call) -> bool:
    answered, reply, ms = call("STATUS")
    print(f"Cabinet: {reply}" + (f"  ({ms:.0f} ms)" if answered else ""))
    return answered and reply.startswith("OK")


def cmd_sweep(args) -> int:
    widths = sorted(set(args.widths))
    check_widths(widths, args.spacing, args.burst, args.burst_gap_ms)
    out = Path(args.out)
    plan_path = out.with_suffix(".plan.json")
    want = {"widths": widths, "reps": args.reps, "blanks": args.blanks, "pulses": args.pulses,
            "spacing": args.spacing, "seed": args.seed, "output": args.output,
            "burst": args.burst, "burst_gap_ms": args.burst_gap_ms}
    if plan_path.exists():
        saved = json.loads(plan_path.read_text())
        saved["settings"].setdefault("burst", 1)
        saved["settings"].setdefault("burst_gap_ms", 300)
        if saved["settings"] != want:
            print(f"{plan_path} holds a sweep with other settings:\n  {saved['settings']}\nUse those settings to "
                  f"resume it, or a new --out for a new sweep.")
            return 2
        plan = saved["plan"]
        print(f"Resuming the sweep in {out}.")
    else:
        plan = make_plan(widths, args.reps, args.blanks, args.seed)
        plan_path.parent.mkdir(parents=True, exist_ok=True)
        plan_path.write_text(json.dumps({"settings": want, "plan": plan}, indent=1))
    call, cabinet = _caller(args)
    if not _status_ok(call):
        print("The cabinet's outputs are not available: start PulseTestServer (or LbrImpedanceControlServer) on "
              "the SmartPad, and check --cabinet. No experiment may be running.")
        return 1
    left = run_batches(plan, out, call, output=args.output, pulses=args.pulses, spacing=args.spacing,
                       cabinet=cabinet, note=args.note, drain=args.drain, burst=args.burst, gap_ms=args.burst_gap_ms)
    return 0 if left == 0 else 3


def _print_profile(p: dict) -> None:
    b = p.get("burst", 1)
    unit = "uL/pulse" if b == 1 else "uL/reward"
    print(f"\nProfile {p['profile_id']}  ({p['model']}, density {p['density_g_per_ml']:g} g/mL"
          + (f"; rewards of {b} pulses, {p.get('burst_gap_ms')} ms apart: width = each pulse" if b > 1 else "") + ")")
    print(f"  {'width ms':>8}  {'n':>2}  {unit:>9}  {'SD':>6}  {'CV %':>5}  {'line resid %':>12}")
    for t in p["per_width"]:
        res = t.get("linear_resid_pct")
        flag = "  excluded (CV)" if t["width_ms"] in p["excluded_widths_ms"] else ""
        print(f"  {t['width_ms']:>8}  {t['n']:>2}  {t['mean_ul']:>9.1f}  {t['sd_ul']:>6.1f}  {t['cv_pct']:>5.1f}  "
              f"{'' if res is None else f'{res:+.1f}':>12}{flag}")
    f = p["fit"]
    print(f"  line: {p['slope_ul_per_ms']:.3f} uL/ms x width {p['intercept_ul']:+.1f} uL   R^2 {f['r2']:.4f}   "
          f"residual SD {f['resid_sd_ul']:.1f} uL   ({f['n_batches']} batches)")
    if f["leave_one_width_out_err_pct"]:
        worst = max(abs(e) for e in f["leave_one_width_out_err_pct"])
        print(f"  a width left out of the fit is predicted within {worst:.1f}%")
    print(f"  valid: {p['valid_width_ms'][0]}-{p['valid_width_ms'][1]} ms = "
          f"{p['valid_volume_ul'][0]:.0f}-{p['valid_volume_ul'][1]:.0f} uL per pulse")
    if p["blanks_g"]:
        print(f"  blanks: {', '.join(f'{b:.3f}' for b in p['blanks_g'])} g (should be ~0: drips or drift otherwise)")
    curved = [t for t in p["per_width"] if abs(t.get("linear_resid_pct") or 0) > 5]
    if p["model"] == "linear" and curved:
        print(f"  NOTE: the line misses {', '.join(str(t['width_ms']) for t in curved)} ms by more than 5%: "
              f"consider --model interp")


def cmd_fit(args) -> int:
    rows, sources = [], []
    for name in args.csv:
        path = Path(name)
        rows += read_rows(path)
        sources.append({"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    p = fit_profile(rows, args.density, args.model, args.max_cv, sources, args.note)
    _print_profile(p)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(p, indent=1))
    print(f"\nWrote {out}. Widths for volumes:  python3 {Path(__file__).name} ms --profile {out} <uL> ...")
    return 0


def cmd_ms(args) -> int:
    p = json.loads(Path(args.profile).read_text())
    b = p.get("burst", 1)
    print(f"Profile {p['profile_id']} ({p['model']}); set reward_pulse.count {b}"
          + (f", reward_pulse.gap_ms {p.get('burst_gap_ms')}" if b > 1 else "") + ", and reward_pulse.pulse_ms to:")
    bad = 0
    for v in args.volumes:
        try:
            w, got = width_for(p, v, args.allow_outside)
            print(f"  {v:>7g} uL -> {w:>5d} ms   (gives {got:.1f} uL)")
        except ValueError as e:
            print(f"  {v:>7g} uL -> refused: {e}")
            bad += 1
    return 1 if bad else 0


def cmd_ul(args) -> int:
    p = json.loads(Path(args.profile).read_text())
    lo, hi = p["valid_width_ms"]
    for w in args.widths:
        out = "" if lo <= w <= hi else "   (outside the calibrated range: an extrapolation)"
        print(f"  {w:>5g} ms -> {volume_for(p, w):.1f} uL{out}")
    return 0


def cmd_check(args) -> int:
    """One batch at a width, against the profile: the pre-session check."""
    p = json.loads(Path(args.profile).read_text())
    burst, gap = int(p.get("burst", 1)), int(p.get("burst_gap_ms") or 300)
    check_widths([args.width], args.spacing, burst, gap)
    call, cabinet = _caller(args)
    if not _status_ok(call):
        return 1
    want = volume_for(p, args.width)
    unit = "pulse" if burst == 1 else f"burst of {burst}"
    if ask(f"{args.width} ms x {args.pulses} ({unit}s), profile says {want:.1f} uL each. Tare the cup, then "
           f"Enter (q = quit): ") == "q":
        return 1
    got, err = dispense(call, args.output, args.width, args.pulses, args.spacing,
                        progress=lambda n: print(f"\r  {n}/{args.pulses}", end="", flush=True),
                        burst=burst, gap_ms=gap)
    print()
    if err:
        print(f"STOPPED after {got} pulses: {err}")
        return 1
    time.sleep(args.drain)
    mass = ask_mass("net mass [g]: ")
    if mass is None:
        return 1
    each = 1000.0 * mass / (p["density_g_per_ml"] * got)
    off = 100.0 * (each - want) / want
    ok = abs(off) <= args.tolerance
    print(f"{each:.1f} uL per {unit}, {off:+.1f}% from the profile: "
          + ("OK." if ok else f"OUTSIDE +/-{args.tolerance:g}%. Check the reservoir fill, the line for air, and "
                              f"the spout; recalibrate if it persists."))
    if args.log:
        append_row(Path(args.log), {"batch": "check", "width_ms": args.width, "pulses_requested": args.pulses,
                                    "pulses_confirmed": got, "spacing_s": args.spacing, "net_mass_g": mass,
                                    "status": "ok", "started": dt.datetime.now().isoformat(timespec="seconds"),
                                    "cabinet": cabinet, "output": args.output, "burst": burst,
                                    "burst_gap_ms": gap if burst > 1 else "",
                                    "note": f"check vs {p['profile_id']}: {off:+.1f}%"})
    return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def cabinet_args(sp):
        sp.add_argument("--cabinet", default=DEFAULT_CABINET, help="cabinet address (nsp_sync.cabinet_ip)")
        sp.add_argument("--output", type=int, default=DEFAULT_OUTPUT, choices=range(1, ttl.CHANNELS + 1),
                        help="the reward output (reward_pulse.output, SYNC 4)")
        sp.add_argument("--pulses", type=int, default=50, help="valve openings per batch")
        sp.add_argument("--spacing", type=float, default=1.0, help="seconds from one opening to the next")
        sp.add_argument("--drain", type=float, default=DRAIN_SEC, help="seconds to wait for the last drop")
        sp.add_argument("--demo", action="store_true", help="a simulated cabinet: try the procedure, no hardware")

    sp = sub.add_parser("sweep", help="dispense and weigh batches at several widths")
    cabinet_args(sp)
    sp.add_argument("--widths", type=int, nargs="+", required=True, help="valve open times to test [ms]")
    sp.add_argument("--reps", type=int, default=3, help="batches per width")
    sp.add_argument("--blanks", type=int, default=2, help="batches with no pulse (drips, drift)")
    sp.add_argument("--seed", type=int, default=1, help="the shuffled batch order")
    sp.add_argument("--out", required=True, help="CSV the batches are appended to")
    sp.add_argument("--note", default="", help="rig conditions, e.g. 'water, full reservoir, spout v2'")
    sp.add_argument("--burst", type=int, default=1,
                    help="pulses per unit, as reward_pulse.count: each unit is a burst of this many pulses")
    sp.add_argument("--burst-gap-ms", type=int, default=300,
                    help="closed valve between a burst's pulses, as reward_pulse.gap_ms [ms]")
    sp.set_defaults(fn=cmd_sweep)

    sp = sub.add_parser("fit", help="fit volume against width; write a profile")
    sp.add_argument("csv", nargs="+", help="sweep CSV(s)")
    sp.add_argument("--density", type=float, default=1.0, help="fluid density [g/mL]: weigh a known volume")
    sp.add_argument("--model", choices=("linear", "interp"), default="linear")
    sp.add_argument("--max-cv", type=float, default=10.0, help="widths more variable than this [%%] are excluded")
    sp.add_argument("--out", required=True, help="profile JSON")
    sp.add_argument("--note", default="")
    sp.set_defaults(fn=cmd_fit)

    sp = sub.add_parser("ms", help="the width [ms] for each volume [uL]")
    sp.add_argument("--profile", required=True)
    sp.add_argument("volumes", type=float, nargs="+")
    sp.add_argument("--allow-outside", action="store_true", help="extrapolate beyond the calibrated range")
    sp.set_defaults(fn=cmd_ms)

    sp = sub.add_parser("ul", help="the volume [uL] each width [ms] gives")
    sp.add_argument("--profile", required=True)
    sp.add_argument("widths", type=float, nargs="+")
    sp.set_defaults(fn=cmd_ul)

    sp = sub.add_parser("check", help="one batch at a width against the profile (before a session)")
    cabinet_args(sp)
    sp.add_argument("--profile", required=True)
    sp.add_argument("--width", type=int, required=True)
    sp.add_argument("--tolerance", type=float, default=5.0, help="allowed difference from the profile [%%]")
    sp.add_argument("--log", default="", help="CSV to append the check to")
    sp.set_defaults(fn=cmd_check)

    args = ap.parse_args(argv)
    try:
        return args.fn(args)
    except (ValueError, FileNotFoundError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
