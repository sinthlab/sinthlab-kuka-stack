#!/usr/bin/env python3
"""Check a recorded trial before you analyse it.

Same idea as end_effector_design/check_layout.py does for the CAD: the things that quietly ruin a
dataset are cheap to test and expensive to discover months later. Run this on a finished trial and
it either says the file is sound or says exactly what is wrong with it.

    python3 validate_recording.py                     # newest trial in this folder
    python3 validate_recording.py --all               # every trial
    python3 validate_recording.py --file robot_trajectory_20260922_160000.csv

Exit code 0 if every trial passed, 1 otherwise -- so it can gate a batch analysis.

What it checks:
  SCHEMA     the column set matches what the experiment should write
  RATE       samples arrive at the FRI rate with no gaps
  CLOCK      the cabinet timestamp advances monotonically, in step with the sample count
  FRI HEALTH session COMMANDING_ACTIVE, safety NORMAL_OPERATION, drives ACTIVE for the WHOLE trial
  EVENTS     the tokens the experiment should emit are present, once, and in order
  SIDECAR    present, matching experiment, clocks captured at both ends
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

CORE = (["t", "t_wall", "t_ros", "fri_s", "fri_ns"]
        + ["x", "y", "z", "qx", "qy", "qz", "qw"]
        + [f"meas_A{i}" for i in range(1, 8)] + [f"cmd_A{i}" for i in range(1, 8)]
        + [f"ext_A{i}" for i in range(1, 8)]
        + ["tracking", "session", "quality", "safety", "drive", "control", "event", "event_arg"])

EXTRA = {"apple_pluck": ["disp_m"], "perturb": ["disp_m"],
         "maze": ["rel_a", "rel_b", "corridor", "off_rail", "rail_dist", "rail_nearest"]}

# Tokens that must appear exactly once, in this order. Anything not listed (checkpoint, the three
# maze endings, prestart_done) is optional or repeatable and is checked separately.
ORDER = {
    "apple_pluck": ["trial_start", "at_start", "quiet_end", "cue_go", "armed", "snap",
                    "cue_snap", "freeze", "recover_start", "trial_end"],
    "perturb": ["trial_start", "at_start", "quiet_end", "cue_go", "perturb_delay_start",
                "perturb_applied", "armed", "snap", "cue_snap", "freeze", "recover_start",
                "trial_end"],
    "maze": ["trial_start", "at_start", "fixture_active", "cue_go", "maze_armed", "trial_end"],
}
MAZE_ENDINGS = ("goal", "timeout", "safety_trip")

HEALTHY = {"session": (4, "COMMANDING_ACTIVE"), "safety": (0, "NORMAL_OPERATION"),
           "drive": (2, "ACTIVE")}


class Report:
    def __init__(self, name):
        self.name, self.bad = name, []

    def check(self, ok, msg):
        print(f"  {'OK  ' if ok else 'FAIL'} {msg}")
        if not ok:
            self.bad.append(msg)

    def note(self, msg):
        print(f"  ---- {msg}")


def read(path):
    with open(path) as f:
        rows = list(csv.reader(f))
    return [h.strip() for h in rows[0]], rows[1:]


def col(head, body, name, cast=float):
    i = head.index(name)
    out = []
    for r in body:
        try:
            out.append(cast(r[i]))
        except (ValueError, IndexError):
            out.append(None)
    return out


def validate(path) -> bool:
    r = Report(os.path.basename(path))
    print(f"\n=== {r.name} ===")
    head, body = read(path)
    meta_path = path[:-4] + ".meta.json"
    meta = None
    if os.path.exists(meta_path):
        try:
            meta = json.load(open(meta_path))
        except Exception as exc:
            print(f"  FAIL sidecar unreadable: {exc}")
            return False

    if meta is None:
        if "rel_a" in head and "t" not in head:
            r.note("9-column pre-spec recording; only the legacy checks apply")
            r.check(len(body) > 10, f"{len(body)} samples")
            return not r.bad
        r.check(False, "no .meta.json sidecar beside the CSV")
        exp = None
    else:
        exp = meta.get("experiment")

    # ---- SCHEMA ----
    print("\n [SCHEMA]")
    if exp in EXTRA:
        want = CORE + EXTRA[exp]
        r.check(head == want,
                f"{len(head)} columns match the {exp} schema ({len(want)} expected)")
        if head != want:
            miss, extra = set(want) - set(head), set(head) - set(want)
            if miss:
                r.note(f"missing: {sorted(miss)}")
            if extra:
                r.note(f"unexpected: {sorted(extra)}")
    else:
        r.check(False, f"unknown experiment {exp!r}; cannot check the schema")
    r.check(len(body) > 10, f"{len(body)} samples recorded")
    if len(body) <= 10:
        return not r.bad

    # ---- RATE ----
    print("\n [RATE]")
    t = [v for v in col(head, body, "t") if v is not None]
    span = t[-1] - t[0]
    hz = (len(t) - 1) / span if span > 0 else 0
    dts = sorted(b - a for a, b in zip(t, t[1:]))
    med = dts[len(dts) // 2]
    p99 = dts[int(0.99 * (len(dts) - 1))]
    r.note(f"{len(t)} samples over {span:.1f} s -> {hz:.1f} Hz; dt median {med*1000:.2f} ms, "
           f"p99 {p99*1000:.2f} ms, worst {dts[-1]*1000:.2f} ms")
    nominal = (meta or {}).get("fri", {}).get("sample_time_s") or 0.01
    r.check(abs(med - nominal) < 0.2 * nominal,
            f"median interval within 20% of the {nominal*1000:.0f} ms FRI period")
    gaps = [d for d in dts if d > 3 * nominal]
    r.check(not gaps, f"no gap longer than {3*nominal*1000:.0f} ms"
                      + (f" ({len(gaps)} found, worst {dts[-1]*1000:.0f} ms)" if gaps else ""))

    # ---- CLOCK ----
    print("\n [CLOCK]")
    fs = col(head, body, "fri_s", int)
    fns = col(head, body, "fri_ns", int)
    if all(v is not None for v in fs) and all(v is not None for v in fns):
        fri = [s + n * 1e-9 for s, n in zip(fs, fns)]
        back = sum(1 for a, b in zip(fri, fri[1:]) if b < a)
        r.check(back == 0, f"cabinet timestamp never goes backwards ({back} inversions)")
        drift = (fri[-1] - fri[0]) - span
        r.check(abs(drift) < 0.05 * max(span, 1.0),
                f"cabinet elapsed {fri[-1]-fri[0]:.2f} s vs wall {span:.2f} s "
                f"(differ by {drift*1000:+.0f} ms)")
        r.check(len({n % 1_000_000_000 for n in fns}) > 1,
                "fri_ns varies (a constant value means 1-second resolution)")
    else:
        r.check(False, "fri_s / fri_ns not parseable")

    # ---- FRI HEALTH ----
    print("\n [FRI HEALTH]")
    for name, (good, label) in HEALTHY.items():
        vals = col(head, body, name, int)
        bad = sum(1 for v in vals if v != good)
        r.check(bad == 0, f"{name} == {good} ({label}) for the whole trial"
                          + (f" -- {bad} samples ({100*bad/len(vals):.1f}%) were not" if bad else ""))

    # ---- EVENTS ----
    print("\n [EVENTS]")
    if meta and meta.get("partial"):
        # The trial never reached stop_and_save(): Ctrl-C, a crash, or an abort. The data up to that
        # point is real and the checks above still apply -- but demanding the full event sequence
        # from it would just be reporting the interruption ten times over.
        got_p = [row[head.index("event")].strip() for row in body if row[head.index("event")].strip()]
        r.note(f"{len(got_p)} events: {', '.join(got_p) if got_p else '(none)'}")
        r.note("TRIAL INCOMPLETE (sidecar says partial) -- it stopped after the last event above.")
        r.note("Everything before that point is valid; event-sequence checks are skipped.")
        print(f"\n  RESULT: {'PASS (partial)' if not r.bad else str(len(r.bad)) + ' PROBLEM(S)'}")
        return not r.bad
    ie, ia = head.index("event"), head.index("event_arg")
    seq = [(k, row[ie].strip(), row[ia].strip()) for k, row in enumerate(body) if row[ie].strip()]
    got = [tok for _, tok, _ in seq]
    r.note(f"{len(seq)} events: {', '.join(got) if got else '(none)'}")
    for tok in ORDER.get(exp, []):
        r.check(got.count(tok) == 1, f"'{tok}' appears exactly once (found {got.count(tok)})")
    idx = [got.index(tok) for tok in ORDER.get(exp, []) if tok in got]
    r.check(idx == sorted(idx), "required events are in the expected order")
    if exp == "maze":
        ends = [t for t in got if t in MAZE_ENDINGS]
        r.check(len(ends) == 1, f"exactly one ending (goal/timeout/safety_trip); found {ends}")
        r.note(f"{got.count('checkpoint')} checkpoint reward(s)")
    if exp in ("apple_pluck", "perturb") and "snap" in got and "armed" in got:
        ta = t[seq[got.index("armed")][0]]
        ts = t[seq[got.index("snap")][0]]
        r.check(ts > ta, f"reaction time snap - armed = {ts - ta:.3f} s")

    # ---- SIDECAR ----
    if meta:
        print("\n [SIDECAR]")
        cs = meta.get("clock_sync") or {}
        r.check(bool(cs.get("start")) and bool(cs.get("end")),
                "clock_sync captured at both trial start and end")
        for f in ("schema_version", "trial_index", "session_id"):
            r.check(meta.get(f) is not None, f"{f} present")
        if exp == "maze":
            g = meta.get("maze_geometry") or {}
            r.check(bool(g.get("corridors")),
                    "maze_geometry embedded (so the run plots against ITS rails, not today's)")
        if exp == "perturb":
            r.check(bool(meta.get("perturbation")), "perturbation recorded")

    print(f"\n  RESULT: {'PASS' if not r.bad else str(len(r.bad)) + ' PROBLEM(S)'}")
    return not r.bad


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--file", help="specific CSV (default: newest here)")
    ap.add_argument("--all", action="store_true", help="validate every trial in the folder")
    a = ap.parse_args()

    files = sorted(glob.glob(os.path.join(HERE, "robot_trajectory_*.csv")))
    if a.file:
        files = [a.file if os.path.isabs(a.file) else os.path.join(HERE, a.file)]
    elif not a.all:
        files = files[-1:] if files else []
    if not files:
        print("No trajectory CSV files found.")
        return 1

    results = [validate(f) for f in files]
    print(f"\n{'='*60}\n{sum(results)}/{len(results)} trial(s) passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
