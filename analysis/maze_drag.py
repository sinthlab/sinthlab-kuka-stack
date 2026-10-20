#!/usr/bin/env python3
"""Where does the resistance along a maze rail come from? Measured from a recording, no extra sensors.

Along a rail the hand feels three things, and each has its own fix:

  1. LAG. The fixture moves the cabinet's spring equilibrium after the hand, a few tens of ms late, so
     the equilibrium trails the arm by (lag x speed) and the spring pulls back with
     K_along x lag x speed.                    Fix: virtual_fixtures.rail_lead_sec, a shorter FRI send
                                               period, or a softer spring along the rail.
  2. DAMPING. The cabinet's Cartesian damper resists flange speed.   Fix: the SmartPad damping ratio.
  3. INERTIA. Accelerating the arm's links.                          Fix: posture.

Every trial CSV already holds what is needed (README section 7): measured and commanded joints and the
external joint torques at the FRI rate. Per sample:

  equilibrium trail  d = FK(commanded) - FK(measured)               (the spring deflection)
  spring force       F_spring = R K R^T d                           (K per FLANGE axis, as the SmartPad
                                                                     profile sets it -- manual §20.5)
  hand force         F_hand = pinv(J)^T tau_ext, tared at rest      (helpers/hand_force.py)

and along the rail being travelled, by least squares over the maze phase:

  d_along      = -lag x v_along                                      -> the lag [ms]
  F_hand_along = c x sign(v) + b x v_along + m x a_along             -> resistance per speed b [N s/m],
                                                                        a constant part c [N], mass m [kg]

The lag share of b is K_along x lag; what is left is damping (and anything else speed-proportional).

    python3 analysis/maze_drag.py expt_iiwa7_maze_20261021_101500          # every trial of one run
    python3 analysis/maze_drag.py run_folder/robot_trajectory_*.csv         # chosen trials
    python3 analysis/maze_drag.py run_folder --plot drag.png                 # force-speed plots
    python3 analysis/maze_drag.py run_folder --stiffness 1000 1000 2500      # if smartpad_params.json is missing

Needs numpy and optas (the ROS environment), and the iiwa7 URDF: $IIWA7_URDF, or --urdf, or it runs
xacro itself when ROS is sourced. Works for Move Vertical / Horizontal too (one rail).

RECORDING THAT ANALYSES WELL: do not touch the arm before the go cue (the quiet window is the force
tare); then drive along one long horizontal rail and one long vertical rail, each several times at a
steady slow, medium and fast pace. One run per setting being compared (e.g. rail_lead_sec 0 vs 0.03).
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
START = ("maze_armed", "armed")                                     # the fixture's live phase begins
END = ("goal", "timeout", "safety_trip", "stop_requested", "session_end", "release_wait")
QUIET = ("fixture_active", "cue_go")                                # untouched arm: the force tare
MIN_SPEED = 0.02           # [m/s] slower samples are "not moving along the rail"
SMOOTH = 5                 # samples in the position smoothing window (~50 ms at the 10 ms FRI period)
JAC_DAMPING = 0.02         # as the grasp detector (helpers/hand_force.py)
SPEEDS = (0.1, 0.2, 0.3)   # [m/s] where the breakdown is reported


# ---------------------------------------------------------------- loading
def load_csv(path):
    with open(path, encoding="utf-8-sig") as f:          # the logger writes a BOM
        rows = list(csv.reader(f))
    head, body = rows[0], rows[1:]
    ie = head.index("event")
    # A row whose FRI stamp repeats the previous one is a stale copy of that sample (the logger ticked
    # before a new FRI packet arrived): drop it, keeping its event on the row before.
    ifs, ifn = head.index("fri_s"), head.index("fri_ns")
    keep, last = [], None
    for r in body:
        stamp = (r[ifs], r[ifn])
        if keep and stamp == last:
            if r[ie] and not keep[-1][ie]:
                keep[-1] = keep[-1][:ie] + [r[ie]] + keep[-1][ie + 1:]
            continue
        keep.append(list(r))
        last = stamp
    body = keep
    events = [(i, r[ie]) for i, r in enumerate(body) if len(r) > ie and r[ie]]
    cols = {}
    for j, name in enumerate(head):
        if name in ("event", "event_arg"):
            continue
        out = np.full(len(body), np.nan)
        for i, r in enumerate(body):
            try:
                out[i] = float(r[j])
            except (ValueError, IndexError):
                pass
        cols[name] = out
    return cols, events


def first_event(events, names, after=-1):
    for i, ev in events:
        if i > after and any(tok in names for tok in ev.split("|")):
            return i
    return None


def smartpad_settings(run_dir):
    try:
        with open(os.path.join(run_dir, "smartpad_params.json")) as f:
            snap = json.load(f)
        return snap.get("settings") if snap.get("received") else None
    except (OSError, ValueError):
        return None


def urdf_path(arg):
    for p in (arg, os.environ.get("IIWA7_URDF")):
        if p and os.path.exists(p):
            return p
    out = os.path.join(tempfile.gettempdir(), "iiwa7_maze_drag.urdf")
    try:
        prefix = subprocess.run(["ros2", "pkg", "prefix", "lbr_description"], capture_output=True,
                                text=True, check=True).stdout.strip()
        xacro = os.path.join(prefix, "share/lbr_description/urdf/iiwa7/iiwa7.xacro")
        with open(out, "w") as f:
            subprocess.run(["xacro", xacro], stdout=f, check=True)
        return out
    except (OSError, subprocess.CalledProcessError):
        sys.exit("No iiwa7 URDF: pass --urdf, set IIWA7_URDF, or source ROS so xacro can build it.")


# ---------------------------------------------------------------- per trial
class Kinematics:
    def __init__(self, urdf):
        import optas
        r = optas.RobotModel(urdf_filename=urdf)
        self.fk = r.get_global_link_transform_function("lbr_link_ee", numpy_output=True)
        self.jac = r.get_global_link_geometric_jacobian_function(link="lbr_link_ee", numpy_output=True)

    def hand_force(self, tau, q):
        U, s, Vt = np.linalg.svd(np.asarray(self.jac(q), dtype=float), full_matrices=False)
        return ((Vt.T @ np.diag(s / (s * s + JAC_DAMPING ** 2)) @ U.T).T @ tau)[0:3]


def smooth_diff(x, t):
    """Moving-average smoothed signal and its time derivative (per column)."""
    k = np.ones(SMOOTH) / SMOOTH
    xs = np.stack([np.convolve(np.pad(c, SMOOTH // 2, mode="edge"), k, mode="valid") for c in x.T], 1)
    return xs, np.gradient(xs, t, axis=0)


def analyse_trial(path, kin, K_flange, meta):
    cols, events = load_csv(path)
    i0 = first_event(events, START)
    if i0 is None:
        return None, f"{os.path.basename(path)}: no '{'/'.join(START)}' event -- skipped"
    i1 = first_event(events, END, after=i0) or len(cols["t"]) - 1
    q_meas = np.stack([cols[f"meas_A{j}"] for j in range(1, 8)], 1)
    q_cmd = np.stack([cols[f"cmd_A{j}"] for j in range(1, 8)], 1)
    tau = np.stack([cols[f"ext_A{j}"] for j in range(1, 8)], 1)
    t = cols["fri_s"] + cols["fri_ns"] * 1e-9          # cabinet clock: the samples' own spacing
    if not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
        t = cols["t"]

    # Tare the force estimate where nobody touches the arm.
    q0, q1 = first_event(events, (QUIET[0],)), first_event(events, (QUIET[1],))
    tare_idx = range(q0, q1) if q0 is not None and q1 is not None and q1 > q0 + 5 else range(0, 0)

    n = len(t)
    p_meas, p_cmd, F_hand = np.zeros((n, 3)), np.zeros((n, 3)), np.zeros((n, 3))
    K_base = np.zeros((n, 3, 3))
    for i in range(n):
        if not (np.all(np.isfinite(q_meas[i])) and np.all(np.isfinite(q_cmd[i]))):
            p_meas[i] = p_cmd[i] = np.nan
            continue
        T = kin.fk(q_meas[i])
        p_meas[i], R = T[0:3, 3], T[0:3, 0:3]
        p_cmd[i] = kin.fk(q_cmd[i])[0:3, 3]
        K_base[i] = R @ np.diag(K_flange) @ R.T
        F_hand[i] = kin.hand_force(tau[i], q_meas[i]) if np.all(np.isfinite(tau[i])) else np.nan
    tare = np.nanmedian(F_hand[list(tare_idx)], 0) if len(tare_idx) else np.zeros(3)
    F_hand -= tare

    ok = np.all(np.isfinite(p_meas), 1)
    sl = np.arange(n)[ok]
    pos, vel = smooth_diff(p_meas[ok], t[ok])
    _, acc = smooth_diff(vel, t[ok])
    d = p_cmd[ok] - p_meas[ok]
    F_spring = np.einsum("nij,nj->ni", K_base[ok], d)

    # Which base axis is "along the rail" at each sample: from the maze geometry if the CSV says which
    # rail the arm is on, else the faster of the two free axes.
    geo = meta.get("maze_geometry") or {}
    locked = {"x": 0, "y": 1, "z": 2}.get(str(geo.get("restricted_axis", "x")).lower(), 0)
    free = [a for a in range(3) if a != locked]
    rail = cols.get("corridor")
    corridors = geo.get("corridors") or []
    along = np.full(len(sl), -1)
    for k, i in enumerate(sl):
        if not (i0 <= i <= i1):
            continue
        r = int(rail[i]) if rail is not None and np.isfinite(rail[i]) else -1
        if 0 <= r < len(corridors):
            a_min, a_max, b_min, b_max = corridors[r]
            along[k] = free[1] if a_min == a_max else free[0]
        else:
            v = vel[k]
            along[k] = free[int(np.argmax(np.abs(v[free])))]
    rows = []
    for k in range(len(sl)):
        ax = along[k]
        if ax < 0:
            continue
        cross = [a for a in free if a != ax][0]
        v_al = vel[k, ax]
        if abs(v_al) < MIN_SPEED or abs(vel[k, cross]) > 0.5 * abs(v_al):
            continue
        rows.append((ax, v_al, acc[k, ax], d[k, ax], F_spring[k, ax], F_hand[sl[k], ax], K_base[sl[k], ax, ax]))
    return np.array(rows) if rows else np.zeros((0, 7)), None


# ---------------------------------------------------------------- fits and report
def fit_axis(R):
    v, a, d, Fs, Fh, K = (R[:, j] for j in range(1, 7))
    lag = -np.linalg.lstsq(np.c_[v, np.ones_like(v)], d, rcond=None)[0][0]
    X = np.c_[np.sign(v), v, a]
    c, b, m = np.linalg.lstsq(X, Fh, rcond=None)[0]
    resid = Fh - X @ np.array([c, b, m])
    r2 = 1 - np.sum(resid ** 2) / max(np.sum((Fh - Fh.mean()) ** 2), 1e-12)
    return dict(n=len(v), lag=lag, K=float(np.median(K)), c=c, b=b, m=m, r2=r2,
                vmax=float(np.percentile(np.abs(v), 95)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="run folder(s) and/or trial CSV files")
    ap.add_argument("--urdf", help="iiwa7 URDF (default: $IIWA7_URDF, else built with xacro)")
    ap.add_argument("--stiffness", nargs=3, type=float, metavar=("KX", "KY", "KZ"),
                    help="translational stiffness per FLANGE axis [N/m], if smartpad_params.json is missing")
    ap.add_argument("--plot", help="save force-speed plots to this PNG")
    args = ap.parse_args()

    files = []
    for p in args.paths:
        files += sorted(glob.glob(os.path.join(p, "robot_trajectory_*.csv"))) if os.path.isdir(p) else [p]
    if not files:
        sys.exit("No trial CSVs found.")
    run_dir = os.path.dirname(os.path.abspath(files[0]))
    sp = smartpad_settings(run_dir)
    if args.stiffness:
        K_flange = np.array(args.stiffness)
    elif sp and "stiffness" in sp:
        K_flange = np.array(sp["stiffness"][0:3], dtype=float)
    else:
        sys.exit("No smartpad_params.json with the stiffness in the run folder: pass --stiffness KX KY KZ.")
    kin = Kinematics(urdf_path(args.urdf))

    print(f"Run: {run_dir}")
    if sp:
        print(f"SmartPad: {sp.get('stiffness_profile', '?')} | stiffness {sp.get('stiffness')} | "
              f"damping {sp.get('damping')} | null-space {sp.get('nullspace_stiffness')} | "
              f"FRI send period {sp.get('send_period_ms')} ms")
    allrows, lead = [], None
    for f in files:
        meta = {}
        try:
            with open(f[:-4] + ".meta.json") as fh:
                meta = json.load(fh)
        except (OSError, ValueError):
            pass
        lead = (meta.get("params") or {}).get("virtual_fixtures.rail_lead_sec", lead)
        rows, err = analyse_trial(f, kin, K_flange, meta)
        if err:
            print("  " + err)
            continue
        print(f"  {os.path.basename(f)}: {len(rows)} samples moving along a rail")
        allrows.append(rows)
    if not allrows or sum(len(r) for r in allrows) < 50:
        sys.exit("Too few samples moving along a rail to fit (need a run with steady guiding).")
    R = np.concatenate(allrows)
    print(f"rail_lead_sec in this run: {lead if lead is not None else 'unknown'}\n")

    fits = {}
    for ax, name in ((0, "X"), (1, "Y (sideways)"), (2, "Z (vertical)")):
        Ra = R[R[:, 0] == ax]
        if len(Ra) < 50:
            continue
        f = fits[name] = fit_axis(Ra)
        lag_b = f["K"] * f["lag"]
        print(f"=== along base {name}: {f['n']} samples, speeds up to {f['vmax']:.2f} m/s, fit R^2 {f['r2']:.2f}")
        print(f"  spring stiffness along it   {f['K']:7.0f} N/m   (the profile's flange axes, rotated)")
        print(f"  equilibrium lag             {f['lag'] * 1000:7.1f} ms")
        print(f"  resistance per speed        {f['b']:7.1f} N per m/s   (of which lag: {lag_b:.1f}, "
              f"damping and the rest: {f['b'] - lag_b:.1f})")
        print(f"  constant part               {f['c']:7.1f} N      apparent mass {f['m']:.1f} kg (rough)")
        for s in SPEEDS:
            print(f"    at {s:.1f} m/s: hand pushes ~{f['c'] + f['b'] * s:5.1f} N  "
                  f"= {lag_b * s:5.1f} lag + {(f['b'] - lag_b) * s:5.1f} damping/rest + {f['c']:4.1f} constant")
        print()

    lags = [f["lag"] for f in fits.values()]
    if lags:
        now = float(lead or 0.0)
        rec = min(0.05, max(0.0, now + float(np.median(lags))))
        print(f"rail_lead_sec: the residual lag suggests {rec:.3f} s (now {now:.3f}; live limit 0.05). "
              "Raise it in steps and stop if the arm starts to pull ahead of the hand.")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        names = [n for n in fits]
        fig, axs = plt.subplots(1, len(names), figsize=(6 * len(names), 4.5), squeeze=False)
        for ax_plot, name in zip(axs[0], names):
            axis = {"X": 0, "Y (sideways)": 1, "Z (vertical)": 2}[name]
            Ra = R[R[:, 0] == axis]
            v = Ra[:, 1]
            ax_plot.scatter(v, Ra[:, 5], s=3, alpha=0.3, label="hand force")
            ax_plot.scatter(v, -Ra[:, 4], s=3, alpha=0.3, label="spring pull-back (lag)")
            vv = np.linspace(v.min(), v.max(), 50)
            f = fits[name]
            ax_plot.plot(vv, f["c"] * np.sign(vv) + f["b"] * vv, "k-", label="fit: hand force")
            ax_plot.set_title(f"along base {name}: lag {f['lag'] * 1000:.0f} ms")
            ax_plot.set_xlabel("speed along the rail [m/s]")
            ax_plot.set_ylabel("force along the rail [N]")
            ax_plot.grid(True, alpha=0.3)
            ax_plot.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=120)
        print(f"Plot: {args.plot}")


if __name__ == "__main__":
    main()
