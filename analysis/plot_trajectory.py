#!/usr/bin/env python3
"""Plot a recorded trajectory from analysis/robot_trajectory_*.csv.

Two views:
  * MAZE view (rel_a vs rel_b) with the actual rails from maze_params.yaml drawn underneath, so you
    can see where the operator went, where they were held off a rail, and which forks they hit. This
    is the useful one for maze runs.
  * 3-D Cartesian path in the robot base frame, ANIMATED. Kept as a real perspective 3-D view: the
    maze panel already shows the Y-Z plane face-on, so pointing this one down the X axis too would
    just duplicate it. --elev/--azim rotate it (use --elev 0 --azim 0 for the face-on view).

Runs on numpy + matplotlib only -- deliberately NO pandas, which is not installed in the ROS
environment on the arm box and made the previous version unrunnable there.

    python3 plot_trajectory.py                 # newest CSV, both views on screen
    python3 plot_trajectory.py --save out.gif  # animated GIF (headless / WSL)
    python3 plot_trajectory.py --save out.png  # static PNG (final frame)
    python3 plot_trajectory.py --fps 40 --frames 400
    python3 plot_trajectory.py --elev 0 --azim 0   # face-on Y-Z instead of perspective
    python3 plot_trajectory.py --file robot_trajectory_20260902_115639.csv
    python3 plot_trajectory.py --all           # overlay every CSV in the folder (maze view)
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.join(HERE, "..", "sinthlab_bringup", "config", "maze_params.yaml")


def load_csv(path):
    """Return {column: np.array}. Missing/short columns simply do not appear."""
    with open(path) as f:
        rows = list(csv.reader(f))
    head, body = rows[0], rows[1:]
    out = {}
    for i, name in enumerate(head):
        vals = []
        for r in body:
            try:
                vals.append(float(r[i]))
            except (ValueError, IndexError):
                vals.append(np.nan)
        out[name.strip()] = np.array(vals)
    return out


def load_rails(profile=None):
    """Read the active maze rails from maze_params.yaml. Returns [] if unavailable."""
    try:
        import yaml
    except Exception:
        return []
    try:
        with open(os.path.abspath(PARAMS)) as f:
            p = yaml.safe_load(f)["/**"]["ros__parameters"]
        vf = p["virtual_fixtures"]
        prof = profile or p["virtual_fixture_profile"]
        c = vf[prof]
        n = len(c["corridor_a_min"])
        return [(c["corridor_a_min"][i], c["corridor_a_max"][i],
                 c["corridor_b_min"][i], c["corridor_b_max"][i]) for i in range(n)], p
    except Exception:
        return []


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", help="specific CSV (default: newest in this folder)")
    ap.add_argument("--all", action="store_true", help="overlay every CSV (maze view only)")
    ap.add_argument("--save", help="write here instead of showing a window; .gif animates, .png is static")
    ap.add_argument("--fps", type=int, default=25, help="animation frames per second")
    ap.add_argument("--frames", type=int, default=250, help="how many frames to render")
    ap.add_argument("--no-anim", action="store_true", help="draw the whole path at once")
    ap.add_argument("--stretch-x", action="store_true",
                    help="give the locked X axis its own scale to magnify drift (DISTORTS the view)")
    # Default to a normal 3-D perspective. Looking straight down X (elev=0, azim=0) makes this panel a
    # near-duplicate of the maze view on the left, which is already the Y-Z plane face-on -- the two
    # panels should show DIFFERENT things. Pass --elev 0 --azim 0 if you do want the face-on view.
    ap.add_argument("--elev", type=float, default=22.0, help="3-D elevation (0 = edge-on to Y-Z)")
    ap.add_argument("--azim", type=float, default=-58.0, help="3-D azimuth (0 = looking down +X)")
    a = ap.parse_args()

    if a.save:
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    files = sorted(glob.glob(os.path.join(HERE, "robot_trajectory_*.csv")))
    if not files:
        print("No trajectory CSV files found next to this script.")
        return 1
    if a.file:
        chosen = [a.file if os.path.isabs(a.file) else os.path.join(HERE, a.file)]
    elif a.all:
        chosen = files
    else:
        chosen = [max(files, key=os.path.getmtime)]

    rails_info = load_rails()
    rails, params = (rails_info if rails_info else ([], None))
    has_maze = all("rel_a" in load_csv(c) for c in chosen[:1])

    fig = plt.figure(figsize=(15, 6.5))
    ax1 = fig.add_subplot(1, 2, 1)
    ax2 = fig.add_subplot(1, 2, 2, projection="3d")

    # ---- maze view -------------------------------------------------------------------------
    for (amn, amx, bmn, bmx) in rails:
        ax1.plot([amn, amx], [bmn, bmx], color="0.75", lw=6, solid_capstyle="round", zorder=1)
    if params:
        cm = params["checkpoint_monitor"]
        ax1.scatter(cm["checkpoint_y"], cm["checkpoint_z"], s=200, facecolors="none",
                    edgecolors="tab:orange", lw=2, zorder=4, label="fork (checkpoint)")
        ax1.scatter([cm["goal_y"]], [cm["goal_z"]], marker="*", s=320, color="tab:green",
                    zorder=5, label="goal")
        ax1.scatter([0], [0], marker="s", s=90, color="tab:blue", zorder=5, label="start")

    for path in chosen:
        d = load_csv(path)
        if "rel_a" not in d:
            print(f"  {os.path.basename(path)}: no maze columns (older recording) -- skipping maze view")
            continue
        a_, b_ = d["rel_a"], d["rel_b"]
        off = d.get("off_rail", np.zeros_like(a_))
        lbl = os.path.basename(path)[18:-4] if len(chosen) > 1 else "path"
        ax1.plot(a_, b_, lw=1.2, alpha=0.85, zorder=3, label=lbl)
        m = off > 0.5
        if m.any():
            ax1.scatter(a_[m], b_[m], s=6, color="tab:red", zorder=6,
                        label=None if len(chosen) > 1 else "off rail")
    ax1.set_xlabel("a  (sideways, m from start)")
    ax1.set_ylabel("b  (up/down, m from start)")
    ax1.set_title("Maze view — rails in grey")
    # Bound the axes explicitly from rails+data and use adjustable="box": with "datalim" the equal
    # aspect is free to expand the limits to fill whatever shape the subplot ends up, which blew the
    # view up to +/-1 m once the 3-D panel changed the layout.
    _a = [v for r in rails for v in r[:2]] or [0.0]
    _b = [v for r in rails for v in r[2:]] or [0.0]
    for path in chosen:
        _d = load_csv(path)
        if "rel_a" in _d:
            _a += [np.nanmin(_d["rel_a"]), np.nanmax(_d["rel_a"])]
            _b += [np.nanmin(_d["rel_b"]), np.nanmax(_d["rel_b"])]
    pad = 0.05
    ax1.set_xlim(min(_a) - pad, max(_a) + pad)
    ax1.set_ylim(min(_b) - pad, max(_b) + pad)
    ax1.set_aspect("equal", adjustable="box")
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=8, loc="best")

    # ---- 3-D Cartesian path, rotated so the Y-Z plane faces the viewer, animated ------------
    d = load_csv(chosen[-1])
    x, y, z = d["x"], d["y"], d["z"]
    # Rails are stored relative to the anchor; recover the anchor from the data itself
    # (anchor_y = y - rel_a) and draw them in the arm's X plane so they sit under the path.
    if "rel_a" in d:
        ay, az = y[0] - d["rel_a"][0], z[0] - d["rel_b"][0]
        xr = float(np.nanmedian(x))
        for (amn, amx, bmn, bmx) in rails:
            ax2.plot([xr, xr], [ay + amn, ay + amx], [az + bmn, az + bmx],
                     color="0.75", lw=5, solid_capstyle="round", zorder=1)
        if params:
            cm = params["checkpoint_monitor"]
            ax2.scatter([xr] * len(cm["checkpoint_y"]),
                        [ay + v for v in cm["checkpoint_y"]],
                        [az + v for v in cm["checkpoint_z"]],
                        s=90, facecolors="none", edgecolors="tab:orange", lw=2)
            ax2.scatter([xr], [ay + cm["goal_y"]], [az + cm["goal_z"]],
                        marker="*", s=260, color="tab:green")
    ax2.scatter([x[0]], [y[0]], [z[0]], marker="s", s=60, color="tab:blue", label="start")
    ax2.set_xlabel("X (m)  — locked")
    ax2.set_ylabel("Y (m)  — sideways")
    ax2.set_zlabel("Z (m)  — height")
    ax2.set_title("Cartesian path (3-D, base frame)")
    # elev=0, azim=0 looks straight down +X, so Y runs across and Z up: the maze face-on.
    ax2.view_init(elev=a.elev, azim=a.azim)
    # ALL THREE AXES SHARE ONE SCALE by default, so the picture is metrically honest: the path really
    # is nearly planar (X is the locked axis and moves only millimetres), and it should LOOK planar.
    #
    # An earlier version gave X its own tighter scale to make drift visible. That magnified X by ~10x
    # relative to Y/Z, so a 7 mm wobble was drawn like 70 mm of in-plane error and the arm appeared to
    # be off the rails for most of the run when it was actually within 2.3 mm of them (median).
    # Magnifying one axis of a trajectory plot is a good way to invent a problem that is not there, so
    # it is now opt-in via --stretch-x and the title says so when it is on.
    rng = max(x.ptp(), y.ptp(), z.ptp()) / 2.0 or 0.1
    cy, cz = y.mean(), z.mean()
    ax2.set_ylim(cy - rng, cy + rng); ax2.set_zlim(cz - rng, cz + rng)
    if a.stretch_x:
        xpad = max(x.ptp(), 0.02) * 0.75
        ax2.set_xlim(x.mean() - xpad, x.mean() + xpad)
        ax2.set_title("Cartesian path (3-D) — X EXAGGERATED %.0fx" % (2 * rng / max(x.ptp(), 0.02)))
    else:
        ax2.set_xlim(x.mean() - rng, x.mean() + rng)
    # Edge-on, the X ticks collapse into an unreadable smear -- drop them and say so on the axis label.
    face_on = abs(a.elev) < 5 and abs(a.azim) < 5
    if face_on:
        ax2.set_xticks([])                 # not just the labels: the tick marks smear too, edge-on
        ax2.set_xlabel("X — locked axis (edge-on)", labelpad=-6)
    ax2.legend(fontsize=8, loc="upper right")

    trail, = ax2.plot([], [], [], lw=1.6, color="tab:blue")
    head, = ax2.plot([], [], [], "o", ms=7, color="tab:red")
    readout = ax2.text2D(0.02, 0.98, "", transform=ax2.transAxes, va="top", fontsize=9,
                         family="monospace", bbox=dict(fc="white", ec="0.8", alpha=0.85))

    n = len(y)
    idx = np.unique(np.linspace(0, n - 1, min(a.frames, n)).astype(int))
    x0 = x[0]

    def draw(k):
        i = idx[k]
        trail.set_data(x[: i + 1], y[: i + 1])
        trail.set_3d_properties(z[: i + 1])
        head.set_data([x[i]], [y[i]])
        head.set_3d_properties([z[i]])
        txt = (f"t   {d['time'][i] - d['time'][0]:6.1f} s\n"
               f"Y {y[i]:+.3f}  Z {z[i]:+.3f}\n"
               f"X drift {1000 * (x[i] - x0):+5.0f} mm")
        if "rail_dist" in d:
            txt += f"\noff rail {1000 * d['rail_dist'][i]:5.0f} mm"
        readout.set_text(txt)
        return trail, head, readout

    # Only build the animation if something will actually consume it -- an unrendered FuncAnimation
    # emits a warning when it is garbage-collected.
    want_anim = (not a.no_anim) and (a.save is None or a.save.lower().endswith(".gif"))
    anim = None
    if want_anim:
        from matplotlib import animation
        anim = animation.FuncAnimation(fig, draw, frames=len(idx), interval=1000 / a.fps,
                                       blit=False, repeat=True)
    else:
        draw(len(idx) - 1)

    # ---- a couple of numbers worth knowing -------------------------------------------------
    d0 = load_csv(chosen[-1])
    dur = d0["time"][-1] - d0["time"][0]
    dist = float(np.nansum(np.hypot(np.diff(d0["x"]), np.hypot(np.diff(d0["y"]), np.diff(d0["z"])))))
    msg = f"{os.path.basename(chosen[-1])}   {dur:.1f} s   path {dist:.2f} m"
    if "rail_dist" in d0:
        rd = d0["rail_dist"]
        msg += f"   off-rail {100*np.mean(d0.get('off_rail', 0) > 0.5):.0f}% of samples, max {1000*np.nanmax(rd):.0f} mm"
    fig.suptitle(msg, fontsize=10)
    fig.subplots_adjust(left=0.07, right=0.97, top=0.88, bottom=0.10, wspace=0.22)

    if a.save:
        if a.save.lower().endswith(".gif") and anim is not None:
            anim.save(a.save, writer="pillow", fps=a.fps, dpi=100)
        else:
            draw(len(idx) - 1)          # freeze on the final frame for a still image
            fig.savefig(a.save, dpi=130)
        print(f"  wrote {a.save}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
