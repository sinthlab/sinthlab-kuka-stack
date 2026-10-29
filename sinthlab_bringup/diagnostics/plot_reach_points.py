#!/usr/bin/env python3
"""Plot iiwa7 joint-space positions in 3D, in the robot base frame (lbr_link_0).

Give it joint angles (A1..A7, degrees); it works out where the apple is and plots those points.
Optionally give the monkey's shoulder position: it draws a line from the shoulder to every point
and prints the distances, so you can see how far the monkey reached.

The kinematics are the repo's own: the iiwa7 URDF loaded through optas (as in
experiment_ctrl_gui/kinematics.py and diagnostics/solve_start_grid.py), the apple at
helpers/floor.APPLE_CENTRE_M along the flange z axis, joint limits and margins from the model /
helpers/start_grid.StartGrid. Nothing is re-implemented here.

    python3 plot_reach_points.py poses.csv
    python3 plot_reach_points.py poses.csv --shoulder 0.85 0.0 1.0
    python3 plot_reach_points.py poses.csv --shoulder 0.85 0.0 1.0 --arms
    python3 plot_reach_points.py --pose "far up=10,-20,0,-50,0,40,0" --pose "far left=40,-10,0,-60,0,30,0"
    python3 plot_reach_points.py poses.csv --save reach.png        # no window: write a PNG

Needs ROS sourced (for xacro and lbr_description) and optas, like the other diagnostics. If
IIWA7_URDF is set to an iiwa7 URDF file, that is used instead (as solve_start_grid.py does).

The CSV needs a header row with columns A1..A7 (A1_deg..A7_deg also works, so the dashboard's own
reach_calib_positions.csv / reach_calib_attempts.csv can be plotted directly). An optional `name`,
`label` or `note` column names each point. A file with no header, 7 numbers per row (and
optionally a name after them), works too.

Coordinates are the dashboard's: metres, +X toward the monkey, +Y the monkey's right, +Z up, origin
at the base of the arm. --tip 0 plots the flange itself instead of the apple.
"""
from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

# The repo's helpers: find the source tree above this file, so it runs from anywhere inside the repo.
for _p in Path(__file__).resolve().parents:
    if (_p / "sinthlab_bringup" / "sinthlab_bringup" / "helpers").is_dir():
        sys.path.insert(0, str(_p / "sinthlab_bringup"))
        break
else:
    raise SystemExit("put this script inside the sinthlab-kuka-stack repo (e.g. sinthlab_bringup/diagnostics/)")
from sinthlab_bringup.helpers.floor import APPLE_CENTRE_M  # noqa: E402

# Colours: ink and the first categorical blue of the reference palette; light chart surface.
SURFACE, INK, INK2, GRID, BLUE = "#fcfcfb", "#0b0b0b", "#52514e", "#dcdbd6", "#2a78d6"
ARM_LINKS = ["lbr_link_0", "lbr_link_2", "lbr_link_4", "lbr_link_6", "lbr_link_ee"]   # skeleton for --arms


class Kin:
    """The repo's iiwa7 model: forward kinematics per link, joint limits and margin (StartGrid)."""

    def __init__(self) -> None:
        try:
            import optas
            from sinthlab_bringup.helpers.start_grid import StartGrid
            urdf = os.environ.get("IIWA7_URDF")
            if not urdf:
                prefix = subprocess.run(["ros2", "pkg", "prefix", "lbr_description"], capture_output=True,
                                        text=True, check=True, timeout=20).stdout.strip()
                urdf = os.path.join(tempfile.gettempdir(), "plot_reach_points_iiwa7.urdf")
                with open(urdf, "w") as f:
                    subprocess.run(["xacro", os.path.join(prefix, "share/lbr_description/urdf/iiwa7/iiwa7.xacro")],
                                   stdout=f, check=True, timeout=60)
            robot = optas.RobotModel(urdf_filename=urdf)
        except Exception as exc:
            raise SystemExit(f"cannot load the iiwa7 model: {exc}\n"
                             "source ROS and the workspace (source /opt/ros/jazzy/setup.bash; "
                             "source ~/lbr-stack/install/setup.bash), or set IIWA7_URDF to an iiwa7 URDF file.")
        self._links = {n: robot.get_global_link_transform_function(n, numpy_output=True) for n in ARM_LINKS}
        self._grid = StartGrid(robot)
        self.limit_deg = np.degrees(np.asarray(robot.upper_actuated_joint_limits.toarray()).ravel())

    def fk(self, q_deg, tip_m: float):
        """Skeleton points (base, A2, A4, A6, flange), flange xyz and apple xyz, base frame [m]."""
        q = np.radians(np.asarray(q_deg, dtype=float))
        Ts = [self._links[n](q) for n in ARM_LINKS]
        T = Ts[-1]
        apple = T[:3, 3] + T[:3, :3] @ np.array([0.0, 0.0, tip_m])
        return np.array([t[:3, 3] for t in Ts]), T[:3, 3].copy(), apple

    def margin_deg(self, q_deg) -> float:
        return self._grid.margin_deg(np.radians(np.asarray(q_deg, dtype=float)))

    def sigma_min(self, q_deg) -> float:
        return self._grid.sigma_min(np.radians(np.asarray(q_deg, dtype=float)))


# ------------------------------------------------------------------ reading poses

def _floats(cells):
    try:
        return [float(c) for c in cells]
    except ValueError:
        return None


def read_csv(path: str):
    """[(name, [A1..A7])] from a CSV with or without a header row."""
    with open(path, newline="", encoding="utf-8-sig") as fh:
        rows = [r for r in csv.reader(fh) if any(c.strip() for c in r)]
    if not rows:
        raise SystemExit(f"{path}: empty")
    first = [c.strip() for c in rows[0]]
    poses = []
    if len(first) >= 7 and _floats(first[:7]) is not None:       # no header: 7 numbers, then a name
        for i, r in enumerate(rows, 1):
            r = [c.strip() for c in r]
            q = _floats(r[:7])
            if q is None:
                raise SystemExit(f"{path}, row {i}: needs 7 numbers (A1..A7 in degrees)")
            poses.append((r[7] if len(r) > 7 and r[7] else f"#{i}", q))
        return poses
    head = [h.lower() for h in first]
    cols = []
    for j in range(1, 8):
        for key in (f"a{j}", f"a{j}_deg"):
            if key in head:
                cols.append(head.index(key))
                break
        else:
            raise SystemExit(f"{path}: no column named A{j} or A{j}_deg (found: {', '.join(first)})")
    name_col = next((head.index(k) for k in ("name", "label", "note") if k in head), None)
    for i, r in enumerate(rows[1:], 2):
        r = [c.strip() for c in r]
        q = _floats([r[c] for c in cols]) if len(r) > max(cols) else None
        if q is None:
            raise SystemExit(f"{path}, row {i}: the A1..A7 cells must be numbers (degrees)")
        name = r[name_col] if name_col is not None and name_col < len(r) and r[name_col] else f"#{len(poses) + 1}"
        poses.append((name, q))
    return poses


def parse_pose_arg(text: str, n: int):
    label, _, nums = text.rpartition("=")
    q = _floats([c.strip() for c in nums.split(",")])
    if q is None or len(q) != 7:
        raise SystemExit(f'--pose "{text}": needs 7 comma-separated angles, e.g. "name=0,-13,0,-60,0,30,0"')
    return (label.strip() or f"#{n}"), q


# ------------------------------------------------------------------ plotting

def plot(kin, poses, tip, shoulder, arms, elev, azim, save, labels=True):
    import matplotlib
    if save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.ticker import MaxNLocator

    results = [(name, q, *kin.fk(q, tip)) for name, q in poses]
    apples = np.array([r[4] for r in results])
    everything = [apples]
    if shoulder is not None:
        everything.append(np.array([shoulder]))
    if arms:                                               # the whole arm down to the base is in view
        everything += [r[2] for r in results] + [np.zeros((1, 3))]
    allp = np.vstack(everything)
    lo, hi = allp.min(axis=0), allp.max(axis=0)
    pad = 0.08 * max(float((hi - lo).max()), 0.3)
    lo, hi = lo - pad, hi + pad
    short = np.maximum(0.30 - (hi - lo), 0.0) / 2          # at least 0.30 m per axis, so none is a sliver
    lo, hi = lo - short, hi + short

    fig = plt.figure(figsize=(10, 8), facecolor=SURFACE)
    ax = fig.add_subplot(projection="3d", facecolor=SURFACE)
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_box_aspect(tuple(hi - lo))                      # equal scale on all three axes
    ax.view_init(elev=elev, azim=azim)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_pane_color((1, 1, 1, 0))
        axis._axinfo["grid"].update(color=GRID, linewidth=0.6)
        axis.line.set_color(GRID)
        axis.set_tick_params(colors=INK2, labelsize=8)
        axis.set_major_locator(MaxNLocator(nbins=5))       # few, readable ticks
    ax.set_xlabel("X, toward the monkey [m]", color=INK2, labelpad=3)
    ax.set_ylabel("Y, monkey's right [m]", color=INK2, labelpad=3)
    ax.set_zlabel("Z, up [m]", color=INK2, labelpad=3)

    if arms:                                               # the whole arm for every pose, faint
        ax.scatter([0], [0], [0], marker="s", s=60, color=INK2, edgecolors=SURFACE, linewidths=1.5, zorder=5)
        ax.text(0, 0, 0, "  robot base", color=INK2, fontsize=8)
        for _, _, pts, _, apple in results:
            line = np.vstack([pts, apple])
            ax.plot(line[:, 0], line[:, 1], line[:, 2], color=BLUE, alpha=0.30, linewidth=1.0)

    ax.scatter(apples[:, 0], apples[:, 1], apples[:, 2], s=46, color=BLUE,
               edgecolors=SURFACE, linewidths=1.5, depthshade=False, zorder=6)

    for name, _, _, _, apple in results:
        if not labels:
            break
        text = name
        if shoulder is not None:
            text += f"  {np.linalg.norm(apple - shoulder):.2f} m"
        ax.text(apple[0], apple[1], apple[2] + 0.012, text, color=INK, fontsize=7.5, ha="center")

    handles = [Line2D([], [], marker="o", linestyle="", color=BLUE, markeredgecolor=SURFACE, markersize=7,
                      label="apple position of each pose")]
    if shoulder is not None:
        s = np.asarray(shoulder)
        for _, _, _, _, apple in results:
            ax.plot(*zip(s, apple), color=INK2, linewidth=0.9, linestyle="--", alpha=0.8)
        ax.scatter(*s, marker="D", s=80, color=INK, edgecolors=SURFACE, linewidths=1.5, depthshade=False, zorder=7)
        ax.text(s[0], s[1], s[2] + 0.02, "monkey shoulder", color=INK, fontsize=9, ha="center", fontweight="bold")
        handles += [Line2D([], [], marker="D", linestyle="", color=INK, markersize=7, label="monkey shoulder"),
                    Line2D([], [], color=INK2, linestyle="--", linewidth=1, label="shoulder to apple (distance in m)")]
        ax.legend(handles=handles, loc="upper left", frameon=False, labelcolor=INK2, fontsize=9)
    ax.set_title(f"Reach points: {len(results)} pose{'s' if len(results) != 1 else ''}, apple in the robot base frame",
                 color=INK, fontsize=12, pad=14)
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=160, facecolor=SURFACE, bbox_inches="tight", pad_inches=0.3)
        print(f"saved {save}")
    elif "agg" in matplotlib.get_backend().lower():
        # No GUI backend (no Tk/Qt for this Python): plt.show() would silently show nothing.
        out = "reach_points.png"
        fig.savefig(out, dpi=160, facecolor=SURFACE, bbox_inches="tight", pad_inches=0.3)
        print(f"no interactive plot window available (matplotlib backend: {matplotlib.get_backend()}); "
              f"saved {os.path.abspath(out)} instead. For a window: sudo apt install python3-tk")
    else:
        plt.show()


def report(kin, poses, tip, shoulder):
    print(f"{'name':<22}{'flange x,y,z [m]':<24}{'apple x,y,z [m]':<24}" + ("shoulder dist [m]" if shoulder is not None else ""))
    dists = []
    for name, q in poses:
        _, flange, apple = kin.fk(q, tip)
        row = f"{name[:21]:<22}{', '.join(f'{v:6.3f}' for v in flange):<24}{', '.join(f'{v:6.3f}' for v in apple):<24}"
        if shoulder is not None:
            dists.append(float(np.linalg.norm(apple - shoulder)))
            row += f"{dists[-1]:.3f}"
        print(row)
        for j, (a, lim) in enumerate(zip(q, kin.limit_deg), 1):
            if abs(a) > lim:
                print(f"   WARNING: A{j} = {a:g} deg is beyond the iiwa7 limit of +-{lim:g} deg")
            elif abs(a) > lim - 5:
                print(f"   note: A{j} = {a:g} deg is within 5 deg of its limit (+-{lim:g})")
        if kin.sigma_min(q) < 0.02:
            print(f"   note: near a singularity (sigma_min {kin.sigma_min(q):.3f})")
    if dists:
        print(f"\nshoulder-to-apple distance over {len(dists)} points: min {min(dists):.3f} m, "
              f"mean {np.mean(dists):.3f} m, max {max(dists):.3f} m")


def export_html(kin, poses, tip, shoulder, arms, path):
    """One self-contained interactive 3D page (rotate / zoom / hover). Opens in any browser, needs no ROS."""
    import plotly.graph_objects as go
    res = [(n, q, *kin.fk(q, tip)) for n, q in poses]
    ap = np.array([r[4] for r in res])
    hover = [f"{n}<br>apple {a[0]:.3f}, {a[1]:.3f}, {a[2]:.3f} m<br>A1..A7: " + ", ".join(f"{v:g}" for v in q)
             + (f"<br>shoulder dist {np.linalg.norm(a - shoulder):.3f} m" if shoulder is not None else "")
             for (n, q, _, _, _), a in zip(res, ap)]
    fig = go.Figure()
    if arms:
        for _, _, pts, _, a in res:
            line = np.vstack([pts, a])
            fig.add_trace(go.Scatter3d(x=line[:, 0], y=line[:, 1], z=line[:, 2], mode="lines", showlegend=False,
                                       line=dict(color=BLUE, width=2), opacity=0.3, hoverinfo="skip"))
    fig.add_trace(go.Scatter3d(x=ap[:, 0], y=ap[:, 1], z=ap[:, 2], mode="markers+text", name="apple position",
                               text=[r[0] for r in res], textposition="top center", hovertext=hover, hoverinfo="text",
                               marker=dict(size=5, color=BLUE, line=dict(color=SURFACE, width=2))))
    if shoulder is not None:
        for a in ap:
            fig.add_trace(go.Scatter3d(x=[shoulder[0], a[0]], y=[shoulder[1], a[1]], z=[shoulder[2], a[2]],
                                       mode="lines", showlegend=False, hoverinfo="skip",
                                       line=dict(color=INK2, width=2, dash="dash")))
        fig.add_trace(go.Scatter3d(x=[shoulder[0]], y=[shoulder[1]], z=[shoulder[2]], mode="markers+text",
                                   name="monkey shoulder", text=["monkey shoulder"], textposition="top center",
                                   marker=dict(size=7, color=INK, symbol="diamond")))
    fig.update_layout(title=f"Reach points: {len(res)} poses, apple in the robot base frame [m]",
                      scene=dict(aspectmode="data", xaxis_title="X, toward the monkey [m]",
                                 yaxis_title="Y, monkey's right [m]", zaxis_title="Z, up [m]"),
                      paper_bgcolor=SURFACE, margin=dict(l=0, r=0, t=50, b=0))
    fig.write_html(path, include_plotlyjs=True)        # plotly.js embedded: works offline
    print(f"saved {os.path.abspath(path)} (open it in a browser)")


def export_csv(kin, poses, tip, shoulder, path):
    """Joint angles plus the computed positions, as plain numbers you can open anywhere."""
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["name"] + [f"A{j}" for j in range(1, 8)] + ["flange_x", "flange_y", "flange_z",
                   "apple_x", "apple_y", "apple_z"] + (["shoulder_dist_m"] if shoulder is not None else []))
        for n, q in poses:
            _, fl, a = kin.fk(q, tip)
            w.writerow([n] + list(q) + [f"{v:.4f}" for v in fl] + [f"{v:.4f}" for v in a]
                       + ([f"{np.linalg.norm(a - shoulder):.4f}"] if shoulder is not None else []))
    print(f"saved {os.path.abspath(path)}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="*", help="CSV file(s) of joint poses (see above)")
    ap.add_argument("--pose", action="append", default=[], metavar='"name=A1,...,A7"', help="one pose on the command line; repeatable")
    ap.add_argument("--shoulder", nargs=3, type=float, metavar=("X", "Y", "Z"), help="monkey shoulder, robot base frame [m]")
    ap.add_argument("--tip", type=float, default=APPLE_CENTRE_M, help=f"apple centre along the flange z axis [m] (default {APPLE_CENTRE_M}, the repo's; 0 = the flange)")
    ap.add_argument("--arms", action="store_true", help="also draw the whole arm for every pose")
    ap.add_argument("--no-labels", action="store_true", help="no name / distance text beside the points (crowded plots)")
    ap.add_argument("--elev", type=float, default=22.0, help="view elevation [deg]")
    ap.add_argument("--azim", type=float, default=-55.0, help="view azimuth [deg]")
    ap.add_argument("--html", metavar="FILE.html", help="write an interactive 3D page (rotate/zoom in any browser; needs: pip install plotly)")
    ap.add_argument("--export", metavar="FILE.csv", help="write joint angles + flange/apple XYZ (+ shoulder distance) to a CSV")
    ap.add_argument("--save", metavar="FILE.png", help="write a PNG instead of opening a window")
    a = ap.parse_args()

    poses = []
    for path in a.csv:
        poses += read_csv(path)
    for text in a.pose:
        poses.append(parse_pose_arg(text, len(poses) + 1))
    if not poses:
        ap.error("no poses: give a CSV file or one or more --pose arguments")
    shoulder = np.array(a.shoulder) if a.shoulder else None
    kin = Kin()
    report(kin, poses, a.tip, shoulder)
    if a.export:
        export_csv(kin, poses, a.tip, shoulder, a.export)
    if a.html:
        export_html(kin, poses, a.tip, shoulder, a.arms, a.html)
        return 0
    plot(kin, poses, a.tip, shoulder, a.arms, a.elev, a.azim, a.save, labels=not a.no_labels)
    return 0


if __name__ == "__main__":
    sys.exit(main())
