"""The live view: the running experiment's bounds, drawn from its own parameters, and the arm in them.

Read-only and dashboard-side: it listens (the measured joints on lbr_state, the CLIK's target_frame) and
reads the run's parameters; nothing in the experiments changes for it.

    geometry   the bounds, in the base frame [m], built from the parameters the run uses (the dashboard's
               copy, with the live values the orchestrator reports): see SHAPES below
    anchor     where the experiment's start-relative bounds sit: the start pose's end effector until the
               trial is armed, then where the arm was at that moment -- the experiments anchor on the arm
               around then (the maze after it settles, Free Move at the tare), so the drawing can sit a
               few mm off theirs
    live       the end effector (forward kinematics of the measured joints), the pull point (the CLIK
               target, where there is one), the phase

SHAPES (each a dict, positions in the base frame [m]):
    box      min, max: [x, y, z], a None side is open (Restricted Plane's cartesian_box)
    seg      a, b: a line (a rail, a zero-width maze rail)
    poly     pts: a polyline (the sine rail)
    sphere   c, r: a sphere (a threshold, the safety distance) -- a circle in both views
    vcyl     c: [x, y], r: a vertical cylinder -- a circle in the top view
    zline    z: a height (the floor and its trip line) -- the front view only
    marker   p, r: a point with a radius (start, checkpoint, goal, rail end)
Each also has role (how it is drawn), label and, where the view lights it, id.
"""
from __future__ import annotations

import math
import time
from typing import Dict, List, Optional

import numpy as np

import experiments as ex  # noqa: F401  (puts sinthlab_bringup on sys.path)
import kinematics
from sinthlab_bringup.helpers.floor import reference_drop
from sinthlab_bringup.helpers.free_move_box import box_bounds
from sinthlab_bringup.helpers import free_reach
from sinthlab_bringup.helpers import forage
from sinthlab_bringup.helpers import switch_rails
from sinthlab_bringup.helpers.rail_shapes import direction_name, frontal_dir as rail_dir, layout_degs  # noqa: E402

# Without a robot model (demo without ROS): the pre-training start, tool along +X.
NOMINAL_START = np.array([0.38, 0.006, 1.029])
NOMINAL_R = np.array([[0.096, -0.032, 0.995], [0.02, 0.999, 0.03], [-0.995, 0.017, 0.097]])
PLANES = {"frontal": (1, 2), "horizontal": (0, 1), "sagittal": (0, 2)}   # as helpers/start_grid.py
AXES = {"x": 0, "y": 1, "z": 2}
END_NAMES = {"z": ("down", "up"), "y": ("left", "right")}               # as actions/centre_out.py
STALE_SEC = 1.0


def _safety_distance(p: dict) -> float:
    for k, v in p.items():
        if k.endswith("_safety.max_displacement_m"):
            return float(v)
    return 0.0


class LiveView:
    def __init__(self, bridge) -> None:
        self._bridge = bridge
        self._key: Optional[str] = None
        self._params: Dict[str, object] = {}
        self._anchor: Optional[np.ndarray] = None
        self._trial = None
        self._armed_trial = None
        self._version = 0
        self._geometry: Optional[dict] = None
        # This trial so far, from the phases as they come (the page may open mid-trial):
        self._target = None                     # centre-out: the goal end ("either" or a name)
        self._reached = None                    # centre-out: the end reached
        self._visited: List[int] = []           # maze: the forks passed

    # ------------------------------------------------------------------ the start and the anchor

    def _start(self):
        """(position, rotation) of the start pose's end effector, and whether it is the real model's."""
        q = self._params.get(ex.START_POSE)
        T = kinematics.ee_pose(np.radians(np.asarray(q, dtype=float))) if q is not None else None
        if T is None:
            return NOMINAL_START.copy(), NOMINAL_R, False
        return T[:3, 3].copy(), T[:3, :3], True

    def _ee(self) -> Optional[np.ndarray]:
        """The end effector now [m], base frame, or None (no fresh state)."""
        live = self._bridge.live() if hasattr(self._bridge, "live") else None
        if not live:
            return None
        if "offset" in live:                          # demo: an offset from the anchor
            return self.anchor() + np.asarray(live["offset"], dtype=float)
        if live.get("q") is None or time.time() - live.get("q_t", 0.0) > STALE_SEC:
            return None
        T = kinematics.ee_pose(live["q"])
        return None if T is None else T[:3, 3].copy()

    def anchor(self) -> np.ndarray:
        return self._anchor if self._anchor is not None else self._start()[0]

    def update(self, key: Optional[str], params: Dict[str, object], status: Optional[dict]) -> None:
        """Call on every status: follows the running experiment and re-anchors once per trial, at armed."""
        merged = dict(params)
        if status and isinstance(status.get("live"), dict):
            merged.update(status["live"])
        changed = key != self._key or merged != self._params
        if key != self._key:
            self._anchor, self._trial, self._armed_trial = None, None, None
        self._key, self._params = key, merged
        trial = status.get("trial") if status else None
        phase = status.get("phase") if status else None
        if trial != self._trial:
            self._trial = trial
            self._target, self._reached, self._visited = None, None, []
            if self._anchor is not None:
                self._anchor, changed = None, True
        arg = status.get("phase_arg") if status else None
        if phase == "target":
            self._target = arg
        elif phase in ("reach", "reach_wrong"):
            self._reached = arg
        elif phase == "checkpoint" and arg is not None and int(arg) not in self._visited:
            self._visited.append(int(arg))
        if phase == "armed" and self._armed_trial != trial:
            ee = self._ee()
            if ee is not None:
                self._anchor, self._armed_trial, changed = ee, trial, True
        if changed or self._geometry is None:
            self._geometry = self._build() if key else None
            self._version += 1

    # ------------------------------------------------------------------ the bounds

    def _build(self) -> dict:
        p = self._params
        start, R, real = self._start()
        a = self.anchor()
        shapes: List[dict] = []
        add = shapes.append

        # Every experiment: the floor (for the flange, at the start's orientation), its trip line, the start.
        if "floor.height_m" in p:
            ref = str(p.get("floor.reference", "apple_bottom"))
            z = float(p["floor.height_m"]) - reference_drop(R, ref)
            add({"kind": "zline", "z": z, "role": "floor",
                 "label": f"floor ({ref.replace('_', ' ')} at {float(p['floor.height_m']):.3f} m)"})
            add({"kind": "zline", "z": z - float(p.get("floor.trip_margin_m", 0.03)), "role": "trip",
                 "label": "safety trip below the floor"})
        add({"kind": "marker", "p": start.tolist(), "r": 0.006, "role": "start", "label": "start"})
        d = _safety_distance(p)
        if d > 0:
            add({"kind": "sphere", "c": a.tolist(), "r": d, "role": "trip", "label": f"safety distance {d:.2f} m"})

        key = self._key
        if key == "apple_pluck":
            self._pluck(p, start, a, add)
        elif key == "restricted_plane":
            self._plane(p, a, add)
        elif key == "maze":
            self._maze(p, a, add)
        elif key in ("free_move", "move_vertical", "move_horizontal", "move_rails", "restricted_move_horizontal",
                     "restricted_move_vertical", "colour_rule", "n_choice", "reach_map", "move_free", "forage"):
            self._free_move(p, a, add)
        return {"experiment": key, "shapes": shapes, "anchor": a.tolist(), "start": start.tolist(),
                "model": real, "version": self._version + 1}

    def _pluck(self, p, start, a, add) -> None:
        pre = "apple_pluck_impedance_control_displacement."
        thr = float(p.get(pre + "cartesian_displacement_threshold_m", 0.1))
        axis = str(p.get(pre + "cartesian_axis", "norm"))
        self._threshold(thr, axis, a, add)
        paradigm = str(p.get("paradigm", "pluck"))
        if paradigm in ("perturb", "perturb_bump"):
            r = float(p.get("perturb_start.polar_r_m", 0.0))
            th = math.radians(float(p.get("perturb_start.polar_theta_deg", 0.0)))
            off = self._in_plane(str(p.get("perturb_start.polar_plane", "frontal")), r * math.cos(th), r * math.sin(th))
            add({"kind": "seg", "a": start.tolist(), "b": (start + off).tolist(), "role": "guide", "label": "perturbation"})
            add({"kind": "marker", "p": (start + off).tolist(), "r": 0.006, "role": "perturbed", "label": "perturbed start"})
        if paradigm == "random_start":
            plane = str(p.get("random_start.plane", "frontal"))
            prefix = "random_start.positions."
            for k, v in p.items():
                if k.startswith(prefix):
                    off = self._in_plane(plane, float(v[0]), float(v[1]))
                    add({"kind": "marker", "p": (start + off).tolist(), "r": 0.004, "role": "option",
                         "label": k[len(prefix):]})

    def _plane(self, p, a, add) -> None:
        pre = "apple_pluck_impedance_control_displacement."
        thr = float(p.get(pre + "cartesian_displacement_threshold_m", 0.12))
        axis = str(p.get(pre + "cartesian_axis", "z"))
        self._threshold(thr, axis, a, add)
        prof = str(p.get("virtual_fixture_profile", ""))
        vf = f"virtual_fixtures.{prof}."
        kind = str(p.get(vf + "type", ""))
        if kind == "sinusoid":
            ip, io = AXES[str(p[vf + "pull_axis"])], AXES[str(p[vf + "osc_axis"])]
            amp, k = float(p[vf + "amplitude"]), float(p[vf + "spatial_freq"])
            span = max(thr, 0.05) * 1.25
            pts = []
            for s in np.linspace(-span, span, 121):
                q = a.copy()
                q[ip] += s
                q[io] += amp * math.sin(k * s)
                pts.append(q.tolist())
            add({"kind": "poly", "pts": pts, "role": "rail", "label": f"sine rail ({prof})"})
        elif kind == "plane":
            i = AXES[str(p[vf + "restricted_axis"])]
            lo, hi = [-math.inf] * 3, [math.inf] * 3
            lo[i] = hi[i] = float(a[i])
            add({"kind": "box", "min": [None if math.isinf(v) else v for v in lo],
                 "max": [None if math.isinf(v) else v for v in hi], "role": "rail", "label": f"plane ({prof})"})
        elif kind == "cartesian_box":
            lo = [p.get(vf + "x_min"), None, p.get(vf + "z_min")]
            hi = [p.get(vf + "x_max"), None, None]
            add({"kind": "box", "min": lo, "max": hi, "role": "wall", "label": f"box ({prof})"})
        elif kind == "cylinder":
            add({"kind": "vcyl", "c": [float(p[vf + "center_x"]), float(p[vf + "center_y"])],
                 "r": float(p[vf + "radius"]), "role": "wall", "label": f"cylinder ({prof})"})

    def _maze(self, p, a, add) -> None:
        prof = str(p.get("virtual_fixture_profile", "maze"))
        vf = f"virtual_fixtures.{prof}."
        i = AXES.get(str(p.get(vf + "restricted_axis", "x")), 0)
        ia, ib = [j for j in range(3) if j != i]
        rel = str(p.get(vf + "corridor_frame", "relative")) == "relative"
        lists = [p.get(vf + k) or [] for k in ("corridor_a_min", "corridor_a_max", "corridor_b_min", "corridor_b_max")]
        for n, (a0, a1, b0, b1) in enumerate(zip(*lists)):
            lo, hi = a.copy(), a.copy()
            base = a if rel else np.zeros(3)
            lo[ia], hi[ia] = base[ia] + float(a0), base[ia] + float(a1)
            lo[ib], hi[ib] = base[ib] + float(b0), base[ib] + float(b1)
            if float(a0) == float(a1) or float(b0) == float(b1):
                add({"kind": "seg", "a": lo.tolist(), "b": hi.tolist(), "role": "rail", "label": f"rail {n + 1}"})
            else:
                add({"kind": "box", "min": lo.tolist(), "max": hi.tolist(), "role": "rail",
                     "label": f"corridor {n + 1}"})
        cm = "checkpoint_monitor."
        base = a if p.get(cm + "relative_to_start", True) else np.zeros(3)
        xs, ys, zs = (p.get(cm + f"checkpoint_{c}") or [] for c in "xyz")
        rs = p.get(cm + "checkpoint_radius") or []
        for n, (x, y, z) in enumerate(zip(xs, ys, zs)):
            r = float(rs[n]) if n < len(rs) else 0.02
            add({"kind": "marker", "p": (base + [float(x), float(y), float(z)]).tolist(), "r": r,
                 "role": "checkpoint", "id": f"checkpoint:{n}", "label": f"fork {n + 1}"})
        if cm + "goal_y" in p:
            g = base + [float(p.get(cm + "goal_x", 0.0)), float(p[cm + "goal_y"]), float(p[cm + "goal_z"])]
            add({"kind": "marker", "p": g.tolist(), "r": float(p.get(cm + "goal_radius", 0.015)),
                 "role": "goal", "id": "goal", "label": "goal"})

    def _free_move(self, p, a, add) -> None:
        axis = str(p.get("free_move.rail_axis", "none"))
        if axis == "rays":            # N-choice: the asterisk of rails, each end marked by its direction
            degs = layout_degs(p.get("free_move.ray_layout", "2"))
            length = float(p.get("free_move.ray_length_m", 0.07))
            for deg in degs:
                end = a + rail_dir(deg) * length
                add({"kind": "seg", "a": a.tolist(), "b": end.tolist(), "role": "rail", "label": f"{direction_name(deg)} rail"})
                add({"kind": "marker", "p": end.tolist(), "r": float(p.get("n_choice.end_tol_m", 0.015)), "role": "end",
                     "id": f"end:{direction_name(deg)}", "label": f"{direction_name(deg)} end"})
            return
        if "forage.n_zones" in p:     # Foraging: the arena and today's zones (the same seed as the run's)
            try:
                lo, hi = box_bounds(p.__getitem__, "free_move.box.")
            except (KeyError, ValueError):
                return
            shape = str(p.get("free_move.box.shape", "rectangle"))
            if shape == "ellipse":
                c, r = (lo + hi) / 2, (hi - lo) / 2
                ring = [(a + c + np.array([0.0, r[1] * math.cos(t), r[2] * math.sin(t)])).tolist()
                        for t in np.linspace(0.0, 2 * math.pi, 61)]
                add({"kind": "poly", "pts": ring, "role": "wall", "label": "arena (ellipse)"})
            else:
                add({"kind": "box", "min": (a + lo).tolist(), "max": (a + hi).tolist(), "role": "wall", "label": "arena"})
            try:
                s = forage.settings_from(p.__getitem__)
                seed = s.seed or forage.day_seed(str(p.get("_subject", "")))
                for i, z in enumerate(forage.zones_for(s, seed)):
                    add({"kind": "marker", "p": (a + z).tolist(), "r": s.zone_radius_m, "role": "end",
                         "id": f"zone:{i}", "label": f"zone {i + 1}"})
            except (KeyError, TypeError, ValueError):
                pass
            return
        if "free_reach.distance_m" in p:     # Cued Reaches, free (3-D or frontal): the box, and where a reach ends
            try:
                lo, hi = box_bounds(p.__getitem__, "free_move.box.")
                add({"kind": "box", "min": (a + lo).tolist(), "max": (a + hi).tolist(), "role": "wall", "label": "box"})
            except (KeyError, ValueError):
                pass
            d = float(p["free_reach.distance_m"])
            ring = [(a + np.array([0.0, d * math.cos(t), d * math.sin(t)])).tolist()
                    for t in np.linspace(0.0, 2 * math.pi, 49)]
            add({"kind": "poly", "pts": ring, "role": "rail", "label": f"a reach ends at {d * 100:.0f} cm"})
            for name in p.get("free_reach.directions") or []:
                if name in free_reach.DIRECTIONS:
                    end = a + free_reach.to_base(d * free_reach.unit(name))
                    add({"kind": "marker", "p": end.tolist(), "r": 0.01, "role": "end", "id": f"end:{name}",
                         "label": f"{name} target"})
            return
        if axis == "frontal":         # the colour rule, No rail: free in the plane, inside the box; a choice
            try:                      # is reaching free_reach_m left or right
                lo, hi = box_bounds(p.__getitem__, "free_move.box.")
                add({"kind": "box", "min": (a + lo).tolist(), "max": (a + hi).tolist(), "role": "wall", "label": "box"})
            except (KeyError, ValueError):
                pass
            d = float(p.get("centre_out.free_reach_m", 0.12))
            for name, x in (("left", -d), ("right", d)):
                add({"kind": "marker", "p": (a + np.array([0.0, x, 0.0])).tolist(), "r": 0.01, "role": "end",
                     "id": f"end:{name}", "label": f"{name}: a choice past here"})
            return
        if axis == "none":
            try:
                lo, hi = box_bounds(p.__getitem__, "free_move.box.")
            except (KeyError, ValueError):
                return
            add({"kind": "box", "min": (a + lo).tolist(), "max": (a + hi).tolist(), "role": "wall", "label": "box"})
            return
        got = switch_rails.effective(p)           # a fixed rail, or the switchable one in force (rails.axis)
        if got is None:
            return
        axis, lo, hi = got
        u = np.eye(3)[AXES[axis]]
        add({"kind": "seg", "a": (a + u * lo).tolist(), "b": (a + u * hi).tolist(), "role": "rail", "label": f"{axis} rail"})
        if p.get("centre_out.enabled"):
            tol = float(p.get("centre_out.end_tol_m", 0.02))
            names = END_NAMES.get(axis, ("min", "max"))
            for name, x in zip(names, (lo, hi)):
                add({"kind": "marker", "p": (a + u * x).tolist(), "r": tol, "role": "end", "id": f"end:{name}",
                     "label": f"{name} end"})

    def _threshold(self, thr: float, axis: str, a, add) -> None:
        if axis in AXES:
            i = AXES[axis]
            for sgn in (-1, 1):
                lo, hi = [None] * 3, [None] * 3
                lo[i] = hi[i] = float(a[i] + sgn * thr)
                add({"kind": "box", "min": lo, "max": hi, "role": "threshold", "label": f"threshold {thr:.2f} m along {axis}"})
        else:
            add({"kind": "sphere", "c": a.tolist(), "r": thr, "role": "threshold", "label": f"threshold {thr:.2f} m"})

    @staticmethod
    def _in_plane(plane: str, u: float, v: float) -> np.ndarray:
        ia, ib = PLANES.get(plane, (1, 2))
        off = np.zeros(3)
        off[ia], off[ib] = u, v
        return off

    # ------------------------------------------------------------------ out

    def geometry(self) -> Optional[dict]:
        return self._geometry

    def live(self, status: Optional[dict]) -> dict:
        ee = self._ee() if self._key else None
        pull = None
        lv = self._bridge.live() if hasattr(self._bridge, "live") else None
        if lv:
            if lv.get("pull_offset") is not None:
                pull = (self.anchor() + np.asarray(lv["pull_offset"], dtype=float)).tolist()
            elif lv.get("pull") is not None and time.time() - lv.get("pull_t", 0.0) < STALE_SEC:
                pull = lv["pull"]
        return {"experiment": self._key, "version": self._version, "t": time.time(),
                "ee": None if ee is None else ee.tolist(), "pull": pull,
                "trial": status.get("trial") if status else None,
                "phase": status.get("phase") if status else None,
                "phase_arg": status.get("phase_arg") if status else None,
                "target": self._target, "reached": self._reached, "visited": list(self._visited)}
