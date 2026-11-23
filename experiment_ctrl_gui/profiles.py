"""Monkey profiles: the settings that belong to the animal, not to the task, applied to every task it runs.

Three layers of settings, from the bottom up:

  rig      the hardware and plumbing (cabinet outputs, cue board, controllers, floor, safety, park pose):
           the experiment YAMLs, as before.
  profile  how the arm fits this monkey: the FEEL of the admittance (damping, threshold, speed, ...), its
           REACH in each direction, the PACING (quiet window, its guard, the response time) and the REWARD.
           One file per subject; applied to every free-move task, so the feel tuned in Free Move is the feel
           of the Colour Rule.
  today    the task's own few settings (Experiment.task_settings) and any one-off edit, made on the dashboard
           for the next Start. A one-off edit wins over the profile, and is not saved to it unless asked.

A profile is a YAML file per subject, <data dir>/subjects/<ID>.yaml (next to the recordings, not in git),
with every change appended to <ID>.changes.jsonl. Each save bumps its version; the run's parameter file
names the profile and version it was made from, and the recording folder gets a copy.

Most profile fields map one to one onto a task parameter of the same meaning (FIELDS). REACH sets the ends of
a horizontal rail (left, right) or a vertical one (down, up): Restricted Move, Cued Reaches, the Colour Rule's
Rail. A value beyond a task's limits is capped there, with a note; the dashboard's and the node's own checks
still apply on top. The choice distances that are part of a task's design stay task settings -- the Colour
Rule's No-rail reach, the N-Choice rail length (kept the same as the number of choices grows) -- and Free
Move's box too; the dashboard notes where one goes past the monkey's reach.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

import experiments as ex
from sinthlab_bringup.helpers.rail_shapes import layout_degs


@dataclass(frozen=True)
class Field:
    name: str              # section.key, as stored in the profile file
    label: str
    param: str             # the task parameter it sets ("" for reach: translated per task)
    help: str
    reward: bool = False   # may be overridden for one task (reward_by_task)


SECTIONS = [("feel", "Feel", "How heavy and responsive the arm is: tune it in Free Move or Restricted Move, then save."),
            ("reach", "Reach", "How far this monkey reaches from the start, in each direction. It sets the ends of the "
                               "rails: left and right on a horizontal rail, down and up on a vertical one."),
            ("pacing", "Pacing", "The wait before the go cue, its guard against early touches, and the time to respond."),
            ("reward", "Reward", "The juice per reward. A task can have its own (Save to profile: this task only).")]

FIELDS: List[Field] = [
    Field("feel.damping_ns_per_m", "Damping", "free_move.damping_ns_per_m",
          "The arm's resistance to motion [N·s/m]: higher = heavier. Below ~40 it rings."),
    Field("feel.deadband_n", "Force threshold", "free_move.deadband_n",
          "Push needed before the arm moves at all [N]."),
    Field("feel.max_speed_mps", "Top speed", "free_move.max_speed_mps", "The fastest the arm will go [m/s]."),
    Field("feel.accel_limit_mps2", "Acceleration limit", "free_move.accel_limit_mps2",
          "How quickly it may speed up or slow down [m/s²]; 0 = no limit."),
    Field("feel.low_force_n", "Light-push range", "free_move.low_force_n",
          "Pushes below this get the light-push damping [N]: steadier when barely touched."),
    Field("feel.low_force_damping_ns_per_m", "Light-push damping", "free_move.low_force_damping_ns_per_m",
          "Damping for light pushes [N·s/m]."),
    Field("reach.left_m", "Left", "", "From the start to the left [m] (−Y)."),
    Field("reach.right_m", "Right", "", "From the start to the right [m] (+Y)."),
    Field("reach.down_m", "Down", "", "From the start downward [m]."),
    Field("reach.up_m", "Up", "", "From the start upward [m] (the arm allows little: ~7 cm from the vertical tasks' start)."),
    Field("pacing.quiet_window_sec", "Quiet window", "quiet_window_sec", "The wait at the start before the go cue [s]."),
    Field("pacing.quiet_gate_enabled", "Guard the quiet window", "quiet_gate.enabled",
          "Touching the arm during the wait: no reward, park, time-out."),
    Field("pacing.quiet_move_m", "Guard: movement", "quiet_gate.move_m", "Movement during the wait that counts as a touch [m]."),
    Field("pacing.quiet_force_n", "Guard: force", "quiet_gate.force_n", "Force during the wait that counts as a touch [N]."),
    Field("pacing.quiet_penalty_sec", "Guard: time-out", "quiet_gate.penalty_sec", "Time-out after an early touch [s]."),
    Field("pacing.response_timeout_sec", "Time to respond", "response_timeout_sec",
          "From the go cue: no choice (or movement) in this time ends the trial unrewarded [s]."),
    Field("reward.pulse_ms", "Valve open", "reward_pulse.pulse_ms", "Each pulse's valve time [ms].", reward=True),
    Field("reward.count", "Pulses", "reward_pulse.count", "Pulses per reward.", reward=True),
    Field("reward.gap_ms", "Gap between pulses", "reward_pulse.gap_ms", "Valve closed between pulses [ms].", reward=True),
]
BY_NAME = {f.name: f for f in FIELDS}
BY_PARAM = {f.param: f for f in FIELDS if f.param}
REACH = ("reach.left_m", "reach.right_m", "reach.down_m", "reach.up_m")
SUBJECT_TEST = "test"           # runs without an animal: never a profile, always the YAML


def applies_to(exp: ex.Experiment) -> bool:
    """Profiles apply to the free-move tasks (one node, one feel). Navin's experiments keep their YAMLs."""
    return exp.node == "free_move_orchestrator"


# ---------------------------------------------------------------- storage
def folder() -> Path:
    return ex.DATA / "subjects"


def path_for(subject: str) -> Path:
    return folder() / f"{subject}.yaml"


def names() -> List[str]:
    d = folder()
    return sorted(p.stem for p in d.glob("*.yaml")) if d.is_dir() else []


def defaults() -> Dict[str, object]:
    """A new profile: the free-move YAML defaults (the same in every free-move task), the rails' reach."""
    from params import ExperimentParams
    fm = ExperimentParams(ex.BY_KEY["free_move"]).defaults
    h = ExperimentParams(ex.BY_KEY["move_horizontal"]).defaults
    v = ExperimentParams(ex.BY_KEY["move_vertical"]).defaults
    out = {f.name: fm[f.param] for f in FIELDS if f.param}
    out.update({"reach.left_m": -h["free_move.rail_min_m"], "reach.right_m": h["free_move.rail_max_m"],
                "reach.down_m": -v["free_move.rail_min_m"], "reach.up_m": v["free_move.rail_max_m"]})
    return out


def _nest(flat: Dict[str, object]) -> dict:
    out: dict = {}
    for name, v in flat.items():
        sec, key = name.split(".", 1)
        out.setdefault(sec, {})[key] = v
    return out


def load(subject: str) -> Optional[dict]:
    """{subject, version, updated, values: {field: value}, reward_by_task: {task: {field: value}}}, or None."""
    p = path_for(subject)
    if not p.is_file():
        return None
    doc = yaml.safe_load(p.read_text()) or {}
    values = {}
    for sec, _, _ in SECTIONS:
        for k, v in (doc.get(sec) or {}).items():
            if f"{sec}.{k}" in BY_NAME:
                values[f"{sec}.{k}"] = v
    base = defaults()
    missing = [n for n in base if n not in values]
    for n in missing:               # a field added after the profile was written: the YAML default
        values[n] = base[n]
    by_task = {t: {f"reward.{k}": v for k, v in (d or {}).items() if f"reward.{k}" in BY_NAME}
               for t, d in (doc.get("reward_by_task") or {}).items()}
    return {"subject": subject, "version": int(doc.get("version", 0)), "updated": str(doc.get("updated", "")),
            "values": values, "reward_by_task": by_task, "missing": missing}


def check(values: Dict[str, object]) -> Optional[str]:
    """None if every value is within its parameter's limits, else the first problem."""
    for name, v in values.items():
        f = BY_NAME.get(name)
        if f is None:
            return f"unknown profile setting {name}"
        if name in REACH:
            if not isinstance(v, (int, float)) or isinstance(v, bool) or not 0.01 <= float(v) <= 0.30:
                return f"{name} must be between 0.01 and 0.30 m, got {v!r}"
            continue
        problem = ex.live_params.check_value(f.param, v)
        if problem:
            return problem
    return None


def coerce_field(name: str, raw, like) -> object:
    from params import coerce
    return coerce(raw, like)


def save(subject: str, values: Dict[str, object], by: str, by_task: Optional[Dict[str, Dict[str, object]]] = None,
         note: str = "") -> dict:
    """Write the profile (values merged over the current one), bump its version, log the change."""
    cur = load(subject)
    old_vals = dict(cur["values"]) if cur else defaults()
    old_by_task = {t: dict(d) for t, d in (cur["reward_by_task"] if cur else {}).items()}
    base = defaults()
    new_vals = dict(old_vals)
    for name, raw in values.items():
        if name not in BY_NAME:
            raise KeyError(f"unknown profile setting {name}")
        new_vals[name] = coerce_field(name, raw, base[name])
    problem = check(new_vals)
    if problem:
        raise ValueError(problem)
    new_by_task = {t: dict(d) for t, d in old_by_task.items()}
    for task, d in (by_task or {}).items():
        if task not in ex.BY_KEY:
            raise KeyError(f"unknown task {task}")
        for name, raw in d.items():
            if name not in BY_NAME or not BY_NAME[name].reward:
                raise ValueError(f"only the reward can be set for one task, not {name}")
            if raw is None:
                new_by_task.get(task, {}).pop(name, None)
            else:
                v = coerce_field(name, raw, base[name])
                problem = check({name: v})
                if problem:
                    raise ValueError(problem)
                new_by_task.setdefault(task, {})[name] = v
    new_by_task = {t: d for t, d in new_by_task.items() if d}
    changes = {n: [old_vals.get(n), v] for n, v in new_vals.items() if old_vals.get(n) != v}
    task_changes = {t: {n: [old_by_task.get(t, {}).get(n), new_by_task.get(t, {}).get(n)]
                        for n in set(old_by_task.get(t, {})) | set(new_by_task.get(t, {}))
                        if old_by_task.get(t, {}).get(n) != new_by_task.get(t, {}).get(n)}
                    for t in set(old_by_task) | set(new_by_task)}
    task_changes = {t: c for t, c in task_changes.items() if c}
    if cur and not changes and not task_changes:
        return cur
    version = (cur["version"] if cur else 0) + 1
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    doc = {"subject": subject, "version": version, "updated": stamp, **_nest(new_vals)}
    if new_by_task:
        doc["reward_by_task"] = {t: {n.split(".", 1)[1]: v for n, v in d.items()} for t, d in new_by_task.items()}
    folder().mkdir(parents=True, exist_ok=True)
    head = (f"# {subject}'s profile: the settings that belong to the monkey, applied to every free-move task.\n"
            f"# Written by the dashboard (experiment_ctrl_gui/profiles.py); every change is logged in "
            f"{subject}.changes.jsonl.\n")
    tmp = path_for(subject).with_suffix(".yaml.tmp")
    tmp.write_text(head + yaml.safe_dump(doc, sort_keys=False, default_flow_style=None))
    tmp.replace(path_for(subject))
    with open(folder() / f"{subject}.changes.jsonl", "a") as fh:
        fh.write(json.dumps({"time": stamp, "version": version, "by": by, "note": note,
                             "changes": changes, "reward_by_task": task_changes}) + "\n")
    return load(subject)


def changes(subject: str, limit: int = 50) -> List[dict]:
    p = folder() / f"{subject}.changes.jsonl"
    if not p.is_file():
        return []
    rows = [json.loads(line) for line in p.read_text().splitlines() if line.strip()]
    return rows[-limit:][::-1]


# ---------------------------------------------------------------- profile -> task
def _ray_reach(deg: float, left: float, right: float, down: float, up: float) -> float:
    """The reach along a direction in the frontal plane (0 = right, 90 = up): the quarter-ellipse through
    the reaches either side of it."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    a = right if c >= 0 else left
    b = up if s >= 0 else down
    return 1.0 / math.sqrt((c / a) ** 2 + (s / b) ** 2)


def reach_values(exp: ex.Experiment, reach: Dict[str, float], context: Dict[str, object]) -> Dict[str, float]:
    """The rail ends the reach sets, given the task's other values in effect (context): a vertical rail (z)
    runs from down to up, any other (the horizontal rail; the Colour Rule, whose Rail is horizontal) from
    left to right."""
    if "free_move.rail_min_m" not in context:
        return {}
    vertical = str(context.get("free_move.rail_axis")) == "z"
    lo, hi = (REACH[2], REACH[3]) if vertical else (REACH[0], REACH[1])
    return {"free_move.rail_min_m": -float(reach[lo]), "free_move.rail_max_m": float(reach[hi])}


def reach_notes(reach: Dict[str, float], context: Dict[str, object]) -> Dict[str, str]:
    """Where a task's own choice distance goes past the monkey's reach (a note, not a change)."""
    left, right, down, up = (float(reach[n]) for n in REACH)
    notes = {}
    r = context.get("centre_out.free_reach_m")
    if isinstance(r, (int, float)) and r > min(left, right) + 1e-9:
        notes["centre_out.free_reach_m"] = f"past this monkey's reach (left {left:g} m, right {right:g} m)"
    length = context.get("free_move.ray_length_m")
    if isinstance(length, (int, float)):
        try:
            degs = layout_degs(context.get("free_move.ray_layout"))
        except ValueError:
            degs = []
        short = [(d, _ray_reach(d, left, right, down, up)) for d in degs]
        short = [(d, r) for d, r in short if length > r + 1e-9]
        if short:
            from sinthlab_bringup.helpers.rail_shapes import direction_name
            notes["free_move.ray_length_m"] = "past this monkey's reach toward " + ", ".join(
                f"{direction_name(d)} ({r:.3f} m)" for d, r in short)
    return notes


def mapped(exp: ex.Experiment, context: Dict[str, object]) -> Dict[str, str]:
    """task parameter -> the profile field it comes from, in this task (with or without a profile loaded)."""
    if not applies_to(exp):
        return {}
    out = {f.param: f.name for f in FIELDS if f.param and f.param in context}
    if "free_move.rail_min_m" in context:
        vertical = str(context.get("free_move.rail_axis")) == "z"
        out["free_move.rail_min_m"] = REACH[2] if vertical else REACH[0]
        out["free_move.rail_max_m"] = REACH[3] if vertical else REACH[1]
    return out


def overlay(profile: Optional[dict], exp: ex.Experiment, context: Dict[str, object]
            ) -> Tuple[Dict[str, object], Dict[str, str], Dict[str, str]]:
    """What the profile sets in this task: (values, notes, sources).

    values   task parameter -> value (only parameters the task has)
    notes    task parameter -> why it differs from the profile (capped by the task's limits)
    sources  task parameter -> the profile field(s) it comes from, for the dashboard"""
    if not profile or not applies_to(exp):
        return {}, {}, mapped(exp, context)
    vals = dict(profile["values"])
    vals.update(profile.get("reward_by_task", {}).get(exp.key, {}))
    out: Dict[str, object] = {}
    notes: Dict[str, str] = {}
    for f in FIELDS:
        if f.param and f.param in context:
            out[f.param] = vals[f.name]
            if f.reward and f.name in profile.get("reward_by_task", {}).get(exp.key, {}):
                notes[f.param] = f"{exp.label} only (the profile's own reward is {profile['values'][f.name]})"
    ctx = {**context, **out}
    for name, v in reach_values(exp, vals, ctx).items():
        lim = ex.live_params.limits_for(name)
        capped = v
        if lim:
            capped = min(max(v, lim[0]), lim[1])
        capped = round(capped, 4)
        if abs(capped - v) > 1e-9:
            notes[name] = f"capped by the task's limit ({lim[0]} to {lim[1]} m); the reach gives {v:.3f}"
        out[name] = capped
    notes.update(reach_notes(vals, ctx))
    return out, notes, mapped(exp, context)


# ---------------------------------------------------------------- task -> profile (Save to profile)
def from_task(exp: ex.Experiment, values: Dict[str, object]) -> Dict[str, object]:
    """The profile fields a task's values imply: one-to-one fields, and the reach from a rail's ends."""
    out = {f.name: values[f.param] for f in FIELDS if f.param and f.param in values}
    axis = str(values.get("free_move.rail_axis", ""))
    if "free_move.rail_min_m" in values and axis in ("y", "z"):       # not the Colour Rule's No rail
        lo, hi = (REACH[0], REACH[1]) if axis == "y" else (REACH[2], REACH[3])
        out[lo] = round(-float(values["free_move.rail_min_m"]), 4)
        out[hi] = round(float(values["free_move.rail_max_m"]), 4)
    return out


def diff(profile: Optional[dict], exp: ex.Experiment, values: Dict[str, object]) -> List[dict]:
    """The profile fields where the task's values in effect differ from the profile: for Save to profile."""
    if not applies_to(exp):
        return []
    have = dict(profile["values"]) if profile else defaults()
    task_reward = (profile or {}).get("reward_by_task", {}).get(exp.key, {})
    rows = []
    for name, v in from_task(exp, values).items():
        f = BY_NAME[name]
        cur = task_reward.get(name, have[name])
        if _differs(v, cur):
            rows.append({"field": name, "label": f"{SECTION_LABEL[name.split('.')[0]]}: {f.label}",
                         "profile": cur, "task": v, "reward": f.reward,
                         "task_only_now": name in task_reward})
    return rows


def _differs(a, b) -> bool:
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
        return abs(float(a) - float(b)) > 1e-9
    return a != b


SECTION_LABEL = {k: lab for k, lab, _ in SECTIONS}


def describe() -> dict:
    """The profile's layout for the dashboard."""
    return {"sections": [{"key": k, "label": lab, "help": h,
                          "fields": [{"name": f.name, "label": f.label, "param": f.param, "help": f.help,
                                      "reward": f.reward, "limits": (list(ex.live_params.limits_for(f.param) or [])
                                                                     if f.param else [0.01, 0.30])}
                                     for f in FIELDS if f.name.startswith(k + ".")]}
                         for k, lab, h in SECTIONS]}
