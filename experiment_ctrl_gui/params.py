"""Experiment YAML <-> the flat, typed parameter list the dashboard shows and edits.

ROS names a nested YAML parameter by joining the keys with dots (visual_cue.colours.play), so that
is the name used everywhere here -- the same one `ros2 param set` takes.

The package YAML is never written. Per-run edits go into a generated copy under runs/, which the
launch then loads through its `params_file` argument; the orchestrator's trial sidecar records the
full resolved parameter set either way.
"""
from __future__ import annotations

import copy
import json
import re
import time
from pathlib import Path
from typing import Dict, List, Tuple

import yaml

import experiments as ex

RUNS_DIR = Path(__file__).resolve().parent / "runs"

# A mapping key: a plain name, or a node key such as `/**/controller_manager`.
_KEY_LINE = re.compile(r"^(\s*)([A-Za-z_/*][\w/*]*):(.*)$")


def _params_root(doc: dict) -> dict:
    """The `ros__parameters` dict under the node-name key (`/**` in every experiment YAML)."""
    for node_block in doc.values():
        if isinstance(node_block, dict) and "ros__parameters" in node_block:
            return node_block["ros__parameters"]
    raise ValueError("no ros__parameters block")


def flatten(d: dict, prefix: str = "") -> Dict[str, object]:
    out: Dict[str, object] = {}
    for k, v in d.items():
        name = f"{prefix}{k}"
        if isinstance(v, dict):
            out.update(flatten(v, name + "."))
        else:
            out[name] = v
    return out


def yaml_docs(text: str) -> Tuple[Dict[str, str], Dict[str, str]]:
    """The YAML's own documentation, per dotted name (parameters AND blocks):

      description  the `# comment` on the key's line -- one line: what it is, with units
      notes        the comment lines directly above the key (no blank line between) -- the why,
                   measurements and knobs. A comment that heads a block therefore belongs to that block.

    Every parameter must have a description; check_param_docs.py enforces it."""
    stack: List[Tuple[int, str]] = []
    desc: Dict[str, str] = {}
    notes: Dict[str, str] = {}
    above: List[str] = []
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("#"):
            c = stripped[1:]
            above.append((c[1:] if c.startswith(" ") else c).rstrip())   # keep deeper indentation
            continue
        if not stripped:
            above = []
            continue
        m = _KEY_LINE.match(line)
        if not m:
            continue                   # a list item ("- 0.0") keeps any notes pending for nothing
        indent, key, rest = len(m.group(1)), m.group(2), m.group(3)
        while stack and stack[-1][0] >= indent:
            stack.pop()
        stack.append((indent, key))
        # Drop the `/**` node key and ros__parameters: what is left is the ROS parameter name.
        name = ".".join(k for _, k in stack if k not in ("ros__parameters", "/**"))
        hash_at = _comment_start(rest)
        if hash_at >= 0:
            desc[name] = rest[hash_at + 1:].strip()
        if above:
            notes[name] = _reflow(above)
        above = []
    return desc, notes


def _reflow(lines: List[str]) -> str:
    """Join a comment's wrapped prose lines into paragraphs; keep indented lines (tables, lists,
    diagrams) and lines starting a list or banner on lines of their own."""
    out: List[str] = []
    for ln in lines:
        structural = (not ln or ln[0].isspace() or ln[:1] in "-*>|+=#" or ln[:2].rstrip(".):").isdigit()
                      or ln.startswith(("R1", "a:", "b ")))
        prev_structural = not out or not out[-1] or out[-1][0].isspace()
        if out and not structural and not prev_structural:
            out[-1] += " " + ln
        else:
            out.append(ln)
    return "\n".join(out).strip()


def _comment_start(rest: str) -> int:
    """Index of a `#` that starts a comment (not one inside quotes), else -1."""
    quote = None
    for i, c in enumerate(rest):
        if quote:
            if c == quote:
                quote = None
        elif c in "\"'":
            quote = c
        elif c == "#" and (i == 0 or rest[i - 1].isspace()):
            return i
    return -1


def type_name(v) -> str:
    if isinstance(v, bool):
        return "bool"
    if isinstance(v, int):
        return "int"
    if isinstance(v, float):
        return "float"
    if isinstance(v, str):
        return "str"
    if isinstance(v, list):
        if v and all(isinstance(x, bool) for x in v):
            return "bool[]"
        if v and all(isinstance(x, int) and not isinstance(x, bool) for x in v):
            return "int[]"
        if all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in v):
            return "float[]"
        return "str[]"
    return "str"


def coerce(value, like):
    """Convert a value from the browser (often a string) to the type of `like`, the YAML default.
    ROS refuses a set that changes a parameter's type, so this has to be exact."""
    t = type_name(like)
    if isinstance(value, str) and t not in ("str",):
        value = value.strip()
        if t.endswith("[]"):
            value = yaml.safe_load(value if value.startswith("[") else f"[{value}]")
        elif t == "bool":
            if value.lower() not in ("true", "false"):
                raise ValueError(f"expected true or false, got {value!r}")
            value = value.lower() == "true"
        else:
            value = yaml.safe_load(value)
    if t == "bool":
        if not isinstance(value, bool):
            raise ValueError(f"expected true or false, got {value!r}")
        return value
    if t == "int":
        if isinstance(value, float) and not value.is_integer():
            raise ValueError(f"expected an integer, got {value}")
        return int(value)
    if t == "float":
        return float(value)
    if t == "int[]":
        return [int(v) for v in value]
    if t == "float[]":
        return [float(v) for v in value]
    if t == "bool[]":
        return [bool(v) for v in value]
    if t == "str[]":
        return [str(v) for v in value]
    return str(value)


class ExperimentParams:
    """One experiment's YAML: its defaults, the per-run edits made in the dashboard, and the
    classification of every parameter."""

    def __init__(self, exp: ex.Experiment) -> None:
        self.exp = exp
        self.path = ex.CONFIG / exp.params_yaml
        self.edits: Dict[str, object] = {}
        self.reload()

    def reload(self) -> None:
        text = self.path.read_text()
        self.doc = yaml.safe_load(text)
        self.defaults = flatten(_params_root(self.doc))
        self.desc, self.notes = yaml_docs(text)

    def current(self) -> Dict[str, object]:
        out = dict(self.defaults)
        out.update(self.edits)
        return out

    def describe(self) -> List[dict]:
        cur = self.current()
        rows = []
        for name, default in self.defaults.items():
            lim = ex.live_params.limits_for(name)
            rows.append({
                "name": name,
                "group": name.split(".")[0] if "." in name else "general",
                "tier": ex.tier(self.exp, name),
                "type": type_name(default),
                "default": default,
                "value": cur[name],
                "edited": name in self.edits,
                "help": self.desc.get(name, ""),
                "notes": self._notes_for(name),
                "caution": ex.caution(name),
                "linked": [n for n in ex.linked(name) if n != name and n in self.defaults]
                          + (["kuka_clik_controller.nullspace_desired_configuration"]
                             if name in ex.linked(ex.START_POSE) and self.exp.clik_nullspace else []),
                "choices": ex.choices(self.exp, name, self.defaults),
                "limits": list(lim) if lim else None,
            })
        return rows

    def _notes_for(self, name: str) -> str:
        """The key's own notes; failing that, a sibling's notes that discuss it by name. One comment
        block often explains several keys (the perturbation's r / theta / plane) but sits above only
        the first of them."""
        if self.notes.get(name):
            return self.notes[name]
        parent, _, leaf = name.rpartition(".")
        for other, text in self.notes.items():
            if other.rpartition(".")[0] == parent and re.search(rf"\b{re.escape(leaf)}\b", text):
                return text
        return ""

    def groups(self) -> Dict[str, dict]:
        """Description and notes for each block (the group headings in the dashboard)."""
        out = {}
        for name in {n.rsplit(".", 1)[0] for n in self.defaults if "." in n}:
            parts = name.split(".")
            for i in range(1, len(parts) + 1):
                g = ".".join(parts[:i])
                out.setdefault(g, {"help": self.desc.get(g, ""), "notes": self.notes.get(g, "")})
        return out

    def set_edit(self, name: str, raw) -> object:
        """Edit one parameter for the next Start (and every parameter linked to it)."""
        if name not in self.defaults:
            raise KeyError(f"unknown parameter {name}")
        default = self.defaults[name]
        value = coerce(raw, default)
        problem = ex.live_params.check_value(name, value) or ex.check_extra(name, value)
        if problem is None:
            ch = ex.choices(self.exp, name, self.defaults)
            if ch is not None and str(value) not in ch:
                problem = f"{name} must be one of {ch}"
        if problem is None and isinstance(default, list) and len(value) != len(default) and (
                ".corridor_" in name or ".checkpoint_" in name):
            # Rails and checkpoints are parallel arrays: one entry per rail / checkpoint across
            # several parameters. Changing one array's length alone breaks the set.
            problem = (f"{name} must keep {len(default)} entries -- it is one of several parallel "
                       f"arrays. To add or remove a rail or checkpoint, edit the YAML.")
        if problem:
            raise ValueError(problem)
        for n in ex.linked(name):
            if n not in self.defaults:
                continue
            if value == self.defaults[n]:
                self.edits.pop(n, None)
            else:
                self.edits[n] = value
        return value

    def start_pose(self):
        return self.current().get(ex.START_POSE)

    def clik_posture_edited(self) -> bool:
        return bool(self.exp.clik_nullspace) and ex.START_POSE in self.edits

    def reset(self) -> None:
        self.edits.clear()

    def launch_overrides(self) -> Dict[str, str]:
        """Launch arguments for this Start: the edited YAML, and -- when a CLIK experiment's start
        pose was edited -- a CLIK posture file that matches it. Empty when nothing was edited."""
        out: Dict[str, str] = {}
        if not self.edits:
            return out
        out["params_file"] = str(self.write_run_file())
        if self.clik_posture_edited():
            out["clik_nullspace_cfg"] = str(self._write_clik_posture())
        return out

    def _write_clik_posture(self) -> Path:
        # The CLIK resolves the arm's redundant DOF toward this posture, so it must equal the start
        # pose (see the notes in config/clik_nullspace_*.yaml). iiwa7_hardware.launch.py joins it
        # onto the package directory; an absolute path replaces that, so this file is used as is.
        RUNS_DIR.mkdir(exist_ok=True)
        out = RUNS_DIR / f"{self.exp.key}_{time.strftime('%Y%m%d_%H%M%S')}_clik_nullspace.yaml"
        doc = {"/**/kuka_clik_controller": {"ros__parameters": {
            "nullspace_desired_configuration": [float(v) for v in self.start_pose()]}}}
        out.write_text(f"# Generated by experiment_ctrl_gui: follows the edited "
                       f"{ex.START_POSE}, replacing sinthlab_bringup/config/{self.exp.clik_nullspace}\n"
                       + yaml.safe_dump(doc, default_flow_style=None))
        return out

    def write_run_file(self) -> Path:
        """The YAML with the edits applied, for this launch only. Returns its path."""
        doc = copy.deepcopy(self.doc)
        root = _params_root(doc)
        for name, value in self.edits.items():
            node = root
            *parents, leaf = name.split(".")
            for p in parents:
                node = node[p]
            node[leaf] = value
        RUNS_DIR.mkdir(exist_ok=True)
        out = RUNS_DIR / f"{self.exp.key}_{time.strftime('%Y%m%d_%H%M%S')}.yaml"
        header = (f"# Generated by experiment_ctrl_gui from sinthlab_bringup/config/{self.exp.params_yaml}\n"
                  f"# Per-run edits: {json.dumps(self.edits)}\n")
        out.write_text(header + yaml.safe_dump(doc, sort_keys=False, default_flow_style=None))
        return out


SMARTPAD_HELP = {
    "SmartPad application": "the Sunrise app that runs the Cartesian impedance spring in the cabinet at 1 kHz",
    "FRI send period [ms]": "how often the cabinet and ROS exchange FRI messages",
    "Remote IP address": "where the cabinet sends FRI -- this ROS computer",
    "Damping ratio (D0)": "Cartesian damping ratio of the cabinet spring; 0.7 = standard",
    "Cartesian stiffness (K diagonal)": "stiffness profile {X, Y, Z, A, B, C} chosen at the SmartPad for this experiment",
}
LAUNCH_HELP = {
    "robot_type": "robot model used for the description (URDF)",
    "robot_name": "ROS namespace of the arm: topics are /<robot_name>/...",
    "startup_delay": "seconds to hold the orchestrator back so the controllers are active first",
    "orchestrator": "the experiment's state machine node",
    "run_name": "names the recording folder: analysis/expt_<run_name>_<time>/",
    "ctrl": "controller spawned ACTIVE (moves to start and recover)",
    "extra_inactive_ctrl": "second controller spawned INACTIVE; the orchestrator switches to it for the fixture",
    "clik_nullspace_cfg": "CLIK redundancy posture file (must equal the start pose)",
    "params_file": "the experiment parameter YAML loaded onto the orchestrator",
}


def _documented_rows(path: Path, block: str, params: dict) -> List[dict]:
    desc, notes = yaml_docs(path.read_text())
    return [{"name": k, "value": v, "help": desc.get(f"{block}.{k}", ""), "notes": notes.get(f"{block}.{k}", "")}
            for k, v in flatten(params).items()]


def read_fixed_config(exp: ex.Experiment, p: "ExperimentParams") -> List[dict]:
    """Hardware, FRI and controller configuration: read-only in the dashboard."""
    sections = [{
        "title": "SmartPad (FRI) — select these when starting the app",
        "source": "KUKA SmartPad dialogs",
        "rows": [{"name": k, "value": v, "help": SMARTPAD_HELP.get(k, "")} for k, v in exp.smartpad().items()],
    }, {
        "title": "Launch arguments",
        "source": f"sinthlab_bringup/launch/{exp.launch_file} → experiment_base.launch.py",
        "rows": [{"name": k, "value": v, "help": LAUNCH_HELP.get(k, "")} for k, v in
                 {**ex.COMMON_LAUNCH_ARGS, "orchestrator": exp.node + ".py",
                  "run_name": exp.run_name or "(none — no TrialRecorder)", **exp.launch_args,
                  "params_file": (f"edited copy of config/{exp.params_yaml}, written to runs/ at Start"
                                  if p.edits else f"config/{exp.params_yaml}"),
                  **({"clik_nullspace_cfg": "generated in runs/ at Start (follows the edited start pose)"}
                     if p.clik_posture_edited() else {})}.items()],
    }]
    hw_path = ex.CONFIG / "iiwa7_hardware_controllers.yaml"
    hw = yaml.safe_load(hw_path.read_text())
    for block, body in hw.items():
        params = body.get("ros__parameters", {}) if isinstance(body, dict) else {}
        name = block.split("/")[-1]
        if exp.ros_controller != "kuka_clik_controller" and name == "kuka_clik_controller":
            continue
        rows = _documented_rows(hw_path, block, params)
        if rows:
            sections.append({"title": f"{name}", "source": "sinthlab_bringup/config/iiwa7_hardware_controllers.yaml",
                             "rows": rows})
    if exp.clik_nullspace:
        ns = yaml.safe_load((ex.CONFIG / exp.clik_nullspace).read_text())
        for block, body in ns.items():
            rows = _documented_rows(ex.CONFIG / exp.clik_nullspace, block, body["ros__parameters"])
            source = f"sinthlab_bringup/config/{exp.clik_nullspace}"
            if p.clik_posture_edited():
                rows = [{**rows[0], "value": p.start_pose()}]
                source += " — overridden: follows the edited move_to_start pose"
            sections.append({"title": "kuka_clik_controller — redundancy posture",
                             "source": source, "rows": rows})
    return sections
