"""Behavioural outcome tables: plain CSVs of what happened, beside the robot recordings.

The robot recording (one CSV of samples + one JSON sidecar per trial) holds everything, but not in a
form you can open and read. These tables are views of it:

    <run folder>/trials.csv       one row per trial: the COMMON columns below, then the task's own
    <run folder>/<unit>.csv       optional: one row per unit, only for a task whose trial holds many of
                                  something (Long moves: movements.csv). Trial-based tasks don't need it.
    <data folder>/sessions.csv    one row per run (a Start-to-Stop session), across every run: the
                                  history "today vs recent sessions" reads

where the data folder is analysis/ (data_dir() below) and a run folder is named YYYYMMDD_subject_task_HHMMSS.

THE SIDECAR IS THE RECORD; THESE ARE VIEWS. Every row is computed from the sidecars by the functions
here, so a table can be rebuilt at any time, for any run, including runs recorded before it existed:

    python3 analysis/outcome_tables.py                 # every run folder: trials.csv, unit tables, sessions.csv
    python3 analysis/outcome_tables.py <run folder>    # one run

Live, TrialRecorder.stop_and_save() appends each trial's rows as the trial ends (so a crash loses at most
the trial under way), and the dashboard adds the run's sessions.csv row when the run ends.

No ROS here: the recorder, the dashboard and the analysis scripts all import it.

ADDING A TASK'S TABLES. Subclass TaskTables and register it for the experiment keys it covers (see
CentreOutTables and LongMoveTables below for real examples):

    @register("my_task")
    class MyTaskTables(TaskTables):
        paradigm_param = "my_task.mode"                          # the card's paradigm parameter, if any
        trial_columns = {"cue": "the cue colour shown at the go cue",
                         "choice": "the end reached"}
        settings = ["my_task.threshold_m"]                       # key settings, for sessions.csv
        session_columns = {"pct_correct": "correct choices over all choices [%]"}

        def trial_fields(self, tr):                              # tr: a Trial (below)
            return {"cue": tr.arg("cue_on"), "choice": tr.arg("choice")}

        def session_fields(self, trials):                        # trials: the run's trials.csv rows
            done = [r for r in trials if r["correct"] in ("0", "1")]
            return {"pct_correct": round(100 * sum(r["correct"] == "1" for r in done) / len(done), 1)
                                   if done else ""}

Every method is optional; an experiment with no registered class gets the COMMON columns only. Column
names are the dict keys; the descriptions go into tables.md beside the tables (write_column_docs). The
trial's events (with their arguments) and its parameters are all a Trial offers, so anything a task
wants in its table must be marked as an event or be a parameter: mark it in the orchestrator
(recorder.mark("choice", end)) and read it here (tr.arg("choice")). A task whose single trial holds many
units (rare) can also return extra per-unit tables: unit_tables + units(), as LongMoveTables does.
"""
from __future__ import annotations

import csv
import datetime as dt
import glob
import json
import os
import re
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

# Where every run folder goes: the recorder's save folder. One place for the recorder and the dashboard,
# so a dashboard started from a test checkout (~/lbr-test) still finds the runs.
DEFAULT_DATA_DIR = "~/lbr-stack/src/sinthlab-kuka-stack/analysis"
DATA_DIR_ENV = "SINTHLAB_DATA_DIR"


def data_dir() -> Path:
    """The data folder: $SINTHLAB_DATA_DIR if set, else ~/lbr-stack/src/sinthlab-kuka-stack/analysis."""
    return Path(os.environ.get(DATA_DIR_ENV) or DEFAULT_DATA_DIR).expanduser()


_RUN_FOLDER = re.compile(r"[0-9]{8}_([A-Za-z0-9_-]+)_([a-z0-9-]+)_[0-9]{6}(?:-[0-9]+)?")


def task_slug(experiment: str) -> str:
    """The task field of the folder name. Hyphens keep it distinct from an ID with underscores."""
    return experiment.lower().replace("colour", "color").replace("_", "-")


def run_folder_name(subject: Optional[str], experiment: str, started: dt.datetime) -> str:
    subject_id = re.sub(r"[^A-Za-z0-9_-]", "-", subject or "unknown")
    return f"{started:%Y%m%d}_{subject_id}_{task_slug(experiment)}_{started:%H%M%S}"


def create_run_folder(parent: Path, subject: Optional[str], experiment: str, started: dt.datetime) -> Path:
    """Reserve a folder for this launch, including when launches share the same second."""
    name = run_folder_name(subject, experiment, started)
    parent.mkdir(parents=True, exist_ok=True)
    number = 1
    while True:
        folder = parent / (name if number == 1 else f"{name}-{number:02d}")
        try:
            folder.mkdir()
        except FileExistsError:
            number += 1
        else:
            return folder


def is_run_folder(folder: Path, experiment: Optional[str] = None, subject: Optional[str] = None) -> bool:
    match = _RUN_FOLDER.fullmatch(folder.name)
    return bool(match and (experiment is None or match[2] == task_slug(experiment))
                and (subject is None or match[1] == subject) and folder.is_dir())


def run_folders(parent: Path, experiment: Optional[str] = None, subject: Optional[str] = None) -> List[Path]:
    if not parent.is_dir():
        return []
    return [folder for folder in parent.iterdir() if is_run_folder(folder, experiment, subject)]


TRIALS_CSV = "trials.csv"
SESSIONS_CSV = "sessions.csv"

# How each trial ended. The first of these in a trial is its outcome.
OUTCOMES = ("snap", "goal", "timeout", "reach", "reach_wrong", "choice", "choice_unrewarded", "forage_done", "probe_end", "session_end",
            "response_timeout", "early_touch", "safety_trip", "smartpad_mismatch", "go_cue_failed", "stop_requested",
            "paused")
# A choice in N-choice completes the trial whether or not it paid (the reward is its own column).
SUCCESS = ("snap", "goal", "reach", "choice", "choice_unrewarded", "forage_done", "session_end")
FAILURE = ("timeout", "reach_wrong", "response_timeout", "early_touch")
# safety_trip, smartpad_mismatch, go_cue_failed, stop_requested, paused: aborted -- neither (success left blank).
# probe_end (Foraging's probe trial, no rewards): neither too; response_timeout in Foraging may follow found zones.

COMMON = {
    "run": "the run folder (one Start-to-Stop session)",
    "session_id": "the recorder's session id (launch time)",
    "subject": "subject ID entered at Start",
    "experiment": "experiment key",
    "paradigm": "the paradigm this trial ran (the card's tab)",
    "trial": "trial number in the run",
    "t_start": "trial_start [Unix s, ROS computer clock]",
    "t_go": "go cue [Unix s]",
    "t_outcome": "the outcome event [Unix s]",
    "t_end": "trial_end [Unix s]",
    "outcome": "how the trial ended: " + ", ".join(OUTCOMES),
    "success": "1 = success, 0 = failure, blank = aborted (safety, Stop, SmartPad) or no outcome",
    "outcome_latency_s": "go cue to outcome [s]",
    "n_rewards": "reward pulses commanded in the trial",
    "reward_ms": "total valve-open time commanded [ms] (every pulse of every burst)",
    "reward_pulses": "valve openings commanded (a reward of reward_pulse.count pulses counts them all)",
    "partial": "1 = the trial's recording was cut short",
    "git_sha": "code version that ran the trial",
    "git_dirty": "1 = the code had uncommitted changes",
}

SESSION_COMMON = {
    "run": "the run folder",
    "date": "date of the first trial (local)",
    "start": "time of the first trial (local)",
    "duration_min": "first trial start to last trial end [min]",
    "subject": "subject ID",
    "experiment": "experiment key",
    "paradigms": "paradigms run, in order of first use",
    "n_trials": "trials",
    "n_success": "successful trials",
    "success_pct": "successful trials over trials with an outcome [%]",
    "n_rewards": "reward pulses",
    "reward_ms": "total valve-open time [ms]",
    "settings": "the task's key settings at the first trial (name=value; ...)",
    "changed": "settings changed during the run (name: first -> last; ...)",
    "git_sha": "code version(s)",
}


# ---------------------------------------------------------------- one trial, as the sidecar holds it
class Trial:
    """A trial's sidecar, with helpers for the task classes."""

    def __init__(self, meta: dict, run: str = "") -> None:
        self.meta = meta
        self.run = run
        # By time, keeping the recorded order for events marked together (a reward right after its outcome).
        self.events: List[Tuple[float, str, object]] = sorted(
            ((float(e.get("t_wall") or 0.0), str(e.get("event")), e.get("arg")) for e in meta.get("events", [])),
            key=lambda e: e[0])
        self.params: dict = meta.get("params") or {}

    def t(self, token: str) -> Optional[float]:
        """When `token` first happened [Unix s], or None."""
        return next((t for t, tok, _ in self.events if tok == token), None)

    def arg(self, token: str):
        """The argument of `token`'s first occurrence, or None."""
        return next((a for _, tok, a in self.events if tok == token), None)

    def all(self, token: str) -> List[Tuple[float, object]]:
        return [(t, a) for t, tok, a in self.events if tok == token]

    def has(self, token: str) -> bool:
        return any(tok == token for _, tok, _ in self.events)

    def param(self, name: str, default=None):
        return self.params.get(name, default)

    def outcome(self) -> Tuple[Optional[str], Optional[float]]:
        return next(((tok, t) for t, tok, _ in self.events if tok in OUTCOMES), (None, None))


# ---------------------------------------------------------------- the task scaffold
class TaskTables:
    """What a task adds to the tables. Every attribute and method is optional (module doc)."""
    paradigm_param: Optional[str] = None
    trial_columns: Dict[str, str] = {}
    settings: List[str] = []
    unit_tables: Dict[str, Dict[str, str]] = {}
    session_columns: Dict[str, str] = {}

    def paradigm(self, tr: Trial) -> str:
        if tr.meta.get("paradigm"):
            return str(tr.meta["paradigm"])
        return str(tr.param(self.paradigm_param, "")) if self.paradigm_param else ""

    def trial_fields(self, tr: Trial) -> dict:
        return {}

    def units(self, tr: Trial) -> Dict[str, List[dict]]:
        return {}

    def session_fields(self, trials: List[dict]) -> dict:
        return {}


_REGISTRY: Dict[str, TaskTables] = {}


def register(*experiments: str):
    """Class decorator: use this TaskTables for these experiment keys."""
    def deco(cls):
        for key in experiments:
            _REGISTRY[key] = cls()
        return cls
    return deco


def tables_for(experiment: str) -> TaskTables:
    return _REGISTRY.get(experiment) or TaskTables()


# ---------------------------------------------------------------- rows
def _num(x, nd=4):
    return "" if x is None else round(float(x), nd)


def trial_row(meta: dict, run: str = "") -> dict:
    """The trials.csv row for one sidecar: COMMON, then the task's columns."""
    tr = Trial(meta, run)
    task = tables_for(str(meta.get("experiment", "")))
    outcome, t_out = tr.outcome()
    t_go = tr.t("cue_go")
    rewards = meta.get("rewards") or []
    git = meta.get("git") or {}
    row = {
        "run": run, "session_id": meta.get("session_id", ""), "subject": meta.get("subject") or "",
        "experiment": meta.get("experiment", ""), "paradigm": task.paradigm(tr),
        "trial": meta.get("trial_index", ""),
        "t_start": _num(tr.t("trial_start"), 3), "t_go": _num(t_go, 3), "t_outcome": _num(t_out, 3),
        "t_end": _num(tr.t("trial_end"), 3),
        "outcome": outcome or "",
        "success": 1 if outcome in SUCCESS else 0 if outcome in FAILURE else "",
        "outcome_latency_s": _num(t_out - t_go, 3) if t_out is not None and t_go is not None else "",
        "n_rewards": len(rewards),
        "reward_ms": sum(int(r.get("ms", 0)) * int(r.get("count", 1)) for r in rewards),
        "reward_pulses": sum(int(r.get("count", 1)) for r in rewards),
        "partial": 1 if meta.get("partial") else 0,
        "git_sha": git.get("sha") or "", "git_dirty": "" if git.get("dirty") is None else int(bool(git["dirty"])),
    }
    extra = task.trial_fields(tr)
    row.update({k: extra.get(k, "") for k in task.trial_columns})
    return row


def trial_columns(experiment: str) -> List[str]:
    return list(COMMON) + list(tables_for(experiment).trial_columns)


def unit_rows(meta: dict, run: str = "") -> Dict[str, List[dict]]:
    """{table name: rows} for one sidecar; each row starts with run and trial."""
    task = tables_for(str(meta.get("experiment", "")))
    out = {}
    for name, rows in task.units(Trial(meta, run)).items():
        out[name] = [{"run": run, "trial": meta.get("trial_index", ""), **r} for r in rows]
    return out


def unit_columns(experiment: str, table: str) -> List[str]:
    return ["run", "trial"] + list(tables_for(experiment).unit_tables[table])


# ---------------------------------------------------------------- writing, live and in bulk
def _append(path: Path, columns: List[str], rows: Iterable[dict]) -> None:
    rows = list(rows)
    if not rows:
        return
    if path.exists():
        with path.open(newline="") as f:
            header = next(csv.reader(f), [])
        if header != columns:      # written by another code version: start a fresh file beside it
            path = path.with_name(f"{path.stem}_{dt.datetime.now():%Y%m%d_%H%M%S}{path.suffix}")
    new = not path.exists()
    with path.open("a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=columns, extrasaction="ignore")
        if new:
            w.writeheader()
        w.writerows(rows)


def append_trial(run_folder, meta: dict) -> None:
    """Add one finished trial to its run's tables. Called by TrialRecorder.stop_and_save()."""
    folder = Path(run_folder)
    exp = str(meta.get("experiment", ""))
    _append(folder / TRIALS_CSV, trial_columns(exp), [trial_row(meta, folder.name)])
    for name, rows in unit_rows(meta, folder.name).items():
        _append(folder / f"{name}.csv", unit_columns(exp, name), rows)


def sidecars(run_folder) -> List[dict]:
    """The run's trial sidecars, in trial order."""
    out = []
    for p in sorted(glob.glob(os.path.join(str(run_folder), "*.meta.json"))):
        try:
            with open(p) as f:
                out.append(json.load(f))
        except (OSError, json.JSONDecodeError):
            continue
    return sorted(out, key=lambda m: int(m.get("trial_index") or 0))


def rebuild_run(run_folder) -> int:
    """Rewrite trials.csv and the unit tables of a run from its sidecars. Returns the number of trials."""
    folder = Path(run_folder)
    metas = sidecars(folder)
    for p in [folder / TRIALS_CSV] + [folder / f"{n}.csv" for m in metas[:1]
                                       for n in tables_for(str(m.get("experiment", ""))).unit_tables]:
        if p.exists():
            p.unlink()
    for m in metas:
        append_trial(folder, m)
    if metas:
        write_column_docs(folder, str(metas[0].get("experiment", "")))
    return len(metas)


def read_trials(run_folder) -> List[dict]:
    p = Path(run_folder) / TRIALS_CSV
    if not p.exists():
        return []
    with p.open(newline="") as f:
        return list(csv.DictReader(f))


def write_column_docs(run_folder, experiment: str) -> None:
    """tables.md beside the tables: what every column means."""
    task = tables_for(experiment)
    lines = [f"# Outcome tables ({experiment})", "",
             "Computed from the trial sidecars by sinthlab_bringup/helpers/outcome_tables.py; rebuild with "
             "`python3 analysis/outcome_tables.py <this folder>`.", "", f"## {TRIALS_CSV}", ""]
    lines += [f"- `{k}`: {v}" for k, v in {**COMMON, **task.trial_columns}.items()]
    for name, cols in task.unit_tables.items():
        lines += ["", f"## {name}.csv", "", "- `run`, `trial`: as in trials.csv"]
        lines += [f"- `{k}`: {v}" for k, v in cols.items()]
    (Path(run_folder) / "tables.md").write_text("\n".join(lines) + "\n")


# ---------------------------------------------------------------- the session (run) row
def _fmt(v) -> str:
    return json.dumps(v) if isinstance(v, (list, dict)) else str(v)


def summarize(run: str, rows: List[dict], first_params: dict, last_params: dict) -> Optional[dict]:
    """A run's sessions.csv row from its trials.csv rows and the parameters at its first and last trial.
    Used for recorded runs (session_row) and by the dashboard's demo mode alike."""
    if not rows:
        return None
    rows = [{k: "" if v is None else str(v) for k, v in r.items()} for r in rows]   # as read from the CSV
    exp = rows[0]["experiment"]
    task = tables_for(exp)
    starts = [float(r["t_start"]) for r in rows if r.get("t_start")]
    ends = [float(r["t_end"]) for r in rows if r.get("t_end")]
    t0 = min(starts) if starts else None
    with_outcome = [r for r in rows if r["success"] in ("0", "1")]
    n_ok = sum(r["success"] == "1" for r in rows)
    paradigms = list(dict.fromkeys(r["paradigm"] for r in rows if r["paradigm"]))
    first, last = first_params or {}, last_params or {}
    settings = "; ".join(f"{k}={_fmt(first[k])}" for k in settings_for(exp) if k in first)
    # Everything that changed between the first and the last trial (live changes during the run).
    changed = "; ".join(f"{k}: {_fmt(first[k])} -> {_fmt(last[k])}" for k in sorted(first)
                        if k in last and first[k] != last[k])
    row = {
        "run": run,
        "date": dt.datetime.fromtimestamp(t0).strftime("%Y-%m-%d") if t0 else "",
        "start": dt.datetime.fromtimestamp(t0).strftime("%H:%M") if t0 else "",
        "duration_min": round((max(ends) - t0) / 60.0, 1) if t0 and ends else "",
        "subject": next((r["subject"] for r in rows if r["subject"]), ""),
        "experiment": exp,
        "paradigms": " | ".join(paradigms),
        "n_trials": len(rows), "n_success": n_ok,
        "success_pct": round(100.0 * n_ok / len(with_outcome), 1) if with_outcome else "",
        "n_rewards": sum(int(r["n_rewards"] or 0) for r in rows),
        "reward_ms": sum(int(r["reward_ms"] or 0) for r in rows),
        "settings": settings, "changed": changed,
        "git_sha": " | ".join(dict.fromkeys(r["git_sha"] for r in rows if r.get("git_sha"))),
    }
    extra = task.session_fields(rows)
    row.update({k: extra.get(k, "") for k in task.session_columns})
    return row


def session_row(run_folder) -> Optional[dict]:
    """The run's sessions.csv row, from its trials.csv and sidecars; None if it has no trials."""
    folder = Path(run_folder)
    rows = read_trials(folder)
    metas = sidecars(folder)
    if not rows or not metas:
        return None
    return summarize(folder.name, rows, metas[0].get("params") or {}, metas[-1].get("params") or {})


def update_sessions(run_folder, sessions_csv=None) -> Optional[dict]:
    """Add (or replace) the run's row in sessions.csv."""
    row = session_row(run_folder)
    if row is None:
        return None
    upsert_session(row, Path(sessions_csv) if sessions_csv else Path(run_folder).parent / SESSIONS_CSV)
    return row


def upsert_session(row: dict, path) -> None:
    """Add (or replace, by run) one row in sessions.csv. Small file: rewritten whole, columns = the common
    ones, then every task's own, in order of first appearance."""
    path = Path(path)
    rows = []
    if path.exists():
        with path.open(newline="") as f:
            rows = [r for r in csv.DictReader(f) if r.get("run") != row["run"]]
    rows.append(row)
    rows.sort(key=lambda r: (r.get("date", ""), r.get("start", ""), r.get("run", "")))
    cols = list(SESSION_COMMON)
    for r in rows:
        cols += [k for k in r if k not in cols]
    tmp = path.with_suffix(".csv.tmp")
    with tmp.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    os.replace(tmp, path)


# Settings worth comparing across sessions in every task, then each task's own (TaskTables.settings).
COMMON_SETTINGS = ["reward_pulse.pulse_ms", "reward_pulse.count", "response_timeout_sec", "quiet_window_sec", "move_to_park.hold_sec",
                   "quiet_gate.penalty_sec", "free_move.damping_ns_per_m"]


def settings_for(experiment: str) -> List[str]:
    """The settings sessions.csv keeps for an experiment (and the dashboard compares across sessions)."""
    return list(dict.fromkeys(tables_for(experiment).settings + COMMON_SETTINGS))


def parse_settings(text: str) -> Dict[str, str]:
    """sessions.csv's `settings` cell -> {name: value as written}."""
    out = {}
    for part in str(text or "").split("; "):
        if "=" in part:
            k, v = part.split("=", 1)
            out[k.strip()] = v.strip()
    return out


def history(experiment: str, subject: str = "", n: int = 3, sessions_csv=None) -> List[dict]:
    """The last n sessions of an experiment (for one subject, if given), oldest first, each with its
    settings parsed (`settings_dict`)."""
    rows = [r for r in read_sessions(sessions_csv) if r.get("experiment") == experiment
            and (not subject or r.get("subject") == subject)]
    out = rows[-n:] if n else rows
    return [{**r, "settings_dict": parse_settings(r.get("settings", ""))} for r in out]


def read_sessions(sessions_csv=None) -> List[dict]:
    path = Path(sessions_csv) if sessions_csv else data_dir() / SESSIONS_CSV
    if not path.exists():
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


# ================================================================ the tasks' own tables
def _split_kv(arg) -> dict:
    """"length=0.123;straightness=0.91" -> {"length": "0.123", ...}"""
    out = {}
    for part in str(arg or "").split(";"):
        if "=" in part:
            k, v = part.split("=", 1)
            out[k.strip()] = v.strip()
    return out


@register("move_vertical", "move_horizontal", "move_rails")
class CentreOutTables(TaskTables):
    """Centre-out reach on a rail (actions/centre_out.py)."""
    paradigm_param = "centre_out.target"
    trial_columns = {
        "target": "the goal: an end's name, or either",
        "closed_end": "the end closed off this trial (restricted cued), else blank",
        "chosen_end": "the end the arm reached, blank if none",
        "correct": "1 = reached a goal end, 0 = the other end, blank = no reach",
        "first_correct": "1 = the FIRST end reached was a goal end, 0 = it was the other end (cued, can correct: "
                         "before he corrected), blank = no reach",
        "n_wrong_end": "cued, can correct: arrivals at the wrong end before the trial ended (0 otherwise)",
        "rail": "the rail: horizontal or vertical (Rails, switchable: rails.axis; else the task's own)",
        "end_tol_m": "how close to an end counted as reaching it [m]",
    }
    settings = ["centre_out.target", "rails.axis", "centre_out.max_repeats", "centre_out.wrong_timeout_sec",
                "centre_out.end_tol_m", "reward_pulse.pulse_ms"]
    session_columns = {"n_reaches": "trials ending in a reach (either end)",
                       "pct_correct": "reaches to a goal end over all reaches [%]",
                       "pct_first_correct": "trials whose first end reached was a goal end, over all reaches [%]"}

    def trial_fields(self, tr):
        chosen = tr.arg("reach") if tr.has("reach") else tr.arg("reach_wrong")
        wrong = len(tr.all("wrong_end"))
        correct = 1 if tr.has("reach") else 0 if tr.has("reach_wrong") else ""
        return {"target": tr.arg("target") or "", "closed_end": tr.arg("closed_end") or "",
                "chosen_end": chosen or "",
                "correct": correct,
                "first_correct": "" if correct == "" else 0 if wrong else correct,
                "n_wrong_end": wrong,
                "rail": tr.param("rails.axis") or {"y": "horizontal", "z": "vertical"}.get(
                    str(tr.param("free_move.rail_axis", "")), ""),
                "end_tol_m": tr.param("centre_out.end_tol_m", "")}

    def session_fields(self, trials):
        reached = [r for r in trials if r.get("correct") in ("0", "1")]
        ok = sum(r["correct"] == "1" for r in reached)
        first = sum(r.get("first_correct") == "1" for r in reached)
        return {"n_reaches": len(reached),
                "pct_correct": round(100.0 * ok / len(reached), 1) if reached else "",
                "pct_first_correct": round(100.0 * first / len(reached), 1) if reached else ""}


@register("colour_rule")
class ColourRuleTables(CentreOutTables):
    """The colour rule with unsignaled reversals (helpers/rule_schedule.py): the choice, plus the rule."""
    paradigm_param = "free_move.rail_axis"
    trial_columns = {
        "target": "the end the cue colour means (the correct end)",
        "chosen_end": "the end chosen, blank if none",
        "correct": "1 = the end the colour means, 0 = the other end, blank = no choice",
        "cue_colour": "the cue colour (red / blue)",
        "mapping": "the mapping in force this trial (red=<end>,blue=<end>)",
        "block": "block number (1 = before the first reversal)",
        "choices_in_block": "choices already made in this block, before this trial",
        "window_pct": "% correct over the criterion window, before this trial",
        "reversal": "1 = first trial after a reversal (the mapping just flipped, unsignaled)",
    }
    settings = ["free_move.rail_axis", "rule.red_end", "rule.window", "rule.criterion_pct", "rule.min_trials",
                "rule.max_repeats", "centre_out.free_reach_m", "reward_pulse.pulse_ms"]
    session_columns = {"n_choices": "trials ending in a choice",
                       "pct_correct": "correct choices over all choices [%]",
                       "n_reversals": "reversals reached",
                       "choices_to_criterion": "mean choices per completed block (to criterion)"}

    def paradigm(self, tr):
        return {"y": "rail", "frontal": "no rail"}.get(str(tr.param("free_move.rail_axis", "")), "")

    def trial_fields(self, tr):
        out = super().trial_fields(tr)
        rule = tr.meta.get("rule") or {}
        out.update({"cue_colour": tr.arg("cue_colour") or "",
                    "mapping": rule.get("mapping", ""), "block": rule.get("block", ""),
                    "choices_in_block": rule.get("choices_in_block", ""),
                    "window_pct": "" if rule.get("window_pct") is None else rule["window_pct"],
                    "reversal": 1 if tr.has("reversal") else 0})
        return out

    def session_fields(self, trials):
        chosen = [r for r in trials if r.get("correct") in ("0", "1")]
        ok = sum(r["correct"] == "1" for r in chosen)
        # A block is complete once a later block starts; its length is its choices.
        blocks: Dict[str, int] = {}
        for r in chosen:
            if r.get("block"):
                blocks[r["block"]] = blocks.get(r["block"], 0) + 1
        last = max((int(r["block"]) for r in trials if r.get("block")), default=0)
        done = [n for b, n in blocks.items() if int(b) < last]
        return {"n_choices": len(chosen),
                "pct_correct": round(100.0 * ok / len(chosen), 1) if chosen else "",
                "n_reversals": sum(r.get("reversal") == "1" for r in trials),
                "choices_to_criterion": round(sum(done) / len(done), 1) if done else ""}


@register("n_choice")
class NChoiceTables(TaskTables):
    """N-choice with depleting rewards (actions/n_choice.py, helpers/bandit_schedule.py)."""
    paradigm_param = "free_move.ray_layout"
    trial_columns = {
        "n_options": "how many directions there were to choose from",
        "choice": "the direction chosen (the rail whose end was reached), blank if none",
        "rewarded": "1 = the choice paid, 0 = it did not, blank = no choice",
        "p_chosen": "the chosen direction's reward probability, before the choice",
        "best": "the direction(s) with the highest probability before the choice (| between ties)",
        "chose_best": "1 = chose a best direction, 0 = not, blank = no choice",
        "p_values": "every direction's probability before the choice (direction=p; ...)",
    }
    settings = ["free_move.ray_layout", "free_move.ray_length_m", "n_choice.p_max", "n_choice.p_min",
                "n_choice.deplete", "n_choice.recover", "n_choice.start", "reward_pulse.pulse_ms"]
    session_columns = {"n_choices": "trials ending in a choice",
                       "reward_rate": "choices that paid over all choices [%]",
                       "pct_chose_best": "choices of a best direction [%]",
                       "pct_switch": "choices of a different direction from the previous choice [%]",
                       "choice_counts": "choices per direction (direction=n; ...)"}

    def trial_fields(self, tr):
        state = tr.meta.get("n_choice") or {}
        p = state.get("p") or {}
        best = state.get("best") or []
        chosen = tr.arg("choice") if tr.has("choice") else tr.arg("choice_unrewarded")
        return {"n_options": len(p) or "", "choice": chosen or "",
                "rewarded": 1 if tr.has("choice") else 0 if tr.has("choice_unrewarded") else "",
                "p_chosen": p.get(chosen, "") if chosen else "",
                "best": " | ".join(best), "chose_best": (1 if chosen in best else 0) if chosen else "",
                "p_values": "; ".join(f"{k}={v}" for k, v in p.items())}

    def session_fields(self, trials):
        chosen = [r for r in trials if r.get("choice")]
        if not chosen:
            return {"n_choices": 0}
        pct = lambda n: round(100.0 * n / len(chosen), 1)   # noqa: E731
        switches = sum(a["choice"] != b["choice"] for a, b in zip(chosen, chosen[1:]))
        counts: Dict[str, int] = {}
        for r in chosen:
            counts[r["choice"]] = counts.get(r["choice"], 0) + 1
        return {"n_choices": len(chosen), "reward_rate": pct(sum(r["rewarded"] == "1" for r in chosen)),
                "pct_chose_best": pct(sum(r["chose_best"] == "1" for r in chosen)),
                "pct_switch": round(100.0 * switches / (len(chosen) - 1), 1) if len(chosen) > 1 else "",
                "choice_counts": "; ".join(f"{k}={v}" for k, v in sorted(counts.items()))}


@register("move_free")
class FreeReachTables(TaskTables):
    """Cued Reaches, free (actions/free_reach.py, helpers/free_reach.py): the reach, its cue, its perturbation."""
    paradigm_param = "free_reach.cue"
    trial_columns = {
        "target": "the cued direction (right / left / down / up), or any with no cue",
        "perturbation": "the trial's planned perturbation: jump, bump, or blank for a standard trial",
        "perturb_angle_deg": "its direction from the cued direction [deg]: 90 = counter-clockwise as he sees it, 270 = clockwise",
        "perturb_m": "its size [m]",
        "perturbed": "1 = the perturbation happened, 0 = planned but not delivered (a jump after he had gripped), blank = none planned",
        "cue_delay_s": "how long the cue showed before the go [s]",
        "grip_s": "the grip (force or first movement), from the go cue [s]: the reaction time",
        "perturb_s": "the jump or bump onset, from the go cue [s]",
        "crossing_deg": "the direction the reach ended in [deg]: 0 = right, 90 = up",
        "error_deg": "how far that was from the cued direction [deg], + = counter-clockwise; blank with no cue",
        "correct": "1 = a reach to the target (rewarded), 0 = outside it, blank = no reach",
        "block": "the schedule's block (a new one at every change of cue, perturbation or settings)",
        "depth_rule": "the bump's depth rule: against / with his depth movement, toward / away (fixed); blank = none",
        "depth_dir": "the bump's depth direction as delivered: toward / away (from the monkey)",
        "force_n": "the bump's force pulse as delivered, base frame x (depth, + toward him), y, z [N]",
        "depth_force_n": "the bump's depth force as delivered [N] (less than set if the box was short of room)",
        "depth_m": "how far that depth force moves the apple if he does not resist it [m]",
        "depth_basis": "what set it: velocity / displacement (his depth movement), random (he was not moving in depth), fixed",
        "depth_speed_mps": "his depth velocity at the bump's onset [m/s], + toward himself",
    }
    settings = ["free_reach.cue", "perturb.kind", "free_reach.directions", "free_reach.distance_m", "free_reach.cone_deg", "perturb.probability",
                "perturb.jump_m", "perturb.bump_force_n", "perturb.bump_sec", "perturb.bump_at_m", "perturb.bump_depth_force_n",
                "perturb.bump_depth_rules", "response_timeout_sec", "reward_pulse.pulse_ms", "free_move.rail_axis"]
    session_columns = {"n_reaches": "trials ending in a reach",
                       "pct_correct": "reaches to the target over all reaches [%]",
                       "pct_correct_standard": "the same, unperturbed trials [%]",
                       "pct_correct_perturbed": "the same, trials with a jump or bump [%]",
                       "pct_correct_depth_against": "the same, bumps against his depth movement [%]",
                       "pct_correct_depth_with": "the same, bumps with his depth movement [%]",
                       "median_grip_s": "median grip time from the go cue [s]"}

    def trial_fields(self, tr):
        plan = tr.meta.get("free_reach") or {}
        res = tr.meta.get("free_reach_result") or {}
        go = tr.t("cue_go")
        since = lambda tok: _num(tr.t(tok) - go, 3) if go is not None and tr.has(tok) else ""   # noqa: E731
        kind = plan.get("perturb") or ""
        reached = tr.arg("reach") if tr.has("reach") else tr.arg("reach_wrong")
        return {"target": plan.get("direction") or ("any" if plan else ""),
                "perturbation": kind,
                "perturb_angle_deg": "" if plan.get("angle_deg") is None else plan["angle_deg"],
                "perturb_m": plan.get("size_m", "") if kind else "",
                "perturbed": (1 if tr.has(kind) else 0) if kind else "",
                "cue_delay_s": plan.get("cue_delay_sec", ""),
                "grip_s": since("grip"), "perturb_s": since(kind) if kind else "",
                "crossing_deg": "" if reached is None else reached,
                "error_deg": "" if res.get("error_deg") is None else res["error_deg"],
                "correct": 1 if tr.has("reach") else 0 if tr.has("reach_wrong") else "",
                "block": plan.get("block", ""),
                "depth_rule": plan.get("depth_rule") or "",
                **{k: ("" if res.get(k) is None else res[k]) for k in ("depth_dir", "depth_m", "depth_basis",
                                                                     "depth_speed_mps")}}

    def session_fields(self, trials):
        reached = [r for r in trials if r.get("correct") in ("0", "1")]
        pct = lambda rows: (round(100.0 * sum(r["correct"] == "1" for r in rows) / len(rows), 1)   # noqa: E731
                            if rows else "")
        grips = sorted(float(r["grip_s"]) for r in trials if r.get("grip_s") not in ("", None))
        return {"n_reaches": len(reached), "pct_correct": pct(reached),
                "pct_correct_standard": pct([r for r in reached if r.get("perturbed") != "1"]),
                "pct_correct_perturbed": pct([r for r in reached if r.get("perturbed") == "1"]),
                "pct_correct_depth_against": pct([r for r in reached if r.get("perturbed") == "1"
                                                  and r.get("depth_rule") == "against"]),
                "pct_correct_depth_with": pct([r for r in reached if r.get("perturbed") == "1"
                                               and r.get("depth_rule") == "with"]),
                "median_grip_s": (round((grips[(len(grips) - 1) // 2] + grips[len(grips) // 2]) / 2, 3)
                                  if grips else "")}


@register("forage")
class ForageTables(TaskTables):
    """Foraging (actions/forage.py, helpers/forage.py): the search, zone by zone."""
    paradigm_param = "free_move.rail_axis"
    trial_columns = {
        "space": "plane (the plane facing him) or 3-D",
        "arena": "the arena's shape",
        "seed": "the environment's seed (the same all session)",
        "n_zones": "zones in the environment",
        "n_found": "zones found (paid) this trial",
        "all_found": "1 = every zone found, 0 = not, blank = a probe or no search",
        "found_order": "the zones in the order found (their index)",
        "t_first_s": "the first zone's reward, from the go cue [s]",
        "t_all_s": "the last zone's reward when all were found, from the go cue [s]",
        "revisits": "entries into a zone already emptied this trial (working-memory errors)",
        "path_m": "the hand's path length during the search [m] (in the plane, or in 3-D)",
        "warm_factor": "the warm radius this trial, as a multiple of the zone radius",
        "warm_brightness": "the warm-zone cue's brightness this trial (0 = off)",
        "warm_on_s": "time the warm-zone cue was lit [s]",
        "probe": "1 = a probe trial (no cue, no rewards)",
        "dwell_s": "the dwell a zone needed to pay [s]",
    }
    settings = ["free_move.rail_axis", "free_move.box.shape", "forage.seed", "forage.n_zones", "forage.zone_radius_m",
                "forage.dwell_sec", "warm.mode", "warm.shrink_trials", "warm.fade_trials", "probe.probability",
                "forage.time_limit_sec", "reward_pulse.pulse_ms"]
    session_columns = {"n_searches": "trials with a search (not probes, not aborted)",
                       "pct_all_found": "searches that found every zone [%]",
                       "t_all_first3_s": "median time to find all, first 3 such trials [s]",
                       "t_all_last3_s": "median time to find all, last 3 such trials [s]",
                       "seed": "the environment's seed"}

    def paradigm(self, tr):
        return {"frontal": "plane", "none": "3-D"}.get(str(tr.param("free_move.rail_axis", "")), "")

    def trial_fields(self, tr):
        plan = tr.meta.get("forage") or {}
        res = tr.meta.get("forage_result") or {}
        times = [t for t in (res.get("t_zone_s") or []) if t is not None]
        probe = bool(plan.get("probe"))
        n_zones = len(plan.get("zones") or []) or res.get("n_zones", "")
        searched = bool(res) and not probe
        return {"space": {"frontal": "plane", "none": "3-D"}.get(plan.get("space"), ""), "arena": plan.get("shape", ""),
                "seed": plan.get("seed", ""), "n_zones": n_zones, "n_found": res.get("n_found", ""),
                "all_found": (1 if tr.has("forage_done") else 0) if searched else "",
                "found_order": " ".join(str(i) for i in res.get("found") or []),
                "t_first_s": times[0] if times else "", "t_all_s": times[-1] if tr.has("forage_done") and times else "",
                "revisits": res.get("revisits", ""), "path_m": res.get("path_m", ""),
                "warm_factor": plan.get("warm_factor", ""), "warm_brightness": plan.get("warm_brightness", ""),
                "warm_on_s": res.get("warm_on_s", ""), "probe": 1 if probe else 0 if plan else "",
                "dwell_s": plan.get("dwell_sec", "")}

    def session_fields(self, trials):
        searched = [r for r in trials if r.get("all_found") in ("0", "1")]
        done = [float(r["t_all_s"]) for r in searched if r.get("t_all_s") not in ("", None)]

        def med(xs):
            xs = sorted(xs)
            return round((xs[(len(xs) - 1) // 2] + xs[len(xs) // 2]) / 2, 2) if xs else ""
        return {"n_searches": len(searched),
                "pct_all_found": round(100.0 * sum(r["all_found"] == "1" for r in searched) / len(searched), 1)
                                 if searched else "",
                "t_all_first3_s": med(done[:3]), "t_all_last3_s": med(done[-3:]),
                "seed": next((r.get("seed") for r in trials if r.get("seed")), "")}


@register("free_move", "reach_map")
class LongMoveTables(TaskTables):
    """Free Move, with the Long moves reward (actions/long_move.py), and the Reach Map (its New best reward
    too): one row per movement."""
    paradigm_param = "long_move.reward"
    trial_columns = {
        "min_m": "shortest rewarded movement in force [m] (Long moves)",
        "n_movements": "movements (rest to rest, each at least confirm_m)",
        "n_long_moves": "movements rewarded (Long moves or New best)",
        "n_new_best": "of them, Reach Map New-best rewards",
        "mean_length_m": "mean movement length [m]",
        "max_length_m": "longest movement [m]",
    }
    settings = ["long_move.reward", "long_move.min_m", "long_move.min_straightness", "long_move.min_gap_sec",
                "session_sec", "free_move.damping_ns_per_m", "reward_pulse.pulse_ms"]
    unit_tables = {"movements": {
        "movement": "movement number in the trial",
        "t_confirmed": "when it had gone confirm_m (event move_start) [Unix s]",
        "t_end": "when it ended (event move_end) [Unix s]",
        "length_m": "straight-line distance, where it started to where it ended [m]",
        "straightness": "length over the path travelled (1 = straight)",
        "duration_s": "speed onset to end [s]",
        "peak_speed_mps": "peak speed [m/s]",
        "rewarded": "1 = earned the reward (event long_move, or new_best in the Reach Map)",
        "reach_m": "Reach Map, New best: how far from the start it got, in its direction [m] (rewarded ones only)",
        "direction_deg": "Reach Map, New best: its direction, 0 = right, 90 = up [deg] (rewarded ones only)",
    }}
    session_columns = {"n_movements": "movements in the run", "n_long_moves": "rewarded movements"}

    def _movements(self, tr):
        out, starts = [], [t for t, _ in tr.all("move_start")]
        evs = tr.events
        for i, (t, tok, arg) in enumerate(evs):
            if tok != "move_end":
                continue
            kv = _split_kv(arg)
            nxt = evs[i + 1][1] if i + 1 < len(evs) else ""
            best = _split_kv(evs[i + 1][2]) if nxt == "new_best" else {}
            t_conf = max((s for s in starts if s <= t), default=None)
            out.append({"movement": len(out) + 1, "t_confirmed": _num(t_conf, 3), "t_end": _num(t, 3),
                        "length_m": kv.get("length", ""), "straightness": kv.get("straightness", ""),
                        "duration_s": kv.get("duration", ""), "peak_speed_mps": kv.get("peak_speed", ""),
                        "rewarded": 1 if nxt in ("long_move", "new_best") else 0,
                        "reach_m": best.get("reach", ""), "direction_deg": best.get("deg", "")})
        return out

    def trial_fields(self, tr):
        mv = self._movements(tr)
        lengths = [float(m["length_m"]) for m in mv if m["length_m"] != ""]
        return {"min_m": tr.param("long_move.min_m", "") if tr.param("long_move.reward") == "each_move" else "",
                "n_new_best": sum(1 for m in mv if m["reach_m"] != ""),
                "n_movements": len(mv), "n_long_moves": sum(m["rewarded"] for m in mv),
                "mean_length_m": round(sum(lengths) / len(lengths), 4) if lengths else "",
                "max_length_m": max(lengths) if lengths else ""}

    def units(self, tr):
        return {"movements": self._movements(tr)}

    def session_fields(self, trials):
        return {"n_movements": sum(int(r.get("n_movements") or 0) for r in trials),
                "n_long_moves": sum(int(r.get("n_long_moves") or 0) for r in trials)}
