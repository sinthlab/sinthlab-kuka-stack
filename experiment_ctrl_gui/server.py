#!/usr/bin/env python3
"""Experiment control dashboard -- a local web page for running the sinthlab KUKA experiments.

    ./run_gui.sh                      # sources ROS + the workspace, then runs this
    python3 server.py --demo          # no robot, no ROS: a simulated experiment, to try the page

Then open http://localhost:8080 . See README.md in this folder.

Serves on 127.0.0.1 only by default: whoever can open this page can start the robot. Standard
library only (plus PyYAML, which ROS already installs), so there is nothing extra to install.
"""
from __future__ import annotations

import argparse
import json
import mimetypes
import queue
import re
import subprocess
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, List, Optional
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import experiments as ex  # noqa: E402
from params import ExperimentParams, read_fixed_config  # noqa: E402
from ros_bridge import DemoBridge, make_bridge  # noqa: E402
import board as cue_board  # noqa: E402
import kinematics  # noqa: E402
import install_check  # noqa: E402
from sinthlab_bringup.helpers import free_reach  # noqa: E402  (on the path via experiments)
import profiles  # noqa: E402
import reach_view  # noqa: E402
import reward_volume  # noqa: E402
from live_view import LiveView  # noqa: E402
import ttl  # noqa: E402
from postprocess import PostProcessor  # noqa: E402
from runner import LaunchRunner  # noqa: E402

STATIC = HERE / "static"
STATUS_STALE_SEC = 3.0         # the orchestrator heartbeats every 1 s
LIVE_VIEW_HZ = 15.0            # the live view's position updates
STOP_PARK_TIMEOUT_SEC = 60.0   # Stop: end the launch anyway if the arm has not parked by then
CSRF_HEADER = "X-Experiment-Ctrl"   # a cross-site page cannot send a custom header without CORS


class App:
    def __init__(self, demo: bool, robot_name: str) -> None:
        self._sessions_done: Optional[str] = None   # the last run folder added to sessions.csv
        if demo:            # demo runs write their sessions.csv rows here, never beside real data
            import tempfile
            ex.DATA = Path(tempfile.mkdtemp(prefix="kuka_demo_data_"))
        self.demo = demo
        self.fake_board = cue_board.FakeBoard()      # the Diagnostics page's board, in --demo
        self.fake_cabinet = ttl.FakeCabinet()        # and its TTL outputs
        self.robot_name = robot_name
        self.params: Dict[str, ExperimentParams] = {e.key: ExperimentParams(e) for e in ex.EXPERIMENTS}
        # The monkey chosen on the dashboard and its profile (profiles.py), applied to every free-move task.
        self.subject = ""
        self.profile: Optional[dict] = None
        self._run_subject: Optional[str] = None
        self._run_profile: Optional[dict] = None    # the profile the current run started with
        self._clients: List[queue.Queue] = []
        self._clients_lock = threading.Lock()
        self.stopping = False   # Stop pressed: the arm is parking, then the launch stops
        self.runner = LaunchRunner(on_line=lambda e: self.broadcast("log", e),
                                   on_state=self._on_runner_state)
        self.bridge = make_bridge(demo, robot_name, self._on_status)
        if isinstance(self.bridge, DemoBridge):
            self.bridge.attach_log(self.runner.emit)
        if not self.bridge.available:
            self.runner.emit(f"ROS not available: {self.bridge.reason}", "warn")
        self.post = PostProcessor(self.broadcast, self.runner.emit, self._busy)
        self.live_view = LiveView(self.bridge)
        threading.Thread(target=self._ticker, daemon=True).start()
        threading.Thread(target=self._live_ticker, daemon=True).start()

    # ------------------------------------------------------------------ events out

    def subscribe(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(maxsize=2000)
        with self._clients_lock:
            self._clients.append(q)
        return q

    def unsubscribe(self, q: queue.Queue) -> None:
        with self._clients_lock:
            if q in self._clients:
                self._clients.remove(q)

    def broadcast(self, kind: str, data) -> None:
        msg = json.dumps({"type": kind, "data": data}, default=str)
        with self._clients_lock:
            for q in list(self._clients):
                try:
                    q.put_nowait(msg)
                except queue.Full:
                    pass     # a stalled tab loses messages rather than stalling the server

    def _ticker(self) -> None:
        while True:
            time.sleep(1.0)
            self.broadcast("state", self.state())

    def live_update(self) -> None:
        """Point the live view at the running experiment, with the parameters it runs with."""
        exp = self.running_experiment()
        status = self.status()
        self.live_view.update(exp.key if exp else None,
                              self.params[exp.key].current() if exp else {},
                              status if status and not status["stale"] else None)

    def _live_ticker(self) -> None:
        while True:
            time.sleep(1.0 / LIVE_VIEW_HZ)
            with self._clients_lock:
                listening = bool(self._clients)
            if not listening:
                continue
            try:
                self.live_update()
                s = self.status()
                self.broadcast("live", self.live_view.live(s if s and not s["stale"] else None))
            except Exception as exc:       # the view must never take the dashboard down
                self.runner.emit(f"live view: {exc}", "warn")
                time.sleep(2.0)

    # ------------------------------------------------------------------ state

    def status(self) -> Optional[dict]:
        s = self.bridge.status()
        if s is None:
            return None
        s = dict(s)
        s["age"] = time.time() - s.get("_rx", 0)
        s["stale"] = s["age"] > STATUS_STALE_SEC
        return s

    def state(self) -> dict:
        r = self.runner
        status = self.status()
        external = (r.state == "idle" and status is not None and not status["stale"])
        return {
            "runner": {"state": r.state, "experiment": r.experiment, "started_at": r.started_at,
                       "exit_code": r.exit_code, "command": r.command, "params_file": r.params_file,
                       "log_path": str(r.log_path) if r.log_path else None},
            "ros": {"available": self.bridge.available, "reason": self.bridge.reason, "demo": self.demo},
            "status": status,
            "external": external,
            "robot": self.bridge.robot(),
            "stopping_now": self.stopping,
            "data_folder": self.data_folder(),
            "edited": {k: len(p.edits) for k, p in self.params.items()},
            "edited_names": {k: sorted(_plain_name(n) for n in p.edits) for k, p in self.params.items() if p.edits},
            "subject": self.subject,
            "profile": ({"version": self.profile["version"], "updated": self.profile["updated"]}
                        if self.profile else None),
        }

    # ------------------------------------------------------------------ the monkey and its profile
    def select_subject(self, subject: str, clear_edits: bool = False) -> dict:
        """Choose the monkey: its profile (if it has one) now applies to every free-move task. With
        clear_edits, today's edits are dropped everywhere (they were made for the previous monkey)."""
        subject = check_subject(subject)
        busy = self.runner.running() and self.runner.experiment
        if busy and subject != self.subject:
            raise RuntimeError(f"{busy} is running for {self.subject or 'another subject'}: stop it before changing the monkey")
        if clear_edits:
            for p in self.params.values():
                p.reset()
        self.subject = subject
        self.profile = None if subject == profiles.SUBJECT_TEST else profiles.load(subject)
        self._apply_profile()
        tag = f"profile v{self.profile['version']}" if self.profile else (
            "no profile: the task YAMLs" if subject != profiles.SUBJECT_TEST else "test: the task YAMLs")
        self.runner.emit(f"Monkey: {subject} ({tag})", "gui")
        self.broadcast("state", self.state())
        return {"ok": True, "subject": subject, "profile": self.profile}

    def _apply_profile(self) -> None:
        """Apply the profile to every experiment but a running one (it keeps the profile it started with,
        and gets the new one when it ends). Today's edits that now equal the profile are no longer edits."""
        running = self.runner.experiment if self.runner.running() else None
        for key, p in self.params.items():
            if key == running:
                continue
            p.profile = self.profile
            base = p.base()
            for name in [n for n, v in p.edits.items() if base.get(n) == v]:
                p.edits.pop(name)

    def profile_info(self) -> dict:
        return {"subject": self.subject, "profile": self.profile, "layout": profiles.describe(),
                "names": profiles.names(), "recent": recent_subjects(),
                "changes": profiles.changes(self.subject) if self.subject else []}

    def _values_in_effect(self, key: str) -> Dict[str, object]:
        """A task's values: while it runs, with the live values the node reports -- and those accepted
        for the next trial, so a change just made can be saved before it takes effect."""
        p = self.params[key]
        vals = p.current()
        st = self.status() or {}
        if self.running_experiment() and self.running_experiment().key == key and not st.get("stale"):
            vals.update(st.get("live") or {})
            vals.update(st.get("pending") or {})
        return vals

    def profile_diff(self, key: str) -> List[dict]:
        if not self.subject or self.subject == profiles.SUBJECT_TEST:
            return []
        return profiles.diff(self.profile, ex.BY_KEY[key], self._values_in_effect(key))

    def save_profile(self, body: dict) -> dict:
        """Save to the monkey's profile: `values` {field: value} for every task, `task_only` {field: value}
        (the reward) for the task `exp` only (None removes that task's own value)."""
        if not self.subject:
            raise ValueError("Choose a monkey first")
        if self.subject == profiles.SUBJECT_TEST:
            raise ValueError("'test' is for runs without an animal: it has no profile")
        key = str(body.get("exp") or "")
        task_only = dict(body.get("task_only") or {})
        if task_only and key not in ex.BY_KEY:
            raise KeyError(f"unknown experiment {key}")
        by = ex.BY_KEY[key].label if key in ex.BY_KEY else "profile editor"
        values = dict(body.get("values") or {})
        if body.get("from_task"):        # a new profile from what this task runs with now
            if key not in ex.BY_KEY:
                raise KeyError(f"unknown experiment {key}")
            values = {**profiles.from_task(ex.BY_KEY[key], self._values_in_effect(key)), **values}
        reach_map = None
        if body.get("reach_map_run"):     # the reach from a Reach Map run: keep a record of the map with it
            data = self.reach_map(key or "reach_map", str(body["reach_map_run"]))
            if data.get("run_subject") != self.subject:
                raise ValueError("Select the subject recorded in this Reach Map before saving it to a profile.")
            reach_map = {**reach_view.record(data), "saved": time.strftime("%Y-%m-%d %H:%M")}
        prof = profiles.save(self.subject, values, by,
                             {key: task_only} if task_only else None, str(body.get("note") or ""), reach_map)
        self.profile = prof
        self._apply_profile()
        self.runner.emit(f"{self.subject}'s profile saved: v{prof['version']} (from {by})", "gui")
        self.broadcast("state", self.state())
        return {"ok": True, "profile": prof}

    def _busy(self) -> Optional[str]:
        """The experiment running now (here or in a terminal), if any."""
        if self.runner.running():
            return self.runner.experiment or "an experiment"
        st = self.status()
        return st.get("experiment") or "an experiment" if st and not st["stale"] else None

    def data_folder(self) -> Optional[str]:
        r = self.runner
        if not r.experiment or not r.started_at:
            return None
        exp = ex.BY_KEY[r.experiment]
        folders = [p for p in ex.outcome_tables.run_folders(ex.DATA, exp.key, self._run_subject)
                   if p.stat().st_mtime >= r.started_at]
        return str(max(folders, key=lambda p: p.stat().st_mtime)) if folders else None

    def _on_status(self, status: dict) -> None:
        try:
            if hasattr(self, "live_view"):
                self.live_update()      # at once, so the live view anchors at the moment of "armed"
        except Exception as exc:
            self.runner.emit(f"live view: {exc}", "warn")
        self.broadcast("state", self.state())
        # Stop: the orchestrator ends the trial at once and parks; once it reports holding there, the
        # launch ends.
        if self.stopping and status.get("held") and self.runner.running():
            self.stopping = False
            self.runner.stop("the arm is parked and holding")

    def _on_runner_state(self) -> None:
        if isinstance(self.bridge, DemoBridge) and not self.runner.running():
            self.bridge.launch_ended()
        if not self.runner.running():
            self.stopping = False
            # The run is over: add its row to sessions.csv (from the trials.csv the recorder wrote).
            folder = self.data_folder()
            if folder and folder != self._sessions_done:
                self._sessions_done = folder
                if self._run_profile:
                    _profile_copy(self, folder, self._run_profile)
                threading.Thread(target=_session_row, args=(self, folder), daemon=True).start()
            self._run_profile = None
            self._apply_profile()        # a profile saved during the run reaches this task now
        self.broadcast("state", self.state())

    # ------------------------------------------------------------------ actions

    def running_experiment(self) -> Optional[ex.Experiment]:
        """The experiment live controls address: ours if we launched it, else whatever is reporting."""
        if self.runner.running() and self.runner.experiment:
            return ex.BY_KEY[self.runner.experiment]
        s = self.status()
        if s and not s["stale"] and s.get("experiment") in ex.BY_KEY:
            return ex.BY_KEY[s["experiment"]]
        return None

    def start(self, key: str, subject: str = "") -> dict:
        exp = ex.BY_KEY[key]
        subject = check_subject(subject or self.subject)
        if subject != self.subject:
            self.select_subject(subject)
            if profiles.applies_to(exp):
                raise RuntimeError(f"Monkey set to {subject}: {'its profile is' if self.profile else 'the task defaults are'} "
                                   f"now applied. Check the settings, then press Start again.")
        if self.runner.running():
            raise RuntimeError(f"{self.runner.experiment} is already running; stop it first")
        s = self.status()
        if s and not s["stale"]:
            raise RuntimeError(f"{s.get('experiment')} is already running outside the dashboard "
                               f"({s.get('node')}). Stop it in its terminal first.")
        if not self.demo:
            # The arm must run this checkout's code: the settings come from here (install_check.py).
            stale = install_check.problem(ex.REPO)
            if stale:
                raise RuntimeError(f"Not starting: {stale}")
            # Leftovers of an earlier run (e.g. an old ros2_control_node holding the FRI connection) make
            # a new launch hang at its controller spawners: end them first. A live experiment started
            # outside the dashboard was refused above, so anything matching now is a leftover.
            self.runner.clean_leftovers()
            left = self.runner.leftovers()
            if left:
                raise RuntimeError("Could not end processes from an earlier run: "
                                   + "; ".join(f"{pid}: {cmd}" for pid, cmd in left[:4]))
        p = self.params[key]
        p.reload()                               # pick up YAML edits made since the page loaded
        problem = kinematics.floor_problem(p.current())
        if problem:                              # refuse here, with the reason, before anything launches
            raise ValueError(f"Not starting: {problem}. Change the floor or the pose on the Per-run tab.")
        overrides = p.launch_overrides()          # edited YAML (+ CLIK posture) or nothing
        params_file = overrides.get("params_file")
        if self.demo:
            cmd = [sys.executable, "-u", str(HERE / "demo_launch.py"), key]
        else:
            cmd = ["ros2", "launch", "sinthlab_bringup", exp.launch_file]
            cmd += [f"{k}:={v}" for k, v in overrides.items()]
            cmd += [f"subject:={subject}"]          # recorded with every trial (TrialRecorder)
        self.stopping = False
        logged = threading.Event()               # the start pulse's result goes into this run's log
        self._experiment_start_pulse(p.current(), logged)
        self._run_subject = subject
        self.runner.start(key, cmd, params_file)
        logged.set()
        self.runner.emit(f"Subject: {subject}" + (f" (profile {p.profile_tag()})" if p.profile_tag() else ""), "gui")
        self._run_profile = dict(self.profile) if p.profile_tag() else None
        if p.edits:
            self.runner.emit(f"Per-run edits: {json.dumps(p.edits)} -> {params_file}", "gui")
        if "clik_nullspace_cfg" in overrides:
            self.runner.emit(f"CLIK posture follows the edited start pose -> "
                             f"{overrides['clik_nullspace_cfg']}", "gui")
        if isinstance(self.bridge, DemoBridge):
            self.bridge.launch_started(exp, p.current(), subject)
        return {"ok": True}

    def _experiment_start_pulse(self, cfg: dict, logged: threading.Event) -> None:
        """With nsp_sync.enabled: pulse nsp_sync.experiment_start_output once, now -- the NSP's marker of
        when Start was pressed. Sent before the launch begins: the cabinet app opens its pulse port
        (UDP 30002) at start-up, before it waits for ROS. The trial pulses come from the orchestrator."""
        if not cfg.get("nsp_sync.enabled"):
            return
        try:
            addr = str(cfg["nsp_sync.cabinet_ip"]).strip()
            cmd = f"PULSE {int(cfg['nsp_sync.experiment_start_output'])} {int(cfg['nsp_sync.pulse_width_ms'])}"
            ttl.check(addr, cmd)
        except (KeyError, TypeError, ValueError) as exc:
            self.runner.emit(f"NSP sync: no experiment start pulse: {exc}", "warn")
            return

        def send():
            t_sent = time.time()
            ok, text, ms = self.fake_cabinet.call(cmd) if self.demo else ttl.call(addr, cmd)
            logged.wait(10.0)
            t_sent = int(t_sent * 1000) / 1000.0
            stamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t_sent)) + f".{round(t_sent % 1 * 1000) % 1000:03d}"
            if ok and text.startswith("OK"):
                self.runner.emit(f"NSP sync: experiment start pulse sent at {stamp} (t_wall {t_sent:.3f}): "
                                 f"{cmd} -> {text} ({ms:.0f} ms)", "gui")
            else:
                self.runner.emit(f"NSP sync: experiment start pulse NOT confirmed ({cmd} at {stamp}): {text}",
                                 "warn")
        threading.Thread(target=send, daemon=True).start()

    def pause(self, resume: bool = False) -> dict:
        """Pause: end the trial now, park, and hold until Resume -- the launch, FRI and the SmartPad app keep
        running. Resume starts the next trial (live changes made meanwhile apply from it)."""
        exp = self.running_experiment()
        if exp is None or not self.runner.running():
            raise RuntimeError("nothing is running")
        if self.stopping:
            raise RuntimeError("the run is stopping")
        ok, msg = (self.bridge.resume if resume else self.bridge.pause)(exp)
        self.runner.emit(f"{'Resume' if resume else 'Pause'}: {msg}", "gui" if ok else "warn")
        if not ok:
            raise RuntimeError(msg)
        return {"ok": True, "message": msg}

    def stop(self) -> dict:
        exp = self.running_experiment()
        if not self.runner.running():
            if exp:
                raise RuntimeError("This experiment was started outside the dashboard; stop it with "
                                   "Ctrl-C in its terminal.")
            raise RuntimeError("nothing is running")
        # Park first, then end the launch -- so FRI always closes with the arm parked. A second press
        # while it parks skips the wait, as does an orchestrator that does not answer.
        if self.stopping:
            self.stopping = False
            self.runner.stop("Stop pressed again: not waiting for the arm to park")
            return {"ok": True}
        ok, msg = self.bridge.stop(exp)
        if not ok:
            self.runner.emit(f"The orchestrator did not take the stop ({msg}); stopping without parking.", "warn")
            self.runner.stop()
            return {"ok": True, "message": msg}
        self.stopping = True
        self.runner.emit("Stopping: the trial ends now, the arm parks, then the launch stops "
                         "(press Stop again to skip parking).", "gui")
        run = self.runner.proc
        def fallback():
            if self.stopping and self.runner.proc is run and self.runner.running():
                self.stopping = False
                self.runner.stop(f"the arm did not report parked within {STOP_PARK_TIMEOUT_SEC:.0f} s")
        threading.Timer(STOP_PARK_TIMEOUT_SEC, fallback).start()
        return {"ok": True, "message": msg}

    def board(self, addr: str, cmd: str, params: dict, path: str = "wifi", port: str = "") -> dict:
        """Diagnostics page: one command to the end-effector cue board (board.py), over its Wi-Fi or,
        with path "serial", over the RS-422 link. A running experiment owns the serial port and reads
        the board's replies, so nothing is sent on that path then."""
        if path == "serial":
            cue_board.serial_check(port, cmd)
            busy = self._busy()
            if busy:
                raise RuntimeError(f"{busy} is running and owns the serial port. Stop it first, or use Wi-Fi.")
            ok, text, ms = (self.fake_board.call(cmd, params) if self.demo
                            else cue_board.serial_call(port, cmd, params))
        else:
            cue_board.check(addr, cmd)
            ok, text, ms = self.fake_board.call(cmd, params) if self.demo else cue_board.call(addr, cmd, params)
        return {"ok": ok, "text": text, "ms": round(ms, 1)}

    # ------------------------------------------------------------------ the run monitor and the history
    def run_trials(self, key: str) -> dict:
        """The run monitor's data: the running run's trials.csv rows, or, between runs, the latest run of
        `key` (so the monitor still shows how the last session went)."""
        exp = self.running_experiment() or ex.BY_KEY.get(key)
        if exp is None:
            return {"experiment": key, "run": None, "rows": [], "running": False}
        running = self.runner.running() and self.runner.experiment == exp.key
        if isinstance(self.bridge, DemoBridge):
            rows = self.bridge.run_rows() if getattr(self.bridge, "_exp", None) is exp else []
            return {"experiment": exp.key, "run": getattr(self.bridge, "_run", None), "rows": rows, "running": running}
        folder = self.data_folder() if running else None
        if folder is None and not running:
            runs = sorted(ex.outcome_tables.run_folders(ex.DATA, exp.key), key=lambda p: p.stat().st_mtime)
            folder = str(runs[-1]) if runs else None
        rows = ex.outcome_tables.read_trials(folder) if folder else []
        return {"experiment": exp.key, "run": Path(folder).name if folder else None, "rows": rows, "running": running}

    def history(self, key: str, subject: str) -> dict:
        """Today vs recent sessions: the last sessions of `key` for `subject`, and today's value of every
        setting sessions.csv keeps for it (dashboard edits included)."""
        exp = ex.BY_KEY.get(key)
        if exp is None:
            raise KeyError(f"unknown experiment {key}")
        p = self.params[key]
        current = p.current()
        described = {d["name"]: d for d in p.describe()}
        sessions = ex.outcome_tables.history(key, subject, 3, ex.DATA / ex.outcome_tables.SESSIONS_CSV)
        settings = []
        for name in ex.outcome_tables.settings_for(key):
            if name not in current:
                continue
            d = described.get(name, {})
            settings.append({"name": name, "today": current[name], "help": d.get("help", ""),
                             "tier": d.get("tier", ""), "edited": bool(d.get("edited")),
                             "history": [sess["settings_dict"].get(name, "") for sess in sessions]})
        for sess in sessions:
            sess.pop("settings_dict", None)
        return {"experiment": key, "subject": subject, "sessions": sessions, "settings": settings,
                "subjects": recent_subjects()}

    def reach_map(self, key: str = "reach_map", run: str = "") -> dict:
        """The Reach Map view: this run's map (or the given run's, or the latest one's)."""
        exp = ex.BY_KEY["reach_map"]
        running = self.runner.running() and self.runner.experiment == exp.key
        folder = None
        if run:
            cand = ex.DATA / run
            if cand.name != run or not ex.outcome_tables.is_run_folder(cand, exp.key):
                raise ValueError(f"no Reach Map run {run!r}")
            folder = str(cand)
        elif running:
            folder = self.data_folder()      # None until the run's first file: then it waits, not the last run's map
        if folder is None and not running:
            runs = sorted(ex.outcome_tables.run_folders(ex.DATA, exp.key), key=lambda p: p.stat().st_mtime)
            folder = str(runs[-1]) if runs else None
        params = dict(self.params[exp.key].current())
        metas = ex.outcome_tables.sidecars(folder) if folder else []
        if metas and metas[-1].get("params"):      # the run's own settings (box, sectors), where recorded
            params.update(metas[-1]["params"])
        data = reach_view.compute(folder, params, self.profile)
        data["running"] = running
        data["subject"] = self.subject
        # Whose recording this is (the subject recorded with its trials): the map can only be saved to them.
        data["run_subject"] = next((m.get("subject") for m in metas if m.get("subject")), None)
        if running and not data["run_subject"] and self.runner.experiment == exp.key:
            data["run_subject"] = self.subject or None          # the run going now, before its first trial is saved
        return data

    def ttl(self, addr: str, cmd: str) -> dict:
        """Diagnostics page: one command to the cabinet's TTL sync outputs (ttl.py). A test pulse
        during an experiment would land in its NSP recording as a trial edge, so only STATUS then."""
        cmd = " ".join(cmd.upper().split())
        ttl.check(addr, cmd)
        busy = self._busy()
        if busy and cmd != "STATUS":
            raise RuntimeError(f"{busy} is running: a test pulse would be recorded by the NSP as a trial "
                               f"edge. Stop it first.")
        ok, text, ms = self.fake_cabinet.call(cmd) if self.demo else ttl.call(addr, cmd)
        return {"ok": ok, "text": text, "ms": round(ms, 1)}


# ---------------------------------------------------------------------- HTTP


def make_handler(app: App):
    class Handler(BaseHTTPRequestHandler):
        server_version = "ExperimentCtrl/1"

        def log_message(self, fmt, *args):   # keep the terminal for the launch, not access logs
            pass

        def _json(self, obj, code=HTTPStatus.OK):
            body = json.dumps(obj, default=str).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _body(self) -> dict:
            n = int(self.headers.get("Content-Length") or 0)
            return json.loads(self.rfile.read(n) or b"{}") if n else {}

        # ---------------- GET
        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/api/events":
                return self._events()
            if path == "/api/state":
                return self._json(app.state())
            if path == "/api/experiments":
                return self._json([{
                    "key": e.key, "label": e.label, "summary": e.summary,
                    "launch_file": e.launch_file, "params_yaml": e.params_yaml, "node": e.node,
                    "task_slug": ex.outcome_tables.task_slug(e.key), "smartpad": e.smartpad(), "group": e.group,
                    "paradigms": [{"key": k, "label": lab} for k, lab in e.paradigms],
                    "paradigm_param": e.paradigm_param,
                    # Cards holding several experiments (experiments.CARDS): the card and this one's variant.
                    "card": e.card, "card_label": ex.CARDS.get(e.card, ("", ""))[0] if e.card else "",
                    "card_summary": ex.CARDS.get(e.card, ("", ""))[1] if e.card else "", "variant": e.variant,
                } for e in ex.EXPERIMENTS])
            if path == "/api/groups":
                return self._json([{"key": k, "label": v} for k, v in ex.GROUPS])
            if path == "/api/live_view":
                app.live_update()
                s = app.status()
                return self._json({"geometry": app.live_view.geometry(),
                                   "live": app.live_view.live(s if s and not s["stale"] else None)})
            if path.startswith("/api/params/"):
                key = path.rsplit("/", 1)[1]
                if key not in app.params:
                    return self._json({"error": "unknown experiment"}, HTTPStatus.NOT_FOUND)
                p = app.params[key]
                p.reload()
                return self._json({"params": p.describe(), "groups": p.groups(),
                                   "fixed": read_fixed_config(ex.BY_KEY[key], p),
                                   "start_height": _start_height(p)})
            if path == "/api/logs":
                return self._json(list(app.runner.lines))
            if path == "/api/subjects":           # for the Start dialog: most recent first
                return self._json(recent_subjects())
            if path == "/api/profile":            # the monkey, its profile, the layout, the change log
                return self._json(app.profile_info())
            if path == "/api/profile/diff":       # Save to profile: where this task differs from it
                q = parse_qs(urlparse(self.path).query)
                key = (q.get("exp") or [""])[0]
                if key not in ex.BY_KEY:
                    return self._json({"error": "unknown experiment"}, HTTPStatus.NOT_FOUND)
                return self._json({"subject": app.subject, "rows": app.profile_diff(key)})
            if path == "/api/run/trials":         # the run monitor: this (or the last) run's trials.csv
                q = parse_qs(urlparse(self.path).query)
                return self._json(app.run_trials((q.get("exp") or [""])[0]))
            if path == "/api/reward/calibration":   # the valve calibration: mL per reward (reward_volume.py)
                return self._json(reward_volume.calibration() or {"set": None, "profiles": []})
            if path == "/api/reward/width":       # the valve time for a volume per reward
                q = parse_qs(urlparse(self.path).query)
                try:
                    return self._json(reward_volume.width_for_ml(reward_volume.calibration(), float((q.get("ml") or ["0"])[0]),
                                                                 int((q.get("count") or ["1"])[0])))
                except ValueError as exc:
                    return self._json({"ok": False, "error": str(exc)}, HTTPStatus.CONFLICT)
            if path == "/api/reach_map":          # the Reach Map view
                q = parse_qs(urlparse(self.path).query)
                try:
                    return self._json(app.reach_map("reach_map", (q.get("run") or [""])[0]))
                except ValueError as exc:
                    return self._json({"error": str(exc)}, HTTPStatus.NOT_FOUND)
            if path == "/api/history":            # today vs recent sessions
                q = parse_qs(urlparse(self.path).query)
                return self._json(app.history((q.get("exp") or [""])[0], (q.get("subject") or [""])[0]))
            if path == "/api/post/state":
                return self._json(app.post.state())
            return self._static(path)

        def _static(self, path: str):
            rel = "index.html" if path in ("/", "") else path.lstrip("/")
            target = (STATIC / rel).resolve()
            if STATIC not in target.parents or not target.is_file():
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            body = target.read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", mimetypes.guess_type(target.name)[0] or "text/plain")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _events(self):
            q = app.subscribe()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(f"data: {json.dumps({'type': 'state', 'data': app.state()}, default=str)}\n\n".encode())
                self.wfile.flush()
                while True:
                    try:
                        msg = q.get(timeout=15)
                        self.wfile.write(f"data: {msg}\n\n".encode())
                    except queue.Empty:
                        self.wfile.write(b": keep-alive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                app.unsubscribe(q)

        # ---------------- POST
        def do_POST(self):
            if self.headers.get(CSRF_HEADER) != "1":
                return self._json({"error": "missing request header"}, HTTPStatus.FORBIDDEN)
            path = self.path.split("?")[0]
            try:
                body = self._body()
                return self._json(self._post(path, body))
            except (RuntimeError, ValueError, KeyError) as exc:
                # KeyError's str() wraps the message in quotes; the others do not.
                msg = exc.args[0] if isinstance(exc, KeyError) and exc.args else str(exc)
                return self._json({"ok": False, "error": str(msg)}, HTTPStatus.CONFLICT)

        def _post(self, path: str, body: dict) -> dict:
            if path == "/api/start":
                return app.start(body["experiment"], str(body.get("subject", "")))
            if path == "/api/diag/board":
                return app.board(str(body.get("addr", "")), str(body.get("cmd", "")), dict(body.get("params") or {}),
                                 str(body.get("path", "wifi")), str(body.get("port", "")))
            if path == "/api/diag/ttl":
                return app.ttl(str(body.get("addr", "")), str(body.get("cmd", "")))
            if path == "/api/stop":
                return app.stop()
            if path in ("/api/pause", "/api/resume"):
                return app.pause(resume=path == "/api/resume")
            if path == "/api/subject":
                return app.select_subject(str(body.get("subject", "")), bool(body.get("clear_edits")))
            if path == "/api/profile/save":
                return app.save_profile(body)
            if path == "/api/live":
                exp = app.running_experiment()
                if exp is None:
                    raise RuntimeError("No experiment is running. Set values for the next run under All settings.")
                p = app.params[exp.key]
                name = body["name"]
                if name not in p.defaults:
                    raise KeyError(f"unknown parameter {name}")
                from params import coerce
                value = coerce(body["value"], p.defaults[name])
                values = {name: value}
                st = app.status() or {}
                context = {**(st.get("live") or {}), **(st.get("pending") or {}), **values}
                try:        # the resistance base is what the SmartPad app reports, not the YAML
                    context["resistance.base_stiffness_n_per_m"] = float(st["smartpad"]["settings"]["stiffness"][0])
                except (KeyError, IndexError, TypeError, ValueError):
                    pass
                for n, v in values.items():
                    problem = ex.live_params.check_value(n, v, context)
                    if problem:
                        raise ValueError(problem)
                if name.startswith(kinematics.FLOOR_RELEVANT):   # the floor, with this change applied
                    problem = kinematics.floor_problem({**p.current(), **(st.get("live") or {}),
                                                        **(st.get("pending") or {}), **values})
                    if problem:
                        raise ValueError(problem)
                # Cued Reaches, free: the reach, its perturbations and the box, together (the node checks the same).
                problem = free_reach.params_problem({**p.current(), **(st.get("live") or {}),
                                                    **(st.get("pending") or {}), **values})
                if problem:
                    raise ValueError(problem)
                ok, msg = app.bridge.set_params(exp, values)
                app.runner.emit(f"Live {', '.join(values)} = {value}: {msg}", "gui" if ok else "warn")
                if not ok:
                    raise RuntimeError(msg)
                return {"ok": True, "message": msg, "value": value}
            if path.startswith("/api/start_height/"):
                key = path.rsplit("/", 1)[1]     # solve the start pose for a height, as a per-run edit
                p = app.params[key]
                if app.runner.running() and app.runner.experiment == key:
                    raise RuntimeError("Stop the experiment before editing the start pose.")
                q, report = kinematics.start_for_height(float(body["height_m"]), p.current())
                value = p.set_edit(ex.START_POSE, q)
                app.runner.emit(f"Start height {float(body['height_m']):.3f} m -> {ex.START_POSE} = {value}: "
                                f"{report}", "gui")
                app.broadcast("state", app.state())
                return {"ok": True, "value": value, "message": report}
            if path.startswith("/api/params/"):
                parts = path.split("/")          # /api/params/<key>[/reset]
                key = parts[3]
                p = app.params[key]
                if app.runner.running() and app.runner.experiment == key:
                    raise RuntimeError("per-run parameters are locked while this experiment runs")
                if len(parts) > 4 and parts[4] == "reset":
                    p.reset()
                    return {"ok": True}
                before = dict(p.edits)
                value = p.set_edit(body["name"], body["value"])
                problem = (kinematics.floor_problem(p.current()) if body["name"].startswith(kinematics.FLOOR_RELEVANT)
                           else None) or free_reach.params_problem(p.current())
                if problem:
                    p.edits = before             # refused: the edit is not kept
                    raise ValueError(problem)
                app.broadcast("state", app.state())
                return {"ok": True, "value": value}
            if path == "/api/post/scan":
                return {"ok": True, **app.post.scan(str(body.get("robot") or "").strip(), str(body.get("nsp") or "").strip())}
            if path == "/api/post/convert":
                return app.post.convert(body)
            if path == "/api/post/upload":
                return app.post.upload(body)
            if path == "/api/post/cancel":
                return app.post.cancel()
            if path == "/api/validate":
                folder = app.data_folder()
                if not folder:
                    raise RuntimeError("This run has no recording folder yet.")
                threading.Thread(target=_validate, args=(app, folder), daemon=True).start()
                return {"ok": True}
            raise KeyError(f"unknown endpoint {path}")

    return Handler


def _start_height(p) -> dict:
    """What the dashboard's start-height field shows: the floor's reference point's height at the
    start pose in effect, or why the field is unavailable."""
    ok, why = kinematics.available()
    cur = p.current()
    if not ok or ex.START_POSE not in cur or "floor.reference" not in cur:
        return {"available": False, "reason": why or "this experiment has no start pose / floor"}
    return {"available": True, "reference": cur["floor.reference"], "floor_m": cur["floor.height_m"],
            "height_m": round(kinematics.height_of(cur[ex.START_POSE], cur["floor.reference"]), 4)}


SUBJECT = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,31}$")


def check_subject(subject: str) -> str:
    """The subject ID for a Start: required; letters, digits, _ and -, up to 32 characters (it becomes a
    launch argument and a file-name part)."""
    subject = (subject or "").strip()
    if not subject:
        raise ValueError("Enter a subject ID to start (use 'test' for a run without an animal)")
    if not SUBJECT.match(subject):
        raise ValueError(f"Subject ID {subject!r}: use letters, digits, _ and - only (up to 32)")
    return subject


def recent_subjects() -> List[str]:
    """Subject IDs in sessions.csv, most recent first."""
    rows = ex.outcome_tables.read_sessions(ex.DATA / ex.outcome_tables.SESSIONS_CSV)
    return list(dict.fromkeys(r["subject"] for r in reversed(rows) if r.get("subject")))


def _session_row(app: App, folder: str) -> None:
    try:
        row = ex.outcome_tables.update_sessions(folder, ex.DATA / ex.outcome_tables.SESSIONS_CSV)
    except Exception as exc:                       # never let the summary break the dashboard
        app.runner.emit(f"sessions.csv not updated ({exc}); rebuild with analysis/outcome_tables.py", "warn")
        return
    if row:
        app.runner.emit(f"sessions.csv: {row['run']}: {row['n_trials']} trials, {row['n_success']} successful, "
                        f"{row['n_rewards']} rewards ({row['reward_ms']} ms valve time)", "gui")


def _plain_name(name: str) -> str:
    """A setting's name as the Task and Monkey views show it (experiments.LABELS, the profile's), else its YAML name."""
    if name in ex.LABELS:
        return ex.LABELS[name]
    f = profiles.BY_PARAM.get(name)
    return f"{profiles.SECTION_LABEL[f.name.split('.')[0]]}: {f.label}" if f else name


def _profile_copy(app: App, folder: str, prof: dict) -> None:
    """The profile the run started with, beside its recording (the run's parameters are in the sidecar)."""
    try:
        import yaml
        (Path(folder) / "subject_profile.yaml").write_text(
            f"# {prof['subject']}'s profile v{prof['version']} ({prof['updated']}), as this run started with it\n"
            + yaml.safe_dump({k: prof[k] for k in ("subject", "version", "updated", "values", "reward_by_task")},
                             sort_keys=False))
    except Exception as exc:
        app.runner.emit(f"profile copy not written to {folder}: {exc}", "warn")


def _validate(app: App, folder: str) -> None:
    app.runner.emit(f"Validating {folder} ...", "gui")
    proc = subprocess.run([sys.executable, str(ex.ANALYSIS / "validate_recording.py"), "--folder", folder],
                          capture_output=True, text=True)
    for line in (proc.stdout + proc.stderr).splitlines():
        app.runner.emit(f"[validate] {line}", "gui" if proc.returncode == 0 else "warn")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1",
                    help="address to serve on (default 127.0.0.1: this computer only)")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--robot-name", default="lbr", help="ROS namespace of the arm (default lbr)")
    ap.add_argument("--demo", action="store_true", help="simulate the robot; nothing is launched")
    a = ap.parse_args()

    app = App(demo=a.demo, robot_name=a.robot_name)
    httpd = ThreadingHTTPServer((a.host, a.port), make_handler(app))
    httpd.daemon_threads = True
    shown = "localhost" if a.host in ("127.0.0.1", "0.0.0.0") else a.host
    print(f"Experiment dashboard: http://{shown}:{a.port}   ({'DEMO -- simulated robot' if a.demo else 'live'})")
    if a.host not in ("127.0.0.1", "localhost"):
        print("WARNING: serving beyond this computer. Anyone who can reach this port can start the robot.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if app.runner.running():
            print("Stopping the running experiment (Ctrl-C to the launch)...")
            app.runner.stop("dashboard shutting down")
            try:
                app.runner.proc.wait(timeout=30)
            except Exception:
                pass
        app.bridge.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
