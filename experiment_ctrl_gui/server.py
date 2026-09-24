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
import subprocess
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, List, Optional

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import experiments as ex  # noqa: E402
from params import ExperimentParams, read_fixed_config  # noqa: E402
from ros_bridge import DemoBridge, make_bridge  # noqa: E402
from runner import LaunchRunner  # noqa: E402

STATIC = HERE / "static"
STATUS_STALE_SEC = 3.0         # the orchestrator heartbeats every 1 s
CSRF_HEADER = "X-Experiment-Ctrl"   # a cross-site page cannot send a custom header without CORS


class App:
    def __init__(self, demo: bool, robot_name: str) -> None:
        self.demo = demo
        self.robot_name = robot_name
        self.params: Dict[str, ExperimentParams] = {e.key: ExperimentParams(e) for e in ex.EXPERIMENTS}
        self._clients: List[queue.Queue] = []
        self._clients_lock = threading.Lock()
        self.stop_after_trial = False
        self.restart_pending: Optional[str] = None
        self.runner = LaunchRunner(on_line=lambda e: self.broadcast("log", e),
                                   on_state=self._on_runner_state)
        self.bridge = make_bridge(demo, robot_name, self._on_status)
        if isinstance(self.bridge, DemoBridge):
            self.bridge.attach_log(self.runner.emit)
        if not self.bridge.available:
            self.runner.emit(f"ROS not available: {self.bridge.reason}", "warn")
        threading.Thread(target=self._ticker, daemon=True).start()

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
            "stop_after_trial": self.stop_after_trial,
            "data_folder": self.data_folder(),
            "edited": {k: len(p.edits) for k, p in self.params.items()},
        }

    def data_folder(self) -> Optional[str]:
        r = self.runner
        if not r.experiment or not r.started_at:
            return None
        exp = ex.BY_KEY[r.experiment]
        if not exp.run_name:
            return None
        folders = [p for p in ex.ANALYSIS.glob(f"expt_{exp.run_name}_*")
                   if p.is_dir() and p.stat().st_mtime >= r.started_at - 5]
        return str(max(folders, key=lambda p: p.stat().st_mtime)) if folders else None

    def _on_status(self, status: dict) -> None:
        self.broadcast("state", self.state())
        # "Stop after this trial" = pause, then stop once the orchestrator reports it is holding.
        if self.stop_after_trial and status.get("held") and self.runner.running():
            self.stop_after_trial = False
            self.runner.stop("the trial finished and the arm is holding at the start")

    def _on_runner_state(self) -> None:
        if isinstance(self.bridge, DemoBridge) and not self.runner.running():
            self.bridge.launch_ended()
        if not self.runner.running():
            self.stop_after_trial = False
            if self.restart_pending:
                key, self.restart_pending = self.restart_pending, None
                threading.Timer(1.0, lambda: self._safe_start(key)).start()
        self.broadcast("state", self.state())

    def _safe_start(self, key: str) -> None:
        try:
            self.start(key)
        except Exception as exc:
            self.runner.emit(f"Restart failed: {exc}", "error")

    # ------------------------------------------------------------------ actions

    def running_experiment(self) -> Optional[ex.Experiment]:
        """The experiment live controls address: ours if we launched it, else whatever is reporting."""
        if self.runner.running() and self.runner.experiment:
            return ex.BY_KEY[self.runner.experiment]
        s = self.status()
        if s and not s["stale"] and s.get("experiment") in ex.BY_KEY:
            return ex.BY_KEY[s["experiment"]]
        return None

    def start(self, key: str) -> dict:
        exp = ex.BY_KEY[key]
        if self.runner.running():
            raise RuntimeError(f"{self.runner.experiment} is already running; stop it first")
        s = self.status()
        if s and not s["stale"]:
            raise RuntimeError(f"{s.get('experiment')} is already running outside the dashboard "
                               f"({s.get('node')}). Stop it in its terminal first.")
        p = self.params[key]
        p.reload()                               # pick up YAML edits made since the page loaded
        overrides = p.launch_overrides()          # edited YAML (+ CLIK posture) or nothing
        params_file = overrides.get("params_file")
        if self.demo:
            cmd = [sys.executable, "-u", str(HERE / "demo_launch.py"), key]
        else:
            cmd = ["ros2", "launch", "sinthlab_bringup", exp.launch_file]
            cmd += [f"{k}:={v}" for k, v in overrides.items()]
        self.stop_after_trial = False
        self.runner.start(key, cmd, params_file)
        if p.edits:
            self.runner.emit(f"Per-run edits: {json.dumps(p.edits)} -> {params_file}", "gui")
        if "clik_nullspace_cfg" in overrides:
            self.runner.emit(f"CLIK posture follows the edited start pose -> "
                             f"{overrides['clik_nullspace_cfg']}", "gui")
        if isinstance(self.bridge, DemoBridge):
            self.bridge.launch_started(exp, p.current())
        return {"ok": True}

    def stop(self, mode: str) -> dict:
        exp = self.running_experiment()
        if not self.runner.running():
            if exp:
                raise RuntimeError("This experiment was started outside the dashboard; stop it with "
                                   "Ctrl-C in its terminal. (Pause works from here.)")
            raise RuntimeError("nothing is running")
        if mode == "after_trial":
            ok, msg = self.bridge.pause(exp, True)
            if not ok:
                raise RuntimeError(f"could not request a stop after this trial: {msg}")
            self.stop_after_trial = True
            self.runner.emit("Will stop once this trial ends and the arm is back at the start.", "gui")
            return {"ok": True, "message": msg}
        self.runner.stop()
        return {"ok": True}

    def restart(self) -> dict:
        if not self.runner.running():
            raise RuntimeError("nothing is running")
        self.restart_pending = self.runner.experiment
        self.runner.stop("restart")
        return {"ok": True}


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
                    "run_name": e.run_name, "smartpad": e.smartpad(),
                } for e in ex.EXPERIMENTS])
            if path.startswith("/api/params/"):
                key = path.rsplit("/", 1)[1]
                if key not in app.params:
                    return self._json({"error": "unknown experiment"}, HTTPStatus.NOT_FOUND)
                p = app.params[key]
                p.reload()
                return self._json({"params": p.describe(), "groups": p.groups(),
                                   "fixed": read_fixed_config(ex.BY_KEY[key], p)})
            if path == "/api/logs":
                return self._json(list(app.runner.lines))
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
                return app.start(body["experiment"])
            if path == "/api/stop":
                return app.stop(body.get("mode", "now"))
            if path == "/api/restart":
                return app.restart()
            if path == "/api/pause":
                exp = app.running_experiment()
                if exp is None:
                    raise RuntimeError("no experiment is running")
                ok, msg = app.bridge.pause(exp, bool(body.get("paused")))
                if not ok:
                    raise RuntimeError(msg)
                if not body.get("paused"):
                    app.stop_after_trial = False
                app.runner.emit(f"{'Pause' if body.get('paused') else 'Resume'}: {msg}", "gui")
                return {"ok": True, "message": msg}
            if path == "/api/live":
                exp = app.running_experiment()
                if exp is None:
                    raise RuntimeError("no experiment is running -- edit it under Per-run instead")
                p = app.params[exp.key]
                name = body["name"]
                if name not in p.defaults:
                    raise KeyError(f"unknown parameter {name}")
                from params import coerce
                value = coerce(body["value"], p.defaults[name])
                problem = ex.live_params.check_value(name, value)
                if problem:
                    raise ValueError(problem)
                ok, msg = app.bridge.set_param(exp, name, value)
                app.runner.emit(f"Live {name} = {value}: {msg}", "gui" if ok else "warn")
                if not ok:
                    raise RuntimeError(msg)
                return {"ok": True, "message": msg, "value": value}
            if path.startswith("/api/params/"):
                parts = path.split("/")          # /api/params/<key>[/reset]
                key = parts[3]
                p = app.params[key]
                if app.runner.running() and app.runner.experiment == key:
                    raise RuntimeError("per-run parameters are locked while this experiment runs")
                if len(parts) > 4 and parts[4] == "reset":
                    p.reset()
                    return {"ok": True}
                value = p.set_edit(body["name"], body["value"])
                app.broadcast("state", app.state())
                return {"ok": True, "value": value}
            if path == "/api/validate":
                folder = app.data_folder()
                if not folder:
                    raise RuntimeError("no recording folder from this run yet")
                threading.Thread(target=_validate, args=(app, folder), daemon=True).start()
                return {"ok": True}
            raise KeyError(f"unknown endpoint {path}")

    return Handler


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
