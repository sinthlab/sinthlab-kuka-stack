"""Starts and stops one `ros2 launch` at a time, and keeps its output for the log panel.

Stop is Ctrl-C, exactly as in a terminal: SIGINT to the launch's whole process group, so ros2 launch
shuts every node down in order and the recorder closes its files. Only if that hangs does it
escalate to SIGTERM and then SIGKILL. Stop is NOT an emergency stop -- the SmartPad E-stop is.
"""
from __future__ import annotations

import os
import re
import signal
import subprocess
import threading
import time
from collections import deque
from pathlib import Path
from typing import Callable, List, Optional, Tuple

LOG_DIR = Path(__file__).resolve().parent / "logs"
_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
_LEVEL = re.compile(r"\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]")

SIGINT_GRACE_SEC = 20.0
# Command-line fragments of the processes a launch starts; any still running blocks the next launch.
LEFTOVER_MARKERS = ("ros2_control_node", "robot_state_publisher", "controller_manager/spawner",
                    "lib/controller_manager/spawner", "_orchestrator.py", "launch sinthlab_bringup")
SIGTERM_GRACE_SEC = 5.0
CLEANUP_GRACE_SEC = 3.0   # per signal, when ending processes a run left behind


def classify(line: str) -> str:
    m = _LEVEL.search(line)
    if not m:
        return "info"
    lvl = m.group(1)
    return {"WARNING": "warn", "FATAL": "error"}.get(lvl, lvl.lower())


class LaunchRunner:
    def __init__(self, on_line: Callable[[dict], None], on_state: Callable[[], None]) -> None:
        self._on_line = on_line
        self._on_state = on_state
        self._lock = threading.Lock()
        self.proc: Optional[subprocess.Popen] = None
        self.state = "idle"              # idle | running | stopping
        self.experiment: Optional[str] = None
        self.started_at: Optional[float] = None
        self.command: Optional[List[str]] = None
        self.params_file: Optional[str] = None
        self.exit_code: Optional[int] = None
        self.log_path: Optional[Path] = None
        self.lines: deque = deque(maxlen=5000)
        self._seq = 0
        self._log_fh = None

    # ------------------------------------------------------------------ control

    @staticmethod
    def _procs() -> List[Tuple[int, int, str]]:
        """(pid, process group, command line) of every process we can see, except this one."""
        out = []
        for d in Path("/proc").iterdir():
            if not d.name.isdigit() or int(d.name) == os.getpid():
                continue
            try:
                stat = (d / "stat").read_text()
                pgrp = int(stat[stat.rindex(")") + 2:].split()[2])     # after "(comm) state ppid"
                cmd = (d / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace").strip()
            except (OSError, ValueError, IndexError):
                continue
            out.append((int(d.name), pgrp, cmd))
        return out

    def _end_all(self, pids: List[int], why: str) -> None:
        """Ctrl-C, then SIGTERM, then SIGKILL, to these processes until none is left."""
        for sig, grace in ((signal.SIGINT, CLEANUP_GRACE_SEC), (signal.SIGTERM, CLEANUP_GRACE_SEC),
                           (signal.SIGKILL, 1.0)):
            alive = [p for p in pids if Path(f"/proc/{p}").exists()]
            if not alive:
                return
            self.emit(f"Cleaning up {why}: {sig.name} to {', '.join(map(str, alive))}", "warn")
            for p in alive:
                try:
                    os.kill(p, sig)
                except (ProcessLookupError, PermissionError):
                    pass
            end = time.time() + grace
            while time.time() < end and any(Path(f"/proc/{p}").exists() for p in alive):
                time.sleep(0.1)

    def reap_group(self, pgid: int) -> None:
        """End whatever is left of a finished launch: its nodes keep its process group after ros2 launch
        itself exits, and one still holding the FRI connection blocks the next run."""
        left = [pid for pid, grp, _ in self._procs() if grp == pgid]
        if left:
            self._end_all(left, "processes the launch left behind")

    def clean_leftovers(self) -> None:
        """End processes of any earlier run -- a dashboard run, or one started from a terminal."""
        left = self.leftovers()
        if left:
            self._end_all([pid for pid, _ in left], "processes from an earlier run")

    @staticmethod
    def leftovers() -> List[Tuple[int, str]]:
        """Processes from an earlier run that would block a new one: a ros2_control_node still holding
        the FRI port (the new launch then never connects -- its controller spawners wait forever), an
        orchestrator, a spawner, a robot_state_publisher. (pid, command line) for each."""
        return [(pid, cmd[:160]) for pid, _, cmd in LaunchRunner._procs()
                if any(k in cmd for k in LEFTOVER_MARKERS)]

    def start(self, experiment: str, command: List[str], params_file: Optional[str]) -> None:
        with self._lock:
            if self.proc is not None and self.proc.poll() is None:
                raise RuntimeError(f"{self.experiment} is already running; stop it first")
            LOG_DIR.mkdir(exist_ok=True)
            self.log_path = LOG_DIR / f"{experiment}_{time.strftime('%Y%m%d_%H%M%S')}.log"
            self._log_fh = open(self.log_path, "w", buffering=1)
            self.experiment, self.command, self.params_file = experiment, command, params_file
            self.started_at, self.exit_code = time.time(), None
            self.emit(f"$ {' '.join(command)}", "cmd")
            # Own process group, so one signal reaches ros2 launch AND every node it spawned.
            self.proc = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                text=True, bufsize=1, start_new_session=True,
                env={**os.environ, "RCUTILS_COLORIZED_OUTPUT": "0", "PYTHONUNBUFFERED": "1"})
            self.state = "running"
        threading.Thread(target=self._pump, args=(self.proc,), daemon=True).start()
        threading.Thread(target=self._watch, args=(self.proc,), daemon=True).start()
        self._on_state()

    def stop(self, reason: str = "stopped from the dashboard") -> None:
        with self._lock:
            proc = self.proc
            if proc is None or proc.poll() is not None or self.state == "stopping":
                return
            self.state = "stopping"
        self.emit(f"Stopping ({reason}): Ctrl-C to the launch...", "gui")
        self._on_state()
        threading.Thread(target=self._escalate, args=(proc,), daemon=True).start()

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    # ------------------------------------------------------------------ internals

    def _signal(self, proc: subprocess.Popen, sig: int) -> None:
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            pass

    def _escalate(self, proc: subprocess.Popen) -> None:
        self._signal(proc, signal.SIGINT)
        for sig, grace in ((signal.SIGTERM, SIGINT_GRACE_SEC), (signal.SIGKILL, SIGTERM_GRACE_SEC)):
            try:
                proc.wait(timeout=grace)
                break
            except subprocess.TimeoutExpired:
                self.emit(f"Launch still running after {grace:.0f} s; sending {sig.name}.", "warn")
                self._signal(proc, sig)
        self.reap_group(proc.pid)        # its nodes may outlive ros2 launch itself

    def _watch(self, proc: subprocess.Popen) -> None:
        """However the launch ends -- stopped, finished, crashed -- end the nodes it left behind. Waits on
        the launch process itself: a leftover holding its output pipe would keep _pump reading forever."""
        proc.wait()
        time.sleep(CLEANUP_GRACE_SEC)    # nodes still shutting down normally get to finish first
        self.reap_group(proc.pid)

    def _pump(self, proc: subprocess.Popen) -> None:
        for raw in proc.stdout:
            line = _ANSI.sub("", raw.rstrip("\n"))
            if line:
                self.emit(line)
        code = proc.wait()
        with self._lock:
            self.exit_code = code
            self.state = "idle"
        self.emit(f"Launch exited with code {code}.", "gui" if code in (0, -2, 130) else "error")
        if self._log_fh:
            self._log_fh.close()
            self._log_fh = None
        self._on_state()

    def emit(self, text: str, level: Optional[str] = None) -> None:
        self._seq += 1
        entry = {"seq": self._seq, "t": time.time(), "level": level or classify(text), "text": text}
        self.lines.append(entry)
        if self._log_fh:
            try:
                self._log_fh.write(text + "\n")
            except ValueError:
                pass
        self._on_line(entry)
