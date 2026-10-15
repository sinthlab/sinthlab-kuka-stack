#!/usr/bin/env python3
"""What the SmartPad app (LbrImpedanceControlServer) is actually running with -- reported by the app.

FRI carries robot state only, not the app's own settings, so the app sends them itself: once its
set-up dialogs are answered it sends one JSON line over UDP to this computer, port SETTINGS_PORT, once
a second until it ends:

    {"app": "LbrImpedanceControlServer", "stiffness_profile": "Uniform 100",
     "stiffness": [100, 100, 100, 300, 300, 300], "damping": 0.7, "nullspace_stiffness": 30,
     "nullspace_damping": 0.7, "send_period_ms": 10, "tool": "SinthLabIiwa7EE"}

SmartPadLink listens in a background thread and keeps the latest report -- what the app last said it
runs with. The selections cannot change during a run, so the report never goes out of date; it is
repeated only so it gets through (ROS starts after the app, and UDP can drop a packet).

It is written ONCE PER RUN to `smartpad_params.json` in the run's folder (write_to: TrialRecorder
names the folder) -- rewritten when a report arrives, so it holds the report however late in the run
it came; until then it says `received: false` and why. The experiment status carries it too (the
dashboard shows it). Best effort: if nothing arrives (an old app, the network, a firewall), everything
runs as before.

One listener per process (the port can only be bound once): use SmartPadLink.get(logger).
"""
from __future__ import annotations

import json
import os
import socket
import threading
from typing import Optional

# Must equal SETTINGS_PORT in sunrise_controller_code/LbrImpedanceControlServer.java.
SETTINGS_PORT = 30001


class SmartPadLink:
    _instance: Optional["SmartPadLink"] = None

    @classmethod
    def get(cls, logger) -> "SmartPadLink":
        if cls._instance is None:
            cls._instance = cls(logger)
        return cls._instance

    def __init__(self, logger) -> None:
        self._log = logger
        self._lock = threading.Lock()
        self._settings: Optional[dict] = None
        self._sender: Optional[str] = None
        self._error: Optional[str] = None
        self._out: Optional[str] = None     # the run's smartpad_params.json (write_to)
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.bind(("", SETTINGS_PORT))
        except OSError as exc:
            self._error = f"cannot listen on UDP {SETTINGS_PORT}: {exc}"
            self._log.warn(f"SmartPad settings: {self._error}. Trials will record them as unavailable.")
            return
        threading.Thread(target=self._listen, name="smartpad-link", daemon=True).start()
        self._log.info(f"SmartPad settings: listening on UDP {SETTINGS_PORT} for LbrImpedanceControlServer.")

    def _listen(self) -> None:
        first = True
        while True:
            try:
                data, (host, _port) = self._sock.recvfrom(4096)
                settings = json.loads(data.decode("utf-8"))
            except (OSError, ValueError, UnicodeDecodeError):
                continue
            with self._lock:
                changed = settings != self._settings
                self._settings, self._sender = settings, host
            if first or changed:
                first = False
                self._log.info(f"SmartPad settings from {host}: {settings}")
                self._write()

    def write_to(self, run_dir: str) -> None:
        """Keep `<run_dir>/smartpad_params.json` up to date: written now (received or not) and again
        when a report arrives."""
        self._out = os.path.join(run_dir, "smartpad_params.json")
        self._write()

    def _write(self) -> None:
        if self._out is None:
            return
        try:
            os.makedirs(os.path.dirname(self._out), exist_ok=True)
            with open(self._out, "w") as f:
                json.dump(self.snapshot(), f, indent=2)
        except OSError as exc:
            self._log.warn(f"SmartPad settings: could not write {self._out}: {exc}")

    def snapshot(self) -> dict:
        """The latest report, for the sidecar and status: `received`, `from`, `settings` -- or
        `received: false` and why there is none."""
        with self._lock:
            if self._settings is None:
                return {"received": False, "reason": self._error or "no report from the SmartPad app yet"}
            return {"received": True, "from": self._sender, "settings": dict(self._settings)}

    def translational_stiffness(self) -> Optional[float]:
        """The running profile's X stiffness [N/m] if a report has it, else None."""
        snap = self.snapshot()
        try:
            return float(snap["settings"]["stiffness"][0]) if snap["received"] else None
        except (KeyError, IndexError, TypeError, ValueError):
            return None
