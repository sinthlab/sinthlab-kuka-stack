#!/usr/bin/env python3
"""Wi-Fi TEST trigger for the visual cue -- for demos and recordings, not experiments.

VisualCue uses this instead of the X76 wire when `visual_cue.remote_test_trigger` is true:

    this computer ── joined to the board's KUKA_NEOPIXEL access point ──► GET http://192.168.4.1/cue

That is the same call as `curl http://192.168.4.1/cue`. The board runs whatever cue /config holds --
colour, pattern, duration -- exactly as if the wire had fired. No X76, no relay.

NOT FOR TRIAL TIMING. The Wi-Fi delay varies from cue to cue, and a dropped link drops the cue. Each
cue's round-trip time is logged so you can see it, but align trial data to the wire, not to this.

GUARANTEES
    * start() never waits on the network: requests are sent from one background thread.
    * Nothing here raises into the orchestrator. A failed request logs a warning; the trial goes on.
    * One request at a time, because the board serves one at a time. A cue that waited longer than
      STALE_SEC behind a slow request is dropped and logged rather than lit late.
"""
from __future__ import annotations

import http.client
import queue
import threading
import time
import urllib.request
from typing import Optional

from rclpy.node import Node as rclpyNode

DEFAULT_BOARD = "192.168.4.1"   # the board's own access point always puts it here
TIMEOUT_SEC = 1.0               # per request
STALE_SEC = 0.5                 # a cue older than this when its turn comes is dropped, never lit late

_HINT = "Is this computer joined to the board's Wi-Fi (KUKA_NEOPIXEL)?"


class RemoteCueTrigger:
    """Sends GET /cue to the board from one background thread. Share one instance per process."""

    def __init__(self, node: rclpyNode, board: str = DEFAULT_BOARD) -> None:
        self._log = node.get_logger()
        self._board = board
        self._jobs: queue.SimpleQueue = queue.SimpleQueue()
        threading.Thread(target=self._run, name="visual_cue_remote", daemon=True).start()
        self._log.info(
            f"Visual cues will fire over Wi-Fi via http://{board}/cue -- a TEST trigger for demos, "
            "not for trial timing.")

    def check(self) -> None:
        """Ask the board for /status and log whether it answered. Returns immediately."""
        self._jobs.put(("/status", None, time.monotonic()))

    def fire(self, label: str) -> None:
        """Run the board's configured cue. Returns immediately."""
        self._jobs.put(("/cue", label, time.monotonic()))

    def _run(self) -> None:
        while True:
            path, label, queued_at = self._jobs.get()
            try:
                self._send(path, label, queued_at)
            except Exception as exc:    # this thread must outlive anything a request can throw
                self._log.warn(f"Visual cue Wi-Fi trigger: unexpected error, carrying on: {exc!r}")

    def _send(self, path: str, label: Optional[str], queued_at: float) -> None:
        url = f"http://{self._board}{path}"
        waited = time.monotonic() - queued_at
        if label is not None and waited > STALE_SEC:
            self._log.warn(
                f"Visual cue '{label}' DROPPED: it waited {waited * 1000:.0f} ms behind a slow "
                "request and would have lit late.")
            return

        sent_at = time.monotonic()
        try:
            with urllib.request.urlopen(url, timeout=TIMEOUT_SEC) as reply:
                reply.read()
        except (OSError, http.client.HTTPException, ValueError) as exc:
            # OSError covers unreachable, refused and timed out; ValueError a malformed remote_board.
            what = f"Visual cue '{label}' NOT shown" if label else "Visual cue board NOT reachable"
            self._log.warn(f"{what}: {url} failed ({exc}). {_HINT}")
            return

        ms = (time.monotonic() - sent_at) * 1000
        if label is None:
            self._log.info(f"Visual cue board reachable at {self._board} ({ms:.0f} ms).")
        else:
            self._log.info(f"Visual cue '{label}' fired over Wi-Fi ({ms:.0f} ms round trip).")
