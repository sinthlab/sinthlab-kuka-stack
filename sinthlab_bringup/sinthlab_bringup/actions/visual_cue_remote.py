#!/usr/bin/env python3
"""Wi-Fi TEST trigger for the visual cue -- for demos and recordings, not experiments.

VisualCue uses this instead of the X76 wire when `visual_cue.remote_test_trigger` is true:

    this computer ── joined to the board's KUKA_NEOPIXEL access point ──► GET http://192.168.4.1/cue

That is the same call as `curl http://192.168.4.1/cue`. The board runs whatever cue /config holds --
colour, pattern, duration -- exactly as if the wire had fired. No X76, no relay.

A cue site with a colour configured (visual_cue.colours.<label>) makes TWO requests: first
`/config?r=..&g=..&b=..&w=..` sets the colour, then `/cue` runs the cue. That uses only what the
firmware already on the board provides, so a sealed effector never needs reflashing. The colour is
set in RAM -- there is no save=1, so nothing is written to flash -- and the board keeps it until the
next cue sets another, or until a reboot restores its saved colour.

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
from typing import Optional, Tuple

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
        self._jobs.put((("/status",), None, time.monotonic()))

    def fire(self, label: str, colour: Optional[Tuple[int, int, int, int]] = None) -> None:
        """Run the board's cue -- in `colour` (r, g, b, w) if given. Returns immediately."""
        if colour is None:
            paths = ("/cue",)
        else:   # set the colour (RAM only: never save=1), then fire -- see the module doc
            paths = ("/config?r={}&g={}&b={}&w={}".format(*colour), "/cue")
        self._jobs.put((paths, label, time.monotonic()))

    def _run(self) -> None:
        while True:
            paths, label, queued_at = self._jobs.get()
            try:
                self._send(paths, label, queued_at)
            except Exception as exc:    # this thread must outlive anything a request can throw
                self._log.warn(f"Visual cue Wi-Fi trigger: unexpected error, carrying on: {exc!r}")

    def _send(self, paths: Tuple[str, ...], label: Optional[str], queued_at: float) -> None:
        waited = time.monotonic() - queued_at
        if label is not None and waited > STALE_SEC:
            self._log.warn(
                f"Visual cue '{label}' DROPPED: it waited {waited * 1000:.0f} ms behind a slow "
                "request and would have lit late.")
            return

        sent_at = time.monotonic()
        for path in paths:      # in order; a failure stops the job, so a cue never fires in a stale colour
            url = f"http://{self._board}{path}"
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
