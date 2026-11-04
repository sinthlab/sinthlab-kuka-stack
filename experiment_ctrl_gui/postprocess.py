"""Post-processing page (postprocess.html): archive a session as NWB, then upload it to the server.

The conversion itself is analysis/nwb_export.py; this module runs it (and the upload) as one
background job at a time, and reports progress on the dashboard's event stream as {"type": "post"}.
Nothing here moves the arm, but a conversion reads and writes gigabytes, so it is refused while an
experiment runs.
"""
from __future__ import annotations

import hashlib
import os
import shutil
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Optional

import experiments as ex

sys.path.insert(0, str(ex.ANALYSIS))
import nwb_export as nx  # noqa: E402

_BROADCAST_SEC = 0.25          # progress events at most this often
_COPY_BLOCK = 8 << 20          # upload copy block, 8 MiB


class Cancelled(Exception):
    pass


class PostProcessor:
    def __init__(self, broadcast: Callable[[str, object], None], log: Callable[[str, str], None],
                 busy: Callable[[], Optional[str]]) -> None:
        self._broadcast, self._log, self._busy = broadcast, log, busy
        self._lock = threading.Lock()
        self._job: Optional[dict] = None
        self._cancel = threading.Event()
        self._last_sent = 0.0

    # ------------------------------------------------------------------ queries

    def state(self) -> dict:
        return {"deps": nx.missing_dependencies(), "runs": self.runs(), "job": self._job}

    @staticmethod
    def runs() -> list:
        """Local run folders, newest first."""
        out = []
        for p in sorted(ex.ANALYSIS.glob("expt_*"), key=lambda p: p.stat().st_mtime, reverse=True):
            if p.is_dir():
                out.append({"path": str(p), "name": p.name, "trials": len(list(p.glob("robot_trajectory_*.csv")))})
        return out

    def scan(self, robot: str, nsp: str) -> dict:
        """What the two recordings hold, before converting: each part, or why it cannot be read."""
        out: dict = {"robot": None, "nsp": None}
        if robot:
            try:
                out["robot"] = nx.describe_robot(nx.load_robot(robot))
            except nx.ExportError as e:
                out["robot"] = {"error": str(e)}
        if nsp:
            problem = nx.missing_dependencies()
            if problem:
                out["nsp"] = {"error": problem}
            else:
                try:
                    out["nsp"] = {**nx.describe_nsp(nx.open_nsp(nsp)), "base": nx.find_nsp_base(nsp)}
                except Exception as e:      # neo raises its own errors for malformed files
                    out["nsp"] = {"error": str(e)}
        return out

    # ------------------------------------------------------------------ jobs

    def convert(self, body: dict) -> dict:
        problem = nx.missing_dependencies()
        if problem:
            raise RuntimeError(problem)
        robot = str(body.get("robot") or "").strip()
        out = str(body.get("out") or "").strip()
        if not robot or not out:
            raise ValueError("the robot run folder and the output file are required")
        if not out.endswith(".nwb"):
            raise ValueError("the output file must end in .nwb")
        meta = nx.Metadata.from_dict(dict(body.get("meta") or {}))
        nsp = str(body.get("nsp") or "").strip() or None
        sync = [str(c).strip() for c in (body.get("sync_channels") or ["ainp6", "ainp7"])]
        exp_start = str(body.get("experiment_start_channel", "ainp8") or "").strip() or None
        args = dict(robot_folder=robot, out_path=out, meta=meta, nsp_path=nsp, sync_channels=sync,
                    experiment_start_channel=exp_start,
                    threshold_v=float(body.get("threshold_v", 2.5)),
                    include_continuous=bool(body.get("include_continuous", True)),
                    include_spikes=bool(body.get("include_spikes", True)))
        return self._start("convert", f"Archiving {os.path.basename(robot.rstrip('/'))} to {out}",
                           lambda say: nx.export(**args, progress=say))

    def upload(self, body: dict) -> dict:
        src = os.path.abspath(os.path.expanduser(str(body.get("nwb") or "")))
        dest_dir = os.path.abspath(os.path.expanduser(str(body.get("dest") or "")))
        if not os.path.isfile(src):
            raise ValueError(f"NWB file not found: {src}")
        if not str(body.get("dest") or "").strip() or not os.path.isdir(dest_dir):
            raise ValueError(f"server folder not found (is it mounted?): {dest_dir}")
        dest = os.path.join(dest_dir, os.path.basename(src))
        if os.path.exists(dest) and not body.get("overwrite"):
            raise ValueError(f"{dest} already exists")
        return self._start("upload", f"Uploading {os.path.basename(src)} to {dest_dir}",
                           lambda say: _copy_verified(src, dest, say))

    def cancel(self) -> dict:
        if not self._job or self._job["state"] != "running":
            raise RuntimeError("nothing is running")
        self._cancel.set()
        return {"ok": True}

    def _start(self, kind: str, title: str, work: Callable[[Callable[[str, float], None]], dict]) -> dict:
        with self._lock:
            if self._job and self._job["state"] == "running":
                raise RuntimeError(f"a {self._job['kind']} is already running")
            running = self._busy()
            if running:
                raise RuntimeError(f"{running} is running: post-processing waits until it stops")
            self._cancel.clear()
            self._job = {"kind": kind, "title": title, "state": "running", "message": "Starting",
                         "fraction": -1, "started": time.time(), "result": None, "error": None}
        self._log(title, "gui")
        self._send(force=True)
        threading.Thread(target=self._run, args=(work,), daemon=True).start()
        return {"ok": True}

    def _run(self, work) -> None:
        job = self._job
        try:
            job["result"] = work(self._progress)
            job["state"], job["message"], job["fraction"] = "done", "Done", 1.0
            self._log(f"{job['title']}: done", "gui")
        except Cancelled:
            job["state"], job["error"] = "cancelled", "cancelled"
            self._log(f"{job['title']}: cancelled", "warn")
        except Exception as e:      # the page shows it; the server keeps running
            job["state"], job["error"] = "error", str(e) or type(e).__name__
            self._log(f"{job['title']}: {job['error']}", "error")
        job["finished"] = time.time()
        self._send(force=True)

    def _progress(self, message: str, fraction: float) -> None:
        if self._cancel.is_set():
            raise Cancelled()
        self._job["message"], self._job["fraction"] = message, fraction
        self._send()

    def _send(self, force: bool = False) -> None:
        now = time.monotonic()
        if force or now - self._last_sent >= _BROADCAST_SEC:
            self._last_sent = now
            self._broadcast("post", self._job)


def _copy_verified(src: str, dest: str, say: Callable[[str, float], None]) -> dict:
    """Copy to dest (via dest.partial), then read the copy back and compare SHA-256."""
    size = os.path.getsize(src)
    tmp = dest + ".partial"
    h_src = hashlib.sha256()
    done = 0
    with open(src, "rb") as fi, open(tmp, "wb") as fo:
        while True:
            block = fi.read(_COPY_BLOCK)
            if not block:
                break
            fo.write(block)
            h_src.update(block)
            done += len(block)
            say("Copying", done / max(size, 1) * 0.5)
        fo.flush()
        os.fsync(fo.fileno())
    h_dest = hashlib.sha256()
    done = 0
    with open(tmp, "rb") as f:
        while True:
            block = f.read(_COPY_BLOCK)
            if not block:
                break
            h_dest.update(block)
            done += len(block)
            say("Verifying the copy", 0.5 + done / max(size, 1) * 0.5)
    if h_src.hexdigest() != h_dest.hexdigest():
        os.remove(tmp)
        raise RuntimeError("the copy on the server does not match the local file (removed): try again")
    shutil.move(tmp, dest)
    return {"dest": dest, "bytes": size, "sha256": h_src.hexdigest()}
