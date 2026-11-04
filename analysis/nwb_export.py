#!/usr/bin/env python3
"""Archive a recording session as one NWB file: the robot's trials and, when given, the Blackrock NSP
recording, aligned by the TTL sync pulses the cabinet sends to the NSP.

    python3 nwb_export.py --robot analysis/expt_iiwa7_apple_pluck_..._20261026_165155 \
        --nsp /mnt/server/blackrock/20261026/datafile001.ns5 \
        --subject-id M1 --species "Macaca mulatta" --sex M --age P8Y \
        --description "apple pluck, uniform 400" --out session.nwb

The dashboard's Post-processing page (experiment_ctrl_gui/postprocess.html) runs the same code.

INPUTS
  robot  a run folder written by the experiments: one robot_trajectory_<ts>.csv + .meta.json per
         trial, and smartpad_params.json. Local.
  nsp    a Blackrock recording: any one of its files (.nev, .ns1-.ns6) or the folder holding it; the
         other files of the same recording are found by name. May be on a network mount. Optional:
         without it the NWB holds the robot data alone, on the robot's clock.

SYNC  (reward_and_sync_box/README.md, "Sync: TTL pulses to the NSP")
  The cabinet pulses EL2124 output 1 at every trial_start and output 2 at every trial_end; the NSP
  records them on two analog inputs (default ainp6, ainp7; set per recording). Rising edges on those channels are matched to
  the robot's trial_start / trial_end events (their wall-clock times), and a straight line
  t_nsp = slope * t_wall + intercept is fitted to the matched pairs (offset and clock drift). Every
  robot time in the file is converted with it, so the whole file runs on the NSP's clock.
  Output 3 pulses once when Start is pressed on the dashboard (experiment start); its edges (default
  ainp8) are stored as they are, not used for the fit. A missing experiment start channel is noted, not
  an error.

NWB CONTENTS
  session start   the NSP recording's start (robot only: the first robot sample)
  subject         from the arguments (subject id, species, sex, age or date of birth, ...)
  acquisition     ElectricalSeries per NSx stream (neural channels, raw int16 with their gains);
                  TimeSeries per NSx stream for the analog inputs (sync lines included)
  units           NEV spike times, one row per channel and unit
  processing/behavior   end-effector position and orientation, joint positions (measured and
                  commanded), external joint torques, the FRI state, every other numeric column, and
                  the robot events
  processing/sync the matched sync edges and the fit, and the experiment start edges
  trials          one row per trial: start/stop from trial_start/trial_end, experiment, paradigm,
                  threshold, the sidecar as JSON
  notes           the SmartPad settings and the git state of the run, as JSON

Needs pynwb and neo (with numpy < 2, which ROS's scipy needs):
    pip install --user --break-system-packages pynwb neo "numpy<2"
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import glob
import json
import os
import re
import sys
import uuid
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

Progress = Callable[[str, float], None]          # (message, fraction 0..1 or -1 for unknown)

NSX_EXTS = (".ns1", ".ns2", ".ns3", ".ns4", ".ns5", ".ns6")
ANALOG_INPUT = re.compile(r"^ainp\d+$", re.IGNORECASE)
SYNC_LINES = ("trial_start", "trial_end")        # EL2124 output 1, output 2


class ExportError(ValueError):
    """Something the person running the export has to fix; the message says what."""


def missing_dependencies() -> Optional[str]:
    """None if pynwb and neo import, else what to install."""
    missing = []
    for mod in ("pynwb", "neo"):
        try:
            __import__(mod)
        except Exception:       # ImportError, or a numpy 2 / scipy mismatch
            missing.append(mod)
    if not missing:
        return None
    return (f"{' and '.join(missing)} not installed: "
            "pip install --user --break-system-packages pynwb neo \"numpy<2\"")


# ---------------------------------------------------------------------------------------- robot

@dataclass
class RobotTrial:
    csv_path: str
    meta: dict
    columns: List[str]
    numeric: Dict[str, np.ndarray]           # every numeric column
    events: List[Tuple[float, str, str]]     # (t_wall, token, arg), one per token


@dataclass
class RobotSession:
    folder: str
    trials: List[RobotTrial]
    smartpad: Optional[dict]

    @property
    def session_id(self) -> str:
        ids = {t.meta.get("session_id") for t in self.trials if t.meta.get("session_id")}
        return ids.pop() if len(ids) == 1 else os.path.basename(self.folder.rstrip("/"))

    @property
    def experiment(self) -> str:
        names = {t.meta.get("experiment") for t in self.trials if t.meta.get("experiment")}
        return ", ".join(sorted(names)) or "unknown"

    def events(self, token: str) -> np.ndarray:
        return np.array(sorted(t for tr in self.trials for t, tok, _ in tr.events if tok == token))

    def t_range(self) -> Tuple[float, float]:
        """First and last time of the run: samples and events."""
        ev = [t for tr in self.trials for t, _, _ in tr.events]
        return (min([float(t.numeric["t_wall"][0]) for t in self.trials] + ev),
                max([float(t.numeric["t_wall"][-1]) for t in self.trials] + ev))


def _float(s: str) -> float:
    try:
        return float(s)
    except ValueError:
        return float("nan")


def _trial_events(rows: List[dict], meta: dict) -> List[Tuple[float, str, str]]:
    """The sidecar's event list (exact times) if it has one, else the CSV's event column."""
    out = []
    if isinstance(meta.get("events"), list) and meta["events"]:
        for e in meta["events"]:
            if "t_wall" not in e or not e.get("event"):
                continue
            for tok in str(e["event"]).split("|"):
                out.append((float(e["t_wall"]), tok, "" if e.get("arg") is None else str(e["arg"])))
        return out
    for r in rows:
        if r.get("event"):
            for tok in r["event"].split("|"):
                out.append((_float(r["t_wall"]), tok, r.get("event_arg") or ""))
    return out


def load_robot(folder: str) -> RobotSession:
    folder = os.path.abspath(os.path.expanduser(folder))
    if not os.path.isdir(folder):
        raise ExportError(f"robot run folder not found: {folder}")
    csvs = sorted(glob.glob(os.path.join(folder, "robot_trajectory_*.csv")))
    if not csvs:
        raise ExportError(f"no robot_trajectory_*.csv in {folder}")
    trials = []
    for path in csvs:
        meta_path = path[:-4] + ".meta.json"
        meta = {}
        if os.path.isfile(meta_path):
            with open(meta_path) as f:
                meta = json.load(f)
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            columns = list(reader.fieldnames or [])
            rows = list(reader)
        if not rows or "t_wall" not in columns:
            continue                     # an empty or foreign file: nothing to archive
        numeric = {}
        for c in columns:
            if c in ("event", "event_arg"):
                continue
            vals = np.array([_float(r[c]) for r in rows])
            if not np.all(np.isnan(vals)) or c == "disp_m":
                numeric[c] = vals
        trials.append(RobotTrial(path, meta, columns, numeric, _trial_events(rows, meta)))
    if not trials:
        raise ExportError(f"no usable trials in {folder}")
    trials.sort(key=lambda t: t.numeric["t_wall"][0])
    smartpad = None
    sp = os.path.join(folder, "smartpad_params.json")
    if os.path.isfile(sp):
        with open(sp) as f:
            smartpad = json.load(f)
    return RobotSession(folder, trials, smartpad)


def describe_robot(s: RobotSession) -> dict:
    t0, t1 = s.t_range()
    return {"folder": s.folder, "session_id": s.session_id, "experiment": s.experiment,
            "trials": len(s.trials), "samples": int(sum(len(t.numeric["t_wall"]) for t in s.trials)),
            "duration_s": round(t1 - t0, 1),
            "start": dt.datetime.fromtimestamp(t0, dt.timezone.utc).isoformat(),
            "trial_start_events": int(len(s.events("trial_start"))),
            "trial_end_events": int(len(s.events("trial_end")))}


# ------------------------------------------------------------------------------------------ NSP

def find_nsp_base(path: str) -> str:
    """The recording's base path (no extension), from one of its files or the folder holding it."""
    path = os.path.abspath(os.path.expanduser(path))
    if os.path.isfile(path):
        base, ext = os.path.splitext(path)
        if ext.lower() not in NSX_EXTS + (".nev",):
            raise ExportError(f"not a Blackrock file (.nev / .ns1-.ns6): {path}")
        return base
    if not os.path.isdir(path):
        raise ExportError(f"NSP recording not found: {path}")
    bases = sorted({os.path.splitext(p)[0] for p in os.listdir(path)
                    if os.path.splitext(p)[1].lower() in NSX_EXTS + (".nev",)})
    if not bases:
        raise ExportError(f"no Blackrock files (.nev / .ns1-.ns6) in {path}")
    if len(bases) > 1:
        raise ExportError(f"{len(bases)} recordings in {path} ({', '.join(bases)}): choose one of their files")
    return os.path.join(path, bases[0])


def open_nsp(path: str):
    """neo's BlackrockRawIO for the recording, header parsed."""
    from neo.rawio import BlackrockRawIO
    base = find_nsp_base(path)
    files = [base + e for e in NSX_EXTS + (".nev",) if os.path.isfile(base + e)]
    if not any(f.endswith(NSX_EXTS) for f in files):
        raise ExportError(f"{base}: no continuous data (.ns1-.ns6)")
    raw = BlackrockRawIO(filename=base, nsx_to_load="all", load_nev=os.path.isfile(base + ".nev"))
    raw.parse_header()
    return raw


def _units_to_volts(units: str) -> float:
    u = (units or "").strip().lower().replace("µ", "u")
    return {"uv": 1e-6, "mv": 1e-3, "v": 1.0, "nv": 1e-9}.get(u, 1.0)


def _nsx_header_time(base: str) -> Optional[dt.datetime]:
    """TimeOrigin (UTC) from an NSx 2.2+ basic header: neo reads it only from the .nev."""
    import struct
    for ext in NSX_EXTS:
        try:
            with open(base + ext, "rb") as f:
                head = f.read(310)
        except OSError:
            continue
        if head[:8] != b"NEURALCD" or len(head) < 310:
            continue
        y, mo, _, d, h, mi, sec, ms = struct.unpack("<8H", head[294:310])
        try:
            return dt.datetime(y, mo, d, h, mi, sec, ms * 1000, tzinfo=dt.timezone.utc)
        except ValueError:
            return None
    return None


def nsp_start_time(raw) -> Optional[dt.datetime]:
    """When the NSP recording started (UTC), from the .nev or, failing that, the NSx header."""
    try:
        t = raw.raw_annotations["blocks"][0]["rec_datetime"]
    except (KeyError, IndexError, TypeError):
        t = None
    if isinstance(t, dt.datetime):
        return t if t.tzinfo else t.replace(tzinfo=dt.timezone.utc)
    return _nsx_header_time(os.path.splitext(raw.filename)[0] if os.path.splitext(raw.filename)[1] else raw.filename)


def nsp_streams(raw) -> List[dict]:
    """Per signal stream: its channels, split into neural and analog inputs."""
    sig = raw.header["signal_channels"]
    out = []
    for si, st in enumerate(raw.header["signal_streams"]):
        idx = [i for i in range(len(sig)) if sig[i]["stream_id"] == st["id"]]
        names = [str(sig[i]["name"]) for i in idx]
        rate = float(sig[idx[0]]["sampling_rate"]) if idx else 0.0
        segs = []
        for seg in range(raw.header["nb_segment"][0]):
            segs.append({"t_start": float(raw.get_signal_t_start(0, seg, si)),
                         "n": int(raw.get_signal_size(0, seg, si))})
        local = list(range(len(idx)))
        out.append({"index": si, "name": str(st["name"]), "id": str(st["id"]), "rate": rate,
                    "channels": names, "segments": segs,
                    "neural": [k for k in local if not ANALOG_INPUT.match(names[k])],
                    "analog": [k for k in local if ANALOG_INPUT.match(names[k])],
                    "gain": [float(sig[i]["gain"]) for i in idx], "offset": [float(sig[i]["offset"]) for i in idx],
                    "units": [str(sig[i]["units"]) for i in idx], "ids": [str(sig[i]["id"]) for i in idx]})
    return out


def describe_nsp(raw) -> dict:
    t0 = nsp_start_time(raw)
    streams = nsp_streams(raw)
    return {"start": t0.isoformat() if t0 else None,
            "segments": int(raw.header["nb_segment"][0]),
            "streams": [{"name": s["name"], "rate_hz": s["rate"], "neural_channels": len(s["neural"]),
                         "analog_inputs": [s["channels"][k] for k in s["analog"]],
                         "duration_s": round(sum(g["n"] for g in s["segments"]) / s["rate"], 1) if s["rate"] else 0}
                        for s in streams],
            "spike_units": int(len(raw.header["spike_channels"])),
            "event_channels": int(len(raw.header["event_channels"]))}


def find_channel(streams: List[dict], name: str) -> Tuple[dict, int]:
    """The stream (highest rate first) and local index of a channel, by name (case-insensitive)."""
    for s in sorted(streams, key=lambda s: -s["rate"]):
        for k, n in enumerate(s["channels"]):
            if n.lower() == name.lower():
                return s, k
    raise ExportError(f"channel {name} is not in the NSP recording "
                      f"(analog inputs: {', '.join(sorted({s['channels'][k] for s in streams for k in s['analog']})) or 'none'})")


def rising_edges(raw, stream: dict, k: int, threshold_v: float, progress: Optional[Progress] = None,
                 dead_time_s: float = 0.001, chunk_s: float = 20.0) -> np.ndarray:
    """Times [s, NSP clock] where the channel crosses threshold_v going up, every segment."""
    scale = stream["gain"][k] * _units_to_volts(stream["units"][k])
    off = stream["offset"][k] * _units_to_volts(stream["units"][k])
    rate = stream["rate"]
    step = max(1, int(chunk_s * rate))
    total = sum(g["n"] for g in stream["segments"]) or 1
    done, edges = 0, []
    for seg, g in enumerate(stream["segments"]):
        prev_high = None
        last = -np.inf
        for i0 in range(0, g["n"], step):
            i1 = min(g["n"], i0 + step)
            x = raw.get_analogsignal_chunk(0, seg, i0, i1, stream["index"], channel_indexes=[k])[:, 0]
            high = (x.astype(np.float64) * scale + off) >= threshold_v
            before = np.concatenate(([prev_high if prev_high is not None else high[0]], high[:-1]))
            for j in np.flatnonzero(high & ~before):
                t = g["t_start"] + (i0 + j) / rate
                if t - last >= dead_time_s:
                    edges.append(t)
                    last = t
            prev_high = bool(high[-1])
            done += i1 - i0
            if progress:
                progress(f"Finding sync edges on {stream['channels'][k]}", done / total)
    return np.array(edges)


# ----------------------------------------------------------------------------------------- sync

@dataclass
class Sync:
    slope: float
    intercept: float
    pairs: Dict[str, List[Tuple[float, float]]]      # line -> [(t_wall, t_nsp)]
    rms_ms: float
    max_ms: float
    unmatched_robot: Dict[str, int] = field(default_factory=dict)
    unmatched_nsp: Dict[str, int] = field(default_factory=dict)

    def nsp_time(self, t_wall):
        return self.slope * np.asarray(t_wall, dtype=np.float64) + self.intercept

    def summary(self) -> dict:
        return {"matched": {k: len(v) for k, v in self.pairs.items()},
                "unmatched_robot_events": self.unmatched_robot, "unmatched_nsp_edges": self.unmatched_nsp,
                "drift_ppm": round((self.slope - 1.0) * 1e6, 2), "rms_ms": round(self.rms_ms, 3),
                "max_ms": round(self.max_ms, 3), "slope": self.slope, "intercept": self.intercept}


def _match(robot: Dict[str, np.ndarray], nsp: Dict[str, np.ndarray], slope: float, intercept: float,
           tol: float) -> Dict[str, List[Tuple[float, float]]]:
    """Each robot event paired with the nearest NSP edge on its line, if within tol (each edge once)."""
    out = {}
    for line, rt in robot.items():
        nt = np.sort(np.asarray(nsp.get(line, []), dtype=np.float64))
        if not len(nt) or not len(rt):
            out[line] = []
            continue
        pred = slope * np.asarray(rt, dtype=np.float64) + intercept
        i = np.clip(np.searchsorted(nt, pred), 1, len(nt) - 1) if len(nt) > 1 else np.zeros(len(pred), int)
        j = np.where(np.abs(nt[i - 1] - pred) <= np.abs(nt[i] - pred), i - 1, i) if len(nt) > 1 else i
        ok = np.abs(nt[j] - pred) <= tol
        _, first = np.unique(j[ok], return_index=True)
        keep = np.flatnonzero(ok)[first]
        out[line] = [(float(rt[k]), float(nt[j[k]])) for k in sorted(keep)]
    return out


def align(robot: Dict[str, np.ndarray], nsp: Dict[str, np.ndarray], coarse_tol: float = 0.5,
          tol: float = 0.05) -> Sync:
    """Fit t_nsp = slope * t_wall + intercept from the sync edges (see the module doc).

    The offset is found by trying every robot/NSP pairing of the first edges on each line and keeping
    the one under which the most events line up (within coarse_tol); a straight line is then fitted
    and the events re-matched within tol."""
    cands = []
    for line in robot:
        for t in robot[line][:20]:
            for n in nsp.get(line, np.array([])):
                cands.append(float(n - t))
    if not cands:
        raise ExportError("no sync edges to match: " + ", ".join(
            f"{line}: {len(robot[line])} robot events, {len(nsp.get(line, []))} NSP edges" for line in robot))
    sample = {line: (rt if len(rt) <= 100 else rt[np.linspace(0, len(rt) - 1, 100).astype(int)])
              for line, rt in robot.items()}          # scoring on a sample keeps long sessions fast
    best, best_key = None, None
    for off in np.unique(np.round(cands, 3)):
        m = _match(sample, nsp, 1.0, off, coarse_tol)
        n = sum(len(v) for v in m.values())
        resid = sum(abs(b - a - off) for v in m.values() for a, b in v)
        key = (n, -resid)
        if best_key is None or key > best_key:
            best, best_key = off, key
    slope, intercept = 1.0, best
    for _ in range(3):
        m = _match(robot, nsp, slope, intercept, coarse_tol if _ == 0 else tol)
        pts = np.array([p for v in m.values() for p in v])
        if len(pts) >= 2 and np.ptp(pts[:, 0]) > 1.0:
            slope, intercept = np.polyfit(pts[:, 0], pts[:, 1], 1)
        elif len(pts):
            intercept = float(np.mean(pts[:, 1] - pts[:, 0]))
    m = _match(robot, nsp, slope, intercept, tol)
    pts = np.array([p for v in m.values() for p in v])
    n_robot = sum(len(v) for v in robot.values())
    if len(pts) < 2 or len(pts) < 0.5 * min(n_robot, sum(len(v) for v in nsp.values())):
        raise ExportError(
            f"the sync edges do not line up with the robot events ({len(pts)} matched of {n_robot} robot "
            "events): check the NSP recording belongs to this run and that the sync channels are right")
    res = (pts[:, 1] - (slope * pts[:, 0] + intercept)) * 1e3
    return Sync(float(slope), float(intercept), m, float(np.sqrt(np.mean(res ** 2))), float(np.max(np.abs(res))),
                {line: int(len(robot[line]) - len(m[line])) for line in robot},
                {line: int(len(nsp.get(line, [])) - len(m[line])) for line in robot})


# ------------------------------------------------------------------------------------------ NWB

@dataclass
class Metadata:
    subject_id: str
    species: str = "Macaca mulatta"
    sex: str = "U"
    age: str = ""                    # ISO 8601 duration, e.g. P8Y
    date_of_birth: str = ""          # YYYY-MM-DD, if no age
    weight: str = ""                 # e.g. 9.5 kg
    subject_description: str = ""
    session_description: str = ""
    experimenter: str = ""           # "Last, First"; several separated by ;
    lab: str = ""
    institution: str = ""
    electrode_location: str = ""     # brain area of the array
    electrode_description: str = ""  # the array
    notes: str = ""

    @classmethod
    def from_dict(cls, d: dict) -> "Metadata":
        known = {k: str(v).strip() for k, v in d.items()
                 if k in cls.__dataclass_fields__ and v is not None and str(v).strip()}
        if not known.get("subject_id"):
            raise ExportError("subject id is required")
        return cls(**known)


def _nsx_iterator(raw, stream: dict, seg: int, chans: List[int], scaled: bool, progress: Optional[Progress],
                  label: str):
    from hdmf.data_utils import GenericDataChunkIterator

    n = stream["segments"][seg]["n"]
    scale = np.array([stream["gain"][k] * _units_to_volts(stream["units"][k]) for k in chans])
    off = np.array([stream["offset"][k] * _units_to_volts(stream["units"][k]) for k in chans])

    class _It(GenericDataChunkIterator):
        def _get_data(self, selection):
            t, c = selection
            cols = chans[c] if isinstance(c, slice) else [chans[i] for i in np.atleast_1d(c)]
            x = raw.get_analogsignal_chunk(0, seg, t.start or 0, t.stop if t.stop is not None else n,
                                           stream["index"], channel_indexes=cols)
            if progress:
                progress(label, (t.stop if t.stop is not None else n) / max(n, 1))
            if scaled:
                return (x.astype(np.float64) * scale[c] + off[c]).astype(np.float32)
            return x

        def _get_maxshape(self):
            return (n, len(chans))

        def _get_dtype(self):
            return np.dtype(np.float32 if scaled else np.int16)

    return _It(chunk_shape=(min(n, 30000), len(chans)), buffer_gb=0.25, display_progress=False)


def export(robot_folder: str, out_path: str, meta: Metadata, nsp_path: Optional[str] = None,
           sync_channels: Sequence[str] = ("ainp6", "ainp7"), threshold_v: float = 2.5,
           include_continuous: bool = True, include_spikes: bool = True,
           progress: Optional[Progress] = None, experiment_start_channel: Optional[str] = "ainp8") -> dict:
    """Write the NWB file; returns a summary (sizes, trials, sync fit)."""
    from dateutil.tz import tzutc  # noqa: F401  (pynwb needs tz-aware times; dateutil ships with it)
    from hdmf.backends.hdf5.h5_utils import H5DataIO
    from pynwb import NWBHDF5IO, NWBFile, TimeSeries
    from pynwb.behavior import Position, SpatialSeries
    from pynwb.ecephys import ElectricalSeries
    from pynwb.file import Subject

    say = progress or (lambda m, f: None)
    say("Reading the robot run", -1)
    robot = load_robot(robot_folder)
    out_path = os.path.abspath(os.path.expanduser(out_path))
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    raw, sync, streams = None, None, []
    exp_start, exp_start_note = None, None        # experiment start edges [s, NSP clock], or why not
    if nsp_path:
        say("Opening the NSP recording", -1)
        raw = open_nsp(nsp_path)
        streams = nsp_streams(raw)
        edges = {}
        for line, ch in zip(SYNC_LINES, sync_channels):
            s, k = find_channel(streams, ch)
            edges[line] = rising_edges(raw, s, k, threshold_v, say)
        if experiment_start_channel:
            try:
                s, k = find_channel(streams, experiment_start_channel)
                exp_start = rising_edges(raw, s, k, threshold_v, say)
            except ExportError as exc:
                exp_start_note = f"experiment start: {exc}"
                say(exp_start_note, -1)
        say("Aligning the robot to the NSP clock", -1)
        sync = align({line: robot.events(line) for line in SYNC_LINES}, edges)
        start = nsp_start_time(raw)
        if start is None:                     # NSP time 0 in wall-clock time, from the fit
            start = dt.datetime.fromtimestamp(-sync.intercept / sync.slope, dt.timezone.utc)
        to_file = sync.nsp_time
    else:
        t0 = robot.t_range()[0]
        start = dt.datetime.fromtimestamp(t0, dt.timezone.utc)
        to_file = lambda t: np.asarray(t, dtype=np.float64) - t0   # noqa: E731

    subject = Subject(subject_id=meta.subject_id, species=meta.species or None, sex=meta.sex or "U",
                      age=meta.age or None, description=meta.subject_description or None,
                      weight=meta.weight or None,
                      date_of_birth=(dt.datetime.fromisoformat(meta.date_of_birth).replace(tzinfo=dt.timezone.utc)
                                     if meta.date_of_birth else None))
    first = robot.trials[0].meta
    notes = {"robot_run_folder": robot.folder, "nsp_recording": find_nsp_base(nsp_path) if nsp_path else None,
             "smartpad": robot.smartpad, "git": first.get("git"), "experiment": robot.experiment,
             "sync": sync.summary() if sync else None, "notes": meta.notes or None,
             "experiment_start_edges_s": exp_start.tolist() if exp_start is not None else exp_start_note}
    nwb = NWBFile(session_description=meta.session_description or f"{robot.experiment} session {robot.session_id}",
                  identifier=str(uuid.uuid4()), session_start_time=start, session_id=robot.session_id,
                  experimenter=[e.strip() for e in meta.experimenter.split(";") if e.strip()] or None,
                  lab=meta.lab or None, institution=meta.institution or None, subject=subject,
                  notes=json.dumps(notes, default=str))

    # ---- robot: one continuous set of series over all trials
    say("Writing the robot data", -1)
    cat = lambda col: np.concatenate([t.numeric[col] for t in robot.trials])   # noqa: E731
    cols = [c for c in robot.trials[0].numeric if all(c in t.numeric for t in robot.trials)]
    ts = to_file(cat("t_wall"))
    behavior = nwb.create_processing_module("behavior", "The robot (KUKA LBR iiwa7) during the session")
    clock = None

    def series(name, columns, unit, description, cls=TimeSeries, **kw):
        nonlocal clock
        present = [c for c in columns if c in cols]
        if not present:
            return
        data = np.column_stack([cat(c) for c in present]) if len(present) > 1 else cat(present[0])
        s = cls(name=name, data=data, unit=unit, timestamps=clock if clock is not None else ts,
                description=f"{description} Columns: {', '.join(present)}.", **kw)
        clock = clock if clock is not None else s
        return s

    pos = series("end_effector_position", ["x", "y", "z"], "meters",
                 "End-effector (lbr_link_ee) position in the robot base frame.", cls=SpatialSeries,
                 reference_frame="robot base, lbr_link_0")
    if pos is not None:
        behavior.add(Position(spatial_series=pos))
    groups = [("end_effector_orientation", ["qx", "qy", "qz", "qw"], "n.a.", "End-effector orientation quaternion."),
              ("joint_position_measured", [f"meas_A{i}" for i in range(1, 8)], "radians", "Measured joint angles A1-A7."),
              ("joint_position_commanded", [f"cmd_A{i}" for i in range(1, 8)], "radians", "Commanded joint angles A1-A7."),
              ("joint_torque_external", [f"ext_A{i}" for i in range(1, 8)], "newton-meters",
               "External joint torques A1-A7 as estimated by the cabinet."),
              ("fri_state", ["session", "quality", "safety", "drive", "control"], "n.a.", "FRI state codes."),
              ("fri_clock", ["fri_s", "fri_ns"], "n.a.", "Cabinet (FRI) timestamp, seconds and nanoseconds."),
              ("wall_clock", ["t_wall"], "seconds", "Host wall-clock time of each sample (Unix)."),
              ("ros_clock", ["t_ros"], "seconds", "ROS time of each sample.")]
    used = {"t"} | {c for _, cs, _, _ in groups for c in cs} | {"x", "y", "z"}
    for name, cs, unit, desc in groups:
        s = series(name, cs, unit, desc)
        if s is not None:
            behavior.add(s)
    for c in cols:                               # everything else, one series each
        if c not in used:
            s = series(c, [c], "n.a." if c not in ("disp_m",) else "meters",
                       "Robot recording column '%s' (see the experiment's recorder)." % c)
            if s is not None:
                behavior.add(s)
    ev = sorted((t, tok, arg) for tr in robot.trials for t, tok, arg in tr.events)
    if ev:
        table = nwb.create_events_table(name="robot_events", source_description="Experiment orchestrator",
                                        description="Experiment events, as the orchestrator emitted them.")
        table.add_column("event", "Event token (trial_start, at_start, armed, snap, trial_end, ...).")
        table.add_column("argument", "The event's argument (e.g. the trial number), or empty.")
        for t, tok, arg in ev:
            table.add_event(timestamp=float(to_file(t)), event=tok, argument=arg)

    # ---- trials
    for col, desc in (("trial_index", "Trial number in the run."), ("experiment", "Experiment."),
                      ("paradigm", "Paradigm."), ("threshold_m", "Displacement threshold [m], if any."),
                      ("partial", "The trial's recording was cut short."),
                      ("sidecar_json", "The trial's sidecar (.meta.json) as written by the recorder.")):
        nwb.add_trial_column(col, desc)
    for i, tr in enumerate(robot.trials):
        tw = tr.numeric["t_wall"]
        starts = [t for t, tok, _ in tr.events if tok == "trial_start"]
        ends = [t for t, tok, _ in tr.events if tok == "trial_end"]
        t_start = float(to_file(starts[0] if starts else tw[0]))
        t_stop = float(to_file(ends[-1] if ends else tw[-1]))
        thr = tr.meta.get("threshold_m")
        nwb.add_trial(start_time=t_start, stop_time=max(t_stop, t_start),
                      trial_index=int(tr.meta.get("trial_index", i + 1)), experiment=str(tr.meta.get("experiment", "")),
                      paradigm=str(tr.meta.get("paradigm", "")),
                      threshold_m=float(thr) if isinstance(thr, (int, float)) else float("nan"),
                      partial=bool(tr.meta.get("partial", False)),
                      sidecar_json=json.dumps(tr.meta, default=str))

    # ---- sync
    if sync:
        mod = nwb.create_processing_module("sync", "Robot-to-NSP clock alignment by the TTL sync pulses")
        for line, pairs in sync.pairs.items():
            if pairs:
                p = np.array(pairs)
                mod.add(TimeSeries(name=f"{line}_edges", data=p[:, 0], unit="seconds", timestamps=p[:, 1],
                                   description=(f"Matched {line} pulses: data = robot wall-clock time (Unix s), "
                                                f"timestamps = NSP rising edge. Fit: t_nsp = {sync.slope!r} * t_wall "
                                                f"+ {sync.intercept!r}; rms {sync.rms_ms:.3f} ms.")))
        if exp_start is not None and len(exp_start):
            mod.add(TimeSeries(name="experiment_start_edges", data=exp_start, unit="seconds", timestamps=exp_start,
                               description=f"Rising edges on {experiment_start_channel}: the experiment start pulse "
                                           f"(Start pressed on the dashboard), NSP clock. Not used for the fit."))

    # ---- NSP
    if raw is not None:
        device = nwb.create_device(name="Blackrock NSP", description="Blackrock Microsystems Neural Signal Processor")
        group = nwb.create_electrode_group(name="array", description=meta.electrode_description or "recording array",
                                           location=meta.electrode_location or "unknown", device=device)
        rows: Dict[str, int] = {}
        for s in streams:
            for k in s["neural"]:
                name = s["channels"][k]
                if name not in rows:
                    nwb.add_electrode(group=group, location=meta.electrode_location or "unknown",
                                      group_name="array")
                    rows[name] = len(rows)
        if rows:
            nwb.electrodes.add_column("channel_name", "NSP channel label",
                                      data=sorted(rows, key=rows.get))
        if include_continuous:
            for s in streams:
                for seg, g in enumerate(s["segments"]):
                    if not g["n"]:
                        continue
                    sfx = f"_{s['name']}" + (f"_seg{seg}" if len(s["segments"]) > 1 else "")
                    sfx = re.sub(r"[^A-Za-z0-9_]", "_", sfx)
                    if s["neural"]:
                        label = f"Writing neural data {s['name']} ({s['rate']:g} Hz)"
                        region = nwb.create_electrode_table_region([rows[s["channels"][k]] for k in s["neural"]],
                                                                   "neural channels")
                        u = _units_to_volts(s["units"][s["neural"][0]])
                        nwb.add_acquisition(ElectricalSeries(
                            name="ElectricalSeries" + sfx, electrodes=region, rate=s["rate"], starting_time=g["t_start"],
                            data=H5DataIO(_nsx_iterator(raw, s, seg, s["neural"], False, say, label),
                                          compression="gzip", compression_opts=4),
                            conversion=u, channel_conversion=[s["gain"][k] for k in s["neural"]],
                            description=f"Neural channels of {s['name']}: {', '.join(s['channels'][k] for k in s['neural'])}."))
                    if s["analog"]:
                        label = f"Writing analog inputs {s['name']}"
                        nwb.add_acquisition(TimeSeries(
                            name="AnalogInputs" + sfx, unit="volts", rate=s["rate"], starting_time=g["t_start"],
                            data=H5DataIO(_nsx_iterator(raw, s, seg, s["analog"], True, say, label),
                                          compression="gzip", compression_opts=4),
                            description=f"NSP analog inputs of {s['name']}: {', '.join(s['channels'][k] for k in s['analog'])}"
                                        f" (sync: {sync_channels[0]} = trial_start, {sync_channels[1]} = trial_end"
                                        + (f", {experiment_start_channel} = experiment start" if experiment_start_channel else "")
                                        + ")."))
        if include_spikes and len(raw.header["spike_channels"]):
            say("Writing spikes", -1)
            nwb.add_unit_column("nsp_unit", "NEV spike channel (channel#unit)")
            for i, sc in enumerate(raw.header["spike_channels"]):
                times = []
                for seg in range(raw.header["nb_segment"][0]):
                    stamps = raw.get_spike_timestamps(0, seg, i, None, None)
                    times.append(raw.rescale_spike_timestamp(stamps, dtype="float64"))
                nwb.add_unit(spike_times=np.sort(np.concatenate(times)) if times else [], nsp_unit=str(sc["name"]))
        if len(raw.header["event_channels"]):
            labels, stamps = [], []
            for i, ec in enumerate(raw.header["event_channels"]):
                for seg in range(raw.header["nb_segment"][0]):
                    t, _, lab = raw.get_event_timestamps(0, seg, i, None, None)
                    stamps.append(raw.rescale_event_timestamp(t, dtype="float64"))
                    labels += [f"{ec['name']}:{x}" for x in lab]
            if labels:
                all_t = np.concatenate(stamps)
                table = nwb.create_events_table(name="nsp_events", source_description="Blackrock NSP (NEV)",
                                                description="NEV digital and comment events.")
                table.add_column("value", "Event channel and value (channel:value).")
                for j in np.argsort(all_t):
                    table.add_event(timestamp=float(all_t[j]), value=labels[j])

    say("Saving the NWB file", -1)
    tmp = os.path.splitext(out_path)[0] + ".partial.nwb"
    try:
        with NWBHDF5IO(tmp, "w") as io:
            io.write(nwb)
    except BaseException:            # failed or cancelled: leave no half-written file behind
        if os.path.exists(tmp):
            os.remove(tmp)
        raise
    os.replace(tmp, out_path)
    say("Done", 1.0)
    return {"out": out_path, "bytes": os.path.getsize(out_path), "trials": len(robot.trials),
            "session_start": start.isoformat(), "time_base": "NSP clock" if sync else "robot wall clock",
            "sync": sync.summary() if sync else None,
            "experiment_start_edges_s": exp_start.tolist() if exp_start is not None else exp_start_note,
            "robot": describe_robot(robot),
            "nsp": describe_nsp(raw) if raw is not None else None}


# ------------------------------------------------------------------------------------------ CLI

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--robot", required=True, help="robot run folder (analysis/expt_...)")
    ap.add_argument("--nsp", help="Blackrock recording: one of its files, or its folder")
    ap.add_argument("--out", required=True, help="NWB file to write")
    ap.add_argument("--sync-channels", nargs=2, default=["ainp6", "ainp7"], metavar=("START", "END"),
                    help="NSP analog inputs of the trial start and trial end lines")
    ap.add_argument("--experiment-start-channel", default="ainp8",
                    help="NSP analog input of the experiment start line ('' = none)")
    ap.add_argument("--threshold-v", type=float, default=2.5)
    ap.add_argument("--no-continuous", action="store_true", help="leave the NSx continuous data out")
    ap.add_argument("--no-spikes", action="store_true", help="leave the NEV spikes out")
    for f in Metadata.__dataclass_fields__:
        ap.add_argument("--" + f.replace("_", "-"), dest=f, default="", help=f"metadata: {f}")
    a = ap.parse_args()
    problem = missing_dependencies()
    if problem:
        print(problem, file=sys.stderr)
        return 2
    try:
        meta = Metadata.from_dict({f: getattr(a, f) for f in Metadata.__dataclass_fields__})
        result = export(a.robot, a.out, meta, a.nsp, a.sync_channels, a.threshold_v, not a.no_continuous,
                        not a.no_spikes, progress=lambda m, f: print(f"  {m}" + (f" {f * 100:.0f}%" if f >= 0 else ""),
                                                                     file=sys.stderr),
                        experiment_start_channel=a.experiment_start_channel or None)
    except ExportError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    print(json.dumps(result, indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
