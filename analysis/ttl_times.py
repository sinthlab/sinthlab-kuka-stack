#!/usr/bin/env python3
"""When the TTL pulses arrived at the NSP, and when the robot sent them.

    python3 ttl_times.py <robot run folder> <blackrock recording: .ns6 (or other .nsX) file>

1) NSP:   rising edges on ainp2 / ainp3 / ainp4 (trial start / trial end / experiment start). The NSP
          counts samples from the start of the recording; the file header holds the date and time the
          recording started (TimeOrigin, by the Central computer's clock), so each edge is shown as
          start + seconds into the recording.
2) Robot: when SYNC 1 / SYNC 2 / SYNC 3 / SYNC 4 were sent, by the robot computer's clock: the
          trial_start / trial_end events in the trial sidecars, the run's experiment_start.json (runs
          recorded before the robot sent SYNC 3 have none; the script says so), and the sidecars'
          rewards (SYNC 4, the reward pulse; it goes to the reward box, not the NSP).

All times are shown in this computer's time zone. The NSP and the robot are separate computers' clocks,
so their times differ by however far those clocks are apart; the pulses themselves line them up.

Prints the lists and plots them, one panel per clock. Needs neo: pip install --user neo "numpy<2"
"""
import datetime as dt
import glob
import json
import os
import struct
import sys

import matplotlib.dates as mdates
import matplotlib.pyplot as plt
import numpy as np
from neo.rawio import BlackrockRawIO

NSP_CHANNELS = {"ainp2": "trial start", "ainp3": "trial end", "ainp4": "experiment start"}
ROBOT_EVENTS = {"trial_start": "SYNC 1 (trial start)", "trial_end": "SYNC 2 (trial end)",
                "experiment_start": "SYNC 3 (experiment start)", "reward": "SYNC 4 (reward)"}
THRESHOLD_V = 2.5
FIRST_ANALOG_ID = 257          # Blackrock electrode id of ainp1


def local(t_unix):
    """Unix seconds -> local datetime."""
    return dt.datetime.fromtimestamp(t_unix).astimezone()


def stamp(d):
    return d.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def nsp_start(path):
    """When the recording started: the NSx header's TimeOrigin (UTC, Central computer's clock)."""
    with open(path, "rb") as f:
        head = f.read(310)
    y, mo, _, d, h, mi, s, ms = struct.unpack("<8H", head[294:310])
    return dt.datetime(y, mo, d, h, mi, s, ms * 1000, tzinfo=dt.timezone.utc).astimezone()


def nsp_pulses(path):
    """{'ainp2': [t, ...], ...}: rising edge times [s from recording start]."""
    raw = BlackrockRawIO(filename=os.path.splitext(path)[0], nsx_to_load=int(path[-1]), load_nev=False)
    raw.parse_header()
    chans = raw.header["signal_channels"]
    rate = float(chans["sampling_rate"][0])
    t0 = raw.get_signal_t_start(0, 0, 0)
    out = {}
    for name in NSP_CHANNELS:
        # find the input by its number, so it is found even if renamed in Central
        k = [i for i, c in enumerate(chans) if int(c["id"]) == FIRST_ANALOG_ID + int(name[4:]) - 1][0]
        x = raw.get_analogsignal_chunk(0, 0, None, None, 0, channel_indexes=[k])
        volts = raw.rescale_signal_raw_to_float(x, channel_indexes=[k])[:, 0] / 1000.0   # mV -> V
        high = volts > THRESHOLD_V
        edges = np.flatnonzero(high[1:] & ~high[:-1]) + 1
        out[name] = t0 + edges / rate
    return out


def robot_pulses(folder):
    """{'trial_start': [t_wall, ...], 'trial_end': [...], 'experiment_start': [...]}: when the robot
    sent each pulse [Unix s]."""
    out = {e: [] for e in ROBOT_EVENTS}
    for meta in sorted(glob.glob(os.path.join(folder, "robot_trajectory_*.meta.json"))):
        side = json.load(open(meta))
        for ev in side.get("events", []):
            if ev["event"] in out:
                out[ev["event"]].append(ev["t_wall"])
        out["reward"] += [r["t_wall"] for r in side.get("rewards") or []]
    start = os.path.join(folder, "experiment_start.json")
    if os.path.isfile(start):
        rec = json.load(open(start))
        if rec.get("pulse_sent", True):
            out["experiment_start"].append(rec["t_wall"])
    else:
        print("note: no experiment_start.json in the robot folder (recorded before the robot sent SYNC 3)")
    return {e: np.array(sorted(t)) for e, t in out.items()}


def panel(ax, rows, title, xlabel):
    """rows: [(label, [datetime, ...])], one row of ticks each."""
    for row, (label, times) in enumerate(rows):
        ax.plot(times, np.full(len(times), row), "|", ms=20, mew=2, label=f"{label}: {len(times)}")
    ax.set_yticks([])
    ax.set_ylim(-0.7, len(rows) - 0.3)
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0))
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S", tz=dt.datetime.now().astimezone().tzinfo))


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    robot_folder, nsp_file = sys.argv[1], sys.argv[2]

    start = nsp_start(nsp_file)
    nsp = {name: [start + dt.timedelta(seconds=float(t)) for t in ts] for name, ts in nsp_pulses(nsp_file).items()}
    robot = {ev: [local(t) for t in ts] for ev, ts in robot_pulses(robot_folder).items()}

    print(f"NSP (Central computer's clock; recording started {stamp(start)})")
    for name, times in nsp.items():
        print(f"  {name} {NSP_CHANNELS[name]}: {len(times)} pulses")
        for d in times:
            print(f"    {stamp(d)}   ({(d - start).total_seconds():9.4f} s into the recording)")
    print("Robot (robot computer's clock)")
    for ev, times in robot.items():
        print(f"  {ROBOT_EVENTS[ev]}: {len(times)} pulses")
        for d in times:
            print(f"    {stamp(d)}")

    tz = dt.datetime.now().astimezone().tzname()
    fig, axes = plt.subplots(2, 1, figsize=(16, 6))
    panel(axes[0], [(f"{n} ({NSP_CHANNELS[n]})", t) for n, t in nsp.items()],
          f"Pulses recorded on the NSP (recording started {stamp(start)})", f"NSP clock [{tz}]")
    panel(axes[1], [(ROBOT_EVENTS[e], t) for e, t in robot.items()],
          "Pulses sent by the robot", f"robot clock [{tz}]")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
