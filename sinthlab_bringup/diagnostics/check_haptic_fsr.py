#!/usr/bin/env python3
"""Bench check: how much does the apple's haptic motor leak into its force sensor?

    ros2 run sinthlab_bringup check_haptic_fsr.py                 # or: python3 diagnostics/check_haptic_fsr.py
    ros2 run sinthlab_bringup check_haptic_fsr.py --port /dev/ttyUSB0 --levels 0 30 60 100 --seconds 3

Talks to the end-effector board straight over the serial link (no experiment running: the port is
the experiments' too). It streams the FSR (FSR <rate>), then holds the motor at each strength in turn
(HAP <pct>, refreshed every 0.2 s so the board's 0.5 s watchdog never cuts it), and prints the force
reading at each: mean, standard deviation and peak-to-peak of the raw value (0-65535). The rows
against 0 % are the motor's leakage.

Run it twice: with nothing touching the apple, then with a fixed weight resting on it
(end_effector_design/README.md, "Keeping the motor out of the force reading").
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--rate", type=int, default=100, help="force samples per second")
    ap.add_argument("--levels", type=int, nargs="+", default=[0, 30, 60, 100], help="haptic strengths [%%]")
    ap.add_argument("--seconds", type=float, default=3.0, help="hold at each strength")
    ap.add_argument("--settle", type=float, default=0.5, help="ignore this long after each change [s]")
    args = ap.parse_args()
    try:
        import serial
    except ImportError:
        print("python3-serial is missing: sudo apt install python3-serial")
        return 2
    try:
        s = serial.Serial(args.port, 115200, timeout=0.05)
    except Exception as exc:
        print(f"Cannot open {args.port}: {exc}. Is the StarTech attached to WSL (run_gui.sh does it)?")
        return 2

    def send(cmd: str) -> None:
        s.write((cmd + "\n").encode())

    s.reset_input_buffer()
    send("PING")
    send(f"FSR {args.rate}")
    print(f"{'haptic %':>9} {'samples':>8} {'mean':>9} {'sd':>8} {'p-p':>7} {'mean - at 0 %':>14}")
    base = None
    rows = []
    try:
        for level in args.levels:
            vals, haps = [], []
            t_end = time.monotonic() + args.settle + args.seconds
            t_keep = time.monotonic() + args.settle
            next_hap = 0.0
            buf = b""
            while time.monotonic() < t_end:
                now = time.monotonic()
                if now >= next_hap:
                    send(f"HAP {level}")
                    next_hap = now + 0.2
                buf += s.read(4096)
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    p = line.decode(errors="replace").split()
                    if len(p) == 4 and p[0] == "S" and time.monotonic() >= t_keep:
                        vals.append(int(p[2]))
                        haps.append(int(p[3]))
                    elif p and p[0] == "ERR":
                        print("board:", " ".join(p))
            if not vals:
                print(f"{level:>9} no samples -- is the FSR streaming? (GET /status: fsr=up)")
                continue
            mean = statistics.fmean(vals)
            sd = statistics.pstdev(vals)
            base = mean if level == 0 else base
            rows.append(level)
            delta = "" if base is None else f"{mean - base:+.0f}"
            driven = "" if all(h == level for h in haps) else "  (some samples at another strength)"
            print(f"{level:>9} {len(vals):>8} {mean:>9.0f} {sd:>8.1f} {max(vals) - min(vals):>7} {delta:>14}{driven}")
    finally:
        send("HAP 0")
        send("FSR 0")
        s.close()
    return 0 if rows else 1


if __name__ == "__main__":
    sys.exit(main())
