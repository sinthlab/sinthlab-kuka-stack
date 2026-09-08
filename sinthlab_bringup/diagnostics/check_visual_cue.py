#!/usr/bin/env python3
"""Check the visual cue trigger path from the ROS box, and fire test cues.

The cue reaches the ring in two steps, set up in two different places:

    TRIGGER   ROS -> cabinet cue server (TCP 30300) -> 24 V flange line -> opto -> board
              This is what the experiment uses, and what this tool checks.

    APPEARANCE  set on the board itself over its own Wi-Fi access point, from a laptop or phone:
                    curl "http://192.168.4.1/config?r=0&g=255&b=0&w=0&duration=1.2&save=1"
                The ROS box is on the KUKA network and cannot reach the board, so this tool
                cannot check or change it — see end_effector_metro_code/README.md.

    python3 check_visual_cue.py config/maze_params.yaml           # check reachability
    python3 check_visual_cue.py config/maze_params.yaml --fire 3  # fire 3 test cues
    python3 check_visual_cue.py --host 172.31.1.147 --fire 1      # no config file needed
"""
from __future__ import annotations

import argparse
import socket
import statistics
import sys
import time

try:
    import yaml
except ImportError:
    yaml = None


def load_block(path: str) -> dict:
    if yaml is None:
        sys.exit("PyYAML is required to read a config:  pip install pyyaml")
    doc = yaml.safe_load(open(path))
    for node in (doc or {}).values():
        if isinstance(node, dict) and "ros__parameters" in node:
            block = node["ros__parameters"].get("visual_cue")
            if block is None:
                sys.exit(f"{path}: no visual_cue block")
            return block
    sys.exit(f"{path}: no ros__parameters block found")


class Cabinet:
    def __init__(self, host: str, port: int, timeout: float = 3.0) -> None:
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.f = self.sock.makefile("rw", encoding="ascii", newline="\n")

    def command(self, text: str):
        t0 = time.perf_counter()
        self.f.write(text + "\n")
        self.f.flush()
        return self.f.readline().strip(), (time.perf_counter() - t0) * 1000.0

    def close(self) -> None:
        try:
            self.f.close()
            self.sock.close()
        except OSError:
            pass


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("params", nargs="?", help="a sinthlab_bringup config/*.yaml")
    ap.add_argument("--host", help="cabinet IP (overrides the config)")
    ap.add_argument("--port", type=int, help="cue server port (overrides the config)")
    ap.add_argument("--fire", type=int, metavar="N", default=0,
                    help="fire N test cues, 2 s apart, and watch the ring")
    ap.add_argument("--latency", action="store_true", help="measure round-trip latency")
    a = ap.parse_args()

    block = load_block(a.params) if a.params else {}
    host = a.host or block.get("host")
    port = a.port or int(block.get("port", 30300))
    if not host:
        sys.exit("no cabinet host: pass --host or a config file")

    if a.params:
        print(f"source     {a.params}")
        enabled = block.get("enabled")
        print(f"enabled    {enabled}"
              + ("   <-- cues are OFF in this config; this only checks reachability"
                 if not enabled else ""))
        print(f"pulse_ms   {block.get('pulse_ms', 0)}"
              + ("   (bare trigger; the board's own duration owns the length)"
                 if not block.get("pulse_ms") else "   (the board must be in 'follow' mode)"))
    print(f"cabinet    {host}:{port}\n")

    try:
        c = Cabinet(host, port)
    except OSError as e:
        print(f"UNREACHABLE — {e}", file=sys.stderr)
        print("  * is the Sunrise application running?", file=sys.stderr)
        print("  * its log shows 'Cue server listening on TCP 30300' at startup", file=sys.stderr)
        return 1

    try:
        reply, rtt = c.command("STATUS")
        print(f"  STATUS  {reply}   ({rtt:.2f} ms)")

        if a.latency:
            rtts = []
            for i in range(50):
                _, r = c.command("PING")
                if i:
                    rtts.append(r)
            rtts.sort()
            print(f"\n  PING x49   min {rtts[0]:.3f}  median {statistics.median(rtts):.3f}  "
                  f"p95 {rtts[int(0.95*len(rtts))]:.3f}  max {rtts[-1]:.3f} ms")
            print("  This is the ROS->cabinet leg only. The rest of the chain (cabinet I/O")
            print("  cycle, opto, the board's debounce, the LED refresh) needs a scope.")

        pulse = int(block.get("pulse_ms", 0) or 0)
        for i in range(a.fire):
            reply, rtt = c.command("CUE" if pulse <= 0 else f"CUE {pulse}")
            print(f"  CUE {i+1}/{a.fire}   {reply}   ({rtt:.2f} ms)")
            if i + 1 < a.fire:
                time.sleep(2.0)
        if a.fire:
            print("\n  Did the ring light each time? If not, in order:")
            print("    1. meter on the flange pin — is the cabinet driving it?")
            print("    2. curl http://192.168.4.1/status from a laptop on KUKA_NEOPIXEL —")
            print("       check trigger_asserted follows the cabinet, and wire_fired increments")
            print("    3. curl http://192.168.4.1/cue — does the ring work at all?")
    finally:
        c.close()

    print("\nOK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
