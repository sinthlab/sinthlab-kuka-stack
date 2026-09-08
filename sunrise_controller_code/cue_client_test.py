#!/usr/bin/env python3
"""Bench client for the cue server in LbrImpedanceControlServer.java.

No ROS, no dependencies -- a plain socket client, so the cue path can be commissioned before the
ROS-side VisualCue action exists.

    python3 cue_client_test.py 172.31.1.147            # interactive
    python3 cue_client_test.py 172.31.1.147 --latency  # measure the ROS->cabinet leg
    python3 cue_client_test.py 172.31.1.147 --cue 2000 # one 2 s pulse ("follow" mode)

The cabinet's IP is the robot controller's address on the KUKA network -- the one the ROS box
already talks FRI to, NOT the client_names_ entries in the Java (those are the ROS box's address).
"""
from __future__ import annotations

import argparse
import socket
import statistics
import sys
import time

PORT = 30300


class CueClient:
    """Persistent connection to the cabinet's cue server.

    The connection is deliberately held open: a TCP handshake per cue would add a round trip to
    the path whose latency this whole design is trying to keep small and predictable.
    """

    def __init__(self, host: str, port: int = PORT, timeout: float = 5.0) -> None:
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)   # match the server
        self.f = self.sock.makefile("rw", encoding="ascii", newline="\n")

    def command(self, text: str) -> tuple[str, float]:
        """Send one command; return (reply, round_trip_seconds)."""
        t0 = time.perf_counter()
        self.f.write(text + "\n")
        self.f.flush()
        reply = self.f.readline().strip()
        return reply, time.perf_counter() - t0

    def close(self) -> None:
        try:
            self.command("OFF")
        except Exception:
            pass
        self.f.close()
        self.sock.close()


def measure_latency(c: CueClient, n: int = 50) -> None:
    """Characterise the ROS->cabinet leg.

    This is the ONLY leg where the socket approach differs from FRI boolean I/O. Everything
    downstream -- the cabinet I/O cycle, the optocoupler, the board's debounce, the LED refresh --
    is common to both and is what actually dominates end-to-end cue latency. To measure THAT you
    need a scope or logic analyser on the flange output; this only bounds the network leg.
    """
    print(f"PING x{n} (excluding the first, which pays for TCP warm-up)")
    rtts = []
    for i in range(n):
        reply, rtt = c.command("PING")
        if not reply.startswith("PONG"):
            print(f"  unexpected reply: {reply}")
            return
        if i:
            rtts.append(rtt * 1000.0)
    rtts.sort()
    print(f"  min    {rtts[0]:7.3f} ms")
    print(f"  median {statistics.median(rtts):7.3f} ms")
    print(f"  p95    {rtts[int(0.95 * len(rtts))]:7.3f} ms")
    print(f"  max    {rtts[-1]:7.3f} ms   <-- the jitter tail is what matters, not the median")
    print()
    print("  Halve these for the one-way ROS->cabinet estimate. The cue itself lands somewhere")
    print("  inside [send, ack], so the ack bounds it -- log both in the trial record.")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("host", help="cabinet IP on the KUKA network")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--cue", type=int, metavar="MS", help="fire one cue of MS milliseconds and exit")
    ap.add_argument("--latency", action="store_true", help="measure round-trip latency and exit")
    a = ap.parse_args()

    try:
        c = CueClient(a.host, a.port)
    except OSError as e:
        print(f"cannot reach the cue server at {a.host}:{a.port} -- {e}", file=sys.stderr)
        print("  * is the Sunrise application running?  (the server starts with it)", file=sys.stderr)
        print("  * check the cabinet log for 'Cue server listening on TCP'", file=sys.stderr)
        return 1

    print(f"connected to {a.host}:{a.port}")
    reply, _ = c.command("STATUS")
    print(f"  {reply}\n")

    try:
        if a.latency:
            measure_latency(c)
            return 0
        if a.cue is not None:
            reply, rtt = c.command(f"CUE {a.cue}")
            print(f"  {reply}   (round trip {rtt * 1000:.2f} ms)")
            return 0

        print("commands: CUE [ms] | OFF | PING | STATUS | quit")
        while True:
            try:
                line = input("cue> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return 0
            if line.lower() in ("quit", "exit", "q"):
                return 0
            if not line:
                continue
            reply, rtt = c.command(line)
            print(f"  {reply}   (round trip {rtt * 1000:.2f} ms)")
    finally:
        c.close()


if __name__ == "__main__":
    sys.exit(main())
