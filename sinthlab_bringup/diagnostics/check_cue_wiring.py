#!/usr/bin/env python3
"""Commission the cue trigger wiring — watch the board's D2 line and report every edge.

Run this while you make and break the connection at X76 on the robot base. It polls the board's
/status over its own Wi-Fi access point and tells you what the firmware actually saw, instead of
leaving you to read raw numbers out of `watch | grep`.

    python3 check_cue_wiring.py                 # 192.168.4.1, watch until Ctrl-C
    python3 check_cue_wiring.py --preflight     # just check the board is ready, then exit
    python3 check_cue_wiring.py --board 10.0.0.7

WHAT IT CATCHES

    * settings that would make the wire look dead when the wiring is fine
      (enabled=0, or a cue so short you miss it)
    * INVERTED POLARITY -- the single failure this test exists to find. With nothing connected,
      an active-low input must read idle. If it reads asserted, something in the trigger path
      inverts and you want /config?active_low=0, not a rewiring.
    * a line that asserts but produces no cue (enabled, retrigger, or a stuck deadline)
    * contact bounce, by reporting edges faster than the configured debounce

WIRING UNDER TEST (see end_effector_design/README.md)

    X76 contact 1 ── CTR1_1 ──► tool connector pin 9  (BK) ──► Metro D2
    X76 contact 2 ── CTR1_2 ──► tool connector pin 10 (BU) ──► Metro GND

    Shorting X76 1 to 2 at the base pulls D2 down through the whole flange harness. With the
    firmware's active_low default and D2's internal pull-up that needs no power and no
    optocoupler, so a failure here is the wiring and nothing else.
"""
from __future__ import annotations

import argparse
import sys
import time
import urllib.error
import urllib.request

DEFAULT_BOARD = "192.168.4.1"
_TRUE = ("true", "1", "yes", "on")


def fetch_status(board: str, timeout: float = 2.0) -> dict:
    """Read /status into a dict. Raises OSError if the board is unreachable."""
    with urllib.request.urlopen(f"http://{board}/status", timeout=timeout) as r:
        body = r.read().decode("utf-8", "replace")
    out = {}
    for line in body.splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, _, v = line.partition("=")
            out[k.strip()] = v.strip()
    return out


def as_bool(st: dict, key: str) -> bool:
    return str(st.get(key, "")).lower() in _TRUE


def preflight(st: dict) -> int:
    """Report the board's settings and flag anything that would spoil the test."""
    print(f"  board      {st.get('ip', '?')}   up {float(st.get('uptime_s', 0)):.0f}s")
    print(f"  cue        rgbw=({st.get('r')},{st.get('g')},{st.get('b')},{st.get('w')}) "
          f"{st.get('pattern')} {st.get('duration')}s @ brightness {st.get('brightness')}")
    print(f"  trigger    mode={st.get('mode')} active_low={st.get('active_low')} "
          f"debounce={st.get('debounce_ms')}ms retrigger={st.get('retrigger')} "
          f"enabled={st.get('enabled')}")
    print(f"  state      pin_raw={st.get('trigger_pin_raw')} "
          f"asserted={st.get('trigger_asserted')} debounced={st.get('trigger_debounced')} "
          f"wire_fired={st.get('wire_fired')}")

    problems = []
    if not as_bool(st, "enabled"):
        problems.append("enabled=0 -- the wire is disarmed and will never fire a cue.\n"
                        "      fix: curl \"http://<board>/config?enabled=1\"")
    try:
        if float(st.get("duration", 0)) < 0.3:
            problems.append(f"duration={st.get('duration')}s is very short -- you may not see it.\n"
                            "      fix: curl \"http://<board>/config?duration=2\"")
    except ValueError:
        pass
    try:
        if float(st.get("brightness", 0)) < 0.05:
            problems.append(f"brightness={st.get('brightness')} is near zero -- the cue will be "
                            "invisible even when it fires.")
    except ValueError:
        pass

    # The point of the whole exercise. Nothing is connected yet, so an idle line must read idle.
    if as_bool(st, "trigger_asserted"):
        problems.append(
            "trigger_asserted=True with the line (presumably) OPEN -- polarity is inverted.\n"
            "      If nothing is connected to X76 yet, this is a firmware setting, not a wiring\n"
            "      fault. Flip it and re-run:\n"
            f"      curl \"http://<board>/config?active_low={0 if as_bool(st,'active_low') else 1}&save=1\"")

    if problems:
        print("\n  PROBLEMS")
        for p in problems:
            print(f"    ! {p}")
        return 1
    print("\n  ready -- line reads idle, cue is armed and visible.")
    return 0


def watch(board: str, interval: float) -> int:
    """Poll until Ctrl-C, announcing every transition."""
    print(f"\nWatching {board}. Short X76 contacts 1 and 2 at the robot base.  Ctrl-C to stop.\n")
    prev = None
    prev_fired = None       # wire_fired from the PREVIOUS poll -- see below
    edges = 0
    last_edge_ns = None
    fired_before_assert = None
    misses = 0
    unreachable = 0
    try:
        while True:
            try:
                st = fetch_status(board)
                unreachable = 0
            except OSError as e:
                unreachable += 1
                if unreachable == 1:
                    print(f"  [!] board unreachable: {e}")
                time.sleep(interval)
                continue

            asserted = as_bool(st, "trigger_asserted")
            raw = as_bool(st, "trigger_pin_raw")
            fired = int(st.get("wire_fired", 0))
            stamp = time.strftime("%H:%M:%S")

            if prev is None:
                print(f"  {stamp}  baseline: {'ASSERTED' if asserted else 'idle'} "
                      f"(pin_raw={raw})  wire_fired={fired}")
            elif asserted != prev:
                edges += 1
                now = time.monotonic_ns()
                gap = "" if last_edge_ns is None else f"  (+{(now-last_edge_ns)/1e6:.0f} ms)"
                last_edge_ns = now
                if asserted:
                    # Baseline from the LAST IDLE POLL, not from this one. The firmware bumps
                    # wire_fired inside poll_trigger() at the debounced edge, so by the time any
                    # /status read shows asserted=True the counter has already moved. Comparing
                    # against this poll would call every successful cue a miss.
                    fired_before_assert = prev_fired
                    print(f"  {stamp}  ── ASSERTED ──  pin_raw={raw}  wire_fired={fired}{gap}")
                else:
                    tag = ""
                    if fired_before_assert is not None and fired <= fired_before_assert:
                        misses += 1
                        tag = "   <-- no cue fired! check enabled / retrigger"
                    print(f"  {stamp}  ── released ──  pin_raw={raw}  wire_fired={fired}{gap}{tag}")
            prev = asserted
            prev_fired = fired
            time.sleep(interval)
    except KeyboardInterrupt:
        print(f"\n  {edges} edge(s) seen, {misses} assertion(s) produced no cue.")
        if edges == 0:
            print("""
  NOTHING CHANGED. In order of likelihood:
    1. X76 contact 1 or 2 is not landing on tool pin 9 / 10 -- ring them out. Remember the
       short-1-to-2 test cannot catch a swap; short contact 1 to contact 3 instead and confirm
       continuity between tool pin 9 and pin 11.
    2. The tool connector is not flush with the flange face -- contact is not guaranteed.
       4x M2x16 at 0.35 N.m.
    3. D2 is not actually the pin the board is watching -- check CUE_PIN in code.py.
    4. Your jumper is not making contact at the X76 end.""")
            return 1
        if misses:
            print("""
  EDGES REACHED THE BOARD BUT NO CUE RAN. The wiring is fine; the firmware chose not to fire:
    * enabled=0 -- the wire is disarmed.       curl "http://<board>/config?enabled=1"
    * retrigger=0 (the default) and you re-asserted while a cue was still running, which is
      correct behaviour, not a fault. Wait for `duration` to elapse between shorts.""")
            return 1
        print("  Wiring good: every assertion produced a cue.")
        return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--board", default=DEFAULT_BOARD, help=f"board address (default {DEFAULT_BOARD})")
    ap.add_argument("--interval", type=float, default=0.2, help="poll period in seconds")
    ap.add_argument("--preflight", action="store_true", help="check readiness and exit")
    a = ap.parse_args()

    print(f"Cue trigger wiring check — {a.board}\n")
    try:
        st = fetch_status(a.board)
    except OSError as e:
        print(f"  UNREACHABLE — {e}\n", file=sys.stderr)
        print("  * are you joined to the board's access point (KUKA_NEOPIXEL)?", file=sys.stderr)
        print("  * the board hosts it itself and is always at 192.168.4.1", file=sys.stderr)
        print("  * if the ring flashed twice at power-up the board is alive; check Wi-Fi",
              file=sys.stderr)
        return 2

    rc = preflight(st)
    if a.preflight or rc:
        return rc
    return watch(a.board, a.interval)


if __name__ == "__main__":
    sys.exit(main())
