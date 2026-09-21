#!/usr/bin/env python3
"""Change how long an existing GIF takes to play, without re-rendering it.

plot_trajectory.py decides playback length at render time (frames / fps), and a 1000-frame
render costs several minutes. When the result turns out to be the wrong speed -- usually
because it has to line up with camera footage -- re-rendering to fix it is wasteful. This
retimes the file you already have, in about a second.

    python3 retime_gif.py maze.gif 57           # make the whole GIF play in 57.0 s
    python3 retime_gif.py maze.gif 57 -o out.gif   # keep the original
    python3 retime_gif.py maze.gif --show        # just report the current length

HOW: a GIF stores a delay per frame, in hundredths of a second, inside each frame's Graphic
Control Extension. Only those delay fields are rewritten; the frames themselves are untouched,
so nothing is re-compressed and no quality is lost. Delays are apportioned so the TOTAL is
exactly the requested duration rather than n * round(per-frame), which drifts on long files.

CAVEAT: a delay is a request, not a guarantee. Some viewers ignore it, and most clamp delays
below ~2 cs (50+ fps) up to 10 cs. Slowing a GIF down stays well clear of that clamp.
"""
from __future__ import annotations

import argparse
import os
import sys

GCE = b"\x21\xf9\x04"       # extension introducer, graphic control label, block size 4


def find_delay_offsets(buf: bytes) -> list[int]:
    """Byte offsets of the 2-byte delay field of every Graphic Control Extension."""
    out, i = [], 0
    while True:
        i = buf.find(GCE, i)
        if i < 0:
            return out
        # A real GCE is exactly 8 bytes: 21 F9 04 <packed> <delay lo> <delay hi> <transp> 00,
        # and is followed by an image descriptor (2C) or another extension (21). Checking both
        # the terminator and the next byte keeps us from patching identical-looking pixel data.
        if i + 8 < len(buf) and buf[i + 7] == 0x00 and buf[i + 8] in (0x2C, 0x21):
            out.append(i + 4)
        i += 3


def read_delays(buf: bytes, offs: list[int]) -> list[int]:
    return [buf[o] | (buf[o + 1] << 8) for o in offs]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("gif")
    ap.add_argument("seconds", nargs="?", type=float,
                    help="target total playback time; omit to just report the current one")
    ap.add_argument("-o", "--out", help="write here instead of editing in place")
    ap.add_argument("--show", action="store_true", help="report only, change nothing")
    a = ap.parse_args()

    buf = bytearray(open(a.gif, "rb").read())
    offs = find_delay_offsets(bytes(buf))
    if not offs:
        print(f"{a.gif}: no GIF frame-delay fields found -- is it a GIF?")
        return 1

    cur = sum(read_delays(bytes(buf), offs)) / 100.0
    print(f"{os.path.basename(a.gif)}: {len(offs)} frames, plays in {cur:.1f} s")
    if a.show or a.seconds is None:
        return 0
    if a.seconds <= 0:
        print("target duration must be positive")
        return 1

    n, total_cs = len(offs), int(round(a.seconds * 100))
    if total_cs // n < 2:
        print(f"  warning: {total_cs / n:.1f} cs per frame -- most viewers clamp anything "
              f"under 2 cs up to 10 cs, so this will play slower than asked.")
    prev = 0
    for k, o in enumerate(offs):
        nxt = round(total_cs * (k + 1) / n)     # apportion so the TOTAL lands exactly
        d = min(max(nxt - prev, 1), 0xFFFF)     # 0 means "as fast as possible" in some viewers
        buf[o], buf[o + 1] = d & 0xFF, d >> 8
        prev = nxt

    dst = a.out or a.gif
    open(dst, "wb").write(bytes(buf))
    got = sum(read_delays(bytes(buf), offs)) / 100.0
    print(f"  -> {dst}: now plays in {got:.1f} s ({got / n * 1000:.0f} ms per frame)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
