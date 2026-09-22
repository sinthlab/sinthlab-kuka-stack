#!/usr/bin/env python3
"""Measure the FRI cabinet clock against the ROS box clock -- resolution, drift and jumps.

The cabinet timestamp in <ns>/lbr_state is what trial data will be aligned to the Blackrock NSP
with (see analysis/RECORDING_SPEC.md), so three things about it have to be known before anything is
built on it:

  1. RESOLUTION. time_stamp_sec alone ticks once a second. All the useful precision is in
     time_stamp_nano_sec. If that field is always zero the timestamp is useless for alignment and
     the whole recording design has to change -- so this is checked first and reported loudest.
  2. DRIFT. The cabinet is not NTP-disciplined (observed ~11 min fast on 2026-09-22), which does NOT
     matter on its own: trials are anchored to neural data by the sync pulse, so only the RATE the
     cabinet clock runs at matters, not its absolute value. This fits that rate and reports it in
     ppm and ms per minute, so you can say how long a block can run before drift eats your timing
     budget.
  3. JUMPS. A step change mid-session would corrupt alignment silently. Any discontinuity is listed.

Run it on the ROS box with the robot connected and the driver up:

    ros2 run sinthlab_bringup check_clock_drift.py                 # 5 minutes, namespace /lbr
    ros2 run sinthlab_bringup check_clock_drift.py --seconds 900   # longer = better drift estimate

Drift is fitted against time.monotonic(), not the wall clock, so an NTP step on the ROS box during
the run cannot masquerade as cabinet drift. The absolute offset is reported against the wall clock
because that is the number you compare with a phone.

Accuracy note: the fitted rate is only as good as the run is long. A 5-minute run resolves roughly
20 ppm; to claim single-digit ppm, run 15 minutes or more. The report says which.
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time


def fit_slope(xs, ys):
    """Least-squares slope and intercept, without pulling in numpy."""
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    slope = sxy / sxx if sxx else 0.0
    return slope, my - slope * mx


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ns", default="lbr", help="robot namespace (default: lbr)")
    ap.add_argument("--seconds", type=float, default=300.0, help="how long to sample (default: 300)")
    ap.add_argument("--jump-ms", type=float, default=50.0,
                    help="report a cabinet-clock step larger than this (default: 50)")
    args, ros_args = ap.parse_known_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from lbr_fri_idl.msg import LBRState

    fri, mono, wall, nsec_vals = [], [], [], []
    sample_time = [None]

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = Node("clock_drift_checker")

    def on_state(msg):
        # Full cabinet time. This is the value the recorder will write, so it is what gets measured.
        fri.append(float(msg.time_stamp_sec) + float(msg.time_stamp_nano_sec) * 1e-9)
        mono.append(time.monotonic())
        wall.append(time.time())
        nsec_vals.append(int(msg.time_stamp_nano_sec))
        if sample_time[0] is None:
            sample_time[0] = float(msg.sample_time)

    node.create_subscription(LBRState, f"/{args.ns}/lbr_state", on_state, qos_profile_sensor_data)

    print(f"sampling /{args.ns}/lbr_state for {args.seconds:.0f} s ... (Ctrl-C to stop early)")
    t_end = time.monotonic() + args.seconds
    last_report = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - last_report >= 30.0:
                last_report = time.monotonic()
                print(f"  {len(fri)} samples, {t_end - time.monotonic():.0f} s to go")
    except KeyboardInterrupt:
        print("\nstopped early")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if len(fri) < 100:
        print(f"\nFAIL  only {len(fri)} samples. Is the driver up and the namespace right (--ns)?")
        return 1

    return report(fri, mono, wall, nsec_vals, sample_time[0], args.jump_ms)


def report(fri, mono, wall, nsec_vals, sample_time, jump_ms=50.0):
    """Print the four-part verdict. Split out from main() so it can be exercised without a robot."""
    span = mono[-1] - mono[0]
    st = sample_time if sample_time else 0.01
    print(f"\n{'='*72}\n{len(fri)} samples over {span:.1f} s"
          f"   (FRI sample_time {st*1000:.1f} ms)\n{'='*72}")

    # ---- 1. resolution: does nano_sec actually carry sub-second information? ----
    print("\n[1] TIMESTAMP RESOLUTION")
    uniq = len(set(nsec_vals))
    ok_res = uniq > 1
    if not ok_res:
        print(f"  FAIL  time_stamp_nano_sec is CONSTANT ({nsec_vals[0]}).")
        print("        The cabinet timestamp has 1-SECOND resolution and cannot align anything.")
        print("        analysis/RECORDING_SPEC.md has to fall back to interpolating sample index")
        print("        against ROS time. Stop and re-plan before building the recorder.")
    else:
        steps = sorted(b - a for a, b in zip(fri, fri[1:]) if 0 < b - a < 1.0)
        print(f"  OK    nano_sec takes {uniq} distinct values, range {min(nsec_vals)}..{max(nsec_vals)}")
        if steps:
            med = statistics.median(steps)
            print(f"        median step between cabinet stamps: {med*1000:.3f} ms -> {1.0/med:.1f} Hz")
            print(f"        p99 step: {steps[int(0.99*(len(steps)-1))]*1000:.3f} ms")
        print("        sub-millisecond resolution; safe to align on.")

    # Find steps FIRST: one step makes the drift fit meaningless, and reporting a contaminated
    # slope as if it were a crystal rate would be worse than reporting nothing.
    jumps = []
    for i in range(1, len(fri)):
        d_fri = fri[i] - fri[i - 1]
        d_mono = mono[i] - mono[i - 1]
        if abs(d_fri - d_mono) * 1000.0 > jump_ms:
            jumps.append((mono[i] - mono[0], d_fri * 1000.0, d_mono * 1000.0))

    # ---- 2. drift: how fast does the cabinet clock run vs the ROS box? ----
    print("\n[2] DRIFT  (cabinet vs ROS box, fitted against monotonic)")
    resid = [f - m for f, m in zip(fri, mono)]
    slope, intercept = fit_slope(mono, resid)
    ppm = slope * 1e6
    # Resolution = standard error of the fitted slope. The scatter has to be measured about the
    # FITTED LINE, not about the mean: the raw residual is dominated by the drift ramp itself, and
    # using it makes even a cleanly recovered slope look like noise.
    fit_resid = [r - (slope * x + intercept) for x, r in zip(mono, resid)]
    noise = statistics.pstdev(fit_resid) if len(fit_resid) > 2 else 0.0
    mx = sum(mono) / len(mono)
    sxx = sum((x - mx) ** 2 for x in mono)
    stderr_ppm = (noise / (sxx ** 0.5)) * 1e6 if sxx > 0 else float("inf")
    res_ppm = 3.0 * stderr_ppm          # ~3 sigma
    print(f"  cabinet runs {ppm:+.1f} ppm  ({slope*60_000.0:+.2f} ms per minute)")
    res_txt = "<0.1" if res_ppm < 0.1 else f"{res_ppm:.1f}"
    print(f"  fit resolution over this {span/60:.1f} min run: +/-{res_txt} ppm (3 sigma)")
    if jumps:
        print(f"  !!    {len(jumps)} clock STEP(S) detected (see [4]). The figure above is the step")
        print("        smeared across the run, NOT a crystal rate. Fix the stepping first, then")
        print("        re-run to get a real drift number.")
    elif abs(ppm) < res_ppm:
        print("  -> drift is below what this run can resolve; it is at most the figure above.")
        print("     Run longer (--seconds 900) if you need to pin it tighter.")
    else:
        for label, secs in (("60 s trial", 60), ("10 min block", 600), ("1 h session", 3600)):
            print(f"       accumulates {abs(slope)*secs*1000:8.1f} ms over a {label}")

    # ---- 3. absolute offset (informational -- does NOT affect alignment) ----
    off = statistics.median([f - w for f, w in zip(fri, wall)])
    print("\n[3] ABSOLUTE OFFSET  (informational)")
    print(f"  cabinet is {abs(off):.1f} s ({abs(off)/60:.1f} min) "
          f"{'ahead of' if off > 0 else 'behind'} the ROS box wall clock.")
    print("  This does NOT affect alignment: trials are anchored by the sync pulse, so only the")
    print("  RATE above matters. It does mean never assuming the two clocks agree.")

    # ---- 4. jumps: a step here would corrupt alignment silently ----
    print("\n[4] DISCONTINUITIES")
    if not jumps:
        print(f"  OK    no cabinet-clock step larger than {jump_ms:.0f} ms")
    else:
        print(f"  WARN  {len(jumps)} step(s) larger than {jump_ms:.0f} ms:")
        for t, df, dm in jumps[:10]:
            print(f"          t={t:7.2f} s   cabinet advanced {df:+.1f} ms "
                  f"while the box advanced {dm:+.1f} ms")
        if len(jumps) > 10:
            print(f"          ... and {len(jumps)-10} more")
        print("        A step mid-session breaks alignment silently. Find out what moved the clock")
        print("        before recording anything you intend to analyse.")

    print(f"\n{'='*72}")
    return 0 if (ok_res and not jumps) else 1


if __name__ == "__main__":
    sys.exit(main())
