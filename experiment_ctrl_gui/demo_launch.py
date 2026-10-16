#!/usr/bin/env python3
"""Stands in for `ros2 launch` in demo mode (server.py --demo): prints a launch-like banner, idles,
and shuts down on Ctrl-C like the real thing, so Start / Stop / Restart are exercised for real.
The trial loop itself is simulated by ros_bridge.DemoBridge."""
import signal
import sys
import time

name = sys.argv[1] if len(sys.argv) > 1 else "experiment"
print(f"[INFO] [launch]: Demo launch for '{name}' -- no robot, no ROS. (experiment_ctrl_gui --demo)")
print("[INFO] [ros2_control_node-1]: process started with pid [0]")
print("[WARN] [launch]: This is a SIMULATION. Nothing is connected to the arm.")
sys.stdout.flush()


def _stop(*_):
    print("[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)")
    time.sleep(0.5)
    print("[INFO] [ros2_control_node-1]: process has finished cleanly [pid 0]")
    sys.stdout.flush()
    sys.exit(0)


signal.signal(signal.SIGINT, _stop)
while True:
    time.sleep(1)
