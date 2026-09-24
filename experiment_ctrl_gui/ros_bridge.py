"""The dashboard's ROS side: listens to the orchestrator and robot, sets live parameters, pauses.

    <ns>/experiment_status   std_msgs/String (JSON)    published by helpers/experiment_control.py
    <ns>/lbr_state           lbr_fri_idl/LBRState      FRI session state and stream rate
    <ns>/<node>/set_parameters                         live parameters (the node rejects the rest)
    <ns>/<node>/pause        std_srvs/SetBool          hold at the start between trials

It talks to whatever orchestrator is running -- one started from the dashboard, or one started in a
terminal with `ros2 launch` -- so the live controls work either way.

DemoBridge stands in for all of it with a simulated orchestrator, so the dashboard can be tried, and
developed, on a computer with no robot and no ROS.
"""
from __future__ import annotations

import json
import random
import threading
import time
from collections import deque
from typing import Callable, Dict, Optional, Tuple

import experiments as ex

SESSION_STATES = {0: "IDLE", 1: "MONITORING_WAIT", 2: "MONITORING_READY",
                  3: "COMMANDING_WAIT", 4: "COMMANDING_ACTIVE"}
CALL_TIMEOUT_SEC = 3.0


class RosBridge:
    def __init__(self, robot_name: str, on_status: Callable[[dict], None]) -> None:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from std_msgs.msg import String

        self.available, self.reason = True, ""
        self._ns = f"/{robot_name}"
        self._on_status = on_status
        self._status: Optional[dict] = None
        self._state_times: deque = deque(maxlen=200)
        self._session_state: Optional[int] = None

        rclpy.init()
        self._rclpy = rclpy
        self._node = rclpy.create_node("experiment_ctrl_gui")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._node.create_subscription(String, f"{self._ns}/experiment_status", self._on_msg, latched)
        try:
            from lbr_fri_idl.msg import LBRState
            self._node.create_subscription(LBRState, f"{self._ns}/lbr_state", self._on_state,
                                           qos_profile_sensor_data)
        except ImportError:
            pass     # robot panel shows "unknown"; everything else works
        self._clients: Dict[Tuple[str, str], object] = {}
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        threading.Thread(target=self._executor.spin, name="ros_bridge", daemon=True).start()

    # ------------------------------------------------------------------ inputs

    def _on_msg(self, msg) -> None:
        try:
            self._status = json.loads(msg.data)
        except ValueError:
            return
        self._status["_rx"] = time.time()
        self._on_status(self._status)

    def _on_state(self, msg) -> None:
        self._state_times.append(time.monotonic())
        self._session_state = int(msg.session_state)

    def status(self) -> Optional[dict]:
        return self._status

    def robot(self) -> dict:
        now = time.monotonic()
        recent = [t for t in self._state_times if now - t < 1.0]
        return {
            "lbr_state_hz": len(recent),
            "session_state": SESSION_STATES.get(self._session_state, None)
            if recent else None,
        }

    # ------------------------------------------------------------------ calls

    def _call(self, srv_type, name: str, request) -> Tuple[bool, object]:
        key = (srv_type.__name__, name)
        client = self._clients.get(key)
        if client is None:
            client = self._clients[key] = self._node.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=CALL_TIMEOUT_SEC):
            return False, f"{name} is not available -- is the experiment running?"
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(CALL_TIMEOUT_SEC):
            return False, f"{name} did not answer within {CALL_TIMEOUT_SEC:.0f} s"
        return True, future.result()

    def set_param(self, exp: ex.Experiment, name: str, value) -> Tuple[bool, str]:
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter
        req = SetParameters.Request()
        req.parameters = [Parameter(name, value=value).to_parameter_msg()]
        ok, res = self._call(SetParameters, f"{self._ns}/{exp.node}/set_parameters", req)
        if not ok:
            return False, res
        r = res.results[0]
        return bool(r.successful), r.reason or "accepted; applies from the next trial"

    def pause(self, exp: ex.Experiment, paused: bool) -> Tuple[bool, str]:
        from std_srvs.srv import SetBool
        ok, res = self._call(SetBool, f"{self._ns}/{exp.node}/pause", SetBool.Request(data=paused))
        if not ok:
            return False, res
        return bool(res.success), res.message

    def shutdown(self) -> None:
        try:
            self._executor.shutdown()
            self._node.destroy_node()
            self._rclpy.shutdown()
        except Exception:
            pass


class UnavailableBridge:
    """ROS is not sourced: the dashboard still starts and stops launches and shows their logs."""

    def __init__(self, reason: str) -> None:
        self.available, self.reason = False, reason

    def status(self):
        return None

    def robot(self) -> dict:
        return {"lbr_state_hz": None, "session_state": None}

    def set_param(self, exp, name, value):
        return False, f"ROS is not available here: {self.reason}"

    def pause(self, exp, paused):
        return False, f"ROS is not available here: {self.reason}"

    def shutdown(self) -> None:
        pass


class DemoBridge:
    """A simulated orchestrator: the same phases, pause and live-parameter rules as the real one,
    on a fast clock. Logs go to the runner's log stream as if the launch had printed them."""

    def __init__(self, on_status: Callable[[dict], None]) -> None:
        self.available, self.reason = True, "demo mode (simulated robot)"
        self._on_status = on_status
        self._emit: Optional[Callable[[str, Optional[str]], None]] = None
        self._lock = threading.Lock()
        self._status: Optional[dict] = None
        self._run_id = 0

    def attach_log(self, emit: Callable[[str, Optional[str]], None]) -> None:
        self._emit = emit

    # -- the runner calls these when the (fake) launch starts and ends
    def launch_started(self, exp: ex.Experiment, params: Dict[str, object]) -> None:
        with self._lock:
            self._run_id += 1
            self._exp = exp
            self._params = dict(params)
            self._effective = {k: v for k, v in params.items() if ex.live_params.is_live(exp.key, k)}
            self._pending: Dict[str, object] = {}
            self._paused = False
            self._held = False
            self._trial = 0
            self._phase, self._phase_arg, self._phase_t = "starting", None, time.time()
        threading.Thread(target=self._loop, args=(self._run_id,), daemon=True).start()

    def launch_ended(self) -> None:
        with self._lock:
            self._run_id += 1
            self._status = None

    def _log(self, text: str, level: str = "INFO") -> None:
        if self._emit:
            self._emit(f"[{self._exp.node}.py-5] [{level}] [{time.time():.3f}] "
                       f"[lbr.{self._exp.node}]: {text}", None)

    def _publish(self) -> None:
        self._status = {
            "experiment": self._exp.key, "node": f"/lbr/{self._exp.node}", "trial": self._trial,
            "phase": self._phase, "phase_arg": self._phase_arg, "phase_t": self._phase_t,
            "t": time.time(), "paused": self._paused, "held": self._held,
            "pending": dict(self._pending), "live": dict(self._effective),
            "nsp_sync": bool(self._effective.get("nsp_sync.enabled", False)), "_rx": time.time(),
        }
        self._on_status(self._status)

    def _phase_to(self, phase: str, arg=None, log: Optional[str] = None) -> None:
        self._phase, self._phase_arg, self._phase_t = phase, arg, time.time()
        if log:
            self._log(log)
        self._publish()

    def _sleep(self, run_id: int, sec: float) -> bool:
        """Sleep, heartbeating once a second like the real node; False if the run was stopped."""
        end = time.time() + sec
        while time.time() < end:
            if run_id != self._run_id:
                return False
            time.sleep(0.05)
            if self._status and time.time() - self._status["t"] >= 1.0:
                self._publish()
        return run_id == self._run_id

    def _loop(self, run_id: int) -> None:
        self._log("=== AUTOMATED MULTI-TRIAL EXPERIMENT INITIALIZED (demo) ===")
        cue = lambda: self._effective.get("audio_cue.enabled", True)
        while run_id == self._run_id:
            with self._lock:
                if self._pending:
                    self._effective.update(self._pending)
                    self._log("Live parameters applied from trial "
                              f"{self._trial + 1}: " + ", ".join(f"{k}={v}" for k, v in self._pending.items()))
                    self._pending = {}
                self._trial += 1
            self._phase_to("trial_start", self._trial, f"--- STARTING TRIAL {self._trial} ---")
            if not self._sleep(run_id, 1.5):
                return
            self._phase_to("at_start", None, "Arm returned to start. Waiting for a quiet window...")
            if not self._sleep(run_id, float(self._effective.get("quiet_window_sec", 2.0))):
                return
            self._phase_to("cue_go", None, "Quiet window complete. Sounding audio cue."
                           if cue() else "Quiet window complete. (audio cues off)")
            if self._effective.get("visual_cue.enabled") and not self._effective.get("visual_cue.remote_test_trigger"):
                self._log("visual_cue.enabled is true, but no trigger switch is wired to this computer yet", "WARN")
            if not self._sleep(run_id, 0.5):
                return
            self._phase_to("armed", None, "Monitor armed (baseline locked). Watching for the pull.")
            if not self._sleep(run_id, random.uniform(2.0, 4.5)):
                return
            if self._exp.key == "maze":
                self._phase_to("checkpoint", 0, "Reward at checkpoint 0.")
                if not self._sleep(run_id, 1.0):
                    return
                self._phase_to("goal", None, "Maze solved! Playing goal cue; waiting for release before reset.")
            else:
                thr = self._effective.get(
                    "apple_pluck_impedance_control_displacement.cartesian_displacement_threshold_m", 0.1)
                self._phase_to("snap", thr, f"Threshold reached ({thr} m) — freezing the arm at its current pose.")
            if not self._sleep(run_id, 1.0):
                return
            self._phase_to("recover_start", None, "Returning to start.")
            if not self._sleep(run_id, 1.5):
                return
            self._phase_to("trial_end", self._trial, f"--- TRIAL {self._trial} COMPLETE ---")
            with self._lock:
                paused = self._paused
                if paused:
                    self._held = True
            if paused:
                self._phase_to("paused", None, "Paused: holding at the start pose. Resume to run the next trial.")
                while self._held:
                    if not self._sleep(run_id, 0.2):
                        return

    def status(self):
        return self._status

    def robot(self) -> dict:
        running = self._status is not None
        return {"lbr_state_hz": 100 if running else 0,
                "session_state": "COMMANDING_ACTIVE" if running else None}

    def set_param(self, exp: ex.Experiment, name: str, value) -> Tuple[bool, str]:
        if self._status is None:
            return False, "no experiment is running"
        if not ex.live_params.is_live(exp.key, name):
            return False, f"'{name}' is a per-run parameter: it was read once at start-up."
        problem = ex.live_params.check_value(name, value)
        if problem:
            return False, problem
        with self._lock:
            self._pending[name] = value
        self._log(f"Live change accepted: {name} = {value} (applies from trial {self._trial + 1})")
        self._publish()
        return True, "accepted; applies from the next trial"

    def pause(self, exp: ex.Experiment, paused: bool) -> Tuple[bool, str]:
        if self._status is None:
            return False, "no experiment is running"
        with self._lock:
            self._paused = paused
            if not paused and self._held:
                self._held = False
                self._log("Resumed.")
        self._publish()
        return True, ("pause requested: holds at the start once this trial ends" if paused else "running")

    def shutdown(self) -> None:
        self.launch_ended()


def make_bridge(demo: bool, robot_name: str, on_status: Callable[[dict], None]):
    if demo:
        return DemoBridge(on_status)
    try:
        return RosBridge(robot_name, on_status)
    except Exception as exc:     # rclpy missing or ROS not sourced
        return UnavailableBridge(f"{type(exc).__name__}: {exc}. Source ROS and the workspace "
                                 "(run_gui.sh does), then restart the dashboard.")
