"""The dashboard's ROS side: listens to the orchestrator and robot, sets live parameters, stops.

    <ns>/experiment_status   std_msgs/String (JSON)    published by helpers/experiment_control.py
    <ns>/lbr_state           lbr_fri_idl/LBRState      FRI session state and stream rate
    <ns>/<node>/set_parameters                         live parameters (the node rejects the rest)
    <ns>/<node>/stop         std_srvs/Trigger          end the trial now, park, and hold
    <ns>/maze_view/geometry  std_msgs/String (JSON)    the maze's path, checkpoints and goal (latched)
    <ns>/maze_view/state     std_msgs/String (JSON)    where the arm is in the maze (helpers/maze_view.py)

It talks to whatever orchestrator is running -- one started from the dashboard, or one started in a
terminal with `ros2 launch` -- so the live controls work either way.

DemoBridge stands in for all of it with a simulated orchestrator, so the dashboard can be tried, and
developed, on a computer with no robot and no ROS.
"""
from __future__ import annotations

import json
import math
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
    def __init__(self, robot_name: str, on_status: Callable[[dict], None],
                 on_maze: Callable[[str, dict], None]) -> None:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from std_msgs.msg import String

        self.available, self.reason = True, ""
        self._ns = f"/{robot_name}"
        self._on_status = on_status
        self._on_maze = on_maze
        self._status: Optional[dict] = None
        self._maze: Dict[str, Optional[dict]] = {"geometry": None, "state": None}
        self._state_times: deque = deque(maxlen=200)
        self._session_state: Optional[int] = None

        rclpy.init()
        self._rclpy = rclpy
        self._node = rclpy.create_node("experiment_ctrl_gui")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._node.create_subscription(String, f"{self._ns}/experiment_status", self._on_msg, latched)
        self._node.create_subscription(String, f"{self._ns}/maze_view/geometry",
                                       lambda m: self._on_maze_msg("geometry", m), latched)
        self._node.create_subscription(String, f"{self._ns}/maze_view/state",
                                       lambda m: self._on_maze_msg("state", m), 1)
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

    def _on_maze_msg(self, kind: str, msg) -> None:
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        data["_rx"] = time.time()
        self._maze[kind] = data
        self._on_maze(kind, data)

    def maze(self) -> dict:
        return dict(self._maze)

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

    def set_params(self, exp: ex.Experiment, values: Dict[str, object]) -> Tuple[bool, str]:
        """Set live parameters atomically (set_parameters_atomically: all or nothing)."""
        from rcl_interfaces.srv import SetParametersAtomically
        from rclpy.parameter import Parameter
        req = SetParametersAtomically.Request()
        req.parameters = [Parameter(n, value=v).to_parameter_msg() for n, v in values.items()]
        ok, res = self._call(SetParametersAtomically,
                             f"{self._ns}/{exp.node}/set_parameters_atomically", req)
        if not ok:
            return False, res
        r = res.result
        return bool(r.successful), r.reason or "accepted; applies from the next trial"

    def stop(self, exp: ex.Experiment) -> Tuple[bool, str]:
        """Stop: the orchestrator ends the trial at once, parks the arm, and holds."""
        from std_srvs.srv import Trigger
        ok, res = self._call(Trigger, f"{self._ns}/{exp.node}/stop", Trigger.Request())
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

    def maze(self) -> dict:
        return {"geometry": None, "state": None}

    def set_params(self, exp, values):
        return False, f"ROS is not available here: {self.reason}"

    def shutdown(self) -> None:
        pass


class _DemoStop(Exception):
    """Stop, inside a demo trial: jump to parking."""


class _DemoMaze:
    """The demo's arm in the maze, for the live maze view: it walks the configured path piece by piece
    (picking a random way at each junction) with a little wobble, and visits the checkpoints and goal
    by the same rules as actions/checkpoint_monitor.py. Publishes what helpers/maze_view.py would."""

    RATE_HZ = 15.0
    STEP_M = 0.004            # along the path per tick (6 cm/s)

    def __init__(self, params: Dict[str, object], on_maze: Callable[[str, dict], None]) -> None:
        from sinthlab_bringup.helpers import maze_path
        self._mp = maze_path
        prof = str(params["virtual_fixture_profile"])
        fx = f"virtual_fixtures.{prof}."
        if params.get(fx + "type") != "maze":
            raise ValueError(f"profile {prof} is not a maze")
        r = float(params[fx + "corner_radius_m"])
        closed = False
        if fx + "path_a" in params:
            closed = bool(params[fx + "path_closed"])
            self.path = maze_path.build_shape(list(zip(params[fx + "path_a"], params[fx + "path_b"])), closed, r)
        else:
            rails = list(zip(*(params[fx + k] for k in ("corridor_a_min", "corridor_a_max",
                                                         "corridor_b_min", "corridor_b_max"))))
            self.path = maze_path.build_path(rails, r)
        self.nbrs = maze_path.neighbours(self.path)
        axis = str(params[fx + "restricted_axis"]).lower()
        ia, ib = {"x": (1, 2), "y": (0, 2), "z": (0, 1)}[axis]
        wp = fx if fx + "checkpoint_y" in params else "checkpoint_monitor."
        xyz = [params[wp + f"checkpoint_{c}"] for c in "xyz"]
        self.cps = [[float(xyz[ia][i]), float(xyz[ib][i]), float(params[wp + "checkpoint_radius"][i])]
                    for i in range(len(xyz[0]))]
        g = [float(params[wp + f"goal_{c}"]) for c in "xyz"]
        self.goal = [g[ia], g[ib], float(params[wp + "goal_radius"])]
        self.goal_after_all = bool(params[wp + "goal_after_all_checkpoints"])
        self._on_maze = on_maze
        self.geometry = {
            "profile": prof, "restricted_axis": axis,
            "relative": str(params[fx + "corridor_frame"]).lower() != "absolute",
            "on_rail_tol_m": 0.015, "corner_radius_m": r, "closed": closed,
            "pieces": [[[round(a, 4), round(b, 4)] for a, b in maze_path.sample([e], 0.005)] for e in self.path],
            "checkpoints": self.cps, "goal": self.goal, "goal_after_all": self.goal_after_all,
            "_rx": time.time(),
        }
        self.state: Dict[str, object] = {"anchored": False, "trial": 0}
        on_maze("geometry", self.geometry)

    def publish(self, trial: int) -> None:
        self.state.update(t=time.time(), trial=trial, _rx=time.time())
        self._on_maze("state", dict(self.state))

    def walk(self, keep_going: Callable[[float], bool], on_checkpoint: Callable[[int], None],
             limit_sec: float) -> str:
        """One trial in the maze: 'goal', or 'response_timeout' after limit_sec. keep_going(sec) sleeps
        and returns False if the run was stopped (it may raise _DemoStop)."""
        mp, dt = self._mp, 1.0 / self.RATE_HZ
        visited: set = set()
        cur = mp.nearest(self.path, 0.0, 0.0)[0]
        pts = mp.sample([self.path[cur]], self.STEP_M)
        k = min(range(len(pts)), key=lambda j: math.dist(pts[j], (0.0, 0.0)))
        pts = pts[k:] if random.random() < 0.5 else pts[:k + 1][::-1]
        prev, wob, t0 = None, [0.0, 0.0], time.time()
        self.state.update(anchored=True, active=True, visited=[], completed=False, monitoring=True,
                          goal_armed=not self.goal_after_all)
        while True:
            for p in pts:
                wob = [0.85 * w + random.gauss(0.0, 0.0015) for w in wob]
                ee = (p[0] + wob[0], p[1] + wob[1])
                d = mp.nearest(self.path, *ee)[1][2] ** 0.5
                for i, (ca, cb, cr) in enumerate(self.cps):
                    if i not in visited and math.dist(ee, (ca, cb)) <= cr:
                        visited.add(i)
                        on_checkpoint(i)
                armed = not self.goal_after_all or len(visited) == len(self.cps)
                self.state.update(ee=[round(ee[0], 4), round(ee[1], 4)], target=[round(p[0], 4), round(p[1], 4)],
                                  dist_m=round(d, 4), visited=sorted(visited), goal_armed=armed)
                if armed and math.dist(ee, self.goal[:2]) <= self.goal[2]:
                    self.state.update(active=False, monitoring=False, completed=True)
                    self.state.pop("target", None)
                    return "goal"
                if time.time() - t0 > limit_sec:
                    self.state.update(active=False, monitoring=False)
                    self.state.pop("target", None)
                    return "response_timeout"
                if not keep_going(dt):
                    return "stopped"
            # The end of this piece: on along a piece that touches it, not back the way we came.
            end = pts[-1]
            options = [j for j in self.nbrs[cur] if j != prev] or sorted(self.nbrs[cur]) or [cur]
            prev, cur = cur, random.choice(options)
            pts = mp.sample([self.path[cur]], self.STEP_M)
            if math.dist(pts[-1], end) < math.dist(pts[0], end):
                pts = pts[::-1]

    def idle(self) -> None:
        """Out of the maze phase: the fixture lets go; the arm stays where it was."""
        self.state.update(active=False, monitoring=False)
        self.state.pop("target", None)


class DemoBridge:
    """A simulated orchestrator: the same phases, Stop and live-parameter rules as the real one,
    on a fast clock. Logs go to the runner's log stream as if the launch had printed them."""

    def __init__(self, on_status: Callable[[dict], None], on_maze: Callable[[str, dict], None]) -> None:
        self.available, self.reason = True, "demo mode (simulated robot)"
        self._on_status = on_status
        self._on_maze = on_maze
        self._maze: Optional[_DemoMaze] = None
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
            self._stopped = False
            self._held = False
            self._stop_now = False
            self._in_trial = False
            self._trial = 0
            self._phase, self._phase_arg, self._phase_t = "starting", None, time.time()
            self._maze = None
            if exp.key == "maze":
                try:
                    self._maze = _DemoMaze(params, self._on_maze)
                except (KeyError, ValueError, TypeError) as exc:
                    self._log(f"demo maze view off: {exc}", "WARN")
        threading.Thread(target=self._loop, args=(self._run_id,), daemon=True).start()
        if self._maze is not None:
            threading.Thread(target=self._maze_heartbeat, args=(self._run_id, self._maze), daemon=True).start()

    def _maze_heartbeat(self, run_id: int, maze: "_DemoMaze") -> None:
        while run_id == self._run_id:
            maze.publish(self._trial)
            time.sleep(1.0 / maze.RATE_HZ)

    def maze(self) -> dict:
        m = self._maze
        return {"geometry": m.geometry if m else None, "state": dict(m.state) if m else None}

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
            "t": time.time(), "held": self._held,
            "pending": dict(self._pending), "live": dict(self._effective),
            "nsp_sync": bool(self._effective.get("nsp_sync.enabled", False)), "_rx": time.time(),
            # As the real app would report it, set up as the checklist says.
            "smartpad": {"received": True, "from": "demo", "settings": {
                "app": "LbrImpedanceControlServer", "stiffness_profile": self._exp.stiffness_profile,
                "stiffness": ex.SMARTPAD_PROFILES[self._exp.stiffness_profile], "damping": 0.7,
                "nullspace_stiffness": 30.0, "nullspace_damping": 0.7, "send_period_ms": 10}},
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
            if self._stop_now and self._in_trial:
                self._stop_now = False
                raise _DemoStop()
            time.sleep(0.05)
            if self._status and time.time() - self._status["t"] >= 1.0:
                self._publish()
        return run_id == self._run_id

    def _loop(self, run_id: int) -> None:
        self._log("=== AUTOMATED MULTI-TRIAL EXPERIMENT INITIALIZED (demo) ===")
        self._phase_to("parking", None, "First move: to the park pose at speed scale 0.25, from wherever the arm is.")
        if not self._sleep(run_id, 3.0):
            return
        while run_id == self._run_id:
            if not self._trial_once(run_id):
                return
            if self._stopped:
                self._held = True
                self._phase_to("stopped", None, "Stopped: holding at park.")
                while self._sleep(run_id, 0.2):
                    pass
                return

    def _trial_once(self, run_id: int) -> bool:
        """One demo trial, ending at park; False if the run was stopped."""
        try:
            self._in_trial = True
            if not self._trial_body(run_id):
                return False
        except _DemoStop:
            self._phase_to("stop_requested", None, "STOP requested: ending the trial now and parking; then holding.")
        finally:
            self._in_trial = False
        self._phase_to("trial_end", self._trial, f"--- TRIAL {self._trial} OVER ---")
        self._phase_to("recover_start", None, "Parking.")
        return self._sleep(run_id, 1.5)

    def _trial_body(self, run_id: int) -> bool:
        cue = lambda: self._effective.get("audio_cue.enabled", True)
        if run_id == self._run_id:
            with self._lock:
                if self._pending:
                    self._effective.update(self._pending)
                    self._log("Live parameters applied from trial "
                              f"{self._trial + 1}: " + ", ".join(f"{k}={v}" for k, v in self._pending.items()))
                    self._pending = {}
                self._trial += 1
            self._phase_to("trial_start", self._trial, f"--- STARTING TRIAL {self._trial} ---")
            # the wait at park, then the move to start
            if not self._sleep(run_id, float(self._effective.get("move_to_park.hold_sec", 2.0)) + 1.5):
                return False
            self._phase_to("at_start", None, "Arm returned to start. Waiting for a quiet window...")
            if not self._sleep(run_id, float(self._effective.get("quiet_window_sec", 2.0))):
                return False
            self._phase_to("cue_go", None, "Quiet window complete. Sounding audio cue."
                           if cue() else "Quiet window complete. (audio cues off)")
            if self._effective.get("visual_cue.enabled"):
                self._log(f"Visual cue 'play' fired over the {self._effective.get('visual_cue.trigger', 'wifi')} trigger.")
            if not self._sleep(run_id, 0.5):
                return False
            self._phase_to("armed", None, "Monitor armed (baseline locked). Watching for the pull.")
            if not self._sleep(run_id, random.uniform(2.0, 4.5)):
                return False
            if self._exp.key == "maze" and self._maze is not None:
                def reward(i):
                    self._phase_to("checkpoint", i, f"Reward at checkpoint {i}.")
                try:
                    how = self._maze.walk(lambda sec: self._sleep(run_id, sec), reward,
                                          float(self._effective.get("response_timeout_sec", 30.0)))
                finally:
                    self._maze.idle()
                if how == "stopped":
                    return False
                self._phase_to(how, None, "Maze solved! Playing goal cue; waiting for release before reset."
                               if how == "goal" else "Timeout reached. Playing timeout cue.")
            elif self._exp.key == "maze":
                self._phase_to("checkpoint", 0, "Reward at checkpoint 0.")
                if not self._sleep(run_id, 1.0):
                    return False
                self._phase_to("goal", None, "Maze solved! Playing goal cue; waiting for release before reset.")
            elif self._exp.group == "pretraining":
                # Runs for session_sec, or (0) until Stop.
                limit = float(self._effective.get("session_sec", 0.0) or 0.0)
                t0 = time.time()
                while not (limit > 0 and time.time() - t0 >= limit):
                    if not self._sleep(run_id, 0.2):
                        return False
                self._phase_to("session_end", None, "Session over. The arm holds; waiting for release before reset.")
            else:
                thr = self._effective.get(
                    "apple_pluck_impedance_control_displacement.cartesian_displacement_threshold_m", 0.1)
                self._phase_to("snap", thr, f"Threshold reached ({thr} m) — freezing the arm at its current pose.")
            if not self._sleep(run_id, 1.0):
                return False
        return run_id == self._run_id

    def status(self):
        return self._status

    def robot(self) -> dict:
        running = self._status is not None
        return {"lbr_state_hz": 100 if running else 0,
                "session_state": "COMMANDING_ACTIVE" if running else None}

    def set_params(self, exp: ex.Experiment, values: Dict[str, object]) -> Tuple[bool, str]:
        if self._status is None:
            return False, "no experiment is running"
        context = {**self._effective, **self._pending, **values}     # as the real node's gate sees it
        for name, value in values.items():
            if not ex.live_params.is_live(exp.key, name):
                return False, f"'{name}' is a per-run parameter: it was read once at start-up."
            problem = ex.live_params.check_value(name, value, context)
            if problem:
                return False, problem
        with self._lock:
            self._pending.update(values)
        for name, value in values.items():
            self._log(f"Live change accepted: {name} = {value} (applies from trial {self._trial + 1})")
        self._publish()
        return True, "accepted; applies from the next trial"

    def stop(self, exp: ex.Experiment) -> Tuple[bool, str]:
        if self._status is None:
            return False, "no experiment is running"
        with self._lock:
            self._stopped = True
            if self._held:
                return True, "already holding at park"
            self._stop_now = True
        self._log("STOP requested: ending the trial now and parking; then holding.", "WARN")
        return True, "stopping: the trial ends now, the arm parks, then holds"

    def shutdown(self) -> None:
        self.launch_ended()


def make_bridge(demo: bool, robot_name: str, on_status: Callable[[dict], None],
                on_maze: Callable[[str, dict], None]):
    if demo:
        return DemoBridge(on_status, on_maze)
    try:
        return RosBridge(robot_name, on_status, on_maze)
    except Exception as exc:     # rclpy missing or ROS not sourced
        return UnavailableBridge(f"{type(exc).__name__}: {exc}. Source ROS and the workspace "
                                 "(run_gui.sh does), then restart the dashboard.")
