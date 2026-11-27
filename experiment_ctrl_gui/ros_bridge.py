"""The dashboard's ROS side: listens to the orchestrator and robot, sets live parameters, stops.

    <ns>/experiment_status   std_msgs/String (JSON)    published by helpers/experiment_control.py
    <ns>/lbr_state           lbr_fri_idl/LBRState      FRI session state and stream rate; the measured
                                                       joints, for the live view
    <ns>/kuka_clik_controller/target_frame             the CLIK's target (the spring's pull point), for
                             geometry_msgs/PoseStamped  the live view
    <ns>/<node>/set_parameters                         live parameters (the node rejects the rest)
    <ns>/<node>/stop         std_srvs/Trigger          end the trial now, park, and hold
    <ns>/<node>/pause        std_srvs/Trigger          end the trial now, park, hold until resume
    <ns>/<node>/resume       std_srvs/Trigger          start the next trial

It talks to whatever orchestrator is running -- one started from the dashboard, or one started in a
terminal with `ros2 launch` -- so the live controls work either way.

DemoBridge stands in for all of it with a simulated orchestrator, so the dashboard can be tried, and
developed, on a computer with no robot and no ROS.
"""
from __future__ import annotations

import datetime as dt
import json
import random
import threading
import time
from collections import deque
from typing import Callable, Dict, List, Optional, Tuple

import experiments as ex
from sinthlab_bringup.helpers.reach_targets import RULE, TargetPicker, modes_for   # on the path via experiments
from sinthlab_bringup.helpers.rule_schedule import ReversalSchedule
from sinthlab_bringup.helpers.bandit_schedule import DepletingBandit
from sinthlab_bringup.helpers.rail_shapes import direction_name, layout_degs
from sinthlab_bringup.helpers import free_reach
from sinthlab_bringup.helpers import forage
from sinthlab_bringup.helpers import switch_rails

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
        self._q: Optional[list] = None          # measured joints [rad], the latest lbr_state
        self._q_t = 0.0
        self._pull: Optional[list] = None       # the CLIK target position [m], base frame
        self._pull_t = 0.0

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
        try:
            from geometry_msgs.msg import PoseStamped
            self._node.create_subscription(PoseStamped, f"{self._ns}/kuka_clik_controller/target_frame",
                                           self._on_pull, 1)
        except ImportError:
            pass
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
        self._q, self._q_t = list(msg.measured_joint_position), time.time()

    def _on_pull(self, msg) -> None:
        p = msg.pose.position
        self._pull, self._pull_t = [p.x, p.y, p.z], time.time()

    def live(self) -> dict:
        """For the live view: the latest measured joints and CLIK target, with their arrival times."""
        return {"q": self._q, "q_t": self._q_t, "pull": self._pull, "pull_t": self._pull_t}

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
        return self._trigger(exp, "stop")

    def pause(self, exp: ex.Experiment) -> Tuple[bool, str]:
        """Pause: as Stop, but the run goes on: the arm holds at park until resume()."""
        return self._trigger(exp, "pause")

    def resume(self, exp: ex.Experiment) -> Tuple[bool, str]:
        return self._trigger(exp, "resume")

    def _trigger(self, exp: ex.Experiment, service: str) -> Tuple[bool, str]:
        from std_srvs.srv import Trigger
        ok, res = self._call(Trigger, f"{self._ns}/{exp.node}/{service}", Trigger.Request())
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

    def live(self) -> dict:
        return {"q": None, "q_t": 0.0, "pull": None, "pull_t": 0.0}

    def set_params(self, exp, values):
        return False, f"ROS is not available here: {self.reason}"

    def shutdown(self) -> None:
        pass


class _DemoStop(Exception):
    """Stop, inside a demo trial: jump to parking."""


class DemoBridge:
    """A simulated orchestrator: the same phases, Stop and live-parameter rules as the real one,
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
    def launch_started(self, exp: ex.Experiment, params: Dict[str, object], subject: str = "") -> None:
        with self._lock:
            self._run_id += 1
            # The behavioural tables, as the recorder would write them: one trials.csv row per trial
            # (outcome_tables.trial_row on a sidecar-like record), and a sessions.csv row at the end.
            self._subject = subject
            self._run = ex.outcome_tables.run_folder_name(subject, exp.key, dt.datetime.now())
            self._meta: Optional[dict] = None
            self._metas: List[dict] = []
            self.trial_rows: List[dict] = []
            self._exp = exp
            self._params = dict(params)
            self._effective = {k: v for k, v in params.items() if ex.live_params.is_live(exp.key, k)}
            self._pending: Dict[str, object] = {}
            self._stopped = False
            self._held = False
            self._stop_now = False
            self._paused, self._pause_note = False, None
            self._in_trial = False
            self._trial = 0
            self._picker: Optional[TargetPicker] = None     # centre-out goals, one schedule per run
            self._rule: Optional[ReversalSchedule] = None   # the Color Rule, built on entering it
            self._bandit: Optional[DepletingBandit] = None  # N-choice, built at the first trial
            self._reaches: Optional[free_reach.ReachSchedule] = None  # Cued Reaches, free: built at the first trial
            self._forage_rng: Optional[random.Random] = None          # Foraging: the probe draws, per run
            self._phase, self._phase_arg, self._phase_t = "starting", None, time.time()
        threading.Thread(target=self._loop, args=(self._run_id,), daemon=True).start()

    def launch_ended(self) -> None:
        with self._lock:
            self._run_id += 1
            self._status = None
            rows, metas, run = list(getattr(self, "trial_rows", [])), list(getattr(self, "_metas", [])), getattr(self, "_run", "")
        if rows:      # the demo session's sessions.csv row, in the demo data folder
            row = ex.outcome_tables.summarize(run, rows, metas[0]["params"], metas[-1]["params"])
            ex.outcome_tables.upsert_session(row, ex.DATA / ex.outcome_tables.SESSIONS_CSV)

    def run_rows(self) -> List[dict]:
        """This (or the last) demo run's trials.csv rows."""
        with self._lock:
            return list(getattr(self, "trial_rows", []))

    def _log(self, text: str, level: str = "INFO") -> None:
        if self._emit:
            self._emit(f"[{self._exp.node}.py-5] [{level}] [{time.time():.3f}] "
                       f"[lbr.{self._exp.node}]: {text}", None)

    def _publish(self) -> None:
        self._status = {
            "experiment": self._exp.key, "node": f"/lbr/{self._exp.node}", "trial": self._trial,
            "phase": self._phase, "phase_arg": self._phase_arg, "phase_t": self._phase_t,
            "t": time.time(), "held": self._held, "paused": getattr(self, "_paused", False),
            "pause_note": getattr(self, "_pause_note", None) if getattr(self, "_paused", False) else None,
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
        if phase == "trial_start":
            self._meta = {"experiment": self._exp.key, "trial_index": arg, "subject": self._subject,
                          "session_id": self._run, "params": {**self._params, **self._effective},
                          "events": [], "rewards": [], "git": {"sha": "demo", "dirty": False}}
        if self._meta is not None:
            self._meta["events"].append({"event": phase, "arg": arg, "t_wall": self._phase_t})
        if log:
            self._log(log)
        self._publish()

    def _reward(self, token: str) -> None:
        """As helpers/experiment_control.py does on a successful outcome (REWARD_EVENTS)."""
        if self._effective.get("reward_pulse.enabled"):
            out, ms = self._effective.get("reward_pulse.output", 4), self._effective.get("reward_pulse.pulse_ms", 500)
            n, gap = int(self._effective.get("reward_pulse.count", 1) or 1), self._effective.get("reward_pulse.gap_ms", 300)
            if n > 1:
                self._log(f"Reward burst: {n} pulses, {gap} ms apart.")
            if self._meta is not None:
                self._meta["rewards"].append({"output": out, "ms": ms, "count": n, "gap_ms": gap, "event": token})
            self._log(f"Reward: PULSE {out} {ms} (SYNC {out}, valve open {ms} ms) on {token}.")

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
        if self._params.get("tare_calibration.enabled"):
            # The pre-task calibration: to the start pose, hold untouched, park; then wait for Resume.
            self._phase_to("tare_calibration", None, "Pre-task calibration: moving to the start pose to measure the "
                           "force reading at rest. Keep clear of the apple.")
            if not self._sleep(run_id, 2.0 + float(self._params.get("tare_calibration.hold_sec", 4.0)) + 2.0):
                return
            self._log("Pre-task calibration done: tare [0.41, -1.18, 4.83] N (steady within 0.2 N). Parking.")
            if self._params.get("tare_calibration.wait_for_resume") and not self._stopped:
                with self._lock:
                    self._paused, self._pause_note = True, "calibrated"
                self._log("Calibrated. Holding at park: bring the monkey in, then press Resume to start the first trial.")
        first = True
        while run_id == self._run_id:
            if first and self._paused and not self._stopped:
                pass                       # calibrated: hold below before the first trial
            elif not self._trial_once(run_id):
                return
            first = False
            if self._paused and not self._stopped:
                self._held = True
                self._phase_to("paused", None, "Paused: holding at park until Resume.")
                while self._paused and not self._stopped and self._sleep(run_id, 0.2):
                    pass
                if run_id != self._run_id:
                    return
                if not self._stopped:
                    self._held = False
                    self._log("RESUME: starting the next trial.")
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
            if self._paused and not self._stopped:
                self._phase_to("paused", None, "PAUSE requested: ending the trial now and parking; then holding until Resume.")
            else:
                self._phase_to("stop_requested", None, "STOP requested: ending the trial now and parking; then holding.")
        finally:
            self._in_trial = False
        self._phase_to("trial_end", self._trial, f"--- TRIAL {self._trial} OVER ---")
        if self._meta is not None:
            row = ex.outcome_tables.trial_row(self._meta, self._run)
            with self._lock:
                self.trial_rows.append(row)
                self._metas.append(self._meta)
            self._meta = None
        if self._effective.get("force_release.snatch") and not (self._stopped or self._paused):
            sec = float(self._effective.get("force_release.snatch_after_sec", 0.5))
            self._phase_to("snatch_wait", sec, f"The arm holds {sec:g} s, then parks (snatch).")
            if not self._sleep(run_id, sec):
                return False
            self._phase_to("snatch", None, "Snatch: parking now, with or without his hand.")
        self._phase_to("recover_start", None, "Parking.")
        return self._sleep(run_id, 1.5)

    def _trial_body(self, run_id: int) -> bool:
        def go():
            if self._effective.get("visual_cue.enabled"):
                self._phase_to("cue_preview", None, "Cue light on. Waiting before Go.")
                if not self._sleep(run_id, 0.30):
                    return False
            keep = self._effective.get("visual_cue.enabled") and str(self._effective.get("ring_cue.off_at", "go")) != "go"
            self._phase_to("cue_go", None, "Go tone played; the cue stays lit." if keep
                           else "Cue light off. Go tone played if enabled.")
            self._phase_to("armed", None, "Ready for movement.")
            if keep and str(self._effective.get("ring_cue.off_at")) == "move":
                if not self._sleep(run_id, random.uniform(0.3, 1.2)):
                    return False
                self._phase_to("cue_off", 0.01, "He has started: the go cue's light goes out.")
            return True

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
            centre_out = self._effective.get("centre_out.enabled", self._params.get("centre_out.enabled"))
            free = "free_reach.cue" in self._params
            foraging = "forage.n_zones" in self._params
            if not centre_out and not free and not foraging:
                if not go() or not self._sleep(run_id, random.uniform(2.0, 4.5)):
                    return False
            if self._exp.key == "maze":
                self._phase_to("checkpoint", 0, "Reward at checkpoint 0.")
                if not self._sleep(run_id, 1.0):
                    return False
                self._phase_to("goal", None, "Maze solved! Playing goal cue; waiting for release before reset.")
                self._reward("goal")
            elif "n_choice.p_max" in self._effective:
                # N-choice: a simulated chooser that prefers directions that have been paying.
                e = self._effective
                if self._bandit is None:
                    names = [direction_name(a) for a in layout_degs(self._params.get("free_move.ray_layout", "4"))]
                    self._bandit = DepletingBandit(names, start=str(e.get("n_choice.start", "full")))
                self._bandit.configure(float(e["n_choice.p_max"]), float(e.get("n_choice.p_min", 0.1)),
                                       float(e.get("n_choice.deplete", 0.3)), float(e.get("n_choice.recover", 0.1)))
                if not self._sleep(run_id, random.uniform(1.0, 2.5)):
                    return False
                if self._meta is not None:
                    self._meta["n_choice"] = {"p": self._bandit.probabilities(), "best": self._bandit.best()}
                weights = [p ** 3 for p in self._bandit.p]
                name = random.choices(self._bandit.names, weights=weights)[0]
                p = self._bandit.probabilities()[name]
                if self._bandit.choose(name):
                    self._phase_to("choice", name, f"N-choice: {name} (p = {p:.2f}) -- reward.")
                    self._reward("choice")
                else:
                    self._phase_to("choice_unrewarded", name, f"N-choice: {name} (p = {p:.2f}) -- no reward.")
            elif foraging:
                # Foraging: a simulated monkey who finds the zones faster as the session goes on (the same
                # environment all session); a probe pays nothing (actions/forage.py, helpers/forage.py).
                e = {**self._params, **self._effective}
                s = forage.settings_from(lambda n: e.get(n))
                seed = s.seed or forage.day_seed(self._subject or "")
                zones = forage.zones_for(s, seed)
                if self._forage_rng is None:
                    self._forage_rng = random.Random(seed + 1)
                plan = forage.plan_trial(s, self._trial, seed, zones, self._forage_rng)
                if self._meta is not None:
                    self._meta["forage"] = plan.record(s)
                if plan.probe:
                    self._phase_to("probe", None, "Foraging: a probe trial (no cue, no rewards).")
                if not go():
                    return False
                limit = float(e.get("forage.time_limit_sec", 60.0))
                t0, found, times, revisits = time.time(), [], [], 0
                for i in random.sample(range(len(zones)), len(zones)):
                    search = max(1.0, random.gauss(10.0 * 0.85 ** self._trial, 1.0))
                    if plan.probe:
                        search = min(search, float(e.get("forage.probe_sec", 20.0)) - (time.time() - t0))
                    if time.time() - t0 + search > limit or search <= 0:
                        break
                    if plan.warm_brightness > 0:
                        self._phase_to("warm_on", None, "")
                    if not self._sleep(run_id, search):
                        return False
                    if plan.probe:
                        self._phase_to("zone_enter", i, f"Foraging (probe): in zone {i + 1}.")
                        continue
                    found.append(i)
                    times.append(round(time.time() - t0, 3))
                    self._phase_to("zone", i, f"Foraging: zone {i + 1} found -- reward.")
                    self._reward("zone")
                    if random.random() < 0.3:
                        revisits += 1
                        self._phase_to("zone_revisit", i, "")
                if self._meta is not None:
                    self._meta["forage_result"] = {"found": found, "n_found": len(found), "n_zones": len(zones),
                                                   "t_zone_s": times, "revisits": revisits,
                                                   "path_m": round(random.uniform(0.1, 0.6), 3),
                                                   "warm_on_s": round(random.uniform(0, 3), 2) if plan.warm_brightness else 0.0}
                if plan.probe:
                    if not self._sleep(run_id, max(0.0, float(e.get("forage.probe_sec", 20.0)) - (time.time() - t0))):
                        return False
                    self._phase_to("probe_end", None, "Foraging: probe over.")
                elif len(found) == len(zones):
                    self._phase_to("forage_done", None, "Foraging: every zone found.")
                else:
                    self._phase_to("response_timeout", None, "Foraging: time up.")
            elif free:
                # Cued Reaches, free: a simulated monkey -- grips after a reaction time, reaches; a perturbation
                # makes him miss more often (actions/free_reach.py, helpers/free_reach.py).
                e = {**self._params, **self._effective}
                s = free_reach.settings_from(lambda n: e.get(n))
                if self._reaches is None:
                    self._reaches = free_reach.ReachSchedule(s)
                self._reaches.configure(s)
                plan = self._reaches.next()
                delay = round(random.uniform(float(e.get("free_reach.cue_delay_min_sec", 0.3)),
                                             max(float(e.get("free_reach.cue_delay_min_sec", 0.3)),
                                                 float(e.get("free_reach.cue_delay_max_sec", 0.3)))), 3)
                if self._meta is not None:
                    self._meta["free_reach"] = {**plan.record(), "cue_delay_sec": delay}
                self._phase_to("target", plan.direction or "any", f"Free reach: {plan.direction or 'any direction'}.")
                if not go():
                    return False
                rt = random.uniform(0.25, 0.6)
                done = 0.0
                if plan.perturb == "jump":
                    at = random.uniform(s.jump_delay_min_sec, s.jump_delay_max_sec)
                    if at < rt:
                        if not self._sleep(run_id, at):
                            return False
                        self._phase_to("jump", plan.angle_deg, "Free reach: the apple jumps.")
                        if not self._sleep(run_id, s.jump_sec):
                            return False
                        self._phase_to("jump_end", None, "")
                        done = at + s.jump_sec
                if not self._sleep(run_id, max(0.05, rt - done)):
                    return False
                self._phase_to("grip", 4.0, "")
                if plan.perturb == "jump" and done == 0.0:
                    self._phase_to("jump_skipped", None, "Free reach: gripped before the jump -- no jump this trial.")
                hit = plan.perturb is not None and (plan.perturb == "bump" or done > 0.0)
                if plan.perturb == "bump":
                    if not self._sleep(run_id, 0.2):
                        return False
                    self._phase_to("bump", plan.angle_deg, "Free reach: bump.")
                    if not self._sleep(run_id, s.bump_sec):
                        return False
                    self._phase_to("bump_end", None, "")
                if not self._sleep(run_id, random.uniform(0.3, 0.7)):
                    return False
                ok = random.random() < (0.97 if plan.direction is None else 0.6 if hit else 0.85)
                if plan.direction is None:
                    err, ang = None, round(random.uniform(0.0, 360.0), 1)
                else:
                    err = round(random.gauss(0.0, s.cone_deg / 3) if ok else
                                random.choice((-1, 1)) * random.uniform(s.cone_deg + 1, 40.0), 1)
                    ang = round((free_reach.angle_of(free_reach.unit(plan.direction)) + err) % 360.0, 1)
                    ok = abs(err) <= s.cone_deg
                if self._meta is not None:
                    self._meta["free_reach_result"] = {"perturbed": int(hit), "crossing_deg": ang, "error_deg": err}
                    if plan.perturb == "bump" and plan.depth_rule:
                        v = round(random.gauss(0.0, 0.04), 3)
                        sign, basis = free_reach.depth_sign(plan.depth_rule, v, 0.0, s.bump_depth_still_mps, random)
                        self._meta["free_reach_result"].update(depth_dir="toward" if sign > 0 else "away",
                                                         depth_force_n=s.bump_depth_force_n,
                                                         depth_m=round(s.bump_extent_m(s.bump_depth_force_n), 4),
                                                         depth_basis=basis, depth_speed_mps=v)
                if ok:
                    self._phase_to("reach", ang, "Free reach: to the target -- reward.")
                    self._reward("reach")
                else:
                    self._phase_to("reach_wrong", ang, "Free reach: outside the target -- no reward.")
            elif centre_out:
                # Move Vertical / Horizontal, centre-out: one reach per trial, to the lit end or the other
                # (none with the other half of the rail closed: actions/centre_out.py).
                mode = str(self._effective.get("centre_out.target", self._params.get("centre_out.target", "either")))
                rail = switch_rails.effective({**self._params, **self._effective})
                ends = ("down", "up") if (rail and rail[0] == "z") else ("left", "right")
                if self._picker is None or tuple(self._picker.ends) != ends:      # Rails, switchable: a new rail
                    self._picker = TargetPicker(ends)
                if mode == RULE:
                    # The Color Rule: a simulated learner, right more often the longer the block runs, so
                    # the criterion is met and the mapping reverses (unsignaled) every few dozen trials.
                    e = self._effective
                    if self._rule is None:
                        self._rule = ReversalSchedule(ends, str(e.get("rule.red_end", ends[1])))
                    self._rule.configure(int(e.get("rule.window", 20)), float(e.get("rule.criterion_pct", 80)),
                                         int(e.get("rule.min_trials", 20)), int(e.get("rule.max_repeats", 3)))
                    mapping = self._rule.mapping() if self._rule.take_reversal() else None
                    if mapping:
                        self._phase_to("reversal", mapping, f"(operator only) Color Rule: now {mapping}.")
                    if self._meta is not None:
                        self._meta["rule"] = self._rule.state()
                    colour = self._rule.draw_colour()
                    target = self._rule.end_for(colour)
                    self._phase_to("cue_colour", colour, "")
                    self._phase_to("target", target, f"Color Rule: {colour} -- the goal is {target}.")
                    if not go() or not self._sleep(run_id, random.uniform(1.0, 3.0)):
                        return False
                    p_right = min(0.95, 0.35 + 0.04 * self._rule.choices_in_block)
                    end = target if random.random() < p_right else [x for x in ends if x != target][0]
                    if self._rule.record(end == target):
                        self._log(f"Color Rule: criterion met -- reverses, unsignaled, from the next trial: "
                                  f"{self._rule.mapping()} (block {self._rule.block}).")
                    closed = None
                else:
                    self._rule = None
                    self._picker.set_mode(mode if mode in modes_for(ends) and mode != RULE else "either",
                                          int(self._effective.get("centre_out.max_repeats", 0) or 0))
                    target = self._picker.pick()
                    closed = (self._picker.other(target) if target in ends and mode == "restricted" else None)
                    if closed:
                        self._phase_to("closed_end", closed, f"Free move: the {closed} half of the rail is closed this trial.")
                    self._phase_to("target", target, f"Centre-out: the goal is {target}.")
                    if not go() or not self._sleep(run_id, random.uniform(1.0, 3.0)):
                        return False
                    end = target if closed else random.choice(ends)
                    if mode == "retry" and target in ends and end != target:
                        # Cued, can correct: the wrong end does nothing; he goes on to the cued end.
                        self._phase_to("wrong_end", end, f"Centre-out: reached the {end} end, the target is {target}: he can go on.")
                        if not self._sleep(run_id, random.uniform(1.0, 2.0)):
                            return False
                        end = target
                if target in ("either", end):
                    self._phase_to("reach", end, f"Centre-out: reached the {end} end -- reward.")
                    self._reward("reach")
                else:
                    self._phase_to("reach_wrong", end, f"Centre-out: reached the {end} end, the target was {target} -- no reward.")
            elif self._exp.group == "pretraining":
                # Runs for session_sec, or (0) until Stop. Long moves (Free Move): a movement every few
                # seconds, rewarded if at least long_move.min_m (actions/long_move.py).
                limit = float(self._effective.get("session_sec", 0.0) or 0.0)
                each_move = self._effective.get("long_move.reward") == "each_move"
                t0, next_move = time.time(), time.time() + random.uniform(1.0, 3.0)
                while not (limit > 0 and time.time() - t0 >= limit):
                    if not self._sleep(run_id, 0.2):
                        return False
                    if each_move and time.time() >= next_move:
                        self._phase_to("move_start", None, "")
                        if not self._sleep(run_id, random.uniform(0.5, 1.2)):
                            return False
                        length = random.uniform(0.03, 0.2)
                        self._phase_to("move_end", f"length={length:.3f}", "")
                        if length >= float(self._effective.get("long_move.min_m", 0.1)):
                            self._phase_to("long_move", round(length, 3),
                                           f"Long move: {length * 100:.1f} cm -- reward.")
                            self._reward("long_move")
                        else:
                            self._log(f"Movement {length * 100:.1f} cm: too short, no reward.")
                        next_move = time.time() + random.uniform(1.5, 4.0)
                self._phase_to("session_end", None, "Session over. The arm holds; waiting for release before reset.")
                if not each_move:
                    self._reward("session_end")
            else:
                thr = self._effective.get(
                    "apple_pluck_impedance_control_displacement.cartesian_displacement_threshold_m", 0.1)
                self._phase_to("snap", thr, f"Threshold reached ({thr} m) — freezing the arm at its current pose.")
                self._reward("snap")
            if not self._sleep(run_id, 1.0):
                return False
        return run_id == self._run_id

    def status(self):
        return self._status

    def robot(self) -> dict:
        running = self._status is not None
        return {"lbr_state_hz": 100 if running else 0,
                "session_state": "COMMANDING_ACTIVE" if running else None}

    def live(self) -> dict:
        """A simulated arm for the live view: an offset from the start [m] -- it wanders while the
        trial is armed (along the rail on a rail task), and is at the start otherwise."""
        if self._status is None:
            return {"q": None, "q_t": 0.0, "pull": None, "pull_t": 0.0}
        moving = self._phase in ("armed", "checkpoint")
        t = time.time() - self._phase_t if moving else 0.0
        import math
        rail = switch_rails.effective({**self._params, **self._effective})
        axis = rail[0] if rail else "none"
        if self._exp.node == "free_move_orchestrator" and axis in ("y", "z"):
            lo, hi = rail[1], rail[2]
            v = (hi if math.sin(0.8 * t) >= 0 else -lo) * math.sin(0.8 * t)
            off = [0.0, v, 0.0] if axis == "y" else [0.0, 0.0, v]
        else:
            r = min(1.0, t / 3.0)
            off = [0.02 * r * math.sin(0.5 * t), 0.12 * r * math.sin(0.9 * t), 0.06 * r * math.sin(1.3 * t) - 0.03 * r]
        lead = [o * 1.05 for o in off]
        return {"offset": off, "pull_offset": lead if self._exp.key != "apple_pluck" else None, "t": time.time()}

    def set_params(self, exp: ex.Experiment, values: Dict[str, object]) -> Tuple[bool, str]:
        if self._status is None:
            return False, "no experiment is running"
        context = {**self._effective, **self._pending, **values}     # as the real node's gate sees it
        for name, value in values.items():
            if not ex.live_params.is_live(exp.key, name):
                return False, f"'{name}' is a per-run parameter: it was read once at start-up."
            problem = ex.live_params.check_value(name, value, context)
            if not problem and name == exp.paradigm_param and exp.paradigms and \
                    str(value) not in [k for k, _ in exp.paradigms]:   # e.g. "left" on the vertical rail
                problem = f"{name} must be one of {[k for k, _ in exp.paradigms]} in {exp.label}, got {value!r}"
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

    def pause(self, exp: ex.Experiment) -> Tuple[bool, str]:
        if self._status is None:
            return False, "no experiment is running"
        with self._lock:
            if self._stopped:
                return False, "stopping: the run is ending"
            if self._paused:
                return True, "already paused"
            self._paused, self._pause_note = True, None
            if not self._held:
                self._stop_now = True
        self._log("PAUSE requested: ending the trial now and parking; then holding until Resume.", "WARN")
        self._publish()
        return True, "pausing: the trial ends now, the arm parks, then holds until Resume"

    def resume(self, exp: ex.Experiment) -> Tuple[bool, str]:
        if self._status is None:
            return False, "no experiment is running"
        with self._lock:
            if not self._paused:
                return True, "not paused"
            self._paused = False
            self._pause_note = None
            self._stop_now = False
        self._publish()
        return True, "resumed: the next trial starts now"

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
