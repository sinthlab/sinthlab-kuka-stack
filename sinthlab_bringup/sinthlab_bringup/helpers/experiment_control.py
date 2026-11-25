#!/usr/bin/env python3
"""What every orchestrator exposes for being driven from outside -- the dashboard or the CLI.

    <ns>/experiment_status       std_msgs/String, JSON, latched   where the experiment is right now
    <ns>/<orchestrator>/stop     std_srvs/Trigger                 end the trial NOW, park, and hold
    <ns>/<orchestrator>/pause    std_srvs/Trigger                 end the trial NOW, park, hold until resume
    <ns>/<orchestrator>/resume   std_srvs/Trigger                 start the next trial
    ros2 param set <ns>/<orchestrator> <name> <value>             live parameters only

LIVE PARAMETERS apply at the next trial boundary, never mid-trial, so each trial runs under one
configuration and its sidecar records it. The exception is IMMEDIATE parameters (live_params.py: the
move speed), applied the moment they are set (from the next move) and logged in the sidecar with when. Which ones are
live is decided in helpers/live_params.py;
a set on anything else is REJECTED with a reason, because the orchestrator read it once at start-up
and the change would otherwise look accepted while doing nothing.

STOP (the dashboard's Stop) ends the trial in progress at once -- the orchestrator's on_stop_request
hook stops whatever is running and parks the arm -- and no further trial starts: the control holds at
park. Only then does the dashboard end the launch, so FRI always closes with the arm parked, out of
reach. Stop is not a safety function -- the arm stays under control and compliant throughout.

PAUSE is Stop without the end: the same stop hooks end the trial at once (marked `paused` instead of
`stop_requested`) and park the arm, and begin_trial then holds at park, keeping the next trial, until
RESUME starts it. The launch, FRI and the SmartPad app keep running throughout, so a pause costs no
restart. Live changes made while paused apply from the resumed trial. Stop while paused ends the run.

NSP SYNC. `on_event` is what TrialRecorder calls synchronously inside mark(); with
`nsp_sync.enabled`, trial_start and trial_end each send the cabinet one UDP datagram,
"PULSE <output> <width ms>" to <nsp_sync.cabinet_ip>:30002, and the cabinet's TtlPulser
(sunrise_controller_code/) pulses that EL2124 output -- one BNC line per event to the Blackrock NSP.
The send never blocks or raises; the cabinet's replies are read on a background thread, and a
refusal or silence is logged (see reward_and_sync_box/README.md, "Sync: TTL pulses to the NSP").

REWARD. With `reward_pulse.enabled`, a successful trial's outcome (REWARD_EVENTS: the snap, the maze
goal, a pre-training session that ran its full session_sec, or -- in the centre-out version of Move
Vertical / Horizontal, which calls set_reward_events(("reach",)) -- each rewarded reach) sends "PULSE <reward_pulse.output> <reward_pulse.pulse_ms>" the same way; that output's jack (SYNC 4)
is cabled to the reward box's NSP jack, so the pulse holds the juice valve open for pulse_ms. With
reward_pulse.count > 1 the reward is a burst: that many pulses, each pulse_ms long, reward_pulse.gap_ms of
closed valve between them (the cabinet stretches a pulse that is still high, so they must not overlap).
Failures (timeout, early touch, safety trip, Stop) send none. Independent of nsp_sync.enabled; same
cabinet_ip.

QUIET-WINDOW STAIRCASE (helpers/quiet_ramp.py, quiet_ramp.*; off by default). At each trial_end, a trial
that reached the go cue (`cue_go`) counts toward lengthening quiet_window_sec and an `early_touch`
shortens it; a change is set as the live parameter quiet_window_sec, so it applies from the next trial
and the dashboard shows it. Every task, since every task marks those events.
"""
from __future__ import annotations

import json
import socket
import threading
import time
from typing import Callable, Dict, List, Optional

from rclpy.node import Node as rclpyNode
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from std_srvs.srv import Trigger

from sinthlab_bringup.helpers.node_utils import get_required_param
from sinthlab_bringup.helpers.live_params import LIVE_PARAMS, check_value, is_immediate, is_live
from sinthlab_bringup.helpers.live_params import PAIRED
from sinthlab_bringup.helpers import quiet_ramp
from sinthlab_bringup.helpers.smartpad_link import SmartPadLink

# Event token -> the parameter naming its EL2124 output. Two per trial are enough for alignment
# (trial_start and trial_end give offset and rate); every other event is in the CSV on the robot
# clock, and the fitted map carries it along. Each event has its own line, so a dropped pulse shows
# as a missing edge on that line instead of mispairing every later trial.
NSP_OUTPUTS: Dict[str, str] = {
    "trial_start": "nsp_sync.trial_start_output",
    "trial_end": "nsp_sync.trial_end_output",
}
# The outcome tokens that mean success: each sends the reward pulse.
REWARD_EVENTS = ("snap", "goal", "session_end")    # the default; set_reward_events() replaces it
# Must equal TtlPulser.DEFAULT_PORT in sunrise_controller_code/TtlPulser.java.
PULSE_PORT = 30002

STATUS_TOPIC = "experiment_status"
_HEARTBEAT_SEC = 1.0


def _plain(value):
    """rclpy hands arrays back as array.array; JSON and comparisons want lists."""
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, (list, tuple)):
        return list(value)
    return value


class ExperimentControl:
    def __init__(self, node: rclpyNode, experiment: str) -> None:
        self._node = node
        self._experiment = experiment
        self._log = node.get_logger()

        self._trial = 0
        self._phase = "starting"
        self._phase_arg = None
        self._phase_t = time.time()
        self._stopped = False                               # no further trial: hold at park
        self._held = False                                  # stopped or paused, and parked
        self._paused = False                                # pause: hold at park until resume
        self._resume_fn: Optional[Callable[[], None]] = None  # the next trial, kept while paused
        self._pending: Dict[str, object] = {}               # accepted, applies at the next trial
        self._reloaders: List[Callable[[], None]] = []
        self._stop_hooks: List[Callable[[], None]] = []
        self._validators: List[Callable[[str, object, dict], Optional[str]]] = []
        self._immediate: Dict[str, Callable[[object], None]] = {}   # name -> apply(value), right away
        self._change_log: Optional[Callable[[str, object], None]] = None
        # The values the CURRENT trial runs with. rclpy stores an accepted set immediately, but the
        # actions only re-read at the next trial boundary, so the parameter store runs ahead of
        # what is in effect; status reports this snapshot, and `pending` separately.
        self._effective: Dict[str, object] = {
            name: _plain(node.get_parameter(name).value)
            for name in node._parameters if is_live(experiment, name)}   # noqa: SLF001
        self._reward_events = REWARD_EVENTS
        self._sequence_rewards = False
        self._reward_ready_at = 0.0
        self._reward_timers: List[threading.Timer] = []
        self._reward_wait = None
        self._nsp = NspPulser(self._log)
        self._read_nsp()
        self._smartpad = SmartPadLink.get(self._log)    # what the SmartPad app reports it runs with

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._pub = node.create_publisher(String, STATUS_TOPIC, latched)
        node.create_service(Trigger, "~/stop", self._on_stop)
        node.create_service(Trigger, "~/pause", self._on_pause)
        node.create_service(Trigger, "~/resume", self._on_resume)
        node.add_on_set_parameters_callback(self._gate)
        node.create_timer(_HEARTBEAT_SEC, self._publish)
        self._publish()

    # ------------------------------------------------------------------ orchestrator API

    def set_reward_events(self, tokens, *, sequence: bool = False) -> None:
        """Select reward events. Sequence them when several can occur in one trial."""
        self._reward_events = tuple(tokens)
        self._sequence_rewards = sequence

    def pays(self, token: str) -> bool:
        """Whether this event sends the reward pulse (reward_pulse.enabled and one of the reward events): the
        moment the reward cue shows."""
        return self._reward is not None and token in self._reward_events

    def on_reload(self, fn: Callable[[], None]) -> None:
        """Register a function that re-reads live parameters. Called at a trial boundary."""
        self._reloaders.append(fn)

    def apply_immediately(self, handlers: Dict[str, Callable[[object], None]],
                          log: Callable[[str, object], None]) -> None:
        """How to apply each IMMEDIATE live parameter (live_params.IMMEDIATE) the moment it is set --
        e.g. "move_to_start.speed_scale" -> that move's set_speed_scale -- and where to record each
        change (TrialRecorder.note_change: the trial's sidecar)."""
        self._immediate.update(handlers)
        self._change_log = log

    def add_validator(self, fn: Callable[[str, object, dict], Optional[str]]) -> None:
        """A check of a live change that needs the orchestrator's own state (the robot model): fn(name,
        value, context) returns the reason to refuse it, or None. Runs after live_params.check_value."""
        self._validators.append(fn)

    def on_stop_request(self, fn: Callable[[], None]) -> None:
        """Register the orchestrator's Stop: end the trial in progress at once and park."""
        self._stop_hooks.append(fn)

    def stop_token(self) -> str:
        """The outcome the orchestrator's stop hook marks the interrupted trial with: `paused` for a pause,
        `stop_requested` for Stop."""
        return "paused" if self._paused and not self._stopped else "stop_requested"

    def hold(self, why: str) -> None:
        """Stop from the orchestrator itself, after the current trial: no further trial starts, and the
        arm holds at park until Stop ends the launch. For a run that must not continue as set up."""
        if not self._stopped:
            self._stopped = True
            self._log.error(f"Holding at park: {why}. Press Stop to end the run.")
            self._publish()

    def smartpad(self) -> SmartPadLink:
        """What the SmartPad app reports it runs with (helpers/smartpad_link.py)."""
        return self._smartpad

    def begin_trial(self, start_fn: Callable[[], None]) -> None:
        """Run the next trial now -- or, once stopped, hold at park instead. Orchestrators call this
        at the end of every trial, with the arm at park."""
        if self._stopped:
            self._held = True
            self._set_phase("stopped")
            self._log.info("Stopped: holding at park.")
            return
        if self._paused:
            self._held = True
            self._resume_fn = start_fn
            self._set_phase("paused")
            self._log.info("Paused: holding at park until Resume.")
            return
        delay = self._reward_ready_at - time.monotonic() if self._sequence_rewards else 0.0
        if delay > 0:
            def after_rewards():
                self._node.destroy_timer(self._reward_wait)
                self._reward_wait = None
                self.begin_trial(start_fn)

            self._reward_wait = self._node.create_timer(delay, after_rewards)
            return
        self._apply_pending()
        start_fn()

    def on_event(self, token: str, arg=None) -> Optional[dict]:
        """Every trial event: NSP sync pulse first (timing matters), then status. Returns what was
        commanded if the event sent the reward pulse ({"output", "ms"}), else None -- the recorder keeps
        it in the trial's sidecar (`rewards`), from which trials.csv counts rewards and valve time."""
        if token == "cue_audio_end":
            return None  # playback telemetry must not replace the current trial phase
        if self._nsp_enabled and token in NSP_OUTPUTS:
            self._nsp.pulse(self._nsp_cfg[token], self._nsp_cfg["width_ms"], token)
        reward = None
        if self._reward is not None and token in self._reward_events:
            output, ms, count, gap = self._reward
            now = time.monotonic()
            spacing = (ms + gap) / 1000.0
            delay = 0.0
            sequence = self._sequence_rewards
            if sequence:
                delay = max(0.0, self._reward_ready_at - now)
                self._reward_ready_at = now + delay + count * spacing
                self._reward_timers = [t for t in self._reward_timers if t.is_alive()]
            for k in range(count):
                label = f"reward on {token}" if k == 0 else f"reward {k + 1}/{count} on {token}"
                wait = delay + k * spacing
                if wait == 0:
                    self._nsp.pulse(output, ms, label)
                else:
                    timer = threading.Timer(wait, self._nsp.pulse, args=(output, ms, label))
                    timer.daemon = True
                    timer.start()
                    if sequence:
                        self._reward_timers.append(timer)
            self._log.info(f"Reward: PULSE {output} {ms} (SYNC {output}, valve open {ms} ms)"
                           + (f" x {count}, {gap} ms apart" if count > 1 else "") + f" on {token}.")
            reward = {"output": output, "ms": ms, "count": count, "gap_ms": gap}
            if delay > 0:
                reward["delay_ms"] = round(delay * 1000, 1)
        self._ramp_event(token)
        if token == "trial_start" and arg is not None:
            self._trial = int(arg)
        self._set_phase(token, arg)
        return reward

    # ------------------------------------------------------------------ the quiet-window staircase

    def _ramp_event(self, token: str) -> None:
        """Note what the trial did (reached the go, or touched early); at trial_end, step the staircase."""
        if token == "trial_start":
            self._ramp_flags = set()
        elif token in ("cue_go", "early_touch"):
            if not hasattr(self, "_ramp_flags"):
                self._ramp_flags = set()
            self._ramp_flags.add(token)
        elif token == "trial_end":
            flags, self._ramp_flags = getattr(self, "_ramp_flags", set()), set()
            try:
                self._ramp_step("cue_go" in flags, "early_touch" in flags)
            except Exception as exc:                      # never let the staircase break a trial
                self._log.warn(f"Quiet-window staircase skipped: {exc}")

    def _ramp_step(self, went: bool, early: bool) -> None:
        node = getattr(self, "_node", None)
        if node is None or not node.has_parameter("quiet_ramp.enabled") or not node.has_parameter("quiet_window_sec"):
            return
        s = quiet_ramp.settings_from(lambda name: node.get_parameter(name).value)
        current = float(node.get_parameter("quiet_window_sec").value)
        new, self._ramp_streak, why = quiet_ramp.step(current, went, early, getattr(self, "_ramp_streak", 0), s)
        if why is None:
            return
        from rclpy.parameter import Parameter
        result = node.set_parameters([Parameter("quiet_window_sec", value=float(new))])
        if result and not result[0].successful:
            self._log.warn(f"Quiet-window staircase: {why} refused ({result[0].reason}).")
            return
        self._log.info(f"Quiet-window staircase: {why}, from the next trial.")

    # ------------------------------------------------------------------ internals

    def _set_phase(self, phase: str, arg=None) -> None:
        self._phase, self._phase_arg, self._phase_t = phase, arg, time.time()
        self._publish()

    def _publish(self) -> None:
        status = {
            "experiment": self._experiment,
            "node": self._node.get_fully_qualified_name(),
            "trial": self._trial,
            "phase": self._phase,
            "phase_arg": self._phase_arg,
            "phase_t": self._phase_t,
            "t": time.time(),
            "held": self._held,
            "paused": self._paused,
            "pending": self._pending,
            "live": self._effective,
            "nsp_sync": self._nsp_enabled,
            "smartpad": self._smartpad.snapshot(),
        }
        self._pub.publish(String(data=json.dumps(status, default=str)))

    def _apply_pending(self) -> None:
        if not self._pending:
            return
        for fn in self._reloaders:
            fn()
        self._read_nsp()
        self._effective.update(self._pending)
        changes = ", ".join(f"{k}={v}" for k, v in self._pending.items())
        self._log.info(f"Live parameters applied from trial {self._trial + 1}: {changes}")
        self._pending = {}

    def _on_stop(self, request, response):
        self._stopped = True
        for timer in self._reward_timers:
            timer.cancel()
        self._reward_timers.clear()
        if self._reward_wait is not None:
            self._node.destroy_timer(self._reward_wait)
            self._reward_wait = None
            self.begin_trial(lambda: None)  # already parked, with the trial saved
        if not self._held:
            self._log.warn("STOP requested: ending the trial now and parking; then holding.")
            for fn in self._stop_hooks:
                try:
                    fn()
                except Exception as exc:
                    self._log.error(f"stop hook failed: {exc}")
            response.message = "stopping: the trial ends now, the arm parks, then holds"
        else:
            response.message = "already holding at park"
        response.success = True
        self._publish()
        return response

    def _on_pause(self, request, response):
        response.success = True
        if self._stopped:
            response.success, response.message = False, "stopping: the run is ending"
        elif self._paused:
            response.message = "already paused" + (" at park" if self._held else "; parking")
        else:
            self._paused = True
            if not self._held:
                self._log.warn("PAUSE requested: ending the trial now and parking; then holding until Resume.")
                for fn in self._stop_hooks:
                    try:
                        fn()
                    except Exception as exc:
                        self._log.error(f"pause (stop hook) failed: {exc}")
            response.message = "pausing: the trial ends now, the arm parks, then holds until Resume"
        self._publish()
        return response

    def _on_resume(self, request, response):
        response.success = True
        if self._stopped:
            response.success, response.message = False, "stopping: the run is ending"
        elif not self._paused:
            response.message = "not paused"
        else:
            self._paused = False
            fn, self._resume_fn = self._resume_fn, None
            if self._held and fn is not None:
                self._held = False
                self._log.info("RESUME: starting the next trial.")
                response.message = "resumed: the next trial starts now"
                self.begin_trial(fn)
            else:
                response.message = "resumed: the run carries on once the arm is parked"
        self._publish()
        return response

    def _gate(self, params) -> SetParametersResult:
        # Values in force once this request lands -- the node's current ones, overridden by the request
        # -- so parameters that are only valid together (the perturbation's r and plane) are checked
        # together, whether they arrive one at a time or in the same call.
        context = {n: _plain(self._node.get_parameter(n).value) for n in PAIRED
                   if self._node.has_parameter(n)}
        context.update({p.name: _plain(p.value) for p in params})
        # The resistance base is what the SmartPad app reports (resistance_softener.py), not the YAML.
        reported = self._smartpad.translational_stiffness()
        if reported is not None and "resistance.base_stiffness_n_per_m" in context:
            context["resistance.base_stiffness_n_per_m"] = reported
        for p in params:
            if not self._node.has_parameter(p.name):
                continue            # a first declaration, not a change
            if not is_live(self._experiment, p.name):
                return SetParametersResult(
                    successful=False,
                    reason=(f"'{p.name}' is a per-run parameter: it was read once at start-up, "
                            f"so changing it now would do nothing. Stop, edit it, and start again. "
                            f"Live parameters: {', '.join(LIVE_PARAMS[self._experiment])}"))
            problem = check_value(p.name, _plain(p.value), context)
            for check in self._validators:
                if problem:
                    break
                try:
                    problem = check(p.name, _plain(p.value), context)
                except Exception as exc:
                    problem = f"could not check {p.name}: {exc}"
            if problem:
                return SetParametersResult(successful=False, reason=problem)
        for p in params:
            if not self._node.has_parameter(p.name):
                continue
            value = _plain(p.value)
            if is_immediate(p.name) and p.name in self._immediate:
                self._immediate[p.name](value)
                self._effective[p.name] = value
                if self._change_log is not None:
                    self._change_log(p.name, value)
                self._log.info(f"Live change applied now: {p.name} = {value}")
            else:
                self._pending[p.name] = value
                self._log.info(f"Live change accepted: {p.name} = {value} "
                               f"(applies from trial {self._trial + 1})")
        self._publish()
        return SetParametersResult(successful=True)

    def _read_nsp(self) -> None:
        """(Re-)read nsp_sync.*: at start-up and at each trial boundary, like every live parameter."""
        p = lambda name: get_required_param(self._node, name)   # noqa: E731
        self._nsp_enabled = bool(p("nsp_sync.enabled"))
        self._nsp_cfg = {token: int(p(name)) for token, name in NSP_OUTPUTS.items()}
        self._nsp_cfg["width_ms"] = int(p("nsp_sync.pulse_width_ms"))
        # The reward: (output, ms, pulses per reward, gap between them [ms]) when on, else None. A YAML
        # without count / gap_ms (an older one) gives one pulse.
        self._reward = None
        if self._node.has_parameter("reward_pulse.enabled") and bool(p("reward_pulse.enabled")):
            has = self._node.has_parameter
            self._reward = (int(p("reward_pulse.output")), int(p("reward_pulse.pulse_ms")),
                            max(1, int(p("reward_pulse.count"))) if has("reward_pulse.count") else 1,
                            int(p("reward_pulse.gap_ms")) if has("reward_pulse.gap_ms") else 300)
        self._nsp.set_target(str(p("nsp_sync.cabinet_ip")).strip(), self._nsp_enabled or self._reward is not None)


class NspPulser:
    """Pulse commands to the cabinet's TtlPulser over UDP, without ever blocking or raising: pulse()
    runs inside TrialRecorder.mark(). The cabinet answers each command ("OK PULSE 1 10" / "ERR <why>");
    a background thread reads the answers, logs the first OK and every refusal, and warns when a
    pulse went unanswered -- no app serving UDP 30002, a wrong cabinet_ip, or the network."""

    _ANSWER_SEC = 1.0      # a pulse not answered in this long is reported as unanswered

    def __init__(self, logger) -> None:
        self._log = logger
        self._lock = threading.Lock()
        self._host: Optional[str] = None
        self._sent: List[tuple] = []        # (time sent, command) awaiting an answer
        self._confirmed = False             # an OK came back from the current host
        self._silent_warned = False
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.bind(("", 0))
            self._sock.settimeout(0.2)
        except OSError as exc:
            self._sock = None
            self._log.error(f"NSP sync: cannot open a UDP socket ({exc}); no pulses will be sent.")
            return
        threading.Thread(target=self._listen, name="nsp-pulse", daemon=True).start()

    def set_target(self, host: str, enabled: bool) -> None:
        with self._lock:
            if host == self._host:
                return
            self._host, self._confirmed, self._silent_warned = host, False, False
            self._sent.clear()
        if enabled:
            self._log.info(f"NSP sync / reward: pulses go to {host}:{PULSE_PORT}.")

    def pulse(self, output: int, width_ms: int, token: str) -> None:
        if self._sock is None or self._host is None:
            return
        command = f"PULSE {output} {width_ms}"
        try:
            self._sock.sendto(command.encode("ascii"), (self._host, PULSE_PORT))
        except OSError as exc:
            self._log.warn(f"NSP sync: '{command}' ({token}) to {self._host}:{PULSE_PORT} failed: {exc}",
                           throttle_duration_sec=5.0)
            return
        with self._lock:
            self._sent.append((time.monotonic(), command))

    def _listen(self) -> None:
        while True:
            try:
                data, (host, _port) = self._sock.recvfrom(256)
                reply = data.decode("ascii", "replace").strip()
            except socket.timeout:
                reply, host = None, None
            except OSError:
                time.sleep(0.2)
                continue
            now = time.monotonic()
            with self._lock:
                if reply is not None and host == self._host and self._sent:
                    self._sent.pop(0)
                    if reply.startswith("OK") and not self._confirmed:
                        self._confirmed = True
                        self._log.info(f"NSP sync: the cabinet confirms pulses ({reply}).")
                    elif not reply.startswith("OK"):
                        self._log.error(f"NSP sync: the cabinet refused a pulse: {reply}")
                late = [c for t, c in self._sent if now - t > self._ANSWER_SEC]
                self._sent = [(t, c) for t, c in self._sent if now - t <= self._ANSWER_SEC]
                warn = bool(late) and not self._silent_warned
                if warn:
                    self._silent_warned = True
                host_now = self._host
            if warn:
                self._log.warn(
                    f"NSP sync: no answer from {host_now}:{PULSE_PORT} to '{late[0]}'. Is the experiment app "
                    f"running on the cabinet, and is nsp_sync.cabinet_ip its address on this network? "
                    f"(Warned once per address.)")
