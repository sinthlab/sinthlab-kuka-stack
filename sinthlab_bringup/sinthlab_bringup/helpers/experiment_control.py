#!/usr/bin/env python3
"""What every orchestrator exposes for being driven from outside -- the dashboard or the CLI.

    <ns>/experiment_status       std_msgs/String, JSON, latched   where the experiment is right now
    <ns>/<orchestrator>/pause    std_srvs/SetBool                 hold at park between trials
    <ns>/<orchestrator>/stop     std_srvs/Trigger                 end the trial NOW, park, and hold
    ros2 param set <ns>/<orchestrator> <name> <value>             live parameters only

LIVE PARAMETERS apply at the next trial boundary, never mid-trial, so each trial runs under one
configuration and its sidecar records it. The exception is IMMEDIATE parameters (live_params.py: the
move speed), applied the moment they are set (from the next move) and logged in the sidecar with when. Which ones are
live is decided in helpers/live_params.py;
a set on anything else is REJECTED with a reason, because the orchestrator read it once at start-up
and the change would otherwise look accepted while doing nothing.

PAUSE takes effect when the current trial ends: the arm finishes moving to the park pose (out of
reach) and holds there. A trial with no natural end (free move) registers on_pause_request() and ends its trial
when the pause arrives -- that is also how the dashboard's "Stop after trial" reaches it. Resume starts the next trial (and applies any live changes made meanwhile). Pause is
not a stop and not a safety function -- the arm stays under control and compliant throughout.

STOP (the dashboard's Stop now) ends the trial in progress at once -- the orchestrator's on_stop_request
hook stops whatever is running and parks the arm -- and pauses, so the control then holds at park. Only
then does the dashboard end the launch, so FRI always closes with the arm parked, out of reach.

NSP SYNC. `on_event` is what TrialRecorder calls synchronously inside mark(); with
`nsp_sync.enabled`, trial_start and trial_end each send the cabinet one UDP datagram,
"PULSE <output> <width ms>" to <nsp_sync.cabinet_ip>:30002, and the cabinet's TtlPulser
(sunrise_controller_code/) pulses that EL2124 output -- one BNC line per event to the Blackrock NSP.
The send never blocks or raises; the cabinet's replies are read on a background thread, and a
refusal or silence is logged (see reward_and_sync_box/README.md, "Sync: TTL pulses to the NSP").
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
from std_srvs.srv import SetBool, Trigger

from sinthlab_bringup.helpers.node_utils import get_required_param
from sinthlab_bringup.helpers.live_params import LIVE_PARAMS, check_value, is_immediate, is_live
from sinthlab_bringup.helpers.live_params import PAIRED
from sinthlab_bringup.helpers.smartpad_link import SmartPadLink

# Event token -> the parameter naming its EL2124 output. Two per trial are enough for alignment
# (trial_start and trial_end give offset and rate); every other event is in the CSV on the robot
# clock, and the fitted map carries it along. Each event has its own line, so a dropped pulse shows
# as a missing edge on that line instead of mispairing every later trial.
NSP_OUTPUTS: Dict[str, str] = {
    "trial_start": "nsp_sync.trial_start_output",
    "trial_end": "nsp_sync.trial_end_output",
}
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
        self._paused = False
        self._held: Optional[Callable[[], None]] = None     # the next trial, while paused
        self._pending: Dict[str, object] = {}               # accepted, applies at the next trial
        self._reloaders: List[Callable[[], None]] = []
        self._pause_hooks: List[Callable[[], None]] = []
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
        self._nsp = NspPulser(self._log)
        self._read_nsp()
        self._smartpad = SmartPadLink.get(self._log)    # what the SmartPad app reports it runs with

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._pub = node.create_publisher(String, STATUS_TOPIC, latched)
        node.create_service(SetBool, "~/pause", self._on_pause)
        node.create_service(Trigger, "~/stop", self._on_stop)
        node.add_on_set_parameters_callback(self._gate)
        node.create_timer(_HEARTBEAT_SEC, self._publish)
        self._publish()

    # ------------------------------------------------------------------ orchestrator API

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
        """Register the orchestrator's Stop now: end the trial in progress at once and park."""
        self._stop_hooks.append(fn)

    def on_pause_request(self, fn: Callable[[], None]) -> None:
        """Register a function called when a pause is requested -- for a trial with no natural end, so
        it can end now instead of never reaching the trial boundary where the pause would hold it."""
        self._pause_hooks.append(fn)

    def pause_requested(self) -> bool:
        return self._paused

    def hold(self, why: str) -> None:
        """Pause from the orchestrator itself: the next trial is held at park, as if Pause had been
        pressed, until Resume (or the launch is stopped). For a run that must not continue as set up."""
        if not self._paused:
            self._paused = True
            self._log.error(f"Holding at park: {why}")
            self._publish()

    def smartpad(self) -> SmartPadLink:
        """What the SmartPad app reports it runs with (helpers/smartpad_link.py)."""
        return self._smartpad

    def begin_trial(self, start_fn: Callable[[], None]) -> None:
        """Run the next trial now, or hold it if paused. Orchestrators call this where they used
        to call start_trial() at the end of a trial."""
        if self._paused:
            self._held = start_fn
            self._set_phase("paused")
            self._log.info("Paused: holding at the start pose. Resume to run the next trial.")
            return
        self._apply_pending()
        start_fn()

    def on_event(self, token: str, arg=None) -> None:
        """Every trial event: NSP sync pulse first (timing matters), then status."""
        if self._nsp_enabled and token in NSP_OUTPUTS:
            self._nsp.pulse(self._nsp_cfg[token], self._nsp_cfg["width_ms"], token)
        if token == "trial_start" and arg is not None:
            self._trial = int(arg)
        self._set_phase(token, arg)

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
            "paused": self._paused,
            "held": self._held is not None,
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

    def _on_pause(self, request, response):
        was_paused = self._paused
        self._paused = bool(request.data)
        if self._paused and not was_paused:
            for fn in self._pause_hooks:
                try:
                    fn()
                except Exception as exc:
                    self._log.error(f"pause hook failed: {exc}")
        if not self._paused and self._held is not None:
            start_fn, self._held = self._held, None
            self._log.info("Resumed.")
            self._apply_pending()
            start_fn()
        response.success = True
        response.message = ("pause requested: holds at park once this trial ends"
                            if self._paused else "running")
        self._publish()
        return response

    def _on_stop(self, request, response):
        self._paused = True
        if self._held is None:
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
        self._nsp.set_target(str(p("nsp_sync.cabinet_ip")).strip(), self._nsp_enabled)


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
            self._log.info(f"NSP sync: pulses go to {host}:{PULSE_PORT}.")

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
