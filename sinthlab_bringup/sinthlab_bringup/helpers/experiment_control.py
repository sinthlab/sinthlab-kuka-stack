#!/usr/bin/env python3
"""What every orchestrator exposes for being driven from outside -- the dashboard or the CLI.

    <ns>/experiment_status       std_msgs/String, JSON, latched   where the experiment is right now
    <ns>/<orchestrator>/pause    std_srvs/SetBool                 hold at the start between trials
    ros2 param set <ns>/<orchestrator> <name> <value>             live parameters only

LIVE PARAMETERS apply at the next trial boundary, never mid-trial, so each trial runs under one
configuration and its sidecar records it. Which ones are live is decided in helpers/live_params.py;
a set on anything else is REJECTED with a reason, because the orchestrator read it once at start-up
and the change would otherwise look accepted while doing nothing.

PAUSE takes effect when the current trial ends: the arm finishes recovering to the start pose and
holds there. Resume starts the next trial (and applies any live changes made meanwhile). Pause is
not a stop and not a safety function -- the arm stays under control and compliant throughout.

NSP SYNC. `on_event` is what TrialRecorder calls synchronously inside mark(); with
`nsp_sync.enabled` it sends that event's code to the Blackrock NSP. The DIO device is not wired
yet, so today it warns once and sends nothing -- implement `_pulse()` when it arrives.
"""
from __future__ import annotations

import json
import time
from typing import Callable, Dict, List, Optional

from rclpy.node import Node as rclpyNode
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from std_srvs.srv import SetBool

from sinthlab_bringup.helpers.common_threshold import get_optional_param
from sinthlab_bringup.helpers.live_params import LIVE_PARAMS, check_value, is_live

# Event token -> code sent to the NSP. Two per trial are enough for alignment (trial_start and
# trial_end give offset and rate); the rest make a dropped pulse visible instead of silently
# mispairing every later trial. See README.md section 7, Data Collected.
NSP_CODES: Dict[str, int] = {
    "trial_start": 1, "at_start": 2, "armed": 3, "snap": 4,
    "checkpoint": 5, "goal": 6, "timeout": 7, "safety_trip": 8, "trial_end": 9,
}

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
        # The values the CURRENT trial runs with. rclpy stores an accepted set immediately, but the
        # actions only re-read at the next trial boundary, so the parameter store runs ahead of
        # what is in effect; status reports this snapshot, and `pending` separately.
        self._effective: Dict[str, object] = {
            name: _plain(node.get_parameter(name).value)
            for name in node._parameters if is_live(experiment, name)}   # noqa: SLF001
        self._nsp_enabled = bool(get_optional_param(node, "nsp_sync.enabled", False))
        self._nsp_warned = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._pub = node.create_publisher(String, STATUS_TOPIC, latched)
        node.create_service(SetBool, "~/pause", self._on_pause)
        node.add_on_set_parameters_callback(self._gate)
        node.create_timer(_HEARTBEAT_SEC, self._publish)
        self._publish()

    # ------------------------------------------------------------------ orchestrator API

    def on_reload(self, fn: Callable[[], None]) -> None:
        """Register a function that re-reads live parameters. Called at a trial boundary."""
        self._reloaders.append(fn)

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
        """Every trial event: NSP code first (timing matters), then status."""
        if self._nsp_enabled and token in NSP_CODES:
            self._pulse(NSP_CODES[token], token)
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
        }
        self._pub.publish(String(data=json.dumps(status, default=str)))

    def _apply_pending(self) -> None:
        if not self._pending:
            return
        for fn in self._reloaders:
            fn()
        self._nsp_enabled = bool(get_optional_param(self._node, "nsp_sync.enabled", False))
        self._effective.update(self._pending)
        changes = ", ".join(f"{k}={v}" for k, v in self._pending.items())
        self._log.info(f"Live parameters applied from trial {self._trial + 1}: {changes}")
        self._pending = {}

    def _on_pause(self, request, response):
        self._paused = bool(request.data)
        if not self._paused and self._held is not None:
            start_fn, self._held = self._held, None
            self._log.info("Resumed.")
            self._apply_pending()
            start_fn()
        response.success = True
        response.message = ("pause requested: holds at the start once this trial ends"
                            if self._paused else "running")
        self._publish()
        return response

    def _gate(self, params) -> SetParametersResult:
        for p in params:
            if not self._node.has_parameter(p.name):
                continue            # a first declaration, not a change
            if not is_live(self._experiment, p.name):
                return SetParametersResult(
                    successful=False,
                    reason=(f"'{p.name}' is a per-run parameter: it was read once at start-up, "
                            f"so changing it now would do nothing. Stop, edit it, and start again. "
                            f"Live parameters: {', '.join(LIVE_PARAMS[self._experiment])}"))
            problem = check_value(p.name, _plain(p.value))
            if problem:
                return SetParametersResult(successful=False, reason=problem)
        for p in params:
            if self._node.has_parameter(p.name):
                self._pending[p.name] = _plain(p.value)
                self._log.info(f"Live change accepted: {p.name} = {_plain(p.value)} "
                               f"(applies from trial {self._trial + 1})")
        self._publish()
        return SetParametersResult(successful=True)

    def _pulse(self, code: int, token: str) -> None:
        """Send `code` to the NSP digital input.   >>> IMPLEMENT WHEN THE DIO ARRIVES <<<

        Must never raise and never block: it runs inside TrialRecorder.mark()."""
        if not self._nsp_warned:
            self._nsp_warned = True
            self._log.warn(
                f"nsp_sync.enabled is true but no DIO device is wired yet: NSP code {code} "
                f"('{token}') and every later code will NOT be sent. See README.md section 7.")
