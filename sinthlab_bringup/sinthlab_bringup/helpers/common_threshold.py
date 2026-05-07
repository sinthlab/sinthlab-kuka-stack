#!/usr/bin/env python3
from __future__ import annotations
from typing import Optional

from rclpy.node import Node as rclpyNode


class DebugTicker:
    """
    Utility to trigger a periodic event at a low rate based on a fixed time step (dt).
    Call tick(dt) each loop; returns True when it's time to log.
    """

    def __init__(self, rate_hz: float) -> None:
        self._rate_hz = float(rate_hz)
        self._accum = 0.0

    def set_rate(self, rate_hz: float) -> None:
        self._rate_hz = float(rate_hz)

    def tick(self, dt: float) -> bool:
        if self._rate_hz <= 1e-6:
            return False
        period = 1.0 / self._rate_hz
        self._accum += float(dt)
        if self._accum >= period:
            self._accum = 0.0
            return True
        return False


def get_required_param(node: rclpyNode, name: str):
    """
    Fetch a required parameter.

    Raises a ValueError if the parameter is missing or unset. Intended for use when
    all parameters must come from YAML or runtime overrides.
    """
    if not node.has_parameter(name):
        raise ValueError(f"Required parameter '{name}' is missing; ensure it's set in YAML or as an override.")
    p = node.get_parameter(name)
    if p is None or p.value is None:
        raise ValueError(f"Required parameter '{name}' is unset or None; provide a concrete value.")
    return p.value