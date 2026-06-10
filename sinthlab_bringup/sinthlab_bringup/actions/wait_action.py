#!/usr/bin/env python3
"""Fixed-duration delay action.

A tiny sequencing primitive with the same ``start()`` / ``on_complete`` shape as the other actions:
on ``start()`` it waits ``duration_sec`` (via a one-shot timer) and then calls ``on_complete()``.
The orchestrators use it for quiet windows and inter-step delays so the trial sequence reads as a
uniform chain of actions instead of raw ``create_timer`` calls.
"""
from __future__ import annotations

from typing import Callable

from rclpy.node import Node as rclpyNode


class WaitAction:
    def __init__(self, node: rclpyNode, *, duration_sec: float,
                 on_complete: Callable[[], None], name: str = "wait") -> None:
        self._node = node
        self._duration_sec = float(duration_sec)
        self._on_complete = on_complete
        self._name = name
        self._timer = None

    def start(self) -> None:
        if self._timer is not None:
            self._node.get_logger().warn(f"WaitAction '{self._name}' is already running.")
            return
        self._node.get_logger().info(f"WaitAction '{self._name}': waiting {self._duration_sec:.2f}s...")
        # rclpy timers are periodic; cancel on the first fire to make this one-shot.
        self._timer = self._node.create_timer(self._duration_sec, self._fire)

    def stop(self) -> None:
        """Cancel a pending wait without firing on_complete (e.g. a timeout the goal beat)."""
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def _fire(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        try:
            self._on_complete()
        except Exception as exc:
            self._node.get_logger().error(f"WaitAction '{self._name}' on_complete error: {exc}")
