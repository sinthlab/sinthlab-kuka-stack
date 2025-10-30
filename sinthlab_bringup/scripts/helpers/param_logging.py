#!/usr/bin/env python3
from __future__ import annotations
from typing import Any, Dict, Optional

from rclpy.node import Node

# Track contexts already logged to avoid duplicate parameter dumps
_LOGGED_CONTEXTS = set()

def log_params_once(node: Node, params: Dict[str, Any], context: Optional[str] = None, title: Optional[str] = None) -> None:
    """
    Log a one-time, nicely formatted dump of provided parameters.

    - node: rclpy Node for logger access
    - params: dict of parameter names -> values to print
    - context: optional key to ensure we only log once per logical context (defaults to node name)
    - title: optional custom header; defaults to f"Parameters for <context>"
    """
    try:
        key = context or node.get_name()
        if key in _LOGGED_CONTEXTS:
            return
        _LOGGED_CONTEXTS.add(key)

        header = title or f"Parameters for '{key}'"
        # Sort keys for stable output
        lines = [f"  - {k}: {params[k]}" for k in sorted(params.keys())]
        msg = header + "\n" + "\n".join(lines)
        node.get_logger().info(msg)
    except Exception:
        # Never let logging errors break the node
        pass
