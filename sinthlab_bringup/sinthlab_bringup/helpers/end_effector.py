"""The end effector's force sensor (FSR) and haptic motor, over the shared serial link.

    link(node)            the serial link (actions/visual_cue_serial.SerialCueTrigger.shared) on
                          visual_cue.serial_port -- the same one the visual cue uses
    start_sensor(node)    stream the FSR at end_effector.fsr_rate_hz (0 = off)
    record_columns(node)  (header, fn) the trial recorder appends to every row, or None

Every experiment records the FSR when end_effector.fsr_rate_hz > 0, whichever path its visual cue
uses: the sensor data only travels on the serial link. The trial file gets, per row (one row per
lbr_state, ~100 Hz):

    fsr_raw        the latest sample, 0-65535 (AnalogIn.value; the FSR divider, README)
    fsr_t_ms       the board's clock at that sample [ms] -- a repeated value is the same sample
    fsr_age_ms     how long before this row the sample arrived here [ms]
    haptic_pct     the haptic strength driven AT that sample [%] -- to mask motor leakage offline

Blank (NaN) until the first sample, and when the stream stops (fsr_age_ms grows).
"""
from __future__ import annotations

import math
import time
from typing import Callable, List, Optional, Tuple

from sinthlab_bringup.helpers.node_utils import get_required_param

RECORD_HEADER = ["fsr_raw", "fsr_t_ms", "fsr_age_ms", "haptic_pct"]


def link(node):
    """The shared serial link to the board, on visual_cue.serial_port."""
    from sinthlab_bringup.actions.visual_cue_serial import SerialCueTrigger
    return SerialCueTrigger.shared(node, str(get_required_param(node, "visual_cue.serial_port")))


def fsr_rate(node) -> int:
    return int(get_required_param(node, "end_effector.fsr_rate_hz")) if node.has_parameter(
        "end_effector.fsr_rate_hz") else 0


def start_sensor(node) -> Optional[object]:
    """Start the FSR stream if end_effector.fsr_rate_hz > 0; the link, or None when off."""
    rate = fsr_rate(node)
    if rate <= 0:
        return None
    lk = link(node)
    lk.set_stream(rate)
    node.get_logger().info(f"Force sensor: recording at {rate} samples/s into every trial file.")
    return lk


def record_columns(node) -> Optional[Tuple[List[str], Callable[[], list]]]:
    """The trial recorder's extra columns for the FSR, or None when it is off."""
    lk = start_sensor(node)
    if lk is None:
        return None

    def row() -> list:
        s = lk.latest()
        if s is None:
            return [math.nan] * 4
        t_ms, raw, hap, rx = s
        return [raw, t_ms, round((time.monotonic() - rx) * 1000.0, 1), hap]

    return list(RECORD_HEADER), row
