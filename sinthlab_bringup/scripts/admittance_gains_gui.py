#!/usr/bin/env python3
"""Simple Qt tool for tuning admittance controller parameters at runtime."""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParameters

from python_qt_binding import QtCore, QtWidgets

_PARAM_NAMES = ["dq_gains", "dx_gains", "exp_smooth"]


def _wait_for_service(node: RclpyNode, client, service_name: str, timeout: float = 1.0) -> None:
    while not client.wait_for_service(timeout_sec=timeout):
        node.get_logger().info(f"Waiting for service {service_name} ...")


def _spin_until_complete(node: RclpyNode, future, timeout: float = 0.1) -> bool:
    start = time.time()
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=timeout)
        if timeout == 0.0:
            break
        if time.time() - start > 10.0:
            break
    return future.done()


class SliderRow(QtWidgets.QWidget):
    valueChanged = QtCore.Signal(float)

    def __init__(self, title: str, minimum: int, maximum: int, scale: float, decimals: int = 2, parent=None) -> None:
        super().__init__(parent)
        self._scale = scale
        self._decimals = decimals

        self._label = QtWidgets.QLabel(title)
        self._current_value = QtWidgets.QLabel("0.00")
        self._slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self._slider.setRange(minimum, maximum)
        self._slider.valueChanged.connect(self._on_slider_value_changed)

        font = self._label.font()
        font.setPointSize(max(font.pointSize() + 2, 11))
        self._label.setFont(font)
        self._current_value.setFont(font)

        layout = QtWidgets.QHBoxLayout(self)
        layout.addWidget(self._label)
        layout.addWidget(self._slider, stretch=1)
        layout.addWidget(self._current_value)
        layout.setContentsMargins(0, 0, 0, 0)

    def set_value(self, value: float) -> None:
        slider_value = int(round(value / self._scale))
        block = QtCore.QSignalBlocker(self._slider)
        self._slider.setValue(slider_value)
        del block
        self._current_value.setText(f"{value:.{self._decimals}f}")

    def _on_slider_value_changed(self, slider_value: int) -> None:
        value = slider_value * self._scale
        self._current_value.setText(f"{value:.{self._decimals}f}")
        self.valueChanged.emit(value)


def _normalize_fqn(node_name: str) -> str:
    stripped = node_name.strip()
    if not stripped:
        raise ValueError("Target node name cannot be empty")
    return "/" + stripped.lstrip("/")


class AdmittanceGainsWindow(QtWidgets.QWidget):
    def __init__(self, node: RclpyNode, target_node: str) -> None:
        super().__init__()
        self._node = node
        self._target_node = _normalize_fqn(target_node)
        self.setWindowTitle(f"Admittance Gains Controller ({self._target_node})")

        self._set_client = self._node.create_client(SetParameters, f"{self._target_node}/set_parameters")
        self._get_client = self._node.create_client(GetParameters, f"{self._target_node}/get_parameters")
        _wait_for_service(self._node, self._set_client, f"{self._target_node}/set_parameters")
        _wait_for_service(self._node, self._get_client, f"{self._target_node}/get_parameters")

        self._pending_futures = []
        self._base_dq = [1.0] * 7
        self._base_dx = [0.1] * 6
        self._init_parameters()
        self._dq_baseline = self._compute_baseline(self._base_dq, default=1.0)
        self._dx_baseline = self._compute_baseline(self._base_dx, default=0.1)

        self._status_label = QtWidgets.QLabel("Ready")

        self._dq_slider = SliderRow("dq_gains scale", 1, 30, 0.1)
        self._dx_slider = SliderRow("dx_gains scale", 1, 40, 0.01)
        self._exp_slider = SliderRow("exp_smooth", 50, 99, 0.01)

        self._dq_slider.valueChanged.connect(self._handle_dq_scale)
        self._dx_slider.valueChanged.connect(self._handle_dx_scale)
        self._exp_slider.valueChanged.connect(self._handle_exp)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self._dq_slider)
        layout.addWidget(self._dx_slider)
        layout.addWidget(self._exp_slider)
        layout.addWidget(self._status_label)

        self.resize(480, 240)

        self._dq_slider.set_value(1.0)
        self._dx_slider.set_value(0.1)
        self._exp_slider.set_value(float(self._current_params.get("exp_smooth", 0.0)))

        self._spin_timer = QtCore.QTimer(self)
        self._spin_timer.timeout.connect(self._spin_ros)
        self._spin_timer.start(50)

    def _init_parameters(self) -> None:
        request = GetParameters.Request()
        request.names = _PARAM_NAMES
        future = self._get_client.call_async(request)
        if not _spin_until_complete(self._node, future):
            raise RuntimeError(f"Failed to retrieve initial parameters from {self._target_node}")
        response = future.result()
        values = {}
        for name, value in zip(_PARAM_NAMES, response.values):
            if hasattr(value, "double_array_value") and value.double_array_value:
                values[name] = list(value.double_array_value)
            elif hasattr(value, "double_value"):
                values[name] = value.double_value
            else:
                self._node.get_logger().warn(f"Unhandled parameter type for {name}")
        self._current_params = values
        if "dq_gains" in values:
            self._base_dq = [float(v) for v in values["dq_gains"]]
        if "dx_gains" in values:
            self._base_dx = [float(v) for v in values["dx_gains"]]

    @staticmethod
    def _compute_baseline(values, default: float) -> float:
        if not values:
            return default
        magnitude = sum(abs(v) for v in values) / len(values)
        return magnitude if magnitude > 0.0 else default

    def _spin_ros(self) -> None:
        if rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.0)
        remaining = []
        for name, future in self._pending_futures:
            if future.done():
                try:
                    result = future.result()
                    success = all(r.successful for r in result.results)
                    if success:
                        self._status_label.setText(f"Updated {name}")
                    else:
                        reasons = ", ".join(r.reason for r in result.results if r.reason)
                        self._status_label.setText(f"Failed {name}: {reasons}")
                except Exception as exc:  # pylint: disable=broad-except
                    self._status_label.setText(f"Error updating {name}: {exc}")
            else:
                remaining.append((name, future))
        self._pending_futures = remaining

    def _handle_dq_scale(self, scale_value: float) -> None:
        scale_factor = scale_value / self._dq_baseline if self._dq_baseline else scale_value
        new_values = [base * scale_factor for base in self._base_dq]
        self._send_parameter_update(
            "dq_gains",
            Parameter(name="dq_gains", value=new_values),
        )

    def _handle_dx_scale(self, scale_value: float) -> None:
        if self._dx_baseline:
            scale_factor = scale_value / self._dx_baseline
            new_values = [base * scale_factor for base in self._base_dx]
        else:
            new_values = [scale_value] * len(self._base_dx)
        self._send_parameter_update(
            "dx_gains",
            Parameter(name="dx_gains", value=new_values),
        )

    def _handle_exp(self, value: float) -> None:
        self._send_parameter_update(
            "exp_smooth",
            Parameter(name="exp_smooth", value=float(value)),
        )

    def _send_parameter_update(self, name: str, parameter: Parameter) -> None:
        request = SetParameters.Request()
        request.parameters = [parameter.to_parameter_msg()]
        future = self._set_client.call_async(request)
        self._pending_futures.append((name, future))

    def closeEvent(self, event):
        self._spin_timer.stop()
        super().closeEvent(event)


def _resolve_target_node(args: argparse.Namespace) -> str:
    if args.target_node:
        return args.target_node
    namespace = args.namespace.strip("/")
    node_name = args.node_name.strip("/")
    if namespace:
        return f"/{namespace}/{node_name}"
    return f"/{node_name}"


def main() -> None:
    parser = argparse.ArgumentParser(description="Qt GUI for admittance gain tuning")
    parser.add_argument("--namespace", default="lbr", help="Namespace containing the admittance controller node")
    parser.add_argument("--node-name", default="admittance_controller", help="Target node name")
    parser.add_argument("--target-node", default="", help="Fully qualified node name override")
    args, remaining = parser.parse_known_args()
    sys.argv = [sys.argv[0]] + remaining

    target_node = _resolve_target_node(args)

    rclpy.init(args=remaining)
    node = rclpy.create_node("admittance_gains_gui")
    try:
        app = QtWidgets.QApplication(sys.argv)
        window = AdmittanceGainsWindow(node, target_node=target_node)
        window.show()
        exit_code = app.exec()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
