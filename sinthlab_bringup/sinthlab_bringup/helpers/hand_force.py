#!/usr/bin/env python3
"""The force a hand applies to the end effector, from the arm's joint torque sensors -- no force sensor.

    F = pinv(J)^T tau_ext        (force part of the wrench, base frame)

tau_ext is the cabinet's external-torque estimate (LBRState.external_torque: measured torque minus
the gravity / tool-load model), J the base-frame geometric Jacobian of the end effector. The
pseudo-inverse is damped (jacobian_damping) so the estimate stays bounded near singular postures.
What remains at rest is the tool-load model's error (a few N); callers subtract a tare for that.

Used by Free Move (AdmittanceMoveAction) and the grasp detector of the perturb_bump paradigm.
"""
from __future__ import annotations

import numpy as np
import optas


class HandForce:
    def __init__(self, robot_description: str, ee_link: str, jacobian_damping: float) -> None:
        model = optas.RobotModel(urdf_string=robot_description)
        # Base-frame geometric Jacobian of the EE: rows [linear; angular] (checked against FK).
        self._jac = model.get_global_link_geometric_jacobian_function(link=ee_link, numpy_output=True)
        self._lam2 = max(0.0, float(jacobian_damping)) ** 2

    def force(self, tau_ext, q) -> np.ndarray:
        """Hand force [N], base frame; zeros if the torque reading is not finite."""
        tau = np.asarray(tau_ext, dtype=float)
        if not np.all(np.isfinite(tau)):
            return np.zeros(3)
        U, sv, Vt = np.linalg.svd(np.array(self._jac(q), dtype=float), full_matrices=False)
        J_pinv = Vt.T @ np.diag(sv / (sv * sv + self._lam2)) @ U.T     # damped pseudo-inverse, 7x6
        return (J_pinv.T @ tau)[0:3]
