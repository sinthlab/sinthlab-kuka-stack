"""Cartesian stiffness profiles broadcast by the Sunrise application."""
from typing import Dict, List

# The SmartPad app's stiffness profiles {X, Y, Z, A, B, C} -- a copy of stiffness_profiles_ /
# stiffness_vals_ in sunrise_controller_code/LbrImpedanceControlServer.java, kept in step with it. Every
# experiment's stiffness_profile must be one of these (checked at import).
SMARTPAD_PROFILES: Dict[str, List[float]] = {
    "Uniform 100": [100.0, 100.0, 100.0, 300.0, 300.0, 300.0],
    "Uniform 400": [400.0, 400.0, 400.0, 300.0, 300.0, 300.0],
    "Uniform 1000": [1000.0, 1000.0, 1000.0, 300.0, 300.0, 300.0],
    "Z lock 2500, X/Y 1000, rot 120": [1000.0, 1000.0, 2500.0, 120.0, 120.0, 120.0],
}


def required_profile(experiment: str) -> str:
    """Required cabinet profile for tasks on the free-move orchestrator."""
    return "Uniform 1000" if experiment == "free_move" else "Z lock 2500, X/Y 1000, rot 120"
