from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class FusionInputs:
    sensor_ok: bool
    front_mm: float
    red_stop: bool
    obstacle: bool
    path_ready: bool
    goal_reached: bool


def decide_fusion(inputs: FusionInputs) -> str:
    if not inputs.sensor_ok:
        return "SENSOR_FAIL"
    if inputs.front_mm < 140.0:
        return "EMERGENCY_STOP"
    if inputs.goal_reached:
        return "GOAL"
    if inputs.red_stop:
        return "RED_STOP"
    if inputs.obstacle:
        return "AVOID"
    if not inputs.path_ready:
        return "NO_PATH"
    return "FOLLOW_PATH"


def default_fusion_command(state: str) -> tuple[float, float]:
    if state in {"SENSOR_FAIL", "EMERGENCY_STOP", "GOAL", "RED_STOP", "NO_PATH"}:
        return 0.0, 0.0
    if state == "AVOID":
        return -11.0, 11.0
    return 14.0, 14.0
