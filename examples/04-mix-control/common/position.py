"""Position-control math and PyBullet target sliders for Module 04."""

from dataclasses import dataclass
from math import asin, cos, sin

import pybullet as p

from common.control import clamp


@dataclass(frozen=True)
class PositionLimits:
    """Home-relative target, velocity, and acceleration limits in SI units."""

    target_m: tuple[tuple[float, float], tuple[float, float], tuple[float, float]]
    velocity_mps: tuple[float, float, float]
    acceleration_mps2: tuple[float, float, float]


class TargetSliders:
    """Read live home-relative X/Y/Z targets from PyBullet debug sliders."""

    def __init__(self, limits: PositionLimits, initial_target_m: tuple[float, float, float]) -> None:
        """Create one native slider for each target-position axis."""
        labels = ("Target X from home (m)", "Target Y from home (m)", "Target Z from home (m)")
        self._ids = tuple(p.addUserDebugParameter(label, bounds[0], bounds[1], initial) for label, bounds, initial in zip(labels, limits.target_m, initial_target_m))

    def read(self) -> tuple[float, float, float]:
        """Return the current XYZ offset target from home."""
        return tuple(float(p.readUserDebugParameter(parameter_id)) for parameter_id in self._ids)


def acceleration_to_attitude_thrust(acceleration_world_mps2: tuple[float, float, float], yaw_rad: float, mass_kg: float, gravity_mps2: float, max_total_thrust_n: float, max_tilt_rad: float) -> tuple[float, tuple[float, float, float]]:
    """Convert world acceleration into level-yaw roll/pitch and collective thrust."""
    ax, ay, az = acceleration_world_mps2
    cos_yaw, sin_yaw = cos(yaw_rad), sin(yaw_rad)
    body_ax = cos_yaw * ax + sin_yaw * ay
    body_ay = -sin_yaw * ax + cos_yaw * ay
    pitch = asin(clamp(body_ax / gravity_mps2, -sin(max_tilt_rad), sin(max_tilt_rad)))
    roll = asin(clamp(-body_ay / gravity_mps2, -sin(max_tilt_rad), sin(max_tilt_rad)))
    thrust = mass_kg * (gravity_mps2 + az) / max(cos(roll) * cos(pitch), 1e-6)
    return clamp(thrust, 0.0, max_total_thrust_n), (roll, pitch, 0.0)
