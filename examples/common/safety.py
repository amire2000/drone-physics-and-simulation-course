"""Shared safety limits for interactive flight-learning simulations."""

from dataclasses import dataclass
from math import degrees, radians


@dataclass(frozen=True)
class FlightSafetyLimits:
    """Conservative limits that pause an interactive simulation before a crash."""

    minimum_altitude_m: float = 0.15
    maximum_tilt_rad: float = radians(45.0)
    maximum_body_rate_rad_s: float = 8.0


def safety_reason(
    altitude_m: float,
    roll_rad: float,
    pitch_rad: float,
    roll_rate_rad_s: float,
    pitch_rate_rad_s: float,
    limits: FlightSafetyLimits = FlightSafetyLimits(),
) -> str | None:
    """Return a readable pause reason, or ``None`` when the state is safe."""
    if altitude_m < limits.minimum_altitude_m:
        return f"Safety pause: altitude below {limits.minimum_altitude_m:.2f} m"
    if abs(roll_rad) > limits.maximum_tilt_rad:
        return f"Safety pause: roll exceeded {degrees(limits.maximum_tilt_rad):.0f} degrees"
    if abs(pitch_rad) > limits.maximum_tilt_rad:
        return f"Safety pause: pitch exceeded {degrees(limits.maximum_tilt_rad):.0f} degrees"
    if abs(roll_rate_rad_s) > limits.maximum_body_rate_rad_s or abs(pitch_rate_rad_s) > limits.maximum_body_rate_rad_s:
        return f"Safety pause: body rate exceeded {limits.maximum_body_rate_rad_s:.1f} rad/s"
    return None
