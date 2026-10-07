"""Small shared control types for interchangeable Module 04 controllers."""

from typing import Protocol


def clamp(value: float, low: float, high: float) -> float:
    """Limit a scalar to an inclusive range."""
    return max(low, min(high, value))


class Controller(Protocol):
    """Interface shared by the PID and ADRC flight controllers."""

    def reset(self) -> None:
        """Clear controller state before a new flight."""
        ...

    def update(
        self,
        position_z_m: float,
        velocity_z_mps: float,
        attitude_rad: tuple[float, float, float],
        body_rates_rad_s: tuple[float, float, float],
        target_altitude_m: float,
        dt: float,
        mass_kg: float,
        gravity_mps2: float,
    ) -> tuple[float, tuple[float, float, float], tuple[float, float, float]]:
        """Return collective thrust, body torque, and desired body rates."""
        ...
