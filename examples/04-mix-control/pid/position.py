"""PID position-to-velocity loop for Module 04."""

from dataclasses import dataclass

from common.control import clamp
from pid.controller import PID


@dataclass(frozen=True)
class Config:
    """Position PID gains and per-axis velocity limits."""

    gains: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    integral_limit: float
    velocity_limits_mps: tuple[float, float, float]


class Controller:
    """Convert home-relative position error into desired world velocity."""

    def __init__(self, config: Config) -> None:
        """Create one PID loop for each world position axis."""
        self.loops = tuple(PID(*gains, config.integral_limit, limit) for gains, limit in zip(config.gains, config.velocity_limits_mps))

    def reset(self) -> None:
        """Clear position-loop integrators."""
        for loop in self.loops:
            loop.reset()

    def update(self, target_position_m: tuple[float, float, float], position_m: tuple[float, float, float], velocity_mps: tuple[float, float, float], dt: float) -> tuple[float, float, float]:
        """Return bounded desired world velocity from position error."""
        return tuple(loop.update(target - measured, -rate, dt) for loop, target, measured, rate in zip(self.loops, target_position_m, position_m, velocity_mps))
