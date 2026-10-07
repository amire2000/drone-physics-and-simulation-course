"""PID velocity-to-acceleration loop for Module 04."""

from dataclasses import dataclass

from pid.controller import PID


@dataclass(frozen=True)
class Config:
    """Velocity PID gains and per-axis acceleration limits."""

    gains: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    integral_limit: float
    acceleration_limits_mps2: tuple[float, float, float]


class Controller:
    """Convert desired world velocity into desired world acceleration."""

    def __init__(self, config: Config) -> None:
        """Create one PID loop for each world velocity axis."""
        self.loops = tuple(PID(*gains, config.integral_limit, limit) for gains, limit in zip(config.gains, config.acceleration_limits_mps2))

    def reset(self) -> None:
        """Clear velocity-loop integrators."""
        for loop in self.loops:
            loop.reset()

    def update(self, target_velocity_mps: tuple[float, float, float], velocity_mps: tuple[float, float, float], dt: float) -> tuple[float, float, float]:
        """Return bounded desired world acceleration from velocity error."""
        return tuple(loop.update(target - measured, 0.0, dt) for loop, target, measured in zip(self.loops, target_velocity_mps, velocity_mps))
