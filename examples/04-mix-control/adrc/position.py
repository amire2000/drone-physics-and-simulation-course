"""ADRC position-to-velocity loop for Module 04."""

from dataclasses import dataclass

from adrc.controller import LinearADRC


@dataclass(frozen=True)
class Config:
    """Position ADRC bandwidths and per-axis velocity limits."""

    b0: tuple[float, float, float]
    control_bandwidth: tuple[float, float, float]
    observer_bandwidth: tuple[float, float, float]
    velocity_limits_mps: tuple[float, float, float]
    damping: float


class Controller:
    """Convert home-relative position error into desired world velocity."""

    def __init__(self, config: Config) -> None:
        """Create one ADRC observer loop for each world position axis."""
        self.damping = config.damping
        self.loops = tuple(LinearADRC.from_bandwidth(b0, control, observer, limit) for b0, control, observer, limit in zip(config.b0, config.control_bandwidth, config.observer_bandwidth, config.velocity_limits_mps))

    def reset(self) -> None:
        """Clear position observer states."""
        for loop in self.loops:
            loop.reset()

    def update(self, target_position_m: tuple[float, float, float], position_m: tuple[float, float, float], velocity_mps: tuple[float, float, float], dt: float) -> tuple[float, float, float]:
        """Return bounded desired world velocity from position error."""
        return tuple(loop.update(target, measured, dt, rate, self.damping) for loop, target, measured, rate in zip(self.loops, target_position_m, position_m, velocity_mps))
