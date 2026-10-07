"""ADRC velocity-to-acceleration loop for Module 04."""

from dataclasses import dataclass

from adrc.controller import LinearADRC


@dataclass(frozen=True)
class Config:
    """Velocity ADRC bandwidths and per-axis acceleration limits."""

    b0: tuple[float, float, float]
    control_bandwidth: tuple[float, float, float]
    observer_bandwidth: tuple[float, float, float]
    acceleration_limits_mps2: tuple[float, float, float]


class Controller:
    """Convert desired world velocity into desired world acceleration."""

    def __init__(self, config: Config) -> None:
        """Create one ADRC observer loop for each world velocity axis."""
        self.loops = tuple(LinearADRC.from_bandwidth(b0, control, observer, limit) for b0, control, observer, limit in zip(config.b0, config.control_bandwidth, config.observer_bandwidth, config.acceleration_limits_mps2))

    def reset(self) -> None:
        """Clear velocity observer states."""
        for loop in self.loops:
            loop.reset()

    def update(self, target_velocity_mps: tuple[float, float, float], velocity_mps: tuple[float, float, float], dt: float) -> tuple[float, float, float]:
        """Return bounded desired world acceleration from velocity error."""
        return tuple(loop.update(target, measured, dt) for loop, target, measured in zip(self.loops, target_velocity_mps, velocity_mps))
