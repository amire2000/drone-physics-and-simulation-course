"""Diagonal terminal-speed trajectory generation."""

from dataclasses import dataclass
from math import cos, sin, sqrt

from .config import StrikeConfig


@dataclass(frozen=True)
class TrajectoryCommand:
    forward_velocity_mps: float
    vertical_velocity_mps: float
    altitude_target_m: float


class DiagonalTrajectory:
    """Generate the configured diagonal path from visual remaining range."""

    def __init__(self, config: StrikeConfig) -> None:
        self.config = config
        self.initial_range_m: float | None = None

    def command(self, remaining_range_m: float) -> TrajectoryCommand:
        self.initial_range_m = self.initial_range_m or max(remaining_range_m, 1e-6)
        remaining_fraction = max(0.0, min(1.0, remaining_range_m / self.initial_range_m))
        traveled_path_m = self.config.path_length_m * (1 - remaining_fraction)
        speed_mps = min(self.config.terminal_speed_mps, max(1.0, sqrt(2 * self.config.path_acceleration_mps2 * traveled_path_m)))
        return TrajectoryCommand(
            speed_mps * cos(self.config.descent_angle_rad),
            -speed_mps * sin(self.config.descent_angle_rad),
            self.config.target_center[2] + (self.config.takeoff_altitude_m - self.config.target_center[2]) * remaining_fraction,
        )
