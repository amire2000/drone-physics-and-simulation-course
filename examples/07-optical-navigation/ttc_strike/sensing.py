"""Simple altitude sensor model used by TTC guidance."""

from dataclasses import dataclass

import numpy as np

from .config import StrikeConfig


@dataclass(frozen=True)
class BarometerReading:
    altitude_m: float
    vertical_velocity_mps: float


class Barometer:
    """Sample altitude at camera rate with optional deterministic noise and bias."""

    def __init__(self, config: StrikeConfig) -> None:
        self.config = config
        self.period_s = 1 / config.camera_hz
        self.rng = np.random.default_rng(config.random_seed)
        self.last_time_s: float | None = None
        self.last_altitude_m: float | None = None
        self.vertical_velocity_mps = 0.0

    def sample(self, true_altitude_m: float, now_s: float) -> BarometerReading | None:
        if self.last_time_s is not None and now_s - self.last_time_s < self.period_s:
            return None
        altitude_m = true_altitude_m + self.config.barometer_bias_m + self.rng.normal(0.0, self.config.barometer_noise_sigma_m)
        if self.last_time_s is not None and self.last_altitude_m is not None:
            measured_velocity = (altitude_m - self.last_altitude_m) / (now_s - self.last_time_s)
            old = self.config.barometer_velocity_old_weight
            self.vertical_velocity_mps = old * self.vertical_velocity_mps + (1 - old) * measured_velocity
        self.last_time_s, self.last_altitude_m = now_s, altitude_m
        return BarometerReading(altitude_m, self.vertical_velocity_mps)
