"""Control configuration and simulator-only scene configuration."""

from dataclasses import dataclass
from math import radians


@dataclass(frozen=True)
class SceneConfig:
    """Demo geometry used to spawn and render the simulated target only."""

    target_center: tuple[float, float, float] = (20.0, 0.0, 1.0)
    target_size_m: float = 2.0


@dataclass(frozen=True)
class StrikeConfig:
    """Control, sensor, and output settings with no target geometry."""

    launch_position: tuple[float, float, float] = (-5.25, 0.0, 0.05)
    vehicle_mass_kg: float = 0.65
    gravity_mps2: float = 9.81
    takeoff_altitude_m: float = 15.0
    impact_altitude_m: float = 1.0
    forward_speed_mps: float = 13.0
    nominal_pitch_deg: float = 20.0
    max_descent_velocity_mps: float = 4.5
    max_climb_velocity_mps: float = 3.0
    min_ttc_s: float = 0.2
    camera_width_px: int = 640
    camera_height_px: int = 480
    camera_hz: int = 30
    camera_fov_deg: float = 90.0
    camera_look_down_deg: float = 0.0
    commit_box_height_fraction: float = 0.1
    ttc_growth_old_weight: float = 0.65
    min_growth_px_per_s: float = 0.01
    barometer_noise_sigma_m: float = 0.0
    barometer_bias_m: float = 0.0
    barometer_velocity_old_weight: float = 0.7
    random_seed: int = 7
    altitude_pid_gains: tuple[float, float, float] = (0.7, 0.05, 1.1)
    altitude_integral_limit: float = 0.5
    forward_speed_pid_gains: tuple[float, float, float] = (0.03, 0.0, 0.002)
    forward_pitch_integral_limit: float = 0.2
    max_pitch_deg: float = 20.0
    vertical_velocity_pid_gains: tuple[float, float, float] = (1.0, 0.0, 0.0)
    vertical_position_correction: float = 0.8
    takeoff_altitude_tolerance_m: float = 0.2
    takeoff_velocity_tolerance_mps: float = 0.5
    commit_timeout_margin_s: float = 5.0
    post_impact_seconds: float = 3.0
    environment_size_px: tuple[int, int] = (960, 540)
    opencv_window_position_px: tuple[int, int] = (20, 80)
    plot_window_position_px: tuple[int, int] = (700, 80)

    @property
    def hover_thrust_n(self) -> float:
        return self.vehicle_mass_kg * self.gravity_mps2

    @property
    def nominal_pitch_rad(self) -> float:
        return radians(self.nominal_pitch_deg)

    @property
    def max_pitch_rad(self) -> float:
        return radians(self.max_pitch_deg)

    @property
    def commit_box_height_px(self) -> float:
        return self.camera_height_px * self.commit_box_height_fraction
