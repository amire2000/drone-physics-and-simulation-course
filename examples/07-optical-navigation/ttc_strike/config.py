"""Immutable configuration and derived geometry for the strike scenario."""

from dataclasses import dataclass
from math import cos, radians, sin


@dataclass(frozen=True)
class StrikeConfig:
    """All tunable values for one TTC diagonal-strike scenario."""

    target_center: tuple[float, float, float] = (20.0, 0.0, 1.0)
    target_size_m: float = 2.0
    launch_position: tuple[float, float, float] = (-5.25, 0.0, 0.05)
    vehicle_mass_kg: float = 0.65
    gravity_mps2: float = 9.81
    takeoff_altitude_m: float = 15.0
    descent_angle_deg: float = 30.0
    terminal_speed_mps: float = 15.0
    camera_width_px: int = 640
    camera_height_px: int = 480
    camera_hz: int = 30
    camera_fov_deg: float = 60.0
    commit_box_height_fraction: float = 0.5
    ttc_growth_old_weight: float = 0.65
    min_growth_px_per_s: float = 0.01
    barometer_noise_sigma_m: float = 0.0
    barometer_bias_m: float = 0.0
    barometer_velocity_old_weight: float = 0.7
    random_seed: int = 7
    altitude_pid_gains: tuple[float, float, float] = (0.7, 0.05, 1.1)
    altitude_integral_limit: float = 0.5
    forward_pid_gains: tuple[float, float, float] = (0.08, 0.0, 0.0)
    vertical_velocity_pid_gains: tuple[float, float, float] = (0.7, 0.0, 0.0)
    vertical_position_correction: float = 0.8
    max_pitch_deg: float = 30.0
    takeoff_altitude_tolerance_m: float = 0.2
    takeoff_velocity_tolerance_mps: float = 0.5
    commit_timeout_margin_s: float = 0.5
    post_impact_seconds: float = 3.0
    accepted_impact_speed_mps: tuple[float, float] = (10.0, 20.0)
    environment_size_px: tuple[int, int] = (960, 540)
    opencv_window_position_px: tuple[int, int] = (20, 80)
    plot_window_position_px: tuple[int, int] = (700, 80)

    @property
    def target_face_x_m(self) -> float:
        return self.target_center[0] - self.target_size_m / 2

    @property
    def hover_thrust_n(self) -> float:
        return self.vehicle_mass_kg * self.gravity_mps2

    @property
    def descent_angle_rad(self) -> float:
        return radians(self.descent_angle_deg)

    @property
    def terminal_vx_mps(self) -> float:
        return self.terminal_speed_mps * cos(self.descent_angle_rad)

    @property
    def terminal_vz_mps(self) -> float:
        return -self.terminal_speed_mps * sin(self.descent_angle_rad)

    @property
    def path_length_m(self) -> float:
        return (self.takeoff_altitude_m - self.target_center[2]) / sin(self.descent_angle_rad)

    @property
    def path_acceleration_mps2(self) -> float:
        return self.terminal_speed_mps**2 / (2 * self.path_length_m)

    @property
    def initial_range_m(self) -> float:
        return self.target_face_x_m - self.launch_position[0]

    @property
    def commit_box_height_px(self) -> float:
        return self.camera_height_px * self.commit_box_height_fraction

    @property
    def max_pitch_rad(self) -> float:
        return radians(self.max_pitch_deg)
