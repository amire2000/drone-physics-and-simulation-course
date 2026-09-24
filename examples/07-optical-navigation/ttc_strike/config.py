"""Control configuration and simulator-only scene configuration."""

from dataclasses import dataclass, replace
from math import isfinite, radians
from pathlib import Path
from numbers import Real

import yaml


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
    pitch_attitude_pid_gains: tuple[float, float, float] = (0.008, 0.0, 0.006)
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


def _position(value: object, field: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{field} must be a list of three numbers")
    if any(isinstance(item, bool) or not isinstance(item, Real) or not isfinite(float(item)) for item in value):
        raise ValueError(f"{field} must contain only finite numbers")
    return tuple(float(item) for item in value)


def load_yaml_config(path: Path) -> tuple[StrikeConfig, SceneConfig]:
    """Load the small set of scenario overrides supported by the example."""
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid YAML in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError("the YAML root must be a mapping")

    drone = data.get("drone", {})
    box = data.get("box", {})
    flight = data.get("flight", {})
    if not all(isinstance(section, dict) for section in (drone, box, flight)):
        raise ValueError("drone, box, and flight must be mappings")

    config = StrikeConfig()
    scene = SceneConfig()
    if "position" in drone:
        config = replace(config, launch_position=_position(drone["position"], "drone.position"))
    if "position" in box:
        scene = replace(scene, target_center=_position(box["position"], "box.position"))
    if "takeoff_altitude_m" in flight:
        altitude = flight["takeoff_altitude_m"]
        if isinstance(altitude, bool) or not isinstance(altitude, Real) or not isfinite(float(altitude)) or float(altitude) < 0:
            raise ValueError("flight.takeoff_altitude_m must be a finite non-negative number")
        config = replace(config, takeoff_altitude_m=float(altitude))
    if "commit_box_height_fraction" in flight:
        fraction = flight["commit_box_height_fraction"]
        if isinstance(fraction, bool) or not isinstance(fraction, Real) or not isfinite(float(fraction)) or not 0 < float(fraction) <= 1:
            raise ValueError("flight.commit_box_height_fraction must be greater than 0 and at most 1")
        config = replace(config, commit_box_height_fraction=float(fraction))
    return config, scene
