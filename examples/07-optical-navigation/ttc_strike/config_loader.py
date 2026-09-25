"""YAML adapter for the typed TTC strike configuration."""

from dataclasses import replace
from math import isfinite
from numbers import Real
from pathlib import Path

import yaml

from .config import RuntimeConfig, SimulationConfig, StrikeConfig


def _mapping(value: object, name: str) -> dict:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")
    return value


def _position(value: object, name: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} must contain three numbers")
    if any(isinstance(item, bool) or not isinstance(item, Real) or not isfinite(float(item)) for item in value):
        raise ValueError(f"{name} must contain finite numbers")
    return tuple(float(item) for item in value)


def _gains(value: object, name: str) -> tuple[float, float, float]:
    return _position(value, name)


def _merge(instance, values: dict, names: tuple[str, ...], section_name: str):
    unknown = set(values) - set(names)
    if unknown:
        raise ValueError(f"unknown {section_name} setting(s): {', '.join(sorted(unknown))}")
    return replace(instance, **{name: values[name] for name in names if name in values})


def load_yaml_config(path: Path) -> StrikeConfig:
    """Load grouped YAML and apply only supported scenario overrides."""
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid YAML in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError("the YAML root must be a mapping")

    simulation_data = _mapping(data.get("simulation"), "simulation")
    runtime_data = _mapping(data.get("runtime"), "runtime")
    if not simulation_data and not runtime_data:
        raise ValueError("YAML must contain simulation and/or runtime sections")

    scene = _mapping(simulation_data.get("scene"), "simulation.scene")
    vehicle = _mapping(simulation_data.get("vehicle_model"), "simulation.vehicle_model")
    sensor_model = _mapping(simulation_data.get("sensor_model"), "simulation.sensor_model")
    display = _mapping(simulation_data.get("display"), "simulation.display")
    recording = _mapping(simulation_data.get("recording"), "simulation.recording")
    physical = _mapping(runtime_data.get("physical_setup"), "runtime.physical_setup")
    camera = _mapping(physical.get("camera"), "runtime.physical_setup.camera")
    mission = _mapping(runtime_data.get("mission"), "runtime.mission")
    limits = _mapping(runtime_data.get("flight_limits"), "runtime.flight_limits")
    ttc = _mapping(runtime_data.get("ttc"), "runtime.ttc")
    filters = _mapping(runtime_data.get("sensor_filters"), "runtime.sensor_filters")
    pid = _mapping(runtime_data.get("pid"), "runtime.pid")
    vertical = _mapping(runtime_data.get("vertical_control"), "runtime.vertical_control")

    simulation = SimulationConfig()
    runtime = RuntimeConfig()
    if "launch_position" in scene:
        simulation = replace(simulation, launch_position=_position(scene["launch_position"], "simulation.scene.launch_position"))
    if "target_center" in scene:
        simulation = replace(simulation, target_center=_position(scene["target_center"], "simulation.scene.target_center"))
    if "environment_size_px" in display:
        simulation = replace(simulation, environment_size_px=tuple(display["environment_size_px"]))
    if "opencv_window_position_px" in display:
        simulation = replace(simulation, opencv_window_position_px=tuple(display["opencv_window_position_px"]))
    if "plot_window_position_px" in display:
        simulation = replace(simulation, plot_window_position_px=tuple(display["plot_window_position_px"]))
    if "target_size_m" in scene:
        simulation = replace(simulation, target_size_m=scene["target_size_m"])
    simulation = _merge(simulation, vehicle, ("vehicle_mass_kg", "gravity_mps2"), "simulation.vehicle_model")
    simulation = _merge(simulation, sensor_model, ("barometer_noise_sigma_m", "barometer_bias_m", "random_seed"), "simulation.sensor_model")
    simulation = _merge(simulation, recording, ("post_impact_seconds",), "simulation.recording")

    runtime = _merge(runtime, camera, ("camera_width_px", "camera_height_px", "camera_hz", "camera_fov_deg", "camera_look_down_deg"), "runtime.physical_setup.camera")
    runtime = _merge(runtime, mission, ("takeoff_altitude_m", "impact_altitude_m", "forward_speed_mps", "nominal_pitch_deg"), "runtime.mission")
    runtime = _merge(runtime, limits, ("max_descent_velocity_mps", "max_climb_velocity_mps", "max_pitch_deg", "takeoff_altitude_tolerance_m", "takeoff_velocity_tolerance_mps", "commit_timeout_margin_s"), "runtime.flight_limits")
    runtime = _merge(runtime, ttc, ("min_ttc_s", "commit_box_height_fraction", "ttc_growth_old_weight", "min_growth_px_per_s"), "runtime.ttc")
    runtime = _merge(runtime, filters, ("barometer_velocity_old_weight",), "runtime.sensor_filters")
    runtime = _merge(runtime, vertical, ("vertical_position_correction",), "runtime.vertical_control")

    for name, field_name in (("altitude", "altitude_pid_gains"), ("forward_speed", "forward_speed_pid_gains"), ("pitch_attitude", "pitch_attitude_pid_gains"), ("vertical_velocity", "vertical_velocity_pid_gains")):
        values = _mapping(pid.get(name), f"runtime.pid.{name}")
        if "gains" in values:
            runtime = replace(runtime, **{field_name: _gains(values["gains"], f"runtime.pid.{name}.gains")})
        if name == "altitude" and "integral_limit" in values:
            runtime = replace(runtime, altitude_integral_limit=values["integral_limit"])
        if name == "forward_speed" and "integral_limit" in values:
            runtime = replace(runtime, forward_pitch_integral_limit=values["integral_limit"])
        unknown = set(values) - {"gains", "integral_limit"}
        if unknown:
            raise ValueError(f"unknown runtime.pid.{name} setting(s): {', '.join(sorted(unknown))}")
    return StrikeConfig(simulation=simulation, runtime=runtime)
