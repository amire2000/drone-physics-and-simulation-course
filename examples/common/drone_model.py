"""Typed drone profiles, physical models, clock settings, and state."""

from dataclasses import dataclass, replace
from functools import lru_cache
from pathlib import Path
from xml.etree import ElementTree

import yaml


@dataclass(frozen=True)
class DroneModel:
    """Physical and actuator constants for the course's four-rotor drone."""

    mass_kg: float = 0.65
    arm_offset_m: float = 0.12
    max_rpm: float = 24_000.0
    max_thrust_per_motor_n: float = 0.65 * 9.81
    motor_time_constant_s: float = 0.05
    rotor_drag_coefficient: float = 2e-6
    motor_yaw_signs: tuple[int, int, int, int] = (1, -1, -1, 1)
    urdf_path: Path = Path(__file__).parent / "assets" / "full_drone.urdf"

    @property
    def thrust_coefficient(self) -> float:
        """Return the teaching-model thrust coefficient in N/RPM²."""
        return self.max_thrust_per_motor_n / self.max_rpm**2

    @property
    def torque_coefficient(self) -> float:
        """Return the reaction-torque coefficient in N m/RPM²."""
        return 0.015 * self.thrust_coefficient

    @property
    def motor_positions_m(self) -> tuple[tuple[float, float], ...]:
        """Return the four X-frame rotor positions in the body frame."""
        arm = self.arm_offset_m
        return ((arm, arm), (arm, -arm), (-arm, arm), (-arm, -arm))


@dataclass(frozen=True)
class PhysicsSettings:
    """World, clock, and environment settings for a PyBullet simulation."""

    gravity_z_mps2: float = -9.81
    physics_hz: int = 240
    control_hz: int = 120
    wind_world_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    wind_enabled: bool = True
    air_density_kg_m3: float = 1.225
    body_drag_enabled: bool = True
    body_drag_cd_area_m2: tuple[float, float, float] = (0.012, 0.012, 0.020)
    angular_damping_enabled: bool = True
    angular_damping_nm_per_rad_s: tuple[float, float, float] = (0.0012, 0.0012, 0.0020)
    rotor_aerodynamics_enabled: bool = False
    propeller_diameter_m: float = 0.14
    inflow_coefficient: float = 0.35
    blade_flapping_coefficient: float = 0.10
    ground_effect_enabled: bool = False
    ground_effect_height_m: float = 0.35
    ground_effect_coefficient: float = 0.10
    ground_effect_max_multiplier: float = 1.25
    gyroscopic_torque_enabled: bool = False
    rotor_inertia_kg_m2: float = 5e-6

    @property
    def time_step_s(self) -> float:
        """Return one PyBullet integration timestep in seconds."""
        return 1.0 / self.physics_hz

    @property
    def control_steps(self) -> int:
        """Return the number of physics steps between controller updates."""
        return self.physics_hz // self.control_hz


@dataclass(frozen=True)
class DroneProfile:
    """One reusable vehicle definition: URDF mass plus actuator and aero constants."""

    name: str
    model: DroneModel
    physics_settings: PhysicsSettings


def _profile_path(name: str) -> Path:
    """Return the YAML path for one named shared drone profile."""
    path = Path(__file__).parent / "drone_profiles" / f"{name}.yaml"
    if not path.is_file():
        raise ValueError(f"unknown drone profile '{name}'")
    return path


def _base_mass_from_urdf(path: Path) -> float:
    """Read the base-link mass from the URDF, the source of rigid-body mass."""
    try:
        root = ElementTree.parse(path).getroot()
        mass = root.find("./link[@name='base_link']/inertial/mass")
        if mass is None or "value" not in mass.attrib:
            raise ValueError("base_link needs an inertial mass")
        value = float(mass.attrib["value"])
    except (ElementTree.ParseError, ValueError) as exc:
        raise ValueError(f"invalid drone URDF '{path}': {exc}") from exc
    if value <= 0:
        raise ValueError(f"invalid drone URDF '{path}': mass must be positive")
    return value


@lru_cache(maxsize=None)
def load_drone_profile(name: str = "default") -> DroneProfile:
    """Load one profile and derive its model mass from the linked URDF."""
    path = _profile_path(name)
    try:
        data = yaml.safe_load(path.read_text())
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid drone profile '{path}': {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"drone profile '{path}' must be a mapping")

    allowed = {"name", "urdf", "actuators", "aerodynamics"}
    unknown = set(data) - allowed
    if unknown:
        raise ValueError(f"unknown drone profile setting(s): {', '.join(sorted(unknown))}")
    actuators = data.get("actuators", {})
    aerodynamics = data.get("aerodynamics", {})
    if not isinstance(actuators, dict) or not isinstance(aerodynamics, dict):
        raise ValueError(f"drone profile '{path}' actuator and aerodynamic sections must be mappings")

    urdf_name = data.get("urdf")
    if not isinstance(urdf_name, str):
        raise ValueError(f"drone profile '{path}' needs a URDF filename")
    urdf_path = Path(__file__).parent / "assets" / urdf_name
    mass_kg = _base_mass_from_urdf(urdf_path)
    model = DroneModel(
        mass_kg=mass_kg,
        urdf_path=urdf_path,
        **{key: tuple(value) if key == "motor_yaw_signs" else value for key, value in actuators.items()},
    )
    settings = replace(
        PhysicsSettings(),
        **{
            key: tuple(value) if key in {"body_drag_cd_area_m2", "angular_damping_nm_per_rad_s"} else value
            for key, value in aerodynamics.items()
        },
    )
    return DroneProfile(str(data.get("name", name)), model, settings)


@dataclass(frozen=True)
class DroneState:
    """Ideal PyBullet state: position, velocity, quaternion, and body rate."""

    position_m: tuple[float, float, float]
    linear_velocity_mps: tuple[float, float, float]
    orientation_quaternion: tuple[float, float, float, float]
    angular_velocity_body_rad_s: tuple[float, float, float]


@dataclass(frozen=True)
class ImuReading:
    """Ideal IMU attitude and body-frame angular-rate measurement."""

    roll_pitch_yaw_rad: tuple[float, float, float]
    angular_velocity_body_rad_s: tuple[float, float, float]


@dataclass(frozen=True)
class PhysicsStep:
    """Actuator, force, and state result produced by one engine step."""

    collective_pwm_us: float
    motor_rpms: tuple[float, float, float, float]
    motor_thrusts_n: tuple[float, float, float, float]
    total_thrust_n: float
    drag_force_body_n: tuple[float, float, float]
    body_drag_force_body_n: tuple[float, float, float]
    angular_damping_torque_body_nm: tuple[float, float, float]
    gyroscopic_torque_body_nm: tuple[float, float, float]
    ground_effect_multipliers: tuple[float, float, float, float]
    state: DroneState


DEFAULT_DRONE_PROFILE = load_drone_profile()
DEFAULT_DRONE_MODEL = DEFAULT_DRONE_PROFILE.model
DEFAULT_PHYSICS_SETTINGS = DEFAULT_DRONE_PROFILE.physics_settings
