"""Typed physical model, clock settings, and state used by course simulations."""

from dataclasses import dataclass
from pathlib import Path


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

    @property
    def time_step_s(self) -> float:
        """Return one PyBullet integration timestep in seconds."""
        return 1.0 / self.physics_hz

    @property
    def control_steps(self) -> int:
        """Return the number of physics steps between controller updates."""
        return self.physics_hz // self.control_hz


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
    state: DroneState


DEFAULT_DRONE_MODEL = DroneModel()
DEFAULT_PHYSICS_SETTINGS = PhysicsSettings()
