"""Small immutable messages exchanged at the Betaflight SITL boundary."""

from dataclasses import dataclass


@dataclass(frozen=True)
class PilotCommand:
    """Pilot-style normalized sticks and switches for Betaflight virtual RC."""

    roll: float
    pitch: float
    throttle: float
    yaw: float
    armed: bool
    angle_mode: bool


@dataclass(frozen=True)
class FlightObservation:
    """PyBullet state with position expressed in takeoff-relative ENU metres."""

    timestamp_s: float
    position_home_enu_m: tuple[float, float, float]
    velocity_world_mps: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    body_rates_rad_s: tuple[float, float, float]
    specific_force_body_mps2: tuple[float, float, float]
    pressure_pa: float


@dataclass(frozen=True)
class MotorCommand:
    """Validated normalized QUADX outputs received from Betaflight SITL."""

    normalized: tuple[float, float, float, float]
    received_at_s: float
