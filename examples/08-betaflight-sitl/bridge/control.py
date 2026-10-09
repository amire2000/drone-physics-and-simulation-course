"""Outer Module 04 PID cascade that emits Betaflight virtual RC commands."""

from __future__ import annotations

from dataclasses import dataclass
from math import asin, atan2, copysign, pi

from common.position import acceleration_to_attitude_thrust
from pid.controller import PID
from pid.position import Controller as PositionController
from pid.position import Config as PositionConfig
from pid.velocity import Controller as VelocityController
from pid.velocity import Config as VelocityConfig

from bridge.protocol import clamp
from bridge.types import FlightObservation, PilotCommand


@dataclass(frozen=True)
class OuterPidConfig:
    """PID, mapping, and multi-rate limits for the position-to-RC boundary."""

    position_gains: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    velocity_gains: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    velocity_limits_mps: tuple[float, float, float]
    acceleration_limits_mps2: tuple[float, float, float]
    position_integral_limit: float
    velocity_integral_limit: float
    max_tilt_rad: float
    max_total_thrust_n: float
    hover_throttle: float
    throttle_per_newton: float
    max_angle_mode_tilt_rad: float
    yaw_gains: tuple[float, float, float]
    yaw_integral_limit: float
    prearm_delay_s: float = 1.0
    arming_delay_s: float = 1.0
    position_hz: int = 30
    velocity_hz: int = 60


class OuterPositionPid:
    """Convert home-relative XYZ/yaw targets into bounded Betaflight RC intent."""

    def __init__(self, config: OuterPidConfig, *, mass_kg: float, gravity_mps2: float = 9.81) -> None:
        """Create the reused Module 04 position/velocity PID cascade."""
        self._config = config
        self._mass_kg = mass_kg
        self._gravity_mps2 = gravity_mps2
        self._position = PositionController(PositionConfig(config.position_gains, config.position_integral_limit, config.velocity_limits_mps))
        self._velocity = VelocityController(VelocityConfig(config.velocity_gains, config.velocity_integral_limit, config.acceleration_limits_mps2))
        self._yaw = PID(*config.yaw_gains, config.yaw_integral_limit, 1.0)
        self._position_elapsed_s = 0.0
        self._velocity_elapsed_s = 0.0
        self._desired_velocity = (0.0, 0.0, 0.0)
        self._desired_acceleration = (0.0, 0.0, 0.0)
        self._arm_request_active = False
        self._arming_elapsed_s = 0.0

    def reset(self) -> None:
        """Clear PID state before arming or after a bridge fault."""
        self._position.reset()
        self._velocity.reset()
        self._yaw.reset()
        self._position_elapsed_s = 0.0
        self._velocity_elapsed_s = 0.0
        self._desired_velocity = (0.0, 0.0, 0.0)
        self._desired_acceleration = (0.0, 0.0, 0.0)
        self._arm_request_active = False
        self._arming_elapsed_s = 0.0

    def update(self, observation: FlightObservation, target_position_m: tuple[float, float, float], target_yaw_rad: float, dt: float, *, armed: bool) -> PilotCommand:
        """Return one Angle-mode RC command from the latest PyBullet observation."""
        if not armed:
            if self._arm_request_active:
                self.reset()
            return PilotCommand(0.0, 0.0, 0.0, 0.0, False, True)

        if not self._arm_request_active:
            self.reset()
            self._arm_request_active = True
        self._arming_elapsed_s += dt
        if self._arming_elapsed_s < self._config.prearm_delay_s:
            # Betaflight must first observe the arm switch low after connecting.
            return PilotCommand(0.0, 0.0, 0.0, 0.0, False, True)
        if self._arming_elapsed_s < self._config.prearm_delay_s + self._config.arming_delay_s:
            # Betaflight must then see an armed, low-throttle command before hover thrust.
            return PilotCommand(0.0, 0.0, 0.0, 0.0, True, True)

        self._position_elapsed_s += dt
        self._velocity_elapsed_s += dt
        position_period_s = 1.0 / self._config.position_hz
        velocity_period_s = 1.0 / self._config.velocity_hz
        if self._position_elapsed_s >= position_period_s:
            self._desired_velocity = self._position.update(target_position_m, observation.position_home_enu_m, observation.velocity_world_mps, self._position_elapsed_s)
            self._position_elapsed_s = 0.0
        if self._velocity_elapsed_s >= velocity_period_s:
            self._desired_acceleration = self._velocity.update(self._desired_velocity, observation.velocity_world_mps, self._velocity_elapsed_s)
            self._velocity_elapsed_s = 0.0

        _, _, measured_yaw = _euler_from_xyzw(observation.orientation_xyzw)
        collective, attitude = acceleration_to_attitude_thrust(
            self._desired_acceleration,
            measured_yaw,
            self._mass_kg,
            self._gravity_mps2,
            self._config.max_total_thrust_n,
            self._config.max_tilt_rad,
        )
        yaw_error = _wrap_angle(target_yaw_rad - measured_yaw)
        # ! Module 08 Betaflight SITL: convert the PyBullet +Z yaw request to RC yaw.
        # PyBullet +Z is counter-clockwise. Betaflight negates RC yaw internally;
        # with yaw_motors_reversed ON this produces positive PyBullet yaw torque.
        yaw_stick = self._yaw.update(yaw_error, -observation.body_rates_rad_s[2], dt)
        # RC roll/pitch are Angle-mode requests, not motor or torque commands.
        return PilotCommand(
            clamp(attitude[0] / self._config.max_angle_mode_tilt_rad, -1.0, 1.0),
            clamp(attitude[1] / self._config.max_angle_mode_tilt_rad, -1.0, 1.0),
            clamp(self._config.hover_throttle + (collective - self._mass_kg * self._gravity_mps2) * self._config.throttle_per_newton, 0.0, 1.0),
            yaw_stick,
            True,
            True,
        )


def _euler_from_xyzw(quaternion_xyzw: tuple[float, float, float, float]) -> tuple[float, float, float]:
    """Return roll, pitch, and yaw from an XYZW quaternion without PyBullet."""
    x, y, z, w = quaternion_xyzw
    roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_sine = 2.0 * (w * y - z * x)
    pitch = copysign(pi / 2.0, pitch_sine) if abs(pitch_sine) >= 1.0 else asin(pitch_sine)
    yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _wrap_angle(angle_rad: float) -> float:
    """Wrap a yaw error to the shortest signed angular distance."""
    return (angle_rad + pi) % (2.0 * pi) - pi
