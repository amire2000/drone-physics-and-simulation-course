"""Composable PyBullet sensor models for the Betaflight FDM bridge."""

from __future__ import annotations

from dataclasses import dataclass
from math import exp
from random import Random

import pybullet as p

from bridge.types import FlightObservation

Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class ImuConfig:
    """Fixed bias and independent Gaussian noise settings for the IMU."""

    gyro_bias_rad_s: Vector3 = (0.0, 0.0, 0.0)
    gyro_noise_sigma_rad_s: Vector3 = (0.0, 0.0, 0.0)
    accelerometer_bias_mps2: Vector3 = (0.0, 0.0, 0.0)
    accelerometer_noise_sigma_mps2: Vector3 = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class BarometerConfig:
    """Fixed bias and Gaussian noise settings for the barometer."""

    pressure_bias_pa: float = 0.0
    pressure_noise_sigma_pa: float = 0.0


def _add_bias_and_noise(value: Vector3, bias: Vector3, sigma: Vector3, rng: Random) -> Vector3:
    """Apply per-axis fixed bias and zero-mean Gaussian sample noise."""
    return tuple(
        sample + offset + (rng.gauss(0.0, deviation) if deviation else 0.0)
        for sample, offset, deviation in zip(value, bias, sigma)
    )  # type: ignore[return-value]


class ImuSensor:
    """Convert PyBullet motion into noisy body-rate and specific-force samples."""

    def __init__(self, gravity_mps2: float = 9.81, config: ImuConfig | None = None, rng: Random | None = None) -> None:
        """Create an IMU model with optional bias, noise, and deterministic RNG."""
        self._gravity_mps2 = gravity_mps2
        self._config = config or ImuConfig()
        self._rng = rng or Random()
        self._previous_velocity: Vector3 | None = None

    def reset(self) -> None:
        """Discard the previous velocity before a new simulation run."""
        self._previous_velocity = None

    def read(
        self,
        orientation: tuple[float, float, float, float],
        velocity_world: Vector3,
        angular_velocity_world: Vector3,
        dt: float,
    ) -> tuple[Vector3, Vector3]:
        """Return noisy body rates and body specific force in SI units."""
        rotation = p.getMatrixFromQuaternion(orientation)
        # Body rate = R_world_to_body * omega_world.
        body_rates = tuple(
            sum(rotation[3 * row + axis] * angular_velocity_world[row] for row in range(3))
            for axis in range(3)
        )
        if self._previous_velocity is None:
            acceleration_world = (0.0, 0.0, 0.0)
        else:
            acceleration_world = tuple(
                (current - previous) / dt for current, previous in zip(velocity_world, self._previous_velocity)
            )
        self._previous_velocity = tuple(velocity_world)
        # Specific force is a - g; a level stationary vehicle reports +g in body +Z.
        non_gravitational_world = (
            acceleration_world[0],
            acceleration_world[1],
            acceleration_world[2] + self._gravity_mps2,
        )
        specific_force_body = tuple(
            sum(rotation[3 * row + axis] * non_gravitational_world[row] for row in range(3))
            for axis in range(3)
        )
        noisy_rates = _add_bias_and_noise(
            body_rates,
            self._config.gyro_bias_rad_s,
            self._config.gyro_noise_sigma_rad_s,
            self._rng,
        )
        noisy_force = _add_bias_and_noise(
            specific_force_body,
            self._config.accelerometer_bias_mps2,
            self._config.accelerometer_noise_sigma_mps2,
            self._rng,
        )
        return noisy_rates, noisy_force


class BarometerSensor:
    """Convert home-relative altitude into a noisy barometric pressure sample."""

    def __init__(self, config: BarometerConfig | None = None, rng: Random | None = None) -> None:
        """Create a barometer model with optional bias, noise, and deterministic RNG."""
        self._config = config or BarometerConfig()
        self._rng = rng or Random()

    def read(self, altitude_m: float) -> float:
        """Return pressure in pascals using the standard atmosphere approximation."""
        # p = p0 * exp(-h / scale_height), with altitude in metres and pressure in Pa.
        pressure_pa = 101_325.0 * exp(-altitude_m / 8_434.5)
        noise = (
            self._rng.gauss(0.0, self._config.pressure_noise_sigma_pa)
            if self._config.pressure_noise_sigma_pa
            else 0.0
        )
        return pressure_pa + self._config.pressure_bias_pa + noise


class PyBulletObservationAdapter:
    """Read PyBullet state and compose the IMU and barometer sensor models."""

    def __init__(
        self,
        home_position_world_m: Vector3,
        gravity_mps2: float = 9.81,
        imu: ImuSensor | None = None,
        barometer: BarometerSensor | None = None,
    ) -> None:
        """Create an adapter whose local ENU origin is the takeoff base position."""
        self._home_position_world_m = home_position_world_m
        self._imu = imu or ImuSensor(gravity_mps2=gravity_mps2)
        self._barometer = barometer or BarometerSensor()

    def reset(self) -> None:
        """Reset stateful sensor history before a new simulation run."""
        self._imu.reset()

    def read(self, drone: int, timestamp_s: float, dt: float) -> FlightObservation:
        """Return a home-relative snapshot with composed sensor measurements."""
        position, orientation = p.getBasePositionAndOrientation(drone)
        # ! Module 08 home-relative odometry: subtract the frozen takeoff base pose.
        # PyBullet X/Y/Z are the course ENU E/N/U axes, all measured in metres.
        position_home_enu_m = tuple(current - home for current, home in zip(position, self._home_position_world_m))
        velocity, angular_velocity_world = p.getBaseVelocity(drone)
        body_rates, specific_force_body = self._imu.read(
            orientation,
            tuple(velocity),
            tuple(angular_velocity_world),
            dt,
        )
        pressure_pa = self._barometer.read(position_home_enu_m[2])
        return FlightObservation(
            timestamp_s,
            position_home_enu_m,
            tuple(velocity),
            tuple(orientation),
            body_rates,
            specific_force_body,
            pressure_pa,
        )
