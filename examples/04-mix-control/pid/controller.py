"""Cascaded PID controller for Module 04."""

from dataclasses import dataclass

from common.control import clamp


@dataclass
class PID:
    """Small PID controller with an externally supplied error derivative."""

    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float
    integral: float = 0.0

    def reset(self) -> None:
        """Clear accumulated error before a new flight."""
        self.integral = 0.0

    def update(self, error: float, derivative: float, dt: float) -> float:
        """Return bounded PID output using SI-unit error and derivative."""
        self.integral = clamp(self.integral + error * dt, -self.integral_limit, self.integral_limit)
        return clamp(self.kp * error + self.ki * self.integral + self.kd * derivative, -self.output_limit, self.output_limit)


@dataclass(frozen=True)
class Config:
    """PID gains and limits supplied by the main module."""

    altitude_gains: tuple[float, float, float]
    altitude_integral_limit: float
    altitude_output_limit_n: float
    attitude_gains: tuple[float, float, float]
    attitude_integral_limit: float
    attitude_rate_limit_rad_s: float
    rate_gains: tuple[float, float, float]
    rate_integral_limit: float
    torque_limit_nm: tuple[float, float, float]
    max_total_thrust_n: float


class Controller:
    """Cascade altitude, attitude, and body-rate PID control."""

    def __init__(self, config: Config) -> None:
        """Create independent PID state for altitude and each body axis."""
        self.config = config
        self.altitude = PID(*config.altitude_gains, config.altitude_integral_limit, config.altitude_output_limit_n)
        self.attitude = tuple(PID(*config.attitude_gains, config.attitude_integral_limit, config.attitude_rate_limit_rad_s) for _ in range(3))
        self.rate = tuple(PID(*config.rate_gains, config.rate_integral_limit, limit) for limit in config.torque_limit_nm)

    def reset(self) -> None:
        """Clear all PID integrators after a reset."""
        for pid in (self.altitude, *self.attitude, *self.rate):
            pid.reset()

    def update_attitude_rate(self, attitude_rad: tuple[float, float, float], body_rates_rad_s: tuple[float, float, float], target_attitude_rad: tuple[float, float, float], dt: float) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """Return body torque for an externally supplied attitude setpoint."""
        desired_rates = tuple(pid.update(target - angle, -rate, dt) for pid, target, angle, rate in zip(self.attitude, target_attitude_rad, attitude_rad, body_rates_rad_s))
        # Body torque: tau = PID(desired_rate - measured_rate).
        torque = tuple(pid.update(target - measured, 0.0, dt) for pid, target, measured in zip(self.rate, desired_rates, body_rates_rad_s))
        return torque, desired_rates

    def update(self, position_z_m: float, velocity_z_mps: float, attitude_rad: tuple[float, float, float], body_rates_rad_s: tuple[float, float, float], target_altitude_m: float, target_attitude_rad: tuple[float, float, float], dt: float, mass_kg: float, gravity_mps2: float) -> tuple[float, tuple[float, float, float], tuple[float, float, float]]:
        """Return collective thrust, body torque, and desired angular rates."""
        # Vertical force: T = m(g + a_command), positive along body +Z when level.
        altitude_output = self.altitude.update(target_altitude_m - position_z_m, -velocity_z_mps, dt)
        collective_thrust = clamp(mass_kg * gravity_mps2 + altitude_output, 0.0, self.config.max_total_thrust_n)
        torque, desired_rates = self.update_attitude_rate(attitude_rad, body_rates_rad_s, target_attitude_rad, dt)
        return collective_thrust, torque, desired_rates
