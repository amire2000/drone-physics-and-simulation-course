"""Pure PID cascade for the Module 04 teaching example."""

from dataclasses import dataclass


def clamp(value: float, low: float, high: float) -> float:
    """Limit a scalar to an inclusive range."""
    return max(low, min(high, value))


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
class ControllerConfig:
    """All controller gains and limits supplied by the main module."""

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


class FlightController:
    """Cascade altitude, attitude, and body-rate control into thrust and torque."""

    def __init__(self, config: ControllerConfig) -> None:
        """Create one controller with independent PID state for each axis."""
        self.config = config
        self.altitude = PID(*config.altitude_gains, config.altitude_integral_limit, config.altitude_output_limit_n)
        self.attitude = tuple(PID(*config.attitude_gains, config.attitude_integral_limit, config.attitude_rate_limit_rad_s) for _ in range(3))
        self.rate = tuple(PID(*config.rate_gains, config.rate_integral_limit, limit) for limit in config.torque_limit_nm)

    def reset(self) -> None:
        """Clear all controller integrators after a simulation reset."""
        for pid in (self.altitude, *self.attitude, *self.rate):
            pid.reset()

    def update(
        self,
        position_z_m: float,
        velocity_z_mps: float,
        attitude_rad: tuple[float, float, float],
        body_rates_rad_s: tuple[float, float, float],
        target_altitude_m: float,
        dt: float,
        mass_kg: float,
        gravity_mps2: float,
    ) -> tuple[float, tuple[float, float, float], tuple[float, float, float]]:
        """Return collective thrust, body torque, and desired angular rates."""
        # Vertical world-frame force: T = m(g + a_command), positive along body +Z when level.
        altitude_output = self.altitude.update(target_altitude_m - position_z_m, -velocity_z_mps, dt)
        collective_thrust = clamp(mass_kg * gravity_mps2 + altitude_output, 0.0, self.config.max_total_thrust_n)

        # Attitude error in radians becomes a desired body-rate command.
        desired_rates = tuple(
            pid.update(-angle, -rate, dt)
            for pid, angle, rate in zip(self.attitude, attitude_rad, body_rates_rad_s)
        )
        # Body torque: tau = PID(desired_rate - measured_rate).
        torque = tuple(
            pid.update(target - measured, 0.0, dt)
            for pid, target, measured in zip(self.rate, desired_rates, body_rates_rad_s)
        )
        return collective_thrust, torque, desired_rates
