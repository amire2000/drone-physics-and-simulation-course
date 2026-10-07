"""Linear ADRC cascade for Module 04."""

from dataclasses import dataclass

from common.control import clamp


@dataclass
class LinearADRC:
    """Linear active-disturbance-rejection loop with a third-order ESO."""

    b0: float
    kp: float
    kd: float
    beta1: float
    beta2: float
    beta3: float
    output_limit: float
    z1: float = 0.0
    z2: float = 0.0
    z3: float = 0.0
    previous_output: float = 0.0

    @classmethod
    def from_bandwidth(cls, b0: float, control_bandwidth: float, observer_bandwidth: float, output_limit: float) -> "LinearADRC":
        """Build an ADRC loop from intuitive controller and observer bandwidths."""
        return cls(b0, control_bandwidth**2, 2.0 * control_bandwidth, 3.0 * observer_bandwidth, 3.0 * observer_bandwidth**2, observer_bandwidth**3, output_limit)

    def reset(self) -> None:
        """Clear extended-state estimates before a new flight."""
        self.z1 = self.z2 = self.z3 = self.previous_output = 0.0

    def update(self, reference: float, measurement: float, dt: float) -> float:
        """Estimate total disturbance and return bounded ADRC control output."""
        # ESO: z1 estimates output, z2 its rate, and z3 the lumped disturbance.
        estimation_error = self.z1 - measurement
        self.z1 += dt * (self.z2 - self.beta1 * estimation_error)
        self.z2 += dt * (self.z3 + self.b0 * self.previous_output - self.beta2 * estimation_error)
        self.z3 += dt * (-self.beta3 * estimation_error)
        # Nonlinear plant inversion is reduced here to u = (u0 - disturbance) / b0.
        desired_output = self.kp * (reference - self.z1) - self.kd * self.z2
        self.previous_output = clamp((desired_output - self.z3) / self.b0, -self.output_limit, self.output_limit)
        return self.previous_output


@dataclass(frozen=True)
class Config:
    """ADRC bandwidths, plant gains, and limits supplied by main."""

    altitude_b0: float
    altitude_control_bandwidth: float
    altitude_observer_bandwidth: float
    altitude_output_limit_mps2: float
    attitude_b0: float
    attitude_control_bandwidth: float
    attitude_observer_bandwidth: float
    attitude_rate_limit_rad_s: float
    rate_b0: tuple[float, float, float]
    rate_control_bandwidth: float
    rate_observer_bandwidth: float
    torque_limit_nm: tuple[float, float, float]
    max_total_thrust_n: float


class Controller:
    """Cascade altitude, attitude, and body-rate linear ADRC control."""

    def __init__(self, config: Config) -> None:
        """Create ADRC loops for altitude, attitude, and each body-rate axis."""
        self.config = config
        self.altitude = LinearADRC.from_bandwidth(config.altitude_b0, config.altitude_control_bandwidth, config.altitude_observer_bandwidth, config.altitude_output_limit_mps2)
        self.attitude = tuple(LinearADRC.from_bandwidth(config.attitude_b0, config.attitude_control_bandwidth, config.attitude_observer_bandwidth, config.attitude_rate_limit_rad_s) for _ in range(3))
        self.rate = tuple(LinearADRC.from_bandwidth(b0, config.rate_control_bandwidth, config.rate_observer_bandwidth, limit) for b0, limit in zip(config.rate_b0, config.torque_limit_nm))

    def reset(self) -> None:
        """Clear all ADRC observer states after a reset."""
        for loop in (self.altitude, *self.attitude, *self.rate):
            loop.reset()

    def update(self, position_z_m: float, velocity_z_mps: float, attitude_rad: tuple[float, float, float], body_rates_rad_s: tuple[float, float, float], target_altitude_m: float, dt: float, mass_kg: float, gravity_mps2: float) -> tuple[float, tuple[float, float, float], tuple[float, float, float]]:
        """Return collective thrust, body torque, and desired angular rates."""
        # ADRC altitude output is commanded vertical acceleration in m/s².
        altitude_acceleration = self.altitude.update(target_altitude_m, position_z_m, dt)
        collective_thrust = clamp(mass_kg * (gravity_mps2 + altitude_acceleration), 0.0, self.config.max_total_thrust_n)
        desired_rates = tuple(loop.update(0.0, angle, dt) for loop, angle in zip(self.attitude, attitude_rad))
        torque = tuple(loop.update(target, measured, dt) for loop, target, measured in zip(self.rate, desired_rates, body_rates_rad_s))
        return collective_thrust, torque, desired_rates
