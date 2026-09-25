"""Motor, force, drag, and PyBullet integration behind one physics-engine step."""

from math import sqrt

import pybullet as p

from .drone_model import DEFAULT_DRONE_MODEL, DEFAULT_PHYSICS_SETTINGS, DroneModel, PhysicsSettings, PhysicsStep
from .pybullet_sensors import read_state, world_to_body_vector

PWM_MIN, PWM_HOVER, PWM_MAX = 1000.0, 1500.0, 2000.0


def clamp(value: float, low: float, high: float) -> float:
    """Limit a scalar to an inclusive lower and upper bound."""
    return max(low, min(high, value))


def thrust_from_pwm(pwm_us: float, model: DroneModel = DEFAULT_DRONE_MODEL) -> float:
    """Map collective PWM to one motor's requested thrust in newtons."""
    normalized = clamp((pwm_us - PWM_MIN) / (PWM_MAX - PWM_MIN), 0.0, 1.0)
    return model.max_thrust_per_motor_n * normalized**2


def rpm_from_thrust(thrust_n: float, model: DroneModel = DEFAULT_DRONE_MODEL) -> float:
    """Convert one motor's bounded thrust request into target RPM."""
    return sqrt(clamp(thrust_n, 0.0, model.max_thrust_per_motor_n) / model.thrust_coefficient)


def pwm_from_thrust(thrust_n: float, model: DroneModel = DEFAULT_DRONE_MODEL) -> float:
    """Convert one motor's bounded thrust request into collective PWM."""
    normalized = sqrt(clamp(thrust_n, 0.0, model.max_thrust_per_motor_n) / model.max_thrust_per_motor_n)
    return PWM_MIN + (PWM_MAX - PWM_MIN) * normalized


class PhysicsEngine:
    """Own actuator state, apply all flight forces, and advance PyBullet once."""

    def __init__(self, model: DroneModel = DEFAULT_DRONE_MODEL, settings: PhysicsSettings = DEFAULT_PHYSICS_SETTINGS) -> None:
        """Create an engine with one model, environment, and four stopped motors."""
        self.model = model
        self.settings = settings
        self._motor_rpms = (0.0, 0.0, 0.0, 0.0)

    def reset(self, initial_rpms: tuple[float, float, float, float] | None = None) -> None:
        """Reset motor state, optionally priming all rotor RPM values for a test."""
        self._motor_rpms = initial_rpms or (0.0, 0.0, 0.0, 0.0)

    def pwm_from_thrust(self, thrust_n: float) -> float:
        """Convert a one-motor thrust request to PWM using this engine's model."""
        return pwm_from_thrust(thrust_n, self.model)

    def thrust_from_pwm(self, pwm_us: float) -> float:
        """Convert collective PWM to one-motor requested thrust for this model."""
        return thrust_from_pwm(pwm_us, self.model)

    def step(self, drone: int, collective_pwm_us: float, body_torque_nm: tuple[float, float, float]) -> PhysicsStep:
        """Mix commands, advance motors, apply forces, integrate, and return the new state."""
        requested_thrusts = self._mix_motor_thrusts(self.thrust_from_pwm(collective_pwm_us), body_torque_nm)
        target_rpms = tuple(rpm_from_thrust(thrust, self.model) for thrust in requested_thrusts)
        self._motor_rpms = self._advance_motor_rpms(target_rpms)
        motor_thrusts, total_thrust = self._apply_rotor_forces(drone)
        drag_force = self._apply_drag(drone)
        p.stepSimulation()
        return PhysicsStep(collective_pwm_us, self._motor_rpms, motor_thrusts, total_thrust, drag_force, read_state(drone))

    def _mix_motor_thrusts(self, collective_thrust_n: float, torque_nm: tuple[float, float, float]) -> tuple[float, float, float, float]:
        """Mix collective and body torque while scaling corrections at motor limits."""
        roll_torque, pitch_torque, yaw_torque = torque_nm
        collective = clamp(collective_thrust_n, 0.0, self.model.max_thrust_per_motor_n)
        arm = self.model.arm_offset_m
        deltas = tuple(
            y * roll_torque / (4 * arm**2)
            - x * pitch_torque / (4 * arm**2)
            + yaw_sign * yaw_torque / (4 * self.model.torque_coefficient / self.model.thrust_coefficient)
            for (x, y), yaw_sign in zip(self.model.motor_positions_m, self.model.motor_yaw_signs)
        )
        scale = 1.0
        for delta in deltas:
            if delta > 0:
                scale = min(scale, (self.model.max_thrust_per_motor_n - collective) / delta)
            elif delta < 0:
                scale = min(scale, collective / -delta)
        return tuple(collective + scale * delta for delta in deltas)

    def _advance_motor_rpms(self, target_rpms: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        """Advance actual RPM with the model's first-order motor response delay."""
        alpha = min(1.0, self.settings.time_step_s / self.model.motor_time_constant_s)
        return tuple(actual + alpha * (target - actual) for actual, target in zip(self._motor_rpms, target_rpms))

    def _apply_rotor_forces(self, drone: int) -> tuple[tuple[float, float, float, float], float]:
        """Apply rotor-link thrust and alternating reaction torque to PyBullet."""
        motor_thrusts = tuple(self.model.thrust_coefficient * rpm**2 for rpm in self._motor_rpms)
        for link_index, (yaw_sign, thrust, rpm) in enumerate(zip(self.model.motor_yaw_signs, motor_thrusts, self._motor_rpms)):
            p.applyExternalForce(drone, link_index, (0, 0, thrust), (0, 0, 0), p.LINK_FRAME)
            p.applyExternalTorque(drone, -1, (0, 0, yaw_sign * self.model.torque_coefficient * rpm**2), p.LINK_FRAME)
        return motor_thrusts, sum(motor_thrusts)

    def _apply_drag(self, drone: int) -> tuple[float, float, float]:
        """Apply rotor-dependent drag opposite velocity relative to configured wind."""
        state = read_state(drone)
        relative_velocity_world = tuple(velocity - wind for velocity, wind in zip(state.linear_velocity_mps, self.settings.wind_world_mps))
        relative_velocity_body = world_to_body_vector(drone, relative_velocity_world)
        drag_force = tuple(-self.model.rotor_drag_coefficient * sum(self._motor_rpms) * component for component in relative_velocity_body)
        p.applyExternalForce(drone, -1, drag_force, (0, 0, 0), p.LINK_FRAME)
        return drag_force
