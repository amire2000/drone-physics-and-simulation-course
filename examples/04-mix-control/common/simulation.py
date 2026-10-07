"""PyBullet world and motor/force integration shared by Module 04 controllers."""

from dataclasses import dataclass
from pathlib import Path
import time

import pybullet as p
import pybullet_data


@dataclass(frozen=True)
class Vehicle:
    """Vehicle values read from the shared URDF plus teaching actuator limits."""

    mass_kg: float
    max_thrust_per_motor_n: float
    motor_time_constant_s: float
    yaw_signs: tuple[int, int, int, int]
    positions_m: tuple[tuple[float, float], ...]
    thrust_coefficient: float
    torque_coefficient: float


@dataclass(frozen=True)
class FlightSample:
    """Small immutable state record used by the validation loop."""

    time_s: float
    altitude_m: float
    attitude_rad: tuple[float, float, float]
    body_rates_rad_s: tuple[float, float, float]
    motor_thrusts_n: tuple[float, ...]
    collective_thrust_n: float


def load_vehicle(urdf_path: Path, max_thrust_per_motor_n: float, motor_time_constant_s: float, yaw_signs: tuple[int, ...]) -> tuple[int, Vehicle]:
    """Create the world, load the shared URDF, and return its measured vehicle model."""
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0.0, 0.0, -9.81)
    p.setTimeStep(1.0 / 240.0)
    p.loadURDF("plane.urdf")
    drone = p.loadURDF(str(urdf_path), (0.0, 0.0, 0.05), flags=p.URDF_USE_INERTIA_FROM_FILE)
    p.changeDynamics(drone, -1, linearDamping=0.0, angularDamping=0.0)
    mass = p.getDynamicsInfo(drone, -1)[0]
    positions = tuple((p.getJointInfo(drone, index)[14][0], p.getJointInfo(drone, index)[14][1]) for index in range(4))
    max_rpm = 1621.6216216 * 14.8
    thrust_coefficient = max_thrust_per_motor_n / max_rpm**2
    return drone, Vehicle(mass, max_thrust_per_motor_n, motor_time_constant_s, yaw_signs, positions, thrust_coefficient, 0.015 * thrust_coefficient)


def read_state(drone: int) -> tuple[float, tuple[float, float, float], tuple[float, float, float]]:
    """Read altitude, Euler attitude, and angular velocity expressed in body axes."""
    position, quaternion = p.getBasePositionAndOrientation(drone)
    _, angular_velocity_world = p.getBaseVelocity(drone)
    rotation = p.getMatrixFromQuaternion(quaternion)
    body_rates = tuple(sum(rotation[3 * row + axis] * angular_velocity_world[row] for row in range(3)) for axis in range(3))
    return position[2], p.getEulerFromQuaternion(quaternion), body_rates


def apply_motor_forces(drone: int, vehicle: Vehicle, motor_thrusts_n: tuple[float, ...], motor_rpms: list[float]) -> None:
    """Apply rotor thrust and reaction yaw torque in the URDF body frame."""
    # Rotor force is +Z in the body frame; reaction torque follows each signed rotor direction.
    for index, (thrust, rpm, yaw_sign) in enumerate(zip(motor_thrusts_n, motor_rpms, vehicle.yaw_signs)):
        p.applyExternalForce(drone, index, (0.0, 0.0, thrust), (0.0, 0.0, 0.0), p.LINK_FRAME)
        p.applyExternalTorque(drone, -1, (0.0, 0.0, yaw_sign * vehicle.torque_coefficient * rpm**2), p.LINK_FRAME)


def run_flight(drone: int, vehicle: Vehicle, controller, mixer, *, target_altitude_m: float, takeoff_seconds: float, hover_seconds: float, physics_hz: int, control_hz: int, show_gui: bool) -> list[FlightSample]:
    """Run the takeoff and hover experiment with either injected controller."""
    dt = 1.0 / physics_hz
    control_steps = physics_hz // control_hz
    motor_rpms = [0.0] * 4
    samples: list[FlightSample] = []
    controller.reset()
    total_steps = None if show_gui else round((takeoff_seconds + hover_seconds) / dt)
    step = 0
    while total_steps is None or step < total_steps:
        if not p.isConnected():
            break
        altitude, attitude, body_rates = read_state(drone)
        if step % control_steps == 0:
            collective, torque, _ = controller.update(altitude, p.getBaseVelocity(drone)[0][2], attitude, body_rates, target_altitude_m, 1.0 / control_hz, vehicle.mass_kg, 9.81)
            motor_thrusts = mixer.mix(collective, torque)
        alpha = min(1.0, dt / vehicle.motor_time_constant_s)
        target_rpms = [(thrust / vehicle.thrust_coefficient) ** 0.5 for thrust in motor_thrusts]
        motor_rpms[:] = [actual + alpha * (target - actual) for actual, target in zip(motor_rpms, target_rpms)]
        actual_thrusts = tuple(vehicle.thrust_coefficient * rpm**2 for rpm in motor_rpms)
        apply_motor_forces(drone, vehicle, actual_thrusts, motor_rpms)
        p.stepSimulation()
        sample = FlightSample(step * dt, *read_state(drone), actual_thrusts, sum(actual_thrusts))
        samples.append(sample)
        if show_gui:
            time.sleep(dt)
        step += 1
    return samples
