"""Stabilize altitude and attitude with PID control and X-frame mixing."""

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
import time

import numpy as np
import pybullet as p

EXAMPLES_ROOT = Path(__file__).resolve().parents[2]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.cli import parse_args
from common.drone_model import DroneProfile, ImuReading
from common.drone_physics import PhysicsEngine, clamp
from common.flight_control import AttitudeController
from common.pid import PID
from common.pybullet_sensors import read_imu
from common.pybullet_utils import create_world, reset_drone
from common.runner import run_topic
from common.safety import FlightSafetyLimits, safety_reason
from common.simulation_utils import ReducedState, integrate_reduced_state
from common.telemetry import Sample

START_HEIGHT_M = 0.05
DEFAULT_TARGET_ALTITUDE_M = 1.0
DEFAULT_SECONDS = 4.0
CONTROL_HZ = 120
ALTITUDE_GAINS = (1.5, 0.05, 2.5)
TOPIC_ARGUMENTS = (
    (("--target-altitude",), {"type": float, "default": DEFAULT_TARGET_ALTITUDE_M, "help": "Target altitude in metres"}),
    (("--seconds",), {"type": float, "default": DEFAULT_SECONDS, "help": "Bounded duration for headless and reduced runs"}),
)


@dataclass
class ReducedAttitude:
    """Minimal roll and pitch state for the reduced controller example."""

    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    roll_rate_rad_s: float = 0.0
    pitch_rate_rad_s: float = 0.0


def create_reference_world(profile: DroneProfile) -> tuple[int, DroneProfile]:
    """Create the real-reference PyBullet world for the controller lesson."""
    return create_world(profile.model, profile.physics_settings), profile


# region Controller methods


# ! TOPIC 11 NEW CONTROL: altitude and attitude PID produce commands before physics integration.
def reset_controller_flight(drone: int, engine: PhysicsEngine, altitude_pid: PID, attitude_controller: AttitudeController) -> None:
    """Reset vehicle state and all controller history for a repeatable run."""
    reset_drone(drone, (0.0, 0.0, START_HEIGHT_M))
    engine.reset()
    altitude_pid.reset()
    attitude_controller.reset()


def controller_command(
    profile: DroneProfile,
    engine: PhysicsEngine,
    altitude_pid: PID,
    attitude_controller: AttitudeController,
    altitude_m: float,
    vertical_rate_mps: float,
    imu: ImuReading,
    target_altitude_m: float,
    bus_voltage_v: float,
) -> tuple[float, tuple[float, float, float], tuple[float, float, float]]:
    """Convert altitude and attitude errors into collective PWM and body torque."""
    # T = mg + Kp*e_h + Ki*integral(e_h) - Kd*v_z; body torque uses attitude error and rate.
    pid_terms = altitude_pid.update_terms(target_altitude_m - altitude_m, vertical_rate_mps, profile.physics_settings.time_step_s * profile.physics_settings.control_steps)
    collective_thrust_n = profile.model.mass_kg * abs(profile.physics_settings.gravity_z_mps2) + sum(pid_terms)
    collective_thrust_n = clamp(collective_thrust_n, 0.0, profile.model.max_thrust_per_motor_n * 4.0)
    voltage_scale = bus_voltage_v / profile.model.battery.nominal_voltage_v
    requested_motor_thrust_n = collective_thrust_n / 4.0 / max(voltage_scale**2, 1e-6)
    # The shared PhysicsEngine performs the X-frame mixer and motor limiting.
    pwm_us = engine.pwm_from_thrust(requested_motor_thrust_n)
    torque_body_nm = attitude_controller.update(imu, 0.0)
    return pwm_us, torque_body_nm, pid_terms


# endregion


# region Topic 11 simulation loops


def capture_sample(profile: DroneProfile, time_s: float, target_altitude_m: float, pwm_us: float, pid_terms: tuple[float, float, float], controller_output_n: float, flight_step) -> Sample:
    """Convert one shared physics-engine result into passive telemetry."""
    state = flight_step.state
    roll, pitch, yaw = p.getEulerFromQuaternion(state.orientation_quaternion)
    return Sample(
        time_s=time_s,
        altitude_m=state.position_m[2],
        vertical_velocity_mps=state.linear_velocity_mps[2],
        total_thrust_n=flight_step.total_thrust_n,
        motor_rpm_mean=float(np.mean(flight_step.motor_rpms)),
        battery_voltage_v=flight_step.bus_voltage_v,
        pwm_command_us=pwm_us,
        motor_kv_rpm_per_v=profile.model.motor_kv_rpm_per_v,
        nominal_battery_voltage_v=profile.model.battery.nominal_voltage_v,
        hover_thrust_target_n=profile.model.mass_kg * abs(profile.physics_settings.gravity_z_mps2),
        roll_rad=roll,
        pitch_rad=pitch,
        yaw_rad=yaw,
        roll_rate_rad_s=state.angular_velocity_body_rad_s[0],
        pitch_rate_rad_s=state.angular_velocity_body_rad_s[1],
        position_world_m=state.position_m,
        velocity_world_mps=state.linear_velocity_mps,
        orientation_quaternion=state.orientation_quaternion,
        angular_velocity_body_rad_s=state.angular_velocity_body_rad_s,
        motor_command_us=(pwm_us,) * 4,
        motor_rpm=flight_step.motor_rpms,
        motor_thrust_n=flight_step.motor_thrusts_n,
        battery_current_a=flight_step.delivered_current_a,
        battery_soc=flight_step.battery_state_of_charge,
        target_altitude_m=target_altitude_m,
        altitude_error_m=target_altitude_m - state.position_m[2],
        pid_p_n=pid_terms[0],
        pid_i_n=pid_terms[1],
        pid_d_n=pid_terms[2],
        controller_output_n=controller_output_n,
    )


def run_experiment(drone: int, profile: DroneProfile, args: argparse.Namespace, frames, controls=None) -> list[Sample]:
    """Run the stabilized PyBullet loop until close or a bounded test ends."""
    if args.seconds <= 0.0 or args.target_altitude <= 0.0:
        raise ValueError("seconds and target altitude must be positive")
    engine = PhysicsEngine(profile.model, profile.physics_settings)
    altitude_pid = PID(*ALTITUDE_GAINS, integral_limit=0.4)
    attitude_controller = AttitudeController()
    limits = FlightSafetyLimits()
    time_step = profile.physics_settings.time_step_s
    control_steps = max(1, profile.physics_settings.physics_hz // CONTROL_HZ)
    total_steps = None if controls is not None else round(args.seconds / time_step)
    reset_controller_flight(drone, engine, altitude_pid, attitude_controller)
    samples: list[Sample] = []
    pwm_us = engine.pwm_from_thrust(profile.model.mass_kg * abs(profile.physics_settings.gravity_z_mps2) / 4.0)
    torque = (0.0, 0.0, 0.0)
    pid_terms = (0.0, 0.0, 0.0)
    bus_voltage_v = profile.model.battery.nominal_voltage_v
    step = 0
    while total_steps is None or step < total_steps:
        if controls is not None:
            action = controls.poll()
            if action == "exit":
                return samples
            if action == "restart":
                reset_controller_flight(drone, engine, altitude_pid, attitude_controller)
                samples.clear()
                step = 0
                continue
            if action == "pause":
                time.sleep(1 / 60)
                continue
        # ! TOPIC 11 NEW CONTROL CALL: close altitude and attitude loops before the physics step.
        if step % control_steps == 0:
            imu = read_imu(drone)
            state = p.getBasePositionAndOrientation(drone)[0]
            velocity = p.getBaseVelocity(drone)[0]
            pwm_us, torque, pid_terms = controller_command(profile, engine, altitude_pid, attitude_controller, state[2], velocity[2], imu, args.target_altitude, bus_voltage_v)
        flight_step = engine.step(drone, pwm_us, torque)
        bus_voltage_v = flight_step.bus_voltage_v
        sample = capture_sample(profile, step * time_step, args.target_altitude, pwm_us, pid_terms, sum(pid_terms), flight_step)
        samples.append(sample)
        if controls is not None:
            reason = safety_reason(sample.altitude_m or 0.0, sample.roll_rad or 0.0, sample.pitch_rad or 0.0, sample.roll_rate_rad_s or 0.0, sample.pitch_rate_rad_s or 0.0, limits)
            if reason:
                controls.pause_with_reason(reason)
            time.sleep(time_step)
        step += 1
    return samples

def run_reduced_order(profile: DroneProfile, args: argparse.Namespace) -> list[Sample]:
    """Run altitude and attitude PID against a reduced-order vehicle state."""
    state = ReducedState(np.array((0.0, 0.0, START_HEIGHT_M)), np.zeros(3))
    attitude = ReducedAttitude()
    altitude_pid = PID(*ALTITUDE_GAINS, integral_limit=0.4)
    attitude_controller = AttitudeController()
    time_step = profile.physics_settings.time_step_s
    inertia = np.array(profile.model.inertia_kg_m2[:2])
    samples: list[Sample] = []
    for step in range(round(args.seconds / time_step)):
        imu = ImuReading((attitude.roll_rad, attitude.pitch_rad, 0.0), (attitude.roll_rate_rad_s, attitude.pitch_rate_rad_s, 0.0))
        pid_terms = altitude_pid.update_terms(args.target_altitude - state.position_m[2], state.velocity_mps[2], time_step)
        collective = clamp(profile.model.mass_kg * abs(profile.physics_settings.gravity_z_mps2) + sum(pid_terms), 0.0, profile.model.max_thrust_per_motor_n * 4.0)
        pwm_us = 1000.0 + 1000.0 * np.sqrt((collective / 4.0) / profile.model.max_thrust_per_motor_n)
        torque = attitude_controller.update(imu, 0.0)
        thrust = np.array((0.0, 0.0, collective))
        gravity = np.array((0.0, 0.0, profile.model.mass_kg * profile.physics_settings.gravity_z_mps2))
        acceleration = integrate_reduced_state(state, gravity + thrust, profile.model.mass_kg, time_step)
        angular_acceleration = np.array(torque[:2]) / inertia
        attitude.roll_rate_rad_s += angular_acceleration[0] * time_step
        attitude.pitch_rate_rad_s += angular_acceleration[1] * time_step
        attitude.roll_rad += attitude.roll_rate_rad_s * time_step
        attitude.pitch_rad += attitude.pitch_rate_rad_s * time_step
        motor_thrusts = (collective / 4.0,) * 4
        samples.append(Sample(
            time_s=step * time_step,
            altitude_m=float(state.position_m[2]),
            vertical_velocity_mps=float(state.velocity_mps[2]),
            vertical_acceleration_mps2=float(acceleration[2]),
            total_thrust_n=collective,
            pwm_command_us=pwm_us,
            battery_voltage_v=profile.model.battery.nominal_voltage_v,
            hover_thrust_target_n=profile.model.mass_kg * abs(profile.physics_settings.gravity_z_mps2),
            roll_rad=attitude.roll_rad,
            pitch_rad=attitude.pitch_rad,
            roll_rate_rad_s=attitude.roll_rate_rad_s,
            pitch_rate_rad_s=attitude.pitch_rate_rad_s,
            roll_torque_nm=torque[0],
            pitch_torque_nm=torque[1],
            target_altitude_m=args.target_altitude,
            altitude_error_m=args.target_altitude - state.position_m[2],
            pid_p_n=pid_terms[0], pid_i_n=pid_terms[1], pid_d_n=pid_terms[2], controller_output_n=sum(pid_terms),
            position_world_m=tuple(float(value) for value in state.position_m),
            velocity_world_mps=tuple(float(value) for value in state.velocity_mps),
            acceleration_world_mps2=tuple(float(value) for value in acceleration),
            motor_thrust_n=motor_thrusts,
        ))
    return samples


# endregion


def run_pybullet_topic(drone, profile, args, frames, controls):
    """Adapt the stabilized PyBullet loop to the common runner."""
    return run_experiment(drone, profile, args, frames, controls)


def run_reduced_topic(profile, args):
    """Adapt the stabilized reduced loop to the common runner."""
    return run_reduced_order(profile, args)


def validate_results(samples: list[Sample], profile: DroneProfile, args: argparse.Namespace) -> None:
    """Verify that PID reaches and holds the requested altitude."""
    assert samples
    assert max(sample.altitude_m or 0.0 for sample in samples) < args.target_altitude + 1.0
    assert abs((samples[-1].altitude_m or 0.0) - args.target_altitude) < 0.25
    assert abs(samples[-1].roll_rad or 0.0) < 0.2
    assert abs(samples[-1].pitch_rad or 0.0) < 0.2
    print("Altitude and attitude PID self-check passed")


def main() -> None:
    """Parse options and run the stabilized controller example."""
    args = parse_args(__doc__, TOPIC_ARGUMENTS)
    run_topic(
        args,
        create_world=create_reference_world,
        run_pybullet=run_pybullet_topic,
        run_reduced=run_reduced_topic,
        validate=validate_results,
        graph_fields=("target_altitude_m", "altitude_m", "pid_p_n", "pid_i_n", "pid_d_n", "controller_output_n", "roll_rad", "pitch_rad"),
        summary_title="Altitude and attitude PID summary",
        graph_title="Topic 11: altitude and attitude PID",
    )


if __name__ == "__main__":
    main()
