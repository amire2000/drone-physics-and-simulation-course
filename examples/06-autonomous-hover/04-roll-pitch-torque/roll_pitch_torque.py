"""Apply roll and pitch torque from unequal rotor thrust."""

import argparse
from dataclasses import dataclass
from math import pi
from pathlib import Path
import sys
import time

import numpy as np
import pybullet as p

EXAMPLES_ROOT = Path(__file__).resolve().parents[2]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.battery import BatteryModel, BatteryState
from common.cli import parse_args
from common.drone_model import DroneProfile, DroneState, ImuReading
from common.flight_control import AttitudeController
from common.pid import PID
from common.pybullet_sensors import read_imu, read_state
from common.pybullet_utils import create_world, reset_drone
from common.runner import run_topic
from common.safety import FlightSafetyLimits, safety_reason
from common.simulation_utils import ReducedState, advance_motor_rpm, capture_scene_frame, clamp, integrate_reduced_state, rpm_from_thrust, thrust_from_pwm
from common.telemetry import Sample
from common.tk_controls import TkSimulationControls

START_HEIGHT_M = 1.0
DEFAULT_SECONDS = 1.0
DEFAULT_PWM_US = 1290.0
DEFAULT_ROLL_DELTA_N = 0.0005
DEFAULT_PITCH_DELTA_N = 0.0
DEFAULT_OSCILLATION_PERIOD_S = 0.8
ALTITUDE_GAINS = (1.5, 0.05, 2.5)
ATTITUDE_GAINS = (0.05, 0.0, 0.02)
MAX_CORRECTION_TORQUE_NM = 0.01
GIF_FPS = 12
GROUND_STIFFNESS_N_PER_M = 2000.0
GROUND_DAMPING_N_S_PER_M = 50.0

TOPIC_ARGUMENTS = (
    (("--pwm",), {"type": float, "default": DEFAULT_PWM_US, "help": "Equal collective motor command in microseconds"}),
    (("--seconds",), {"type": float, "default": DEFAULT_SECONDS, "help": "Simulation duration in seconds"}),
    (("--roll-delta",), {"type": float, "default": DEFAULT_ROLL_DELTA_N, "help": "Per-motor thrust imbalance for roll in newtons"}),
    (("--pitch-delta",), {"type": float, "default": DEFAULT_PITCH_DELTA_N, "help": "Per-motor thrust imbalance for pitch in newtons"}),
    (("--oscillation-period",), {"type": float, "default": DEFAULT_OSCILLATION_PERIOD_S, "help": "Seconds for the roll/pitch imbalance to complete one cycle"}),
)


@dataclass
class ReducedAttitudeState:
    """Reduced-order roll and pitch state integrated from body torque."""

    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    roll_rate_rad_s: float = 0.0
    pitch_rate_rad_s: float = 0.0


def create_reference_world(profile: DroneProfile) -> tuple[int, DroneProfile]:
    """Create the cumulative PyBullet world from the shared profile."""
    return create_world(profile.model, profile.physics_settings), profile


# region Previous topic forces: gravity, contact, and rotor thrust


def gravity_force(profile: DroneProfile) -> np.ndarray:
    """Return the cumulative gravitational force in world coordinates."""
    # F_g = m * g; the negative world-z sign points downward.
    return np.array((0.0, 0.0, profile.model.mass_kg * profile.physics_settings.gravity_z_mps2))


def apply_gravity(drone: int, profile: DroneProfile) -> np.ndarray:
    """Apply the copied Topic 2 gravity force to a PyBullet body."""
    force_world_n = gravity_force(profile)
    p.applyExternalForce(drone, -1, force_world_n, (0.0, 0.0, 0.0), p.WORLD_FRAME)
    return force_world_n


def ground_contact_force(state: ReducedState, profile: DroneProfile) -> np.ndarray:
    """Return the copied Topic 2 reduced-order ground-contact force."""
    penetration_m = max(0.0, -state.position_m[2])
    normal_n = 0.0
    if penetration_m > 0.0:
        normal_n = max(0.0, GROUND_STIFFNESS_N_PER_M * penetration_m - GROUND_DAMPING_N_S_PER_M * state.velocity_mps[2])
    elif state.position_m[2] <= 0.0 and state.velocity_mps[2] <= 0.0:
        normal_n = profile.model.mass_kg * abs(profile.physics_settings.gravity_z_mps2)
    return np.array((0.0, 0.0, normal_n))


def apply_rotor_thrust(drone: int, profile: DroneProfile, motor_rpms: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """Apply copied Topic 3 collective lift at the center of mass."""
    # T_i = k_T * RPM_i²; equal RPM values provide the collective lift baseline.
    thrusts = tuple(profile.model.thrust_coefficient * rpm**2 for rpm in motor_rpms)
    # Applying the sum at the center avoids numerical arm torque; Topic 4 owns attitude torque below.
    p.applyExternalForce(drone, -1, (0.0, 0.0, sum(thrusts)), (0.0, 0.0, 0.0), p.LINK_FRAME)
    return thrusts


# endregion

# region Current topic force: roll and pitch torque


def alternating_thrust_delta(amplitude_n: float, time_s: float, period_s: float) -> float:
    """Return a cosine thrust imbalance that alternates sign every half-cycle."""
    # A cosine starts with the requested positive sign and reverses every period/2.
    return amplitude_n * np.cos(2.0 * pi * time_s / period_s)


def torque_from_rotor_thrusts(profile: DroneProfile, rotor_thrusts_n: tuple[float, float, float, float]) -> np.ndarray:
    """Calculate body torque from rotor lever arms and upward thrust forces."""
    # Topic 4 equation: τ = r × F, with r in body meters and F along body +z.
    torque_body_nm = np.zeros(3)
    for position_m, thrust_n in zip(profile.model.rotor_positions_m, rotor_thrusts_n):
        torque_body_nm += np.cross(np.array(position_m), np.array((0.0, 0.0, thrust_n)))
    return torque_body_nm


# ! TOPIC 4 NEW FORCE: ROLL AND PITCH TORQUE
# ! This method is the new behavior introduced by unequal rotor thrust.
def apply_roll_pitch_torque(
    drone: int,
    profile: DroneProfile,
    collective_thrust_n: float,
    roll_delta_n: float,
    pitch_delta_n: float,
) -> tuple[tuple[float, float, float, float], np.ndarray]:
    """Apply τ = r × F for signed roll and pitch thrust differences."""
    # Positive roll_delta increases rotors with positive body-y; positive pitch_delta increases negative body-x rotors.
    rotor_thrusts_n = tuple(
        max(0.0, collective_thrust_n + roll_delta_n * np.sign(y) - pitch_delta_n * np.sign(x))
        for x, y, _ in profile.model.rotor_positions_m
    )
    torque_body_nm = torque_from_rotor_thrusts(profile, rotor_thrusts_n)
    # The lever arms are body-frame coordinates, so apply the resulting torque in the body frame.
    p.applyExternalTorque(drone, -1, tuple(float(value) for value in torque_body_nm), p.LINK_FRAME)
    return rotor_thrusts_n, torque_body_nm


# endregion

# region Motor state and telemetry


def advance_motor_state(profile: DroneProfile, battery: BatteryModel, motor_rpms: tuple[float, float, float, float], pwm_us: float) -> tuple[tuple[float, float, float, float], BatteryState]:
    """Convert PWM through KV, voltage sag, and motor lag into actual rotor RPM."""
    requested_thrust_n = thrust_from_pwm(pwm_us, profile.model.max_thrust_per_motor_n)
    nominal_rpm = rpm_from_thrust(requested_thrust_n, profile.model.max_thrust_per_motor_n, profile.model.thrust_coefficient)
    command_fraction = clamp(nominal_rpm / profile.model.max_rpm, 0.0, 1.0)
    battery_state = battery.step((command_fraction,) * 4, profile.physics_settings.time_step_s)
    target_rpm = nominal_rpm * battery_state.bus_voltage_v / profile.model.battery.nominal_voltage_v * battery_state.motor_command_scale
    next_rpms = tuple(advance_motor_rpm(actual, target_rpm, profile.physics_settings.time_step_s, profile.model.motor_time_constant_s) for actual in motor_rpms)
    return next_rpms, battery_state


def prime_motor_state(profile: DroneProfile, pwm_us: float) -> tuple[float, float, float, float]:
    """Start motors at the selected collective command so Topic 4 begins near hover."""
    requested_thrust_n = thrust_from_pwm(pwm_us, profile.model.max_thrust_per_motor_n)
    nominal_rpm = rpm_from_thrust(requested_thrust_n, profile.model.max_thrust_per_motor_n, profile.model.thrust_coefficient)
    full_pack_voltage_v = profile.model.battery.cell_count * profile.model.battery.cell_voltage_full_v
    primed_rpm = nominal_rpm * full_pack_voltage_v / profile.model.battery.nominal_voltage_v
    return (primed_rpm,) * 4


def capture_sample(
    profile: DroneProfile,
    step: int,
    pwm_us: float,
    motor_rpms: tuple[float, float, float, float],
    motor_thrusts: tuple[float, float, float, float],
    battery_state: BatteryState,
    torque_body_nm: np.ndarray,
    controller_torque_nm: tuple[float, float, float],
    pid_terms: tuple[float, float, float],
    target_altitude_m: float,
    state: DroneState | ReducedAttitudeState,
    reduced_state: ReducedState | None = None,
) -> Sample:
    """Build one shared telemetry sample for either backend."""
    total_thrust_n = sum(motor_thrusts)
    if reduced_state is None:
        pybullet_state = state
        roll, pitch, yaw = p.getEulerFromQuaternion(pybullet_state.orientation_quaternion)
        return Sample(
            time_s=step * profile.physics_settings.time_step_s,
            altitude_m=pybullet_state.position_m[2],
            vertical_velocity_mps=pybullet_state.linear_velocity_mps[2],
            total_thrust_n=total_thrust_n,
            motor_rpm_mean=float(np.mean(motor_rpms)),
            battery_voltage_v=battery_state.bus_voltage_v,
            pwm_command_us=pwm_us,
            motor_kv_rpm_per_v=profile.model.motor_kv_rpm_per_v,
            nominal_battery_voltage_v=profile.model.battery.nominal_voltage_v,
            hover_thrust_target_n=profile.model.mass_kg * abs(profile.physics_settings.gravity_z_mps2),
            roll_rad=roll,
            pitch_rad=pitch,
            yaw_rad=yaw,
            roll_rate_rad_s=pybullet_state.angular_velocity_body_rad_s[0],
            pitch_rate_rad_s=pybullet_state.angular_velocity_body_rad_s[1],
            roll_torque_nm=float(torque_body_nm[0]),
            pitch_torque_nm=float(torque_body_nm[1]),
            yaw_torque_nm=float(torque_body_nm[2]),
            target_altitude_m=target_altitude_m,
            altitude_error_m=target_altitude_m - pybullet_state.position_m[2],
            pid_p_n=pid_terms[0],
            pid_i_n=pid_terms[1],
            pid_d_n=pid_terms[2],
            controller_output_n=sum(pid_terms),
            disturbance_roll_torque_nm=float(torque_body_nm[0]),
            disturbance_pitch_torque_nm=float(torque_body_nm[1]),
            controller_roll_torque_nm=controller_torque_nm[0],
            controller_pitch_torque_nm=controller_torque_nm[1],
            position_world_m=pybullet_state.position_m,
            velocity_world_mps=pybullet_state.linear_velocity_mps,
            orientation_quaternion=pybullet_state.orientation_quaternion,
            angular_velocity_body_rad_s=pybullet_state.angular_velocity_body_rad_s,
            gravity_force_world_n=tuple(float(value) for value in gravity_force(profile)),
            thrust_force_world_n=(0.0, 0.0, total_thrust_n),
            motor_command_us=(pwm_us,) * 4,
            motor_rpm=motor_rpms,
            motor_thrust_n=motor_thrusts,
            battery_current_a=battery_state.delivered_current_a,
            battery_soc=battery_state.state_of_charge,
        )
    return Sample(
        time_s=step * profile.physics_settings.time_step_s,
        altitude_m=float(reduced_state.position_m[2]),
        vertical_velocity_mps=float(reduced_state.velocity_mps[2]),
        total_thrust_n=total_thrust_n,
        motor_rpm_mean=float(np.mean(motor_rpms)),
        battery_voltage_v=battery_state.bus_voltage_v,
        pwm_command_us=pwm_us,
        motor_kv_rpm_per_v=profile.model.motor_kv_rpm_per_v,
        nominal_battery_voltage_v=profile.model.battery.nominal_voltage_v,
        hover_thrust_target_n=profile.model.mass_kg * abs(profile.physics_settings.gravity_z_mps2),
        roll_rad=state.roll_rad,
        pitch_rad=state.pitch_rad,
        roll_rate_rad_s=state.roll_rate_rad_s,
        pitch_rate_rad_s=state.pitch_rate_rad_s,
        roll_torque_nm=float(torque_body_nm[0]),
        pitch_torque_nm=float(torque_body_nm[1]),
        yaw_torque_nm=float(torque_body_nm[2]),
        target_altitude_m=target_altitude_m,
        altitude_error_m=target_altitude_m - reduced_state.position_m[2],
        pid_p_n=pid_terms[0],
        pid_i_n=pid_terms[1],
        pid_d_n=pid_terms[2],
        controller_output_n=sum(pid_terms),
        disturbance_roll_torque_nm=float(torque_body_nm[0]),
        disturbance_pitch_torque_nm=float(torque_body_nm[1]),
        controller_roll_torque_nm=controller_torque_nm[0],
        controller_pitch_torque_nm=controller_torque_nm[1],
        position_world_m=tuple(float(value) for value in reduced_state.position_m),
        velocity_world_mps=tuple(float(value) for value in reduced_state.velocity_mps),
        motor_command_us=(pwm_us,) * 4,
        motor_rpm=motor_rpms,
        motor_thrust_n=motor_thrusts,
        battery_current_a=battery_state.delivered_current_a,
        battery_soc=battery_state.state_of_charge,
    )


# endregion

# region Topic 4 simulation loops


def controller_command(
    profile: DroneProfile,
    altitude_pid: PID,
    attitude_controller: AttitudeController,
    altitude_m: float,
    vertical_velocity_mps: float,
    imu,
    target_altitude_m: float,
    bus_voltage_v: float,
) -> tuple[float, tuple[float, float, float], tuple[float, float, float]]:
    """Return collective PWM, attitude correction, and altitude PID terms."""
    time_step = profile.physics_settings.time_step_s
    pid_terms = altitude_pid.update_terms(target_altitude_m - altitude_m, vertical_velocity_mps, time_step)
    hover_thrust_n = profile.model.mass_kg * abs(profile.physics_settings.gravity_z_mps2)
    collective_thrust_n = clamp(hover_thrust_n + sum(pid_terms), 0.0, profile.model.max_thrust_per_motor_n * 4.0)
    voltage_scale = bus_voltage_v / profile.model.battery.nominal_voltage_v
    motor_thrust_n = collective_thrust_n / 4.0 / max(voltage_scale**2, 1e-6)
    pwm_us = 1000.0 + 1000.0 * np.sqrt(motor_thrust_n / profile.model.max_thrust_per_motor_n)
    requested_torque_nm = attitude_controller.update(imu, 0.0)
    controller_torque_nm = tuple(clamp(value, -MAX_CORRECTION_TORQUE_NM, MAX_CORRECTION_TORQUE_NM) for value in requested_torque_nm)
    return pwm_us, controller_torque_nm, pid_terms


def create_attitude_controller() -> AttitudeController:
    """Create the deliberately modest Topic 4 roll and pitch stabilizer."""
    controller = AttitudeController(pitch_gains=ATTITUDE_GAINS)
    controller.roll_pid = PID(*ATTITUDE_GAINS)
    return controller


def disturbance_deltas(args: argparse.Namespace, step: int, time_step: float, controls: TkSimulationControls | None) -> tuple[float, float]:
    """Return the latched GUI disturbance or the deterministic headless disturbance."""
    if controls is not None:
        return (
            args.roll_delta if controls.action_active("roll_torque") else 0.0,
            args.pitch_delta if controls.action_active("pitch_torque") else 0.0,
        )
    return (
        alternating_thrust_delta(args.roll_delta, step * time_step, args.oscillation_period),
        alternating_thrust_delta(args.pitch_delta, step * time_step, args.oscillation_period),
    )


def run_experiment(
    drone: int,
    profile: DroneProfile,
    args: argparse.Namespace,
    frames: list[np.ndarray] | None,
    controls: TkSimulationControls | None = None,
) -> list[Sample]:
    """Run the cumulative Topic 4 PyBullet loop."""
    if args.seconds <= 0.0:
        raise ValueError("seconds must be positive")
    if args.oscillation_period <= 0.0:
        raise ValueError("oscillation period must be positive")
    p.setGravity(0.0, 0.0, 0.0)
    time_step = profile.physics_settings.time_step_s
    total_steps = None if controls is not None else round(args.seconds / time_step)
    safety_limits = FlightSafetyLimits()
    altitude_pid = PID(*ALTITUDE_GAINS, integral_limit=0.4)
    attitude_controller = create_attitude_controller()
    bus_voltage_v = profile.model.battery.cell_count * profile.model.battery.cell_voltage_full_v
    while True:
        reset_drone(drone, (0.0, 0.0, START_HEIGHT_M))
        if frames is not None:
            frames.clear()
        samples: list[Sample] = []
        battery = BatteryModel(profile.model.battery)
        motor_rpms = prime_motor_state(profile, args.pwm)
        altitude_pid.reset()
        attitude_controller.reset()
        if controls is not None:
            controls.clear_actions()
        step = 0
        while total_steps is None or step < total_steps:
            if controls is not None:
                action = controls.poll()
                if action == "exit":
                    return samples
                if action == "restart":
                    break
                if action == "pause":
                    time.sleep(1 / 60)
                    continue
            state = read_state(drone)
            pwm_us, controller_torque_nm, pid_terms = controller_command(
                profile,
                altitude_pid,
                attitude_controller,
                state.position_m[2],
                state.linear_velocity_mps[2],
                read_imu(drone),
                START_HEIGHT_M,
                bus_voltage_v,
            )
            motor_rpms, battery_state = advance_motor_state(profile, battery, motor_rpms, pwm_us)
            bus_voltage_v = battery_state.bus_voltage_v
            apply_gravity(drone, profile)
            motor_thrusts = apply_rotor_thrust(drone, profile, motor_rpms)
            roll_delta, pitch_delta = disturbance_deltas(args, step, time_step, controls)
            # ! TOPIC 4 NEW FORCE CALL: apply roll/pitch disturbance after previous forces and before correction.
            _, disturbance_torque_nm = apply_roll_pitch_torque(drone, profile, float(np.mean(motor_thrusts)), roll_delta, pitch_delta)
            # ! TOPIC 4 STABILIZATION CALL: apply requested PID attitude correction after the new disturbance.
            p.applyExternalTorque(drone, -1, controller_torque_nm, p.LINK_FRAME)
            p.stepSimulation()
            sample = capture_sample(profile, step, pwm_us, motor_rpms, motor_thrusts, battery_state, disturbance_torque_nm, controller_torque_nm, pid_terms, START_HEIGHT_M, read_state(drone))
            samples.append(sample)
            if controls is not None:
                reason = safety_reason(sample.altitude_m, sample.roll_rad or 0.0, sample.pitch_rad or 0.0, sample.roll_rate_rad_s or 0.0, sample.pitch_rate_rad_s or 0.0, safety_limits)
                if reason:
                    controls.pause_with_reason(reason)
            capture_scene_frame(step, profile.physics_settings.physics_hz, frames, GIF_FPS)
            if controls is not None:
                time.sleep(time_step)
            step += 1
        if controls is None:
            return samples


def run_reduced_order(profile: DroneProfile, args: argparse.Namespace) -> list[Sample]:
    """Run the same cumulative forces and rotational equation as a point mass."""
    if args.oscillation_period <= 0.0:
        raise ValueError("oscillation period must be positive")
    state = ReducedState(np.array((0.0, 0.0, START_HEIGHT_M)), np.zeros(3))
    attitude = ReducedAttitudeState()
    battery = BatteryModel(profile.model.battery)
    motor_rpms = prime_motor_state(profile, args.pwm)
    altitude_pid = PID(*ALTITUDE_GAINS, integral_limit=0.4)
    attitude_controller = create_attitude_controller()
    bus_voltage_v = profile.model.battery.cell_count * profile.model.battery.cell_voltage_full_v
    time_step = profile.physics_settings.time_step_s
    inertia = np.array(profile.model.inertia_kg_m2[:2])
    samples: list[Sample] = []
    for step in range(round(args.seconds / time_step)):
        pwm_us, controller_torque_nm, pid_terms = controller_command(
            profile,
            altitude_pid,
            attitude_controller,
            state.position_m[2],
            state.velocity_mps[2],
            ImuReading(
                (attitude.roll_rad, attitude.pitch_rad, 0.0),
                (attitude.roll_rate_rad_s, attitude.pitch_rate_rad_s, 0.0),
            ),
            START_HEIGHT_M,
            bus_voltage_v,
        )
        motor_rpms, battery_state = advance_motor_state(profile, battery, motor_rpms, pwm_us)
        bus_voltage_v = battery_state.bus_voltage_v
        motor_thrusts = tuple(profile.model.thrust_coefficient * rpm**2 for rpm in motor_rpms)
        gravity = gravity_force(profile)
        contact = ground_contact_force(state, profile)
        thrust = np.array((0.0, 0.0, sum(motor_thrusts)))
        roll_delta = alternating_thrust_delta(args.roll_delta, step * time_step, args.oscillation_period)
        pitch_delta = alternating_thrust_delta(args.pitch_delta, step * time_step, args.oscillation_period)
        _, disturbance_torque_nm = apply_roll_pitch_torque_reduced(profile, float(np.mean(motor_thrusts)), roll_delta, pitch_delta)
        torque_body_nm = disturbance_torque_nm + np.array(controller_torque_nm)
        acceleration = integrate_reduced_state(state, gravity + contact + thrust, profile.model.mass_kg, time_step)
        angular_acceleration = torque_body_nm[:2] / inertia
        attitude.roll_rate_rad_s += angular_acceleration[0] * time_step
        attitude.pitch_rate_rad_s += angular_acceleration[1] * time_step
        attitude.roll_rad += attitude.roll_rate_rad_s * time_step
        attitude.pitch_rad += attitude.pitch_rate_rad_s * time_step
        state.position_m[2] = max(0.0, state.position_m[2])
        if state.position_m[2] == 0.0 and state.velocity_mps[2] < 0.0:
            state.velocity_mps[2] = 0.0
        samples.append(capture_sample(profile, step, pwm_us, motor_rpms, motor_thrusts, battery_state, disturbance_torque_nm, controller_torque_nm, pid_terms, START_HEIGHT_M, attitude, state))
    return samples


def apply_roll_pitch_torque_reduced(profile: DroneProfile, collective_thrust_n: float, roll_delta_n: float, pitch_delta_n: float) -> tuple[tuple[float, float, float, float], np.ndarray]:
    """Calculate the Topic 4 torque without requiring a PyBullet connection."""
    rotor_thrusts_n = tuple(
        max(0.0, collective_thrust_n + roll_delta_n * np.sign(y) - pitch_delta_n * np.sign(x))
        for x, y, _ in profile.model.rotor_positions_m
    )
    return rotor_thrusts_n, torque_from_rotor_thrusts(profile, rotor_thrusts_n)


# endregion


def run_pybullet_topic(
    drone: int,
    profile: DroneProfile,
    args: argparse.Namespace,
    frames: list[np.ndarray] | None,
    controls: TkSimulationControls | None,
) -> list[Sample]:
    """Adapt the Topic 4 PyBullet loop to the common runner interface."""
    return run_experiment(drone, profile, args, frames, controls)


def run_reduced_topic(profile: DroneProfile, args: argparse.Namespace) -> list[Sample]:
    """Adapt the Topic 4 reduced loop to the common runner interface."""
    return run_reduced_order(profile, args)


def validate_results(samples: list[Sample], profile: DroneProfile, args: argparse.Namespace) -> None:
    """Validate that the commanded torque creates the expected angular response."""
    expected_sign = np.sign(args.roll_delta if args.roll_delta else args.pitch_delta)
    torque_field = "roll_torque_nm" if args.roll_delta else "pitch_torque_nm"
    rate_field = "roll_rate_rad_s" if args.roll_delta else "pitch_rate_rad_s"
    response = next(sample for sample in samples if abs(getattr(sample, rate_field)) > 1e-6)
    rate = getattr(response, rate_field)
    torque = getattr(response, torque_field)
    assert expected_sign != 0.0
    assert safety_reason(0.10, 0.0, 0.0, 0.0, 0.0) is not None
    assert safety_reason(1.0, np.deg2rad(50.0), 0.0, 0.0, 0.0) is not None
    assert np.sign(torque) == expected_sign, "Torque sign should follow the selected thrust imbalance"
    assert np.sign(rate) == expected_sign, "Angular-rate sign should follow the applied torque"
    assert abs(samples[-1].altitude_m - START_HEIGHT_M) < 0.15
    assert samples[-1].controller_roll_torque_nm is not None
    assert samples[-1].controller_pitch_torque_nm is not None
    if args.backend == "pybullet":
        reduced = run_reduced_order(profile, args)
        reduced_response = next(sample for sample in reduced if abs(getattr(sample, rate_field)) > 1e-6)
        assert np.sign(getattr(reduced_response, rate_field)) == expected_sign
    print("Roll and pitch torque self-check passed")


def main() -> None:
    """Parse options and delegate lifecycle orchestration to the common runner."""
    args = parse_args(__doc__, TOPIC_ARGUMENTS)
    run_topic(
        args,
        create_world=create_reference_world,
        run_pybullet=run_pybullet_topic,
        run_reduced=run_reduced_topic,
        validate=validate_results,
        graph_fields=("disturbance_roll_torque_nm", "controller_roll_torque_nm", "disturbance_pitch_torque_nm", "controller_pitch_torque_nm", "roll_rad", "pitch_rad", "altitude_m", "target_altitude_m"),
        graph_panels=(
            ("disturbance_roll_torque_nm", "controller_roll_torque_nm"),
            ("disturbance_pitch_torque_nm", "controller_pitch_torque_nm"),
            ("roll_rad", "pitch_rad"),
            ("altitude_m", "target_altitude_m"),
        ),
        summary_title="Roll and pitch torque summary",
        graph_title="Topic 4: roll and pitch torque",
        gif_fps=GIF_FPS,
        topic_actions=(("Roll torque", "roll_torque"), ("Pitch torque", "pitch_torque")),
    )


if __name__ == "__main__":
    main()
