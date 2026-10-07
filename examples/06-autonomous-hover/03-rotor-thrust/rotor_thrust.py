"""Apply collective rotor thrust and observe the Topic 0 drone response."""

import argparse
from pathlib import Path
import sys
import time

import numpy as np
import pybullet as p

EXAMPLES_ROOT = Path(__file__).resolve().parents[2]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.drone_model import DroneProfile
from common.battery import BatteryModel, BatteryState
from common.cli import parse_args
from common.pybullet_sensors import read_state
from common.pybullet_utils import create_world, reset_drone
from common.runner import run_topic
from common.simulation_utils import ReducedState, advance_motor_rpm, capture_scene_frame, clamp, integrate_reduced_state, rpm_from_thrust, thrust_from_pwm
from common.telemetry import Sample
from common.tk_controls import TkSimulationControls

START_HEIGHT_M = 1.0
DEFAULT_SECONDS = 1.0
DEFAULT_PWM_US = 1400.0
GIF_FPS = 12
GROUND_STIFFNESS_N_PER_M = 2000.0
GROUND_DAMPING_N_S_PER_M = 50.0

TOPIC_ARGUMENTS = (
    (("--pwm",), {"type": float, "default": DEFAULT_PWM_US, "help": "Equal collective motor command in microseconds"}),
    (("--seconds",), {"type": float, "default": DEFAULT_SECONDS, "help": "Simulation duration in seconds"}),
)


def create_reference_world(profile: DroneProfile) -> tuple[int, DroneProfile]:
    """Create the cumulative PyBullet world from the shared profile."""
    return create_world(profile.model, profile.physics_settings), profile


# region Previous topic forces: gravity and contact


def gravity_force(profile: DroneProfile) -> np.ndarray:
    """Return the cumulative Topic 2 gravitational force in world coordinates."""
    # Copied cumulative method: F_g = m * g remains active before thrust.
    return np.array((0.0, 0.0, profile.model.mass_kg * profile.physics_settings.gravity_z_mps2))


def apply_gravity(drone: int, profile: DroneProfile) -> np.ndarray:
    """Apply the cumulative Topic 2 gravity force to a PyBullet body."""
    force_world_n = gravity_force(profile)
    # Keep gravity visible in this topic-owned loop instead of hiding it in setup.
    p.applyExternalForce(drone, -1, force_world_n, (0.0, 0.0, 0.0), p.WORLD_FRAME)
    return force_world_n


def ground_contact_force(state: ReducedState, profile: DroneProfile) -> np.ndarray:
    """Return the copied Topic 2 reduced-order ground-contact force."""
    # This spring-damper is only the reduced-order adapter; PyBullet resolves contacts natively.
    penetration_m = max(0.0, -state.position_m[2])
    normal_n = 0.0
    if penetration_m > 0.0:
        normal_n = max(0.0, GROUND_STIFFNESS_N_PER_M * penetration_m - GROUND_DAMPING_N_S_PER_M * state.velocity_mps[2])
    elif state.position_m[2] <= 0.0 and state.velocity_mps[2] <= 0.0:
        normal_n = profile.model.mass_kg * abs(profile.physics_settings.gravity_z_mps2)
    return np.array((0.0, 0.0, normal_n))


# endregion

# region Current topic force: rotor thrust


# =============================================================================
# ! TOPIC 3 NEW FORCE: COLLECTIVE ROTOR THRUST
# ! This method is the new behavior introduced by this lesson.
# =============================================================================
def apply_rotor_thrust(drone: int, profile: DroneProfile, motor_rpms: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """Apply four upward rotor forces using T_i = k_T RPM_i² and return thrusts in newtons."""
    # Topic 3 equation: T_i = k_T * RPM_i², where k_T is calibrated in N/RPM².
    # RPM is the actual lagged, voltage-scaled motor speed, not the raw PWM command.
    thrusts = tuple(profile.model.thrust_coefficient * rpm**2 for rpm in motor_rpms)
    for link_index, thrust_n in enumerate(thrusts):
        # LINK_FRAME makes +z the rotor's local thrust axis; the force is applied at that rotor link.
        # Reaction torque is intentionally left for Topic 5; this method adds force only.
        p.applyExternalForce(drone, link_index, (0.0, 0.0, thrust_n), (0.0, 0.0, 0.0), p.LINK_FRAME)
    # The caller records these four forces and then integrates after gravity is already applied.
    return thrusts


# endregion

# region Motor state and telemetry


def advance_motor_state(profile: DroneProfile, battery: BatteryModel, motor_rpms: tuple[float, float, float, float], pwm_us: float) -> tuple[tuple[float, float, float, float], BatteryState]:
    """Convert PWM through KV, voltage sag, and motor lag into actual rotor RPM."""
    # PWM requests thrust; the calibrated thrust coefficient converts that request to RPM.
    requested_thrust_n = thrust_from_pwm(pwm_us, profile.model.max_thrust_per_motor_n)
    nominal_rpm = rpm_from_thrust(requested_thrust_n, profile.model.max_thrust_per_motor_n, profile.model.thrust_coefficient)
    command_fraction = clamp(nominal_rpm / profile.model.max_rpm, 0.0, 1.0)
    battery_state = battery.step((command_fraction,) * 4, profile.physics_settings.time_step_s)
    voltage_scale = battery_state.bus_voltage_v / profile.model.battery.nominal_voltage_v
    target_rpm = nominal_rpm * voltage_scale * battery_state.motor_command_scale
    next_rpms = tuple(
        advance_motor_rpm(actual, target_rpm, profile.physics_settings.time_step_s, profile.model.motor_time_constant_s)
        for actual in motor_rpms
    )
    return next_rpms, battery_state


def capture_pybullet_sample(
    drone: int,
    profile: DroneProfile,
    step: int,
    pwm_us: float,
    motor_rpms: tuple[float, float, float, float],
    motor_thrusts: tuple[float, float, float, float],
    battery_state: BatteryState,
) -> Sample:
    """Build one Topic 3 telemetry sample from the measured PyBullet state."""
    state = read_state(drone)
    total_thrust_n = sum(motor_thrusts)
    return Sample(
        time_s=step * profile.physics_settings.time_step_s,
        altitude_m=state.position_m[2],
        vertical_velocity_mps=state.linear_velocity_mps[2],
        total_thrust_n=total_thrust_n,
        motor_rpm_mean=float(np.mean(motor_rpms)),
        battery_voltage_v=battery_state.bus_voltage_v,
        pwm_command_us=pwm_us,
        motor_kv_rpm_per_v=profile.model.motor_kv_rpm_per_v,
        nominal_battery_voltage_v=profile.model.battery.nominal_voltage_v,
        hover_thrust_target_n=profile.model.mass_kg * abs(profile.physics_settings.gravity_z_mps2),
        position_world_m=state.position_m,
        velocity_world_mps=state.linear_velocity_mps,
        orientation_quaternion=state.orientation_quaternion,
        angular_velocity_body_rad_s=state.angular_velocity_body_rad_s,
        gravity_force_world_n=tuple(float(value) for value in gravity_force(profile)),
        thrust_force_world_n=(0.0, 0.0, total_thrust_n),
        motor_command_us=(pwm_us,) * 4,
        motor_rpm=motor_rpms,
        motor_thrust_n=motor_thrusts,
        battery_current_a=battery_state.delivered_current_a,
        battery_soc=battery_state.state_of_charge,
    )


def capture_reduced_sample(
    profile: DroneProfile,
    step: int,
    pwm_us: float,
    state: ReducedState,
    acceleration: np.ndarray,
    gravity: np.ndarray,
    contact: np.ndarray,
    thrust: np.ndarray,
    motor_rpms: tuple[float, float, float, float],
    motor_thrusts: tuple[float, float, float, float],
    battery_state: BatteryState,
) -> Sample:
    """Build one Topic 3 telemetry sample from the reduced-order state."""
    total_thrust_n = sum(motor_thrusts)
    return Sample(
        time_s=step * profile.physics_settings.time_step_s,
        altitude_m=float(state.position_m[2]),
        vertical_velocity_mps=float(state.velocity_mps[2]),
        total_thrust_n=total_thrust_n,
        motor_rpm_mean=float(np.mean(motor_rpms)),
        battery_voltage_v=battery_state.bus_voltage_v,
        pwm_command_us=pwm_us,
        motor_kv_rpm_per_v=profile.model.motor_kv_rpm_per_v,
        nominal_battery_voltage_v=profile.model.battery.nominal_voltage_v,
        hover_thrust_target_n=profile.model.mass_kg * abs(profile.physics_settings.gravity_z_mps2),
        position_world_m=tuple(float(value) for value in state.position_m),
        velocity_world_mps=tuple(float(value) for value in state.velocity_mps),
        acceleration_world_mps2=tuple(float(value) for value in acceleration),
        gravity_force_world_n=tuple(float(value) for value in gravity),
        contact_force_world_n=tuple(float(value) for value in contact),
        thrust_force_world_n=tuple(float(value) for value in thrust),
        net_force_world_n=tuple(float(value) for value in gravity + contact + thrust),
        motor_command_us=(pwm_us,) * 4,
        motor_rpm=motor_rpms,
        motor_thrust_n=motor_thrusts,
        battery_current_a=battery_state.delivered_current_a,
        battery_soc=battery_state.state_of_charge,
    )


# endregion

# region Topic 3 simulation loops


def run_experiment(
    drone: int,
    profile: DroneProfile,
    pwm_us: float,
    seconds: float,
    frames: list[np.ndarray] | None = None,
    realtime: bool = False,
    controls: TkSimulationControls | None = None,
) -> list[Sample]:
    """Run the cumulative Topic 3 loop through PyBullet."""
    if seconds <= 0.0:
        raise ValueError("seconds must be positive")
    p.setGravity(0.0, 0.0, 0.0)
    time_step = profile.physics_settings.time_step_s
    total_steps = None if controls is not None else round(seconds / time_step)
    while True:
        reset_drone(drone, (0.0, 0.0, START_HEIGHT_M))
        if frames is not None:
            frames.clear()
        battery = BatteryModel(profile.model.battery)
        motor_rpms = (0.0, 0.0, 0.0, 0.0)
        samples: list[Sample] = []
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
            motor_rpms, battery_state = advance_motor_state(profile, battery, motor_rpms, pwm_us)
            apply_gravity(drone, profile)
            # =============================================================================
            # ! TOPIC 3 NEW FORCE CALL: rotor thrust follows previous-topic gravity.
            # ! This is the lesson's new force contribution before integration.
            # =============================================================================
            motor_thrusts = apply_rotor_thrust(drone, profile, motor_rpms)
            # Topic 3 closes the cumulative force loop with one PyBullet integration step.
            p.stepSimulation()
            samples.append(capture_pybullet_sample(drone, profile, step, pwm_us, motor_rpms, motor_thrusts, battery_state))
            capture_scene_frame(step, profile.physics_settings.physics_hz, frames, GIF_FPS)
            if realtime:
                time.sleep(time_step)
            step += 1
        if controls is None:
            return samples


# endregion


def run_reduced_order(profile: DroneProfile, pwm_us: float, seconds: float) -> list[Sample]:
    """Run the same gravity, contact, motor, and thrust equations as a point mass."""
    state = ReducedState(np.array((0.0, 0.0, START_HEIGHT_M)), np.zeros(3))
    battery = BatteryModel(profile.model.battery)
    motor_rpms = (0.0, 0.0, 0.0, 0.0)
    time_step = profile.physics_settings.time_step_s
    samples: list[Sample] = []
    for step in range(round(seconds / time_step)):
        motor_rpms, battery_state = advance_motor_state(profile, battery, motor_rpms, pwm_us)
        # =============================================================================
        # ! TOPIC 3 NEW FORCE EQUATION: the reduced loop mirrors apply_rotor_thrust.
        # ! T_i = k_T * RPM_i² is accumulated after gravity/contact and before integration.
        # =============================================================================
        thrusts = tuple(profile.model.thrust_coefficient * rpm**2 for rpm in motor_rpms)
        gravity = gravity_force(profile)
        contact = ground_contact_force(state, profile)
        thrust = np.array((0.0, 0.0, sum(thrusts)))
        integrate_reduced_state(state, gravity + contact + thrust, profile.model.mass_kg, time_step)
        state.position_m[2] = max(0.0, state.position_m[2])
        if state.position_m[2] == 0.0 and state.velocity_mps[2] < 0.0:
            state.velocity_mps[2] = 0.0
        samples.append(capture_reduced_sample(
            profile, step, pwm_us, state, (gravity + contact + thrust) / profile.model.mass_kg,
            gravity, contact, thrust, motor_rpms, thrusts, battery_state,
        ))
    return samples


def run_pybullet_topic(
    drone: int,
    profile: DroneProfile,
    args: argparse.Namespace,
    frames: list[np.ndarray] | None,
    controls: TkSimulationControls | None,
) -> list[Sample]:
    """Adapt the Topic 3 PyBullet loop to the common runner interface."""
    return run_experiment(
        drone,
        profile,
        args.pwm,
        args.seconds,
        frames,
        realtime=not (args.headless or args.self_check),
        controls=controls,
    )


def run_reduced_topic(profile: DroneProfile, args: argparse.Namespace) -> list[Sample]:
    """Adapt the Topic 3 reduced loop to the common runner interface."""
    return run_reduced_order(profile, args.pwm, args.seconds)


def validate_results(samples: list[Sample], profile: DroneProfile, args: argparse.Namespace) -> None:
    """Validate Topic 3 thrust, RPM, and upward response for the selected backend."""
    assert max(sample.total_thrust_n for sample in samples) > 0.0
    assert max(sample.motor_rpm_mean for sample in samples) > 0.0
    assert samples[-1].altitude_m > START_HEIGHT_M, "The reference collective command should climb"
    if args.backend == "pybullet":
        reduced_samples = run_reduced_order(profile, args.pwm, args.seconds)
        assert max(sample.total_thrust_n for sample in reduced_samples) > 0.0
        assert reduced_samples[-1].altitude_m > START_HEIGHT_M, "The reduced reference command should climb"
    print("Rotor thrust self-check passed")


def main() -> None:
    """Parse options and delegate to the example composition root."""
    args = parse_args(__doc__, TOPIC_ARGUMENTS)
    run_topic(
        args,
        create_world=create_reference_world,
        run_pybullet=run_pybullet_topic,
        run_reduced=run_reduced_topic,
        validate=validate_results,
        graph_fields=("altitude_m", "vertical_velocity_mps", "total_thrust_n", "motor_rpm_mean"),
        summary_title="Rotor thrust summary",
        graph_title="Topic 3: collective rotor thrust",
        gif_fps=GIF_FPS,
    )


if __name__ == "__main__":
    main()
