"""Measure gravity and ground contact for the Topic 0 reference drone."""

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
from common.cli import parse_args
from common.pybullet_sensors import read_state
from common.pybullet_utils import create_world, reset_drone
from common.runner import run_topic
from common.simulation_utils import ReducedState, capture_scene_frame, integrate_reduced_state
from common.telemetry import Sample
from common.tk_controls import TkSimulationControls

START_HEIGHT_M = 2.0
RUN_SECONDS = 1.0
GIF_FPS = 12
GROUND_STIFFNESS_N_PER_M = 2000.0
GROUND_DAMPING_N_S_PER_M = 50.0


def create_reference_world(profile: DroneProfile) -> tuple[int, DroneProfile]:
    """Create the cumulative PyBullet world from the shared profile."""
    return create_world(profile.model, profile.physics_settings), profile


def reset_experiment(drone: int) -> None:
    """Place the motor-off drone at the deterministic two-metre start height."""
    reset_drone(drone, (0.0, 0.0, START_HEIGHT_M))


def gravity_force(profile: DroneProfile) -> np.ndarray:
    """Return the Topic 2 gravitational force in world coordinates, in newtons."""
    # Topic 2 equation: F_g = m * g. The negative z sign points downward.
    return np.array((0.0, 0.0, profile.model.mass_kg * profile.physics_settings.gravity_z_mps2))


def apply_gravity(drone: int, profile: DroneProfile) -> np.ndarray:
    """Apply Topic 2 gravity to a PyBullet body and return the applied force."""
    force_world_n = gravity_force(profile)
    # Apply gravity explicitly so this topic's loop visibly owns the new force.
    p.applyExternalForce(drone, -1, force_world_n, (0.0, 0.0, 0.0), p.WORLD_FRAME)
    return force_world_n


def ground_contact_force(state: ReducedState, profile: DroneProfile) -> np.ndarray:
    """Return the reduced-order upward spring-damper contact force in newtons."""
    # The reduced model approximates PyBullet's contact solver only after z < 0.
    penetration_m = max(0.0, -state.position_m[2])
    normal_n = 0.0
    if penetration_m > 0.0:
        normal_n = max(0.0, GROUND_STIFFNESS_N_PER_M * penetration_m - GROUND_DAMPING_N_S_PER_M * state.velocity_mps[2])
    elif state.position_m[2] <= 0.0 and state.velocity_mps[2] <= 0.0:
        normal_n = profile.model.mass_kg * abs(profile.physics_settings.gravity_z_mps2)
    return np.array((0.0, 0.0, normal_n))


def contact_normal_force(drone: int) -> float:
    """Return the total normal force reported by current ground contacts."""
    return sum(point[9] for point in p.getContactPoints(bodyA=drone))


def capture_sample(drone: int, profile: DroneProfile, time_s: float, time_step: float, previous_velocity: float) -> Sample:
    """Read one state sample and estimate its vertical acceleration."""
    state = read_state(drone)
    velocity = state.linear_velocity_mps[2]
    acceleration = 0.0 if time_s == 0.0 else (velocity - previous_velocity) / time_step
    normal_force_n = contact_normal_force(drone)
    return Sample(
        time_s=time_s,
        altitude_m=state.position_m[2],
        vertical_velocity_mps=velocity,
        vertical_acceleration_mps2=acceleration,
        normal_force_n=normal_force_n,
        position_world_m=state.position_m,
        velocity_world_mps=state.linear_velocity_mps,
        orientation_quaternion=state.orientation_quaternion,
        angular_velocity_body_rad_s=state.angular_velocity_body_rad_s,
        gravity_force_world_n=tuple(float(value) for value in gravity_force(profile)),
        contact_force_world_n=(0.0, 0.0, normal_force_n),
    )


def measure_experiment(drone: int, profile: DroneProfile, frames: list[np.ndarray] | None = None) -> list[Sample]:
    """Run the cumulative Topic 2 loop with gravity and native PyBullet contact."""
    reset_experiment(drone)
    p.setGravity(0.0, 0.0, 0.0)
    time_step = profile.physics_settings.time_step_s
    samples: list[Sample] = []
    previous_velocity = 0.0
    for step in range(round(RUN_SECONDS / time_step) + 1):
        sample = capture_sample(drone, profile, step * time_step, time_step, previous_velocity)
        samples.append(sample)
        previous_velocity = sample.vertical_velocity_mps
        capture_scene_frame(step, profile.physics_settings.physics_hz, frames, GIF_FPS)
        if step < round(RUN_SECONDS / time_step):
            apply_gravity(drone, profile)
            p.stepSimulation()
    return samples


def run_reduced_order(profile: DroneProfile) -> list[Sample]:
    """Run the same Topic 2 force equations with a reduced-order point mass."""
    state = ReducedState(np.array((0.0, 0.0, START_HEIGHT_M)), np.zeros(3))
    time_step = profile.physics_settings.time_step_s
    samples: list[Sample] = []
    for step in range(round(RUN_SECONDS / time_step) + 1):
        gravity = gravity_force(profile)
        contact = ground_contact_force(state, profile)
        acceleration = integrate_reduced_state(state, gravity + contact, profile.model.mass_kg, time_step)
        state.position_m[2] = max(0.0, state.position_m[2])
        if state.position_m[2] == 0.0 and state.velocity_mps[2] < 0.0:
            state.velocity_mps[2] = 0.0
        samples.append(Sample(
            time_s=step * time_step,
            altitude_m=float(state.position_m[2]),
            vertical_velocity_mps=float(state.velocity_mps[2]),
            vertical_acceleration_mps2=float(acceleration[2]),
            normal_force_n=float(contact[2]),
            position_world_m=tuple(float(value) for value in state.position_m),
            velocity_world_mps=tuple(float(value) for value in state.velocity_mps),
            acceleration_world_mps2=tuple(float(value) for value in acceleration),
            gravity_force_world_n=tuple(float(value) for value in gravity),
            contact_force_world_n=tuple(float(value) for value in contact),
            net_force_world_n=tuple(float(value) for value in gravity + contact),
        ))
    return samples


def run_pybullet_topic(
    drone: int,
    profile: DroneProfile,
    args: argparse.Namespace,
    frames: list[np.ndarray] | None,
    controls: TkSimulationControls | None,
) -> list[Sample]:
    """Select the Topic 2 rendered or headless PyBullet loop."""
    return measure_experiment(drone, profile, frames) if args.headless or args.self_check else run_gui(drone, profile, frames, controls)


def run_reduced_topic(profile: DroneProfile, args: argparse.Namespace) -> list[Sample]:
    """Adapt the Topic 2 reduced loop to the common runner interface."""
    return run_reduced_order(profile)


def run_gui(
    drone: int,
    profile: DroneProfile,
    frames: list[np.ndarray] | None = None,
    controls: TkSimulationControls | None = None,
) -> list[Sample]:
    """Run the same experiment visibly and leave the final state for inspection."""
    p.setGravity(0.0, 0.0, 0.0)
    p.resetDebugVisualizerCamera(cameraDistance=3.0, cameraYaw=45, cameraPitch=-20, cameraTargetPosition=(0, 0, 1.0))
    time_step = profile.physics_settings.time_step_s
    total_steps = None if controls is not None else round(RUN_SECONDS / time_step)
    while True:
        reset_experiment(drone)
        if frames is not None:
            frames.clear()
        samples: list[Sample] = []
        previous_velocity = 0.0
        step = 0
        while total_steps is None or step <= total_steps:
            if controls is not None:
                action = controls.poll()
                if action == "exit":
                    return samples
                if action == "restart":
                    break
                if action == "pause":
                    time.sleep(1 / 60)
                    continue
            sample = capture_sample(drone, profile, step * time_step, time_step, previous_velocity)
            samples.append(sample)
            previous_velocity = sample.vertical_velocity_mps
            capture_scene_frame(step, profile.physics_settings.physics_hz, frames, GIF_FPS)
            p.addUserDebugText(
                f"gravity/contact  t={sample.time_s:.2f}s  z={sample.altitude_m:.2f}m  vz={sample.vertical_velocity_mps:.2f}m/s",
                (0.3, -0.4, 1.7), textSize=1.2,
            )
            if total_steps is None or step < total_steps:
                apply_gravity(drone, profile)
                p.stepSimulation()
                time.sleep(time_step)
            step += 1
        if controls is None:
            return samples


def validate_results(samples: list[Sample], profile: DroneProfile, args: argparse.Namespace) -> None:
    """Validate Topic 2 gravity and contact for the selected backend."""
    airborne = next(sample for sample in samples if sample.time_s >= 0.4)
    assert abs(airborne.vertical_acceleration_mps2 - profile.physics_settings.gravity_z_mps2) < (0.1 if args.backend == "reduced" else 0.05)
    assert any(sample.normal_force_n > 0.0 for sample in samples), "The run should reach ground contact"
    if args.backend == "pybullet":
        reduced_samples = run_reduced_order(profile)
        reduced_airborne = next(sample for sample in reduced_samples if sample.time_s >= 0.4)
        assert abs(reduced_airborne.vertical_acceleration_mps2 - profile.physics_settings.gravity_z_mps2) < 0.1
        assert any(sample.normal_force_n > 0.0 for sample in reduced_samples), "The reduced run should reach ground contact"
    print("Gravity and contact self-check passed")


def main() -> None:
    """Parse options and delegate to the example composition root."""
    args = parse_args(__doc__)
    run_topic(
        args,
        create_world=create_reference_world,
        run_pybullet=run_pybullet_topic,
        run_reduced=run_reduced_topic,
        validate=validate_results,
        graph_fields=("altitude_m", "vertical_velocity_mps", "vertical_acceleration_mps2", "normal_force_n"),
        summary_title="Gravity and contact summary",
        graph_title="Topic 2: gravity and ground contact",
        gif_fps=GIF_FPS,
    )


if __name__ == "__main__":
    main()
