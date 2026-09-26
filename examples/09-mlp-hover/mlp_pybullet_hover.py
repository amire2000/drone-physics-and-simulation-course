"""Hover at a fixed altitude in PyBullet using a NumPy MLP or its PD teacher."""

import argparse
from dataclasses import dataclass, field
from pathlib import Path
import sys
import time
from typing import Callable

import matplotlib.pyplot as plt
import pybullet as p

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.drone_model import DEFAULT_DRONE_MODEL, DEFAULT_PHYSICS_SETTINGS
from common.drone_physics import PhysicsEngine, clamp
from common.flight_control import AttitudeController
from common.pybullet_sensors import read_imu, read_state
from common.pybullet_utils import create_world, draw_force_vectors, state_text
from vertical_acceleration_mlp import VerticalAccelerationMlp, teacher_acceleration


MODEL = DEFAULT_DRONE_MODEL
SETTINGS = DEFAULT_PHYSICS_SETTINGS
TARGET_ALTITUDE_M = 3.0
SIMULATION_SECONDS = 15.0
START_ALTITUDE_M = 0.05


@dataclass
class HoverTelemetry:
    """Store the physical values needed to inspect a closed-loop hover."""

    time_s: list[float] = field(default_factory=list)
    altitude_m: list[float] = field(default_factory=list)
    vertical_velocity_mps: list[float] = field(default_factory=list)
    desired_acceleration_mps2: list[float] = field(default_factory=list)
    total_thrust_n: list[float] = field(default_factory=list)


def make_controller(name: str, model_path: Path) -> Callable[[float, float], float]:
    """Return either the transparent PD teacher or the saved MLP policy."""
    if name == "teacher":
        return lambda error_m, vertical_velocity_mps: float(teacher_acceleration(error_m, vertical_velocity_mps))
    if not model_path.is_file():
        raise FileNotFoundError(f"MLP model not found: {model_path}. Run vertical_acceleration_mlp.py first.")
    return VerticalAccelerationMlp.load(model_path).predict_acceleration


def run(controller: Callable[[float, float], float], target_altitude_m: float, seconds: float, gui: bool) -> HoverTelemetry:
    """Run one fixed-altitude flight through MLP acceleration, thrust, PWM, and physics."""
    drone = create_world()
    engine = PhysicsEngine()
    attitude_controller = AttitudeController()
    initial_yaw = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])[2]
    telemetry = HoverTelemetry()
    pwm_us, torque_nm = 1000.0, (0.0, 0.0, 0.0)
    desired_acceleration_mps2 = 0.0
    text_id, force_lines = -1, [-1, -1, -1, -1]

    for step in range(round(seconds / SETTINGS.time_step_s)):
        now_s = step * SETTINGS.time_step_s
        if step % SETTINGS.control_steps == 0:
            state = read_state(drone)
            error_m = target_altitude_m - state.position_m[2]
            desired_acceleration_mps2 = controller(error_m, state.linear_velocity_mps[2])
            assert abs(desired_acceleration_mps2) <= 4.0, "Controller command exceeded the documented acceleration limit"
            total_thrust_n = MODEL.mass_kg * (9.81 + desired_acceleration_mps2)
            pwm_us = engine.pwm_from_thrust(
                clamp(total_thrust_n / 4.0, 0.0, MODEL.max_thrust_per_motor_n)
            )
            torque_nm = attitude_controller.update(read_imu(drone), initial_yaw)

        flight_step = engine.step(drone, pwm_us, torque_nm)
        state = flight_step.state
        telemetry.time_s.append(now_s)
        telemetry.altitude_m.append(state.position_m[2])
        telemetry.vertical_velocity_mps.append(state.linear_velocity_mps[2])
        telemetry.desired_acceleration_mps2.append(desired_acceleration_mps2)
        telemetry.total_thrust_n.append(flight_step.total_thrust_n)

        if gui:
            draw_force_vectors(drone, flight_step, force_lines)
            text_id = p.addUserDebugText(
                f"MLP desired acceleration: {desired_acceleration_mps2:.2f} m/s²\n"
                f"Target altitude: {target_altitude_m:.2f} m\n{state_text(flight_step, True)}",
                (0.45, -0.45, 1.5),
                textColorRGB=(0.05, 0.05, 0.05),
                textSize=1.1,
                replaceItemUniqueId=text_id,
            )
            time.sleep(SETTINGS.time_step_s)
    return telemetry


def verify(telemetry: HoverTelemetry, target_altitude_m: float) -> None:
    """Assert the documented first-flight height, hold, and final-speed limits."""
    hold_samples = round(3.0 / SETTINGS.time_step_s)
    assert max(telemetry.altitude_m) < 3.5, "MLP hover exceeded the 3.5 m first-flight ceiling"
    assert all(abs(altitude_m - target_altitude_m) <= 0.15 for altitude_m in telemetry.altitude_m[-hold_samples:]), "MLP hover did not hold target altitude within ±0.15 m for the final 3 seconds"
    assert abs(telemetry.vertical_velocity_mps[-1]) <= 0.05, "MLP hover did not finish with near-zero vertical velocity"


def save_plot(telemetry: HoverTelemetry, target_altitude_m: float, controller_name: str, output_path: Path) -> None:
    """Save altitude, velocity, acceleration, and thrust traces from the PyBullet flight."""
    figure, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    axes[0].plot(telemetry.time_s, telemetry.altitude_m, color="#2563eb", label="altitude")
    axes[0].axhline(target_altitude_m, color="#16a34a", linestyle="--", label="target")
    axes[0].set(ylabel="altitude (m)", title=f"PyBullet vertical hover controlled by {controller_name}")
    axes[0].legend()
    axes[1].plot(telemetry.time_s, telemetry.vertical_velocity_mps, color="#ea580c")
    axes[1].axhline(0.0, color="#64748b", linewidth=1)
    axes[1].set_ylabel("vertical velocity (m/s)")
    axes[2].plot(telemetry.time_s, telemetry.desired_acceleration_mps2, color="#7c3aed")
    axes[2].axhline(0.0, color="#64748b", linewidth=1)
    axes[2].set_ylabel("desired acceleration (m/s²)")
    axes[3].plot(telemetry.time_s, telemetry.total_thrust_n, color="#0891b2")
    axes[3].axhline(MODEL.mass_kg * 9.81, color="#16a34a", linestyle="--", label="hover thrust")
    axes[3].set(xlabel="time (s)", ylabel="total thrust (N)")
    axes[3].legend()
    for axis in axes:
        axis.grid(alpha=0.3)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def main() -> None:
    """Run, verify, and plot a teacher or learned closed-loop PyBullet hover."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=("mlp", "teacher"), default="mlp")
    parser.add_argument("--model", type=Path, default=Path("outputs/vertical_acceleration_mlp.npz"))
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--seconds", type=float, default=SIMULATION_SECONDS)
    parser.add_argument("--target-altitude", type=float, default=TARGET_ALTITUDE_M)
    parser.add_argument("--output", type=Path, default=Path("outputs/mlp_pybullet_hover.png"))
    args = parser.parse_args()
    try:
        controller = make_controller(args.controller, args.model)
    except FileNotFoundError as error:
        parser.error(str(error))
    client = p.connect(p.DIRECT if args.headless else p.GUI)
    try:
        telemetry = run(controller, args.target_altitude, args.seconds, gui=not args.headless)
    finally:
        if p.isConnected(client):
            p.disconnect(client)
    verify(telemetry, args.target_altitude)
    save_plot(telemetry, args.target_altitude, args.controller, args.output)
    print(f"CONTROLLER: {args.controller}")
    print(f"PEAK ALTITUDE: {max(telemetry.altitude_m):.2f} m")
    print(f"FINAL ALTITUDE: {telemetry.altitude_m[-1]:.2f} m")
    print(f"FINAL VELOCITY: {telemetry.vertical_velocity_mps[-1]:.2f} m/s")
    print(f"PLOT SAVED: {args.output}")


if __name__ == "__main__":
    main()
