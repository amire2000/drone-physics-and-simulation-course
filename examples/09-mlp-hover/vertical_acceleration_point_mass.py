"""Visualize a PD vertical-acceleration teacher with a simple 1-D point mass."""

import argparse
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import matplotlib.pyplot as plt
import numpy as np

from vertical_acceleration_mlp import VerticalAccelerationMlp, teacher_acceleration


TARGET_ALTITUDE_M = 3.0
START_ALTITUDE_M = 0.05
TIME_STEP_S = 0.02
SIMULATION_SECONDS = 12.0
MAX_ACCELERATION_MPS2 = 4.0


@dataclass
class PointMassState:
    """Store the simplified drone altitude and vertical velocity."""

    altitude_m: float = START_ALTITUDE_M
    vertical_velocity_mps: float = 0.0


@dataclass
class Telemetry:
    """Collect the values needed to inspect one vertical-control run."""

    time_s: list[float] = field(default_factory=list)
    altitude_m: list[float] = field(default_factory=list)
    vertical_velocity_mps: list[float] = field(default_factory=list)
    acceleration_mps2: list[float] = field(default_factory=list)


def teacher_acceleration_command(error_m: float, vertical_velocity_mps: float) -> float:
    """Return the bounded PD teacher acceleration for the current vertical state."""
    return float(teacher_acceleration(error_m, vertical_velocity_mps))


def make_mlp_acceleration_command(model_path: Path) -> Callable[[float, float], float]:
    """Load a trained MLP and expose it through the same controller function contract."""
    model = VerticalAccelerationMlp.load(model_path)
    return model.predict_acceleration


def integrate(state: PointMassState, acceleration_mps2: float, time_step_s: float) -> PointMassState:
    """Advance one point-mass state with semi-implicit Euler integration and ground contact."""
    velocity_mps = state.vertical_velocity_mps + acceleration_mps2 * time_step_s
    altitude_m = state.altitude_m + velocity_mps * time_step_s
    if altitude_m < 0.0:
        altitude_m, velocity_mps = 0.0, 0.0
    return PointMassState(altitude_m, velocity_mps)


def simulate(acceleration_command: Callable[[float, float], float], seconds: float) -> Telemetry:
    """Run the shared point-mass simulation using any acceleration-command function."""
    state = PointMassState()
    telemetry = Telemetry()
    for step in range(round(seconds / TIME_STEP_S)):
        now_s = step * TIME_STEP_S
        error_m = TARGET_ALTITUDE_M - state.altitude_m
        acceleration_mps2 = acceleration_command(error_m, state.vertical_velocity_mps)
        assert abs(acceleration_mps2) <= MAX_ACCELERATION_MPS2, "Controller command exceeded the documented acceleration limit"
        state = integrate(state, acceleration_mps2, TIME_STEP_S)
        assert state.altitude_m >= 0.0, "Ground contact must keep altitude non-negative"
        telemetry.time_s.append(now_s)
        telemetry.altitude_m.append(state.altitude_m)
        telemetry.vertical_velocity_mps.append(state.vertical_velocity_mps)
        telemetry.acceleration_mps2.append(acceleration_mps2)
    return telemetry


def save_plot(telemetry: Telemetry, output_path: Path, controller_name: str) -> None:
    """Save altitude, velocity, and desired-acceleration traces for one run."""
    figure, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(telemetry.time_s, telemetry.altitude_m, color="#2563eb", label="altitude")
    axes[0].axhline(TARGET_ALTITUDE_M, color="#16a34a", linestyle="--", label="target")
    axes[0].set_ylabel("altitude (m)")
    axes[0].set_title(f"1-D vertical point mass controlled by {controller_name}")
    axes[0].legend()

    axes[1].plot(telemetry.time_s, telemetry.vertical_velocity_mps, color="#ea580c")
    axes[1].axhline(0.0, color="#64748b", linewidth=1)
    axes[1].set_ylabel("vertical velocity (m/s)")

    axes[2].plot(telemetry.time_s, telemetry.acceleration_mps2, color="#7c3aed")
    axes[2].axhline(0.0, color="#64748b", linewidth=1)
    axes[2].set(xlabel="time (s)", ylabel="desired acceleration (m/s²)")
    axes[2].set_ylim(-MAX_ACCELERATION_MPS2 - 0.3, MAX_ACCELERATION_MPS2 + 0.3)
    for axis in axes:
        axis.grid(alpha=0.3)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def animate(telemetry: Telemetry) -> None:
    """Replay telemetry as a moving drone dot with live plots in one window."""
    figure = plt.figure("PD vertical acceleration: live replay", figsize=(11, 7))
    grid = figure.add_gridspec(3, 2, width_ratios=(1, 2))
    scene_axis = figure.add_subplot(grid[:, 0])
    altitude_axis = figure.add_subplot(grid[0, 1])
    velocity_axis = figure.add_subplot(grid[1, 1], sharex=altitude_axis)
    acceleration_axis = figure.add_subplot(grid[2, 1], sharex=altitude_axis)
    duration_s = telemetry.time_s[-1]

    scene_axis.axhline(0.0, color="#475569", linewidth=3)
    scene_axis.axhline(TARGET_ALTITUDE_M, color="#16a34a", linestyle="--", label="target")
    drone_dot, = scene_axis.plot([0.0], [START_ALTITUDE_M], "o", color="#2563eb", markersize=16, label="point mass")
    scene_axis.set(xlim=(-1, 1), ylim=(-0.2, max(4.2, max(telemetry.altitude_m) + 0.4)), xticks=[], ylabel="altitude (m)")
    scene_axis.legend(loc="upper right")

    altitude_line, = altitude_axis.plot([], [], color="#2563eb", label="altitude")
    altitude_axis.axhline(TARGET_ALTITUDE_M, color="#16a34a", linestyle="--", label="target")
    altitude_axis.set(ylabel="altitude (m)", title="Live controller replay")
    altitude_axis.legend(loc="upper right")
    velocity_line, = velocity_axis.plot([], [], color="#ea580c")
    velocity_axis.set(ylabel="velocity (m/s)")
    acceleration_line, = acceleration_axis.plot([], [], color="#7c3aed")
    acceleration_axis.set(xlabel="time (s)", ylabel="acceleration (m/s²)")
    for axis in (altitude_axis, velocity_axis, acceleration_axis):
        axis.set_xlim(0.0, duration_s)
        axis.grid(alpha=0.3)

    plt.show(block=False)
    for index in range(0, len(telemetry.time_s), 4):
        if not plt.fignum_exists(figure.number):
            return
        drone_dot.set_data([0.0], [telemetry.altitude_m[index]])
        altitude_line.set_data(telemetry.time_s[: index + 1], telemetry.altitude_m[: index + 1])
        velocity_line.set_data(telemetry.time_s[: index + 1], telemetry.vertical_velocity_mps[: index + 1])
        acceleration_line.set_data(telemetry.time_s[: index + 1], telemetry.acceleration_mps2[: index + 1])
        for axis in (altitude_axis, velocity_axis, acceleration_axis):
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)
        figure.canvas.draw_idle()
        plt.pause(0.02)
    plt.show()


def main() -> None:
    """Run the PD teacher simulation, save its plot, and optionally replay it live."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=("teacher", "mlp"), default="teacher", help="Use the visible PD teacher or trained MLP.")
    parser.add_argument("--model", type=Path, default=Path("outputs/vertical_acceleration_mlp.npz"), help="NPZ model used with --controller mlp.")
    parser.add_argument("--animate", action="store_true", help="Show a live Matplotlib replay after simulation.")
    parser.add_argument("--seconds", type=float, default=SIMULATION_SECONDS, help="Simulation duration in seconds.")
    parser.add_argument("--output", type=Path, default=Path("outputs/vertical_acceleration_point_mass.png"), help="PNG path for the final plot.")
    args = parser.parse_args()
    acceleration_command = teacher_acceleration_command if args.controller == "teacher" else make_mlp_acceleration_command(args.model)
    telemetry = simulate(acceleration_command, args.seconds)
    save_plot(telemetry, args.output, args.controller)
    print(f"CONTROLLER: {args.controller}")
    print(f"FINAL ALTITUDE: {telemetry.altitude_m[-1]:.2f} m")
    print(f"PEAK ALTITUDE: {max(telemetry.altitude_m):.2f} m")
    print(f"FINAL VELOCITY: {telemetry.vertical_velocity_mps[-1]:.2f} m/s")
    print(f"PLOT SAVED: {args.output}")
    if args.animate:
        animate(telemetry)


if __name__ == "__main__":
    main()
