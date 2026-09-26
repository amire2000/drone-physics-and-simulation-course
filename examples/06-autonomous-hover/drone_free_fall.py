"""Measure a motor-off drone's airborne free fall at the fixed 240 Hz timestep."""

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
import time

import pybullet as p

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.drone_model import DEFAULT_PHYSICS_SETTINGS
from common.gui_helper import SimulationControls, add_simulation_buttons
from common.pybullet_sensors import read_state
from common.pybullet_utils import create_world, reset_drone

SETTINGS = DEFAULT_PHYSICS_SETTINGS
TIME_STEP = SETTINGS.time_step_s
START_HEIGHT_M = 2.0
RUN_SECONDS = 0.4
STEPS = round(RUN_SECONDS / TIME_STEP)


@dataclass(frozen=True)
class FreeFallSummary:
    """Measured airborne state at the planned end of one free-fall run."""

    duration_s: float
    start_height_m: float
    end_height_m: float
    end_velocity_mps: float
    measured_acceleration_mps2: float


def reset_experiment(drone: int) -> None:
    """Return the motor-off drone to the 2 m, zero-velocity starting pose."""
    reset_drone(drone, (0.0, 0.0, START_HEIGHT_M))


def summarize(drone: int, duration_s: float) -> FreeFallSummary:
    """Calculate end velocity and average airborne acceleration from PyBullet state."""
    state = read_state(drone)
    return FreeFallSummary(
        duration_s=duration_s,
        start_height_m=START_HEIGHT_M,
        end_height_m=state.position_m[2],
        end_velocity_mps=state.linear_velocity_mps[2],
        measured_acceleration_mps2=state.linear_velocity_mps[2] / duration_s,
    )


def print_summary(summary: FreeFallSummary) -> None:
    """Print the experiment result with units and the expected Earth gravity."""
    expected = SETTINGS.gravity_z_mps2
    error = summary.measured_acceleration_mps2 - expected
    print(
        "\n========== FREE-FALL SUMMARY ==========\n"
        f"Duration: {summary.duration_s:.2f} s\n"
        f"Start altitude: {summary.start_height_m:.2f} m\n"
        f"End altitude: {summary.end_height_m:.2f} m\n"
        f"End vertical velocity: {summary.end_velocity_mps:.2f} m/s\n"
        f"Measured airborne acceleration: {summary.measured_acceleration_mps2:.2f} m/s²\n"
        f"Expected gravity: {expected:.2f} m/s²\n"
        f"Gravity error: {error:+.3f} m/s²\n"
        "========================================"
    )


def overlay_text(drone: int, elapsed_s: float, running: bool, text_id: int) -> int:
    """Show current free-fall state in the PyBullet viewport without advancing it."""
    state = read_state(drone)
    status = "running" if running else "paused"
    text = (
        f"Free fall: {status}\n"
        f"time: {elapsed_s:.3f} s / {RUN_SECONDS:.2f} s\n"
        f"altitude: {state.position_m[2]:.3f} m\n"
        f"vertical velocity: {state.linear_velocity_mps[2]:.3f} m/s"
    )
    return p.addUserDebugText(text, (0.45, -0.45, 1.7), textColorRGB=(0.05, 0.05, 0.05), textSize=1.2, replaceItemUniqueId=text_id)


def run_headless(drone: int) -> FreeFallSummary:
    """Advance the complete airborne run immediately and return its measurement."""
    reset_experiment(drone)
    for _ in range(STEPS):
        p.stepSimulation()
    return summarize(drone, STEPS * TIME_STEP)


def run_gui(drone: int) -> None:
    """Run the same free-fall experiment through Start, Stop, Reset, and Exit buttons."""
    import matplotlib.pyplot as plt

    plt.ion()
    figure = plt.figure("Free-fall controls", figsize=(6.5, 1.6))
    controls: SimulationControls = add_simulation_buttons(figure, y=0.38)
    plt.show(block=False)
    p.resetDebugVisualizerCamera(cameraDistance=3.0, cameraYaw=45, cameraPitch=-20, cameraTargetPosition=(0, 0, 1.0))

    elapsed_s, completed, text_id = 0.0, False, -1
    while p.isConnected() and not controls.exit_requested:
        figure.canvas.flush_events()
        if controls.consume_reset():
            reset_experiment(drone)
            elapsed_s, completed = 0.0, False
        if controls.running and not completed:
            p.stepSimulation()
            elapsed_s += TIME_STEP
            if elapsed_s >= RUN_SECONDS:
                controls.stop()
                completed = True
                print_summary(summarize(drone, elapsed_s))
        text_id = overlay_text(drone, elapsed_s, controls.running, text_id)
        time.sleep(TIME_STEP if controls.running else 1 / 60)
    plt.close(figure)


def self_check() -> None:
    """Verify that the complete measurement stays airborne and matches gravity."""
    drone = create_world()
    summary = run_headless(drone)
    assert summary.end_height_m > 1.0, "The 0.4 s measurement must end before ground contact"
    assert summary.end_velocity_mps < -3.8, "Gravity should create a downward end velocity"
    assert abs(summary.measured_acceleration_mps2 - SETTINGS.gravity_z_mps2) < 0.05, "Airborne acceleration should match configured gravity"
    print_summary(summary)
    print("Drone free-fall self-check passed")


def main() -> None:
    """Run the free-fall lesson in GUI, headless, or self-check mode."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Run the complete measurement without a GUI")
    parser.add_argument("--self-check", action="store_true", help="Check the expected airborne gravity result")
    args = parser.parse_args()

    client = p.connect(p.DIRECT if args.headless or args.self_check else p.GUI)
    try:
        if args.self_check:
            self_check()
            return
        drone = create_world()
        if args.headless:
            print_summary(run_headless(drone))
        else:
            reset_experiment(drone)
            run_gui(drone)
    finally:
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
