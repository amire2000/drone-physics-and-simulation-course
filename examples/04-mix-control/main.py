"""Module 04: take off and hold 3 m with cascaded PID control."""

import argparse
from pathlib import Path

import pybullet as p

from controller import ControllerConfig, FlightController
from mixer import Mixer
from simulation import load_vehicle, run_flight

START_HEIGHT_M = 0.05
TARGET_ALTITUDE_M = 3.0
TAKEOFF_SECONDS = 3.0
HOVER_SECONDS = 6.0
PHYSICS_HZ = 240
CONTROL_HZ = 120
MAX_THRUST_PER_MOTOR_N = 6.3765
MOTOR_TIME_CONSTANT_S = 0.05
MOTOR_YAW_SIGNS = (1, -1, -1, 1)
TORQUE_TO_THRUST_M = 0.015
ALTITUDE_GAINS = (3.0, 0.35, 2.0)
ALTITUDE_INTEGRAL_LIMIT = 1.0
ALTITUDE_OUTPUT_LIMIT_N = 5.0
ATTITUDE_GAINS = (5.0, 0.0, 0.15)
ATTITUDE_INTEGRAL_LIMIT = 0.2
ATTITUDE_RATE_LIMIT_RAD_S = 3.0
RATE_GAINS = (0.018, 0.0, 0.001)
RATE_INTEGRAL_LIMIT = 0.5
TORQUE_LIMIT_NM = (0.08, 0.08, 0.04)


def parse_args() -> argparse.Namespace:
    """Parse the small set of runtime options for the standalone example."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Do not open the PyBullet GUI")
    parser.add_argument("--self-check", action="store_true", help="Run mixer and flight assertions")
    return parser.parse_args()


def check_mixer(mixer: Mixer) -> None:
    """Check collective symmetry and bounded torque allocation."""
    collective = mixer.mix(4.0, (0.0, 0.0, 0.0))
    assert max(collective) - min(collective) < 1e-9
    assert all(0.0 <= value <= MAX_THRUST_PER_MOTOR_N for value in mixer.mix(4.0, (0.2, 0.2, 0.02)))


def main() -> None:
    """Build the local controller and run the 3 m PyBullet experiment."""
    args = parse_args()
    client = p.connect(p.DIRECT if args.headless or args.self_check else p.GUI)
    try:
        urdf_path = Path(__file__).resolve().parents[2] / "examples/common/assets/full_drone.urdf"
        drone, vehicle = load_vehicle(urdf_path, MAX_THRUST_PER_MOTOR_N, MOTOR_TIME_CONSTANT_S, MOTOR_YAW_SIGNS)
        config = ControllerConfig(
            ALTITUDE_GAINS, ALTITUDE_INTEGRAL_LIMIT, ALTITUDE_OUTPUT_LIMIT_N,
            ATTITUDE_GAINS, ATTITUDE_INTEGRAL_LIMIT, ATTITUDE_RATE_LIMIT_RAD_S,
            RATE_GAINS, RATE_INTEGRAL_LIMIT, TORQUE_LIMIT_NM,
            MAX_THRUST_PER_MOTOR_N * 4.0,
        )
        controller = FlightController(config)
        mixer = Mixer(vehicle.positions_m, vehicle.yaw_signs, vehicle.torque_coefficient / vehicle.thrust_coefficient, MAX_THRUST_PER_MOTOR_N)
        if args.self_check:
            check_mixer(mixer)
        samples = run_flight(drone, vehicle, controller, mixer, target_altitude_m=TARGET_ALTITUDE_M, takeoff_seconds=TAKEOFF_SECONDS, hover_seconds=HOVER_SECONDS, physics_hz=PHYSICS_HZ, control_hz=CONTROL_HZ, show_gui=not args.headless and not args.self_check)
        if args.self_check:
            final = samples[-1]
            assert abs(final.altitude_m - TARGET_ALTITUDE_M) < 0.15, final.altitude_m
            assert abs(final.attitude_rad[0]) < 0.2 and abs(final.attitude_rad[1]) < 0.2, final.attitude_rad
            print(f"Module 04 self-check passed: final altitude={final.altitude_m:.3f} m")
        elif not args.headless:
            input("Flight complete. Press Enter to close PyBullet. ")
    finally:
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
