"""Module 04: compare cascaded PID and ADRC control during a 3 m hover."""

import argparse
from pathlib import Path

import pybullet as p

from adrc.controller import Config as ADRCConfig
from adrc.controller import Controller as ADRCController
from common.mixer import Mixer
from common.simulation import load_vehicle, run_flight
from pid.controller import Config as PIDConfig
from pid.controller import Controller as PIDController

TARGET_ALTITUDE_M = 3.0
TAKEOFF_SECONDS = 3.0
HOVER_SECONDS = 6.0
PHYSICS_HZ = 240
CONTROL_HZ = 120
MAX_THRUST_PER_MOTOR_N = 6.3765
MOTOR_TIME_CONSTANT_S = 0.05
MOTOR_YAW_SIGNS = (1, -1, -1, 1)
ALTITUDE_GAINS = (3.0, 0.35, 2.0)
ALTITUDE_INTEGRAL_LIMIT = 1.0
ALTITUDE_OUTPUT_LIMIT_N = 5.0
ATTITUDE_GAINS = (5.0, 0.0, 0.15)
ATTITUDE_INTEGRAL_LIMIT = 0.2
ATTITUDE_RATE_LIMIT_RAD_S = 3.0
RATE_GAINS = (0.018, 0.0, 0.001)
RATE_INTEGRAL_LIMIT = 0.5
TORQUE_LIMIT_NM = (0.08, 0.08, 0.04)
ADRC_ALTITUDE_B0 = 1.0
ADRC_ALTITUDE_CONTROL_BANDWIDTH = 1.5
ADRC_ALTITUDE_OBSERVER_BANDWIDTH = 6.0
ADRC_ALTITUDE_OUTPUT_LIMIT_MPS2 = 5.0
ADRC_ATTITUDE_B0 = 1.0
ADRC_ATTITUDE_CONTROL_BANDWIDTH = 1.5
ADRC_ATTITUDE_OBSERVER_BANDWIDTH = 6.0
ADRC_RATE_B0 = (1.0 / 0.00348, 1.0 / 0.00348, 1.0 / 0.00396)
ADRC_RATE_CONTROL_BANDWIDTH = 1.5
ADRC_RATE_OBSERVER_BANDWIDTH = 6.0
HYBRID_ADRC_ATTITUDE_CONTROL_BANDWIDTH = 4.5
HYBRID_ADRC_ATTITUDE_OBSERVER_BANDWIDTH = 18.0
HYBRID_ADRC_RATE_CONTROL_BANDWIDTH = 4.5
HYBRID_ADRC_RATE_OBSERVER_BANDWIDTH = 18.0


def build_controller(name: str):
    """Construct the requested controller while keeping the mixer unchanged."""
    if name == "pid":
        return PIDController(PIDConfig(
            ALTITUDE_GAINS, ALTITUDE_INTEGRAL_LIMIT, ALTITUDE_OUTPUT_LIMIT_N,
            ATTITUDE_GAINS, ATTITUDE_INTEGRAL_LIMIT, ATTITUDE_RATE_LIMIT_RAD_S,
            RATE_GAINS, RATE_INTEGRAL_LIMIT, TORQUE_LIMIT_NM,
            MAX_THRUST_PER_MOTOR_N * 4.0,
        ))
    return ADRCController(ADRCConfig(
        ADRC_ALTITUDE_B0, ADRC_ALTITUDE_CONTROL_BANDWIDTH, ADRC_ALTITUDE_OBSERVER_BANDWIDTH, ADRC_ALTITUDE_OUTPUT_LIMIT_MPS2,
        ADRC_ATTITUDE_B0, ADRC_ATTITUDE_CONTROL_BANDWIDTH, ADRC_ATTITUDE_OBSERVER_BANDWIDTH, ATTITUDE_RATE_LIMIT_RAD_S,
        ADRC_RATE_B0, ADRC_RATE_CONTROL_BANDWIDTH, ADRC_RATE_OBSERVER_BANDWIDTH, TORQUE_LIMIT_NM,
        MAX_THRUST_PER_MOTOR_N * 4.0,
    ))


def build_hybrid_controller():
    """Build the faster ADRC inner loop used by the PID/ADRC hybrid mode."""
    return ADRCController(ADRCConfig(
        ADRC_ALTITUDE_B0, ADRC_ALTITUDE_CONTROL_BANDWIDTH, ADRC_ALTITUDE_OBSERVER_BANDWIDTH, ADRC_ALTITUDE_OUTPUT_LIMIT_MPS2,
        ADRC_ATTITUDE_B0, HYBRID_ADRC_ATTITUDE_CONTROL_BANDWIDTH, HYBRID_ADRC_ATTITUDE_OBSERVER_BANDWIDTH, ATTITUDE_RATE_LIMIT_RAD_S,
        ADRC_RATE_B0, HYBRID_ADRC_RATE_CONTROL_BANDWIDTH, HYBRID_ADRC_RATE_OBSERVER_BANDWIDTH, TORQUE_LIMIT_NM,
        MAX_THRUST_PER_MOTOR_N * 4.0,
    ))


def parse_args() -> argparse.Namespace:
    """Parse the small set of runtime options for the standalone example."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Do not open the PyBullet GUI")
    parser.add_argument("--self-check", action="store_true", help="Run mixer and flight assertions")
    parser.add_argument("--controller", choices=("pid", "adrc"), default="pid", help="Select the flight controller implementation")
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
        controller = build_controller(args.controller)
        mixer = Mixer(vehicle.positions_m, vehicle.yaw_signs, vehicle.torque_coefficient / vehicle.thrust_coefficient, MAX_THRUST_PER_MOTOR_N)
        if args.self_check:
            check_mixer(mixer)
        samples = run_flight(drone, vehicle, controller, mixer, target_altitude_m=TARGET_ALTITUDE_M, takeoff_seconds=TAKEOFF_SECONDS, hover_seconds=HOVER_SECONDS, physics_hz=PHYSICS_HZ, control_hz=CONTROL_HZ, show_gui=not args.headless and not args.self_check)
        expected_steps = round((TAKEOFF_SECONDS + HOVER_SECONDS) * PHYSICS_HZ)
        if args.self_check:
            final = samples[-1]
            hover_samples = samples[-2 * PHYSICS_HZ:]
            assert abs(final.altitude_m - TARGET_ALTITUDE_M) < 0.15, final.altitude_m
            assert abs(final.attitude_rad[0]) < 0.2 and abs(final.attitude_rad[1]) < 0.2, final.attitude_rad
            assert max(max(abs(rate) for rate in sample.body_rates_rad_s) for sample in hover_samples) < 0.2
            print(f"Module 04 self-check passed: final altitude={final.altitude_m:.3f} m")
        elif not args.headless:
            banner = "SIMULATION COMPLETED" if len(samples) == expected_steps else "SIMULATION EXITED BY USER"
            print(f"\n{'=' * 52}\n  MODULE 04: {banner}\n{'=' * 52}")
            if p.isConnected():
                input("Press Enter to close PyBullet. ")
    finally:
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
