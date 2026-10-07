"""Module 04 entry point: hold altitude while commanding roll, pitch, and yaw."""

import argparse
from pathlib import Path

import pybullet as p

from common.joystick import JoystickLimits, KeyboardJoystick, command_from_keys
from common.mixer import Mixer
from common.simulation import load_vehicle, run_flight
from main import (
    MAX_THRUST_PER_MOTOR_N,
    MOTOR_TIME_CONSTANT_S,
    MOTOR_YAW_SIGNS,
    CONTROL_HZ,
    HOVER_SECONDS,
    PHYSICS_HZ,
    TARGET_ALTITUDE_M,
    TAKEOFF_SECONDS,
    build_controller,
)

MAX_ROLL_COMMAND_RAD = 0.26
MAX_PITCH_COMMAND_RAD = 0.26
MAX_YAW_COMMAND_RAD = 0.52


def parse_args() -> argparse.Namespace:
    """Parse controller selection and deterministic test options."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=("pid", "adrc"), default="pid", help="Select PID or ADRC")
    parser.add_argument("--headless", action="store_true", help="Run without opening the GUI")
    parser.add_argument("--self-check", action="store_true", help="Run the bounded altitude-hold check")
    return parser.parse_args()


def check_joystick() -> None:
    """Check neutral and opposing virtual-stick commands."""
    limits = JoystickLimits(MAX_ROLL_COMMAND_RAD, MAX_PITCH_COMMAND_RAD, MAX_YAW_COMMAND_RAD)
    assert command_from_keys({}, limits) == (0.0, 0.0, 0.0)
    assert command_from_keys({ord("d"): 1}, limits) == (MAX_ROLL_COMMAND_RAD, 0.0, 0.0)
    assert command_from_keys({ord("q"): 1}, limits) == (0.0, 0.0, -MAX_YAW_COMMAND_RAD)


def main() -> None:
    """Hold 3 m while keyboard commands set desired attitude angles."""
    args = parse_args()
    client = p.connect(p.DIRECT if args.headless or args.self_check else p.GUI)
    try:
        urdf_path = Path(__file__).resolve().parents[2] / "examples/common/assets/full_drone.urdf"
        drone, vehicle = load_vehicle(urdf_path, MAX_THRUST_PER_MOTOR_N, MOTOR_TIME_CONSTANT_S, MOTOR_YAW_SIGNS)
        controller = build_controller(args.controller)
        mixer = Mixer(vehicle.positions_m, vehicle.yaw_signs, vehicle.torque_coefficient / vehicle.thrust_coefficient, MAX_THRUST_PER_MOTOR_N)
        joystick = None if args.headless or args.self_check else KeyboardJoystick(JoystickLimits(MAX_ROLL_COMMAND_RAD, MAX_PITCH_COMMAND_RAD, MAX_YAW_COMMAND_RAD))
        if args.self_check:
            check_joystick()
        samples = run_flight(drone, vehicle, controller, mixer, target_altitude_m=TARGET_ALTITUDE_M, takeoff_seconds=TAKEOFF_SECONDS, hover_seconds=HOVER_SECONDS, physics_hz=PHYSICS_HZ, control_hz=CONTROL_HZ, show_gui=not args.headless and not args.self_check, command_source=joystick)
        if args.self_check:
            assert abs(samples[-1].altitude_m - TARGET_ALTITUDE_M) < 0.15
            print(f"Attitude-hold self-check passed: final altitude={samples[-1].altitude_m:.3f} m")
        elif not args.headless:
            print("\n====================================================\n  ATTITUDE-HOLD SIMULATION EXITED\n====================================================")
    finally:
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
