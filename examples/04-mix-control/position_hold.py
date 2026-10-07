"""Module 04 entry point: control position and velocity from home-relative targets."""

import argparse
from pathlib import Path

import pybullet as p

from adrc.position import Config as ADRCPositionConfig
from adrc.velocity import Config as ADRCVelocityConfig
from common.position import PositionLimits, TargetSliders
from common.mixer import Mixer
from common.simulation import load_vehicle, run_position_flight
from main import (
CONTROL_HZ,
    MAX_THRUST_PER_MOTOR_N,
    MOTOR_TIME_CONSTANT_S,
    MOTOR_YAW_SIGNS,
    PHYSICS_HZ,
    TARGET_ALTITUDE_M,
    TAKEOFF_SECONDS,
    HOVER_SECONDS,
    build_controller,
)
from pid.position import Config as PIDPositionConfig
from pid.velocity import Config as PIDVelocityConfig

POSITION_TARGET_DEFAULT_M = (0.0, 0.0, TARGET_ALTITUDE_M)
POSITION_TARGET_LIMITS = ((-5.0, 5.0), (-5.0, 5.0), (0.0, 5.0))
MAX_VELOCITY_MPS = (2.0, 2.0, 1.5)
MAX_ACCELERATION_MPS2 = (5.0, 5.0, 5.0)
MAX_TILT_RAD = 0.35
POSITION_CONTROL_HZ = 30
VELOCITY_CONTROL_HZ = 60
POSITION_PID_GAINS = ((0.8, 0.0, 0.0), (0.8, 0.0, 0.0), (0.8, 0.0, 0.0))
POSITION_PID_INTEGRAL_LIMIT = 1.0
VELOCITY_PID_GAINS = ((1.5, 0.0, 0.0), (1.5, 0.0, 0.0), (2.0, 0.0, 0.0))
VELOCITY_PID_INTEGRAL_LIMIT = 1.0
POSITION_ADRC_B0 = (1.0, 1.0, 1.0)
POSITION_ADRC_CONTROL_BANDWIDTH = (0.4, 0.4, 1.1)
POSITION_ADRC_OBSERVER_BANDWIDTH = (2.0, 2.0, 3.0)
POSITION_ADRC_DAMPING = 0.6
VELOCITY_ADRC_B0 = (1.0, 1.0, 1.0)
VELOCITY_ADRC_CONTROL_BANDWIDTH = (0.6, 0.6, 1.5)
VELOCITY_ADRC_OBSERVER_BANDWIDTH = (3.0, 3.0, 4.0)


def build_outer_loops(name: str):
    """Build matching position and velocity loops for the selected controller."""
    if name == "pid":
        return (
            PIDPositionConfig(POSITION_PID_GAINS, POSITION_PID_INTEGRAL_LIMIT, MAX_VELOCITY_MPS),
            PIDVelocityConfig(VELOCITY_PID_GAINS, VELOCITY_PID_INTEGRAL_LIMIT, MAX_ACCELERATION_MPS2),
        )
    return (
            ADRCPositionConfig(POSITION_ADRC_B0, POSITION_ADRC_CONTROL_BANDWIDTH, POSITION_ADRC_OBSERVER_BANDWIDTH, MAX_VELOCITY_MPS, POSITION_ADRC_DAMPING),
        ADRCVelocityConfig(VELOCITY_ADRC_B0, VELOCITY_ADRC_CONTROL_BANDWIDTH, VELOCITY_ADRC_OBSERVER_BANDWIDTH, MAX_ACCELERATION_MPS2),
    )


def create_outer_controllers(name: str):
    """Construct position and velocity controller implementations."""
    position_config, velocity_config = build_outer_loops(name)
    if name == "pid":
        from pid.position import Controller as PositionController
        from pid.velocity import Controller as VelocityController
    else:
        from adrc.position import Controller as PositionController
        from adrc.velocity import Controller as VelocityController
    return PositionController(position_config), VelocityController(velocity_config)


def parse_args() -> argparse.Namespace:
    """Parse controller selection and deterministic test options."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=("pid", "adrc"), default="pid", help="Select PID or ADRC")
    parser.add_argument("--headless", action="store_true", help="Run without opening the GUI")
    parser.add_argument("--self-check", action="store_true", help="Run the bounded position-control check")
    return parser.parse_args()


def main() -> None:
    """Hold a home-relative target position with selectable PID or ADRC loops."""
    args = parse_args()
    client = p.connect(p.DIRECT if args.headless or args.self_check else p.GUI)
    try:
        urdf_path = Path(__file__).resolve().parents[2] / "examples/common/assets/full_drone.urdf"
        drone, vehicle = load_vehicle(urdf_path, MAX_THRUST_PER_MOTOR_N, MOTOR_TIME_CONSTANT_S, MOTOR_YAW_SIGNS)
        home_position = tuple(p.getBasePositionAndOrientation(drone)[0])
        position_controller, velocity_controller = create_outer_controllers(args.controller)
        attitude_controller = build_controller(args.controller)
        mixer = Mixer(vehicle.positions_m, vehicle.yaw_signs, vehicle.torque_coefficient / vehicle.thrust_coefficient, MAX_THRUST_PER_MOTOR_N)
        target_source = None
        if not args.headless and not args.self_check:
            target_source = TargetSliders(PositionLimits(POSITION_TARGET_LIMITS, MAX_VELOCITY_MPS, MAX_ACCELERATION_MPS2), POSITION_TARGET_DEFAULT_M)
        target = (1.0, 0.0, TARGET_ALTITUDE_M) if args.self_check else POSITION_TARGET_DEFAULT_M
        samples = run_position_flight(drone, vehicle, position_controller, velocity_controller, attitude_controller, mixer, home_position_m=home_position, target_offset_m=target, target_source=target_source, takeoff_seconds=TAKEOFF_SECONDS, hover_seconds=HOVER_SECONDS, physics_hz=PHYSICS_HZ, control_hz=CONTROL_HZ, position_control_hz=POSITION_CONTROL_HZ, velocity_control_hz=VELOCITY_CONTROL_HZ, show_gui=not args.headless and not args.self_check, max_total_thrust_n=MAX_THRUST_PER_MOTOR_N * 4.0, max_tilt_rad=MAX_TILT_RAD)
        if args.self_check:
            final = samples[-1]
            assert abs(final.position_relative_m[0] - target[0]) < 0.35, final.position_relative_m
            assert abs(final.position_relative_m[2] - target[2]) < 0.2, final.position_relative_m
            assert max(abs(value) for value in final.velocity_world_mps) < 0.4, final.velocity_world_mps
            print(f"Position-control self-check passed: final position={tuple(round(value, 3) for value in final.position_relative_m)} m")
        elif not args.headless:
            print("\n====================================================\n  POSITION-HOLD SIMULATION EXITED\n====================================================")
    finally:
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
