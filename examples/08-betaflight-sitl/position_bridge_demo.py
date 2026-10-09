"""Run a Module 04 outer PID position scenario through Betaflight SITL."""

from __future__ import annotations

import argparse
from pathlib import Path
import struct
import sys

import pybullet as p

MODULE04 = Path(__file__).resolve().parents[1] / "04-mix-control"
if str(MODULE04) not in sys.path:
    sys.path.insert(0, str(MODULE04))

from bridge.client import BetaflightBridge
from bridge.control import OuterPidConfig, OuterPositionPid
from bridge.protocol import FDM_PACKET_BYTES, MOTOR_PACKET_BYTES, RC_PACKET_BYTES, pack_fdm, pack_rc, unpack_motor
from bridge.simulation import BETAFLIGHT_TO_URDF_ROTOR, PositionScenario, remap_betaflight_motors, run_position_scenario
from bridge.types import FlightObservation, PilotCommand
from common.simulation import load_vehicle

PHYSICS_HZ = 240
RC_HZ = 50
MAX_THRUST_PER_MOTOR_N = 6.3765
MOTOR_TIME_CONSTANT_S = 0.05
MOTOR_YAW_SIGNS = (1, -1, -1, 1)
MOTOR_TIMEOUT_S = 0.1


def build_outer_pid(mass_kg: float) -> OuterPositionPid:
    """Construct the Module 04 outer PID cascade and its RC mapping limits."""
    config = OuterPidConfig(
        position_gains=((0.8, 0.0, 0.0), (0.8, 0.0, 0.0), (1.2, 0.0, 0.0)),
        velocity_gains=((1.5, 0.0, 0.0), (1.5, 0.0, 0.0), (3.0, 0.0, 0.0)),
        velocity_limits_mps=(2.0, 2.0, 1.5),
        acceleration_limits_mps2=(5.0, 5.0, 5.0),
        position_integral_limit=1.0,
        velocity_integral_limit=1.0,
        max_tilt_rad=0.35,
        max_total_thrust_n=MAX_THRUST_PER_MOTOR_N * 4.0,
        hover_throttle=0.50,
        throttle_per_newton=0.05,
        max_angle_mode_tilt_rad=0.35,
        # ! Module 08 yaw hold: PID output is normalized RC, not rad/s.
        yaw_gains=(0.15, 0.0, 0.02),
        yaw_integral_limit=0.5,
        prearm_delay_s=1.0,
        arming_delay_s=1.0,
    )
    return OuterPositionPid(config, mass_kg=mass_kg)


def parse_args() -> argparse.Namespace:
    """Parse standalone demo options without managing the SITL process."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Run PyBullet without its GUI")
    parser.add_argument("--self-check", action="store_true", help="Check pure bridge packet and PID boundaries without SITL")
    return parser.parse_args()


def self_check() -> None:
    """Prove packet sizes, invalid output rejection, and the low-throttle arm gate."""
    packet = pack_rc(1.0, PilotCommand(0.0, 0.0, 0.0, 0.0, False, True))
    assert len(packet) == RC_PACKET_BYTES
    assert FDM_PACKET_BYTES == 144 and MOTOR_PACKET_BYTES == 16
    try:
        unpack_motor(b"", 0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("An empty motor packet must be rejected")
    controller = build_outer_pid(1.0)
    observation = FlightObservation(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), (1.0, 2.0, 3.0), (4.0, 5.0, 6.0), 101_325.0)
    arm_command = controller.update(observation, (0.0, 0.0, 3.0), 0.0, 1.0 / PHYSICS_HZ, armed=True)
    assert not arm_command.armed and arm_command.throttle == 0.0
    fdm_values = struct.unpack("<18d", pack_fdm(observation))
    assert fdm_values[1:4] == (1.0, -2.0, -3.0), fdm_values[1:4]
    assert fdm_values[4:7] == (-4.0, -5.0, -6.0), fdm_values[4:7]
    assert abs(fdm_values[7] - 2**-0.5) < 1e-12 and abs(fdm_values[10] - 2**-0.5) < 1e-12
    home_observation = FlightObservation(0.0, (1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 9.81), 100_000.0)
    sensor_values = struct.unpack("<18d", pack_fdm(home_observation))
    assert sensor_values[11:17] == (0.0,) * 6
    assert sensor_values[17] == 100_000.0
    assert BETAFLIGHT_TO_URDF_ROTOR == (3, 1, 2, 0)
    assert remap_betaflight_motors((0.1, 0.2, 0.3, 0.4)) == (0.4, 0.2, 0.3, 0.1)
    print("Betaflight bridge self-check passed")


def main() -> None:
    """Connect the standalone outer-PID PyBullet demo to an existing SITL process."""
    args = parse_args()
    if args.self_check:
        self_check()
        return
    client = p.connect(p.DIRECT if args.headless else p.GUI)
    try:
        urdf_path = Path(__file__).resolve().parents[1] / "common/assets/full_drone.urdf"
        drone, vehicle = load_vehicle(urdf_path, MAX_THRUST_PER_MOTOR_N, MOTOR_TIME_CONSTANT_S, MOTOR_YAW_SIGNS)
        controller = build_outer_pid(vehicle.mass_kg)
        with BetaflightBridge() as bridge:
            run_position_scenario(drone, vehicle, bridge, controller, PositionScenario(), physics_hz=PHYSICS_HZ, rc_hz=RC_HZ, motor_timeout_s=MOTOR_TIMEOUT_S, real_time=True)
    finally:
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
