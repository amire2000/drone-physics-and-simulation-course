"""Fly the Betaflight SITL bridge from PyBullet XYZ, yaw, and arm sliders."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import pybullet as p

MODULE04 = Path(__file__).resolve().parents[1] / "04-mix-control"
if str(MODULE04) not in sys.path:
    sys.path.insert(0, str(MODULE04))

from bridge.client import BetaflightBridge
from bridge.simulation import run_bridge_flight
from common.position import PositionLimits, TargetSliders
from common.simulation import load_vehicle
from position_bridge_demo import (
    MAX_THRUST_PER_MOTOR_N,
    MOTOR_TIME_CONSTANT_S,
    MOTOR_TIMEOUT_S,
    MOTOR_YAW_SIGNS,
    PHYSICS_HZ,
    RC_HZ,
    build_outer_pid,
)

POSITION_LIMITS = PositionLimits(((-5.0, 5.0), (-5.0, 5.0), (0.0, 5.0)), (2.0, 2.0, 1.5), (5.0, 5.0, 5.0))
ESCAPE_KEY = 27


class SliderCommandSource:
    """Read home-relative position, yaw-heading, and arm intent from PyBullet."""

    def __init__(self, *, auto_takeoff: bool) -> None:
        """Create native sliders, optionally preloaded with an armed 3 m target."""
        initial_target = (0.0, 0.0, 3.0) if auto_takeoff else (0.0, 0.0, 0.0)
        self._position = TargetSliders(POSITION_LIMITS, initial_target)
        self._yaw = p.addUserDebugParameter("Target yaw from home (rad)", -3.14159, 3.14159, 0.0)
        self._arm = p.addUserDebugParameter("Arm Betaflight", 0.0, 1.0, 1.0 if auto_takeoff else 0.0)

    def read(self, _: float) -> tuple[bool, tuple[float, float, float], float] | None:
        """Return the live slider command or stop when Escape is pressed."""
        events = p.getKeyboardEvents()
        if events.get(ESCAPE_KEY, 0) & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED):
            return None
        return p.readUserDebugParameter(self._arm) >= 0.5, self._position.read(), float(p.readUserDebugParameter(self._yaw))


def parse_args() -> argparse.Namespace:
    """Parse whether the GUI should arm and climb before accepting slider changes."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--auto-takeoff", action="store_true", help="Arm through the low-throttle gate and target 3 m at startup")
    return parser.parse_args()


def main() -> None:
    """Connect a slider-controlled outer PID to a separately running SITL process."""
    args = parse_args()
    client = p.connect(p.GUI)
    try:
        urdf_path = Path(__file__).resolve().parents[1] / "common/assets/full_drone.urdf"
        drone, vehicle = load_vehicle(urdf_path, MAX_THRUST_PER_MOTOR_N, MOTOR_TIME_CONSTANT_S, MOTOR_YAW_SIGNS)
        command_source = SliderCommandSource(auto_takeoff=args.auto_takeoff)
        with BetaflightBridge() as bridge:
            run_bridge_flight(
                drone,
                vehicle,
                bridge,
                build_outer_pid(vehicle.mass_kg),
                command_source.read,
                physics_hz=PHYSICS_HZ,
                rc_hz=RC_HZ,
                motor_timeout_s=MOTOR_TIMEOUT_S,
                real_time=True,
            )
    finally:
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
