"""Open the course drone environment and inspect its initial resting state."""

import argparse
from pathlib import Path
import sys
import time

import pybullet as p

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.pybullet_sensors import read_state
from common.pybullet_utils import create_world, format_drone_state


def verify_resting_state(drone: int) -> None:
    """Assert that loading the environment has not advanced the drone yet."""
    state = read_state(drone)
    assert abs(state.position_m[2] - 0.05) < 1e-9, "The drone should spawn 5 cm above the plane"
    assert state.linear_velocity_mps == (0.0, 0.0, 0.0), "The unstepped drone should have no velocity"


def wait_for_exit() -> None:
    """Keep the GUI open until Q or Esc is pressed, without advancing physics."""
    while p.isConnected():
        keys = p.getKeyboardEvents()
        if any(key in (ord("q"), ord("Q"), 27) and state & p.KEY_WAS_TRIGGERED for key, state in keys.items()):
            return
        time.sleep(1 / 60)


def main() -> None:
    """Create the environment, print its state, and optionally wait for inspection."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Print the state without opening PyBullet's GUI")
    parser.add_argument("--self-check", action="store_true", help="Verify the initial resting state and exit")
    args = parser.parse_args()

    client = p.connect(p.DIRECT if args.headless or args.self_check else p.GUI)
    try:
        drone = create_world()
        print(format_drone_state(read_state(drone)))
        if args.headless or args.self_check:
            verify_resting_state(drone)
            if args.self_check:
                print("Environment initialization self-check passed")
            return
        p.resetDebugVisualizerCamera(cameraDistance=2.2, cameraYaw=45, cameraPitch=-25, cameraTargetPosition=(0, 0, 0.35))
        print("Inspect the drone and ground plane. Press Q or Esc to exit.")
        wait_for_exit()
    finally:
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
