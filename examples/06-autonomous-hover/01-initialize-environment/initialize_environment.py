"""Open the course drone environment and inspect its initial resting state."""

import argparse
from pathlib import Path
import sys

import cv2
import numpy as np
import pybullet as p

EXAMPLES_ROOT = Path(__file__).resolve().parents[2]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.pybullet_sensors import read_state
from common.drone_model import load_drone_profile
from common.pybullet_utils import create_world, format_drone_state
from common.simulation_utils import wait_for_exit


def verify_resting_state(drone: int) -> None:
    """Assert that loading the environment has not advanced the drone yet."""
    state = read_state(drone)
    assert abs(state.position_m[2] - 0.05) < 1e-9, "The drone should spawn 5 cm above the plane"
    assert state.linear_velocity_mps == (0.0, 0.0, 0.0), "The unstepped drone should have no velocity"


def save_snapshot(path: Path) -> None:
    """Render the initial world from a deterministic camera and save a PNG."""
    view = p.computeViewMatrixFromYawPitchRoll((0.0, 0.0, 0.1), 0.8, 45.0, -30.0, 0.0, 2)
    projection = p.computeProjectionMatrixFOV(50.0, 4.0 / 3.0, 0.01, 10.0)
    _, _, rgba, _, _ = p.getCameraImage(800, 600, view, projection, renderer=p.ER_TINY_RENDERER)
    rgb = np.asarray(rgba, dtype=np.uint8).reshape(600, 800, 4)[:, :, :3]
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
        raise RuntimeError(f"could not write snapshot to {path}")


def parse_args() -> argparse.Namespace:
    """Parse command-line options for the environment inspection example."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Print the state without opening PyBullet's GUI")
    parser.add_argument("--self-check", action="store_true", help="Verify the initial resting state and exit")
    parser.add_argument("--snapshot", type=Path, help="Render the initial world to a PNG and exit")
    return parser.parse_args()


def create_reference_world() -> int:
    """Load the course drone profile and create its initial PyBullet world."""
    profile = load_drone_profile("real_reference")
    return create_world(profile.model, profile.physics_settings)


def print_initial_state(drone: int) -> None:
    """Print the initial state exposed by the shared sensor adapter."""
    print(format_drone_state(read_state(drone)))


def run_snapshot(drone: int, path: Path) -> None:
    """Validate the untouched world and save its deterministic camera view."""
    verify_resting_state(drone)
    save_snapshot(path)
    print(f"Saved environment snapshot: {path}")


def run_headless(drone: int, self_check: bool) -> None:
    """Validate the untouched world in a non-interactive run."""
    verify_resting_state(drone)
    if self_check:
        print("Environment initialization self-check passed")


def run_gui() -> None:
    """Show the initial world until the user presses Q or Esc."""
    p.resetDebugVisualizerCamera(
        cameraDistance=2.2,
        cameraYaw=45,
        cameraPitch=-25,
        cameraTargetPosition=(0, 0, 0.35),
    )
    print("Inspect the drone and ground plane. Press Q or Esc to exit.")
    wait_for_exit()


def run(args: argparse.Namespace) -> None:
    """Compose the world services and execute the requested inspection mode."""
    client = p.connect(p.DIRECT if args.headless or args.self_check or args.snapshot else p.GUI)

    try:
        drone = create_reference_world()
        print_initial_state(drone)
        if args.snapshot:
            run_snapshot(drone, args.snapshot)
        elif args.headless or args.self_check:
            run_headless(drone, args.self_check)
        else:
            run_gui()
    finally:
        if p.isConnected(client):
            p.disconnect(client)


def main() -> None:
    """Parse options and delegate execution to the example composition root."""
    run(parse_args())


if __name__ == "__main__":
    main()
