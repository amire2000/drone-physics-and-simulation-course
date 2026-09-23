"""Load the course drone URDF and verify its mass and inertia."""

import argparse
import time
from pathlib import Path

import numpy as np
import pybullet as p

TIME_STEP = 1 / 240
STEPS = 480
URDF_PATH = Path(__file__).parent / "assets" / "drone_frame.urdf"
EXPECTED_MASS = 0.65
EXPECTED_INERTIA = np.array((0.00348, 0.00348, 0.00396))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    client = p.connect(p.DIRECT if args.headless else p.GUI)
    try:
        p.setGravity(0, 0, 0)
        p.setTimeStep(TIME_STEP)
        drone = p.loadURDF(str(URDF_PATH), (0, 0, 1), flags=p.URDF_USE_INERTIA_FROM_FILE)
        mass, _, inertia = p.getDynamicsInfo(drone, -1)[:3]
        print(f"Mass: {mass:.3f} kg")
        print("Inertia (Ixx, Iyy, Izz):", tuple(round(value, 5) for value in inertia))
        if args.headless:
            assert abs(mass - EXPECTED_MASS) < 1e-9
            assert np.allclose(inertia, EXPECTED_INERTIA, atol=1e-9)

        if not args.headless:
            for _ in range(STEPS):
                p.stepSimulation()
                time.sleep(TIME_STEP)
    finally:
        p.disconnect(client)


if __name__ == "__main__":
    main()
