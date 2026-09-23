"""Spawn one cube and watch gravity pull it onto a ground plane."""

import argparse
import time

import pybullet as p
import pybullet_data

GRAVITY_Z = -9.81
CUBE_START_HEIGHT = 3.0
TIME_STEP = 1 / 240
STEPS = 480


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    client = p.connect(p.DIRECT if args.headless else p.GUI)
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, GRAVITY_Z)
        p.setTimeStep(TIME_STEP)
        p.loadURDF("plane.urdf")
        cube = p.loadURDF("cube.urdf", (0, 0, CUBE_START_HEIGHT))
        p.changeVisualShape(cube, -1, rgbaColor=(0.15, 0.55, 1.0, 1.0))

        for _ in range(STEPS):
            p.stepSimulation()
            if not args.headless:
                time.sleep(TIME_STEP)

        final_height = p.getBasePositionAndOrientation(cube)[0][2]
        print(f"Cube: {CUBE_START_HEIGHT:.3f} m -> {final_height:.3f} m")
        if args.headless:
            assert final_height < CUBE_START_HEIGHT, "The cube should fall"
    finally:
        p.disconnect(client)


if __name__ == "__main__":
    main()
