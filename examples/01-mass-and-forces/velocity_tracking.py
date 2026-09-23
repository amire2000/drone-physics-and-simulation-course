"""Estimate gravitational acceleration from a falling cube's velocity."""

import argparse
import time

import pybullet as p
import pybullet_data

GRAVITY_Z = -9.81
TIME_STEP = 1 / 240
STEPS = 120


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    client = p.connect(p.DIRECT if args.headless else p.GUI)
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, GRAVITY_Z)
        p.setTimeStep(TIME_STEP)
        cube = p.loadURDF("cube_small.urdf", (0, 0, 10.0))
        p.changeDynamics(cube, -1, linearDamping=0, angularDamping=0)

        initial_velocity = p.getBaseVelocity(cube)[0][2]
        for _ in range(STEPS):
            p.stepSimulation()
            if not args.headless:
                time.sleep(TIME_STEP)

        final_velocity = p.getBaseVelocity(cube)[0][2]
        elapsed = STEPS * TIME_STEP
        acceleration = (final_velocity - initial_velocity) / elapsed
        print(f"Vertical velocity: {initial_velocity:.3f} -> {final_velocity:.3f} m/s")
        print(f"Estimated acceleration: {acceleration:.3f} m/s²")
        if args.headless:
            assert abs(acceleration - GRAVITY_Z) < 0.1, "Expected Earth gravity"
    finally:
        p.disconnect(client)


if __name__ == "__main__":
    main()
