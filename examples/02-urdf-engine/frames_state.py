"""Visualize world and body frames for a level drone with yaw."""

import argparse
import math
import time
from pathlib import Path

import numpy as np
import pybullet as p

TIME_STEP = 1 / 240
STEPS = 480
YAW_DEGREES = 90
URDF_PATH = Path(__file__).parent / "assets" / "drone_frame.urdf"


def draw_axis(origin: tuple[float, float, float], axis: tuple[float, float, float], color: tuple[float, float, float]) -> None:
    end = tuple(start + direction for start, direction in zip(origin, axis))
    p.addUserDebugLine(origin, end, color, lineWidth=3)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    client = p.connect(p.DIRECT if args.headless else p.GUI)
    try:
        p.setGravity(0, 0, 0)
        p.setTimeStep(TIME_STEP)
        yaw = math.radians(YAW_DEGREES)
        orientation = p.getQuaternionFromEuler((0, 0, yaw))
        drone = p.loadURDF(str(URDF_PATH), (0, 0, 1), orientation, flags=p.URDF_USE_INERTIA_FROM_FILE)

        world_forward = (math.cos(yaw), math.sin(yaw), 0.0)
        print("Position:", p.getBasePositionAndOrientation(drone)[0])
        print("Linear velocity:", p.getBaseVelocity(drone)[0])
        print("Body +X in world frame:", tuple(round(value, 3) for value in world_forward))
        if args.headless:
            assert np.allclose(world_forward, (0, 1, 0), atol=1e-9)

        if not args.headless:
            origin = (0, 0, 1)
            draw_axis(origin, (0.4, 0, 0), (1, 0, 0))
            draw_axis(origin, (0, 0.4, 0), (0, 1, 0))
            draw_axis(origin, (0, 0, 0.4), (0, 0, 1))
            draw_axis(origin, tuple(0.32 * value for value in world_forward), (1, 1, 0))
            for _ in range(STEPS):
                p.stepSimulation()
                time.sleep(TIME_STEP)
    finally:
        p.disconnect(client)


if __name__ == "__main__":
    main()
