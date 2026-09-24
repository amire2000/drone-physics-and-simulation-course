"""Visualize world and body frames for a level drone with yaw."""

import argparse
import math
import time
from pathlib import Path

import numpy as np
import pybullet as p

TIME_STEP = 1 / 240
YAW_DEGREES = -45
DRONE_POSITION = (1.0, -0.75, 1.0)
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
        drone = p.loadURDF(str(URDF_PATH), DRONE_POSITION, orientation, flags=p.URDF_USE_INERTIA_FROM_FILE)

        world_forward = (math.cos(yaw), math.sin(yaw), 0.0)
        print("Position:", p.getBasePositionAndOrientation(drone)[0])
        print("Linear velocity:", p.getBaseVelocity(drone)[0])
        print("Body +X in world frame:", tuple(round(value, 3) for value in world_forward))
        if args.headless:
            assert np.allclose(world_forward, (math.sqrt(0.5), -math.sqrt(0.5), 0), atol=1e-9)

        if not args.headless:
            world_origin = (0, 0, 0)
            draw_axis(world_origin, (0.5, 0, 0), (1, 0, 0))
            draw_axis(world_origin, (0, 0.5, 0), (0, 1, 0))
            draw_axis(world_origin, (0, 0, 0.5), (0, 0, 1))
            draw_axis(DRONE_POSITION, tuple(0.4 * value for value in world_forward), (1, 1, 0))
            p.addUserDebugText("World origin", (0, 0, 0.55), textColorRGB=(1, 1, 1), textSize=1.2)
            p.addUserDebugText("Drone body +X", (DRONE_POSITION[0], DRONE_POSITION[1], DRONE_POSITION[2] + 0.2), textColorRGB=(1, 1, 0), textSize=1.2)
            p.addUserDebugText("Press Q to exit", (0, 0, 1.2), textColorRGB=(1, 1, 1), textSize=1.3)
            print("GUI: press Q to exit.")
            while p.isConnected():
                keys = p.getKeyboardEvents()
                if any(key in (ord("q"), ord("Q")) and state & p.KEY_WAS_TRIGGERED for key, state in keys.items()):
                    break
                p.stepSimulation()
                time.sleep(TIME_STEP)
    finally:
        p.disconnect(client)


if __name__ == "__main__":
    main()
