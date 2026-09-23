"""Compare upward forces below, at, and above a cube's weight."""

import argparse
import time

import pybullet as p
import pybullet_data

GRAVITY_Z = -9.81
START_HEIGHT = 2.0
TIME_STEP = 1 / 240
STEPS = 240
TRIALS = {"under": 0.8, "hover": 1.0, "over": 1.2}


def run_trial(multiplier: float, show_gui: bool) -> float:
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, GRAVITY_Z)
    p.setTimeStep(TIME_STEP)
    cube = p.loadURDF("cube_small.urdf", (0, 0, START_HEIGHT))
    p.changeDynamics(cube, -1, linearDamping=0, angularDamping=0)
    mass = p.getDynamicsInfo(cube, -1)[0]
    upward_force = multiplier * mass * abs(GRAVITY_Z)

    for _ in range(STEPS):
        p.applyExternalForce(cube, -1, (0, 0, upward_force), (0, 0, 0), p.WORLD_FRAME)
        p.stepSimulation()
        if show_gui:
            time.sleep(TIME_STEP)

    return p.getBasePositionAndOrientation(cube)[0][2]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    client = p.connect(p.DIRECT if args.headless else p.GUI)
    try:
        heights = {name: run_trial(multiplier, not args.headless) for name, multiplier in TRIALS.items()}
        for name, height in heights.items():
            print(f"{name:>5} force: {height:.3f} m")

        if args.headless:
            assert heights["under"] < START_HEIGHT < heights["over"]
            assert abs(heights["hover"] - START_HEIGHT) < 0.001
    finally:
        p.disconnect(client)


if __name__ == "__main__":
    main()
