"""Inspect the racing-quad URDF adapted from the gym-pybullet-drones Racer."""

from pathlib import Path

import pybullet as p

URDF_PATH = Path(__file__).parent / "assets" / "racing_quad.urdf"


def main() -> None:
    client = p.connect(p.DIRECT)
    try:
        drone = p.loadURDF(str(URDF_PATH), flags=p.URDF_USE_INERTIA_FROM_FILE)
        mass, inertia = p.getDynamicsInfo(drone, -1)[0], p.getDynamicsInfo(drone, -1)[2]
        print(f"base mass: {mass:.2f} kg")
        print(f"base inertia: {tuple(round(value, 6) for value in inertia)} kg m^2")
        for index in range(p.getNumJoints(drone)):
            joint = p.getJointInfo(drone, index)
            position = p.getLinkState(drone, index)[0]
            print(f"{joint[12].decode()}: {tuple(round(value, 4) for value in position)} m")
        assert mass == 0.83 and p.getNumJoints(drone) == 4
    finally:
        p.disconnect(client)


if __name__ == "__main__":
    main()
