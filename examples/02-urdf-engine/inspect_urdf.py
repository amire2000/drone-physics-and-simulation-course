"""Inspect the physical and structural blocks in the editable anatomy URDF."""

from pathlib import Path

import pybullet as p

URDF_PATH = Path(__file__).parent / "assets" / "anatomy_drone.urdf"


def main() -> None:
    client = p.connect(p.DIRECT)
    try:
        drone = p.loadURDF(str(URDF_PATH), flags=p.URDF_USE_INERTIA_FROM_FILE)
        mass, inertia = p.getDynamicsInfo(drone, -1)[0], p.getDynamicsInfo(drone, -1)[2]
        joint = p.getJointInfo(drone, 0)
        print(f"base mass: {mass:.2f} kg")
        print(f"base inertia: {tuple(round(value, 6) for value in inertia)} kg m^2")
        print(f"joint: {joint[1].decode()} -> child link {joint[12].decode()}")
        assert mass == 0.65 and p.getNumJoints(drone) == 1
    finally:
        p.disconnect(client)


if __name__ == "__main__":
    main()
