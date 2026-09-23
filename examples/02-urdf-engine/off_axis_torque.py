"""Show that force location determines whether a drone rotates."""

import argparse
import time
from pathlib import Path

import pybullet as p

TIME_STEP = 1 / 240
STEPS = 240
FORCE_Z = 0.05
ARM_OFFSET = 0.12
URDF_PATH = Path(__file__).parent / "assets" / "drone_frame.urdf"


def run_trial(kind: str, show_gui: bool) -> float:
    p.resetSimulation()
    p.setGravity(0, 0, 0)
    p.setTimeStep(TIME_STEP)
    drone = p.loadURDF(str(URDF_PATH), (0, 0, 1), flags=p.URDF_USE_INERTIA_FROM_FILE)
    p.changeDynamics(drone, -1, linearDamping=0, angularDamping=0)

    for _ in range(STEPS):
        if kind == "center_force":
            p.applyExternalForce(drone, -1, (0, 0, FORCE_Z), (0, 0, 0), p.LINK_FRAME)
        elif kind == "off_axis_force":
            p.applyExternalForce(drone, -1, (0, 0, FORCE_Z), (ARM_OFFSET, ARM_OFFSET, 0), p.LINK_FRAME)
        else:
            p.applyExternalTorque(drone, -1, (ARM_OFFSET * FORCE_Z, -ARM_OFFSET * FORCE_Z, 0), p.LINK_FRAME)
        p.stepSimulation()
        if show_gui:
            time.sleep(TIME_STEP)

    roll, pitch, _ = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])
    return max(abs(roll), abs(pitch))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    client = p.connect(p.DIRECT if args.headless else p.GUI)
    try:
        angles = {kind: run_trial(kind, not args.headless) for kind in ("center_force", "off_axis_force", "direct_torque")}
        for kind, angle in angles.items():
            print(f"{kind:>14}: {angle:.3f} rad")
        if args.headless:
            assert angles["center_force"] < 0.001
            assert angles["off_axis_force"] > 0.1
            assert angles["direct_torque"] > 0.1
    finally:
        p.disconnect(client)


if __name__ == "__main__":
    main()
