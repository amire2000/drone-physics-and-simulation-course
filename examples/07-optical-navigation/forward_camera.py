"""Show the RGB image from a forward-facing camera on the hovering drone."""

import argparse
from math import radians, tan
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import pybullet as p

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.drone_control import (
    CONTROL_STEPS,
    MASS,
    PHYSICS_HZ,
    TIME_STEP,
    attitude_torque,
    clamp,
    create_world,
    draw_force_vectors,
    make_controllers,
    pwm_from_thrust,
    step_drone,
)
from common.pid import PID

TARGET_ALTITUDE = 3.0
CAMERA_WIDTH, CAMERA_HEIGHT = 640, 480
CAMERA_HZ = 30


def add_red_cube() -> int:
    """Add a 2 m static visual target centered at world position (20, 0, 1)."""
    shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=(1, 1, 1))
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=(1, 1, 1), rgbaColor=(0.9, 0.05, 0.05, 1))
    return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape, baseVisualShapeIndex=visual, basePosition=(20, 0, 1))


def forward_rgb(
    drone: int,
    renderer: int,
    look_down_degrees: float = 0.0,
    mount_forward_m: float = 0.18,
    world_target: tuple[float, float, float] | None = None,
) -> np.ndarray:
    """Render a body camera, optionally with a stabilized optical axis."""
    position, orientation = p.getBasePositionAndOrientation(drone)
    eye, _ = p.multiplyTransforms(position, orientation, (mount_forward_m, 0.0, 0.03), (0, 0, 0, 1))
    target, _ = p.multiplyTransforms(position, orientation, (20.0, 0.0, 0.03 - 20.0 * tan(radians(look_down_degrees))), (0, 0, 0, 1))
    up_point, _ = p.multiplyTransforms(position, orientation, (0.0, 0.0, 1.0), (0, 0, 0, 1))
    up = tuple(axis - origin for axis, origin in zip(up_point, eye))
    view = p.computeViewMatrix(eye, world_target or target, (0, 0, 1) if world_target else up)
    projection = p.computeProjectionMatrixFOV(fov=60, aspect=CAMERA_WIDTH / CAMERA_HEIGHT, nearVal=0.05, farVal=50.0)
    _, _, rgba, _, _ = p.getCameraImage(CAMERA_WIDTH, CAMERA_HEIGHT, view, projection, renderer=renderer)
    return np.reshape(rgba, (CAMERA_HEIGHT, CAMERA_WIDTH, 4))[:, :, :3]


def run(gui: bool, max_seconds: float) -> None:
    drone = create_world()
    add_red_cube()
    altitude_pid = PID(kp=0.7, ki=0.05, kd=1.1, integral_limit=0.4)
    attitude_pids = make_controllers()
    motor_rpms = (0.0, 0.0, 0.0, 0.0)
    pwm = 1000.0
    torque = (0.0, 0.0, 0.0)
    force_lines = [-1, -1, -1, -1]
    renderer = p.ER_BULLET_HARDWARE_OPENGL if gui else p.ER_TINY_RENDERER

    for step in range(round(max_seconds / TIME_STEP)):
        position, _ = p.getBasePositionAndOrientation(drone)
        vertical_velocity = p.getBaseVelocity(drone)[0][2]
        if step % CONTROL_STEPS == 0:
            total_thrust = MASS * 9.81 + altitude_pid.update(TARGET_ALTITUDE - position[2], vertical_velocity)
            pwm = pwm_from_thrust(clamp(total_thrust / 4, 0.0, MASS * 9.81))
            torque = attitude_torque(drone, attitude_pids, yaw_target=0.0)
        motor_rpms, motor_thrusts, _ = step_drone(drone, pwm, torque, motor_rpms)

        if step % (PHYSICS_HZ // CAMERA_HZ) == 0:
            rgb = forward_rgb(drone, renderer)
            if gui:
                cv2.imshow("Drone forward RGB camera", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                    return

        if gui:
            draw_force_vectors(drone, motor_thrusts, force_lines)
            time.sleep(TIME_STEP)


def self_check() -> None:
    drone = create_world()
    add_red_cube()
    image = forward_rgb(drone, p.ER_TINY_RENDERER)
    red_pixels = (image[:, :, 0] > 120) & (image[:, :, 0] > image[:, :, 1] * 2) & (image[:, :, 0] > image[:, :, 2] * 2)
    assert image.shape == (CAMERA_HEIGHT, CAMERA_WIDTH, 3), "Forward camera must return an RGB image"
    assert red_pixels.any(), "Forward camera should see the red target cube"
    print("Forward camera self-check passed")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--self-check", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=30.0)
    args = parser.parse_args()
    client = p.connect(p.DIRECT if args.headless or args.self_check else p.GUI)
    try:
        if args.self_check:
            self_check()
        else:
            run(not args.headless, args.max_seconds)
    finally:
        cv2.destroyAllWindows()
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
