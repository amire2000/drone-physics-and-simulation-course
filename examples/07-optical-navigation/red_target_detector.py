"""Detect the red camera target with HSV thresholding and draw its bounding box."""

import argparse
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
from forward_camera import CAMERA_HZ, TARGET_ALTITUDE, add_environment_buildings, add_red_cube, forward_rgb


def detect_red_box(rgb: np.ndarray) -> tuple[np.ndarray, tuple[int, int, int, int] | None]:
    """Return an annotated BGR frame and the largest red target bounding box."""
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    low_red = cv2.inRange(hsv, (0, 100, 80), (10, 255, 255))
    high_red = cv2.inRange(hsv, (170, 100, 80), (180, 255, 255))
    mask = cv2.morphologyEx(low_red | high_red, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return bgr, None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 80:
        return bgr, None
    x, y, width, height = cv2.boundingRect(contour)
    cv2.rectangle(bgr, (x, y), (x + width, y + height), (0, 255, 255), 2)
    cv2.putText(bgr, f"red target: {width} x {height} px", (x, max(24, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    return bgr, (x, y, width, height)


def run(gui: bool, max_seconds: float) -> None:
    drone = create_world()
    add_red_cube()
    add_environment_buildings()
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
            annotated, _ = detect_red_box(forward_rgb(drone, renderer))
            if gui:
                cv2.imshow("HSV red target detector", annotated)
                if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                    return

        if gui:
            draw_force_vectors(drone, motor_thrusts, force_lines)
            time.sleep(TIME_STEP)


def self_check() -> None:
    drone = create_world()
    add_red_cube()
    _, box = detect_red_box(forward_rgb(drone, p.ER_TINY_RENDERER))
    assert box is not None, "HSV detector should find the red target cube"
    _, _, width, height = box
    assert width > 10 and height > 10, "Target bounding box should have visible size"
    print(f"Red target detector self-check passed: {width} x {height} px")


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
