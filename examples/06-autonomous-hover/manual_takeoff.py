"""Manual collective control using the shared quadcopter control module."""

import argparse
from pathlib import Path
import sys
import time

import pybullet as p

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.drone_model import DEFAULT_DRONE_MODEL, DEFAULT_PHYSICS_SETTINGS
from common.drone_physics import PWM_HOVER, PWM_MAX, PWM_MIN, PhysicsEngine
from common.flight_control import AttitudeController
from common.pybullet_sensors import read_imu, read_state
from common.pybullet_utils import create_world, draw_force_vectors, state_text

MODEL = DEFAULT_DRONE_MODEL
SETTINGS = DEFAULT_PHYSICS_SETTINGS
MASS = MODEL.mass_kg
PHYSICS_HZ = SETTINGS.physics_hz
CONTROL_STEPS = SETTINGS.control_steps
TIME_STEP = SETTINGS.time_step_s
START_HEIGHT = 0.05


def run_headless(pwm: float, seconds: float, stabilized: bool, disturb_roll: float) -> tuple[float, float]:
    drone = create_world()
    if disturb_roll:
        p.resetBasePositionAndOrientation(drone, (0, 0, 1), p.getQuaternionFromEuler((disturb_roll, 0, 0)))
    engine = PhysicsEngine()
    controller = AttitudeController()
    yaw_target = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])[2]
    first_motor_thrusts = None
    torque = (0.0, 0.0, 0.0)
    for step in range(round(seconds / TIME_STEP)):
        if stabilized and step % CONTROL_STEPS == 0:
            torque = controller.update(read_imu(drone), yaw_target)
        flight_step = engine.step(drone, pwm, torque if stabilized else (0.0, 0.0, 0.0))
        first_motor_thrusts = first_motor_thrusts or flight_step.motor_thrusts_n
    altitude = flight_step.state.position_m[2]
    vertical_velocity = flight_step.state.linear_velocity_mps[2]
    print(f"PWM {pwm:.0f} us -> altitude {altitude:.3f} m, vertical velocity {vertical_velocity:.3f} m/s")
    if pwm <= PWM_MIN:
        assert altitude < 0.1, "Motors off should stay on the ground"
    elif abs(pwm - PWM_HOVER) < 1e-9 and not disturb_roll:
        assert abs(altitude - START_HEIGHT) < 0.1, "Hover PWM should hold altitude"
    elif pwm > PWM_HOVER:
        assert altitude > START_HEIGHT + 0.25, "Higher PWM should take off"
    if stabilized and not disturb_roll:
        assert max(first_motor_thrusts) - min(first_motor_thrusts) < 1e-9, "Level attitude should use equal motor thrust"
    if stabilized and disturb_roll:
        final_roll = abs(p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])[0])
        assert final_roll < abs(disturb_roll) / 2, "Attitude PID should reduce a roll disturbance"
    return altitude, vertical_velocity


def run_step_check() -> None:
    drone = create_world()
    engine = PhysicsEngine()
    for _ in range(PHYSICS_HZ):
        engine.step(drone, PWM_HOVER, (0.0, 0.0, 0.0))
    flight_step = engine.step(drone, 1750, (0.0, 0.0, 0.0))
    assert max(flight_step.motor_thrusts_n) < engine.thrust_from_pwm(1750), "Motor lag should soften a PWM step"
    assert max(flight_step.motor_thrusts_n) - min(flight_step.motor_thrusts_n) < 1e-9, "A collective step must keep motors equal"
    for _ in range(PHYSICS_HZ // 2):
        engine.step(drone, 1750, (0.0, 0.0, 0.0))
    flight_step = engine.step(drone, 1200, (0.0, 0.0, 0.0))
    assert min(flight_step.motor_thrusts_n) > engine.thrust_from_pwm(1200), "Motor lag should soften a throttle-down step"
    roll, pitch, _ = read_imu(drone).roll_pitch_yaw_rad
    assert max(abs(roll), abs(pitch)) < 0.01, "A collective step must not tilt the drone"
    print("PWM step check passed")


def run_gui() -> None:
    drone = create_world()
    p.resetDebugVisualizerCamera(cameraDistance=2.2, cameraYaw=45, cameraPitch=-25, cameraTargetPosition=(0, 0, 0.6))
    slider = p.addUserDebugParameter("Collective PWM (us)", PWM_MIN, PWM_MAX, PWM_MIN)
    stabilize_slider = p.addUserDebugParameter("Attitude hold (0=off, 1=on)", 0, 1, 1)
    text_id, force_lines = -1, [-1, -1, -1, -1]
    engine = PhysicsEngine()
    controller = AttitudeController()
    stabilized_before, yaw_target, torque, step = True, 0.0, (0.0, 0.0, 0.0), 0
    while p.isConnected():
        pwm = p.readUserDebugParameter(slider)
        stabilized = p.readUserDebugParameter(stabilize_slider) >= 0.5
        if stabilized and not stabilized_before:
            controller.reset()
            yaw_target = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])[2]
        if stabilized and step % CONTROL_STEPS == 0:
            torque = controller.update(read_imu(drone), yaw_target)
        flight_step = engine.step(drone, pwm, torque if stabilized else (0.0, 0.0, 0.0))
        draw_force_vectors(drone, flight_step, force_lines)
        text_id = p.addUserDebugText(state_text(flight_step, stabilized), (0.4, -0.4, 1.4), textColorRGB=(0.05, 0.05, 0.05), textSize=1.2, replaceItemUniqueId=text_id)
        stabilized_before, step = stabilized, step + 1
        time.sleep(TIME_STEP)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--pwm", type=float, default=PWM_MIN)
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--stabilized", action="store_true")
    parser.add_argument("--disturb-roll", type=float, default=0.0)
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    client = p.connect(p.DIRECT if args.headless or args.self_check else p.GUI)
    try:
        if args.self_check:
            run_step_check()
        elif args.headless:
            run_headless(args.pwm, args.seconds, args.stabilized, args.disturb_roll)
        else:
            run_gui()
    finally:
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
