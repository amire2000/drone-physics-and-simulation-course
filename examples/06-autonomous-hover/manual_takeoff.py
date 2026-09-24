"""Manual collective control using the shared quadcopter control module."""

import argparse
import time

import pybullet as p

from drone_control import (
    CONTROL_STEPS,
    MASS,
    MAX_THRUST_PER_MOTOR,
    PHYSICS_HZ,
    PWM_HOVER,
    PWM_MAX,
    PWM_MIN,
    START_HEIGHT,
    TIME_STEP,
    attitude_torque,
    create_world,
    draw_force_vectors,
    make_controllers,
    mix_motor_thrusts,
    reset_controllers,
    rotor_drag,
    state_text,
    step_drone,
    thrust_from_pwm,
)


def run_headless(pwm: float, seconds: float, stabilized: bool, disturb_roll: float) -> tuple[float, float]:
    drone = create_world()
    if disturb_roll:
        p.resetBasePositionAndOrientation(drone, (0, 0, 1), p.getQuaternionFromEuler((disturb_roll, 0, 0)))
    controllers = make_controllers()
    yaw_target = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])[2]
    motor_rpms = (0.0, 0.0, 0.0, 0.0)
    first_motor_thrusts = None
    torque = (0.0, 0.0, 0.0)
    for step in range(round(seconds / TIME_STEP)):
        if stabilized and step % CONTROL_STEPS == 0:
            torque = attitude_torque(drone, controllers, yaw_target)
        motor_rpms, motor_thrusts, _ = step_drone(drone, pwm, torque if stabilized else (0.0, 0.0, 0.0), motor_rpms)
        first_motor_thrusts = first_motor_thrusts or motor_thrusts
    altitude = p.getBasePositionAndOrientation(drone)[0][2]
    vertical_velocity = p.getBaseVelocity(drone)[0][2]
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
    motor_rpms = (0.0, 0.0, 0.0, 0.0)
    for _ in range(PHYSICS_HZ):
        motor_rpms, _, _ = step_drone(drone, PWM_HOVER, (0.0, 0.0, 0.0), motor_rpms)
    motor_rpms, motor_thrusts, _ = step_drone(drone, 1750, (0.0, 0.0, 0.0), motor_rpms)
    assert max(motor_thrusts) < thrust_from_pwm(1750), "Motor lag should soften a PWM step"
    assert max(motor_thrusts) - min(motor_thrusts) < 1e-9, "A collective step must keep motors equal"
    for _ in range(PHYSICS_HZ // 2):
        motor_rpms, _, _ = step_drone(drone, 1750, (0.0, 0.0, 0.0), motor_rpms)
    motor_rpms, motor_thrusts, _ = step_drone(drone, 1200, (0.0, 0.0, 0.0), motor_rpms)
    assert min(motor_thrusts) > thrust_from_pwm(1200), "Motor lag should soften a throttle-down step"
    roll, pitch, _ = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])
    assert max(abs(roll), abs(pitch)) < 0.01, "A collective step must not tilt the drone"
    saturated = mix_motor_thrusts(MAX_THRUST_PER_MOTOR * 0.9, (10.0, 10.0, 10.0))
    assert all(0.0 <= thrust <= MAX_THRUST_PER_MOTOR for thrust in saturated), "Mixer outputs must stay in range"
    assert sum(a * b for a, b in zip(rotor_drag((12_000.0,) * 4, (1.0, -2.0, 3.0)), (1.0, -2.0, 3.0))) < 0, "Drag must oppose velocity"
    print("PWM step check passed")


def run_gui() -> None:
    drone = create_world()
    p.resetDebugVisualizerCamera(cameraDistance=2.2, cameraYaw=45, cameraPitch=-25, cameraTargetPosition=(0, 0, 0.6))
    slider = p.addUserDebugParameter("Collective PWM (us)", PWM_MIN, PWM_MAX, PWM_MIN)
    stabilize_slider = p.addUserDebugParameter("Attitude hold (0=off, 1=on)", 0, 1, 1)
    text_id, force_lines = -1, [-1, -1, -1, -1]
    controllers = make_controllers()
    motor_rpms = (0.0, 0.0, 0.0, 0.0)
    stabilized_before, yaw_target, torque, step = True, 0.0, (0.0, 0.0, 0.0), 0
    while p.isConnected():
        pwm = p.readUserDebugParameter(slider)
        stabilized = p.readUserDebugParameter(stabilize_slider) >= 0.5
        if stabilized and not stabilized_before:
            reset_controllers(controllers)
            yaw_target = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])[2]
        if stabilized and step % CONTROL_STEPS == 0:
            torque = attitude_torque(drone, controllers, yaw_target)
        motor_rpms, motor_thrusts, total_thrust = step_drone(drone, pwm, torque if stabilized else (0.0, 0.0, 0.0), motor_rpms)
        draw_force_vectors(drone, motor_thrusts, force_lines)
        text_id = p.addUserDebugText(state_text(drone, pwm, motor_rpms, motor_thrusts, total_thrust, stabilized), (0.4, -0.4, 1.4), textColorRGB=(0.05, 0.05, 0.05), textSize=1.2, replaceItemUniqueId=text_id)
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
