"""Manual collective control with a stable four-motor quadcopter model."""

import argparse
from math import pi, sqrt
from pathlib import Path
import time

import pybullet as p
import pybullet_data

from pid import PID

MASS = 0.65
GRAVITY_Z = -9.81
PHYSICS_HZ = 240
CONTROL_HZ = 120
TIME_STEP = 1 / PHYSICS_HZ
CONTROL_STEPS = PHYSICS_HZ // CONTROL_HZ
PWM_MIN, PWM_HOVER, PWM_MAX = 1000.0, 1500.0, 2000.0
MAX_RPM = 24_000.0
MAX_THRUST_PER_MOTOR = MASS * abs(GRAVITY_Z)
KF = MAX_THRUST_PER_MOTOR / MAX_RPM**2
KM = 0.015 * KF
MOTOR_TIME_CONSTANT = 0.05
ROTOR_DRAG_COEFFICIENT = 2e-6
START_HEIGHT = 0.05
ARM_OFFSET = 0.12
MOTOR_YAW_SIGNS = (1, -1, -1, 1)
URDF_PATH = Path(__file__).parent / "assets" / "full_drone.urdf"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def thrust_from_pwm(pwm: float) -> float:
    """Return the commanded thrust of one rotor before motor lag."""
    normalized = clamp((pwm - PWM_MIN) / (PWM_MAX - PWM_MIN), 0.0, 1.0)
    return MAX_THRUST_PER_MOTOR * normalized**2


def rpm_from_thrust(thrust: float) -> float:
    return sqrt(clamp(thrust, 0.0, MAX_THRUST_PER_MOTOR) / KF)


def pwm_from_thrust(thrust: float) -> float:
    return PWM_MIN + (PWM_MAX - PWM_MIN) * sqrt(clamp(thrust, 0.0, MAX_THRUST_PER_MOTOR) / MAX_THRUST_PER_MOTOR)


def body_vector(drone: int, world_vector: tuple[float, float, float]) -> tuple[float, float, float]:
    rotation = p.getMatrixFromQuaternion(p.getBasePositionAndOrientation(drone)[1])
    return (
        rotation[0] * world_vector[0] + rotation[3] * world_vector[1] + rotation[6] * world_vector[2],
        rotation[1] * world_vector[0] + rotation[4] * world_vector[1] + rotation[7] * world_vector[2],
        rotation[2] * world_vector[0] + rotation[5] * world_vector[1] + rotation[8] * world_vector[2],
    )


def read_imu(drone: int) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return Euler attitude and body angular rate, as an IMU would report."""
    _, orientation = p.getBasePositionAndOrientation(drone)
    angular_velocity_world = p.getBaseVelocity(drone)[1]
    return p.getEulerFromQuaternion(orientation), body_vector(drone, angular_velocity_world)


def mix_motor_thrusts(collective: float, torque: tuple[float, float, float]) -> tuple[float, float, float, float]:
    """Mix a collective force and body torque, scaling corrections instead of clipping them."""
    roll_torque, pitch_torque, yaw_torque = torque
    collective = clamp(collective, 0.0, MAX_THRUST_PER_MOTOR)
    positions = ((ARM_OFFSET, ARM_OFFSET), (ARM_OFFSET, -ARM_OFFSET), (-ARM_OFFSET, ARM_OFFSET), (-ARM_OFFSET, -ARM_OFFSET))
    deltas = tuple(
        y * roll_torque / (4 * ARM_OFFSET**2)
        - x * pitch_torque / (4 * ARM_OFFSET**2)
        + yaw_sign * yaw_torque / (4 * KM / KF)
        for (x, y), yaw_sign in zip(positions, MOTOR_YAW_SIGNS)
    )
    scale = 1.0
    for delta in deltas:
        if delta > 0:
            scale = min(scale, (MAX_THRUST_PER_MOTOR - collective) / delta)
        elif delta < 0:
            scale = min(scale, collective / -delta)
    return tuple(collective + scale * delta for delta in deltas)


def update_motor_rpms(actual_rpms: tuple[float, float, float, float], target_rpms: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    alpha = min(1.0, TIME_STEP / MOTOR_TIME_CONSTANT)
    return tuple(actual + alpha * (target - actual) for actual, target in zip(actual_rpms, target_rpms))


def rotor_drag(motor_rpms: tuple[float, float, float, float], velocity_body: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(-ROTOR_DRAG_COEFFICIENT * sum(motor_rpms) * component for component in velocity_body)


def apply_flight_forces(drone: int, motor_rpms: tuple[float, float, float, float]) -> tuple[tuple[float, float, float, float], float]:
    motor_thrusts = tuple(KF * rpm**2 for rpm in motor_rpms)
    for link_index, (yaw_sign, thrust) in enumerate(zip(MOTOR_YAW_SIGNS, motor_thrusts)):
        p.applyExternalForce(drone, link_index, (0, 0, thrust), (0, 0, 0), p.LINK_FRAME)
        p.applyExternalTorque(drone, -1, (0, 0, yaw_sign * KM * motor_rpms[link_index] ** 2), p.LINK_FRAME)
    velocity_body = body_vector(drone, p.getBaseVelocity(drone)[0])
    p.applyExternalForce(drone, -1, rotor_drag(motor_rpms, velocity_body), (0, 0, 0), p.LINK_FRAME)
    return motor_thrusts, sum(motor_thrusts)


def wrap_angle(angle: float) -> float:
    return (angle + pi) % (2 * pi) - pi


def attitude_torque(drone: int, controllers: tuple[PID, PID, PID], yaw_target: float) -> tuple[float, float, float]:
    (roll, pitch, yaw), (roll_rate, pitch_rate, yaw_rate) = read_imu(drone)
    roll_pid, pitch_pid, yaw_pid = controllers
    return (
        roll_pid.update(-roll, roll_rate),
        pitch_pid.update(-pitch, pitch_rate),
        yaw_pid.update(wrap_angle(yaw_target - yaw), yaw_rate),
    )


def make_controllers() -> tuple[PID, PID, PID]:
    return (PID(0.002, 0.0, 0.001), PID(0.002, 0.0, 0.001), PID(0.001, 0.0, 0.0005))


def reset_controllers(controllers: tuple[PID, PID, PID]) -> None:
    for controller in controllers:
        controller.reset()


def create_world() -> int:
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, GRAVITY_Z)
    p.setTimeStep(TIME_STEP)
    p.loadURDF("plane.urdf")
    drone = p.loadURDF(str(URDF_PATH), (0, 0, START_HEIGHT), flags=p.URDF_USE_INERTIA_FROM_FILE)
    p.changeDynamics(drone, -1, linearDamping=0, angularDamping=0)
    return drone


def state_text(drone: int, pwm: float, motor_rpms: tuple[float, float, float, float], motor_thrusts: tuple[float, float, float, float], total_thrust: float, stabilized: bool) -> str:
    altitude = p.getBasePositionAndOrientation(drone)[0][2]
    vertical_velocity = p.getBaseVelocity(drone)[0][2]
    attitude, angular_velocity = read_imu(drone)
    return (
        f"PWM command: {pwm:.0f} us\n"
        f"Rotor RPM: {', '.join(f'{value:.0f}' for value in motor_rpms)}\n"
        f"Motor thrusts: {', '.join(f'{value:.2f}' for value in motor_thrusts)} N\n"
        f"Total thrust / weight: {total_thrust:.2f} / {MASS * abs(GRAVITY_Z):.2f} N\n"
        f"Altitude: {altitude:.2f} m, vertical velocity: {vertical_velocity:.2f} m/s\n"
        f"Roll/pitch/yaw: {', '.join(f'{value:.2f}' for value in attitude)} rad\n"
        f"Body rate: {', '.join(f'{value:.2f}' for value in angular_velocity)} rad/s\n"
        f"Attitude hold: {'on' if stabilized else 'off'}"
    )


def step_drone(drone: int, pwm: float, torque: tuple[float, float, float], motor_rpms: tuple[float, float, float, float]) -> tuple[tuple[float, float, float, float], tuple[float, float, float, float], float]:
    target_rpm = tuple(rpm_from_thrust(thrust) for thrust in mix_motor_thrusts(thrust_from_pwm(pwm), torque))
    motor_rpms = update_motor_rpms(motor_rpms, target_rpm)
    motor_thrusts, total_thrust = apply_flight_forces(drone, motor_rpms)
    p.stepSimulation()
    return motor_rpms, motor_thrusts, total_thrust


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


def draw_force_vectors(drone: int, motor_thrusts: tuple[float, float, float, float], line_ids: list[int]) -> None:
    rotation = p.getMatrixFromQuaternion(p.getBasePositionAndOrientation(drone)[1])
    direction = (rotation[2], rotation[5], rotation[8])
    for index, thrust in enumerate(motor_thrusts):
        position = p.getLinkState(drone, index)[0]
        endpoint = tuple(start + axis * thrust * 0.15 for start, axis in zip(position, direction))
        line_ids[index] = p.addUserDebugLine(position, endpoint, (0.1, 0.8, 0.2), lineWidth=3, replaceItemUniqueId=line_ids[index])


def run_gui() -> None:
    drone = create_world()
    p.resetDebugVisualizerCamera(cameraDistance=2.2, cameraYaw=45, cameraPitch=-25, cameraTargetPosition=(0, 0, 0.6))
    slider = p.addUserDebugParameter("Collective PWM (us)", PWM_MIN, PWM_MAX, PWM_MIN)
    stabilize_slider = p.addUserDebugParameter("Attitude hold (0=off, 1=on)", 0, 1, 1)
    text_id, force_lines = -1, [-1, -1, -1, -1]
    controllers = make_controllers()
    motor_rpms = (0.0, 0.0, 0.0, 0.0)
    stabilized_before = True
    yaw_target = 0.0
    torque = (0.0, 0.0, 0.0)
    step = 0
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
        stabilized_before = stabilized
        step += 1
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
