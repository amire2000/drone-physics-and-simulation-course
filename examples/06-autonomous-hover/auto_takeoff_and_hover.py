"""Take off to 3 m, turn 180 degrees, then land with shared PID controllers."""

import argparse
from pathlib import Path
import sys
import time

import pybullet as p

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.drone_model import DEFAULT_DRONE_MODEL, DEFAULT_PHYSICS_SETTINGS
from common.drone_physics import PhysicsEngine, clamp
from common.flight_control import AttitudeController, wrap_angle
from common.pybullet_sensors import read_imu, read_state
from common.pybullet_utils import create_world, draw_force_vectors, state_text
from common.pid import PID

TARGET_ALTITUDE = 3.0
HOVER_SECONDS = 2.0
YAW_TARGET_OFFSET = 3.141592653589793
MAX_SECONDS = 35.0
MODEL = DEFAULT_DRONE_MODEL
SETTINGS = DEFAULT_PHYSICS_SETTINGS
MASS = MODEL.mass_kg
START_HEIGHT = 0.05
TIME_STEP = SETTINGS.time_step_s
CONTROL_STEPS = SETTINGS.control_steps


def run(gui: bool, max_seconds: float = MAX_SECONDS) -> None:
    drone = create_world()
    engine = PhysicsEngine()
    altitude_pid = PID(kp=0.7, ki=0.05, kd=1.1, integral_limit=0.4)
    attitude_controller = AttitudeController()
    attitude_controller.yaw_pid = PID(kp=0.004, ki=0.0, kd=0.0015)
    initial_yaw = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])[2]
    yaw_target = initial_yaw
    phase, phase_started = "takeoff", 0.0
    torque = (0.0, 0.0, 0.0)
    text_id, force_lines = -1, [-1, -1, -1, -1]
    peak_altitude = START_HEIGHT

    for step in range(round(max_seconds / TIME_STEP)):
        now = step * TIME_STEP
        state = read_state(drone)
        position = state.position_m
        vertical_velocity = state.linear_velocity_mps[2]
        if phase == "takeoff" or phase == "hover" or phase == "yaw":
            target_altitude = TARGET_ALTITUDE
        else:
            target_altitude = START_HEIGHT

        if step % CONTROL_STEPS == 0:
            total_thrust = MASS * 9.81 + altitude_pid.update(target_altitude - position[2], vertical_velocity)
            pwm = engine.pwm_from_thrust(clamp(total_thrust / 4, 0.0, MODEL.max_thrust_per_motor_n))
            torque = attitude_controller.update(read_imu(drone), yaw_target)
        flight_step = engine.step(drone, pwm, torque)

        position = flight_step.state.position_m
        peak_altitude = max(peak_altitude, position[2])
        imu = read_imu(drone)
        (_, _, yaw), (_, _, yaw_rate) = imu.roll_pitch_yaw_rad, imu.angular_velocity_body_rad_s
        if phase == "takeoff" and position[2] > TARGET_ALTITUDE - 0.08 and abs(vertical_velocity) < 0.12:
            phase, phase_started = "hover", now
        elif phase == "hover" and now - phase_started >= HOVER_SECONDS:
            phase, phase_started, yaw_target = "yaw", now, wrap_angle(initial_yaw + YAW_TARGET_OFFSET)
        elif phase == "yaw" and abs(wrap_angle(yaw_target - yaw)) < 0.06 and abs(yaw_rate) < 0.12:
            phase, phase_started = "land", now
            altitude_pid.reset()
        elif phase == "land" and position[2] < 0.12 and abs(vertical_velocity) < 0.15:
            assert peak_altitude < TARGET_ALTITUDE + 0.25, "Altitude controller overshot its target"
            print(f"Landed after {now:.1f} s at yaw {yaw:.2f} rad; peak altitude {peak_altitude:.2f} m")
            return

        if gui:
            draw_force_vectors(drone, flight_step, force_lines)
            text_id = p.addUserDebugText(
                f"Automatic phase: {phase}\n" + state_text(flight_step, True),
                (0.4, -0.4, 1.4),
                textColorRGB=(0.05, 0.05, 0.05),
                textSize=1.2,
                replaceItemUniqueId=text_id,
            )
            time.sleep(TIME_STEP)

    raise AssertionError(f"Automatic flight did not finish within {max_seconds:.0f} seconds; stopped in {phase}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=MAX_SECONDS)
    args = parser.parse_args()
    client = p.connect(p.DIRECT if args.headless else p.GUI)
    try:
        run(not args.headless, args.max_seconds)
    finally:
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
