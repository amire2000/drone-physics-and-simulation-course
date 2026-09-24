"""Visual TTC-guided diagonal descent that intentionally strikes the red cube."""

import argparse
from dataclasses import dataclass
from math import cos, radians, sin, sqrt, tan
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
from forward_camera import CAMERA_HEIGHT, CAMERA_HZ, add_red_cube, forward_rgb
from red_target_detector import detect_red_box

TAKEOFF_ALTITUDE = 15.0
CUBE_FACE_X, CUBE_CENTER_Z = 19.0, 1.0
START_X = -5.25
PATH_ANGLE = radians(30)
TERMINAL_SPEED = 15.0
TERMINAL_VX = TERMINAL_SPEED * cos(PATH_ANGLE)
TERMINAL_VZ = -TERMINAL_SPEED * sin(PATH_ANGLE)
PATH_LENGTH = (TAKEOFF_ALTITUDE - CUBE_CENTER_Z) / sin(PATH_ANGLE)
PATH_ACCELERATION = TERMINAL_SPEED**2 / (2 * PATH_LENGTH)
COMMIT_BOX_HEIGHT = CAMERA_HEIGHT * 0.5
INITIAL_RANGE = CUBE_FACE_X - START_X
POST_IMPACT_SECONDS = 3.0
ENVIRONMENT_SIZE = (960, 540)


@dataclass(frozen=True)
class BarometerReading:
    altitude: float
    vertical_velocity: float


class Barometer:
    """Fixed-rate altitude sensor with optional deterministic noise and bias."""

    def __init__(self, rate_hz: float = CAMERA_HZ, noise_sigma: float = 0.0, bias: float = 0.0, seed: int = 7) -> None:
        self.period = 1 / rate_hz
        self.noise_sigma, self.bias = noise_sigma, bias
        self.rng = np.random.default_rng(seed)
        self.last_time: float | None = None
        self.last_altitude: float | None = None
        self.vertical_velocity = 0.0

    def sample(self, true_altitude: float, now: float) -> BarometerReading | None:
        if self.last_time is not None and now - self.last_time < self.period:
            return None
        altitude = true_altitude + self.bias + self.rng.normal(0.0, self.noise_sigma)
        if self.last_time is not None:
            measured_rate = (altitude - self.last_altitude) / (now - self.last_time)
            self.vertical_velocity = 0.7 * self.vertical_velocity + 0.3 * measured_rate
        self.last_time, self.last_altitude = now, altitude
        return BarometerReading(altitude, self.vertical_velocity)


@dataclass(frozen=True)
class TtcObservation:
    box: tuple[int, int, int, int]
    range_m: float
    ttc_s: float
    forward_velocity: float


class BboxTtcTracker:
    """Turns a known-size target's bounding-box growth into range and TTC."""

    def __init__(self) -> None:
        self.focal_pixels = CAMERA_HEIGHT / (2 * tan(radians(60) / 2))
        self.last_scale: float | None = None
        self.last_time: float | None = None
        self.filtered_growth = 0.0
        self.commit_ready = False
        self.last_observation: TtcObservation | None = None

    def update(self, box: tuple[int, int, int, int] | None, now: float) -> TtcObservation | None:
        if box is None:
            return None
        _, _, width, height = box
        scale = sqrt(width * height)
        if height >= COMMIT_BOX_HEIGHT:
            self.commit_ready = True
        if self.last_time is None:
            self.last_scale, self.last_time = scale, now
            return None
        dt = now - self.last_time
        growth = (scale - self.last_scale) / dt
        self.last_scale, self.last_time = scale, now
        self.filtered_growth = 0.65 * self.filtered_growth + 0.35 * growth
        if self.filtered_growth <= 0.01:
            return None
        range_m = self.focal_pixels * 2.0 / scale
        ttc_s = scale / self.filtered_growth
        observation = TtcObservation(box, range_m, ttc_s, range_m / ttc_s)
        self.last_observation = observation
        return observation


@dataclass(frozen=True)
class TrajectoryCommand:
    forward_velocity: float
    vertical_velocity: float
    altitude_target: float


class DiagonalTrajectory:
    """Generates a 30 degree path that reaches terminal speed at target contact."""

    def __init__(self) -> None:
        self.initial_range: float | None = None

    def command(self, remaining_range: float) -> TrajectoryCommand:
        self.initial_range = self.initial_range or max(remaining_range, 1e-6)
        remaining_fraction = clamp(remaining_range / self.initial_range, 0.0, 1.0)
        traveled_path = PATH_LENGTH * (1.0 - remaining_fraction)
        path_speed = min(TERMINAL_SPEED, max(1.0, sqrt(2 * PATH_ACCELERATION * traveled_path)))
        return TrajectoryCommand(
            path_speed * cos(PATH_ANGLE),
            -path_speed * sin(PATH_ANGLE),
            CUBE_CENTER_Z + (TAKEOFF_ALTITUDE - CUBE_CENTER_Z) * remaining_fraction,
        )


def annotate(frame: np.ndarray, phase: str, observation: TtcObservation | None, command: TrajectoryCommand | None, pitch: float, thrust: float) -> np.ndarray:
    lines = [f"phase: {phase}", f"pitch: {np.degrees(pitch):.1f} deg", f"thrust: {thrust:.2f} N"]
    if observation:
        lines.extend((f"range: {observation.range_m:.1f} m", f"TTC: {observation.ttc_s:.2f} s", f"vx visual: {observation.forward_velocity:.1f} m/s"))
    if command:
        lines.extend((f"vx command: {command.forward_velocity:.1f} m/s", f"vz command: {command.vertical_velocity:.1f} m/s"))
    for index, text in enumerate(lines):
        cv2.putText(frame, text, (12, 28 + 24 * index), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return frame


def environment_rgb(renderer: int) -> np.ndarray:
    """Render a fixed wide view containing the launch point, path, and cube."""
    view = p.computeViewMatrix(cameraEyePosition=(7.0, -32.0, 22.0), cameraTargetPosition=(7.0, 0.0, 7.0), cameraUpVector=(0.0, 0.0, 1.0))
    projection = p.computeProjectionMatrixFOV(55.0, ENVIRONMENT_SIZE[0] / ENVIRONMENT_SIZE[1], 0.1, 80.0)
    image = p.getCameraImage(*ENVIRONMENT_SIZE, view, projection, renderer=renderer)
    return np.reshape(image[2], (*ENVIRONMENT_SIZE[::-1], 4))[:, :, :3]


def print_summary(success: bool, phase: str, speed: float, duration: float, video: Path | None) -> None:
    print("\n--- TTC strike summary ---")
    print(f"result: {'target contacted' if success else 'no valid contact'}")
    print(f"final phase: {phase}; simulated time: {duration:.1f} s")
    if speed:
        print(f"impact speed: {speed:.1f} m/s")
    if video:
        print(f"environment video: {video}")


def run(gui: bool, max_seconds: float, video: Path | None) -> tuple[bool, float]:
    drone = create_world()
    p.resetBasePositionAndOrientation(drone, (START_X, 0, 0.05), (0, 0, 0, 1))
    cube = add_red_cube()
    barometer = Barometer()
    tracker, trajectory = BboxTtcTracker(), DiagonalTrajectory()
    altitude_pid = PID(0.7, 0.05, 1.1, integral_limit=0.5)
    forward_pid = PID(0.08, 0.0, 0.0)
    vertical_velocity_pid = PID(0.7, 0.0, 0.0)
    attitude_pids = make_controllers()
    motor_rpms = (0.0, 0.0, 0.0, 0.0)
    thrust, pitch_target = MASS * 9.81, 0.0
    phase, commit_deadline = "takeoff", 0.0
    observation: TtcObservation | None = None
    command: TrajectoryCommand | None = None
    baro = BarometerReading(0.05, 0.0)
    force_lines = [-1, -1, -1, -1]
    renderer = p.ER_BULLET_HARDWARE_OPENGL if gui else p.ER_TINY_RENDERER
    impact_speed, stop_at = 0.0, None
    writer = None
    if video:
        video.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"mp4v"), CAMERA_HZ, ENVIRONMENT_SIZE)
        if not writer.isOpened():
            raise RuntimeError(f"Could not open video output: {video}")
    if gui:
        p.resetDebugVisualizerCamera(36.0, 48.0, -25.0, (7.0, 0.0, 7.0))

    try:
        for step in range(round(max_seconds / TIME_STEP)):
            now = step * TIME_STEP
            position, _ = p.getBasePositionAndOrientation(drone)
            sample = barometer.sample(position[2], now)
            if sample:
                baro = sample

            frame = None
            if step % (PHYSICS_HZ // CAMERA_HZ) == 0:
                if writer:
                    writer.write(cv2.cvtColor(environment_rgb(renderer), cv2.COLOR_RGB2BGR))
                frame, box = detect_red_box(forward_rgb(drone, renderer, world_target=(20.0, 0.0, 1.0)))
                observation = tracker.update(box, now)
                if phase == "track" and box is None:
                    if tracker.commit_ready and tracker.last_observation:
                        phase = "commit"
                        commit_deadline = now + tracker.last_observation.ttc_s + 0.5
                    else:
                        phase = "abort"

            if step % CONTROL_STEPS == 0:
                if phase == "post-impact":
                    thrust, torque = 0.0, (0.0, 0.0, 0.0)
                else:
                    if phase == "takeoff":
                        altitude_target = TAKEOFF_ALTITUDE
                        pitch_target = 0.0
                        if baro.altitude > TAKEOFF_ALTITUDE - 0.2 and abs(baro.vertical_velocity) < 0.5 and tracker.last_observation:
                            phase = "track"
                            tracker = BboxTtcTracker()  # Takeoff changes apparent scale but is not forward closure.
                            observation = None
                    elif phase == "track" and observation:
                        command = trajectory.command(observation.range_m)
                        pitch_target = clamp(radians(30) + forward_pid.update(command.forward_velocity - observation.forward_velocity, 0.0), 0.0, radians(30))
                        altitude_target = command.altitude_target
                    elif phase == "track":
                        command = trajectory.command(INITIAL_RANGE)
                        pitch_target = radians(30)  # Start moving so bbox scale can reveal TTC.
                        altitude_target = command.altitude_target
                    elif phase == "abort":
                        altitude_target = baro.altitude
                        pitch_target = 0.0
                    else:  # Commit holds the last pitch and thrust command.
                        altitude_target = baro.altitude

                    if phase == "track" and command:
                        corrected_vertical_velocity = clamp(command.vertical_velocity + 0.8 * (command.altitude_target - baro.altitude), TERMINAL_VZ, 3.0)
                        thrust = MASS * 9.81 + vertical_velocity_pid.update(corrected_vertical_velocity - baro.vertical_velocity, 0.0)
                    elif phase != "commit":
                        thrust = MASS * 9.81 + altitude_pid.update(altitude_target - baro.altitude, baro.vertical_velocity)
                    torque = attitude_torque(drone, attitude_pids, yaw_target=0.0, pitch_target=pitch_target)

            pwm = pwm_from_thrust(clamp(thrust / 4, 0.0, MASS * 9.81))
            motor_rpms, motor_thrusts, _ = step_drone(drone, pwm, torque, motor_rpms)
            if not impact_speed and p.getContactPoints(drone, cube):
                velocity = p.getBaseVelocity(drone)[0]
                impact_speed = sqrt(sum(component**2 for component in velocity))
                phase, stop_at = "post-impact", now + POST_IMPACT_SECONDS
                print(f"Impact: {impact_speed:.1f} m/s; recording aftermath for {POST_IMPACT_SECONDS:.0f} s")
            if stop_at is not None and now >= stop_at:
                success = 10.0 <= impact_speed <= 20.0
                print_summary(success, phase, impact_speed, now, video)
                return success, impact_speed
            if phase == "commit" and now > commit_deadline:
                print("Commit deadline expired without contact")
                print_summary(False, phase, 0.0, now, video)
                return False, 0.0
            if phase == "abort":
                last_height = tracker.last_observation.box[3] if tracker.last_observation else 0
                print(f"Aborted: target lost before commit at ({position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f}); last bbox height {last_height} px")
                print_summary(False, phase, 0.0, now, video)
                return False, 0.0

            if gui:
                if frame is not None:
                    cv2.imshow("TTC diagonal strike", annotate(frame, phase, observation, command, pitch_target, thrust))
                    if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                        print_summary(False, phase, impact_speed, now, video)
                        return False, impact_speed
                draw_force_vectors(drone, motor_thrusts, force_lines)
                time.sleep(TIME_STEP)
        final_position, _ = p.getBasePositionAndOrientation(drone)
        print(f"Strike timed out in {phase} phase at position ({final_position[0]:.1f}, {final_position[1]:.1f}, {final_position[2]:.1f})")
        print_summary(False, phase, impact_speed, max_seconds, video)
        return False, impact_speed
    finally:
        if writer:
            writer.release()


def self_check() -> None:
    tracker = BboxTtcTracker()
    assert tracker.update((0, 0, 20, 20), 0.0) is None
    observation = tracker.update((0, 0, 30, 30), 0.1)
    assert observation and observation.ttc_s > 0 and observation.forward_velocity > 0
    tracker.update((0, 0, 30, int(COMMIT_BOX_HEIGHT)), 0.2)
    assert tracker.commit_ready, "A large bbox should arm the terminal commit phase"
    command = DiagonalTrajectory().command(0.0)
    assert abs(command.forward_velocity - TERMINAL_VX) < 0.01
    assert abs(command.vertical_velocity - TERMINAL_VZ) < 0.01
    print("TTC strike component self-check passed")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--self-check", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=35.0)
    parser.add_argument("--video", type=Path, default=Path("outputs/ttc_diagonal_strike.mp4"))
    parser.add_argument("--no-video", action="store_true")
    args = parser.parse_args()
    client = p.connect(p.DIRECT if args.headless or args.self_check else p.GUI)
    try:
        if args.self_check:
            self_check()
        else:
            success, speed = run(not args.headless, args.max_seconds, None if args.no_video else args.video)
            if args.headless:
                assert success, f"Strike failed; impact speed was {speed:.1f} m/s"
    finally:
        cv2.destroyAllWindows()
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
