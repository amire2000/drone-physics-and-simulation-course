"""PyBullet adapter that composes sensing, TTC, guidance, views, and telemetry."""

from dataclasses import dataclass
from math import sqrt
from pathlib import Path
import time

import cv2
import pybullet as p

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
from forward_camera import add_environment_buildings, add_red_cube, forward_rgb
from red_target_detector import detect_red_box

from .config import SceneConfig, StrikeConfig
from .guidance import FlightPhase, GuidanceCommand, GuidanceInput, StrikeGuidance
from .sensing import Barometer, BarometerReading
from .telemetry import FlightLog, make_plot, move_plot_window, refresh_plot, save_csv, save_plot
from .ttc import BboxTtcTracker, TtcObservation
from .views import annotate, environment_rgb


@dataclass(frozen=True)
class StrikeResult:
    success: bool
    phase: str
    simulated_time_s: float
    impact_speed_mps: float
    video: Path | None
    plot: Path | None
    csv: Path | None


class StrikeSimulation:
    """Run one configured strike against the concrete PyBullet simulator."""

    def __init__(self, config: StrikeConfig | None = None, scene: SceneConfig | None = None) -> None:
        self.config = config or StrikeConfig()
        self.scene = scene or SceneConfig()

    def run(self, gui: bool, max_seconds: float, video: Path | None, plot: Path | None, csv: Path | None = None) -> StrikeResult:
        config = self.config
        drone = create_world()
        p.resetBasePositionAndOrientation(drone, config.launch_position, (0, 0, 0, 1))
        cube = add_red_cube(self.scene.target_center, self.scene.target_size_m)
        add_environment_buildings()
        barometer, tracker, guidance = Barometer(config), BboxTtcTracker(config), StrikeGuidance(config)
        attitude_pids = make_controllers()
        motor_rpms = (0.0, 0.0, 0.0, 0.0)
        motor_thrusts = (0.0, 0.0, 0.0, 0.0)
        torque = (0.0, 0.0, 0.0)
        command = GuidanceCommand(FlightPhase.TAKEOFF, config.hover_thrust_n, 0.0, None)
        baro = BarometerReading(config.launch_position[2], 0.0)
        observation: TtcObservation | None = None
        target_visible = False
        force_lines = [-1, -1, -1, -1]
        renderer = p.ER_BULLET_HARDWARE_OPENGL if gui else p.ER_TINY_RENDERER
        impact_speed, stop_at_s = 0.0, None
        log = FlightLog()
        writer = self._video_writer(video, config)
        live_plot = self._live_plot(gui, plot, config, self.scene)
        if gui:
            cv2.namedWindow("TTC diagonal strike", cv2.WINDOW_NORMAL)
            cv2.moveWindow("TTC diagonal strike", *config.opencv_window_position_px)
            p.resetDebugVisualizerCamera(36.0, 48.0, -25.0, (7.0, 0.0, 7.0))

        def finish(success: bool, phase: str, now_s: float) -> StrikeResult:
            if plot:
                save_plot(log, config, self.scene, plot)
            if csv:
                save_csv(log, csv)
            if live_plot:
                refresh_plot(live_plot, log)
            result = StrikeResult(success, phase, now_s, impact_speed, video, plot, csv)
            self._print_summary(result)
            return result

        try:
            for step in range(round(max_seconds / TIME_STEP)):
                now_s = step * TIME_STEP
                position, _ = p.getBasePositionAndOrientation(drone)
                sample = barometer.sample(position[2], now_s)
                if sample:
                    baro = sample

                frame = None
                if step % (PHYSICS_HZ // config.camera_hz) == 0:
                    if writer:
                        writer.write(cv2.cvtColor(environment_rgb(renderer, config), cv2.COLOR_RGB2BGR))
                    frame, box = detect_red_box(
                        forward_rgb(
                            drone,
                            renderer,
                            look_down_degrees=config.camera_look_down_deg,
                            width_px=config.camera_width_px,
                            height_px=config.camera_height_px,
                            fov_deg=config.camera_fov_deg,
                        )
                    )
                    target_visible = box is not None
                    observation = tracker.update(box, now_s)

                if step % CONTROL_STEPS == 0:
                    if stop_at_s is None:
                        current_velocity = p.getBaseVelocity(drone)[0]
                        command = guidance.update(GuidanceInput(now_s, baro, observation, tracker.last_observation, target_visible, tracker.commit_ready, current_velocity[0]))
                        if command.reset_ttc:
                            # This flag belongs to the takeoff-to-track handoff:
                            # ignore bbox scale accumulated during vertical climb.
                            tracker.reset()
                            observation = None
                        if command.commit_expired:
                            # The held terminal command exceeded its predicted
                            # TTC window without contacting the cube.
                            print("Commit deadline expired without contact")
                            return finish(False, command.phase.value, now_s)
                        if command.phase == FlightPhase.ABORT:
                            # Guidance has already neutralized its pitch request;
                            # stop before applying another flight-control cycle.
                            last_height = tracker.last_observation.box[3] if tracker.last_observation else 0
                            print(f"Aborted: target lost before commit; last bbox height {last_height} px")
                            return finish(False, command.phase.value, now_s)
                        # pitch_target_rad is a high-level attitude request.
                        # attitude_torque compares it with the IMU attitude and
                        # returns the body torque needed by the motor mixer.
                        torque = attitude_torque(drone, attitude_pids, yaw_target=0.0, pitch_target=command.pitch_target_rad)
                    else:
                        # Post-impact: do not keep steering or accelerating.
                        torque = (0.0, 0.0, 0.0)

                # thrust_n is the collective force. Split it evenly before
                # mapping force to a PWM signal for the four motors.
                collective = 0.0 if stop_at_s is not None else command.thrust_n
                pwm = pwm_from_thrust(clamp(collective / 4, 0.0, MASS * 9.81))
                motor_rpms, motor_thrusts, _ = step_drone(drone, pwm, torque, motor_rpms)
                position, _ = p.getBasePositionAndOrientation(drone)
                velocity, _ = p.getBaseVelocity(drone)
                pitch_rad = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])[1]
                # trajectory is observational here: FlightLog plots its vx,
                # vz, and altitude target beside measured vehicle state.
                log.append(now_s, position, velocity, command, pitch_rad, observation)
                if live_plot and step % (PHYSICS_HZ // config.camera_hz) == 0:
                    refresh_plot(live_plot, log)

                if not impact_speed and p.getContactPoints(drone, cube):
                    impact_speed = sqrt(sum(component**2 for component in velocity))
                    stop_at_s = now_s + config.post_impact_seconds
                    print(f"Impact: {impact_speed:.1f} m/s; recording aftermath for {config.post_impact_seconds:.0f} s")
                if stop_at_s is not None and now_s >= stop_at_s:
                    # Contact is the geometry-free success condition.  Keep
                    # impact speed as telemetry instead of rejecting a valid
                    # strike because the simulated vehicle model is tuned
                    # differently from a real airframe.
                    return finish(True, "post-impact", now_s)

                if gui:
                    if frame is not None:
                        cv2.imshow("TTC diagonal strike", annotate(frame, command, observation))
                        if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                            return finish(False, command.phase.value, now_s)
                    draw_force_vectors(drone, motor_thrusts, force_lines)
                    time.sleep(TIME_STEP)
            print(f"Strike timed out in {command.phase.value} phase")
            return finish(False, command.phase.value, max_seconds)
        finally:
            if writer:
                writer.release()

    @staticmethod
    def _video_writer(video: Path | None, config: StrikeConfig):
        if not video:
            return None
        video.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"mp4v"), config.camera_hz, config.environment_size_px)
        if not writer.isOpened():
            raise RuntimeError(f"Could not open video output: {video}")
        return writer

    @staticmethod
    def _live_plot(gui: bool, output: Path | None, config: StrikeConfig, scene: SceneConfig):
        if not gui or not output:
            return None
        import matplotlib.pyplot as plt

        plt.ion()
        live_plot = make_plot(config, scene)
        live_plot.figure.canvas.manager.set_window_title("Live TTC strike telemetry")
        plt.show(block=False)
        move_plot_window(live_plot, config.plot_window_position_px)
        return live_plot

    @staticmethod
    def _print_summary(result: StrikeResult) -> None:
        print("\n--- TTC strike summary ---")
        print(f"result: {'target contacted' if result.success else 'no valid contact'}")
        print(f"final phase: {result.phase}; simulated time: {result.simulated_time_s:.1f} s")
        if result.impact_speed_mps:
            print(f"impact speed: {result.impact_speed_mps:.1f} m/s")
        if result.video:
            print(f"environment video: {result.video}")
        if result.plot:
            print(f"trajectory plot: {result.plot}")
        if result.csv:
            print(f"telemetry CSV: {result.csv}")
        print("environment: red target cube and 3 static buildings")
