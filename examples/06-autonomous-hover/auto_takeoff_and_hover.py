"""Take off to 3 m, turn 180 degrees, then land with shared PID controllers."""

import argparse
from dataclasses import replace
from pathlib import Path
import sys
import time

import pybullet as p

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.drone_model import load_drone_profile
from common.drone_physics import PhysicsEngine, clamp
from common.flight_control import AttitudeController, wrap_angle
from common.pybullet_sensors import read_imu, read_state
from common.pybullet_utils import create_world, draw_force_vectors, state_text
from common.pid import PID

TARGET_ALTITUDE = 3.0
HOVER_SECONDS = 4.0
YAW_TARGET_OFFSET = 3.141592653589793
MAX_SECONDS = 35.0
PROFILE = load_drone_profile("real_reference")
MODEL = PROFILE.model
SETTINGS = PROFILE.physics_settings
MASS = MODEL.mass_kg
START_HEIGHT = 0.05
TIME_STEP = SETTINGS.time_step_s
CONTROL_STEPS = SETTINGS.control_steps


class WindControls:
    """Matplotlib controls for the final PyBullet wind experiment."""

    def __init__(self) -> None:
        """Create wind sliders, presets, reset, and the enable toggle."""
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button, CheckButtons, Slider

        self.plt = plt
        self.enabled = True
        self.figure = plt.figure("Final-flight wind controls", figsize=(7.5, 4.5))
        self.figure.subplots_adjust(left=0.12, right=0.95, bottom=0.36)
        self.sliders = {}
        for index, axis_name in enumerate(("X", "Y", "Z")):
            axis = self.figure.add_axes((0.16, 0.25 - index * 0.07, 0.54, 0.035))
            self.sliders[axis_name] = Slider(axis, f"wind {axis_name} (m/s)", -10.0, 10.0, valinit=0.0)
        toggle_axis = self.figure.add_axes((0.76, 0.22, 0.16, 0.10))
        self.toggle = CheckButtons(toggle_axis, ["Enable wind"], [True])
        self.toggle.on_clicked(self._toggle_wind)
        self.buttons = {}
        for label, left in (("Calm", 0.16), ("+Y", 0.31), ("-Y", 0.46), ("Gust", 0.61), ("Reset", 0.76)):
            axis = self.figure.add_axes((left, 0.08, 0.11, 0.07))
            button = Button(axis, label)
            button.on_clicked(lambda _, preset=label: self._preset(preset))
            self.buttons[label] = button
        self.readout = self.figure.text(0.16, 0.02, "Wind: enabled | (0.0, 0.0, 0.0) m/s")
        plt.show(block=False)

    def _toggle_wind(self, _: object) -> None:
        """Toggle whether the shared physics engine uses the wind vector."""
        self.enabled = not self.enabled

    def _preset(self, name: str) -> None:
        """Set a documented wind preset without changing the flight code."""
        values = {
            "Calm": (0.0, 0.0, 0.0),
            "+Y": (0.0, 5.0, 0.0),
            "-Y": (0.0, -5.0, 0.0),
            "Gust": (0.0, 7.0, 0.0),
            "Reset": (0.0, 0.0, 0.0),
        }[name]
        for axis_name, value in zip(("X", "Y", "Z"), values):
            self.sliders[axis_name].set_val(value)
        if name == "Reset":
            self.enabled = True

    def read(self) -> tuple[bool, tuple[float, float, float]]:
        """Read the current vector, refresh the controls, and keep the window responsive."""
        vector = tuple(float(self.sliders[axis].val) for axis in ("X", "Y", "Z"))
        status = "enabled" if self.enabled else "disabled"
        self.readout.set_text(f"Wind: {status} | {tuple(round(value, 1) for value in vector)} m/s")
        self.plt.pause(0.001)
        return self.enabled, vector


def run(gui: bool, max_seconds: float = MAX_SECONDS, wind_gui: bool = False) -> None:
    drone = create_world(MODEL, SETTINGS)
    engine = PhysicsEngine(MODEL, SETTINGS)
    controls = WindControls() if wind_gui else None
    altitude_pid = PID(kp=3.0, ki=0.0, kd=3.0, integral_limit=0.4)
    attitude_controller = AttitudeController()
    attitude_controller.yaw_pid = PID(kp=0.004, ki=0.0, kd=0.0015)
    initial_yaw = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])[2]
    yaw_target = initial_yaw
    phase, phase_started = "takeoff", 0.0
    torque = (0.0, 0.0, 0.0)
    text_id, force_lines = -1, [-1, -1, -1, -1]
    peak_altitude = START_HEIGHT
    last_wind = SETTINGS.wind_world_mps

    for step in range(round(max_seconds / TIME_STEP)):
        now = step * TIME_STEP
        if controls is not None:
            wind_enabled, wind_vector = controls.read()
            engine.settings = replace(SETTINGS, wind_enabled=wind_enabled, wind_world_mps=wind_vector)
            if wind_vector != last_wind:
                print(f"wind changed at {now:.2f} s: {last_wind} -> {wind_vector} m/s")
                last_wind = wind_vector
        state = read_state(drone)
        position = state.position_m
        vertical_velocity = state.linear_velocity_mps[2]
        if phase == "takeoff" or phase == "hover" or phase == "yaw":
            target_altitude = TARGET_ALTITUDE
        else:
            target_altitude = START_HEIGHT

        if step % CONTROL_STEPS == 0:
            total_thrust = MASS * 9.81 + altitude_pid.update(target_altitude - position[2], vertical_velocity)
            if phase == "land" and position[2] > 0.25:
                total_thrust = min(total_thrust, MASS * 9.81 * 0.60)
            pwm = engine.pwm_from_thrust(clamp(total_thrust / 4, 0.0, MODEL.max_thrust_per_motor_n))
            torque = attitude_controller.update(read_imu(drone), yaw_target)
        flight_step = engine.step(drone, pwm, torque)

        position = flight_step.state.position_m
        peak_altitude = max(peak_altitude, position[2])
        imu = read_imu(drone)
        (_, _, yaw), (_, _, yaw_rate) = imu.roll_pitch_yaw_rad, imu.angular_velocity_body_rad_s
        if phase == "takeoff" and position[2] > TARGET_ALTITUDE - 0.08:
            phase, phase_started = "hover", now
        elif phase == "hover" and now - phase_started >= HOVER_SECONDS:
            phase, phase_started, yaw_target = "yaw", now, wrap_angle(initial_yaw + YAW_TARGET_OFFSET)
        elif phase == "yaw" and abs(wrap_angle(yaw_target - yaw)) < 0.06 and abs(yaw_rate) < 0.12:
            phase, phase_started = "land", now
            altitude_pid.reset()
        elif phase == "land" and position[2] < 0.12 and abs(vertical_velocity) < 0.15:
            assert peak_altitude < TARGET_ALTITUDE + 0.5, "Altitude controller overshot its target"
            print(f"Landed after {now:.1f} s at yaw {yaw:.2f} rad; peak altitude {peak_altitude:.2f} m")
            return

        if gui:
            draw_force_vectors(drone, flight_step, force_lines)
            text_id = p.addUserDebugText(
                f"Automatic phase: {phase}\n"
                f"Wind: {engine.settings.wind_world_mps} m/s ({'on' if engine.settings.wind_enabled else 'off'})\n"
                f"Air-relative velocity: {tuple(round(value, 2) for value in engine._air_velocity_body(drone, flight_step.state))} m/s\n"
                f"Drag: {tuple(round(value, 3) for value in flight_step.drag_force_body_n)} N\n"
                + state_text(flight_step, True),
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
    parser.add_argument("--wind-gui", action="store_true", help="Open live wind controls during the PyBullet flight")
    parser.add_argument("--max-seconds", type=float, default=MAX_SECONDS)
    args = parser.parse_args()
    client = p.connect(p.DIRECT if args.headless else p.GUI)
    try:
        run(not args.headless, args.max_seconds, args.wind_gui and not args.headless)
    finally:
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
