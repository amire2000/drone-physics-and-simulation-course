"""Tune the altitude PID live and compare true and noisy altitude readings."""

import argparse
import csv
import json
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pybullet as p

from drone_control import (
    CONTROL_STEPS,
    MASS,
    START_HEIGHT,
    TIME_STEP,
    attitude_torque,
    clamp,
    create_world,
    draw_force_vectors,
    make_controllers,
    pwm_from_thrust,
    step_drone,
)
from pid import PID

TARGET_ALTITUDE = 3.0
DEFAULT_GAINS = (0.7, 0.05, 1.1)
PLOT_HZ = 20
PRESETS = {
    "Stable hover": (TARGET_ALTITUDE, DEFAULT_GAINS, 0.0),
    "Gentle response": (TARGET_ALTITUDE, (0.35, 0.02, 0.7), 0.0),
    "Aggressive response": (TARGET_ALTITUDE, (2.0, 0.1, 0.1), 0.0),
    "Noisy sensor": (TARGET_ALTITUDE, DEFAULT_GAINS, 0.05),
}


@dataclass
class Telemetry:
    time: list[float] = field(default_factory=list)
    target: list[float] = field(default_factory=list)
    altitude: list[float] = field(default_factory=list)
    measured_altitude: list[float] = field(default_factory=list)
    thrust: list[float] = field(default_factory=list)
    proportional: list[float] = field(default_factory=list)
    integral: list[float] = field(default_factory=list)
    derivative: list[float] = field(default_factory=list)
    controller_output: list[float] = field(default_factory=list)

    def append(
        self,
        now: float,
        target: float,
        altitude: float,
        measured_altitude: float,
        thrust: float,
        terms: tuple[float, float, float],
        controller_output: float,
    ) -> None:
        self.time.append(now)
        self.target.append(target)
        self.altitude.append(altitude)
        self.measured_altitude.append(measured_altitude)
        self.thrust.append(thrust)
        self.proportional.append(terms[0])
        self.integral.append(terms[1])
        self.derivative.append(terms[2])
        self.controller_output.append(controller_output)


def reset_flight(drone: int, altitude_pid: PID) -> tuple[float, float, float, float]:
    """Reset only the vehicle state so GUI controls remain available."""
    p.resetBasePositionAndOrientation(drone, (0, 0, START_HEIGHT), (0, 0, 0, 1))
    p.resetBaseVelocity(drone, (0, 0, 0), (0, 0, 0))
    altitude_pid.reset()
    return (0.0, 0.0, 0.0, 0.0)


def apply_gains(controller: PID, gains: tuple[float, float, float]) -> None:
    controller.kp, controller.ki, controller.kd = gains
    controller.reset()


def save_run(telemetry: Telemetry, output: str, settings: dict[str, float | int]) -> tuple[Path, Path, Path]:
    base = Path(output).with_suffix("")
    base.parent.mkdir(parents=True, exist_ok=True)
    csv_path = base.with_suffix(".csv")
    png_path = base.with_suffix(".png")
    settings_path = base.with_suffix(".json")
    with csv_path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(("time_s", "target_m", "true_altitude_m", "measured_altitude_m", "collective_thrust_n", "p_term_n", "i_term_n", "d_term_n", "controller_output_n"))
        writer.writerows(zip(telemetry.time, telemetry.target, telemetry.altitude, telemetry.measured_altitude, telemetry.thrust, telemetry.proportional, telemetry.integral, telemetry.derivative, telemetry.controller_output))
    figure, altitude_axis, thrust_axis, controller_axis, lines = make_plot()
    refresh_plot(figure, altitude_axis, thrust_axis, controller_axis, lines, telemetry)
    figure.savefig(png_path, dpi=140, bbox_inches="tight")
    settings_path.write_text(json.dumps(settings, indent=2) + "\n")
    return csv_path, png_path, settings_path


def make_plot():
    import matplotlib.pyplot as plt

    figure, (altitude_axis, controller_axis) = plt.subplots(2, 1, num="Altitude PID tuning", figsize=(9, 7), sharex=True)
    thrust_axis = altitude_axis.twinx()
    target_line, = altitude_axis.plot([], [], "--", color="#2563eb", label="target altitude")
    altitude_line, = altitude_axis.plot([], [], color="#16a34a", label="true altitude")
    measured_line, = altitude_axis.plot([], [], color="#f97316", alpha=0.75, label="measured altitude")
    thrust_line, = thrust_axis.plot([], [], color="#7c3aed", alpha=0.75, label="collective thrust")
    altitude_axis.set_xlabel("time (s)")
    altitude_axis.set_ylabel("altitude (m)")
    thrust_axis.set_ylabel("collective thrust (N)")
    altitude_axis.grid(True, alpha=0.25)
    altitude_axis.legend((target_line, altitude_line, measured_line, thrust_line), ("target altitude", "true altitude", "measured altitude", "collective thrust"), loc="upper left")
    proportional_line, = controller_axis.plot([], [], color="#2563eb", label="P term")
    integral_line, = controller_axis.plot([], [], color="#16a34a", label="I term")
    derivative_line, = controller_axis.plot([], [], color="#f97316", label="D term")
    output_line, = controller_axis.plot([], [], color="#7c3aed", linewidth=2, label="PID output")
    controller_axis.axhline(0, color="#475569", linewidth=0.8)
    controller_axis.set_xlabel("time (s)")
    controller_axis.set_ylabel("controller correction (N)")
    controller_axis.grid(True, alpha=0.25)
    controller_axis.legend(loc="upper left")
    figure.tight_layout()
    return figure, altitude_axis, thrust_axis, controller_axis, (target_line, altitude_line, measured_line, thrust_line, proportional_line, integral_line, derivative_line, output_line)


def refresh_plot(figure, altitude_axis, thrust_axis, controller_axis, lines, telemetry: Telemetry) -> None:
    for line, values in zip(lines, (telemetry.target, telemetry.altitude, telemetry.measured_altitude, telemetry.thrust, telemetry.proportional, telemetry.integral, telemetry.derivative, telemetry.controller_output)):
        line.set_data(telemetry.time, values)
    altitude_axis.relim()
    altitude_axis.autoscale_view()
    thrust_axis.relim()
    thrust_axis.autoscale_view()
    controller_axis.relim()
    controller_axis.autoscale_view()
    figure.canvas.draw_idle()
    figure.canvas.flush_events()


def make_sliders(target: float, gains: tuple[float, float, float], noise_sigma: float, existing: dict[str, int] | None = None) -> dict[str, int]:
    if existing:
        for slider in existing.values():
            p.removeUserDebugItem(slider)
    return {
        "target": p.addUserDebugParameter("Target altitude (m)", 0.05, 5.0, target),
        "kp": p.addUserDebugParameter("Altitude Kp", 0.0, 10.0, gains[0]),
        "ki": p.addUserDebugParameter("Altitude Ki", 0.0, 10.0, gains[1]),
        "kd": p.addUserDebugParameter("Altitude Kd", 0.0, 10.0, gains[2]),
        "noise": p.addUserDebugParameter("Altitude noise sigma (m)", 0.0, 0.25, noise_sigma),
    }


def make_control_panel() -> tuple[object, dict[str, object]]:
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, RadioButtons

    state: dict[str, object] = {"running": False, "exit": False, "reset": False, "preset": None}
    figure = plt.figure("Simulation controls", figsize=(6.5, 3.2))
    start = Button(figure.add_axes((0.06, 0.67, 0.18, 0.18)), "Start")
    pause = Button(figure.add_axes((0.28, 0.67, 0.18, 0.18)), "Stop")
    reset = Button(figure.add_axes((0.50, 0.67, 0.18, 0.18)), "Reset")
    exit_button = Button(figure.add_axes((0.72, 0.67, 0.18, 0.18)), "Exit")
    radio = RadioButtons(figure.add_axes((0.08, 0.08, 0.82, 0.48)), tuple(PRESETS))
    start.on_clicked(lambda _: state.update(running=True))
    pause.on_clicked(lambda _: state.update(running=False))
    reset.on_clicked(lambda _: state.update(reset=True))
    exit_button.on_clicked(lambda _: state.update(exit=True))
    radio.on_clicked(lambda label: state.update(preset=label))
    figure.canvas.mpl_connect("close_event", lambda _: state.update(exit=True))
    state["widgets"] = (start, pause, reset, exit_button, radio)
    return figure, state


def simulate(seconds: float, target: float, gains: tuple[float, float, float], noise_sigma: float, seed: int, gui: bool = False, output: str | None = None) -> Telemetry:
    """Run one repeatable flight. GUI mode supplies live sliders and a plot."""
    drone = create_world()
    altitude_pid = PID(*gains, integral_limit=0.4)
    roll_pid, pitch_pid, yaw_pid = make_controllers()
    attitude_pids = (roll_pid, pitch_pid, yaw_pid)
    yaw_target = 0.0
    motor_rpms = (0.0, 0.0, 0.0, 0.0)
    torque = (0.0, 0.0, 0.0)
    total_thrust = MASS * 9.81
    pwm = pwm_from_thrust(total_thrust / 4)
    rng = np.random.default_rng(seed)
    telemetry = Telemetry()
    plot_state = None
    sliders = None
    control_figure = None
    control_state = None
    gains_before = gains
    force_lines = [-1, -1, -1, -1]
    pid_terms = (0.0, 0.0, 0.0)
    controller_output = 0.0

    if gui:
        import matplotlib.pyplot as plt

        plt.ion()
        plot_state = make_plot()
        control_figure, control_state = make_control_panel()
        plt.show(block=False)
        p.resetDebugVisualizerCamera(cameraDistance=2.2, cameraYaw=45, cameraPitch=-25, cameraTargetPosition=(0, 0, 0.8))
        sliders = make_sliders(target, gains, noise_sigma)

    total_steps = round(seconds / TIME_STEP) if not gui else None
    step = 0
    while p.isConnected() and (total_steps is None or step < total_steps) and (not gui or not control_state["exit"]):
        if gui:
            control_figure.canvas.flush_events()
            preset = control_state["preset"]
            if preset:
                target, gains, noise_sigma = PRESETS[preset]
                sliders = make_sliders(target, gains, noise_sigma, sliders)
                apply_gains(altitude_pid, gains)
                gains_before = gains
                control_state["preset"] = None
                control_state["reset"] = True
            if control_state["reset"]:
                motor_rpms = reset_flight(drone, altitude_pid)
                telemetry = Telemetry()
                rng = np.random.default_rng(seed)
                pid_terms = (0.0, 0.0, 0.0)
                controller_output = 0.0
                step = 0
                control_state["reset"] = False
            if not control_state["running"]:
                time.sleep(0.02)
                continue

        now = step * TIME_STEP
        if gui and step % CONTROL_STEPS == 0:
            target = p.readUserDebugParameter(sliders["target"])
            noise_sigma = p.readUserDebugParameter(sliders["noise"])
            gains = tuple(p.readUserDebugParameter(sliders[name]) for name in ("kp", "ki", "kd"))
            if gains != gains_before:
                apply_gains(altitude_pid, gains)
                gains_before = gains

        position, _ = p.getBasePositionAndOrientation(drone)
        altitude = position[2]
        vertical_velocity = p.getBaseVelocity(drone)[0][2]
        if step % CONTROL_STEPS == 0:
            measured_altitude = altitude + rng.normal(0.0, noise_sigma)
            pid_terms = altitude_pid.update_terms(target - measured_altitude, vertical_velocity)
            controller_output = sum(pid_terms)
            total_thrust = MASS * 9.81 + controller_output
            pwm = pwm_from_thrust(clamp(total_thrust / 4, 0.0, MASS * 9.81))
            torque = attitude_torque(drone, attitude_pids, yaw_target)
        motor_rpms, motor_thrusts, total_thrust = step_drone(drone, pwm, torque, motor_rpms)

        if step % max(1, round(1 / (PLOT_HZ * TIME_STEP))) == 0:
            position, _ = p.getBasePositionAndOrientation(drone)
            altitude = position[2]
            measured_altitude = altitude + rng.normal(0.0, noise_sigma)
            telemetry.append(now, target, altitude, measured_altitude, total_thrust, pid_terms, controller_output)
            if gui:
                refresh_plot(*plot_state, telemetry)

        if gui:
            draw_force_vectors(drone, motor_thrusts, force_lines)
            time.sleep(TIME_STEP)
        step += 1

    if output:
        csv_path, png_path, settings_path = save_run(
            telemetry,
            output,
            {
                "target_altitude_m": target,
                "kp": gains[0],
                "ki": gains[1],
                "kd": gains[2],
                "altitude_noise_sigma_m": noise_sigma,
                "seed": seed,
            },
        )
        print(f"Saved {csv_path}, {png_path}, and {settings_path}")
    return telemetry


def self_check() -> None:
    clean = simulate(12.0, TARGET_ALTITUDE, DEFAULT_GAINS, 0.0, seed=7)
    assert max(clean.altitude) < 3.5, "Default altitude PID overshot the safe limit"
    assert all(abs(value - TARGET_ALTITUDE) <= 0.15 for value in clean.altitude[-60:]), "Default altitude PID did not hold 3 m"
    noisy_a = simulate(2.0, TARGET_ALTITUDE, DEFAULT_GAINS, 0.03, seed=11)
    noisy_b = simulate(2.0, TARGET_ALTITUDE, DEFAULT_GAINS, 0.03, seed=11)
    noisy_c = simulate(2.0, TARGET_ALTITUDE, DEFAULT_GAINS, 0.03, seed=12)
    assert noisy_a.measured_altitude == noisy_b.measured_altitude, "A seed must reproduce altitude noise"
    assert noisy_a.measured_altitude != noisy_c.measured_altitude, "Different seeds must change altitude noise"
    drone = create_world()
    p.disconnect()
    draw_force_vectors(drone, (0.0, 0.0, 0.0, 0.0), [-1, -1, -1, -1])
    print("PID tuning hover self-check passed")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--self-check", action="store_true")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--target", type=float, default=TARGET_ALTITUDE)
    parser.add_argument("--kp", type=float, default=DEFAULT_GAINS[0])
    parser.add_argument("--ki", type=float, default=DEFAULT_GAINS[1])
    parser.add_argument("--kd", type=float, default=DEFAULT_GAINS[2])
    parser.add_argument("--noise-sigma", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--output")
    args = parser.parse_args()
    if args.headless or args.self_check:
        import matplotlib

        matplotlib.use("Agg")
    client = p.connect(p.DIRECT if args.headless or args.self_check else p.GUI)
    try:
        if args.self_check:
            self_check()
        else:
            simulate(args.seconds, args.target, (args.kp, args.ki, args.kd), args.noise_sigma, args.seed, gui=not args.headless, output=args.output)
    finally:
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
