"""Plot real-drone force behavior without importing PyBullet."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, replace
from pathlib import Path
import sys

import numpy as np

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.battery import BatteryModel
from common.drone_model import DroneModel, DroneProfile, load_drone_profile
from common.pid import PID


PWM_MIN, PWM_MAX = 1000.0, 2000.0
TARGET_ALTITUDE_M = 3.0
SCENARIOS = ("gravity", "thrust", "torque", "yaw", "drag", "wind", "battery", "altitude-pid")
SETUPS = (
    ("reference", 1750.0, 6),
    ("low-voltage", 1750.0, 4),
    ("higher-kv-4s", 2300.0, 4),
)


@dataclass(frozen=True)
class SimulationOptions:
    """Parameters that can be changed without changing the simulator code."""

    mass_kg: float
    motor_kv_rpm_per_v: float
    battery_cells: int
    wind_world_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    drag_scale: float = 1.0
    kp: float = 0.7


def reference_options(profile: DroneProfile) -> SimulationOptions:
    """Return playground values copied from the selected real-drone profile."""
    return SimulationOptions(
        profile.model.mass_kg,
        profile.model.motor_kv_rpm_per_v,
        profile.model.battery.cell_count,
        profile.physics_settings.wind_world_mps,
    )


def _empty_trace() -> dict[str, list[float]]:
    """Create the named time-series channels used by every scenario."""
    return {name: [] for name in ("time_s", "x_m", "z_m", "vx_mps", "vz_mps", "roll_rad", "yaw_rad", "roll_rate_rad_s", "yaw_rate_rad_s", "thrust_n", "command_thrust_n", "drag_n", "torque_nm", "voltage_v", "rpm", "target_rpm", "command_pwm_us", "soc", "pid_p_n", "pid_i_n", "pid_d_n")}


def _append(trace: dict[str, list[float]], **values: float) -> None:
    """Append one scalar sample to each supplied trace channel."""
    for name, value in values.items():
        trace[name].append(float(value))


def thrust_from_pwm(pwm_us: float, model: DroneModel, thrust_coefficient: float | None = None) -> float:
    """Map PWM to one-motor thrust without importing the PyBullet engine."""
    coefficient = model.thrust_coefficient if thrust_coefficient is None else thrust_coefficient
    normalized = np.clip((pwm_us - PWM_MIN) / (PWM_MAX - PWM_MIN), 0.0, 1.0)
    rotor_rpm = model.max_rpm * normalized
    return coefficient * rotor_rpm**2


def pwm_from_thrust(thrust_n: float, model: DroneModel, thrust_coefficient: float | None = None) -> float:
    """Map one-motor thrust to PWM without importing the PyBullet engine."""
    coefficient = model.thrust_coefficient if thrust_coefficient is None else thrust_coefficient
    max_thrust = coefficient * model.max_rpm**2
    normalized = np.sqrt(np.clip(thrust_n, 0.0, max_thrust) / max_thrust)
    return PWM_MIN + (PWM_MAX - PWM_MIN) * normalized


def simulate(profile: DroneProfile, scenario: str, seconds: float, options: SimulationOptions) -> dict[str, list[float]]:
    """Run one semi-implicit-Euler experiment using the real vehicle profile."""
    if scenario not in SCENARIOS:
        raise ValueError(f"unknown scenario '{scenario}'")
    if seconds <= 0.0:
        raise ValueError("seconds must be positive")

    battery = replace(profile.model.battery, cell_count=options.battery_cells)
    model = replace(
        profile.model,
        mass_kg=options.mass_kg,
        motor_kv_rpm_per_v=options.motor_kv_rpm_per_v,
        battery=battery,
    )
    # Keep the reference propeller calibration fixed while KV and pack voltage vary.
    thrust_coefficient = profile.model.thrust_coefficient
    settings = profile.physics_settings
    dt = 1.0 / settings.physics_hz
    inertia = np.array(model.inertia_kg_m2[:3], dtype=float)
    state_position = np.array((0.0, 0.0, 0.05), dtype=float)
    state_velocity = np.zeros(3)
    angles = np.zeros(3)
    rates = np.zeros(3)
    motor_rpm = 0.0
    battery = BatteryModel(model.battery)
    altitude_pid = PID(options.kp, 0.05, 1.1, integral_limit=0.4)
    trace = _empty_trace()
    total_steps = round(seconds / dt)

    for step in range(total_steps):
        now = step * dt
        if scenario == "gravity":
            pwm, applied_torque, target_altitude = PWM_MIN, np.zeros(3), None
        elif scenario == "thrust":
            pwm, applied_torque, target_altitude = 1550.0, np.zeros(3), None
        elif scenario == "torque":
            pwm = 1500.0
            applied_torque = np.array((0.025 if now < 0.4 else 0.0, 0.0, 0.0))
            target_altitude = None
        elif scenario == "yaw":
            pwm = 1500.0
            applied_torque = np.array((0.0, 0.0, 0.015 if now < 0.4 else 0.0))
            target_altitude = None
        elif scenario == "battery":
            pwm, applied_torque, target_altitude = 1750.0, np.zeros(3), None
        elif scenario == "altitude-pid":
            target_altitude = TARGET_ALTITUDE_M
            p_term, i_term, d_term = altitude_pid.update_terms(target_altitude - state_position[2], state_velocity[2], dt)
            correction = p_term + i_term + d_term
            available_thrust = thrust_coefficient * model.max_rpm**2
            pwm = pwm_from_thrust(
                np.clip(model.mass_kg * 9.81 + correction, 0.0, 4.0 * available_thrust) / 4.0,
                model,
                thrust_coefficient,
            )
            applied_torque = np.zeros(3)
        else:
            pwm, applied_torque, target_altitude = 1500.0, np.zeros(3), None

        if scenario == "wind":
            wind = np.array(options.wind_world_mps)
        else:
            wind = np.zeros(3)
        requested_motor_thrust = thrust_from_pwm(pwm, model, thrust_coefficient)
        target_rpm = np.sqrt(max(requested_motor_thrust, 0.0) / thrust_coefficient)
        fraction = np.clip(target_rpm / model.max_rpm, 0.0, 1.0)
        battery_state = battery.step((fraction,) * 4, dt)
        target_rpm *= battery_state.bus_voltage_v / model.battery.nominal_voltage_v
        target_rpm *= battery_state.motor_command_scale
        alpha = min(1.0, dt / model.motor_time_constant_s)
        motor_rpm += alpha * (target_rpm - motor_rpm)
        motor_thrust = thrust_coefficient * motor_rpm**2
        total_thrust = 4.0 * motor_thrust if scenario != "gravity" else 0.0

        relative_velocity = state_velocity - wind
        speed = float(np.linalg.norm(relative_velocity))
        cd_area = np.array(settings.body_drag_cd_area_m2) * options.drag_scale
        body_drag = -0.5 * settings.air_density_kg_m3 * cd_area * speed * relative_velocity
        rotor_drag = -model.rotor_drag_coefficient * 4.0 * motor_rpm * relative_velocity
        drag = body_drag + rotor_drag if scenario in {"drag", "wind", "altitude-pid", "thrust", "battery"} else np.zeros(3)
        force = drag + np.array((0.0, 0.0, total_thrust - model.mass_kg * 9.81))
        acceleration = force / model.mass_kg
        state_velocity += acceleration * dt
        state_position += state_velocity * dt

        damping = np.array(settings.angular_damping_nm_per_rad_s) * rates
        angular_acceleration = (applied_torque - damping) / inertia
        rates += angular_acceleration * dt
        angles += rates * dt

        _append(
            trace,
            time_s=now,
            x_m=state_position[0],
            z_m=state_position[2],
            vx_mps=state_velocity[0],
            vz_mps=state_velocity[2],
            roll_rad=angles[0],
            yaw_rad=angles[2],
            roll_rate_rad_s=rates[0],
            yaw_rate_rad_s=rates[2],
            thrust_n=total_thrust,
            command_thrust_n=4.0 * thrust_coefficient * target_rpm**2,
            drag_n=float(np.linalg.norm(drag)),
            torque_nm=float(np.linalg.norm(applied_torque)),
            voltage_v=battery_state.bus_voltage_v,
            rpm=motor_rpm,
            target_rpm=target_rpm,
            command_pwm_us=pwm,
            soc=battery_state.state_of_charge,
            pid_p_n=p_term if scenario == "altitude-pid" else 0.0,
            pid_i_n=i_term if scenario == "altitude-pid" else 0.0,
            pid_d_n=d_term if scenario == "altitude-pid" else 0.0,
        )
    return trace


def save_csv(trace: dict[str, list[float]], path: Path) -> None:
    """Write one trace with units encoded in its column names."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        names = list(trace)
        writer.writerow(names)
        writer.writerows(zip(*(trace[name] for name in names)))


def save_plot(trace: dict[str, list[float]], scenario: str, path: Path, baseline: dict[str, list[float]] | None = None) -> None:
    """Save three focused graphs for one reduced-order experiment."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    time_s = trace["time_s"]
    figure, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    rpm_axis = axes[2].twinx()
    for source, style, label in ((baseline, "--", "reference"), (trace, "-", "active")):
        if source is None:
            continue
        axes[0].plot(source["time_s"], source["z_m"], style, label=f"{label} altitude")
        axes[0].plot(source["time_s"], source["vx_mps"], style, label=f"{label} x velocity")
        axes[1].plot(source["time_s"], source["thrust_n"], style, label=f"{label} thrust")
        axes[1].plot(source["time_s"], source["drag_n"], style, label=f"{label} drag magnitude")
        axes[2].plot(source["time_s"], source["voltage_v"], style, label=f"{label} voltage")
        rpm_axis.plot(source["time_s"], source["rpm"], style, label=f"{label} RPM")
    axes[0].set_ylabel("state (m, m/s)")
    axes[1].set_ylabel("force (N)")
    axes[2].set_ylabel("voltage (V)")
    rpm_axis.set_ylabel("RPM")
    axes[2].set_xlabel("time (s)")
    if scenario == "altitude-pid":
        axes[0].axhline(TARGET_ALTITUDE_M, color="black", linestyle=":", label="target altitude 3 m")
    for axis in axes:
        axis.grid(True, alpha=0.25)
    for axis in axes[:2]:
        axis.legend(loc="best")
    axes[2].legend(loc="upper left")
    rpm_axis.legend(loc="upper right")
    figure.suptitle(f"Reduced-order {scenario} experiment")
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=150)
    plt.close(figure)


def compare_setups(profile: DroneProfile, seconds: float, path: Path) -> None:
    """Save one graph and CSV summary comparing the three teaching setups."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    traces = []
    for name, motor_kv, battery_cells in SETUPS:
        options = SimulationOptions(profile.model.mass_kg, motor_kv, battery_cells)
        traces.append((name, motor_kv, battery_cells, simulate(profile, "altitude-pid", seconds, options)))

    figure, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
    rpm_axis = axes[2].twinx()
    colors = ("tab:blue", "tab:orange", "tab:green")
    graph_path = path.with_suffix(".png")
    summary_path = path.with_suffix(".csv")
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    with summary_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("setup", "motor_kv_rpm_per_v", "battery_cells", "peak_altitude_m", "final_altitude_m", "peak_thrust_n", "final_voltage_v", "final_rpm"))
        for (name, motor_kv, battery_cells, trace), color in zip(traces, colors):
            label = f"{name}: {motor_kv:.0f} KV / {battery_cells}S"
            axes[0].plot(trace["time_s"], trace["z_m"], color=color, label=label)
            axes[1].plot(trace["time_s"], trace["thrust_n"], color=color, label=label)
            axes[2].plot(trace["time_s"], trace["voltage_v"], color=color, label=label)
            rpm_axis.plot(trace["time_s"], trace["rpm"], color=color, linestyle="--", label=f"{label} RPM")
            writer.writerow((name, motor_kv, battery_cells, max(trace["z_m"]), trace["z_m"][-1], max(trace["thrust_n"]), trace["voltage_v"][-1], trace["rpm"][-1]))

    axes[0].axhline(TARGET_ALTITUDE_M, color="black", linestyle=":", label="target altitude 3 m")
    axes[0].set_ylabel("altitude (m)")
    axes[1].set_ylabel("total thrust (N)")
    axes[2].set_ylabel("voltage (V)")
    rpm_axis.set_ylabel("RPM; dashed")
    axes[2].set_xlabel("time (s)")
    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best")
    rpm_axis.legend(loc="upper right")
    figure.suptitle("Three motor and battery setup comparison")
    figure.tight_layout()
    figure.savefig(graph_path, dpi=150)
    plt.close(figure)
    print(f"Saved {graph_path} and {summary_path}")
    for name, motor_kv, battery_cells, trace in traces:
        print(f"{name}: {motor_kv:.0f} KV / {battery_cells}S; peak altitude {max(trace['z_m']):.2f} m; final altitude {trace['z_m'][-1]:.2f} m")


def save_motor_lag_plot(trace: dict[str, list[float]], path: Path) -> None:
    """Save a graph showing commanded versus delayed RPM and thrust."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(trace["time_s"], trace["target_rpm"], "--", label="target RPM")
    axes[0].plot(trace["time_s"], trace["rpm"], label="actual RPM")
    axes[1].plot(trace["time_s"], trace["command_thrust_n"], "--", label="requested thrust")
    axes[1].plot(trace["time_s"], trace["thrust_n"], label="actual thrust")
    axes[0].set_ylabel("motor speed (RPM)")
    axes[1].set_ylabel("total thrust (N)")
    axes[1].set_xlabel("time (s)")
    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best")
    figure.suptitle("Motor lag: command versus physical response")
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=150)
    plt.close(figure)
    print(f"Saved {path}")


def interactive(profile: DroneProfile) -> None:
    """Open sliders for mass, motor KV, battery cells, drag, wind, and PID gain."""
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, RadioButtons, Slider

    baseline_options = reference_options(profile)
    figure, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    rpm_axis = axes[2].twinx()
    figure.subplots_adjust(left=0.10, bottom=0.36, hspace=0.35)
    mass_axis = figure.add_axes((0.12, 0.27, 0.32, 0.03))
    kv_axis = figure.add_axes((0.12, 0.22, 0.32, 0.03))
    drag_axis = figure.add_axes((0.12, 0.17, 0.32, 0.03))
    wind_axis = figure.add_axes((0.12, 0.12, 0.32, 0.03))
    kp_axis = figure.add_axes((0.58, 0.27, 0.32, 0.03))
    battery_axis = figure.add_axes((0.58, 0.10, 0.16, 0.12))
    reset_axis = figure.add_axes((0.80, 0.12, 0.12, 0.05))
    mass_slider = Slider(mass_axis, "mass kg", 0.3, 1.2, valinit=baseline_options.mass_kg)
    kv_slider = Slider(kv_axis, "motor KV", 1000.0, 3000.0, valinit=baseline_options.motor_kv_rpm_per_v, valstep=50.0)
    drag_slider = Slider(drag_axis, "drag ×", 0.0, 3.0, valinit=1.0)
    wind_slider = Slider(wind_axis, "wind Y m/s", -10.0, 10.0, valinit=0.0)
    kp_slider = Slider(kp_axis, "Kp", 0.0, 2.0, valinit=baseline_options.kp)
    battery_buttons = RadioButtons(battery_axis, ("4S", "5S", "6S"), active=baseline_options.battery_cells - 4)
    reset_button = Button(reset_axis, "Reset")

    def redraw(_: object = None) -> None:
        """Rerun the active experiment and redraw baseline versus changed curves."""
        battery_cells = int(battery_buttons.value_selected.rstrip("S"))
        active = SimulationOptions(
            mass_slider.val,
            kv_slider.val,
            battery_cells,
            (0.0, wind_slider.val, 0.0),
            drag_slider.val,
            kp_slider.val,
        )
        trace = simulate(profile, "altitude-pid", 12.0, active)
        reference = simulate(profile, "altitude-pid", 12.0, baseline_options)
        for axis in axes:
            axis.clear()
        rpm_axis.clear()
        reference_label = f"reference {baseline_options.motor_kv_rpm_per_v:.0f} KV {baseline_options.battery_cells}S"
        active_label = f"active {kv_slider.val:.0f} KV {battery_cells}S"
        axes[0].plot(reference["time_s"], reference["z_m"], "--", label=f"{reference_label} altitude")
        axes[0].plot(trace["time_s"], trace["z_m"], label=f"{active_label} altitude")
        axes[0].axhline(TARGET_ALTITUDE_M, color="black", linestyle=":", label="target altitude 3 m")
        axes[1].plot(reference["time_s"], reference["thrust_n"], "--", label=f"{reference_label} thrust")
        axes[1].plot(trace["time_s"], trace["thrust_n"], label=f"{active_label} thrust")
        axes[1].plot(trace["time_s"], trace["drag_n"], label="active drag")
        axes[2].plot(reference["time_s"], reference["voltage_v"], "--", label=f"{reference_label} voltage")
        axes[2].plot(trace["time_s"], trace["voltage_v"], label=f"{active_label} voltage")
        rpm_axis.plot(reference["time_s"], reference["rpm"], "--", label=f"{reference_label} RPM")
        rpm_axis.plot(trace["time_s"], trace["rpm"], label=f"{active_label} RPM")
        axes[0].set_ylabel("altitude (m)")
        axes[1].set_ylabel("force (N)")
        axes[2].set_ylabel("voltage (V)")
        rpm_axis.set_ylabel("RPM")
        axes[2].set_xlabel("time (s)")
        for axis in axes:
            axis.grid(True, alpha=0.25)
        for axis in axes[:2]:
            axis.legend(loc="best")
        axes[2].legend(loc="upper left")
        rpm_axis.legend(loc="upper right")
        figure.canvas.draw_idle()

    def reset(_: object) -> None:
        """Restore all sliders to the real-drone reference values."""
        mass_slider.reset()
        kv_slider.reset()
        drag_slider.reset()
        wind_slider.reset()
        kp_slider.reset()
        battery_buttons.set_active(baseline_options.battery_cells - 4)

    for slider in (mass_slider, kv_slider, drag_slider, wind_slider, kp_slider):
        slider.on_changed(redraw)
    battery_buttons.on_clicked(redraw)
    reset_button.on_clicked(reset)
    redraw()
    plt.show()


def self_check(profile: DroneProfile) -> None:
    """Check gravity, thrust, battery sag, and altitude-PID direction."""
    options = reference_options(profile)
    gravity = simulate(profile, "gravity", 0.4, options)
    thrust = simulate(profile, "thrust", 1.0, options)
    battery = simulate(profile, "battery", 2.0, options)
    pid = simulate(profile, "altitude-pid", 8.0, options)
    low_voltage_pid = simulate(profile, "altitude-pid", 8.0, SimulationOptions(1.2, options.motor_kv_rpm_per_v, 4))
    assert gravity["vz_mps"][-1] < -3.5
    assert max(thrust["z_m"]) > 0.05
    assert battery["voltage_v"][-1] < battery["voltage_v"][0]
    assert pid["z_m"][-1] > 2.0
    assert max(low_voltage_pid["z_m"]) > 3.0
    print("Reduced-order simulation self-check passed")


def main() -> None:
    """Run a graphable reduced-order experiment or open its interactive playground."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=SCENARIOS, default="altitude-pid")
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--output", type=Path, default=Path("outputs/06-autonomous-hover/reduced_order"))
    parser.add_argument("--interactive", action="store_true")
    parser.add_argument("--compare-setups", action="store_true")
    parser.add_argument("--motor-lag-plot", action="store_true")
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    profile = load_drone_profile("real_reference")
    if args.self_check:
        self_check(profile)
        return
    if args.interactive:
        interactive(profile)
        return
    if args.compare_setups:
        compare_setups(profile, args.seconds, args.output)
        return
    if args.motor_lag_plot:
        trace = simulate(profile, "thrust", args.seconds, reference_options(profile))
        save_motor_lag_plot(trace, args.output.with_suffix(".png"))
        return
    trace = simulate(profile, args.scenario, args.seconds, reference_options(profile))
    save_csv(trace, args.output.with_suffix(".csv"))
    save_plot(trace, args.scenario, args.output.with_suffix(".png"))
    print(f"Saved {args.output.with_suffix('.csv')} and {args.output.with_suffix('.png')}")


if __name__ == "__main__":
    main()
