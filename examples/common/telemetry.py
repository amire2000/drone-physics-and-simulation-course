"""Shared passive telemetry records and output for Module 6 simulations."""

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from .drone_model import DroneProfile

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
Motors4 = tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class Sample:
    """One simulation observation; ``None`` means the topic has not modeled it yet."""

    time_s: float
    altitude_m: float | None = None
    vertical_velocity_mps: float | None = None
    vertical_acceleration_mps2: float | None = None
    normal_force_n: float | None = None
    total_thrust_n: float | None = None
    motor_rpm_mean: float | None = None
    battery_voltage_v: float | None = None
    pwm_command_us: float | None = None
    motor_kv_rpm_per_v: float | None = None
    nominal_battery_voltage_v: float | None = None
    hover_thrust_target_n: float | None = None
    roll_rad: float | None = None
    pitch_rad: float | None = None
    yaw_rad: float | None = None
    roll_rate_rad_s: float | None = None
    pitch_rate_rad_s: float | None = None
    yaw_rate_rad_s: float | None = None
    roll_torque_nm: float | None = None
    pitch_torque_nm: float | None = None
    yaw_torque_nm: float | None = None
    position_world_m: Vector3 | None = None
    velocity_world_mps: Vector3 | None = None
    acceleration_world_mps2: Vector3 | None = None
    orientation_quaternion: Quaternion | None = None
    angular_velocity_body_rad_s: Vector3 | None = None
    gravity_force_world_n: Vector3 | None = None
    contact_force_world_n: Vector3 | None = None
    thrust_force_world_n: Vector3 | None = None
    net_force_world_n: Vector3 | None = None
    net_torque_body_nm: Vector3 | None = None
    motor_command_us: Motors4 | None = None
    motor_rpm: Motors4 | None = None
    motor_thrust_n: Motors4 | None = None
    motor_reaction_torque_nm: Motors4 | None = None
    battery_current_a: float | None = None
    battery_soc: float | None = None
    target_altitude_m: float | None = None
    altitude_error_m: float | None = None
    pid_p_n: float | None = None
    pid_i_n: float | None = None
    pid_d_n: float | None = None
    controller_output_n: float | None = None
    disturbance_roll_torque_nm: float | None = None
    disturbance_pitch_torque_nm: float | None = None
    controller_roll_torque_nm: float | None = None
    controller_pitch_torque_nm: float | None = None


SCALAR_FIELDS = (
    "time_s",
    "altitude_m",
    "vertical_velocity_mps",
    "vertical_acceleration_mps2",
    "normal_force_n",
    "total_thrust_n",
    "motor_rpm_mean",
    "battery_voltage_v",
    "pwm_command_us",
    "motor_kv_rpm_per_v",
    "nominal_battery_voltage_v",
    "hover_thrust_target_n",
    "roll_rad",
    "pitch_rad",
    "yaw_rad",
    "roll_rate_rad_s",
    "pitch_rate_rad_s",
    "yaw_rate_rad_s",
    "roll_torque_nm",
    "pitch_torque_nm",
    "yaw_torque_nm",
    "battery_current_a",
    "battery_soc",
    "target_altitude_m",
    "altitude_error_m",
    "pid_p_n",
    "pid_i_n",
    "pid_d_n",
    "controller_output_n",
    "disturbance_roll_torque_nm",
    "disturbance_pitch_torque_nm",
    "controller_roll_torque_nm",
    "controller_pitch_torque_nm",
)
VECTOR_FIELDS = {
    "position_world_m": 3,
    "velocity_world_mps": 3,
    "acceleration_world_mps2": 3,
    "orientation_quaternion": 4,
    "angular_velocity_body_rad_s": 3,
    "gravity_force_world_n": 3,
    "contact_force_world_n": 3,
    "thrust_force_world_n": 3,
    "net_force_world_n": 3,
    "net_torque_body_nm": 3,
    "motor_command_us": 4,
    "motor_rpm": 4,
    "motor_thrust_n": 4,
    "motor_reaction_torque_nm": 4,
}


def _columns() -> tuple[str, ...]:
    """Return the stable complete CSV column order."""
    columns = list(SCALAR_FIELDS)
    for field_name, size in VECTOR_FIELDS.items():
        columns.extend(f"{field_name}_{index}" for index in range(size))
    return tuple(columns)


def _value(sample: Sample, field_name: str, index: int | None = None) -> float | None:
    """Read one scalar or indexed vector value from a passive sample."""
    value = getattr(sample, field_name)
    if index is not None:
        return None if value is None else value[index]
    return value


def _rows(samples: Iterable[Sample]) -> Iterable[tuple[float | None, ...]]:
    """Convert samples into rows matching the stable complete CSV schema."""
    for sample in samples:
        row: list[float | None] = [_value(sample, field_name) for field_name in SCALAR_FIELDS]
        for field_name, size in VECTOR_FIELDS.items():
            row.extend(_value(sample, field_name, index) for index in range(size))
        yield tuple(row)


def _plot_value(sample: Sample, field_name: str) -> float | None:
    """Read a graph field while allowing an omitted value to remain absent."""
    return getattr(sample, field_name)


def save_results(
    samples: list[Sample],
    path: Path,
    profile: DroneProfile,
    graph_fields: tuple[str, ...],
    title: str,
    graph_panels: tuple[tuple[str, ...], ...] | None = None,
) -> None:
    """Save the complete telemetry CSV and selected topic graph panels."""
    import matplotlib.pyplot as plt

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.with_suffix(".csv").open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(_columns())
        writer.writerows(_rows(samples))

    times = [sample.time_s for sample in samples]
    panels = graph_panels or tuple((field_name,) for field_name in graph_fields)
    figure, axes = plt.subplots(len(panels), 1, figsize=(8, 2 * len(panels)), sharex=True, squeeze=False)
    for axis, panel in zip(axes[:, 0], panels):
        for field_name in panel:
            values = [_plot_value(sample, field_name) for sample in samples]
            axis.plot(times, values, label=field_name)
            if field_name == "total_thrust_n":
                axis.axhline(
                    profile.model.mass_kg * abs(profile.physics_settings.gravity_z_mps2),
                    color="black",
                    linestyle=":",
                    label="weight",
                )
        axis.set_ylabel("value" if len(panel) > 1 else panel[0].replace("_", " "))
        axis.grid(True, alpha=0.3)
        axis.legend()
    axes[-1, 0].set_xlabel("Time (s)")
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(path, dpi=140)
    plt.close(figure)


def print_summary(samples: list[Sample], title: str = "Simulation summary") -> None:
    """Print only telemetry fields populated by the current topic."""
    if not samples:
        return
    final = samples[-1]
    labels = (
        ("altitude_m", "Altitude", "m"),
        ("vertical_velocity_mps", "Vertical velocity", "m/s"),
        ("vertical_acceleration_mps2", "Vertical acceleration", "m/s²"),
        ("normal_force_n", "Normal force", "N"),
        ("total_thrust_n", "Total thrust", "N"),
        ("motor_rpm_mean", "Mean motor RPM", "RPM"),
        ("battery_voltage_v", "Battery voltage", "V"),
        ("pwm_command_us", "PWM command", "us"),
        ("motor_kv_rpm_per_v", "Motor KV", "RPM/V"),
        ("nominal_battery_voltage_v", "Nominal battery voltage", "V"),
        ("hover_thrust_target_n", "Hover thrust target", "N"),
        ("roll_rad", "Roll", "rad"),
        ("pitch_rad", "Pitch", "rad"),
        ("roll_rate_rad_s", "Roll rate", "rad/s"),
        ("pitch_rate_rad_s", "Pitch rate", "rad/s"),
        ("roll_torque_nm", "Roll torque", "N·m"),
        ("pitch_torque_nm", "Pitch torque", "N·m"),
        ("battery_current_a", "Battery current", "A"),
        ("battery_soc", "Battery state of charge", "%"),
    )
    lines = [f"\n========== {title.upper()} =========="]
    for field_name, label, unit in labels:
        value = getattr(final, field_name)
        if value is not None:
            display_value = value * 100.0 if field_name == "battery_soc" else value
            lines.append(f"{label}: {display_value:.2f} {unit}")
    if len(lines) == 1:
        return
    lines.append("=" * 42)
    print("\n".join(lines))
