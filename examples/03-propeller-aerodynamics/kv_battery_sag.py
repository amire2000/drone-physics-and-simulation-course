"""Compare battery voltage sag, KV-limited RPM, and thrust for one drone profile."""

import argparse
from dataclasses import replace
from pathlib import Path
import sys

import matplotlib.pyplot as plt

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.battery import BatteryModel, BatterySpec
from common.drone_model import load_drone_profile


def simulate(spec: BatterySpec, motor_kv_rpm_per_v: float, thrust_coefficient: float, seconds: float, throttle: float, initial_state_of_charge: float) -> tuple[list[float], list[float], list[float], list[float]]:
    """Simulate one constant-throttle pack and return time, voltage, RPM, and total thrust."""
    battery = BatteryModel(spec)
    battery.reset(initial_state_of_charge)
    time_step_s = 1.0 / 240.0
    times, voltages, rpms, thrusts = [], [], [], []
    for step in range(round(seconds / time_step_s) + 1):
        state = battery.step((throttle,) * 4, time_step_s)
        rpm = throttle * state.motor_command_scale * motor_kv_rpm_per_v * state.bus_voltage_v
        total_thrust_n = 4.0 * thrust_coefficient * rpm**2
        times.append(step * time_step_s)
        voltages.append(state.bus_voltage_v)
        rpms.append(rpm)
        thrusts.append(total_thrust_n)
    return times, voltages, rpms, thrusts


def main() -> None:
    """Plot fresh, low-charge, and high-resistance battery behavior for one profile."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default="default", help="vehicle profile name")
    parser.add_argument("--seconds", type=float, default=60.0, help="simulation duration")
    parser.add_argument("--throttle", type=float, default=0.7, help="constant motor command fraction from 0 to 1")
    parser.add_argument("--output", type=Path, default=Path("outputs/kv_battery_sag.png"), help="PNG output path")
    args = parser.parse_args()
    if args.seconds <= 0.0 or not 0.0 <= args.throttle <= 1.0:
        parser.error("seconds must be positive and throttle must be from 0 to 1")

    profile = load_drone_profile(args.profile)
    model, spec = profile.model, profile.model.battery
    cases = {
        "fresh pack": (spec, 1.0),
        "25% charge": (spec, 0.25),
        "2x resistance": (replace(spec, internal_resistance_ohm=2.0 * spec.internal_resistance_ohm), 1.0),
    }
    figure, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    first_voltages: dict[str, float] = {}
    for label, (case_spec, state_of_charge) in cases.items():
        times, voltages, rpms, thrusts = simulate(case_spec, model.motor_kv_rpm_per_v, model.thrust_coefficient, args.seconds, args.throttle, state_of_charge)
        first_voltages[label] = voltages[0]
        axes[0].plot(times, voltages, label=label)
        axes[1].plot(times, rpms, label=label)
        axes[2].plot(times, thrusts, label=label)

    axes[0].set_ylabel("bus voltage (V)")
    axes[1].set_ylabel("motor RPM")
    axes[2].set(xlabel="time (s)", ylabel="total thrust (N)")
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend()
    figure.suptitle(f"KV and battery sag: {profile.name}, throttle {args.throttle:.0%}")
    figure.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=160)
    plt.close(figure)
    assert first_voltages["2x resistance"] < first_voltages["fresh pack"]
    assert first_voltages["25% charge"] < first_voltages["fresh pack"]
    print(f"Saved plot: {args.output}")
    print(f"Fresh bus voltage: {first_voltages['fresh pack']:.2f} V")
    print(f"Low-charge bus voltage: {first_voltages['25% charge']:.2f} V")
    print(f"High-resistance bus voltage: {first_voltages['2x resistance']:.2f} V")


if __name__ == "__main__":
    main()
