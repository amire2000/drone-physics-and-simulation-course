"""Print and validate the Topic 0 real-reference vehicle profile."""

from pathlib import Path
import sys

EXAMPLES_ROOT = Path(__file__).resolve().parents[2]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.drone_model import load_drone_profile


def main() -> None:
    """Print the physical, actuator, battery, and propeller reference values."""
    profile = load_drone_profile("real_reference")
    model = profile.model
    battery = model.battery
    print(f"vehicle: {profile.name}")
    print(f"mass: {model.mass_kg:.3f} kg")
    print(f"inertia: {model.inertia_kg_m2} kg m^2")
    print(f"rotor positions: {model.rotor_positions_m} m")
    print(f"motor: {model.motor_kv_rpm_per_v:.0f} KV, {model.max_thrust_per_motor_n:.2f} N max model thrust")
    print(f"battery: {battery.cell_count}S {battery.capacity_ah:.2f} Ah, {battery.nominal_voltage_v:.1f} V nominal")
    print(f"propeller: {profile.propeller.name}, {profile.propeller.diameter_in:.1f} x {profile.propeller.pitch_in:.1f} in")
    assert len(model.rotor_positions_m) == 4
    assert abs(model.mass_kg - 0.62) < 1e-9
    print("real-reference profile self-check passed")


if __name__ == "__main__":
    main()
