"""Normalize a propeller listing and suggest a thrust coefficient from a complete test row."""

import argparse
from math import pi
from pathlib import Path
import sys

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.drone_model import PropellerSpec, load_drone_profile


def thrust_coefficient(thrust_g: float, rpm: float) -> float:
    """Convert thrust in grams at one measured RPM into a candidate N/RPM² value."""
    return thrust_g * 9.80665 / 1000.0 / rpm**2


def listing_from_arguments(args: argparse.Namespace) -> PropellerSpec:
    """Use command-line values when supplied, otherwise reuse one vehicle profile listing."""
    profile_propeller = load_drone_profile(args.profile).propeller
    supplied_listing = any(
        value is not None
        for value in (args.name, args.diameter_in, args.pitch_in, args.blades, args.rotation_set, args.material, args.source)
    )
    if supplied_listing:
        return PropellerSpec(
            name=args.name or "Unnamed listing propeller",
            diameter_in=args.diameter_in if args.diameter_in is not None else profile_propeller.diameter_in,
            pitch_in=args.pitch_in,
            blade_count=args.blades,
            rotation_set=args.rotation_set,
            material=args.material,
            source=args.source,
        )
    return PropellerSpec(
        name=profile_propeller.name,
        diameter_in=profile_propeller.diameter_in,
        pitch_in=profile_propeller.pitch_in,
        blade_count=profile_propeller.blade_count,
        rotation_set=profile_propeller.rotation_set,
        hub_diameter_mm=profile_propeller.hub_diameter_mm,
        shaft_diameter_mm=profile_propeller.shaft_diameter_mm,
        material=profile_propeller.material,
        weight_g=profile_propeller.weight_g,
        package_count=profile_propeller.package_count,
        source=profile_propeller.source,
    )


def print_yaml(spec: PropellerSpec) -> None:
    """Print a copyable YAML fragment while omitting values a listing did not provide."""
    fields = {
        "name": spec.name,
        "diameter_in": spec.diameter_in,
        "pitch_in": spec.pitch_in,
        "blade_count": spec.blade_count,
        "rotation_set": spec.rotation_set,
        "hub_diameter_mm": spec.hub_diameter_mm,
        "shaft_diameter_mm": spec.shaft_diameter_mm,
        "material": spec.material,
        "weight_g": spec.weight_g,
        "package_count": spec.package_count,
        "source": spec.source,
    }
    print("\nCopyable profile fragment:")
    print("propeller:")
    for name, value in fields.items():
        if value is not None:
            print(f"  {name}: {value}")


def main() -> None:
    """Print readable listing data, SI geometry, and an optional suggested test calibration."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default="default", help="profile used for missing listing fields")
    parser.add_argument("--name", help="exact product name from the listing")
    parser.add_argument("--diameter-in", type=float, help="listed diameter in inches")
    parser.add_argument("--pitch-in", type=float, help="listed pitch in inches")
    parser.add_argument("--blades", type=int, help="listed blade count")
    parser.add_argument("--rotation-set", help="listed rotation direction or set")
    parser.add_argument("--material", help="listed propeller material")
    parser.add_argument("--source", help="listing URL or saved product title")
    parser.add_argument("--thrust-g", type=float, help="measured thrust in grams from one test row")
    parser.add_argument("--rpm", type=float, help="measured motor RPM for that same test row")
    parser.add_argument("--voltage-v", type=float, help="test-pack voltage for that same row")
    parser.add_argument("--current-a", type=float, help="test current for that same row")
    parser.add_argument("--motor", help="motor model or KV stated for that same test row")
    parser.add_argument("--self-check", action="store_true", help="verify the course default conversion")
    args = parser.parse_args()
    test_values = (args.thrust_g, args.rpm, args.voltage_v, args.current_a, args.motor)
    if any(value is not None for value in test_values) and any(value is None for value in test_values):
        parser.error("a test row needs --thrust-g, --rpm, --voltage-v, --current-a, and --motor together")
    if args.thrust_g is not None and any(value <= 0.0 for value in (args.thrust_g, args.rpm, args.voltage_v, args.current_a)):
        parser.error("test thrust, RPM, voltage, and current must be positive")

    propeller = listing_from_arguments(args)
    disk_area_m2 = pi * (propeller.diameter_m / 2.0) ** 2
    print(f"Propeller: {propeller.name}")
    print(f"Diameter: {propeller.diameter_in:.2f} in = {propeller.diameter_m:.4f} m")
    print(f"Pitch: {propeller.pitch_in if propeller.pitch_in is not None else 'unknown'} in")
    print(f"Blades: {propeller.blade_count if propeller.blade_count is not None else 'unknown'}")
    print(f"Rotor disk area: {disk_area_m2:.4f} m^2")
    print_yaml(propeller)

    if args.thrust_g is not None:
        coefficient = thrust_coefficient(args.thrust_g, args.rpm)
        print("\nVerified test row:")
        print(f"Motor: {args.motor}; {args.voltage_v:.2f} V, {args.current_a:.2f} A, {args.rpm:.0f} RPM, {args.thrust_g:.1f} g")
        print(f"Suggested thrust coefficient: {coefficient:.3e} N/RPM^2")
        print("Compare it with your profile before manually changing max_thrust_per_motor_n.")

    if args.self_check:
        assert propeller.diameter_m > 0.0 and disk_area_m2 > 0.0
        assert abs(PropellerSpec("five inch", 5.0).diameter_m - 0.127) < 1e-12
        print("Propeller listing self-check passed")


if __name__ == "__main__":
    main()
