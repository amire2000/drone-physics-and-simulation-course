"""Compare the quadratic thrust model with ideal momentum theory."""

import argparse
import math
from pathlib import Path
import sys

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.drone_model import DEFAULT_DRONE_MODEL, DEFAULT_PHYSICS_SETTINGS


def momentum_induced_velocity(thrust_n: float, diameter_m: float, density: float = 1.225) -> float:
    area = math.pi * (diameter_m / 2) ** 2
    return math.sqrt(thrust_n / (2 * density * area))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--diameter", type=float, default=0.127, help="Rotor diameter in metres")
    args = parser.parse_args()
    assert args.diameter > 0
    hover_total = DEFAULT_DRONE_MODEL.mass_kg * abs(DEFAULT_PHYSICS_SETTINGS.gravity_z_mps2)
    hover_per_rotor = hover_total / 4
    rpm = math.sqrt(hover_per_rotor / DEFAULT_DRONE_MODEL.thrust_coefficient)
    induced = momentum_induced_velocity(hover_per_rotor, args.diameter)
    disk_area = math.pi * (args.diameter / 2) ** 2
    print(f"Drone hover weight: {hover_total:.3f} N")
    print(f"Hover thrust per rotor: {hover_per_rotor:.3f} N")
    print(f"Quadratic model RPM per rotor: {rpm:.0f} RPM ({rpm / DEFAULT_DRONE_MODEL.max_rpm:.1%} of maximum)")
    print(f"Rotor disk area: {disk_area:.4f} m^2")
    print(f"Ideal hover induced velocity: {induced:.3f} m/s")
    assert induced > 0 and disk_area > 0


if __name__ == "__main__":
    main()
