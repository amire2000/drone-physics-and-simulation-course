"""Calculate Topic 0 rotor geometry and provisional inertia checks."""

from math import hypot
from pathlib import Path
import sys

EXAMPLES_ROOT = Path(__file__).resolve().parents[2]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.drone_model import load_drone_profile


def main() -> None:
    """Print motor spacing and compare the diagonal with the 240 mm frame target."""
    profile = load_drone_profile("real_reference")
    positions = profile.model.rotor_positions_m
    diagonal = hypot(positions[0][0] - positions[3][0], positions[0][1] - positions[3][1])
    adjacent = hypot(positions[0][0] - positions[1][0], positions[0][1] - positions[1][1])
    print(f"diagonal motor spacing: {diagonal * 1000:.1f} mm")
    print(f"adjacent motor spacing: {adjacent * 1000:.1f} mm")
    print(f"inertia tensor: {profile.model.inertia_kg_m2} kg m^2")
    assert abs(diagonal - 0.24) < 0.001
    print("vehicle geometry self-check passed")


if __name__ == "__main__":
    main()
