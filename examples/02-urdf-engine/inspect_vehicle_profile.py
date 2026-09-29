"""Print the resolved URDF geometry and non-URDF profile settings for one quad."""

import argparse
from pathlib import Path
import sys

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.drone_model import load_drone_profile


def format_vector(values: tuple[float, ...]) -> str:
    """Format one SI vector for concise terminal inspection output."""
    return "(" + ", ".join(f"{value:.4f}" for value in values) + ")"


def main() -> None:
    """Load one profile and show which values come from URDF versus YAML."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", nargs="?", default="default", help="profile name from examples/common/drone_profiles")
    args = parser.parse_args()
    profile = load_drone_profile(args.profile)
    model, settings = profile.model, profile.physics_settings

    print(f"profile: {profile.name}")
    print(f"URDF: {model.urdf_path.name}")
    print(f"URDF mass: {model.mass_kg:.3f} kg")
    print(f"URDF center of mass: {format_vector(model.center_of_mass_m)} m")
    print(f"URDF inertia (ixx, iyy, izz, ixy, ixz, iyz): {format_vector(model.inertia_kg_m2)} kg m^2")
    for index, position in enumerate(model.rotor_positions_m):
        print(f"URDF rotor_{index}: {format_vector(position)} m")
    print(f"profile max thrust per motor: {model.max_thrust_per_motor_n:.3f} N")
    print(f"profile max RPM: {model.max_rpm:.0f}")
    print(f"profile propeller diameter: {settings.propeller_diameter_m:.4f} m")
    assert len(model.rotor_positions_m) == len(model.motor_yaw_signs) == 4


if __name__ == "__main__":
    main()
