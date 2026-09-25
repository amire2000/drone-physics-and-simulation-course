"""Show how CW/CCW rotor reaction torques cancel or create yaw torque."""

import argparse
from pathlib import Path
import sys

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.drone_model import DEFAULT_DRONE_MODEL


def yaw_torque(motor_rpms: tuple[float, float, float, float]) -> float:
    return sum(sign * DEFAULT_DRONE_MODEL.torque_coefficient * rpm**2 for sign, rpm in zip(DEFAULT_DRONE_MODEL.motor_yaw_signs, motor_rpms))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rpm", type=float, default=0.5 * DEFAULT_DRONE_MODEL.max_rpm)
    args = parser.parse_args()
    equal = (args.rpm,) * 4
    yaw_step = (args.rpm * 1.1, args.rpm, args.rpm, args.rpm)
    print(f"Thrust coefficient KF: {DEFAULT_DRONE_MODEL.thrust_coefficient:.3e} N/RPM^2 (course model)")
    print(f"Reaction-torque coefficient KM: {DEFAULT_DRONE_MODEL.torque_coefficient:.3e} N m/RPM^2 (course model)")
    print(f"Equal CW/CCW speeds {equal[0]:.0f} RPM -> yaw torque {yaw_torque(equal):.6f} N m")
    print(f"One rotor 10% faster -> yaw torque {yaw_torque(yaw_step):.6f} N m")
    assert abs(yaw_torque(equal)) < 1e-9
    assert abs(yaw_torque(yaw_step)) > 0


if __name__ == "__main__":
    main()
