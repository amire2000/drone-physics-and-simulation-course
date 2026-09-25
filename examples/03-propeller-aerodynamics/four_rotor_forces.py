"""Calculate collective thrust and X-frame roll, pitch, and yaw torque."""

import argparse
from pathlib import Path
import sys

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.drone_control import ARM_OFFSET, KF, KM, MOTOR_YAW_SIGNS

POSITIONS = ((ARM_OFFSET, ARM_OFFSET), (ARM_OFFSET, -ARM_OFFSET), (-ARM_OFFSET, ARM_OFFSET), (-ARM_OFFSET, -ARM_OFFSET))


def force_and_torque(thrusts: tuple[float, float, float, float]) -> tuple[float, tuple[float, float, float]]:
    total = sum(thrusts)
    roll = sum(y * thrust for (_, y), thrust in zip(POSITIONS, thrusts))
    pitch = sum(-x * thrust for (x, _), thrust in zip(POSITIONS, thrusts))
    yaw = sum(sign * KM / KF * thrust for sign, thrust in zip(MOTOR_YAW_SIGNS, thrusts))
    return total, (roll, pitch, yaw)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("thrusts", nargs="*", type=float, default=[1.6, 1.6, 1.6, 1.6], help="T1 T2 T3 T4 in newtons")
    args = parser.parse_args()
    if len(args.thrusts) != 4:
        parser.error("provide exactly four rotor thrusts in newtons")
    total, torque = force_and_torque(tuple(args.thrusts))
    print(f"Rotor thrusts: {', '.join(f'{value:.3f}' for value in args.thrusts)} N")
    print(f"Collective thrust: {total:.3f} N")
    print(f"Torque: roll={torque[0]:.4f}, pitch={torque[1]:.4f}, yaw={torque[2]:.4f} N m")
    equal_total, equal_torque = force_and_torque((1.0, 1.0, 1.0, 1.0))
    assert equal_total == 4.0 and abs(equal_torque[0]) < 1e-12 and abs(equal_torque[1]) < 1e-12 and abs(equal_torque[2]) < 1e-12


if __name__ == "__main__":
    main()
