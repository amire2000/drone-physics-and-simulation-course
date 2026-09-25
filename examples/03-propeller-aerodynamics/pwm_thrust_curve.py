"""Sweep the course PWM model and print the resulting RPM and thrust."""

import argparse
import math
from pathlib import Path
import sys

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.drone_control import MAX_RPM, MAX_THRUST_PER_MOTOR, PWM_MAX, PWM_MIN, thrust_from_pwm


def rpm_from_pwm(pwm: float) -> float:
    normalized = max(0.0, min(1.0, (pwm - PWM_MIN) / (PWM_MAX - PWM_MIN)))
    return MAX_RPM * math.sqrt(normalized)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=11)
    args = parser.parse_args()
    assert args.steps >= 2
    print("PWM (us) | RPM | thrust (N)")
    for index in range(args.steps):
        pwm = PWM_MIN + (PWM_MAX - PWM_MIN) * index / (args.steps - 1)
        print(f"{pwm:8.0f} | {rpm_from_pwm(pwm):5.0f} | {thrust_from_pwm(pwm):9.3f}")
    print(f"Maximum single-rotor thrust: {MAX_THRUST_PER_MOTOR:.3f} N")
    assert abs(thrust_from_pwm(PWM_MIN)) < 1e-12
    assert abs(thrust_from_pwm(PWM_MAX) - MAX_THRUST_PER_MOTOR) < 1e-12


if __name__ == "__main__":
    main()
