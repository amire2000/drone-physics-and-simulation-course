"""Validate gravity, thrust, torque, and wind with the Module 6 physics engine."""

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import pybullet as p

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.drone_model import DEFAULT_DRONE_MODEL, DEFAULT_PHYSICS_SETTINGS, PhysicsSettings
from common.drone_physics import PWM_HOVER, PhysicsEngine, rpm_from_thrust
from common.pybullet_utils import create_world, reset_drone

MODEL = DEFAULT_DRONE_MODEL
SETTINGS = DEFAULT_PHYSICS_SETTINGS
HOVER_RPM = rpm_from_thrust(MODEL.mass_kg * abs(SETTINGS.gravity_z_mps2) / 4)


@dataclass(frozen=True)
class ValidationResult:
    """Measured result and expected direction for one physics validation case."""

    name: str
    measurement: float
    unit: str
    expectation: str


def run_steps(engine: PhysicsEngine, drone: int, seconds: float, pwm_us: float, torque_nm: tuple[float, float, float]) -> None:
    """Advance one fixed motor/torque command for a requested simulation duration."""
    for _ in range(round(seconds / engine.settings.time_step_s)):
        engine.step(drone, pwm_us, torque_nm)


def gravity_validation() -> ValidationResult:
    """Measure free-fall acceleration before the drone reaches the ground."""
    drone = create_world()
    engine = PhysicsEngine()
    reset_drone(drone, (0.0, 0.0, 10.0))
    before = p.getBaseVelocity(drone)[0][2]
    run_steps(engine, drone, 0.4, 1000.0, (0.0, 0.0, 0.0))
    after = p.getBaseVelocity(drone)[0][2]
    acceleration = (after - before) / 0.4
    assert abs(acceleration - SETTINGS.gravity_z_mps2) < 0.15
    return ValidationResult("gravity", acceleration, "m/s²", "near -9.81 m/s²")


def hover_validation() -> ValidationResult:
    """Verify equal hover thrust keeps a primed drone near its initial altitude."""
    drone = create_world()
    engine = PhysicsEngine()
    reset_drone(drone, (0.0, 0.0, 5.0))
    engine.reset((HOVER_RPM,) * 4)
    run_steps(engine, drone, 2.0, PWM_HOVER, (0.0, 0.0, 0.0))
    altitude_error = p.getBasePositionAndOrientation(drone)[0][2] - 5.0
    assert abs(altitude_error) < 0.08
    return ValidationResult("hover", altitude_error, "m", "near 0 m altitude error")


def vertical_validation() -> ValidationResult:
    """Verify equal PWM above hover produces positive vertical velocity."""
    drone = create_world()
    engine = PhysicsEngine()
    reset_drone(drone, (0.0, 0.0, 5.0))
    engine.reset((HOVER_RPM,) * 4)
    run_steps(engine, drone, 0.5, 1650.0, (0.0, 0.0, 0.0))
    velocity = p.getBaseVelocity(drone)[0][2]
    assert velocity > 0.5
    return ValidationResult("vertical acceleration", velocity, "m/s", "positive upward velocity")


def attitude_validation(axis: str, torque_nm: tuple[float, float, float], rate_index: int) -> ValidationResult:
    """Verify a motor-mixer torque request creates angular velocity on one body axis."""
    drone = create_world()
    engine = PhysicsEngine()
    reset_drone(drone, (0.0, 0.0, 5.0))
    engine.reset((HOVER_RPM,) * 4)
    run_steps(engine, drone, 0.25, PWM_HOVER, torque_nm)
    rate = p.getBaseVelocity(drone)[1][rate_index]
    assert rate > 0.05
    return ValidationResult(axis, rate, "rad/s", "positive angular velocity")


def wind_validation() -> ValidationResult:
    """Verify crosswind creates drift in the wind direction while hovering."""
    settings = PhysicsSettings(wind_world_mps=(0.0, 5.0, 0.0))
    drone = create_world(settings=settings)
    engine = PhysicsEngine(settings=settings)
    reset_drone(drone, (0.0, 0.0, 5.0))
    engine.reset((HOVER_RPM,) * 4)
    run_steps(engine, drone, 2.0, PWM_HOVER, (0.0, 0.0, 0.0))
    drift = p.getBasePositionAndOrientation(drone)[0][1]
    assert drift > 0.2
    return ValidationResult("wind", drift, "m", "positive lateral drift")


def run_validation(selected: str) -> list[ValidationResult]:
    """Run one named validation or the complete seven-case validation suite."""
    cases = {
        "gravity": gravity_validation,
        "hover": hover_validation,
        "vertical": vertical_validation,
        "roll": lambda: attitude_validation("roll", (0.02, 0.0, 0.0), 0),
        "pitch": lambda: attitude_validation("pitch", (0.0, 0.02, 0.0), 1),
        "yaw": lambda: attitude_validation("yaw", (0.0, 0.0, 0.002), 2),
        "wind": wind_validation,
    }
    if selected == "all":
        return [case() for case in cases.values()]
    return [cases[selected]()]


def save_plot(results: list[ValidationResult], output: Path) -> None:
    """Save a bar chart of the measured validation values and their units."""
    output.parent.mkdir(parents=True, exist_ok=True)
    figure, axis = plt.subplots(figsize=(10, 5))
    bars = axis.bar([result.name for result in results], [result.measurement for result in results], color="#2563eb")
    axis.axhline(0.0, color="#334155", linewidth=1)
    axis.set_ylabel("measured value (mixed units; see labels)")
    axis.set_title("Module 6 physics-engine validation results")
    axis.tick_params(axis="x", rotation=20)
    for bar, result in zip(bars, results):
        axis.text(bar.get_x() + bar.get_width() / 2, bar.get_height(), f"{result.measurement:.2f} {result.unit}", ha="center", va="bottom")
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)


def main() -> None:
    """Run selected validations, print measurements, and optionally save their plot."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("all", "gravity", "hover", "vertical", "roll", "pitch", "yaw", "wind"), default="all")
    parser.add_argument("--headless", action="store_true", help="Use PyBullet DIRECT mode")
    parser.add_argument("--plot", type=Path, default=Path("outputs/physics_engine_validation.png"))
    parser.add_argument("--no-plot", action="store_true", help="Do not save the result plot")
    parser.add_argument("--self-check", action="store_true", help="Run all validation assertions without a plot")
    args = parser.parse_args()
    client = p.connect(p.DIRECT if args.headless or args.self_check else p.GUI)
    try:
        results = run_validation("all" if args.self_check else args.scenario)
        for result in results:
            print(f"{result.name:>22}: {result.measurement:.3f} {result.unit} ({result.expectation})")
        if not args.no_plot and not args.self_check:
            save_plot(results, args.plot)
            print(f"Saved plot: {args.plot}")
        if args.self_check:
            print("Physics-engine validation self-check passed")
    finally:
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
