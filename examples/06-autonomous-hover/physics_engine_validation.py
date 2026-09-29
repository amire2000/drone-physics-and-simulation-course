"""Validate gravity, thrust, torque, and wind with the Module 6 physics engine."""

import argparse
from dataclasses import dataclass, replace
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import pybullet as p

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.drone_model import DEFAULT_DRONE_MODEL, DEFAULT_PHYSICS_SETTINGS, PhysicsSettings
from common.battery import BatteryModel
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
    """Advance one fixed command at the profile reference voltage for repeatable force checks."""
    for _ in range(round(seconds / engine.settings.time_step_s)):
        engine.step(drone, pwm_us, torque_nm, engine.model.battery.nominal_voltage_v)


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


def body_drag_validation() -> ValidationResult:
    """Verify quadratic frame drag opposes a known forward body velocity."""
    drone = create_world()
    engine = PhysicsEngine()
    reset_drone(drone, (0.0, 0.0, 5.0))
    p.resetBaseVelocity(drone, linearVelocity=(10.0, 0.0, 0.0))
    step = engine.step(drone, PWM_HOVER, (0.0, 0.0, 0.0))
    force = step.body_drag_force_body_n[0]
    assert force < 0.0
    return ValidationResult("body drag", force, "N", "negative against forward velocity")


def angular_damping_validation() -> ValidationResult:
    """Verify aerodynamic damping torque opposes a known roll rate."""
    drone = create_world()
    engine = PhysicsEngine()
    reset_drone(drone, (0.0, 0.0, 5.0))
    p.resetBaseVelocity(drone, angularVelocity=(1.0, 0.0, 0.0))
    step = engine.step(drone, PWM_HOVER, (0.0, 0.0, 0.0))
    torque = step.angular_damping_torque_body_nm[0]
    assert torque < 0.0
    return ValidationResult("angular damping", torque, "N m", "negative against roll rate")


def mixer_geometry_validation() -> ValidationResult:
    """Verify longer URDF rotor lever arms need smaller thrust differences for roll torque."""
    baseline_engine = PhysicsEngine()
    wider_model = replace(MODEL, rotor_positions_m=tuple((2.0 * x, 2.0 * y, z) for x, y, z in MODEL.rotor_positions_m))
    wider_engine = PhysicsEngine(wider_model)
    baseline = baseline_engine._mix_motor_thrusts(3.0, (0.02, 0.0, 0.0))
    wider = wider_engine._mix_motor_thrusts(3.0, (0.02, 0.0, 0.0))
    baseline_span = max(baseline) - min(baseline)
    wider_span = max(wider) - min(wider)
    assert wider_span < baseline_span
    return ValidationResult("URDF mixer geometry", wider_span / baseline_span, "ratio", "longer arms need less thrust difference")


def battery_voltage_validation() -> ValidationResult:
    """Verify a lower bus voltage lowers KV-limited RPM and higher resistance increases sag."""
    nominal_voltage_v = MODEL.battery.nominal_voltage_v
    drone = create_world()
    engine = PhysicsEngine()
    reset_drone(drone, (0.0, 0.0, 5.0))
    engine.reset()
    for _ in range(round(0.5 / SETTINGS.time_step_s)):
        nominal_step = engine.step(drone, 2000.0, (0.0, 0.0, 0.0), nominal_voltage_v)

    reset_drone(drone, (0.0, 0.0, 5.0))
    engine.reset()
    for _ in range(round(0.5 / SETTINGS.time_step_s)):
        low_voltage_step = engine.step(drone, 2000.0, (0.0, 0.0, 0.0), nominal_voltage_v * 0.9)

    baseline_battery = BatteryModel(MODEL.battery)
    resistant_battery = BatteryModel(replace(MODEL.battery, internal_resistance_ohm=MODEL.battery.internal_resistance_ohm * 2.0))
    baseline_bus = baseline_battery.step((1.0, 1.0, 1.0, 1.0), SETTINGS.time_step_s).bus_voltage_v
    resistant_bus = resistant_battery.step((1.0, 1.0, 1.0, 1.0), SETTINGS.time_step_s).bus_voltage_v
    rpm_ratio = max(low_voltage_step.motor_rpms) / max(nominal_step.motor_rpms)
    assert 0.89 < rpm_ratio < 0.91
    assert resistant_bus < baseline_bus
    return ValidationResult("battery KV voltage", rpm_ratio, "ratio", "90% bus voltage gives about 90% RPM")


def optional_forces_validation() -> ValidationResult:
    """Verify optional rotor, ground, and gyro effects are configurable and bounded."""
    settings = PhysicsSettings(
        rotor_aerodynamics_enabled=True,
        ground_effect_enabled=True,
        gyroscopic_torque_enabled=True,
    )
    drone = create_world(settings=settings)
    engine = PhysicsEngine(settings=settings)
    reset_drone(drone, (0.0, 0.0, 0.15))
    engine.reset((HOVER_RPM * 1.2, HOVER_RPM, HOVER_RPM, HOVER_RPM))
    p.resetBaseVelocity(drone, linearVelocity=(8.0, 0.0, 0.0), angularVelocity=(1.0, 0.0, 0.0))
    step = engine.step(drone, PWM_HOVER, (0.0, 0.0, 0.0))
    assert max(step.ground_effect_multipliers) > 1.0
    assert step.gyroscopic_torque_body_nm[1] != 0.0
    assert max(step.motor_thrusts_n) < MODEL.max_thrust_per_motor_n * settings.ground_effect_max_multiplier
    return ValidationResult("optional forces", max(step.ground_effect_multipliers), "x", "ground effect bounded; gyro active")


def run_validation(selected: str) -> list[ValidationResult]:
    """Run one named validation or the complete validation suite."""
    cases = {
        "gravity": gravity_validation,
        "hover": hover_validation,
        "vertical": vertical_validation,
        "roll": lambda: attitude_validation("roll", (0.02, 0.0, 0.0), 0),
        "pitch": lambda: attitude_validation("pitch", (0.0, 0.02, 0.0), 1),
        "yaw": lambda: attitude_validation("yaw", (0.0, 0.0, 0.002), 2),
        "wind": wind_validation,
        "body-drag": body_drag_validation,
        "angular-damping": angular_damping_validation,
        "mixer-geometry": mixer_geometry_validation,
        "battery-voltage": battery_voltage_validation,
        "optional-forces": optional_forces_validation,
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


def print_results(results: list[ValidationResult], selected: str) -> None:
    """Print labelled validation measurements and a completion summary."""
    print("\n====== PHYSICS-ENGINE VALIDATION ======")
    for result in results:
        print(f"{result.name:>22}: {result.measurement:.3f} {result.unit} ({result.expectation})")
    if selected == "all":
        print(f"{len(results)}/12 checks passed")
    print("========================================")


def main() -> None:
    """Run selected validations, print measurements, and optionally save their plot."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("all", "gravity", "hover", "vertical", "roll", "pitch", "yaw", "wind", "body-drag", "angular-damping", "mixer-geometry", "battery-voltage", "optional-forces"), default="all")
    parser.add_argument("--headless", action="store_true", help="Use PyBullet DIRECT mode")
    parser.add_argument("--plot", type=Path, default=Path("outputs/physics_engine_validation.png"))
    parser.add_argument("--no-plot", action="store_true", help="Do not save the result plot")
    parser.add_argument("--self-check", action="store_true", help="Run all validation assertions without a plot")
    args = parser.parse_args()
    client = p.connect(p.DIRECT if args.headless or args.self_check else p.GUI)
    try:
        selected = "all" if args.self_check else args.scenario
        results = run_validation(selected)
        print_results(results, selected)
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
