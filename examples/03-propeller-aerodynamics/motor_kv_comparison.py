"""Compare KV and battery-voltage choices for one 5-inch FPV frame."""

import argparse
from dataclasses import dataclass


NOMINAL_CELL_VOLTAGE = 3.7
BASELINE_KV = 1750
BASELINE_CELLS = 6


@dataclass(frozen=True)
class BuildOption:
    """One teaching configuration for the same 5-inch FPV frame."""

    name: str
    cells: int
    kv: int
    propeller: str
    purpose: str


OPTIONS = (
    BuildOption("Baseline freestyle", 6, 1750, "5x4.3x3", "balanced reference"),
    BuildOption("4S freestyle", 4, 2550, "5x4.3x3", "similar no-load RPM"),
    BuildOption("Efficient cruising", 6, 1650, "5x3.2x2", "lower propeller load"),
    BuildOption("Aggressive racing", 6, 2000, "5x4.8x3", "high current and heat"),
)


def no_load_rpm(kv: float, cells: int) -> float:
    """Return the ideal no-load RPM at nominal battery voltage."""
    return kv * cells * NOMINAL_CELL_VOLTAGE


def relative_thrust(kv: float, cells: int) -> float:
    """Estimate thrust ratio for the same propeller and equal load conditions."""
    baseline_rpm = no_load_rpm(BASELINE_KV, BASELINE_CELLS)
    return (no_load_rpm(kv, cells) / baseline_rpm) ** 2


def main() -> None:
    """Print the baseline and comparable motor/battery configurations."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kv", type=float, help="Optional motor KV to compare")
    parser.add_argument("--cells", type=int, help="Optional battery cell count to compare")
    args = parser.parse_args()

    print("Same 5-inch frame; nominal voltage; no-load RPM only")
    print("configuration        | battery | KV   | no-load RPM | same-prop thrust ratio | propeller      | purpose")
    for option in OPTIONS:
        print(
            f"{option.name:20} | {option.cells}S      | {option.kv:4} | {no_load_rpm(option.kv, option.cells):11.0f} | "
            f"{relative_thrust(option.kv, option.cells):20.2f} | {option.propeller:14} | {option.purpose}"
        )

    if (args.kv is None) != (args.cells is None):
        parser.error("use --kv and --cells together")
    if args.kv is not None and args.cells is not None:
        if args.kv <= 0 or args.cells <= 0:
            parser.error("KV and cell count must be positive")
        print(f"Custom {args.cells}S {args.kv:.0f} KV: {no_load_rpm(args.kv, args.cells):.0f} no-load RPM, "
              f"{relative_thrust(args.kv, args.cells):.2f}x same-prop thrust estimate")

    print("A propeller change invalidates the thrust-ratio estimate; check motor, ESC, and propeller data before building.")
    assert no_load_rpm(BASELINE_KV, BASELINE_CELLS) == 38_850
    assert relative_thrust(BASELINE_KV, BASELINE_CELLS) == 1.0


if __name__ == "__main__":
    main()
