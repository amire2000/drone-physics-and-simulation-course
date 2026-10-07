"""Command-line parsing shared by independent Module 6 topic examples."""

import argparse
from pathlib import Path
from typing import Any

ArgumentSpec = tuple[tuple[str, ...], dict[str, Any]]


def parse_args(description: str, topic_arguments: tuple[ArgumentSpec, ...] = ()) -> argparse.Namespace:
    """Parse common simulation options plus declarative topic arguments."""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--headless", action="store_true", help="Run without opening PyBullet's GUI")
    parser.add_argument("--self-check", action="store_true", help="Validate the example and exit")
    parser.add_argument("--backend", choices=("pybullet", "reduced"), default="pybullet", help="Select the PyBullet or reduced-order loop")
    parser.add_argument("--output", type=Path, help="Save a PNG graph and CSV samples with this path stem")
    parser.add_argument("--gif", type=Path, help="Save a deterministic PyBullet scene animation as a GIF")
    for flags, kwargs in topic_arguments:
        parser.add_argument(*flags, **kwargs)
    return parser.parse_args()
