"""Run the TTC diagonal-strike example; implementation lives in ttc_strike."""

from pathlib import Path
import sys

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from ttc_strike.cli import main


if __name__ == "__main__":
    main()
