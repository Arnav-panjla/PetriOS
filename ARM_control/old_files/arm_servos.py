"""Start the six-servo CLI.

The package lives in robotic-arm/. From that directory, after
`pip install -e ".[hardware]"`:

    python -m robot_arm.interfaces.cli
"""

from __future__ import annotations

import sys
from pathlib import Path


def _main() -> int:
    src = Path(__file__).resolve().parent / "robotic-arm" / "src"
    sys.path.insert(0, str(src))
    from robot_arm.interfaces.cli import main

    return main()


if __name__ == "__main__":
    raise SystemExit(_main())
