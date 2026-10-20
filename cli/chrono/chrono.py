"""Run commands for chrono, very raw rn"""

import os
import subprocess
import sys

from ..output import die
from ..paths import CHRONO_TERRAIN_DIR, CHRONO_WHEELS_DIR

# Always available - built into the sim binary itself, no mesh file needed
BUILTIN_WHEELS = ("lugged", "cylindrical")


def list_wheels() -> list[str]:
    """Every name `sim chrono run <wheel>` accepts: the built-ins plus one per *.obj
    file in chrono/data/models/wheels/ (see wheel_catalog.hpp for how the sim binary
    resolves the same list)."""
    custom = (
        sorted(p.stem for p in CHRONO_WHEELS_DIR.glob("*.obj"))
        if CHRONO_WHEELS_DIR.is_dir()
        else []
    )
    return [*BUILTIN_WHEELS, *custom]


def run(wheel: str | None = None):
    env = None
    if wheel is not None:
        available = list_wheels()
        if wheel not in available:
            die(f"Unknown wheel '{wheel}'. Available: {', '.join(available)}")
        env = {**os.environ, "TRICKFIRE_WHEEL": wheel}

    result = subprocess.run(["make", "run"], cwd=CHRONO_TERRAIN_DIR, env=env, check=False)
    if result.returncode != 0:
        sys.exit(result.returncode)


def clean():
    result = subprocess.run(["make", "clean"], cwd=CHRONO_TERRAIN_DIR, check=False)
    if result.returncode != 0:
        sys.exit(result.returncode)
