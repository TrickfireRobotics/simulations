"""Building, launching and bookkeeping a single run.

The CLI's job is everything around the simulator: resolve the configs, write the
manifest, stage the assets, launch the binary, then compress the telemetry. The
simulator's job is the physics. Keeping that boundary sharp is what lets the same run
be reproduced from its own output directory.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from typing import Any

import yaml

from ..output import die, info, warn
from ..paths import CHRONO_BIN, CHRONO_DIR, REPO_DIR
from . import manifest as manifest_mod
from . import results
from .schema import SchemaError


class RunError(Exception):
    pass


def ensure_built(rebuild: bool = False) -> Path:
    """Build the simulator if the binary is missing or out of date.

    Delegating to the existing Makefile keeps one build path: `make -C chrono` works by
    hand and is what CI calls, and this adds no second opinion about build flags.
    """
    if CHRONO_BIN.is_file() and not rebuild:
        newest = max(
            (p.stat().st_mtime for p in (CHRONO_DIR / "src").rglob("*.?pp")),
            default=0.0,
        )
        cmake_time = (CHRONO_DIR / "CMakeLists.txt").stat().st_mtime
        if CHRONO_BIN.stat().st_mtime >= max(newest, cmake_time):
            return CHRONO_BIN

    info("building the simulator")
    result = subprocess.run(["make"], cwd=CHRONO_DIR, check=False)
    if result.returncode != 0:
        raise RunError("the simulator failed to build")
    if not CHRONO_BIN.is_file():
        raise RunError(f"the build reported success but {CHRONO_BIN} does not exist")
    return CHRONO_BIN


def prepare(
    scenario: str,
    *,
    run_id: str,
    directory: Path,
    study: str = "",
    seed: int = 0,
    headless: bool = True,
    wheel: str | None = None,
    actuator: str | None = None,
    terrain: str | None = None,
    rover: str | None = None,
    overrides: dict[str, Any] | None = None,
    threads: int | None = None,
) -> Path:
    """Resolve a run into a self-contained directory and return its manifest path."""
    built = manifest_mod.build(
        scenario,
        run_id=run_id,
        output_dir=directory,
        study=study,
        seed=seed,
        headless=headless,
        wheel_override=wheel,
        actuator_override=actuator,
        terrain_override=terrain,
        rover_override=rover,
        overrides=overrides,
        threads=threads,
    )

    built = _expand_rock_field(built)

    directory.mkdir(parents=True, exist_ok=True)
    staged = results.stage_assets(built, directory)
    manifest_mod.validate_manifest(staged, source="resolved manifest")
    return results.write_manifest(staged, directory)


def prepare_from_manifest(manifest_path: Path, directory: Path) -> Path:
    """Re-run an existing manifest, copying it into a fresh results directory.

    The point of re-running a manifest rather than a scenario is that nothing is
    re-resolved: the same meshes, the same soil numbers, the same seed. Only the output
    directory changes.
    """
    data = yaml.safe_load(manifest_path.read_text()) or {}
    if not data:
        raise RunError(f"{manifest_path} is empty")

    # Asset paths in a saved manifest are relative to it, so they are made absolute
    # against the original directory before being staged into the new one.
    source_dir = manifest_path.parent
    data = _absolutise(data, source_dir)

    data.setdefault("run", {})["output_dir"] = str(directory)
    directory.mkdir(parents=True, exist_ok=True)
    staged = results.stage_assets(data, directory)
    manifest_mod.validate_manifest(staged, source=str(manifest_path))
    return results.write_manifest(staged, directory)


def launch(manifest_path: Path, headless: bool, timeout: float | None = None) -> int:
    """Run the simulator on a manifest and return its exit code."""
    binary = ensure_built()
    command = [str(binary), "--manifest", str(manifest_path)]
    command.append("--headless" if headless else "--gui")

    # The simulator resolves CHRONO_DATA_PATH relative to the chrono/ directory, and the
    # Makefile-built binary expects to be launched from the build tree.
    environment = dict(os.environ)

    try:
        result = subprocess.run(
            command, cwd=binary.parent, env=environment, check=False, timeout=timeout
        )
    except subprocess.TimeoutExpired:
        warn(f"the run exceeded its {timeout:.0f} s wall-clock budget and was stopped")
        return 124
    except KeyboardInterrupt:
        return 130
    return result.returncode


def validate(manifest_path: Path) -> int:
    """Ask the simulator itself to validate a manifest.

    Separate from the CLI's own schema check on purpose: the two validators disagree
    about different things, and a manifest that passes the schema but not the simulator's
    loader is exactly the case worth catching before a sweep of a hundred runs.
    """
    binary = ensure_built()
    result = subprocess.run(
        [str(binary), "--manifest", str(manifest_path), "--validate"],
        cwd=binary.parent,
        check=False,
    )
    return result.returncode


def finish(directory: Path, keep_plain_csv: bool = False) -> results.RunResult | None:
    """Compress the telemetry and read the run back."""
    compressed = results.compress_telemetry(directory, keep_plain=keep_plain_csv)
    if compressed:
        info(f"telemetry -> {compressed.name}")
    return results.load_run(directory)


def report_outcome(result: results.RunResult | None, exit_code: int) -> None:
    """Print a one-screen verdict for a single run."""
    if result is None:
        warn(f"the run left no summary.json (exit code {exit_code})")
        return

    termination = result.termination
    verdict = "PASS" if result.passed else "FAIL"
    info(f"{verdict}  {termination.get('rule', '?')} - {termination.get('detail', '')}")

    for check in result.checks:
        mark = "ok  " if check["passed"] else "FAIL"
        if not check.get("evaluated", True):
            mark = "n/a "
        actual = check.get("actual")
        print(
            f"  [{mark}] {check['expression']}  (actual {actual:.4g})"
            if isinstance(actual, (int, float))
            else f"  [{mark}] {check['expression']}"
        )

    headline = [
        ("distance", "mobility.distance_m", "m"),
        ("energy", "energy.total_wh", "Wh"),
        ("per metre", "energy.wh_per_m", "Wh/m"),
        ("peak motor", "motor.peak_temperature_c", "C"),
        ("mean slip", "wheel.mean_longitudinal_slip.mean", ""),
        ("real-time", "numerics.real_time_factor.mean", "x"),
    ]
    for label, key, unit in headline:
        value = result.metric(key)
        if value is not None:
            print(f"  {label:<12} {value:>10.4g} {unit}")

    print(f"\n  results: {_relative(result.directory)}")


# ---------------------------------------------------------------------------- #
#                                   internals                                  #
# ---------------------------------------------------------------------------- #


def _relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_DIR))
    except ValueError:
        return str(path)


def _absolutise(node: Any, base: Path) -> Any:
    """Make relative asset paths in a saved manifest absolute against its directory."""
    if isinstance(node, dict):
        result = {}
        for key, value in node.items():
            if key in {"file", "heightmap_file", "mesh_file", "replay_file"} and isinstance(
                value, str
            ):
                path = Path(value)
                result[key] = str(path if path.is_absolute() else (base / path).resolve())
            else:
                result[key] = _absolutise(value, base)
        return result
    if isinstance(node, list):
        return [_absolutise(item, base) for item in node]
    return node


def _expand_rock_field(built: dict[str, Any]) -> dict[str, Any]:
    """Turn a terrain's `rock_field` description into explicit obstacles.

    Done here rather than in the simulator so the manifest fully determines the scene:
    two wheels compared on "the same rock field" are compared on the identical rocks,
    and a result's rock layout can be read out of its own manifest years later.
    """
    import random

    terrain = built.get("terrain", {})
    field = terrain.pop("rock_field", None)
    if not field:
        return built

    seed = int(field.get("seed", built["run"]["seed"]))
    rng = random.Random(seed)

    count = int(field.get("count", 0))
    x_range = field.get("x_range", [-5.0, 5.0])
    y_range = field.get("y_range", [-2.0, 2.0])
    radius_range = field.get("radius_range", [0.05, 0.15])
    embed = float(field.get("embed_fraction", 0.4))
    friction = float(field.get("friction", 0.9))
    loose_fraction = float(field.get("loose_fraction", 0.0))
    density = float(field.get("loose_density_kg_m3", 2600.0))

    obstacles = list(terrain.get("obstacles", []))
    for _ in range(count):
        radius = rng.uniform(*radius_range)
        rock = {
            "type": "sphere",
            "radius": round(radius, 4),
            # Sunk by `embed` of its radius, so a rock is an obstacle rather than
            # something the wheel rolls over the top of.
            "position": [
                round(rng.uniform(*x_range), 4),
                round(rng.uniform(*y_range), 4),
                round(-embed * radius, 4),
            ],
            "friction": friction,
            "fixed": True,
        }
        if rng.random() < loose_fraction:
            rock["fixed"] = False
            rock["mass"] = round(density * (4.0 / 3.0) * 3.14159 * radius**3, 4)
        obstacles.append(rock)

    terrain["obstacles"] = obstacles
    terrain.setdefault("source", {})["rock_field_seed"] = seed
    built["terrain"] = terrain
    info(f"expanded {count} rocks from seed {seed}")
    return built


def abort_on_config_error(exc: Exception) -> None:
    """Report a configuration failure the way the plan asks: every problem, then stop."""
    if isinstance(exc, SchemaError):
        die(str(exc))
    die(str(exc))


def ensure_results_root() -> None:
    (REPO_DIR / "results").mkdir(parents=True, exist_ok=True)


def python_executable() -> str:
    return sys.executable or "python3"
