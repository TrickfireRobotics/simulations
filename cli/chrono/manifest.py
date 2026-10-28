"""Building a resolved run manifest.

This is the single place where a scenario, a rover, its wheel packages, actuators,
battery, terrain and soils become one file. The simulator reads that file and nothing
else, so after a run the manifest in the results directory is a complete, re-runnable
description of what was simulated.

Everything the simulator needs is copied in by value. Nothing is left as a reference to
a config that could later change.
"""

from __future__ import annotations

import datetime as dt
import os
import platform
import subprocess
from pathlib import Path
from typing import Any

import yaml

from ..paths import CHRONO_DIR, REPO_DIR
from . import configs, wheels
from .schema import SchemaError, find_placeholders, validate

MANIFEST_SCHEMA_VERSION = 1


class ManifestError(Exception):
    def __init__(self, problems: list[str]) -> None:
        self.problems = problems
        body = "\n".join(f"  - {p}" for p in problems)
        super().__init__(f"{len(problems)} problem(s) resolving the run:\n{body}")


def timestamp() -> str:
    """UTC timestamp used in results directory names, filesystem-safe and sortable."""
    return dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H%M%SZ")


def git_commit() -> str:
    try:
        result = subprocess.run(
            ["git", "-C", str(REPO_DIR), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        )
        commit = result.stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "unknown"

    try:
        dirty = subprocess.run(
            ["git", "-C", str(REPO_DIR), "status", "--porcelain"],
            capture_output=True,
            text=True,
            check=True,
        )
        # A dirty tree is recorded, not refused: most runs during development happen on
        # one. It is the comparison step that should care.
        if dirty.stdout.strip():
            commit += "-dirty"
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    return commit


def build(
    scenario_ref: str,
    *,
    run_id: str,
    output_dir: Path,
    study: str = "",
    seed: int = 0,
    headless: bool = True,
    wheel_override: str | None = None,
    actuator_override: str | None = None,
    terrain_override: str | None = None,
    rover_override: str | None = None,
    overrides: dict[str, Any] | None = None,
    threads: int | None = None,
) -> dict[str, Any]:
    """Resolve everything a run needs into one manifest dictionary.

    `overrides` holds dotted-path edits applied to the *scenario* after inheritance,
    which is how a sweep varies one parameter at a time.
    """
    problems: list[str] = []
    provenance: dict[str, Any] = {}

    scenario, scenario_path, scenario_chain = configs.load("scenario", scenario_ref)
    if overrides:
        scenario = configs.apply_overrides(scenario, overrides)

    _validate(scenario, "scenario", scenario_path, problems)
    provenance["scenario"] = _trace(scenario_path, scenario_chain, scenario)

    rover_ref = rover_override or scenario.get("rover")
    if not rover_ref:
        problems.append("scenario.rover: no rover given, and no --rover on the command line")
        raise ManifestError(problems)

    rover, rover_path, rover_chain = configs.load("rover", rover_ref)
    _validate(rover, "rover", rover_path, problems)
    provenance["rover"] = _trace(rover_path, rover_chain, rover)

    terrain_ref = terrain_override or scenario.get("terrain")
    if not terrain_ref:
        problems.append("scenario.terrain: no terrain given, and no --terrain on the command line")
        raise ManifestError(problems)

    terrain, terrain_path, terrain_chain = configs.load("terrain", terrain_ref)
    _validate(terrain, "terrain", terrain_path, problems)
    provenance["terrain"] = _trace(terrain_path, terrain_chain, terrain)

    battery_ref = scenario.get("battery") or rover.get("battery")
    if not battery_ref:
        problems.append("no battery given by the scenario or the rover config")
        raise ManifestError(problems)

    battery, battery_path, battery_chain = configs.load("battery", battery_ref)
    _validate(battery, "battery", battery_path, problems)
    provenance["battery"] = _trace(battery_path, battery_chain, battery)

    stations = _resolve_stations(rover, wheel_override, actuator_override, problems)
    wheel_specs, actuator_specs = _resolve_parts(stations, problems, provenance)
    soils = _resolve_soils(terrain, problems, provenance)

    if problems:
        raise ManifestError(problems)

    simulation = {
        "step_size": scenario.get("step_size", 2.0e-3),
        "threads": threads or scenario.get("threads", _default_threads()),
        "contact_method": scenario.get("contact_method", "smc"),
        "up_axis": scenario.get("up_axis", "z"),
        "gravity": scenario.get("gravity", 9.81),
        "solver_iterations": scenario.get("solver_iterations", 200),
    }

    manifest: dict[str, Any] = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "run": {
            "id": run_id,
            "study": study,
            "seed": seed,
            "headless": headless,
            "output_dir": str(output_dir),
            "created": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"),
            "git_commit": git_commit(),
            "host": platform.node(),
        },
        "simulation": simulation,
        "wheels": wheel_specs,
        "actuators": actuator_specs,
        "battery": _battery_section(battery),
        "rover": _rover_section(rover, stations),
        "terrain": _terrain_section(terrain, soils),
        "scenario": _scenario_section(scenario),
        "provenance": provenance,
    }
    return manifest


def write(manifest: dict[str, Any], path: Path) -> Path:
    """Write a manifest, making every path inside it relative to the manifest itself.

    This is what lets a results directory be copied to another machine and re-run: the
    meshes are referenced relative to the manifest, and `sim chrono run` copies them in
    alongside it.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(manifest, sort_keys=False, default_flow_style=None))
    return path


def validate_manifest(manifest: dict[str, Any], source: str = "") -> None:
    validate(manifest, "manifest", source=source)

    placeholders = find_placeholders(manifest)
    if placeholders:
        raise SchemaError(
            [f"{key}: still a REPLACE_WITH placeholder" for key in placeholders]
            + [
                (
                    "These are the values the development plan says must be measured "
                    "before the results mean anything. Fill them in, or run a scenario "
                    "that does not need them."
                )
            ]
        )


# ---------------------------------------------------------------------------- #
#                                   resolution                                 #
# ---------------------------------------------------------------------------- #


def _default_threads() -> int:
    # Leave a core free: SCM's ray casting saturates whatever it is given, and a run that
    # starves the viewer or the shell looks like a hang.
    return max(1, (os.cpu_count() or 4) - 1)


def _validate(data: dict, kind: str, path: Path, problems: list[str]) -> None:
    try:
        validate(data, configs.schema_for(kind), source=str(path))
    except SchemaError as exc:
        problems += exc.problems


def _trace(path: Path, chain: list[Path], resolved: dict) -> dict[str, Any]:
    """Where a config came from, for the manifest's provenance section."""
    return {
        "file": _relative(path),
        "extends_chain": [_relative(p) for p in chain],
        "resolved_hash": configs.content_hash(resolved),
    }


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_DIR))
    except ValueError:
        return str(path)


def _resolve_stations(
    rover: dict, wheel_override: str | None, actuator_override: str | None, problems: list[str]
) -> list[dict[str, Any]]:
    """The rover's wheel stations with --wheel / --actuator applied to all of them."""
    stations = []
    for index, station in enumerate(rover.get("wheels", [])):
        resolved = dict(station)
        if wheel_override:
            resolved["wheel_package"] = wheel_override
        if actuator_override:
            resolved["actuator"] = actuator_override

        if not resolved.get("wheel_package"):
            problems.append(f"rover.wheels[{index}]: no wheel_package, and no --wheel given")
        if not resolved.get("actuator"):
            problems.append(f"rover.wheels[{index}]: no actuator, and no --actuator given")
        stations.append(resolved)

    if not stations:
        problems.append("rover.wheels: the rover config declares no wheel stations")
    return stations


def _resolve_parts(
    stations: list[dict], problems: list[str], provenance: dict
) -> tuple[list[dict], list[dict]]:
    """Inline every referenced wheel package and actuator config by value."""
    wheel_specs: dict[str, dict] = {}
    actuator_specs: dict[str, dict] = {}
    wheel_trace: dict[str, Any] = {}
    actuator_trace: dict[str, Any] = {}

    for station in stations:
        name = station.get("wheel_package")
        if name and name not in wheel_specs:
            try:
                package = wheels.load_package(name)
                wheels.validate_package(package)
                wheel_specs[name] = _wheel_section(package)
                wheel_trace[name] = {
                    "directory": _relative(package.directory),
                    "revision": package.revision,
                    "source_hash": package.manifest.get("source", {}).get("content_hash", ""),
                }
            except SchemaError as exc:
                problems += [f"wheel '{name}': {p}" for p in exc.problems]
            except SystemExit:
                # wheels.load_package calls die() for a missing package; turn that into a
                # collected problem so every missing part is reported in one pass.
                problems.append(f"wheel package '{name}' does not exist")

        actuator = station.get("actuator")
        if actuator and actuator not in actuator_specs:
            try:
                spec, path, chain = configs.load("actuator", actuator)
                _validate(spec, "actuator", path, problems)
                spec = dict(spec)
                spec.setdefault("name", actuator)
                actuator_specs[actuator] = spec
                actuator_trace[actuator] = _trace(path, chain, spec)
            except configs.ConfigError as exc:
                problems.append(str(exc))

    provenance["wheels"] = wheel_trace
    provenance["actuators"] = actuator_trace
    return list(wheel_specs.values()), list(actuator_specs.values())


def _resolve_soils(terrain: dict, problems: list[str], provenance: dict) -> list[dict]:
    """Inline the terrain's soil configs by value."""
    soils: list[dict] = []
    trace: dict[str, Any] = {}

    for reference in terrain.get("soils", []):
        try:
            spec, path, chain = configs.load("soil", reference)
            _validate(spec, "soil", path, problems)
            spec = dict(spec)
            spec.setdefault("name", Path(reference).stem)
            soils.append(spec)
            trace[spec["name"]] = _trace(path, chain, spec)
        except configs.ConfigError as exc:
            problems.append(str(exc))

    provenance["soils"] = trace
    return soils


# ---------------------------------------------------------------------------- #
#                                   sections                                   #
# ---------------------------------------------------------------------------- #


def _wheel_section(package: wheels.WheelPackage) -> dict[str, Any]:
    """A wheel package flattened into the shape the simulator reads.

    Mesh paths are made absolute here. `sim chrono run` rewrites them to be relative to
    the manifest when it copies the assets into the results directory.
    """
    manifest = package.manifest
    geometry = manifest["geometry"]

    collision = []
    for shape in manifest.get("collision", []):
        entry = dict(shape)
        if "file" in entry:
            entry["file"] = str(wheels.resolve_path(package, entry["file"]))
        collision.append(entry)

    return {
        "name": package.name,
        "revision": package.revision,
        "content_hash": manifest.get("source", {}).get("content_hash", ""),
        "radius": geometry["radius"],
        "width": geometry["width"],
        "roll_axis": geometry.get("roll_axis", [0, 1, 0]),
        "mass_properties": _mass_section(manifest["mass_properties"]),
        "visual": {"file": str(wheels.resolve_path(package, manifest["visual"]["file"]))},
        "collision": collision,
    }


def _mass_section(properties: dict) -> dict[str, Any]:
    return {
        "mass": properties["mass"],
        "com": properties.get("com", [0, 0, 0]),
        "inertia_xx": properties.get("inertia_xx", [0, 0, 0]),
        "inertia_xy": properties.get("inertia_xy", [0, 0, 0]),
        "provenance": properties.get("provenance", "estimated"),
    }


def _battery_section(battery: dict) -> dict[str, Any]:
    section = dict(battery)
    section.pop("extends", None)
    section.pop("description", None)
    return section


def _rover_section(rover: dict, stations: list[dict]) -> dict[str, Any]:
    """The rover config with mesh paths absolutised and the resolved stations attached."""
    section = {
        "name": rover["name"],
        "source": rover.get("source", {}),
        "body_friction": rover.get("body_friction", 0.8),
        "body_restitution": rover.get("body_restitution", 0.0),
        "chassis": _absolutise_body(rover["chassis"]),
        "pivots": [_absolutise_body(pivot) for pivot in rover.get("pivots", [])],
        "differential": rover.get("differential", {"type": "none"}),
        "wheels": stations,
    }
    return section


def _absolutise_body(body: dict) -> dict[str, Any]:
    """Make every mesh path in a body spec absolute, relative to chrono/."""
    result = dict(body)

    def fix(entry: dict) -> dict:
        fixed = dict(entry)
        if "file" in fixed:
            path = Path(fixed["file"])
            if not path.is_absolute():
                path = (CHRONO_DIR / path).resolve()
            fixed["file"] = str(path)
        return fixed

    if isinstance(result.get("visual"), list):
        result["visual"] = [fix(entry) for entry in result["visual"]]
    if isinstance(result.get("collision"), list):
        result["collision"] = [fix(entry) for entry in result["collision"]]
    return result


def _terrain_section(terrain: dict, soils: list[dict]) -> dict[str, Any]:
    section = dict(terrain)
    section.pop("extends", None)
    section.pop("description", None)
    section["soils"] = soils

    for key in ("heightmap_file", "mesh_file"):
        if section.get(key):
            path = Path(section[key])
            if not path.is_absolute():
                path = (CHRONO_DIR / path).resolve()
            section[key] = str(path)
    return section


def _scenario_section(scenario: dict) -> dict[str, Any]:
    section = dict(scenario)
    # These are references the manifest already resolved into their own sections; leaving
    # them would imply the simulator still looks them up.
    for key in (
        "extends",
        "description",
        "rover",
        "terrain",
        "battery",
        "step_size",
        "threads",
        "contact_method",
        "up_axis",
        "gravity",
        "solver_iterations",
    ):
        section.pop(key, None)

    driver = section.get("driver", {})
    if driver.get("replay_file"):
        path = Path(driver["replay_file"])
        if not path.is_absolute():
            path = (CHRONO_DIR / path).resolve()
        driver = dict(driver)
        driver["replay_file"] = str(path)
        section["driver"] = driver
    return section
