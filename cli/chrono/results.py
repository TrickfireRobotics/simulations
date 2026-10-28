"""The results directory layout, and reading a finished run back.

Layout, per the development plan:

    results/<timestamp>-<study>/<run-id>/
      manifest.yaml          fully resolved inputs and asset hashes
      environment.json       git commit, Chrono commit, build flags, machine, solver
      telemetry.csv[.zst]    time-series data
      events.jsonl           limit, contact, cutoff, rollover and termination events
      summary.json           scalar results and pass/fail checks
      report.html            plots and comparison tables
      preview.png            optional final or worst-case frame

The simulator writes the CSV, the events and the summary. Everything else is this
module's job, including compression - the Arrow stack is not worth a dependency just to
write Parquet at this scale.
"""

from __future__ import annotations

import json
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from ..output import info, warn
from ..paths import RESULTS_DIR


@dataclass
class RunResult:
    """One finished run, read back from its directory."""

    directory: Path
    run_id: str
    manifest: dict[str, Any] = field(default_factory=dict)
    summary: dict[str, Any] = field(default_factory=dict)
    environment: dict[str, Any] = field(default_factory=dict)

    @property
    def passed(self) -> bool:
        return bool(self.summary.get("passed", False))

    @property
    def metrics(self) -> dict[str, float]:
        return self.summary.get("metrics", {})

    @property
    def checks(self) -> list[dict]:
        return self.summary.get("checks", [])

    @property
    def termination(self) -> dict:
        return self.summary.get("termination", {})

    @property
    def telemetry_path(self) -> Path | None:
        for name in ("telemetry.csv.zst", "telemetry.csv.xz", "telemetry.csv"):
            candidate = self.directory / name
            if candidate.is_file():
                return candidate
        return None

    def metric(self, key: str, default: float | None = None) -> float | None:
        value = self.metrics.get(key, default)
        return None if value is None else float(value)

    def label(self) -> str:
        """Short description of what varied, for report tables and legends."""
        parts = []
        wheels = self.manifest.get("rover", {}).get("wheels", [])
        packages = sorted({w.get("wheel_package", "") for w in wheels if w.get("wheel_package")})
        if packages:
            parts.append("/".join(packages))

        actuators = sorted({w.get("actuator", "") for w in wheels if w.get("actuator")})
        if len(actuators) == 1 and actuators[0]:
            parts.append(actuators[0])

        terrain = self.manifest.get("terrain", {}).get("name")
        if terrain:
            parts.append(terrain)

        scenario = self.manifest.get("scenario", {}).get("name")
        if scenario:
            parts.append(scenario)

        seed = self.manifest.get("run", {}).get("seed")
        if seed is not None:
            parts.append(f"seed {seed}")
        return " | ".join(parts) or self.run_id


def study_dir(timestamp: str, study: str) -> Path:
    slug = study or "run"
    return RESULTS_DIR / f"{timestamp}-{slug}"


def run_dir(study_path: Path, run_id: str) -> Path:
    return study_path / run_id


def load_run(directory: Path) -> RunResult | None:
    """Read a run directory back, or None if it holds no finished run."""
    summary_path = directory / "summary.json"
    if not summary_path.is_file():
        return None

    result = RunResult(directory, directory.name)
    try:
        result.summary = json.loads(summary_path.read_text())
    except json.JSONDecodeError as exc:
        warn(f"{summary_path} is not valid JSON ({exc}); treating the run as incomplete")
        return None

    manifest_path = directory / "manifest.yaml"
    if manifest_path.is_file():
        result.manifest = yaml.safe_load(manifest_path.read_text()) or {}

    environment_path = directory / "environment.json"
    if environment_path.is_file():
        try:
            result.environment = json.loads(environment_path.read_text())
        except json.JSONDecodeError:
            pass
    return result


def load_study(study_path: Path) -> list[RunResult]:
    """Every finished run in a study directory, in directory-name order."""
    if not study_path.is_dir():
        return []

    runs = []
    for directory in sorted(p for p in study_path.iterdir() if p.is_dir()):
        result = load_run(directory)
        if result:
            runs.append(result)
    return runs


def latest_study() -> Path | None:
    """The most recent study directory, so `report` can be called without an argument."""
    if not RESULTS_DIR.is_dir():
        return None
    candidates = [p for p in RESULTS_DIR.iterdir() if p.is_dir()]
    return max(candidates, key=lambda p: p.name) if candidates else None


def resolve_study(reference: str | None) -> Path:
    """Resolve a study reference: a path, a directory name under results/, or the latest."""
    if not reference:
        latest = latest_study()
        if not latest:
            raise FileNotFoundError(
                f"no results in {RESULTS_DIR}. Run `sim chrono run` or `sim chrono sweep` first."
            )
        return latest

    for candidate in (Path(reference), RESULTS_DIR / reference, Path.cwd() / reference):
        if candidate.is_dir():
            return candidate.resolve()
    raise FileNotFoundError(f"no study directory matching '{reference}'")


# ---------------------------------------------------------------------------- #
#                                  compression                                 #
# ---------------------------------------------------------------------------- #


def compress_telemetry(directory: Path, keep_plain: bool = False) -> Path | None:
    """Compress telemetry.csv in place.

    Prefers zstd for speed and ratio, falling back to xz, which is in the standard
    library. Telemetry is highly compressible - mostly slowly varying floats - so this
    typically cuts a long run by an order of magnitude, which matters once a sweep has
    a few hundred runs in it.
    """
    source = directory / "telemetry.csv"
    if not source.is_file():
        return None

    try:
        import zstandard

        target = directory / "telemetry.csv.zst"
        compressor = zstandard.ZstdCompressor(level=10)
        with source.open("rb") as raw, target.open("wb") as out:
            compressor.copy_stream(raw, out)
    except ImportError:
        import lzma

        target = directory / "telemetry.csv.xz"
        with source.open("rb") as raw, lzma.open(target, "wb", preset=3) as out:
            shutil.copyfileobj(raw, out)

    if not keep_plain:
        source.unlink()
    return target


def open_telemetry(path: Path):
    """Open a telemetry file for text reading, transparently decompressing it."""
    if path.suffix == ".zst":
        import zstandard

        return zstandard.ZstdDecompressor().stream_reader(path.open("rb"))
    if path.suffix == ".xz":
        import lzma

        return lzma.open(path, "rt")
    return path.open("r")


def read_telemetry(path: Path) -> tuple[list[str], list[list[float]]]:
    """Read a telemetry file into its header and columns.

    Returns column-major data, because every plot and every statistic wants one channel
    at a time rather than one row at a time.
    """
    import csv
    import io

    handle = open_telemetry(path)
    text = handle.read()
    if isinstance(text, bytes):
        text = text.decode()

    reader = csv.reader(io.StringIO(text))
    header = next(reader, [])
    columns: list[list[float]] = [[] for _ in header]

    for row in reader:
        for index, cell in enumerate(row[: len(header)]):
            # Blank means the simulator wrote a non-finite value; nan keeps the column
            # aligned and plots as a gap.
            columns[index].append(float(cell) if cell else float("nan"))
    return header, columns


# ---------------------------------------------------------------------------- #
#                               assets and manifest                            #
# ---------------------------------------------------------------------------- #


def stage_assets(manifest: dict[str, Any], directory: Path) -> dict[str, Any]:
    """Copy every mesh and image the run needs into the results directory.

    This is what makes a results directory self-contained and re-runnable: the manifest
    is rewritten so its asset paths are relative to itself, and the assets travel with
    it. Copying a result to another machine and running `sim chrono run <that manifest>`
    then works even if the wheel package has since been replaced.
    """
    assets = directory / "assets"
    copied: dict[str, str] = {}

    def stage(path_value: str) -> str:
        source = Path(path_value)
        if not source.is_absolute() or not source.is_file():
            return path_value
        if path_value in copied:
            return copied[path_value]

        assets.mkdir(parents=True, exist_ok=True)
        # Prefix with the parent directory so two wheels' visual.obj do not collide.
        target_name = f"{source.parent.name}_{source.name}"
        target = assets / target_name
        if not target.exists():
            shutil.copy2(source, target)

        relative = f"assets/{target_name}"
        copied[path_value] = relative
        return relative

    def walk(node: Any) -> Any:
        if isinstance(node, dict):
            result = {}
            for key, value in node.items():
                if key in {"file", "heightmap_file", "mesh_file", "replay_file"} and isinstance(
                    value, str
                ):
                    result[key] = stage(value)
                else:
                    result[key] = walk(value)
            return result
        if isinstance(node, list):
            return [walk(item) for item in node]
        return node

    staged = walk(manifest)
    if copied:
        info(f"staged {len(copied)} asset(s) into {assets.name}/")
    return staged


def write_manifest(manifest: dict[str, Any], directory: Path) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "manifest.yaml"
    path.write_text(
        "# Fully resolved run manifest. Everything the simulator read is here, by value.\n"
        "# Re-run with: sim chrono run <this file>\n\n"
        + yaml.safe_dump(manifest, sort_keys=False, default_flow_style=None)
    )
    return path
