"""Sweep orchestration: expanding a study into runs, executing them, and resuming.

A study crosses scenarios, wheels, actuators, terrains, seeds and a parameter grid. Each
combination becomes a run with its own directory and manifest, so a sweep is a set of
independent, individually reproducible runs rather than one monolithic job.

Interrupted sweeps are resumable: a run is "done" when its summary.json exists, so
--resume picks up exactly where it stopped without re-running anything.
"""

from __future__ import annotations

import argparse
import itertools
import json
import os
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from ..output import die, info, warn
from ..paths import REPO_DIR
from . import configs, results, runner
from . import manifest as manifest_mod
from . import report as report_mod
from .schema import SchemaError


@dataclass
class Job:
    """One run in a sweep."""

    run_id: str
    scenario: str
    seed: int
    wheel: str | None = None
    actuator: str | None = None
    terrain: str | None = None
    overrides: dict[str, Any] = field(default_factory=dict)

    exit_code: int | None = None
    error: str = ""

    def describe(self) -> str:
        parts = [self.scenario]
        if self.wheel:
            parts.append(self.wheel)
        if self.actuator:
            parts.append(self.actuator)
        if self.terrain:
            parts.append(self.terrain)
        parts += [f"{k}={v}" for k, v in sorted(self.overrides.items())]
        parts.append(f"seed {self.seed}")
        return " / ".join(parts)


def expand(study: dict[str, Any], args: argparse.Namespace) -> list[Job]:
    """Cross everything a study varies into a flat list of jobs.

    Command-line lists override the study's own, so one study file serves both "the
    standard comparison" and "just these two wheels, quickly".
    """
    scenarios = study.get("scenario")
    scenarios = [scenarios] if isinstance(scenarios, str) else list(scenarios or [])
    if not scenarios:
        die("the study names no scenario")

    def pick(flag: str | None, key: str) -> list[str | None]:
        if flag:
            return [part.strip() for part in flag.split(",") if part.strip()]
        values = study.get(key) or []
        return list(values) if values else [None]

    wheels = pick(args.wheels, "wheels")
    actuators = pick(args.actuators, "actuators")
    terrains = pick(args.terrains, "terrains")

    if args.seeds:
        seeds = [int(part) for part in args.seeds.split(",") if part.strip()]
    else:
        seeds = [int(s) for s in (study.get("seeds") or [1])]

    # The grid is a cross product of dotted scenario keys.
    grid = study.get("grid") or {}
    grid_keys = sorted(grid)
    grid_rows = (
        [
            dict(zip(grid_keys, values))
            for values in itertools.product(*(grid[k] for k in grid_keys))
        ]
        if grid_keys
        else [{}]
    )

    jobs = []
    for index, (scenario, wheel, actuator, terrain, overrides, seed) in enumerate(
        itertools.product(scenarios, wheels, actuators, terrains, grid_rows, seeds)
    ):
        jobs.append(
            Job(
                run_id=f"run-{index:03d}",
                scenario=scenario,
                seed=seed,
                wheel=wheel,
                actuator=actuator,
                terrain=terrain,
                overrides=dict(overrides),
            )
        )
    return jobs


def run_sweep(args: argparse.Namespace) -> None:
    try:
        study, study_path, chain = configs.load("study", args.study)
    except configs.ConfigError as exc:
        die(str(exc))

    try:
        from .schema import validate

        validate(study, "study", source=str(study_path))
    except SchemaError as exc:
        die(str(exc))

    jobs = expand(study, args)

    if args.resume:
        directory = results.resolve_study(args.resume)
        info(f"resuming {_relative(directory)}")
    elif args.out:
        directory = Path(args.out).resolve()
    else:
        directory = results.study_dir(manifest_mod.timestamp(), study["name"])

    directory.mkdir(parents=True, exist_ok=True)

    # Keep the study definition next to the results, so the report knows which objectives
    # to rank on and a reader can see what was intended.
    (directory / "study.yaml").write_text(
        f"# Resolved study definition for {study['name']}.\n"
        f"# Source: {_relative(study_path)}\n"
        + (
            "# Inherited from: " + ", ".join(p.name for p in chain[1:]) + "\n"
            if len(chain) > 1
            else ""
        )
        + "\n"
        + yaml.safe_dump(study, sort_keys=False, default_flow_style=None)
    )

    pending = []
    skipped = 0
    for job in jobs:
        run_directory = directory / job.run_id
        if args.resume and (run_directory / "summary.json").is_file():
            skipped += 1
            continue
        pending.append(job)

    info(f"{len(jobs)} run(s) in this study" + (f", {skipped} already finished" if skipped else ""))

    if args.dry_run:
        for job in jobs:
            print(f"  {job.run_id}  {job.describe()}")
        return

    if not pending:
        info("nothing left to run")
        _finalise(directory, args)
        return

    jobs_in_parallel = max(1, args.jobs)
    if jobs_in_parallel > 1:
        # Each run already uses several threads for SCM, so oversubscribing turns a
        # parallel sweep into a slower one. Split the machine between the runs instead.
        per_run = max(1, (os.cpu_count() or 4) // jobs_in_parallel)
        info(f"{jobs_in_parallel} runs in parallel, {per_run} solver thread(s) each")
    else:
        per_run = None

    started = time.monotonic()
    completed = 0

    def execute(job: Job) -> Job:
        run_directory = directory / job.run_id

        try:
            manifest_path = runner.prepare(
                job.scenario,
                run_id=job.run_id,
                directory=run_directory,
                study=study["name"],
                seed=job.seed,
                headless=True,
                wheel=job.wheel,
                actuator=job.actuator,
                terrain=job.terrain,
                overrides=dict(job.overrides),
                # Solver threads are a property of the machine, not of the scenario, so
                # they are passed separately rather than smuggled in as an override.
                threads=per_run,
            )
        except Exception as exc:  # noqa: BLE001 - one bad combination must not stop the sweep
            job.error = str(exc)
            job.exit_code = 3
            _write_failure(run_directory, job)
            return job

        job.exit_code = runner.launch(manifest_path, headless=True, timeout=args.timeout)
        runner.finish(run_directory, keep_plain_csv=args.keep_csv)
        return job

    finished: list[Job] = []
    try:
        if jobs_in_parallel == 1:
            for job in pending:
                finished.append(execute(job))
                completed += 1
                _progress(completed, len(pending), started, finished[-1])
        else:
            with ThreadPoolExecutor(max_workers=jobs_in_parallel) as pool:
                futures = {pool.submit(execute, job): job for job in pending}
                for future in as_completed(futures):
                    finished.append(future.result())
                    completed += 1
                    _progress(completed, len(pending), started, finished[-1])
    except KeyboardInterrupt:
        warn(
            f"interrupted after {completed}/{len(pending)} run(s). Resume with:\n"
            f"  sim chrono sweep {args.study} --resume {_relative(directory)}"
        )
        _finalise(directory, args)
        raise SystemExit(130) from None

    broken = [job for job in finished if job.error]
    for job in broken:
        warn(f"{job.run_id} ({job.describe()}) could not be set up: {job.error}")

    _finalise(directory, args)

    if broken:
        raise SystemExit(1)


def _progress(done: int, total: int, started: float, job: Job) -> None:
    elapsed = time.monotonic() - started
    rate = elapsed / done if done else 0
    remaining = rate * (total - done)
    state = "setup failed" if job.error else f"exit {job.exit_code}"
    info(
        f"[{done}/{total}] {job.run_id} {state}  "
        f"({elapsed / 60:.1f} min elapsed, ~{remaining / 60:.1f} min left)"
    )


def _write_failure(directory: Path, job: Job) -> None:
    """Leave a usable diagnostic for a run that could not even be set up.

    The plan asks that failed or interrupted runs retain useful diagnostics. Without
    this, a sweep with one bad combination leaves an empty directory and no explanation.
    """
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "summary.json").write_text(
        json.dumps(
            {
                "passed": False,
                "samples": 0,
                "termination": {
                    "reason": "setup_failed",
                    "rule": "setup",
                    "detail": job.error,
                    "time": 0,
                    "pass": False,
                },
                "checks": [],
                "metrics": {},
                "sweep_job": {
                    "scenario": job.scenario,
                    "wheel": job.wheel,
                    "actuator": job.actuator,
                    "terrain": job.terrain,
                    "seed": job.seed,
                    "overrides": job.overrides,
                },
            },
            indent=2,
        )
        + "\n"
    )


def _finalise(directory: Path, args: argparse.Namespace) -> None:
    runs = results.load_study(directory)
    if not runs:
        warn(f"no finished runs in {_relative(directory)}")
        return

    passed = sum(1 for run in runs if run.passed)
    info(f"{passed}/{len(runs)} run(s) passed")

    if getattr(args, "no_report", False):
        return
    try:
        path = report_mod.build_report(directory)
        info(f"report: {_relative(path)}")
    except Exception as exc:  # noqa: BLE001 - a report failure must not lose the results
        warn(f"could not build the report ({exc}); the results are still in {directory}")


def _relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_DIR))
    except ValueError:
        return str(path)
