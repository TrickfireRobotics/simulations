"""Argument parsing and dispatch for `sim chrono`.

The command surface follows the development plan's target workflow:

    sim chrono wheel import|generate|list|validate|inspect
    sim chrono rover import
    sim chrono terrain heightmap|rocks
    sim chrono validate <kind> <config>
    sim chrono run <scenario|manifest> [--wheel ...] [--headless]
    sim chrono sweep <study> [--jobs N] [--resume]
    sim chrono report|compare <results dir>
    sim chrono list <kind>
"""

from __future__ import annotations

import argparse
from pathlib import Path

from ..output import die, info, warn
from ..paths import CHRONO_DIR, REPO_DIR


def build_parser(subparsers: argparse._SubParsersAction) -> tuple:
    chrono = subparsers.add_parser("chrono", help="Chrono rover, wheel and terrain simulation")
    commands = chrono.add_subparsers(dest="command")

    _add_run(commands)
    _add_sweep(commands)
    _add_wheel(commands)
    _add_rover(commands)
    _add_terrain(commands)
    _add_report(commands)
    _add_inspection(commands)

    commands.add_parser("build", help="Build the simulator")
    commands.add_parser("clean", help="Delete the simulator's build directory")

    return chrono, commands


# ---------------------------------------------------------------------------- #
#                                   parsers                                    #
# ---------------------------------------------------------------------------- #


def _add_run(commands: argparse._SubParsersAction) -> None:
    run = commands.add_parser(
        "run",
        help="Run one scenario, or re-run a saved manifest",
        description=(
            "Resolves a scenario into a run manifest, writes it to a fresh results "
            "directory with its assets, and runs it. Pass a saved manifest.yaml instead "
            "to re-run an old result exactly."
        ),
    )
    run.add_argument(
        "target",
        nargs="?",
        default=None,
        help="scenario name or path, or a saved manifest.yaml (default: the smoke scenario)",
    )
    run.add_argument("--wheel", help="use this wheel package on every station")
    run.add_argument("--actuator", help="use this actuator on every station")
    run.add_argument("--terrain", help="override the scenario's terrain")
    run.add_argument("--rover", help="override the scenario's rover")
    run.add_argument("--seed", type=int, default=1, help="random seed (default: 1)")
    run.add_argument(
        "--headless",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="run without a window (default: windowed for `run`, headless in a sweep)",
    )
    run.add_argument("--out", help="results directory to write into")
    run.add_argument("--study", default="", help="label for the results directory")
    run.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="override a scenario key, e.g. --set driver.value=4.0 (repeatable)",
    )
    run.add_argument(
        "--timeout",
        type=float,
        default=None,
        help="wall-clock budget in seconds; the run is stopped if it is exceeded",
    )
    run.add_argument(
        "--dry-run",
        action="store_true",
        help="resolve and validate the manifest, but do not run it",
    )
    run.add_argument("--keep-csv", action="store_true", help="keep the uncompressed telemetry")
    run.add_argument("--report", action="store_true", help="build the HTML report afterwards")


def _add_sweep(commands: argparse._SubParsersAction) -> None:
    sweep = commands.add_parser(
        "sweep",
        help="Run a parameter study",
        description=(
            "Crosses a study's scenarios, wheels, terrains, seeds and parameter grid, "
            "runs every combination headlessly, and builds a comparison report."
        ),
    )
    sweep.add_argument("study", help="study name or path")
    sweep.add_argument("--jobs", type=int, default=1, help="runs to execute in parallel")
    sweep.add_argument("--wheels", help="comma-separated wheel packages, overriding the study")
    sweep.add_argument("--actuators", help="comma-separated actuators, overriding the study")
    sweep.add_argument("--terrains", help="comma-separated terrains, overriding the study")
    sweep.add_argument("--seeds", help="comma-separated seeds, overriding the study")
    sweep.add_argument("--out", help="results directory to write into")
    sweep.add_argument(
        "--resume",
        help="continue an interrupted sweep in this results directory, skipping finished runs",
    )
    sweep.add_argument(
        "--timeout", type=float, default=None, help="per-run wall-clock budget in seconds"
    )
    sweep.add_argument(
        "--dry-run", action="store_true", help="list the runs the study expands to, then stop"
    )
    sweep.add_argument("--keep-csv", action="store_true")
    sweep.add_argument(
        "--no-report", action="store_true", help="skip the comparison report at the end"
    )


def _add_wheel(commands: argparse._SubParsersAction) -> None:
    wheel = commands.add_parser("wheel", help="Wheel packages")
    actions = wheel.add_subparsers(dest="action")

    imp = actions.add_parser(
        "import",
        help="Build a wheel package from a CAD mesh export",
        description=(
            "Units and roll axis are required: a wheel ten times too large still runs "
            "and still produces plausible-looking data, so neither is ever inferred."
        ),
    )
    imp.add_argument("source", help="STL or OBJ export (STEP needs converting first)")
    imp.add_argument("--name", required=True, help="package name")
    imp.add_argument(
        "--units",
        required=True,
        choices=["m", "mm", "cm", "in", "ft"],
        help="units the mesh was authored in",
    )
    imp.add_argument(
        "--roll-axis",
        required=True,
        choices=["x", "y", "z"],
        help="the mesh's axis of rotation",
    )
    imp.add_argument("--mass", type=float, help="measured mass in kg (preferred)")
    imp.add_argument(
        "--density",
        type=float,
        help="kg/m3, to compute mass from a closed mesh when none has been measured",
    )
    imp.add_argument("--radius", type=float, help="override the radius measured from the mesh")
    imp.add_argument(
        "--hub-origin",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="hub centre in the mesh's own units; defaults to the bounding-box centre",
    )
    imp.add_argument(
        "--collision-faces",
        type=int,
        default=3000,
        help="triangle budget for the contact mesh (default: 3000)",
    )
    imp.add_argument("--revision", default="", help="hardware revision label")
    imp.add_argument("--material", default="", help="what it is made of")
    imp.add_argument("--notes", default="")
    imp.add_argument("--force", action="store_true", help="replace an existing package")

    gen = actions.add_parser(
        "generate",
        help="Generate a parametric reference wheel without a CAD export",
        description=(
            "The smooth cylinder is the baseline every mesh wheel is compared against; "
            "the grousered version is a controlled tread for checking that SCM responds."
        ),
    )
    gen.add_argument("--name", required=True)
    gen.add_argument("--radius", type=float, required=True, help="m, to the tread surface")
    gen.add_argument("--width", type=float, required=True, help="m")
    gen.add_argument("--mass", type=float, required=True, help="kg")
    gen.add_argument("--grousers", type=int, default=0, help="number of radial grousers")
    gen.add_argument("--grouser-height", type=float, default=0.0, help="m")
    gen.add_argument("--revision", default="r1")
    gen.add_argument("--force", action="store_true")

    actions.add_parser("list", help="List wheel packages")

    validate = actions.add_parser("validate", help="Validate wheel packages")
    validate.add_argument("name", nargs="?", help="package name (default: all of them)")

    inspect = actions.add_parser("inspect", help="Show one package in detail")
    inspect.add_argument("name")


def _add_rover(commands: argparse._SubParsersAction) -> None:
    rover = commands.add_parser("rover", help="Rover models")
    actions = rover.add_subparsers(dest="action")

    imp = actions.add_parser(
        "import",
        help="Generate a rover config from a URDF and its hints file",
        description=(
            "Renders the Xacro, collapses fixed joints, restores the pivots named in the "
            "hints file, converts the meshes and approximates the chassis collision volume."
        ),
    )
    imp.add_argument(
        "name",
        help="rover name; reads configs/rovers/<name>.import.yaml",
    )
    imp.add_argument("--out", help="write the config here instead of configs/rovers/")

    actions.add_parser("list", help="List rover configs")


def _add_terrain(commands: argparse._SubParsersAction) -> None:
    terrain = commands.add_parser("terrain", help="Terrain preparation")
    actions = terrain.add_subparsers(dest="action")

    heightmap = actions.add_parser(
        "heightmap",
        help="Convert a DEM, GeoTIFF or point cloud into an SCM height map",
        description=(
            "Crops, levels and resamples the source, writes a greyscale height map plus "
            "a metadata sidecar recording the CRS, transform, resolution, vertical scale, "
            "crop and content hash, and prints the terrain config stanza to paste in."
        ),
    )
    heightmap.add_argument("source", help="GeoTIFF, ASCII grid, or XYZ point cloud")
    heightmap.add_argument("--name", required=True)
    heightmap.add_argument(
        "--crop",
        nargs=4,
        type=float,
        metavar=("MINX", "MINY", "MAXX", "MAXY"),
        help="crop window in the source's own coordinates",
    )
    heightmap.add_argument(
        "--resolution", type=float, default=0.05, help="output grid spacing in metres"
    )
    heightmap.add_argument(
        "--level",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="remove the mean plane so the patch is not a tilted slab (default: on)",
    )
    heightmap.add_argument(
        "--vertical-scale", type=float, default=1.0, help="exaggerate or shrink relief"
    )
    heightmap.add_argument("--force", action="store_true")

    rocks = actions.add_parser(
        "rocks", help="Expand a terrain's rock_field into explicit obstacles and print them"
    )
    rocks.add_argument("terrain", help="terrain config name")
    rocks.add_argument("--seed", type=int, help="override the config's seed")

    actions.add_parser("list", help="List terrain configs")


def _add_report(commands: argparse._SubParsersAction) -> None:
    report = commands.add_parser("report", help="Rebuild a report from saved results")
    report.add_argument(
        "results", nargs="?", help="results directory (default: the most recent one)"
    )
    report.add_argument("--open", action="store_true", help="print the path to open")

    compare = commands.add_parser(
        "compare", help="Compare runs across one or more results directories"
    )
    compare.add_argument("results", nargs="+", help="results directories")
    compare.add_argument(
        "--metric", action="append", default=[], help="summary metric to compare (repeatable)"
    )


def _add_inspection(commands: argparse._SubParsersAction) -> None:
    validate = commands.add_parser(
        "validate", help="Validate a config against its schema, without running"
    )
    validate.add_argument(
        "kind",
        choices=[
            "rover",
            "terrain",
            "scenario",
            "soil",
            "actuator",
            "battery",
            "study",
            "manifest",
            "all",
        ],
    )
    validate.add_argument("config", nargs="?", help="config name or path (default: all of them)")

    listing = commands.add_parser("list", help="List configs of a kind")
    listing.add_argument(
        "kind",
        nargs="?",
        default="all",
        choices=[
            "rover",
            "terrain",
            "scenario",
            "soil",
            "actuator",
            "battery",
            "study",
            "wheel",
            "all",
        ],
    )

    # The prototype's command, kept working so old notes and muscle memory still do.
    wheels = commands.add_parser("wheels", help="List wheel packages (alias for `wheel list`)")
    wheels.set_defaults(action="list")


# ---------------------------------------------------------------------------- #
#                                   dispatch                                   #
# ---------------------------------------------------------------------------- #


def dispatch(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
    handlers = {
        "run": _run,
        "sweep": _sweep,
        "wheel": _wheel,
        "wheels": _wheel_list,
        "rover": _rover,
        "terrain": _terrain,
        "report": _report,
        "compare": _compare,
        "validate": _validate,
        "list": _list,
        "build": _build,
        "clean": _clean,
    }
    handler = handlers.get(args.command)
    if not handler:
        parser.print_help()
        return
    handler(args)


def _parse_overrides(pairs: list[str]) -> dict[str, object]:
    """Parse `--set key=value` into a dotted-path override dict."""
    import yaml

    overrides: dict[str, object] = {}
    for pair in pairs:
        if "=" not in pair:
            die(f"--set expects KEY=VALUE, got '{pair}'")
        key, _, raw = pair.partition("=")
        # YAML-parse the value so numbers stay numbers and `true` stays a boolean.
        overrides[key.strip()] = yaml.safe_load(raw)
    return overrides


def _run(args: argparse.Namespace) -> None:
    from . import manifest as manifest_mod
    from . import report as report_mod
    from . import results, runner
    from .configs import ConfigError
    from .manifest import ManifestError
    from .schema import SchemaError

    target = args.target or "smoke"
    overrides = _parse_overrides(args.set)

    is_manifest = Path(target).name in {"manifest.yaml", "manifest.yml"} and Path(target).is_file()
    headless = args.headless if args.headless is not None else is_manifest

    timestamp = manifest_mod.timestamp()
    study = args.study or (Path(target).stem if is_manifest else target.replace("/", "-"))
    directory = (
        Path(args.out).resolve() if args.out else results.study_dir(timestamp, study) / "run-000"
    )

    try:
        if is_manifest:
            if overrides:
                die(
                    "--set cannot be combined with re-running a manifest: the point of a "
                    "manifest is that nothing is re-resolved. Run the scenario instead."
                )
            manifest_path = runner.prepare_from_manifest(Path(target).resolve(), directory)
        else:
            manifest_path = runner.prepare(
                target,
                run_id="run-000",
                directory=directory,
                study=study,
                seed=args.seed,
                headless=headless,
                wheel=args.wheel,
                actuator=args.actuator,
                terrain=args.terrain,
                rover=args.rover,
                overrides=overrides,
            )
    except (ConfigError, ManifestError, SchemaError, runner.RunError) as exc:
        die(str(exc))

    info(f"manifest: {manifest_path}")

    if args.dry_run:
        code = runner.validate(manifest_path)
        info("manifest validates" if code == 0 else "the simulator rejected the manifest")
        return

    code = runner.launch(manifest_path, headless=headless, timeout=args.timeout)
    result = runner.finish(directory, keep_plain_csv=args.keep_csv)
    runner.report_outcome(result, code)

    if args.report and result:
        path = report_mod.build_report(directory.parent)
        info(f"report: {path}")

    if code not in (0, 1):
        raise SystemExit(code)
    if result and not result.passed:
        raise SystemExit(1)


def _sweep(args: argparse.Namespace) -> None:
    from . import study as study_mod

    study_mod.run_sweep(args)


def _wheel(args: argparse.Namespace) -> None:
    from . import wheels
    from .schema import SchemaError

    action = getattr(args, "action", None)

    if action == "import":
        wheels.import_wheel(
            source=Path(args.source).resolve(),
            name=args.name,
            units=args.units,
            roll_axis=args.roll_axis,
            mass=args.mass,
            density=args.density,
            radius=args.radius,
            hub_origin=args.hub_origin,
            collision_faces=args.collision_faces,
            revision=args.revision,
            material=args.material,
            notes=args.notes,
            force=args.force,
        )
    elif action == "generate":
        wheels.generate_cylinder(
            name=args.name,
            radius=args.radius,
            width=args.width,
            mass=args.mass,
            grouser_count=args.grousers,
            grouser_height=args.grouser_height,
            revision=args.revision,
            force=args.force,
        )
    elif action == "list":
        _wheel_list(args)
    elif action == "validate":
        packages = [wheels.load_package(args.name)] if args.name else wheels.list_packages()
        if not packages:
            die("no wheel packages to validate")
        failed = 0
        for package in packages:
            try:
                for warning in wheels.validate_package(package):
                    warn(f"{package.name}: {warning}")
                info(f"{package.name}: ok")
            except SchemaError as exc:
                failed += 1
                print(f"{package.name}:\n{exc}")
        if failed:
            raise SystemExit(1)
    elif action == "inspect":
        print(wheels.inspect_package(wheels.load_package(args.name)))
    else:
        die("expected one of: import, generate, list, validate, inspect")


def _wheel_list(args: argparse.Namespace) -> None:
    from . import wheels

    packages = wheels.list_packages()
    if not packages:
        info(f"no wheel packages in {wheels.packages_dir()}")
        info("create one with `sim chrono wheel import` or `sim chrono wheel generate`")
        return

    print(f"{'name':<22} {'rev':<14} {'radius':>8} {'width':>8} {'mass':>8}  provenance")
    for package in packages:
        geometry = package.manifest.get("geometry", {})
        print(
            f"{package.name:<22} {package.revision:<14} "
            f"{geometry.get('radius', 0):>8.4f} {geometry.get('width', 0):>8.4f} "
            f"{package.mass:>8.3f}  {package.provenance}"
        )


def _rover(args: argparse.Namespace) -> None:
    from . import configs, rover
    from .configs import kind_dir

    action = getattr(args, "action", None)
    if action == "import":
        hints = kind_dir("rover") / f"{args.name}.import.yaml"
        if not hints.is_file():
            available = sorted(
                p.stem.replace(".import", "") for p in kind_dir("rover").glob("*.import.yaml")
            )
            die(
                f"no hints file at {hints}. Available: {', '.join(available) or '(none)'}. "
                "A rover import needs one: it says which fixed joints are really pivots."
            )
        try:
            rover.import_rover(hints, Path(args.out).resolve() if args.out else None)
        except rover.RoverImportError as exc:
            die(str(exc))
    elif action == "list":
        for name, description in configs.list_configs("rover"):
            if name.endswith(".import"):
                continue
            print(f"{name:<24} {description}")
    else:
        die("expected one of: import, list")


def _terrain(args: argparse.Namespace) -> None:
    from . import configs, terrain_prep

    action = getattr(args, "action", None)
    if action == "heightmap":
        try:
            terrain_prep.build_heightmap(
                source=Path(args.source).resolve(),
                name=args.name,
                crop=args.crop,
                resolution=args.resolution,
                level=args.level,
                vertical_scale=args.vertical_scale,
                force=args.force,
            )
        except terrain_prep.TerrainPrepError as exc:
            die(str(exc))
    elif action == "rocks":
        terrain_prep.print_rock_field(args.terrain, args.seed)
    elif action == "list":
        for name, description in configs.list_configs("terrain"):
            print(f"{name:<28} {description}")
    else:
        die("expected one of: heightmap, rocks, list")


def _report(args: argparse.Namespace) -> None:
    from . import report as report_mod
    from . import results

    try:
        directory = results.resolve_study(args.results)
    except FileNotFoundError as exc:
        die(str(exc))

    path = report_mod.build_report(directory)
    info(f"report: {path}")
    if args.open:
        print(path)


def _compare(args: argparse.Namespace) -> None:
    from . import report as report_mod
    from . import results

    studies = []
    for reference in args.results:
        try:
            studies.append(results.resolve_study(reference))
        except FileNotFoundError as exc:
            die(str(exc))

    report_mod.print_comparison(studies, args.metric)


def _validate(args: argparse.Namespace) -> None:
    from . import configs, wheels
    from .schema import SchemaError, find_placeholders, validate

    kinds = (
        ["rover", "terrain", "scenario", "soil", "actuator", "battery", "study"]
        if args.kind == "all"
        else [args.kind]
    )

    failures = 0
    placeholders_found = 0

    for kind in kinds:
        if kind == "manifest":
            if not args.config:
                die("validating a manifest needs a path")
            import yaml

            from . import manifest as manifest_mod

            data = yaml.safe_load(Path(args.config).read_text())
            try:
                manifest_mod.validate_manifest(data, source=args.config)
                info(f"{args.config}: ok")
            except SchemaError as exc:
                failures += 1
                print(str(exc))
            continue

        names = (
            [args.config]
            if args.config
            else [name for name, _ in configs.list_configs(kind) if not name.endswith(".import")]
        )
        for name in names:
            try:
                data, path, _ = configs.load(kind, name)
                validate(data, configs.schema_for(kind), source=str(path))
                placeholders = find_placeholders(data)
                if placeholders:
                    placeholders_found += 1
                    warn(
                        f"{kind}/{name}: {len(placeholders)} unmeasured placeholder(s): "
                        + ", ".join(placeholders[:4])
                        + ("..." if len(placeholders) > 4 else "")
                    )
                else:
                    info(f"{kind}/{name}: ok")
            except (configs.ConfigError, SchemaError) as exc:
                failures += 1
                print(f"{kind}/{name}:\n{exc}")

    if args.kind in {"all", "wheel"} and not args.config:
        for package in wheels.list_packages():
            try:
                wheels.validate_package(package)
                info(f"wheel/{package.name}: ok")
            except SchemaError as exc:
                failures += 1
                print(f"wheel/{package.name}:\n{exc}")

    if placeholders_found:
        warn(
            f"{placeholders_found} config(s) hold REPLACE_WITH placeholders. They validate "
            "structurally but the simulator will refuse to run them, which is intended: "
            "see docs/chrono/development-plan.mdx."
        )
    if failures:
        raise SystemExit(1)


def _list(args: argparse.Namespace) -> None:
    from . import configs

    kinds = (
        ["wheel", "rover", "terrain", "scenario", "soil", "actuator", "battery", "study"]
        if args.kind == "all"
        else [args.kind]
    )

    for kind in kinds:
        print(f"\n{kind}s")
        print("-" * (len(kind) + 1))
        if kind == "wheel":
            _wheel_list(args)
            continue
        entries = configs.list_configs(kind)
        if not entries:
            print("  (none)")
        for name, description in entries:
            if name.endswith(".import"):
                continue
            summary = " ".join(description.split())[:70]
            print(f"  {name:<28} {summary}")


def _build(args: argparse.Namespace) -> None:
    from . import runner

    try:
        binary = runner.ensure_built(rebuild=True)
    except runner.RunError as exc:
        die(str(exc))
    info(f"built {binary.relative_to(REPO_DIR)}")


def _clean(args: argparse.Namespace) -> None:
    import shutil

    build = CHRONO_DIR / "build"
    if build.exists():
        shutil.rmtree(build)
        info(f"removed {build.relative_to(REPO_DIR)}")
    else:
        info("nothing to clean")
