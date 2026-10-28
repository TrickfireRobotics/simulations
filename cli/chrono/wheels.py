"""Wheel packages: import, validate, list and inspect.

A wheel is a directory, not a loose OBJ next to a TOML. It holds the manifest, the
visual mesh, the contact mesh, the CAD source it came from, and a preview - so a wheel
that produced a result two months ago can still be reproduced exactly.
"""

from __future__ import annotations

import shutil
from dataclasses import dataclass
from pathlib import Path

import yaml

from ..output import die, info, warn
from ..paths import CHRONO_WHEELS_DIR
from . import meshtools
from .schema import SchemaError, find_placeholders, validate

WHEEL_SCHEMA_VERSION = 1

# Default contact-mesh budget. High enough to keep grousers on a rover wheel, low enough
# that Bullet's mesh-mesh tests stay affordable with six wheels on screen.
DEFAULT_COLLISION_FACES = 3000


@dataclass
class WheelPackage:
    name: str
    directory: Path
    manifest: dict

    @property
    def manifest_path(self) -> Path:
        return self.directory / "wheel.yaml"

    @property
    def revision(self) -> str:
        return str(self.manifest.get("revision", ""))

    @property
    def radius(self) -> float:
        return float(self.manifest["geometry"]["radius"])

    @property
    def mass(self) -> float:
        return float(self.manifest["mass_properties"]["mass"])

    @property
    def provenance(self) -> str:
        return str(self.manifest["mass_properties"].get("provenance", "placeholder"))


def packages_dir() -> Path:
    return CHRONO_WHEELS_DIR


def list_packages() -> list[WheelPackage]:
    """Every wheel package, sorted by name. Unreadable ones are skipped with a warning."""
    root = packages_dir()
    if not root.is_dir():
        return []

    packages = []
    for directory in sorted(p for p in root.iterdir() if p.is_dir()):
        manifest_path = directory / "wheel.yaml"
        if not manifest_path.is_file():
            continue
        try:
            manifest = yaml.safe_load(manifest_path.read_text()) or {}
        except yaml.YAMLError as exc:
            warn(f"skipping {directory.name}: {exc}")
            continue
        packages.append(WheelPackage(directory.name, directory, manifest))
    return packages


def load_package(name: str) -> WheelPackage:
    directory = packages_dir() / name
    manifest_path = directory / "wheel.yaml"
    if not manifest_path.is_file():
        available = ", ".join(p.name for p in list_packages()) or "(none)"
        die(f"no wheel package named '{name}'. Available: {available}")
    return WheelPackage(name, directory, yaml.safe_load(manifest_path.read_text()) or {})


def resolve_path(package: WheelPackage, relative: str) -> Path:
    """Resolve a path from a wheel manifest against its own directory."""
    path = Path(relative)
    return path if path.is_absolute() else package.directory / path


# ---------------------------------------------------------------------------- #
#                                    import                                    #
# ---------------------------------------------------------------------------- #


def import_wheel(
    source: Path,
    name: str,
    units: str,
    roll_axis: str,
    mass: float | None = None,
    density: float | None = None,
    radius: float | None = None,
    hub_origin: list[float] | None = None,
    collision_faces: int = DEFAULT_COLLISION_FACES,
    revision: str = "",
    material: str = "",
    notes: str = "",
    force: bool = False,
) -> WheelPackage:
    """Build a wheel package from a CAD mesh export.

    Nothing is inferred silently: units and roll axis are required arguments, and a mass
    is either measured (--mass) or computed from a closed mesh and a density (--density).
    """
    directory = packages_dir() / name
    if directory.exists() and not force:
        die(f"{directory} already exists. Pass --force to replace it, or pick another --name.")

    if mass is None and density is None:
        die(
            "give either --mass (measured, preferred) or --density (computed from the "
            "mesh volume). A wheel with no mass source cannot be simulated honestly."
        )

    info(f"loading {source}")
    raw = meshtools.load(source)

    report = meshtools.MeshReport()
    report.merge(meshtools.check_geometry(raw))

    normalised, transform = meshtools.normalise(raw, units, roll_axis, hub_origin)
    report.merge(meshtools.check_scale(normalised, "y"))
    report.merge(meshtools.check_roll_axis(normalised, "y"))

    measured_radius, measured_width = meshtools.measure(normalised, "y")
    final_radius = radius if radius is not None else measured_radius

    if radius is not None and abs(radius - measured_radius) > 0.02 * max(radius, 1e-9):
        report.warnings.append(
            f"--radius {radius:.4f} m differs from the mesh's {measured_radius:.4f} m by "
            f"{100 * abs(radius - measured_radius) / radius:.1f}%. The given value is used "
            "for the start height and the SCM active domain."
        )

    computed = meshtools.mass_properties(normalised, density) if density else None

    if mass is not None:
        properties = {"mass": mass, "provenance": "measured"}
        if computed:
            # A measured mass with a CAD tensor: scale the tensor by the mass ratio, which
            # is right as long as the real part has the same shape as the model.
            ratio = mass / float(computed["mass"])
            properties["com"] = computed["com"]
            properties["inertia_xx"] = [round(v * ratio, 9) for v in computed["inertia_xx"]]
            properties["inertia_xy"] = [round(v * ratio, 9) for v in computed["inertia_xy"]]
            properties["inertia_provenance"] = "cad, scaled to the measured mass"
        else:
            estimate = meshtools.cylinder_inertia(mass, final_radius, measured_width)
            properties["com"] = [0.0, 0.0, 0.0]
            properties.update(estimate)
            properties["provenance"] = "estimated"
            properties["inertia_provenance"] = "solid-cylinder estimate"
            report.warnings.append(
                "the mesh is not closed, so the inertia tensor is a solid-cylinder "
                "estimate and the package is marked 'estimated'. A scenario requiring "
                "measured properties will refuse it."
            )
    else:
        if not computed:
            die(
                "--density was given but the mesh is not watertight, so its volume is "
                "undefined. Repair the mesh or supply --mass."
            )
        properties = dict(computed)

    report.merge(
        meshtools.drop_spin_smoke_test(final_radius, measured_width, float(properties["mass"]))
    )

    for problem in report.problems:
        warn(problem)
    for warning in report.warnings:
        warn(warning)
    if not report.ok:
        die(f"{len(report.problems)} problem(s) found - nothing was written")

    info(f"building a contact mesh ({collision_faces} triangle budget)")
    collision, collision_report = meshtools.make_collision_mesh(normalised, collision_faces)
    for problem in collision_report.problems:
        warn(problem)
    for warning in collision_report.warnings:
        warn(warning)
    if not collision_report.ok:
        die("the contact mesh does not preserve the tread - nothing was written")

    if directory.exists():
        shutil.rmtree(directory)
    directory.mkdir(parents=True)

    meshtools.write_obj(normalised, directory / "visual.obj")
    meshtools.write_obj(collision, directory / "collision.obj")

    source_copy = directory / f"source{source.suffix.lower()}"
    shutil.copy2(source, source_copy)

    if meshtools.render_preview(normalised, directory / "preview.png"):
        info("wrote preview.png")
    else:
        warn("matplotlib is unavailable, so no preview was rendered")

    manifest = {
        "schema_version": WHEEL_SCHEMA_VERSION,
        "name": name,
        "revision": revision or "r1",
        "source": {
            "file": source_copy.name,
            "original_path": str(source.resolve()),
            "content_hash": meshtools.content_hash(source),
            "units": units,
            "roll_axis": roll_axis,
            "hub_origin": transform["hub_origin_m"],
            "hub_origin_source": transform.get("hub_origin_source", "given explicitly"),
        },
        "geometry": {
            "radius": round(final_radius, 6),
            "width": round(measured_width, 6),
            "roll_axis": [0, 1, 0],
        },
        "mass_properties": properties,
        "visual": {"file": "visual.obj"},
        "collision": [{"type": "mesh", "file": "collision.obj", "margin": 0.005}],
        "material": material or "unspecified",
        "notes": notes,
        "checks": {**report.notes, **collision_report.notes},
    }

    manifest = meshtools.plain(manifest)
    manifest_path = directory / "wheel.yaml"
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False, default_flow_style=None))

    package = WheelPackage(name, directory, manifest)
    try:
        validate_package(package)
    except SchemaError as exc:
        die(f"the generated manifest does not validate, which is a bug:\n{exc}")

    info(f"wrote {directory}")
    info(f"next: sim chrono run scenarios/wheel-drop.yaml --wheel {name}")
    return package


def generate_cylinder(
    name: str,
    radius: float,
    width: float,
    mass: float,
    grouser_count: int = 0,
    grouser_height: float = 0.0,
    revision: str = "r1",
    force: bool = False,
) -> WheelPackage:
    """Generate a parametric wheel package without a CAD export.

    The smooth case is the baseline every mesh wheel is compared against: a run that
    cannot tell a paddle wheel from a plain cylinder is measuring the pipeline, not the
    wheel. Adding grousers gives a cheap, exactly-known tread for checking that the SCM
    contact actually responds to one.

    The mass properties are analytic rather than computed from the mesh, so they are
    exact for the shape and marked `cad` rather than `measured`.
    """
    import numpy as np
    import trimesh

    directory = packages_dir() / name
    if directory.exists() and not force:
        die(f"{directory} already exists. Pass --force to replace it.")

    # Chrono's wheel convention has the roll axis along the package's Y, and trimesh
    # builds cylinders about Z, so the primitive is rotated once here.
    to_y = trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])
    mesh = trimesh.creation.cylinder(radius=radius, height=width, sections=64, transform=to_y)

    if grouser_count > 0 and grouser_height > 0:
        for index in range(grouser_count):
            angle = 2 * np.pi * index / grouser_count
            grouser = trimesh.creation.box(
                extents=[grouser_height * 2, width, max(0.006, radius * 0.04)]
            )
            # Sit each grouser on the tread, pointing outward, spaced evenly around it.
            placement = trimesh.transformations.rotation_matrix(angle, [0, 1, 0])
            offset = trimesh.transformations.translation_matrix(
                [radius + grouser_height * 0.5, 0, 0]
            )
            grouser.apply_transform(placement @ offset)
            mesh = trimesh.util.concatenate([mesh, grouser])

    outer_radius = radius + grouser_height
    properties = {
        "mass": round(mass, 6),
        "com": [0.0, 0.0, 0.0],
        "provenance": "cad",
        "inertia_provenance": "analytic solid cylinder",
        **meshtools.cylinder_inertia(mass, radius, width),
    }

    if directory.exists():
        shutil.rmtree(directory)
    directory.mkdir(parents=True)

    meshtools.write_obj(mesh, directory / "visual.obj")
    meshtools.write_obj(mesh, directory / "collision.obj")
    meshtools.render_preview(mesh, directory / "preview.png")

    smooth = grouser_count == 0 or grouser_height <= 0
    manifest = meshtools.plain(
        {
            "schema_version": WHEEL_SCHEMA_VERSION,
            "name": name,
            "revision": revision,
            "source": {
                "file": "visual.obj",
                "original_path": "generated by `sim chrono wheel generate`",
                "content_hash": meshtools.content_hash(directory / "visual.obj"),
                "units": "m",
                "roll_axis": "y",
                "hub_origin": [0.0, 0.0, 0.0],
                "hub_origin_source": "generated about the hub",
            },
            "geometry": {
                "radius": round(outer_radius, 6),
                "width": round(width, 6),
                "roll_axis": [0, 1, 0],
            },
            "mass_properties": properties,
            "visual": {"file": "visual.obj"},
            # A smooth wheel uses a cylinder primitive rather than its own mesh: it is the
            # exact shape, and it spares the contact solver a few thousand triangles.
            "collision": (
                [
                    {
                        "type": "cylinder",
                        "radius": radius,
                        "length": width,
                        "rotation": [90.0, 0.0, 0.0],
                    }
                ]
                if smooth
                else [{"type": "mesh", "file": "collision.obj", "margin": 0.005}]
            ),
            "material": "generated primitive",
            "notes": (
                f"Generated: radius {radius} m, width {width} m, {grouser_count} grousers "
                f"of {grouser_height} m. Not a real wheel - a controlled reference shape."
            ),
            "checks": {
                "triangles": len(mesh.faces),
                "generated": True,
                "smooth": smooth,
            },
        }
    )

    (directory / "wheel.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False, default_flow_style=None)
    )

    package = WheelPackage(name, directory, manifest)
    validate_package(package)
    info(f"wrote {directory}")
    return package


# ---------------------------------------------------------------------------- #
#                              validate and inspect                            #
# ---------------------------------------------------------------------------- #


def validate_package(package: WheelPackage) -> list[str]:
    """Schema-validate a package and check that its referenced files exist.

    Returns the list of warnings; raises SchemaError for anything that blocks a run.
    """
    validate(package.manifest, "wheel", source=str(package.manifest_path))

    problems = []
    warnings = []

    placeholders = find_placeholders(package.manifest)
    problems += [f"{key}: still a REPLACE_WITH placeholder" for key in placeholders]

    visual = resolve_path(package, package.manifest["visual"]["file"])
    if not visual.is_file():
        problems.append(f"visual.file: {visual} is missing")

    for index, shape in enumerate(package.manifest.get("collision", [])):
        if shape.get("type") not in {"mesh", "convex_hull"}:
            continue
        path = resolve_path(package, shape["file"])
        if not path.is_file():
            problems.append(f"collision[{index}].file: {path} is missing")

    source = package.manifest.get("source", {})
    source_file = source.get("file")
    if source_file:
        path = resolve_path(package, source_file)
        if not path.is_file():
            warnings.append(f"source.file: {path} is missing, so the CAD provenance is broken")
        elif source.get("content_hash"):
            actual = meshtools.content_hash(path)
            if actual != source["content_hash"]:
                problems.append(
                    "source.content_hash does not match source.file. The CAD export "
                    "was replaced without re-importing, so the meshes and the manifest "
                    "no longer describe the same wheel."
                )

    if package.provenance == "placeholder":
        warnings.append("mass_properties.provenance is 'placeholder'")

    if problems:
        raise SchemaError(problems)
    return warnings


def inspect_package(package: WheelPackage) -> str:
    """A readable summary of one package, for `sim chrono wheel inspect`."""
    manifest = package.manifest
    geometry = manifest.get("geometry", {})
    properties = manifest.get("mass_properties", {})
    source = manifest.get("source", {})
    checks = manifest.get("checks", {})

    lines = [
        f"{package.name}  (revision {package.revision or 'unset'})",
        f"  directory      {package.directory}",
        f"  radius         {geometry.get('radius')} m",
        f"  width          {geometry.get('width')} m",
        f"  roll axis      {geometry.get('roll_axis')}",
        "",
        f"  mass           {properties.get('mass')} kg  [{properties.get('provenance')}]",
        f"  centre of mass {properties.get('com')}",
        f"  inertia (xx)   {properties.get('inertia_xx')}",
        f"  inertia (xy)   {properties.get('inertia_xy')}",
    ]
    if properties.get("inertia_provenance"):
        lines.append(f"  inertia source {properties['inertia_provenance']}")

    lines += [
        "",
        (
            f"  source         {source.get('file')}  ({source.get('units')}, "
            f"roll axis {source.get('roll_axis')})"
        ),
        f"  hash           {source.get('content_hash')}",
        f"  hub origin     {source.get('hub_origin')}  [{source.get('hub_origin_source')}]",
        f"  material       {manifest.get('material')}",
    ]
    if manifest.get("notes"):
        lines.append(f"  notes          {manifest['notes']}")

    if checks:
        lines += ["", "  import checks"]
        for key in sorted(checks):
            lines.append(f"    {key:<28} {checks[key]}")

    return "\n".join(lines)
