"""Mesh loading, checking and conversion for the wheel import pipeline.

Everything here is deliberately suspicious of its input. A wheel mesh that is ten times
too large, authored about the wrong axis, or not watertight will still load, still render
and still produce a run - and the numbers from that run will be wrong in a way nobody
notices. So each check reports what it found rather than silently repairing it.
"""

from __future__ import annotations

import hashlib
import itertools
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import trimesh

# unit name -> metres per unit
UNITS = {
    "m": 1.0,
    "mm": 1e-3,
    "cm": 1e-2,
    "in": 0.0254,
    "ft": 0.3048,
}

AXES = {"x": np.array([1.0, 0, 0]), "y": np.array([0, 1.0, 0]), "z": np.array([0, 0, 1.0])}

# A wheel outside this radius range is almost certainly a unit mistake rather than an
# unusual design. The same bounds are enforced by the simulator's manifest loader.
MIN_RADIUS_M = 0.02
MAX_RADIUS_M = 2.0


class MeshError(Exception):
    pass


def plain(value):
    """Convert numpy scalars and arrays into plain Python types.

    Numbers that come back from numpy are np.float64, which PyYAML refuses to represent.
    Everything written into a wheel manifest goes through here so a manifest is readable
    by any YAML parser, not only one with numpy loaded.
    """
    if isinstance(value, dict):
        return {str(k): plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if isinstance(value, np.ndarray):
        return [plain(v) for v in value.tolist()]
    if isinstance(value, np.generic):
        return value.item()
    return value


@dataclass
class MeshReport:
    """What the checks found. `problems` blocks the import, `warnings` do not."""

    problems: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    notes: dict[str, object] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return not self.problems

    def merge(self, other: MeshReport) -> None:
        self.problems += other.problems
        self.warnings += other.warnings
        self.notes.update(other.notes)


def content_hash(path: Path) -> str:
    """SHA-256 of a file, recorded in the wheel manifest as its provenance."""
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return f"sha256:{digest.hexdigest()}"


def load(path: Path) -> trimesh.Trimesh:
    """Load a mesh as a single Trimesh, concatenating a scene if the file holds one."""
    if not path.is_file():
        raise MeshError(f"no such file: {path}")

    if path.suffix.lower() == ".step" or path.suffix.lower() == ".stp":
        raise MeshError(
            f"{path.name} is STEP, which needs a CAD converter. Convert it to STL or OBJ "
            "first (FreeCAD, Fusion, or `gmsh -2`), then import that. Keep the STEP next "
            "to the package as source.step so the provenance survives."
        )

    loaded = trimesh.load(path, force="mesh", process=False)
    if isinstance(loaded, trimesh.Scene):
        loaded = trimesh.util.concatenate(tuple(loaded.geometry.values()))
    if not isinstance(loaded, trimesh.Trimesh):
        raise MeshError(f"{path.name} did not load as a triangle mesh")
    if len(loaded.faces) == 0:
        raise MeshError(f"{path.name} has no triangles")
    return loaded


def check_geometry(mesh: trimesh.Trimesh) -> MeshReport:
    """Degenerate triangles, disconnected components, normals and watertightness."""
    report = MeshReport()

    report.notes["triangles"] = len(mesh.faces)
    report.notes["vertices"] = len(mesh.vertices)

    areas = mesh.area_faces
    degenerate = int(np.count_nonzero(areas <= 1e-12))
    if degenerate:
        report.notes["degenerate_triangles"] = degenerate
        # Zero-area triangles make SCM's ray casting and Bullet's mesh collision both
        # behave unpredictably, so these are a hard problem rather than a warning.
        report.problems.append(
            f"{degenerate} zero-area triangles. Clean the mesh before importing "
            "(trimesh's process=True, or the CAD tool's own mesh repair)."
        )

    # Face adjacency rather than mesh.split(): splitting calls fill_holes, which needs
    # networkx, and all that is wanted here is the component count.
    components = trimesh.graph.connected_components(
        mesh.face_adjacency, nodes=np.arange(len(mesh.faces))
    )
    report.notes["components"] = len(components)
    if len(components) > 1:
        sizes = sorted((len(c) for c in components), reverse=True)
        report.warnings.append(
            f"mesh has {len(components)} disconnected components "
            f"(largest {sizes[0]} triangles, smallest {sizes[-1]}). "
            "A wheel is normally one shell - check for stray geometry."
        )

    report.notes["watertight"] = bool(mesh.is_watertight)
    if not mesh.is_watertight:
        # Not fatal: a watertight mesh is only needed to compute mass properties, and
        # measured values are preferred over computed ones anyway.
        report.warnings.append(
            "mesh is not watertight, so mass and inertia cannot be computed from it. "
            "Supply measured values instead."
        )
    elif mesh.volume <= 0:
        report.problems.append(
            "mesh volume is negative, which means the face normals point inward. "
            "Flip them in the CAD export."
        )

    if not mesh.is_winding_consistent:
        report.warnings.append("face winding is inconsistent, so normals will be unreliable")

    return report


def check_scale(mesh: trimesh.Trimesh, roll_axis: str) -> MeshReport:
    """Catch the "wheel ten times too large" class of mistake from the mesh bounds."""
    report = MeshReport()

    radius, width = measure(mesh, roll_axis)
    report.notes["radius_m"] = round(radius, 6)
    report.notes["width_m"] = round(width, 6)
    report.notes["bounds_m"] = [[round(float(v), 6) for v in b] for b in mesh.bounds]

    if radius < MIN_RADIUS_M or radius > MAX_RADIUS_M:
        report.problems.append(
            f"implied wheel radius is {radius:.4f} m, outside the plausible "
            f"{MIN_RADIUS_M}-{MAX_RADIUS_M} m range. Check --units: a CAD export in "
            "millimetres read as metres comes out 1000x too large."
        )
    if width > 4 * radius:
        report.problems.append(
            f"the wheel is {width:.3f} m wide but only {radius:.3f} m in radius, which "
            f"means --roll-axis {roll_axis} is probably not the spin axis"
        )
    return report


def measure(mesh: trimesh.Trimesh, roll_axis: str) -> tuple[float, float]:
    """Radius and width about a cardinal roll axis, from the mesh extents."""
    index = "xyz".index(roll_axis)
    extents = mesh.bounds[1] - mesh.bounds[0]

    width = float(extents[index])
    radial = [extents[i] for i in range(3) if i != index]
    radius = float(max(radial)) / 2.0
    return radius, width


def check_roll_axis(mesh: trimesh.Trimesh, roll_axis: str) -> MeshReport:
    """Verify the declared roll axis really is the axis of rotational symmetry.

    Compares the two radial extents: a wheel is round, so they should match. A mismatch
    means the mesh was authored about a different axis, which produces a model that looks
    fine and rolls like a cam.
    """
    report = MeshReport()
    index = "xyz".index(roll_axis)
    extents = mesh.bounds[1] - mesh.bounds[0]
    radial = [extents[i] for i in range(3) if i != index]

    larger, smaller = max(radial), min(radial)
    ratio = smaller / larger if larger > 0 else 0.0
    report.notes["radial_symmetry"] = round(ratio, 4)

    if ratio < 0.9:
        other = [a for i, a in enumerate("xyz") if i != index]
        report.problems.append(
            f"the two extents perpendicular to {roll_axis} differ by "
            f"{100 * (1 - ratio):.0f}% ({other[0]}={radial[0]:.4f} m, "
            f"{other[1]}={radial[1]:.4f} m). A wheel is round about its roll axis, so "
            f"--roll-axis {roll_axis} looks wrong."
        )
    elif ratio < 0.98:
        report.warnings.append(
            f"radial extents differ by {100 * (1 - ratio):.1f}%. Expected for a paddle or "
            "grouser pattern, but worth confirming."
        )
    return report


def normalise(
    mesh: trimesh.Trimesh,
    units: str,
    roll_axis: str,
    hub_origin: list[float] | None = None,
) -> tuple[trimesh.Trimesh, dict[str, object]]:
    """Convert to metres, rotate the roll axis onto +Y, and centre on the hub frame.

    The rover convention is Z up with wheel spin axes along the body's Y, so every
    package is stored that way and the rover's wheel stations only ever rotate it onto
    their own axis.

    The hub origin must be given explicitly or be the mesh's own bounding-box centre; an
    inferred visual centre is never used silently, because a wheel with an asymmetric
    tread has a visual centre that is not its hub.
    """
    if units not in UNITS:
        raise MeshError(f"unknown units '{units}'. Use one of: {', '.join(UNITS)}")
    if roll_axis not in AXES:
        raise MeshError(f"unknown roll axis '{roll_axis}'. Use x, y or z")

    mesh = mesh.copy()
    info: dict[str, object] = {"units": units, "source_roll_axis": roll_axis}

    scale = UNITS[units]
    if scale != 1.0:
        mesh.apply_scale(scale)
    info["scale_to_m"] = scale

    if hub_origin is not None:
        if len(hub_origin) != 3:
            raise MeshError("--hub-origin takes three numbers")
        offset = np.array(hub_origin, dtype=float) * scale
        info["hub_origin_m"] = [round(v, 6) for v in offset]
    else:
        offset = (mesh.bounds[0] + mesh.bounds[1]) / 2.0
        info["hub_origin_m"] = [round(float(v), 6) for v in offset]
        info["hub_origin_source"] = "bounding box centre"
    mesh.apply_translation(-offset)

    if roll_axis != "y":
        rotation = trimesh.geometry.align_vectors(AXES[roll_axis], AXES["y"])
        mesh.apply_transform(rotation)
    info["roll_axis"] = "y"

    return mesh, info


def mass_properties(mesh: trimesh.Trimesh, density: float) -> dict[str, object] | None:
    """Mass, centre of mass and inertia tensor from a closed mesh and a density.

    Returns None for an open mesh, where the volume integral is meaningless. Measured
    values always override these; they exist so a new wheel is usable before it has
    been weighed.
    """
    if not mesh.is_watertight or mesh.volume <= 0:
        return None

    mesh = mesh.copy()
    mesh.density = density

    inertia = mesh.moment_inertia
    return {
        "mass": round(float(mesh.mass), 6),
        "com": [round(float(v), 6) for v in mesh.center_mass],
        "inertia_xx": [
            round(float(inertia[0][0]), 9),
            round(float(inertia[1][1]), 9),
            round(float(inertia[2][2]), 9),
        ],
        "inertia_xy": [
            round(float(inertia[0][1]), 9),
            round(float(inertia[0][2]), 9),
            round(float(inertia[1][2]), 9),
        ],
        "provenance": "cad",
        "density_kg_m3": density,
        "volume_m3": round(float(mesh.volume), 9),
    }


def make_collision_mesh(
    mesh: trimesh.Trimesh, target_faces: int, keep_features: bool = True
) -> tuple[trimesh.Trimesh, MeshReport]:
    """Decimate the visual mesh into a contact mesh that keeps the tread.

    Grousers and paddles are what an SCM study is measuring, so the reduction is checked
    against the original's radial profile rather than only its triangle count. A
    simplification that rounds the tread off is reported, not accepted quietly.
    """
    report = MeshReport()
    report.notes["visual_triangles"] = len(mesh.faces)

    if len(mesh.faces) <= target_faces:
        report.notes["collision_triangles"] = len(mesh.faces)
        report.notes["decimated"] = False
        return mesh.copy(), report

    reduced = None
    try:
        reduced = mesh.simplify_quadric_decimation(face_count=target_faces)
    except Exception as exc:  # noqa: BLE001 - any backend failure falls back below
        report.warnings.append(f"quadric decimation unavailable ({exc}); using a convex hull")

    if reduced is None or len(reduced.faces) == 0:
        # A convex hull loses the tread entirely, so this is only a last resort and is
        # loudly reported: a hulled paddle wheel is a smooth wheel.
        reduced = mesh.convex_hull
        report.warnings.append(
            "fell back to a convex hull, which removes every grouser and paddle. "
            "This wheel's SCM results will not reflect its tread."
        )

    report.notes["collision_triangles"] = len(reduced.faces)
    report.notes["decimated"] = True

    if keep_features:
        report.merge(_check_feature_retention(mesh, reduced))
    return reduced, report


def _check_feature_retention(original: trimesh.Trimesh, reduced: trimesh.Trimesh) -> MeshReport:
    """Compare radial profiles to see whether the tread survived decimation."""
    report = MeshReport()

    def radial_spread(mesh: trimesh.Trimesh) -> float:
        # Distance of every vertex from the roll axis (+Y after normalisation). The
        # difference between the outer tread and the troughs between grousers is exactly
        # the peak-to-trough spread of this distribution.
        vertices = np.asarray(mesh.vertices)
        radii = np.sqrt(vertices[:, 0] ** 2 + vertices[:, 2] ** 2)
        if radii.size == 0:
            return 0.0
        return float(np.percentile(radii, 98) - np.percentile(radii, 50))

    before = radial_spread(original)
    after = radial_spread(reduced)
    report.notes["tread_depth_visual_m"] = round(before, 5)
    report.notes["tread_depth_collision_m"] = round(after, 5)

    if before > 1e-4:
        retained = after / before
        report.notes["tread_retained"] = round(retained, 3)
        if retained < 0.7:
            report.problems.append(
                f"the collision mesh keeps only {100 * retained:.0f}% of the visual mesh's "
                f"tread depth ({after * 1000:.1f} mm of {before * 1000:.1f} mm). Raise "
                "--collision-faces: the grousers are what an SCM study measures."
            )
        elif retained < 0.9:
            report.warnings.append(
                f"the collision mesh keeps {100 * retained:.0f}% of the tread depth; "
                "consider a higher --collision-faces"
            )
    return report


def write_obj(mesh: trimesh.Trimesh, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(trimesh.exchange.obj.export_obj(mesh, include_normals=True))


def render_preview(mesh: trimesh.Trimesh, path: Path, size: int = 640) -> bool:
    """Render an orthographic three-view preview of the wheel.

    Matplotlib rather than an offscreen GL context: the container has no display, and a
    wireframe three-view is actually better than a shaded render for spotting a wrong
    scale or a missing grouser.
    """
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    vertices = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.faces)

    # Sample the faces when there are many: a 200k-triangle wireframe takes minutes to
    # draw and looks like a solid block anyway.
    if len(faces) > 4000:
        step = len(faces) // 4000 + 1
        faces = faces[::step]

    views = [("side (x-z)", 0, 2), ("front (y-z)", 1, 2), ("top (x-y)", 0, 1)]
    fig, axes = plt.subplots(1, 3, figsize=(3 * size / 100, size / 100), dpi=100)

    for ax, (title, i, j) in zip(axes, views):
        segments = []
        for face in faces:
            loop = list(face) + [face[0]]
            for a, b in itertools.pairwise(loop):
                segments.append(
                    [(vertices[a][i], vertices[a][j]), (vertices[b][i], vertices[b][j])]
                )

        from matplotlib.collections import LineCollection

        ax.add_collection(LineCollection(segments, linewidths=0.2, colors="#2b6cb0", alpha=0.6))
        ax.set_title(title, fontsize=9)
        ax.set_aspect("equal")
        ax.autoscale_view()
        ax.set_xlabel("m", fontsize=7)
        ax.tick_params(labelsize=7)
        ax.grid(True, linewidth=0.3, alpha=0.3)

    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    return True


def cylinder_inertia(mass: float, radius: float, width: float) -> dict[str, list[float]]:
    """Inertia of a solid cylinder about its centre, with the roll axis along Y.

    Used when a wheel's mass is measured but its mesh is not closed, so the tensor has
    to be estimated. Recorded with "estimated" provenance so a scenario that demands
    measured values will refuse it.
    """
    axial = 0.5 * mass * radius**2
    transverse = mass * (3 * radius**2 + width**2) / 12.0
    return {
        "inertia_xx": [round(transverse, 9), round(axial, 9), round(transverse, 9)],
        "inertia_xy": [0.0, 0.0, 0.0],
    }


def drop_spin_smoke_test(radius: float, width: float, mass: float) -> MeshReport:
    """Sanity-check a package's bulk numbers before it is registered.

    Not a Chrono run: the real smoke test is `sim chrono run` on the drop scenario, which
    the importer prints as the next step. This catches the combinations that would make
    that run meaningless - a wheel whose density is absurd, or whose aspect ratio means
    the roll axis is wrong.
    """
    report = MeshReport()

    volume = math.pi * radius**2 * width
    if volume <= 0:
        report.problems.append("radius and width must both be positive")
        return report

    density = mass / volume
    report.notes["bulk_density_kg_m3"] = round(density, 1)

    # A wheel is mostly air. Below balsa or above aluminium means a units mistake in the
    # mass, the mesh, or both.
    if density < 20:
        report.problems.append(
            f"bulk density is {density:.1f} kg/m3, lighter than balsa. Check the mass "
            "and the mesh units."
        )
    elif density > 2700:
        report.problems.append(
            f"bulk density is {density:.0f} kg/m3, denser than solid aluminium. Check "
            "the mass and the mesh units."
        )
    elif density > 900:
        report.warnings.append(
            f"bulk density is {density:.0f} kg/m3, which is high for a hollow wheel"
        )

    return report
