"""Deterministic Xacro-to-URDF preprocessing and URDF reading.

The chassis URDF is an onshape-to-robot export carrying Xacro properties, an argument,
and `package://` mesh paths, so it is not a plain URDF. Rendering it needs to be
repeatable without a ROS environment, because the Chrono pipeline runs in a container
that has no reason to source ROS and in CI that has no ROS at all.

This covers the subset the export actually uses: xacro:property, xacro:arg,
xacro:include, and ${...} substitution of properties, arguments and arithmetic. Anything
beyond that (macros, conditionals, Python expressions) raises rather than guessing, so a
URDF this cannot render fails loudly instead of rendering wrongly.
"""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

XACRO_NS = "http://www.ros.org/wiki/xacro"
SUBSTITUTION = re.compile(r"\$\{([^}]*)\}|\$\(([^)]*)\)")

# Only these names are reachable from a ${...} expression. A property file that needs
# anything else is a sign the model should be simplified, not that this should grow.
SAFE_NAMES = {
    "pi": math.pi,
    "radians": math.radians,
    "degrees": math.degrees,
    "sin": math.sin,
    "cos": math.cos,
    "tan": math.tan,
    "sqrt": math.sqrt,
    "abs": abs,
    "min": min,
    "max": max,
    "round": round,
    "True": True,
    "False": False,
}


class UrdfError(Exception):
    pass


@dataclass
class Inertial:
    mass: float = 0.0
    com: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    # Ixx, Iyy, Izz, Ixy, Ixz, Iyz about the centre of mass
    inertia: list[float] = field(default_factory=lambda: [0.0] * 6)


@dataclass
class Geometry:
    kind: str  # "mesh", "box", "sphere", "cylinder"
    position: list[float]
    rpy: list[float]
    mesh: Path | None = None
    scale: float = 1.0
    dimensions: list[float] = field(default_factory=list)
    radius: float = 0.0
    length: float = 0.0
    color: list[float] = field(default_factory=lambda: [0.6, 0.6, 0.6])


@dataclass
class Link:
    name: str
    inertial: Inertial = field(default_factory=Inertial)
    visual: list[Geometry] = field(default_factory=list)
    collision: list[Geometry] = field(default_factory=list)


@dataclass
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    position: list[float]
    rpy: list[float]
    axis: list[float]
    limits: tuple[float, float] | None = None
    effort: float = 0.0
    velocity: float = 0.0


@dataclass
class Model:
    name: str
    links: dict[str, Link]
    joints: dict[str, Joint]
    source: Path
    root: str

    def children_of(self, link: str) -> list[Joint]:
        return [j for j in self.joints.values() if j.parent == link]

    def joint_to(self, link: str) -> Joint | None:
        for joint in self.joints.values():
            if joint.child == link:
                return joint
        return None


# ---------------------------------------------------------------------------- #
#                                 xacro rendering                              #
# ---------------------------------------------------------------------------- #


def render_xacro(
    path: Path, args: dict[str, str] | None = None, search_paths: list[Path] | None = None
) -> ET.Element:
    """Render a Xacro file to a plain URDF element tree.

    The result is deterministic: no timestamps, no environment lookups, and properties
    resolved in document order. That is what lets the generated URDF be hashed and the
    hash recorded in the rover config.
    """
    args = dict(args or {})
    search_paths = search_paths or [path.parent]

    tree = ET.parse(path)
    root = tree.getroot()

    _expand_includes(root, path.parent, search_paths, depth=0)

    properties: dict[str, object] = {}
    _collect_properties(root, properties, args)
    _strip_xacro_elements(root)
    _substitute_attributes(root, properties, args, path)

    return root


def _expand_includes(element: ET.Element, base: Path, search_paths: list[Path], depth: int) -> None:
    if depth > 8:
        raise UrdfError("xacro:include nesting is deeper than 8 levels")

    for parent in list(element.iter()):
        for child in list(parent):
            if child.tag != f"{{{XACRO_NS}}}include":
                continue

            filename = child.get("filename", "")
            resolved = _resolve_include(filename, base, search_paths)
            included = ET.parse(resolved).getroot()
            _expand_includes(included, resolved.parent, search_paths, depth + 1)

            index = list(parent).index(child)
            parent.remove(child)
            for offset, node in enumerate(list(included)):
                parent.insert(index + offset, node)


FIND_PACKAGE = re.compile(r"\$\(find\s+([^)]+)\)")


def resolve_package_path(reference: str, search_paths: list[Path]) -> Path | None:
    """Resolve `package://pkg/rest` or `$(find pkg)/rest` against the search paths.

    Both forms appear in the same export: `package://` on mesh filenames and
    `$(find ...)` on the include. Treating them identically is what ROS does, and it
    means the preprocessor does not need a package index.
    """
    if reference.startswith("package://"):
        relative = reference[len("package://") :]
    else:
        match = FIND_PACKAGE.match(reference)
        if not match:
            return None
        relative = match.group(1).strip() + reference[match.end() :]

    package, _, rest = relative.lstrip("/").partition("/")
    for root in search_paths:
        candidate = root / package / rest
        if candidate.is_file():
            return candidate.resolve()
    return None


def _resolve_include(filename: str, base: Path, search_paths: list[Path]) -> Path:
    resolved = resolve_package_path(filename, search_paths)
    if resolved:
        return resolved
    if filename.startswith(("package://", "$(find")):
        raise UrdfError(
            f"cannot resolve {filename}. Searched: {', '.join(str(p) for p in search_paths)}"
        )

    candidate = base / filename
    if not candidate.is_file():
        raise UrdfError(f"xacro:include target not found: {filename}")
    return candidate


def _collect_properties(
    root: ET.Element, properties: dict[str, object], args: dict[str, str]
) -> None:
    for element in root.iter():
        if element.tag == f"{{{XACRO_NS}}}property":
            name = element.get("name")
            if not name:
                raise UrdfError("xacro:property without a name")
            raw = element.get("value", "")
            properties[name] = _evaluate(raw, properties, args, "property " + name)
        elif element.tag == f"{{{XACRO_NS}}}arg":
            name = element.get("name")
            if name and name not in args:
                args[name] = element.get("default", "")


def _strip_xacro_elements(root: ET.Element) -> None:
    for parent in list(root.iter()):
        for child in list(parent):
            if child.tag.startswith(f"{{{XACRO_NS}}}"):
                tag = child.tag.split("}", 1)[1]
                if tag in {"property", "arg"}:
                    parent.remove(child)
                else:
                    raise UrdfError(
                        f"xacro:{tag} is not supported by this preprocessor. Simplify the "
                        "URDF, or render it with the real xacro tool and commit the result."
                    )


def _substitute_attributes(
    root: ET.Element, properties: dict[str, object], args: dict[str, str], source: Path
) -> None:
    for element in root.iter():
        for key, value in list(element.attrib.items()):
            if "$" in value:
                element.set(key, _substitute(value, properties, args, f"{source}:{key}"))


def _substitute(text: str, properties: dict, args: dict, where: str) -> str:
    def replace(match: re.Match) -> str:
        expression, argument = match.group(1), match.group(2)
        if argument is not None:
            name = argument.split()[-1] if argument.startswith("arg ") else argument
            if name not in args:
                raise UrdfError(f"{where}: $({argument}) is not defined")
            return str(args[name])
        return str(_evaluate(expression, properties, args, where))

    return SUBSTITUTION.sub(replace, text)


def _evaluate(raw: str, properties: dict, args: dict, where: str) -> object:
    """Evaluate a Xacro expression, or return the string unchanged if it is not one."""
    if "$" in raw:
        raw = _substitute(raw, properties, args, where)

    # A bare value (a mesh path, a material name) is used as-is.
    if not re.fullmatch(r"[\s\d.+\-*/()eE,]*|[\w\s.+\-*/()]*", raw) or not raw.strip():
        return raw
    try:
        return eval(raw, {"__builtins__": {}}, {**SAFE_NAMES, **properties})
    except Exception:  # noqa: BLE001 - a non-expression is a literal, which is fine
        return raw


# ---------------------------------------------------------------------------- #
#                                  URDF reading                                #
# ---------------------------------------------------------------------------- #


def parse(
    path: Path, args: dict[str, str] | None = None, search_paths: list[Path] | None = None
) -> Model:
    """Render if needed, then read the URDF into a Model."""
    root = render_xacro(path, args, search_paths)

    links: dict[str, Link] = {}
    for element in root.findall("link"):
        link = _read_link(element, path.parent, search_paths or [path.parent])
        links[link.name] = link

    joints: dict[str, Joint] = {}
    for element in root.findall("joint"):
        joint = _read_joint(element)
        if joint.parent not in links or joint.child not in links:
            raise UrdfError(
                f"joint '{joint.name}' references a link that does not exist "
                f"({joint.parent} -> {joint.child})"
            )
        joints[joint.name] = joint

    children = {j.child for j in joints.values()}
    roots = [name for name in links if name not in children]
    if len(roots) != 1:
        raise UrdfError(
            f"expected exactly one root link, found {len(roots)}: {', '.join(sorted(roots))}. "
            "A URDF with several roots is not one connected robot."
        )

    return Model(root.get("name", path.stem), links, joints, path, roots[0])


def _read_link(element: ET.Element, base: Path, search_paths: list[Path]) -> Link:
    link = Link(element.get("name", ""))

    inertial = element.find("inertial")
    if inertial is not None:
        mass = inertial.find("mass")
        link.inertial.mass = float(mass.get("value", 0)) if mass is not None else 0.0

        origin = inertial.find("origin")
        if origin is not None:
            link.inertial.com = _floats(origin.get("xyz", "0 0 0"))

        tensor = inertial.find("inertia")
        if tensor is not None:
            link.inertial.inertia = [
                float(tensor.get("ixx", 0)),
                float(tensor.get("iyy", 0)),
                float(tensor.get("izz", 0)),
                float(tensor.get("ixy", 0)),
                float(tensor.get("ixz", 0)),
                float(tensor.get("iyz", 0)),
            ]

    for tag, target in (("visual", link.visual), ("collision", link.collision)):
        for node in element.findall(tag):
            geometry = _read_geometry(node, base, search_paths)
            if geometry:
                target.append(geometry)
    return link


def _read_geometry(node: ET.Element, base: Path, search_paths: list[Path]) -> Geometry | None:
    origin = node.find("origin")
    position = _floats(origin.get("xyz", "0 0 0")) if origin is not None else [0.0, 0.0, 0.0]
    rpy = _floats(origin.get("rpy", "0 0 0")) if origin is not None else [0.0, 0.0, 0.0]

    color = [0.6, 0.6, 0.6]
    material = node.find("material/color")
    if material is not None:
        rgba = _floats(material.get("rgba", "0.6 0.6 0.6 1"))
        color = rgba[:3]

    mesh = node.find("geometry/mesh")
    if mesh is not None:
        filename = mesh.get("filename", "")
        scale = 1.0
        if mesh.get("scale"):
            scales = _floats(mesh.get("scale"))
            if len({round(s, 9) for s in scales}) != 1:
                raise UrdfError(
                    f"mesh {filename} has a non-uniform scale {scales}. Chrono applies one "
                    "scalar, so rescale the mesh instead."
                )
            scale = scales[0]
        return Geometry(
            "mesh", position, rpy, _resolve_mesh(filename, base, search_paths), scale, color=color
        )

    box = node.find("geometry/box")
    if box is not None:
        return Geometry(
            "box", position, rpy, dimensions=_floats(box.get("size", "0 0 0")), color=color
        )

    sphere = node.find("geometry/sphere")
    if sphere is not None:
        return Geometry("sphere", position, rpy, radius=float(sphere.get("radius", 0)), color=color)

    cylinder = node.find("geometry/cylinder")
    if cylinder is not None:
        return Geometry(
            "cylinder",
            position,
            rpy,
            radius=float(cylinder.get("radius", 0)),
            length=float(cylinder.get("length", 0)),
            color=color,
        )

    return None


def _resolve_mesh(filename: str, base: Path, search_paths: list[Path]) -> Path:
    resolved = resolve_package_path(filename, search_paths)
    if resolved:
        return resolved
    if filename.startswith(("package://", "$(find")):
        raise UrdfError(
            f"cannot resolve {filename}. Searched: {', '.join(str(p) for p in search_paths)}"
        )

    filename = filename.removeprefix("file://")
    candidate = Path(filename)
    if not candidate.is_absolute():
        candidate = base / candidate
    if not candidate.is_file():
        raise UrdfError(f"mesh not found: {filename}")
    return candidate.resolve()


def _read_joint(element: ET.Element) -> Joint:
    origin = element.find("origin")
    axis = element.find("axis")
    limit = element.find("limit")

    limits = None
    if limit is not None and limit.get("lower") is not None and limit.get("upper") is not None:
        limits = (float(limit.get("lower")), float(limit.get("upper")))

    return Joint(
        name=element.get("name", ""),
        kind=element.get("type", "fixed"),
        parent=element.find("parent").get("link"),
        child=element.find("child").get("link"),
        position=_floats(origin.get("xyz", "0 0 0")) if origin is not None else [0.0] * 3,
        rpy=_floats(origin.get("rpy", "0 0 0")) if origin is not None else [0.0] * 3,
        axis=_floats(axis.get("xyz", "0 0 1")) if axis is not None else [0.0, 0.0, 1.0],
        limits=limits,
        effort=float(limit.get("effort", 0)) if limit is not None else 0.0,
        velocity=float(limit.get("velocity", 0)) if limit is not None else 0.0,
    )


def _floats(text: str) -> list[float]:
    return [float(part) for part in text.replace(",", " ").split()]


def write_urdf(root: ET.Element, path: Path) -> Path:
    """Write a rendered URDF, for inspection and hashing.

    This file is generated, never edited: the source of truth stays the Xacro in the
    gazebo package, and anything hand-edited here would be silently lost on the next
    import.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="    ")
    header = (
        "<?xml version='1.0'?>\n"
        "<!-- GENERATED by `sim chrono rover import`. Do not edit: edit the Xacro source\n"
        "     in gazebo/<robot>_description/urdf/ and re-import. -->\n"
    )
    path.write_text(header + ET.tostring(root, encoding="unicode"))
    return path
