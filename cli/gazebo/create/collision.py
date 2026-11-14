"""Generate primitive collision geometry for articulated CAD exports."""

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh


def _parse(urdf_text: str) -> ET.Element:
    cleaned = re.sub(r"<xacro:[^>]*/?>", "", urdf_text)
    cleaned = re.sub(r"</xacro:[^>]*>", "", cleaned)
    cleaned = re.sub(r'xmlns:xacro="[^"]*"', "", cleaned)
    return ET.fromstring(cleaned)


def _pose_matrix(element: ET.Element | None) -> np.ndarray:
    if element is None:
        return np.eye(4)

    xyz = [float(value) for value in element.get("xyz", "0 0 0").split()]
    rpy = [float(value) for value in element.get("rpy", "0 0 0").split()]
    return trimesh.transformations.translation_matrix(xyz) @ trimesh.transformations.euler_matrix(
        *rpy, axes="sxyz"
    )


def _rigid_groups(root: ET.Element) -> list[tuple[str, list[tuple[ET.Element, np.ndarray]]]]:
    links = {link.get("name"): link for link in root.findall("link") if link.get("name")}
    children: dict[str, list[tuple[str, ET.Element]]] = {}
    child_links = set()

    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        parent_name = parent.get("link")
        child_name = child.get("link")
        if not parent_name or not child_name or child_name not in links:
            continue
        children.setdefault(parent_name, []).append((child_name, joint))
        child_links.add(child_name)

    roots = [name for name in links if name not in child_links and name != "world"]
    if not roots and "world" in links:
        roots = [
            child_name for child_name, _joint in children.get("world", []) if child_name in links
        ]

    groups: list[tuple[str, list[tuple[ET.Element, np.ndarray]]]] = []

    def collect_fixed(name: str, transform: np.ndarray) -> list[tuple[ET.Element, np.ndarray]]:
        members = [(links[name], transform)]
        for child_name, joint in children.get(name, []):
            if joint.get("type") == "fixed":
                members.extend(
                    collect_fixed(child_name, transform @ _pose_matrix(joint.find("origin")))
                )
        return members

    def visit(name: str) -> None:
        groups.append((name, collect_fixed(name, np.eye(4))))
        stack = [name]
        while stack:
            current = stack.pop()
            for child_name, joint in children.get(current, []):
                if joint.get("type") == "fixed":
                    stack.append(child_name)
                else:
                    visit(child_name)

    for name in roots:
        visit(name)

    return groups


def _mesh_file(meshes_dir: Path, filename: str) -> Path:
    path = meshes_dir / Path(filename).name
    if not path.is_file():
        raise FileNotFoundError(f"Collision source mesh not found: {path}")
    return path


def _visual_bounds(
    members: list[tuple[ET.Element, np.ndarray]], meshes_dir: Path
) -> tuple[np.ndarray, np.ndarray] | None:
    vertices = []
    for link, link_transform in members:
        for visual in link.findall("visual"):
            mesh_element = visual.find("./geometry/mesh")
            if mesh_element is None or not mesh_element.get("filename"):
                continue

            mesh = trimesh.load_mesh(
                _mesh_file(meshes_dir, mesh_element.get("filename", "")),
                force="mesh",
                process=False,
            )
            scale = np.array([float(value) for value in mesh_element.get("scale", "1 1 1").split()])
            points = np.asarray(mesh.vertices, dtype=float) * scale
            transform = link_transform @ _pose_matrix(visual.find("origin"))
            homogeneous = np.column_stack((points, np.ones(len(points))))
            vertices.append((transform @ homogeneous.T).T[:, :3])

    if not vertices:
        return None

    points = np.vstack(vertices)
    return points.min(axis=0), points.max(axis=0)


def _remove_generated_collisions(urdf_text: str, robot_name: str) -> str:
    collision = re.compile(r"\n[ \t]*<collision\b.*?</collision>", re.DOTALL)
    proxy_name = f"{robot_name}_collision_proxy"
    return collision.sub(
        lambda match: (
            ""
            if "<mesh" in match.group(0) or f'name="{proxy_name}"' in match.group(0)
            else match.group(0)
        ),
        urdf_text,
    )


def _format_vector(values: np.ndarray) -> str:
    return " ".join(f"{value:.9g}" for value in values)


def _collision_xml(
    child_indent: str,
    indent_unit: str,
    robot_name: str,
    center: np.ndarray,
    size: np.ndarray,
) -> str:
    geometry_indent = child_indent + indent_unit
    shape_indent = geometry_indent + indent_unit
    return "\n".join(
        [
            f'{child_indent}<collision name="{robot_name}_collision_proxy">',
            f'{geometry_indent}<origin xyz="{_format_vector(center)}" rpy="0 0 0" />',
            f"{geometry_indent}<geometry>",
            f'{shape_indent}<box size="{_format_vector(size)}" />',
            f"{geometry_indent}</geometry>",
            f"{child_indent}</collision>",
        ]
    )


def _insert_collision(
    urdf_text: str,
    link_name: str,
    robot_name: str,
    center: np.ndarray,
    size: np.ndarray,
) -> str:
    escaped_name = re.escape(link_name)
    self_closing = re.compile(
        rf'^(?P<indent>[ \t]*)(?P<open><link\s+name="{escaped_name}"[^>]*?)\s*/>',
        re.MULTILINE,
    )
    indent_units = []
    for style in re.finditer(
        r"^([ \t]*)<link\b[^>]*(?<!/)>\s*\n([ \t]+)<",
        urdf_text,
        flags=re.MULTILINE,
    ):
        link_indent, child_indent = style.groups()
        if child_indent.startswith(link_indent) and child_indent != link_indent:
            indent_units.append(child_indent[len(link_indent) :])
    indent_unit = max(set(indent_units), key=indent_units.count) if indent_units else "  "

    def replace_self_closing(match: re.Match) -> str:
        link_indent = match.group("indent")
        child_indent = link_indent + indent_unit
        collision = _collision_xml(child_indent, indent_unit, robot_name, center, size)
        return f"{link_indent}{match.group('open')}>\n{collision}\n{link_indent}</link>"

    if self_closing.search(urdf_text):
        return self_closing.sub(replace_self_closing, urdf_text, count=1)

    link = re.compile(
        rf'^(?P<indent>[ \t]*)(?P<open><link\s+name="{escaped_name}"[^>]*>)'
        r"(?P<body>.*?)^[ \t]*</link>",
        re.DOTALL | re.MULTILINE,
    )
    if not link.search(urdf_text):
        raise ValueError(f"Could not insert collision into link: {link_name}")

    def replace_link(match: re.Match) -> str:
        link_indent = match.group("indent")
        child_match = re.search(r"\n([ \t]+)<", match.group("body"))
        child_indent = link_indent + indent_unit if child_match is None else child_match.group(1)
        local_indent_unit = child_indent[len(link_indent) :] or indent_unit
        collision = _collision_xml(
            child_indent,
            local_indent_unit,
            robot_name,
            center,
            size,
        )
        body = match.group("body").rstrip()
        return f"{link_indent}{match.group('open')}{body}\n{collision}\n{link_indent}</link>"

    return link.sub(replace_link, urdf_text, count=1)


def add_collision_boxes(
    urdf_text: str,
    meshes_dir: str | Path,
    robot_name: str,
) -> str:
    """Generate one axis-aligned box collision proxy per articulated rigid body."""
    meshes_path = Path(meshes_dir)
    source_root = _parse(urdf_text)
    proxy_name = f"{robot_name}_collision_proxy"
    primitive_groups = {
        link_name
        for link_name, members in _rigid_groups(source_root)
        if any(
            collision.get("name") != proxy_name and collision.find("./geometry/mesh") is None
            for link, _transform in members
            for collision in link.findall("collision")
        )
    }
    urdf_text = _remove_generated_collisions(urdf_text, robot_name)
    root = _parse(urdf_text)

    for link_name, members in _rigid_groups(root):
        if link_name in primitive_groups:
            continue
        bounds = _visual_bounds(members, meshes_path)
        if bounds is None:
            continue
        lower, upper = bounds
        urdf_text = _insert_collision(
            urdf_text,
            link_name,
            robot_name,
            (lower + upper) / 2,
            upper - lower,
        )

    for existing in meshes_path.glob(f"{robot_name}_collision_*.stl"):
        existing.unlink()

    return urdf_text


def enable_model_collisions(urdf_text: str) -> str:
    """Enable contacts between generated proxies and the assembled model."""
    root = _parse(urdf_text)
    if any(
        gazebo.get("reference") is None and gazebo.findtext("self_collide", "").lower() == "true"
        for gazebo in root.findall("gazebo")
    ):
        return urdf_text

    tag = "\n  <gazebo><self_collide>true</self_collide></gazebo>"
    return urdf_text.replace("</robot>", tag + "\n</robot>")
