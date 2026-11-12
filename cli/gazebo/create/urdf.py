"""URDF post-processing: xacro injection, mesh path rewriting, control generation"""

import re
import xml.etree.ElementTree as ET
from pathlib import Path


def _ensure_xacro_ns(urdf_text: str) -> str:
    if "xmlns:xacro" in urdf_text:
        return urdf_text
    return urdf_text.replace("<robot ", '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" ', 1)


def _inject_xacro_properties(urdf_text: str, description_package: str) -> str:
    props = (
        "\n"
        "  <!-- XACRO -->\n"
        f'  <xacro:property name="mesh_path" value="package://{description_package}/meshes"/>\n'
        '  <xacro:arg name="controller_config" default=""/>\n'
    )
    pattern = re.compile(r"(<robot\b[^>]*>)")
    return pattern.sub(r"\1" + props, urdf_text, count=1)


def _rewrite_mesh_paths(urdf_text: str) -> str:
    return re.sub(
        r'filename="package://(?:assets|[^/]+/meshes)/([^"]+)"',
        r'filename="${mesh_path}/\1"',
        urdf_text,
    )


def _strip_xacro_for_parsing(urdf_text: str) -> str:
    cleaned = re.sub(r"<xacro:[^>]*/?>", "", urdf_text)
    cleaned = re.sub(r"</xacro:[^>]*>", "", cleaned)
    cleaned = re.sub(r'xmlns:xacro="[^"]*"', "", cleaned)
    cleaned = re.sub(r"\$\{[^}]*\}", "", cleaned)
    return cleaned


def _extract_links(urdf_text: str) -> list:
    root = ET.fromstring(_strip_xacro_for_parsing(urdf_text))
    return [link.get("name") for link in root.findall("link") if link.get("name")]


def _prefix_link_names(urdf_text: str, robot_name: str) -> str:
    """Prefix every link name with the robot name, except "world".

    CAD exports name links after generic part names (e.g. "part_1"), which collide
    once two robot descriptions are combined into one (for example Viator including
    both its chassis and arm_description). Prefixing here, at
    generation time, means that can't happen again no matter what gets merged later -
    a hand edit to the generated .urdf instead would just get overwritten by the next
    `sim update`.

    "world" is excluded: it's not a CAD part name, it's the sentinel _inject_world_base_link
    adds, and Gazebo's URDF->SDF conversion specifically recognizes a fixed joint's parent
    named exactly "world" as an anchor to the simulation's static global frame rather than an
    ordinary link - renaming it turns the robot into an ordinary, unanchored floating body.
    """
    for name in _extract_links(urdf_text):
        if name == "world":
            continue
        escaped = re.escape(name)
        prefixed = f"{robot_name}_{name}"
        urdf_text = re.sub(rf'(<link\s+name)="{escaped}"', rf'\1="{prefixed}"', urdf_text)
        urdf_text = re.sub(rf'(<parent\s+link)="{escaped}"', rf'\1="{prefixed}"', urdf_text)
        urdf_text = re.sub(rf'(<child\s+link)="{escaped}"', rf'\1="{prefixed}"', urdf_text)
    return urdf_text


def _extract_revolute_joints(urdf_text: str) -> list:
    root = ET.fromstring(_strip_xacro_for_parsing(urdf_text))
    joints = []
    for j in root.findall("joint"):
        if j.get("type") == "revolute" and j.get("name"):
            limit = j.find("limit")
            lower = float(limit.get("lower", "0")) if limit is not None else 0.0
            upper = float(limit.get("upper", "0")) if limit is not None else 0.0
            joints.append((j.get("name"), lower, upper))
    return joints


def _generate_control_xacro(robot_name: str, joints: list) -> str:
    lines = [
        '<?xml version="1.0" ?>',
        f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{robot_name}_control">',
        "",
        '    <xacro:arg name="controller_config" default=""/>',
        '    <xacro:arg name="plugin_extension" default=".so"/>',
        '    <xacro:property name="plugin_extension" value="$(arg plugin_extension)"/>',
        "",
        "    <!-- ros2_control hardware interface -->",
        '    <ros2_control name="GazeboSimSystem" type="system">',
        "        <hardware>",
        "            <plugin>gz_ros2_control/GazeboSimSystem</plugin>",
        "        </hardware>",
    ]

    for name, lower, upper in joints:
        initial_value = (lower + upper) / 2 if lower < upper else 0.0
        lines += [
            "",
            f'        <joint name="{name}">',
            '            <command_interface name="position">',
            f'                <param name="min">{lower:.2f}</param>',
            f'                <param name="max">{upper:.2f}</param>',
            "            </command_interface>",
            "",
            '            <state_interface name="position">',
            f'                <param name="initial_value">{initial_value:.2f}</param>',
            "            </state_interface>",
            '            <state_interface name="velocity"/>',
            '            <state_interface name="effort"/>',
            "        </joint>",
        ]

    lines += [
        "    </ros2_control>",
        "",
        "    <gazebo>",
        "        <!-- JOINT CONTROLLER -->",
        '        <plugin filename="libgz_ros2_control-system${plugin_extension}" name="gz_ros2_control::GazeboSimROS2ControlPlugin">',
        "            <robot_param>robot_description</robot_param>",
        "            <robot_param_node>robot_state_publisher</robot_param_node>",
        "            <parameters>$(arg controller_config)</parameters>",
        "            <update_rate>60</update_rate>",
        "        </plugin>",
        "    </gazebo>",
        "</robot>",
        "",
    ]

    return "\n".join(lines)


def _inject_world_base_link(urdf_text: str) -> str:
    root = ET.fromstring(_strip_xacro_for_parsing(urdf_text))
    links = {link.get("name") for link in root.findall("link") if link.get("name")}
    children = {
        child.get("link")
        for joint in root.findall("joint")
        if (child := joint.find("child")) is not None and child.get("link")
    }
    roots = links - children
    if len(roots) != 1:
        raise ValueError(f"Expected one root link, found: {', '.join(sorted(roots))}")
    root_link = roots.pop()
    # guarded by `standalone` (default true) so a robot that gets merged into another
    # one (e.g. arm_description included into viator_description) can pass
    # standalone:=false and drop this link instead of ending up with two root links -
    # its own world link plus the other robot's base_link
    snippet = (
        "\n"
        '  <xacro:arg name="standalone" default="true"/>\n'
        '  <xacro:if value="$(arg standalone)">\n'
        "  <!-- World base link -->\n"
        '  <link name="world"/>\n'
        "\n"
        f'  <joint name="world_to_{root_link}" type="fixed">\n'
        '    <parent link="world"/>\n'
        f'    <child link="{root_link}"/>\n'
        '    <origin xyz="0 0 0" rpy="0 0 0"/>\n'
        "  </joint>\n"
        "  </xacro:if>\n"
        "\n"
    )
    match = re.search(r"<link\b", urdf_text)
    if match:
        pos = match.start()
        return urdf_text[:pos] + snippet + urdf_text[pos:]
    return urdf_text.replace("</robot>", snippet + "</robot>", 1)


def _disable_gravity(urdf_text: str) -> str:
    """Disable gravity on every link, via a per-link <gazebo reference="..."> tag
    (SDF has no single model-wide gravity switch).

    Only called for world_base_link robots: a fixed-base manipulator here is driven
    by gz_ros2_control's bare position_proportional_gain loop, which has no gravity
    compensation term (see arm_control.urdf.xacro's comment on that gain) - without
    this, the arm's own weight sags/swings against that bare P controller. A
    free-moving robot (world_base_link=False) needs real gravity to sit/drive, so
    this is never applied there.
    """
    tags = "".join(
        f'\n  <gazebo reference="{name}"><gravity>false</gravity></gazebo>'
        for name in _extract_links(urdf_text)
    )
    return urdf_text.replace("</robot>", tags + "\n</robot>")


def _inject_control_include(urdf_text: str, description_package: str, control_name: str) -> str:
    include = (
        f'  <xacro:include filename="$(find {description_package})'
        f'/urdf/{control_name}_control.urdf.xacro"/>\n'
    )
    return urdf_text.replace("</robot>", include + "</robot>")


def _reindent(text: str, from_spaces: int = 2, to_spaces: int = 4) -> str:
    result = []
    for line in text.splitlines(keepends=True):
        stripped = line.lstrip(" ")
        n = len(line) - len(stripped)
        level = round(n / from_spaces)
        result.append(" " * (level * to_spaces) + stripped)
    return "".join(result)


def postprocess(
    raw_urdf_path: str | Path,
    robot_name: str,
    world_base_link: bool = False,
    description_package: str | None = None,
    control_name: str | None = None,
) -> tuple:
    """Read raw URDF and apply all transforms.

    Returns (geometry_urdf_string, control_xacro_string, joint_list, link_list).
    joint_list items are (name, lower_limit, upper_limit).
    """
    description_package = description_package or f"{robot_name}_description"
    control_name = control_name or robot_name

    text = Path(raw_urdf_path).read_text(encoding="utf-8")
    text = _ensure_xacro_ns(text)
    text = _inject_xacro_properties(text, description_package)
    text = _rewrite_mesh_paths(text)
    if world_base_link:
        text = _inject_world_base_link(text)
    text = _prefix_link_names(text, robot_name)
    if world_base_link:
        text = _disable_gravity(text)
    links = _extract_links(text)
    joints = _extract_revolute_joints(text)
    control_xacro = _generate_control_xacro(robot_name, joints)
    text = _inject_control_include(text, description_package, control_name)
    text = _reindent(text)
    return text, control_xacro, joints, links
