"""URDF post-processing: xacro injection, mesh path rewriting, control generation"""

import re
import xml.etree.ElementTree as ET
from pathlib import Path


def _ensure_xacro_ns(urdf_text: str) -> str:
    if "xmlns:xacro" in urdf_text:
        return urdf_text
    return urdf_text.replace("<robot ", '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" ', 1)


def _inject_xacro_properties(urdf_text: str, robot_name: str) -> str:
    props = (
        "\n"
        "  <!-- XACRO -->\n"
        f'  <xacro:property name="mesh_path" value="package://{robot_name}_description/meshes"/>\n'
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
    snippet = (
        "\n"
        "  <!-- World base link -->\n"
        '  <link name="world"/>\n'
        "\n"
        f'  <joint name="world_to_{root_link}" type="fixed">\n'
        '    <parent link="world"/>\n'
        f'    <child link="{root_link}"/>\n'
        '    <origin xyz="0 0 0" rpy="0 0 0"/>\n'
        "  </joint>\n"
        "\n"
    )
    match = re.search(r"<link\b", urdf_text)
    if match:
        pos = match.start()
        return urdf_text[:pos] + snippet + urdf_text[pos:]
    return urdf_text.replace("</robot>", snippet + "</robot>", 1)


def _inject_control_include(urdf_text: str, robot_name: str) -> str:
    include = (
        f'  <xacro:include filename="$(find {robot_name}_description)'
        f'/urdf/{robot_name}_control.urdf.xacro"/>\n'
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


def postprocess(raw_urdf_path: str | Path, robot_name: str, world_base_link: bool = False) -> tuple:
    """Read raw URDF and apply all transforms.

    Returns (geometry_urdf_string, control_xacro_string, joint_list, link_list).
    joint_list items are (name, lower_limit, upper_limit).
    """
    text = Path(raw_urdf_path).read_text(encoding="utf-8")
    text = _ensure_xacro_ns(text)
    text = _inject_xacro_properties(text, robot_name)
    text = _rewrite_mesh_paths(text)
    if world_base_link:
        text = _inject_world_base_link(text)
    links = _extract_links(text)
    joints = _extract_revolute_joints(text)
    control_xacro = _generate_control_xacro(robot_name, joints)
    text = _inject_control_include(text, robot_name)
    text = _reindent(text)
    return text, control_xacro, joints, links
