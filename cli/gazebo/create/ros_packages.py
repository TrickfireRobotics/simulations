"""ROS 2 package generation and updating"""

import shutil
from pathlib import Path

from ...output import die as err
from ...output import info
from . import TEMPLATES
from .template import write_template


def generate_description_pkg(
    robot: str,
    geometry_urdf: str,
    control_xacro: str,
    meshes_src: Path,
    out_dir: Path,
) -> None:
    pkg_dir = out_dir / f"{robot}_description"
    urdf_dir = pkg_dir / "urdf"
    meshes_dir = pkg_dir / "meshes"

    urdf_dir.mkdir(parents=True, exist_ok=True)
    meshes_dir.mkdir(parents=True, exist_ok=True)

    (urdf_dir / f"{robot}.urdf").write_text(geometry_urdf)
    (urdf_dir / f"{robot}_control.urdf.xacro").write_text(control_xacro)

    if meshes_src.exists():
        for item in meshes_src.iterdir():
            dest = meshes_dir / item.name
            if item.is_dir():
                shutil.copytree(item, dest, dirs_exist_ok=True)
            else:
                shutil.copy2(item, dest)

    tmpl = TEMPLATES / "description"
    write_template(tmpl / "CMakeLists.txt", pkg_dir / "CMakeLists.txt", robot)
    write_template(tmpl / "package.xml", pkg_dir / "package.xml", robot)

    info(f"Created {pkg_dir.relative_to(out_dir.parent)}")


def generate_bringup_pkg(robot: str, joints: list, links: list, out_dir: Path) -> None:
    pkg_dir = out_dir / f"{robot}_bringup"
    tmpl = TEMPLATES / "bringup"

    write_template(tmpl / "CMakeLists.txt", pkg_dir / "CMakeLists.txt", robot)
    write_template(tmpl / "package.xml", pkg_dir / "package.xml", robot)

    if joints:
        joints_yaml = "".join(f"      - {name}\n" for name, _, _ in joints)
    else:
        joints_yaml = "      # no revolute joints found - fill in manually\n"

    write_template(
        tmpl / "config" / "controller.yaml",
        pkg_dir / "config" / f"{robot}.controller.yaml",
        robot,
        JOINTS=joints_yaml,
    )

    links_yaml = ""
    for name in sorted(links):
        links_yaml += f"        {name}:\n"
        links_yaml += "          Alpha: 1\n"
        links_yaml += "          Show Axes: false\n"
        links_yaml += "          Show Trail: false\n"
        if name != "base_link":
            links_yaml += "          Value: true\n"

    write_template(
        tmpl / "config" / "__ROBOT__.rviz",
        pkg_dir / "config" / f"{robot}.rviz",
        robot,
        LINKS=links_yaml,
    )

    write_template(
        tmpl / "launch" / "__ROBOT__.launch.py", pkg_dir / "launch" / f"{robot}.launch.py", robot
    )

    info(f"Created {pkg_dir.relative_to(out_dir.parent)}")


def update_description_pkg(
    robot: str,
    geometry_urdf: str,
    generated_meshes_src: Path,
    out_dir: Path,
    package_name: str | None = None,
    description_file: str | None = None,
) -> None:
    package_name = package_name or f"{robot}_description"
    description_file = description_file or f"{robot}.urdf"
    pkg_dir = out_dir / package_name
    urdf_dir = pkg_dir / "urdf"
    meshes_dir = pkg_dir / "meshes"

    if not pkg_dir.exists():
        err(f"Package {pkg_dir} does not exist. Use 'sim create' first.")

    (urdf_dir / description_file).write_text(geometry_urdf)

    if generated_meshes_src.exists():
        if meshes_dir.exists():
            shutil.rmtree(meshes_dir)
        meshes_dir.mkdir(parents=True, exist_ok=True)
        for item in generated_meshes_src.iterdir():
            dest = meshes_dir / item.name
            if item.is_dir():
                shutil.copytree(item, dest, dirs_exist_ok=True)
            else:
                shutil.copy2(item, dest)

    info(f"Updated {pkg_dir.relative_to(out_dir.parent)}/urdf and meshes")
