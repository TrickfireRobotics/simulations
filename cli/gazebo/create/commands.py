"""Subcommands main logic"""

import shutil
import tempfile
from pathlib import Path

from ...output import die as err
from ...output import info, warn
from . import PACKAGE_DIR
from .onshape import download, parse_onshape_url
from .reduce_stl import batch_process_directory
from .registry import load_robots_json, save_robots_json
from .ros_packages import (
    generate_bringup_pkg,
    generate_description_pkg,
    update_description_pkg,
)
from .urdf import postprocess


def _prompt_yes_no(question: str) -> bool:
    while True:
        answer = input(f"[sim] {question} [y/n]: ").strip().lower()
        if answer in ("y", "yes"):
            return True
        if answer in ("n", "no"):
            return False
        print("[sim] Please answer y or n.")


def _reduce_meshes(meshes_dir: Path, no_reduce: bool) -> None:
    if no_reduce:
        info("Skipping STL reduction")
        return
    if not meshes_dir.exists() or not any(meshes_dir.iterdir()):
        return
    info("Reducing STL triangle counts...")
    batch_process_directory(str(meshes_dir), str(meshes_dir))


def _scaffold_packages(
    robot: str, urdf_path: Path, meshes_src: Path, out_dir: Path, world_base_link: bool = False
) -> None:
    if not meshes_src.exists():
        warn(f"Meshes directory {meshes_src} was not created")
    elif not any(meshes_src.iterdir()):
        warn(f"Meshes directory {meshes_src} is empty")

    info("Post-processing URDF...")
    geometry_urdf, control_xacro, joints, links = postprocess(urdf_path, robot, world_base_link)

    if joints:
        joint_names = [name for name, _, _ in joints]
        info(f"Found {len(joints)} revolute joint(s): {', '.join(joint_names)}")
    else:
        info("No revolute joints found")

    info("Generating description package...")
    generate_description_pkg(robot, geometry_urdf, control_xacro, meshes_src, out_dir)

    info("Generating bringup package...")
    generate_bringup_pkg(robot, joints, links, out_dir)

    info("Done!")
    info(f"Packages written to: {out_dir}")
    info(f" ├─ {robot}_description/")
    info(f" └─ {robot}_bringup/")


def cmd_create(args) -> None:
    robot = args.robot_name
    out_dir = Path(args.output_dir)

    info(f"Creating packages for robot: {robot}")

    api_url, doc_id, context_type, context_id, el_id = parse_onshape_url(args.onshape_url)
    info(f"documentId={doc_id}  {context_type}Id={context_id}  elementId={el_id}")

    workdir = download(robot, doc_id, context_type, context_id, el_id, api_url)

    urdf_path = workdir / "robot.urdf"
    if not urdf_path.exists():
        err(
            f"Expected {urdf_path} but it was not found.\n"
            "  Check onshape-to-robot output above for errors."
        )

    if args.attach_to_world is not None:
        world_base_link = args.attach_to_world
    else:
        world_base_link = _prompt_yes_no(
            "Fix robot to world? (arm/fixed base = yes, mobile chassis = no)"
        )

    _reduce_meshes(workdir / "assets", args.no_reduce)
    _scaffold_packages(robot, urdf_path, workdir / "assets", out_dir, world_base_link)

    registry = load_robots_json()
    registry = [e for e in registry if e["name"] != robot]
    registry.append({"name": robot, "url": args.onshape_url, "world_base_link": world_base_link})
    save_robots_json(registry)
    info(f"Registered '{robot}' in robots.json (world_base_link={world_base_link})")


def cmd_local(args) -> None:
    robot = args.robot_name
    out_dir = Path(args.output_dir)
    raw_default = PACKAGE_DIR / "tests" / robot  # cli/create/tests/<robot>/
    urdf_path = Path(args.urdf) if args.urdf else raw_default / "robot.urdf"
    meshes_src = Path(args.assets) if args.assets else urdf_path.parent / "assets"

    if not urdf_path.exists():
        err(f"URDF file not found: {urdf_path}")

    info(f"Creating packages for robot: {robot} (local mode)")

    if args.attach_to_world is not None:
        world_base_link = args.attach_to_world
    else:
        world_base_link = _prompt_yes_no(
            "Fix robot to world? (arm/fixed base = yes, mobile chassis = no)"
        )

    if not args.no_reduce and meshes_src.exists() and any(meshes_src.iterdir()):
        with tempfile.TemporaryDirectory() as tmp:
            reduced = Path(tmp) / "assets"
            shutil.copytree(meshes_src, reduced)
            _reduce_meshes(reduced, no_reduce=False)
            _scaffold_packages(robot, urdf_path, reduced, out_dir, world_base_link)
    else:
        if args.no_reduce:
            info("Skipping STL reduction (--no-reduce)")
        _scaffold_packages(robot, urdf_path, meshes_src, out_dir, world_base_link)


def cmd_raw(args) -> None:
    robot = args.robot_name
    out_dir = Path(args.output_dir)

    if args.onshape_url:
        url = args.onshape_url
    else:
        registry = load_robots_json()
        entry = next((e for e in registry if e["name"] == robot), None)
        if entry is None:
            names = ", ".join(e["name"] for e in registry) or "(none)"
            err(
                f"Robot '{robot}' not found in robots.json.\n"
                f"  Available robots: {names}\n"
                "  Pass an OnShape URL as the second argument to use it directly."
            )
        url = entry["url"]

    api_url, doc_id, context_type, context_id, el_id = parse_onshape_url(url)
    info(f"documentId={doc_id}  {context_type}Id={context_id}  elementId={el_id}")

    workdir = download(robot, doc_id, context_type, context_id, el_id, api_url)

    dest = out_dir / robot
    if dest.exists():
        shutil.rmtree(dest)
    shutil.copytree(workdir, dest)

    info(f"Saved raw URDF and assets to: {dest}")
    info(f"  {dest}/robot.urdf")
    info(f"  {dest}/assets/")
    info("")
    info("Re-run without hitting OnShape again:")
    info(f"  sim create {robot} --local --urdf {dest}/robot.urdf --assets {dest}/assets/")


def cmd_update(args) -> None:
    robot = args.robot_name
    out_dir = Path(args.output_dir)

    info(f"Updating packages for robot: {robot}")

    registry = load_robots_json()
    entry = next((e for e in registry if e["name"] == robot), None)
    if entry is None:
        names = ", ".join(e["name"] for e in registry) or "(none)"
        err(
            f"Robot '{robot}' not found in robots.json.\n"
            f"  Available robots: {names}\n"
            "  Use 'sim create' to add a new robot."
        )

    if "url" not in entry:
        err(
            f"Robot '{robot}' is assembled from other descriptions and cannot be "
            "updated directly. Update its source components instead."
        )

    api_url, doc_id, context_type, context_id, el_id = parse_onshape_url(entry["url"])
    info(f"documentId={doc_id}  {context_type}Id={context_id}  elementId={el_id}")

    source_name = entry.get("source_name", robot)
    workdir = download(source_name, doc_id, context_type, context_id, el_id, api_url)

    exported_urdf_path = workdir / "robot.urdf"
    exported_meshes_src = workdir / "assets"

    if not exported_urdf_path.exists():
        err(
            f"Expected {exported_urdf_path} but it was not found.\n"
            "  Check onshape-to-robot output above for errors."
        )

    if not exported_meshes_src.exists():
        warn(f"Meshes directory {exported_meshes_src} was not created")
    elif not any(exported_meshes_src.iterdir()):
        warn(f"Meshes directory {exported_meshes_src} is empty")

    world_base_link = entry.get("world_base_link", False)
    info(f"world_base_link={world_base_link} (from robots.json)")

    _reduce_meshes(exported_meshes_src, args.no_reduce)

    info("Post-processing URDF...")
    description_package = entry.get("description_package", f"{robot}_description")
    description_file = entry.get("description_file", f"{robot}.urdf")
    control_name = entry.get("control_name", Path(description_file).stem.split(".urdf")[0])
    geometry_urdf, _, _, _ = postprocess(
        exported_urdf_path,
        source_name,
        world_base_link,
        description_package=description_package,
        control_name=control_name,
    )

    info("Updating description package...")
    update_description_pkg(
        robot,
        geometry_urdf,
        exported_meshes_src,
        out_dir,
        package_name=description_package,
        description_file=description_file,
    )

    info("Done!")
    info(f"Updated {description_package}/urdf and meshes only.")
    info("Bringup package and control xacro were NOT modified.")
