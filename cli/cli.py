"""Main CLI entry point for the sim command"""

import argparse
import shutil
import sys
import threading

from .chrono import chrono
from .drpc import rpc_start
from .output import die, info
from .paths import GAZEBO_WORKSPACE_DIR

_GAZEBO_SUBCOMMANDS = frozenset({"clean", "create", "update", "auth"})


def _build_gazebo_parser(subparsers: argparse._SubParsersAction) -> tuple:
    gazebo = subparsers.add_parser("gazebo", help="Gazebo + ROS2 robot simulation")
    gsubs = gazebo.add_subparsers(dest="command")

    gsubs.add_parser("clean", help="Delete build artifacts (build/, install/, log/)")

    create_p = gsubs.add_parser("create", help="Generate robot packages from an OnShape model")
    create_p.add_argument("robot", help="Robot name (e.g., arm, gripper)")
    create_p.add_argument(
        "onshape_url",
        nargs="?",
        default=None,
        help="Full OnShape document URL (required unless --local is set)",
    )
    create_p.add_argument(
        "--raw", action="store_true", help="Download raw OnShape files only (for debugging)"
    )
    create_p.add_argument(
        "--local",
        action="store_true",
        help="Generate packages from a local URDF instead of OnShape",
    )
    create_p.add_argument(
        "--urdf", default=None, help="Path to local robot.urdf (used with --local)"
    )
    create_p.add_argument(
        "--assets", default=None, help="Path to local assets/ dir (used with --local)"
    )
    create_p.add_argument("--output-dir", default=None)
    create_p.add_argument(
        "--no-reduce", action="store_true", help="Skip STL mesh triangle-count reduction"
    )
    create_p.add_argument(
        "--attach-to-world",
        dest="attach_to_world",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Fix robot to world frame (arm = yes, mobile = no)",
    )

    update_p = gsubs.add_parser("update", help="Update existing robot URDF and meshes from OnShape")
    update_p.add_argument("robot", help="Robot name (must exist in robots.json)")
    update_p.add_argument("--output-dir", default=None)
    update_p.add_argument("--no-reduce", action="store_true")

    gsubs.add_parser("auth", help="Configure dashboard API key for OnShape access")

    return gazebo, gsubs, create_p


def _build_chrono_parser(subparsers: argparse._SubParsersAction) -> tuple:
    chrono = subparsers.add_parser("chrono", help="chrono simulation (Chrono SCM)")
    wsubs = chrono.add_subparsers(dest="command")
    wsubs.add_parser("run", help="Run a chrono simulation")
    wsubs.add_parser("clean", help="Clean chrono build files")
    return chrono, wsubs


def main() -> None:
    # Pre-route "sim gazebo <robot> [flags]" before argparse sees it,
    # since argparse subparsers always claim positionals before other handlers.
    if (
        len(sys.argv) >= 3
        and sys.argv[1] == "gazebo"
        and sys.argv[2] not in _GAZEBO_SUBCOMMANDS
        and not sys.argv[2].startswith("-")
    ):
        try:
            _launch_gazebo(sys.argv[2], sys.argv[3:])
        except KeyboardInterrupt:
            sys.exit(130)
        except Exception as e:  # noqa: BLE001 - top-level handler, report and exit
            die(str(e))
        return

    parser = argparse.ArgumentParser(
        prog="sim",
        description="TrickFire simulation launcher",
    )
    subparsers = parser.add_subparsers(dest="sim")

    gazebo_parser, _gazebo_subs, create_p = _build_gazebo_parser(subparsers)
    chrono_parser, _chrono_subs = _build_chrono_parser(subparsers)

    args = parser.parse_args()

    try:
        if args.sim == "gazebo":
            if not args.command:
                gazebo_parser.print_help()
                return
            _dispatch_gazebo(args, create_p)

        elif args.sim == "chrono":
            if not args.command:
                chrono_parser.print_help()
                return
            _dispath_chrono(args)
        else:
            parser.print_help()
    except KeyboardInterrupt:
        sys.exit(130)
    except Exception as e:  # noqa: BLE001 - top-level handler, report and exit
        die(str(e))


def _launch_gazebo(robot: str, remaining: list[str]) -> None:
    p = argparse.ArgumentParser(prog="sim gazebo")
    p.add_argument("--build-only", action="store_true")
    p.add_argument("--no-build", action="store_true")
    p.add_argument(
        "-lq",
        "--local-qgc",
        action="store_true",
        help="Don't launch QGroundControl in-container (drone only) - use your own local one",
    )
    args = p.parse_args(remaining)

    from .gazebo.launch import build_and_launch, in_pixi

    threading.Thread(
        target=rpc_start,
        kwargs={"is_docker": not in_pixi(), "robot_name": robot},
        daemon=True,
    ).start()
    build_and_launch(
        robot, build_only=args.build_only, no_build=args.no_build, local_qgc=args.local_qgc
    )


def _dispath_chrono(args) -> None:
    if args.command == "run":
        chrono.run()
    elif args.command == "clean":
        chrono.clean()


def _dispatch_gazebo(args, create_p) -> None:
    from .auth import auth as cmd_auth
    from .gazebo.create import (
        PACKAGE_DIR,
    )
    from .gazebo.create import (
        create as robot_create,
    )
    from .gazebo.create import (
        update as robot_update,
    )
    from .gazebo.create.commands import cmd_local, cmd_raw

    if args.command == "clean":
        for name in ("build", "install", "log"):
            path = GAZEBO_WORKSPACE_DIR / name
            if path.exists():
                info(f"Removing {name}/...")
                shutil.rmtree(path)
        info("Cleaned")

    elif args.command == "create":
        if args.raw:
            cmd_raw(
                argparse.Namespace(
                    robot_name=args.robot,
                    onshape_url=args.onshape_url,
                    output_dir=args.output_dir or str(PACKAGE_DIR / "tests"),
                )
            )
        elif args.local:
            cmd_local(
                argparse.Namespace(
                    robot_name=args.robot,
                    output_dir=args.output_dir or str(GAZEBO_WORKSPACE_DIR),
                    urdf=args.urdf,
                    assets=args.assets,
                    no_reduce=args.no_reduce,
                    attach_to_world=args.attach_to_world,
                )
            )
        else:
            if not args.onshape_url:
                create_p.error("onshape_url is required unless --local or --raw is set")
            robot_create(
                args.robot,
                args.onshape_url,
                output_dir=args.output_dir,
                no_reduce=args.no_reduce,
                attach_to_world=args.attach_to_world,
            )

    elif args.command == "update":
        robot_update(
            args.robot,
            output_dir=args.output_dir,
            no_reduce=args.no_reduce,
        )

    elif args.command == "auth":
        cmd_auth()


if __name__ == "__main__":
    main()
