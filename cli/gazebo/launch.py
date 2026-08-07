"""Gazebo simulation launcher for the `sim` CLI"""

from __future__ import annotations

import os
import re
import shutil
import socket
import subprocess
from datetime import datetime
from pathlib import Path

from .. import paths
from ..output import die, info, warn
from ..paths import REPO_DIR, WORKSPACE_DIR

_ANSI_RE = re.compile(r"\x1B\[[0-9;]*[mK]")


def _robot_type(robot_name: str) -> str | None:
    from .create.registry import load_robots_json

    entry = next((e for e in load_robots_json() if e["name"] == robot_name), None)
    return entry.get("type") if entry else None


def _configure_ardupilot_env(robot_type: str | None, env: dict[str, str]) -> None:
    """Point Gazebo at the plugin/models built into the Docker image"""
    if robot_type != "ardupilot":
        return

    if not paths.ARDUCOPTER_BIN.is_file() or not paths.ARDUPILOT_GAZEBO_PLUGIN.is_file():
        die("ArduPilot SITL + ardupilot_gazebo plugin are missing!")

    for var, plugin_paths in (
        (
            "GZ_SIM_RESOURCE_PATH",
            [paths.ARDUPILOT_GAZEBO_DIR / "models", paths.ARDUPILOT_GAZEBO_DIR / "worlds"],
        ),
        ("GZ_SIM_SYSTEM_PLUGIN_PATH", [paths.ARDUPILOT_GAZEBO_BUILD_DIR]),
    ):
        existing = [p for p in env.get(var, "").split(":") if p]
        additions = [str(p) for p in plugin_paths if str(p) not in existing]
        env[var] = ":".join(additions + existing)

    env["ARDUCOPTER_BIN"] = str(paths.ARDUCOPTER_BIN)


def in_pixi() -> bool:
    return bool(os.environ.get("PIXI_PROJECT_ROOT") or os.environ.get("CONDA_PREFIX"))


def _strip_ansi(text: str) -> str:
    return _ANSI_RE.sub("", text)


def _validate_robot_layout(robot_name: str) -> tuple[str, str, str]:
    bringup_pkg = f"{robot_name}_bringup"
    description_pkg = f"{robot_name}_description"
    launch_file_name = f"{robot_name}.launch.py"

    bringup_dir = WORKSPACE_DIR / bringup_pkg
    if not bringup_dir.is_dir():
        die(
            f"Package '{bringup_pkg}' not found in {WORKSPACE_DIR}\n"
            f"Expected directory: {bringup_dir}"
        )

    description_dir = WORKSPACE_DIR / description_pkg
    if not description_dir.is_dir():
        die(
            f"Package '{description_pkg}' not found in {WORKSPACE_DIR}\n"
            f"Expected directory: {description_dir}"
        )

    launch_file = bringup_dir / "launch" / launch_file_name
    if not launch_file.is_file():
        die(f"Launch file not found: {launch_file}")

    return bringup_pkg, description_pkg, launch_file_name


def _drop_missing_prefix_paths(env: dict[str, str]) -> None:
    for key in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH"):
        if key not in env:
            continue
        live = [p for p in env[key].split(":") if p and Path(p).exists()]
        if live:
            env[key] = ":".join(live)
        else:
            del env[key]


def _run_logged_command(
    command: list[str] | str,
    *,
    cwd: Path,
    env: dict[str, str],
    log_path: Path,
    shell: bool = False,
) -> None:
    command_display = command if isinstance(command, str) else " ".join(command)

    with log_path.open("a", encoding="utf-8") as log_file:
        log_file.write(f"$ {command_display}\n")
        log_file.flush()

        process = subprocess.Popen(
            command,
            cwd=cwd,
            env=env,
            shell=shell,
            executable="/bin/bash" if shell else None,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

        try:
            assert process.stdout is not None
            for line in process.stdout:
                print(line, end="")
                log_file.write(_strip_ansi(line))
                log_file.flush()
        except KeyboardInterrupt:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            raise

        return_code = process.wait()

    if return_code != 0:
        raise subprocess.CalledProcessError(return_code, command)


def _x11_port_for(display: str) -> int:
    """The X11 protocol's TCP port for a display spec is 6000 + display number."""
    try:
        num_part = display.rsplit(":", 1)[-1].split(".")[0]
        return 6000 + int(num_part)
    except (ValueError, IndexError):
        return 6000


def _diagnose_display(display: str, xdpyinfo_stderr: str) -> str:
    """Pin down which layer (local socket, DNS, TCP, or X11 auth) is broken."""
    lines = [f"Cannot connect to display {display}", ""]

    if display.startswith(":"):
        lines += [
            "Expected a Wayland/X11 socket forwarded in from the hosts",
        ]
        return "\n".join(lines)

    host = display.split(":", 1)[0]

    try:
        ip = socket.gethostbyname(host)
        lines.append(f"[OK] DNS: '{host}' resolves to {ip}")
    except OSError as e:
        lines += [f"[FAIL] DNS: '{host}' did not resolve ({e})", "", "Is Docker Desktop running?"]
        return "\n".join(lines)

    port = _x11_port_for(display)
    try:
        with socket.create_connection((host, port), timeout=3):
            lines.append(f"[OK] TCP: port {port} on {host} is reachable")
    except OSError as e:
        lines += f"[FAIL] TCP: could not connect to {host}:{port} ({e})"
        return "\n".join(lines)

    lines += [
        "[FAIL] X11: connected over TCP, but the X server rejected the session:",
        f"{xdpyinfo_stderr.strip() or '(no error output captured)'}",
    ]
    return "\n".join(lines)


def _display_reachable(display: str) -> bool:
    """Whether an X server is answering on `display`."""
    return (
        subprocess.run(
            ["xdpyinfo", "-display", display],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        ).returncode
        == 0
    )


def _check_display() -> None:
    info("Checking for display...")
    display = os.environ.get("DISPLAY")
    if not display:
        die("DISPLAY not set! Try restarting the container")

    result = subprocess.run(
        ["xdpyinfo", "-display", display],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        die(_diagnose_display(display, result.stderr))


def _configure_virtualgl_rendering(env: dict[str, str]) -> list[str]:
    """Route GL rendering through VirtualGL when displaying on a remote X server"""
    display = os.environ.get("DISPLAY", "")
    if display.startswith(":"):
        return []

    if not shutil.which("vglrun"):
        warn("vglrun not installed! GL rendering will fail.")
        return []

    vgl_display = os.environ.get("VGL_DISPLAY", ":88")
    if not _display_reachable(vgl_display):
        warn(
            f"VirtualGL's 3D X server on {vgl_display} isn't running - run .devcontainer/x_server.sh"
        )
        return []

    for stale in ("LIBGL_ALWAYS_INDIRECT", "MESA_LOADER_DRIVER_OVERRIDE"):
        env.pop(stale, None)

    env["VGL_DISPLAY"] = vgl_display
    env.setdefault("VGL_COMPRESS", "proxy")

    info(f"Rendering through VirtualGL ({vgl_display} -> {display})")
    return ["vglrun"]


def _describe_display(env: dict[str, str], render_prefix: list[str]) -> str:
    """Launch banner"""
    display = env.get("DISPLAY", "?")
    if in_pixi():
        return "native (pixi environment, no container)"
    if env.get("FORCE_VNC"):
        return f"VNC / noVNC, software rendering ({display})"
    if render_prefix:
        return f"VirtualGL ({env.get('VGL_DISPLAY', '?')} -> {display})"
    return f"direct passthrough ({display})"


def _print_launch_banner(
    *,
    robot_name: str,
    robot_type: str | None,
    env: dict[str, str],
    render_prefix: list[str],
    log_path: Path,
) -> None:
    simulator = "gazebo + ArduPilot SITL" if robot_type == "ardupilot" else "gazebo"
    rows = [
        ("Robot", robot_name),
        ("Simulator", simulator),
        ("Display", _describe_display(env, render_prefix)),
        ("Log", str(log_path)),
    ]

    divider = "-" * 64
    label_width = max(len(label) for label, _ in rows) + 1
    print(divider)
    for label, value in rows:
        print(f"{label + ':':<{label_width}} {value}")
    print(divider)


def _configure_rendering(env: dict[str, str]) -> list[str]:
    """Pick how Gazebo/OGRE2 should get its GL context, based on where it's being displayed."""
    if os.environ.get("FORCE_VNC"):
        info("FORCE_VNC: forcing software rendering (llvmpipe)")
        env["LIBGL_ALWAYS_SOFTWARE"] = "1"
        return []

    return _configure_virtualgl_rendering(env)


def _setup_pixi_env() -> None:
    """Configure Gazebo plugin/resource paths and Qt platform for a pixi environment"""
    pixi_dir = REPO_DIR / ".pixi"
    if not pixi_dir.is_dir():
        die(f"No pixi environment at {pixi_dir}\nInstall dependencies first: pixi install")

    conda_prefix = os.environ.get("CONDA_PREFIX", "")
    if conda_prefix:
        plugin_lib = f"{conda_prefix}/lib"
        for var in ("GZ_SIM_SYSTEM_PLUGIN_PATH", "GZ_SIM_RESOURCE_PATH"):
            existing = os.environ.get(var, "")
            paths = [p for p in existing.split(":") if p]
            if plugin_lib not in paths:
                os.environ[var] = ":".join([plugin_lib] + paths)

        engine_plugins = f"{conda_prefix}/lib/gz-rendering-8/engine-plugins"
        existing = os.environ.get("GZ_RENDERING_PLUGIN_PATH", "")
        paths = [p for p in existing.split(":") if p]
        if engine_plugins not in paths:
            os.environ["GZ_RENDERING_PLUGIN_PATH"] = ":".join([engine_plugins] + paths)

        ogre2_media = f"{conda_prefix}/share/gz/gz-rendering8/ogre2/media"
        existing = os.environ.get("GZ_RENDERING_RESOURCE_PATH", "")
        paths = [p for p in existing.split(":") if p]
        if ogre2_media not in paths:
            os.environ["GZ_RENDERING_RESOURCE_PATH"] = ":".join([ogre2_media] + paths)

    if os.environ.get("QT_QPA_PLATFORM") == "wayland":
        os.environ["QT_QPA_PLATFORM"] = "xcb"


def build_and_launch(robot_name: str, *, build_only: bool = False, no_build: bool = False) -> None:
    """Build the ROS 2 workspace and launch a robot simulation."""
    if build_only and no_build:
        die("Use either --build-only or --no-build, not both")

    if in_pixi():
        _setup_pixi_env()
    else:
        _check_display()

    build = not no_build
    launch = not build_only
    robot_type = _robot_type(robot_name)
    env = os.environ.copy()
    render_prefix = _configure_rendering(env)
    _configure_ardupilot_env(robot_type, env)
    bringup_pkg, description_pkg, launch_file_name = _validate_robot_layout(robot_name)

    if build:
        for directory_name in ("build", "install", "log"):
            directory_path = WORKSPACE_DIR / directory_name
            if directory_path.exists():
                shutil.rmtree(directory_path)
        _drop_missing_prefix_paths(env)

    log_dir = WORKSPACE_DIR / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"{robot_name}-gazebo-{datetime.now():%Y-%m-%d_%H-%M}.log"  # noqa: DTZ005

    _print_launch_banner(
        robot_name=robot_name,
        robot_type=robot_type,
        env=env,
        render_prefix=render_prefix,
        log_path=log_path,
    )

    setup_bash = WORKSPACE_DIR / "install" / "setup.bash"

    if build:
        info("Building ROS2 workspace...")
        _run_logged_command(
            [
                "colcon",
                "build",
                "--packages-up-to",
                bringup_pkg,
                description_pkg,
                "sim_worlds",
                "sim_common",
                "--cmake-args",
                "-DBUILD_TESTING=OFF",
            ],
            cwd=WORKSPACE_DIR,
            env=env,
            log_path=log_path,
        )
        info("Build complete")

    if not setup_bash.is_file():
        die("Missing install/setup.bash - run without --no-build once to generate it")

    if not launch:
        info("Build-only requested; skipping launch")
        return

    info("Sourcing ROS2 environment and launching simulation...")
    launch_script = f"""
set -e
source install/setup.bash

SIM_WORLDS_SHARE=\"$PWD/install/sim_worlds/share/sim_worlds\"
if [ -d \"$SIM_WORLDS_SHARE/worlds\" ]; then
    export GZ_SIM_RESOURCE_PATH=\"${{GZ_SIM_RESOURCE_PATH:+$GZ_SIM_RESOURCE_PATH:}}$SIM_WORLDS_SHARE\"
    echo \"[INFO] GZ_SIM_RESOURCE_PATH set to: $GZ_SIM_RESOURCE_PATH\"
else
    echo \"[WARN] sim_worlds share directory not found - world files may not load\"
    echo \"       Expected: $SIM_WORLDS_SHARE/worlds\"
fi

exec {" ".join(render_prefix)} ros2 launch \"{bringup_pkg}\" \"{launch_file_name}\" gui:=true
""".strip()

    try:
        _run_logged_command(
            launch_script,
            cwd=WORKSPACE_DIR,
            env=env,
            log_path=log_path,
            shell=True,
        )
    except subprocess.CalledProcessError as error:
        warn(f"Launch failed with exit code {error.returncode}")
        raise
