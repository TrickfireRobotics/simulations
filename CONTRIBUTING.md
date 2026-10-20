# Contributing

Thanks for wanting to contribute to this repo! This guide covers the layout of the repo and how to work on each part.

## Repo structure

```
simulations/
├── gazebo/          # ROS 2 workspace - Gazebo simulation packages
├── chrono/          # Project Chrono terrain/wheel simulation
├── cli/             # Python CLI (the `sim` command)
│   ├── cli.py       # Main entry point and argument parsing
│   ├── auth.py      # `sim gazebo auth` - dashboard API key management
│   ├── config.py    # Persistent config storage
│   ├── drpc.py      # Discord RPC integration
│   ├── paths.py     # Workspace path utilities
│   ├── output.py    # Terminal output helpers (info, warn, die)
│   ├── gazebo/      # `sim gazebo *` sub-commands
│   └── chrono/      # `sim chrono *` sub-commands
├── docker/          # Dockerfile and docker-compose files
├── pixi.toml        # Native environment definition (ROS 2 Jazzy + Gazebo Harmonic)
├── assets/          # Club branding
├── robots.json      # Robot configuration registry
├── ruff.toml        # Ruff linter/formatter configuration
└── pyproject.toml   # Python package config
```

## Projects

### `gazebo/` - Gazebo Simulation

The ROS 2 workspace containing the Gazebo Harmonic simulation packages. Built with `colcon` via the native pixi environment or the Dev Container.

**Packages:**

- `<robot>_bringup` - launch files, controller YAML, RViz config
- `<robot>_description` - URDF/SDF models and meshes
- `sim_common` - shared ROS utilities and the joint GUI node
- `sim_worlds` - Gazebo world definitions

When adding or modifying a package, keep `package.xml` and `CMakeLists.txt` up to date so `colcon` can resolve build order and dependencies.

### `chrono/` - Chrono Terrain Simulation

Project Chrono simulations for wheel-soil interaction research using the SCM (Soil Contact Model). Still in early development.

```
chrono/
├── CMakeLists.txt
├── data/               # Chrono runtime data (meshes, colormaps, shaders)
└── src/
    ├── main.cpp        # wiring and the run loop
    ├── sim_config.*    # startup parameters (SimConfig) and runtime state (SimState)
    ├── scm_scene.*     # the Chrono system: wheel, motor, SCM terrain, reset
    ├── vis_system.*    # ChVisualSystemVSG subclass - branding and camera swap
    ├── orbit_camera.*  # turntable camera handler, replaces vsg::Trackball
    └── sim_gui.*       # the ImGui control panel
```

The GUI uses the ImGui that Chrono's VSG module already links (`vsgImGui`, exported as an
interface dependency of `Chrono::Chrono_vsg`). Do not vendor a second copy of ImGui - two
`ImGuiContext`s in one process will not end well.

`vsg::Trackball` is disabled via the protected `m_camera_trackball` flag, which is why
`SimVisualSystem` subclasses `ChVisualSystemVSG` rather than just configuring it.

### `cli/` - Simulation CLI

Python package providing the `sim` command. Two top-level subcommands:

**`sim gazebo`** - Gazebo/ROS 2 simulation:

- `cli/gazebo/docker.py` - `sim gazebo docker` build and launch logic
- `cli/gazebo/native.py` - `sim gazebo native` build and launch logic (pixi environment)
- `cli/gazebo/create/` - `sim gazebo create` / `sim gazebo update` (OnShape → URDF pipeline)

**`sim chrono`** - Chrono SCM terrain simulation:

- `cli/chrono/chrono.py` - `sim chrono run` / `sim chrono clean`

**Shared:**

- `cli/auth.py` - `sim gazebo auth` (dashboard API key)
- `cli/paths.py` - Workspace path utilities
- `cli/output.py` - Terminal output helpers (`info`, `warn`, `die`)

### `docs/` - Documentation site

Go look at the [trickfire-docs documentation](https://docs.trickfirerobotics.com/trickfire-docs/) for more info.

## Dev setup

The primary workflow uses pixi for a self-contained native environment. Follow the [Getting Started](https://docs.trickfirerobotics.com/gazebo-simulations/setup/getting-started/) guide.

For `docs/` changes, only Node.js is required.

After installing, run `make hooks` once to register the git pre-commit and commit-msg hooks locally.

## Formatting

All formatters run automatically on save in VS Code. Install the recommended extensions when prompted.

| Language               | Formatter                                                                                                         |
| ---------------------- | ----------------------------------------------------------------------------------------------------------------- |
| Python                 | [Ruff](https://docs.astral.sh/ruff/) (`charliermarsh.ruff`)                                                       |
| JS / TS / JSON / JSONC | Prettier (`esbenp.prettier-vscode`) - follow the project's Prettier configuration (including 4-space indentation) |
| XML                    | XML Tools (`DotJoshJohnson.xml`)                                                                                  |
| Dockerfile             | Docker (`ms-azuretools.vscode-containers`)                                                                        |

Pre-commit hooks enforce all formatters automatically at commit time. You can also run them manually against staged files:

```bash
pre-commit run
```

For Python specifically:

```bash
ruff check .
ruff format .
```

Configuration lives in `ruff.toml`. The `gazebo/build`, `gazebo/install`, and `gazebo/log` directories are excluded automatically.
