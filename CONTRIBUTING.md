# Contributing

Thanks for wanting to contribute to this repo! This guide covers the layout of the repo and how to work on each part.

## Repo structure

```
simulations/
├── gazebo/          # ROS 2 workspace - Gazebo simulation packages
├── chrono/          # Project Chrono terrain/wheel simulation
├── cli/             # Rust CLI (the `sim` command)
│   ├── src/         # binary sources
│   ├── tests/       # integration tests, run the built binary
│   └── Cargo.toml   # package manifest
├── cli-python/      # the previous Python CLI, kept for reference during the port
├── docker/          # Dockerfile and docker-compose files
├── pixi.toml        # Native environment definition (ROS 2 Jazzy + Gazebo Harmonic)
├── assets/          # Club branding
├── robots.json      # Robot configuration registry
├── ruff.toml        # Ruff linter/formatter configuration
├── Cargo.toml       # Cargo workspace (lints and release profile live here)
├── rust-toolchain.toml  # pinned Rust version, CI uses the same one
└── rustfmt.toml     # Rust formatter configuration
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

Rust binary crate providing the `sim` command. It lives in a Cargo workspace rooted at the repo
root, so `cargo` commands work from anywhere in the tree.

The CLI is mid-port from Python. The old implementation is still in `cli-python/` for reference and
will be deleted once the Rust side reaches parity.

```bash
make build         # debug build
make release       # optimized build
make rebuild       # clean just the sim crate, then build
make run ARGS="gazebo launch"
make test          # cargo test --workspace --all-targets
make lint          # clippy, warnings are errors
make check         # fmt + lint + test, same gate as CI
make doc           # build and open the API docs
```

#### Getting `sim` on your PATH

```bash
make link
```

That symlinks `target/debug/sim` into `~/.local/bin`, which `shell/config.sh` already puts on your
PATH. Because it is a symlink to the build output, `make build` is all you need after a change -
no reinstall step. `make unlink` removes it.

For a standalone optimized copy in `~/.cargo/bin` instead, use `make install` / `make uninstall`.
The Dev Container runs `make link` on create.

#### Tests

Integration tests in `cli/tests/` spawn the real binary through `env!("CARGO_BIN_EXE_sim")`, so
they exercise the CLI exactly as a user would. Unit tests go in `#[cfg(test)]` modules next to the
code they cover. `make test` runs both.

Shared lints are declared once in the workspace `Cargo.toml` under `[workspace.lints]` and
inherited by the package, so adding a crate later picks them up automatically. Clippy runs with
`clippy::all` and `clippy::pedantic`, and CI fails on any warning.

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
| Rust                   | rustfmt via [rust-analyzer](https://rust-analyzer.github.io/) (`rust-lang.rust-analyzer`)                         |
| Python                 | [Ruff](https://docs.astral.sh/ruff/) (`charliermarsh.ruff`)                                                       |
| JS / TS / JSON / JSONC | Prettier (`esbenp.prettier-vscode`) - follow the project's Prettier configuration (including 4-space indentation) |
| XML                    | XML Tools (`DotJoshJohnson.xml`)                                                                                  |
| Dockerfile             | Docker (`ms-azuretools.vscode-containers`)                                                                        |

Pre-commit hooks enforce all formatters automatically at commit time. You can also run them manually against staged files:

```bash
pre-commit run
```

`make format` runs every formatter in the repo at once (Rust, Python, C++, shell).

For Rust specifically:

```bash
cargo fmt --all        # or make format
make lint              # clippy with -D warnings
make check             # what CI runs
```

Configuration lives in `rustfmt.toml`, and lint levels in the `[workspace.lints]` section of the
root `Cargo.toml`.

For Python specifically:

```bash
ruff check .
ruff format .
```

Configuration lives in `ruff.toml`. Generated Gazebo files under `gazebo/.artifacts` are excluded automatically.

## CI

| Workflow                           | Checks                                                  |
| ---------------------------------- | ------------------------------------------------------- |
| `code-style.yml` ("Style Check")   | `ruff check`, `shfmt -d`, `cargo fmt --check`, clang-format |
| `rust.yml` ("Rust")                | clippy (`-D warnings`), `cargo test`, release build      |

The Rust workflow only triggers on changes under `cli/`, the Cargo manifests, or the toolchain
file. It installs the toolchain from `rust-toolchain.toml`, so CI and your machine always run the
same compiler.

`cargo fmt` runs as a pre-commit hook and clippy as a pre-push hook, so CI should rarely be the
first place you find out something is off. `make hooks` installs both.
