"""Loading and merging the authored YAML configs.

Configs live under chrono/configs/<kind>/ and are referenced by name or by path. A
config may inherit from another of the same kind via `extends:`, which is how a study
expresses "the hill scenario but at 15 degrees" without copying the whole file.

Inheritance is resolved here, in the CLI, and the result is written into the run
manifest. The simulator never sees an `extends:` - so changing a base config later
cannot alter what an old result meant.
"""

from __future__ import annotations

import copy
import hashlib
from pathlib import Path
from typing import Any

import yaml

from ..paths import CHRONO_CONFIGS_DIR

# config kind -> subdirectory and schema name
KINDS = {
    "actuator": ("actuators", "actuator"),
    "battery": ("batteries", "battery"),
    "rover": ("rovers", "rover"),
    "soil": ("soils", "soil"),
    "terrain": ("terrains", "terrain"),
    "scenario": ("scenarios", "scenario"),
    "study": ("studies", "study"),
}

MAX_INHERITANCE_DEPTH = 8


class ConfigError(Exception):
    pass


def kind_dir(kind: str) -> Path:
    if kind not in KINDS:
        raise ConfigError(f"unknown config kind '{kind}'. Known: {', '.join(sorted(KINDS))}")
    return CHRONO_CONFIGS_DIR / KINDS[kind][0]


def schema_for(kind: str) -> str:
    if kind not in KINDS:
        raise ConfigError(f"unknown config kind '{kind}'")
    return KINDS[kind][1]


def locate(kind: str, reference: str) -> Path:
    """Resolve a config reference to a file.

    Accepts a bare name ("urc-hill"), a path relative to chrono/ ("configs/scenarios/
    urc-hill.yaml"), or any absolute or working-directory-relative path - so the command
    lines in the development plan work as written and tab completion also works.
    """
    candidates = []

    path = Path(reference)
    if path.suffix in {".yaml", ".yml"}:
        candidates += [path, CHRONO_CONFIGS_DIR.parent / path, Path.cwd() / path]
    else:
        for suffix in (".yaml", ".yml"):
            candidates.append(kind_dir(kind) / f"{reference}{suffix}")
        candidates += [path, CHRONO_CONFIGS_DIR.parent / path]

    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()

    available = ", ".join(sorted(p.stem for p in kind_dir(kind).glob("*.y*ml"))) or "(none)"
    raise ConfigError(
        f"no {kind} config matching '{reference}'. Available in "
        f"{kind_dir(kind).relative_to(CHRONO_CONFIGS_DIR.parent.parent)}: {available}"
    )


def read(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text())
    except yaml.YAMLError as exc:
        raise ConfigError(f"{path} is not valid YAML: {exc}") from exc
    if data is None:
        raise ConfigError(f"{path} is empty")
    if not isinstance(data, dict):
        raise ConfigError(f"{path} must be a YAML mapping, got {type(data).__name__}")
    return data


def load(kind: str, reference: str) -> tuple[dict[str, Any], Path, list[Path]]:
    """Load a config with `extends:` resolved.

    Returns the merged config, the file it came from, and every file that contributed,
    so the manifest can record the whole inheritance chain.
    """
    path = locate(kind, reference)
    chain: list[Path] = []
    merged = _load_with_inheritance(kind, path, chain, depth=0)
    merged.pop("extends", None)
    return merged, path, chain


def _load_with_inheritance(kind: str, path: Path, chain: list[Path], depth: int) -> dict[str, Any]:
    if depth > MAX_INHERITANCE_DEPTH:
        trail = " -> ".join(p.name for p in chain)
        raise ConfigError(f"extends: chain is too deep or circular: {trail}")
    if path in chain:
        trail = " -> ".join(p.name for p in chain + [path])
        raise ConfigError(f"extends: cycle detected: {trail}")

    chain.append(path)
    data = read(path)

    parent_ref = data.get("extends")
    if not parent_ref:
        return data

    parent_path = locate(kind, parent_ref)
    parent = _load_with_inheritance(kind, parent_path, chain, depth + 1)
    return merge(parent, data)


def merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """Recursively merge `override` onto `base`.

    Lists replace rather than append. Appending would make "the hill scenario but with
    only a distance termination" impossible to express, and a half-inherited list of
    termination rules is exactly the kind of surprise this whole pipeline exists to
    avoid.
    """
    result = copy.deepcopy(base)
    for key, value in override.items():
        if key == "extends":
            continue
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def apply_overrides(config: dict[str, Any], overrides: dict[str, Any]) -> dict[str, Any]:
    """Apply dotted-path overrides, as a sweep's parameter grid produces.

    `{"driver.value": 2.0}` sets config["driver"]["value"]. Creating a key that the base
    config does not have is an error: a sweep over a misspelled parameter would otherwise
    produce a grid of identical runs.
    """
    result = copy.deepcopy(config)
    for dotted, value in overrides.items():
        parts = dotted.split(".")
        node = result
        for part in parts[:-1]:
            if part not in node or not isinstance(node[part], dict):
                raise ConfigError(
                    f"override '{dotted}' does not match the config: '{part}' is not a "
                    "mapping in it"
                )
            node = node[part]
        if parts[-1] not in node:
            raise ConfigError(
                f"override '{dotted}' sets a key the config does not have. Check the "
                "spelling - a sweep over a key that does not exist produces identical runs."
            )
        node[parts[-1]] = value
    return result


def content_hash(data: Any) -> str:
    """Stable hash of a resolved config, recorded in the manifest.

    Hashes the canonical YAML rather than the source file, so reformatting a config does
    not look like a change but editing a value does.
    """
    canonical = yaml.safe_dump(data, sort_keys=True, default_flow_style=False)
    return "sha256:" + hashlib.sha256(canonical.encode()).hexdigest()[:32]


def file_hash(path: Path) -> str:
    digest = hashlib.sha256(path.read_bytes())
    return "sha256:" + digest.hexdigest()[:32]


def list_configs(kind: str) -> list[tuple[str, str]]:
    """(name, description) for every config of a kind, for the `list` commands."""
    entries = []
    directory = kind_dir(kind)
    if not directory.is_dir():
        return entries

    for path in sorted(directory.glob("*.y*ml")):
        try:
            data = read(path)
        except ConfigError:
            entries.append((path.stem, "(unreadable)"))
            continue
        description = data.get("description") or data.get("name") or ""
        entries.append((path.stem, str(description)))
    return entries
