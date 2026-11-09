"""JSON Schema validation for the Chrono configs.

Uses `jsonschema` when it is installed and falls back to a small built-in validator
otherwise. The fallback covers the subset the schemas in chrono/schemas/ actually use -
type, required, enum, minimum/maximum, minItems/maxItems, items, properties,
additionalProperties, and oneOf - which is enough to catch a typo'd key or a value in
the wrong units, and it means `sim chrono validate` works in a bare container.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from ..paths import CHRONO_SCHEMAS_DIR

# the marker the authored configs use for values nobody has measured yet
PLACEHOLDER_PREFIX = "REPLACE_WITH"


class SchemaError(Exception):
    """One or more configuration problems, each already prefixed with its key path."""

    def __init__(self, problems: list[str]) -> None:
        self.problems = problems
        count = len(problems)
        body = "\n".join(f"  - {p}" for p in problems)
        super().__init__(f"{count} problem{'s' if count != 1 else ''}:\n{body}")


def schema_path(name: str) -> Path:
    return CHRONO_SCHEMAS_DIR / f"{name}.schema.json"


def load_schema(name: str) -> dict[str, Any]:
    path = schema_path(name)
    if not path.is_file():
        raise SchemaError([f"no schema named '{name}' in {CHRONO_SCHEMAS_DIR}"])
    return json.loads(path.read_text())


def validate(data: Any, schema_name: str, source: str = "") -> None:
    """Validate `data` against a named schema, raising SchemaError with every problem."""
    schema = load_schema(schema_name)
    problems = _validate_with_jsonschema(data, schema)
    if problems is None:
        problems = _Validator(schema).check(data, "")

    if problems:
        prefix = f"{source}: " if source else ""
        raise SchemaError([prefix + p for p in problems])


def find_placeholders(data: Any, path: str = "") -> list[str]:
    """Every key still holding a REPLACE_WITH_* placeholder.

    Separate from schema validation because a placeholder is valid YAML of the right
    shape. The simulator rejects them too, but reporting them here names the authored
    file rather than the generated manifest.
    """
    found: list[str] = []
    if isinstance(data, dict):
        for key, value in data.items():
            found += find_placeholders(value, f"{path}.{key}" if path else key)
    elif isinstance(data, list):
        for index, value in enumerate(data):
            found += find_placeholders(value, f"{path}[{index}]")
    elif isinstance(data, str) and data.startswith(PLACEHOLDER_PREFIX):
        found.append(path)
    return found


def is_placeholder(value: Any) -> bool:
    return isinstance(value, str) and value.startswith(PLACEHOLDER_PREFIX)


def _validate_with_jsonschema(data: Any, schema: dict[str, Any]) -> list[str] | None:
    """Full validation when jsonschema is available, or None to use the fallback."""
    try:
        import jsonschema
    except ImportError:
        return None

    validator = jsonschema.Draft202012Validator(schema)
    problems = []
    for error in sorted(validator.iter_errors(data), key=lambda e: list(e.absolute_path)):
        # A REPLACE_WITH marker sitting in a numeric field is structurally a string, so
        # every type check on it fires. That is not what it means: the value is not
        # malformed, it has not been measured yet. find_placeholders reports those, and
        # the simulator refuses to run them, so the type complaint is dropped here to
        # keep the two kinds of problem distinguishable.
        if is_placeholder(error.instance):
            continue
        problems.append(
            f"{'.'.join(str(part) for part in error.absolute_path) or '(root)'}: {error.message}"
        )
    return problems


class _Validator:
    """The fallback validator. Deliberately small, and only as strict as the schemas need."""

    def __init__(self, schema: dict[str, Any]) -> None:
        self.root = schema

    def check(self, data: Any, path: str) -> list[str]:
        return self._check(data, self.root, path)

    def _check(self, data: Any, schema: dict[str, Any], path: str) -> list[str]:
        if "$ref" in schema:
            schema = self._resolve(schema["$ref"])

        problems: list[str] = []
        where = path or "(root)"

        if "oneOf" in schema:
            # Report the branch that got furthest rather than every branch's complaints,
            # which for a tagged union is almost always the one the author meant.
            branches = [self._check(data, option, path) for option in schema["oneOf"]]
            if not any(not b for b in branches):
                best = min(branches, key=len)
                problems += best or [f"{where}: does not match any allowed variant"]
            return problems

        # See _validate_with_jsonschema: an unmeasured placeholder is reported by
        # find_placeholders, not as a type error.
        if is_placeholder(data):
            return []

        expected = schema.get("type")
        if expected and not _is_type(data, expected):
            return [f"{where}: expected {expected}, got {_type_name(data)}"]

        if "enum" in schema and data not in schema["enum"]:
            allowed = ", ".join(repr(v) for v in schema["enum"])
            problems.append(f"{where}: must be one of {allowed}, got {data!r}")

        if isinstance(data, (int, float)) and not isinstance(data, bool):
            if "minimum" in schema and data < schema["minimum"]:
                problems.append(f"{where}: must be >= {schema['minimum']}, got {data}")
            if "maximum" in schema and data > schema["maximum"]:
                problems.append(f"{where}: must be <= {schema['maximum']}, got {data}")
            if "exclusiveMinimum" in schema and data <= schema["exclusiveMinimum"]:
                problems.append(f"{where}: must be > {schema['exclusiveMinimum']}, got {data}")

        if isinstance(data, list):
            problems += self._check_list(data, schema, path, where)

        if isinstance(data, dict):
            problems += self._check_dict(data, schema, path, where)

        return problems

    def _check_list(self, data: list, schema: dict, path: str, where: str) -> list[str]:
        problems = []
        if "minItems" in schema and len(data) < schema["minItems"]:
            problems.append(f"{where}: needs at least {schema['minItems']} items")
        if "maxItems" in schema and len(data) > schema["maxItems"]:
            problems.append(f"{where}: allows at most {schema['maxItems']} items")

        item_schema = schema.get("items")
        if isinstance(item_schema, dict):
            for index, value in enumerate(data):
                problems += self._check(value, item_schema, f"{path}[{index}]")
        return problems

    def _check_dict(self, data: dict, schema: dict, path: str, where: str) -> list[str]:
        problems = []
        for key in schema.get("required", []):
            if key not in data:
                problems.append(f"{where}: missing required key '{key}'")

        properties = schema.get("properties", {})
        for key, value in data.items():
            child = f"{path}.{key}" if path else key
            if key in properties:
                problems += self._check(value, properties[key], child)
            elif schema.get("additionalProperties") is False:
                known = ", ".join(sorted(properties)) or "(none)"
                problems.append(f"{child}: unknown key. Known keys: {known}")
            elif isinstance(schema.get("additionalProperties"), dict):
                problems += self._check(value, schema["additionalProperties"], child)
        return problems

    def _resolve(self, ref: str) -> dict[str, Any]:
        if not ref.startswith("#/"):
            raise SchemaError([f"only local $ref is supported, got '{ref}'"])
        node: Any = self.root
        for part in ref[2:].split("/"):
            node = node[part]
        return node


_TYPE_CHECKS = {
    "object": lambda v: isinstance(v, dict),
    "array": lambda v: isinstance(v, list),
    "string": lambda v: isinstance(v, str),
    "boolean": lambda v: isinstance(v, bool),
    "integer": lambda v: isinstance(v, int) and not isinstance(v, bool),
    "number": lambda v: isinstance(v, (int, float)) and not isinstance(v, bool),
    "null": lambda v: v is None,
}


def _is_type(value: Any, expected: str | list[str]) -> bool:
    if isinstance(expected, list):
        return any(_is_type(value, option) for option in expected)
    check = _TYPE_CHECKS.get(expected)
    return check(value) if check else True


def _type_name(value: Any) -> str:
    for name, check in _TYPE_CHECKS.items():
        if name != "number" and check(value):
            return name
    return type(value).__name__
