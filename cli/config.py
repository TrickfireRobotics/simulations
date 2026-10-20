"""Local CLI configuration stored at ~/.config/trickfire/token.json

Two credentials are accepted for dashboard access:
  - a personal API key (`tf_...`), for anyone with a dashboard account
  - the rotating 6-digit access code, which any dashboard user can read off
    Settings and pass along. It stops working when the code rotates at midnight.

Priority order:
  1. SIM_API_KEY / SIM_ACCESS_CODE env vars   (CI / automated)
  2. ~/.config/trickfire/token.json           (developer local, written by sim gazebo auth)
"""

import json
import os
import re
from datetime import date
from pathlib import Path

_CONFIG_DIR = Path.home() / ".config" / "trickfire"
_CONFIG_FILE = _CONFIG_DIR / "token.json"

API_KEY = "api_key"
ACCESS_CODE = "access_code"

_CODE_RE = re.compile(r"^\d{6}$")


def normalize_code(value: str) -> str:
    """Strip spaces and dashes people paste along with the code"""
    return re.sub(r"\D", "", value)


def looks_like_code(value: str) -> bool:
    return bool(_CODE_RE.match(normalize_code(value)))


def _read_file() -> dict:
    if not _CONFIG_FILE.exists():
        return {}
    try:
        data = json.loads(_CONFIG_FILE.read_text())
    except (json.JSONDecodeError, OSError):
        return {}
    if isinstance(data, dict):
        return data
    return {API_KEY: data}  # legacy: bare string


def load() -> tuple[str, str] | None:
    """Return (kind, value) for the active credential, or None if there isn't one"""
    env_key = os.environ.get("SIM_API_KEY")
    if env_key:
        return API_KEY, env_key

    env_code = os.environ.get("SIM_ACCESS_CODE")
    if env_code:
        return ACCESS_CODE, normalize_code(env_code)

    data = _read_file()

    if data.get(API_KEY):
        return API_KEY, data[API_KEY]

    code = data.get(ACCESS_CODE)
    if code and data.get("saved_on") == date.today().isoformat():
        return ACCESS_CODE, code

    return None


def headers() -> dict | None:
    """Auth headers for dashboard requests, or None if not configured"""
    cred = load()
    if not cred:
        return None
    kind, value = cred
    if kind == API_KEY:
        return {"x-api-key": value}
    return {"x-access-code": value}


def save(kind: str, value: str) -> None:
    _CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    payload = {kind: value}
    if kind == ACCESS_CODE:
        payload["saved_on"] = date.today().isoformat()
    _CONFIG_FILE.write_text(json.dumps(payload, indent=2))
    _CONFIG_FILE.chmod(0o600)


def clear() -> None:
    if _CONFIG_FILE.exists():
        _CONFIG_FILE.unlink()
