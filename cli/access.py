"""Sim access code - asked for on every OnShape request through the dashboard

There is nothing to configure and nothing stored on disk. A lead reads the
current code off the dashboard (API Keys page) and passes it along; it rotates
at 3am Pacific. Set SIM_ACCESS_CODE for non-interactive runs.
"""

import os
import re
import sys

from .output import die

DASHBOARD_URL = os.environ.get("SIM_DASHBOARD_URL", "https://dashboard.trickfirerobotics.com")

_CODE_RE = re.compile(r"^\d{6}$")


def _clean(value: str) -> str:
    return re.sub(r"\D", "", value)


def prompt_for_code(retry: bool = False) -> str:
    """Ask for the 6-digit code, or read it from the environment when headless"""
    env_code = os.environ.get("SIM_ACCESS_CODE")
    if env_code and not retry:
        if not _CODE_RE.match(_clean(env_code)):
            die("SIM_ACCESS_CODE is not a 6-digit code.")
        return _clean(env_code)

    if not sys.stdin.isatty():
        if env_code:
            die("SIM_ACCESS_CODE was rejected. The code rotates at 3am, get the current one.")
        die(
            "No sim access code available.\n"
            " └─ Set SIM_ACCESS_CODE, or run this in a terminal to be asked for it."
        )

    if not retry:
        print()
        print("This needs the sim access code. Any lead can read the current one off")
        print(f"{DASHBOARD_URL} → API Keys. It rotates at 3am.")
        print()

    for _ in range(3):
        try:
            entered = _clean(input("Access code: "))
        except (KeyboardInterrupt, EOFError):
            sys.exit(130)
        if _CODE_RE.match(entered):
            return entered
        print("  That is not 6 digits, try again.")

    die("No valid access code entered.")


def headers(code: str) -> dict:
    return {"x-access-code": code}
