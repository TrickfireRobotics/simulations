"""sim gazebo auth - configure dashboard access for OnShape exports"""

import os
import sys

import requests

from . import config as cfg
from .output import die, info

DASHBOARD_URL = os.environ.get("SIM_DASHBOARD_URL", "https://dashboard.trickfirerobotics.com")


def _prompt() -> tuple[str, str]:
    print()
    print("Enter the 6-digit sim access code for today.")
    print("Any team lead can read it off dashboard.trickfirerobotics.com → Settings.")
    print("The code rotates at midnight, so you will need a fresh one tomorrow.")
    print()
    try:
        entered = input("Access code (or a dashboard API key): ").strip()
    except (KeyboardInterrupt, EOFError):
        sys.exit(130)

    if not entered:
        die("An access code is required!")

    if cfg.looks_like_code(entered):
        return cfg.ACCESS_CODE, cfg.normalize_code(entered)
    if entered.startswith("tf_"):
        return cfg.API_KEY, entered

    die("That is not a 6-digit code or a dashboard API key.")


def auth() -> None:
    existing = cfg.load()
    kind, value = existing if existing else _prompt()

    header = {"x-api-key": value} if kind == cfg.API_KEY else {"x-access-code": value}

    info("Verifying...")
    try:
        resp = requests.post(
            f"{DASHBOARD_URL}/api/service/verify",
            headers=header,
            timeout=10,
        )
    except requests.ConnectionError:
        die(f"Could not connect to {DASHBOARD_URL}. Check your network connection")
    except requests.Timeout:
        die("Connection timed out")

    if resp.status_code == 401:
        if kind == cfg.ACCESS_CODE:
            die("That code was rejected. Ask a lead for today's code and try again.")
        die("Invalid API key! Generate a new one at Settings → CLI access.")
    if resp.status_code == 429:
        die("Too many attempts from this network. Wait a bit and try again.")
    if not resp.ok:
        die(f"Verification failed ({resp.status_code}). Check the dashboard URL.")

    data = resp.json()

    cfg.save(kind, value)
    print()
    if kind == cfg.ACCESS_CODE:
        info(f"Access code accepted, good until {data.get('expiresAtLabel', 'midnight')}")
    else:
        info(f"Authenticated as: {data.get('name', 'unknown')}")
    info(f"Saved to {cfg._CONFIG_FILE}")
