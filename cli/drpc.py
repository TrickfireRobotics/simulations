"""Discord Rich Presence (DRPC) integration for TrickFire Simulation"""

import atexit
import os
import sys
import threading

from pypresence import Presence

# Discord keeps showing the last activity it was handed until the connection is
# cleared, so a run that just exits leaves a ghost "Playing" card behind. The
# presence lives on a daemon thread that gets killed at interpreter exit, so the
# clear has to happen while that thread is still alive: atexit asks it to stop
# and waits for it to confirm.
_stop = threading.Event()
_stopped = threading.Event()
_SHUTDOWN_TIMEOUT = 2  # seconds to let the thread clear before giving up


def _shutdown() -> None:
    _stop.set()
    _stopped.wait(timeout=_SHUTDOWN_TIMEOUT)


def _find_discord_socket_dir() -> str | None:
    """Return the directory containing a discord-ipc-N socket, or None if not found"""
    candidates = [
        os.environ.get("XDG_RUNTIME_DIR"),  # standard Linux user session
        os.environ.get("TMPDIR"),
        "/run/host-runtime",  # devcontainer: host XDG_RUNTIME_DIR mounted here
        "/run/user/1000",  # fallback for some Linux distros
        "/tmp",
    ]
    for base in filter(None, candidates):
        for i in range(10):
            if os.path.exists(os.path.join(base, f"discord-ipc-{i}")):
                return base
    return None


def rpc_start(is_docker: bool = False, robot_name: str = "Unknown Robot") -> None:
    """Start DRPC in a separate thread"""
    try:
        if sys.platform == "linux":
            socket_dir = _find_discord_socket_dir()
            if socket_dir is None:
                print("Discord RPC unavailable: no Discord IPC socket found")
                return
            # pypresence checks TMPDIR early in its path search; point it at the socket dir
            # so it works both natively and inside the devcontainer (/run/host-runtime).
            os.environ["TMPDIR"] = socket_dir

        client_id = "1504990746527404063"
        rpc = Presence(client_id)
        rpc.connect()
        rpc.update(
            name="TrickFire Simulation (" + robot_name + ")",
            details="Running TrickFire Simulation on " + robot_name,
            state="Running in Docker" if is_docker else "Running locally",
        )
        atexit.register(_shutdown)
        try:
            while not _stop.wait(15):
                pass
        finally:
            try:
                rpc.clear()
                rpc.close()
            except Exception:  # noqa: BLE001 - teardown is best-effort too
                pass
            _stopped.set()
    except Exception as e:  # noqa: BLE001 - RPC is best-effort, never fatal
        print(f"Discord RPC unavailable: {e}")
