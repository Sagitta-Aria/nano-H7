"""Launcher for stations_browser.cjs: start an isolated demo server, run the check.

The browser check itself is JavaScript because it drives a real page; this only
owns the server lifetime so the check cannot run against the field box by
accident. No serial port is opened: the server runs --demo on a spare port.

    python -m tests.stations_browser
    python -m tests.stations_browser --keep   (leave the server up for triage)
"""
import argparse
import os
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CHECK = Path(__file__).with_suffix(".cjs")
PORT = int(os.environ.get("STATIONS_PORT", "8791"))
# The demo server reads and writes these next to host/, and the browser check
# edits them on purpose. They are tracked files, so a check that aborts midway
# would otherwise leave the repository holding test coordinates.
GUARDED = ("navigation.json", "mission.json", "auxiliary.json", "optical_flow.json")


def wait_for(url, timeout=60):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(url + "api/host/state", timeout=3) as response:
                if response.status == 200:
                    return True
        except Exception:
            time.sleep(.4)
    return False


def save_configs():
    """Remember the on-disk plans so an aborted check cannot keep its edits."""
    saved = {}
    for name in GUARDED:
        path = ROOT / "host" / name
        saved[name] = path.read_bytes() if path.exists() else None
    return saved


def restore_configs(saved):
    for name, data in saved.items():
        path = ROOT / "host" / name
        if data is None:
            if path.exists():
                path.unlink()
        elif path.read_bytes() != data:
            path.write_bytes(data)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--keep", action="store_true", help="leave the demo server running")
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args(argv)
    url = "http://127.0.0.1:{}/".format(args.port)
    saved = save_configs()

    server = subprocess.Popen(
        [sys.executable, "-u", "-m", "host.webserver", "--demo", "--no-browser",
         "--host", "127.0.0.1", "--port", str(args.port)],
        cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        if not wait_for(url):
            server.terminate()
            tail = server.stdout.read().decode("utf-8", "replace")[-2000:]
            print("demo server never came up:\n" + tail, file=sys.stderr)
            return 2
        environment = dict(os.environ, NAV_URL=url)
        result = subprocess.run(["node", str(CHECK)], cwd=str(ROOT), env=environment)
        return result.returncode
    finally:
        if args.keep:
            print("demo server left running at", url, "pid", server.pid)
            print("warning: host/*.json still hold the check's edits")
        else:
            server.terminate()
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill()
            # After the server exits, so a late write cannot land afterwards.
            restore_configs(saved)


if __name__ == "__main__":
    sys.exit(main())
