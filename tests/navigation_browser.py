"""Run navigation_browser.cjs against an isolated SIM with temporary settings."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
from http.server import ThreadingHTTPServer

from host.webapp import Console
from host.webserver import Handler, Session


def main(script=None):
    """Exercise the real Web flow without opening hardware or editing field files."""
    root = Path(__file__).resolve().parents[1]
    (root / 'tmp').mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)
        console = Console(path / 'nav.json', path / 'aux.json', path / 'flow.json')
        console.connect('SIM', True)
        session = Session(console)
        handler = type('NavigationHandler', (Handler,), {'session': session})
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
        server.daemon_threads = True
        serving = threading.Thread(target=server.serve_forever, daemon=True)
        pumping = threading.Thread(target=session.run, daemon=True)
        serving.start()
        pumping.start()
        try:
            environment = dict(os.environ, NAV_URL='http://127.0.0.1:{}/'.format(server.server_port))
            browser_test = Path(__file__).with_name(script) if script else Path(__file__).with_suffix('.cjs')
            return subprocess.run(['node', str(browser_test)],
                                  cwd=root, env=environment).returncode
        finally:
            session.stopping.set()
            pumping.join(timeout=2)
            console.disconnect()
            session.cameras.close()
            server.shutdown()
            server.server_close()
            serving.join(timeout=2)


if __name__ == '__main__':
    sys.exit(main())
