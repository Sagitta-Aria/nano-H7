"""Isolated recovery UI check; ephemeral SIM server/settings, no hardware access.

The injection command/endpoint exist only in this test harness, never production.
Run with python -m tests.recovery_browser and the existing Playwright/Edge runtime.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import threading
from http.server import ThreadingHTTPServer
from unittest.mock import patch

from host.simulator import SimulatedBoard, SimulatedSerial
from host.webapp import Console
from host.webserver import Handler, Session


class FaultBoard(SimulatedBoard):
    """Inject faults on the serial owner thread, ordered with real commands."""
    def dispatch(self, sequence, body):
        if body == 'SIMTEST FAULT':
            self.inject_motion_fault('step_deadline')
            return 'OK cmd=SIMTEST'
        return super().dispatch(sequence, body)


class FaultSerial(SimulatedSerial):
    def __init__(self):
        super().__init__()
        self.board = FaultBoard()


def main():
    """Own server/pump/worker lifetimes and remove only temporary settings."""
    with tempfile.TemporaryDirectory() as directory, patch('host.transport.SimulatedSerial', FaultSerial):
        root = Path(directory)
        console = Console(root/'navigation.json', root/'auxiliary.json', root/'flow.json')
        session = Session(console, exit_when_idle=False)

        class TestHandler(Handler):
            def do_POST(self):
                if self.path == '/api/test/fault':
                    self.rfile.read(int(self.headers.get('Content-Length', '0')))
                    console.connection.submit('SIMTEST FAULT')
                    self.json({'ok': True})
                else:
                    super().do_POST()

        TestHandler.session = session
        server = ThreadingHTTPServer(('127.0.0.1', 0), TestHandler)
        server.daemon_threads = True
        threads = [threading.Thread(target=server.serve_forever, daemon=True),
                   threading.Thread(target=session.run, daemon=True)]
        for thread in threads:thread.start()
        try:
            console.connect('SIM')
            env = dict(os.environ, NAV_URL='http://127.0.0.1:{}/'.format(server.server_address[1]))
            return subprocess.run(['node', str(Path(__file__).with_suffix('.cjs'))], env=env).returncode
        finally:
            session.stopping.set()
            worker = console.connection
            if worker:worker.close();worker.join(2)
            server.shutdown();server.server_close()
            for thread in threads:thread.join(2)


if __name__ == '__main__':raise SystemExit(main())
