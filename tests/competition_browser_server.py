"""Temporary SIM-only competition fixture; never reads field configuration."""
import tempfile
import threading
import time
from pathlib import Path
from http.server import ThreadingHTTPServer
from host.webapp import Console
from host.webserver import Handler, Session


def main():
    """Expose a short saved route and normal HTTP/SSE handlers for browser QA."""
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        console = Console(root/'navigation.json', root/'auxiliary.json', root/'flow.json')
        config = console.mission.snapshot()
        for station in config['stations']:
            station['x'] //= 100; station['y'] //= 100
        config['grid'] = {k: [v//100 for v in values] for k, values in config['grid'].items()}
        config['route'] = [dict(station=1, hold=1), dict(station=0, hold=0)]
        config['dwell_ms'] = 0
        console.save_navigation(config)
        session = Session(console)
        session.cameras.bindings = {}
        session.camera.display = False
        console.connect('SIM')
        # Exercise production reply wakeups and snapshot throttling as well as
        # the serial worker. No bindings/display can open a physical camera.
        thread = threading.Thread(target=session.run, daemon=True); thread.start()
        server = ThreadingHTTPServer(('127.0.0.1', 18784), type('CompetitionHandler', (Handler,), {'session':session}))
        print('SIM competition fixture: http://127.0.0.1:18784', flush=True)
        try: server.serve_forever()
        except KeyboardInterrupt: pass
        finally:
            session.stopping.set(); console.wakeup.set()
            thread.join(2); console.disconnect(); server.server_close()
            if console.connection: console.connection.join(3)


if __name__ == '__main__': main()
