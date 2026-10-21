"""Isolated SIM-only web fixture; temporary configs and no camera startup.

Run with python -m tests.pick_place_browser_server, then the matching Playwright
script. Never point the browser motion test at a physical-board web service.
"""
import tempfile
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace

from host.webapp import Console
from host.webserver import Handler, Session
from tests.test_task_code import FakeCameraControl


def main():
    """Serve a local simulated board until interrupted, then release resources."""
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        console = Console(root/'nav.json', root/'aux.json', root/'flow.json')
        saved = console.aux_settings.snapshot()
        saved['5']['max'] = 9400
        saved['6'].update(min=-2950, max=0, presets=[0, -2950])
        console.aux_settings.save(saved)
        session = Session(console)
        console.task_cameras = cameras = FakeCameraControl()
        alignment = console.camera_alignment
        alignment.hub = SimpleNamespace(snapshot=lambda:cameras.slots.states['1'],
            start_tracking=lambda config:None, stop_tracking=lambda:None)
        alignment.capture = lambda image=None:setattr(alignment, 'capturing', True)
        console.connect('SIM')
        stopped = threading.Event()

        def pump():
            """Mirror the normal serial/SSE loop without opening saved cameras."""
            opening_at = None
            while not stopped.wait(.05):
                if cameras.snapshot()['state'] == 'opening':
                    if opening_at is None: opening_at = time.monotonic()
                    if time.monotonic()-opening_at >= 1: cameras.finish()
                else:
                    opening_at = None
                console.last_client = time.monotonic()
                for event in console.drain():session.broadcast(event)
                console.tick()
                session.broadcast({'type':'snapshot', 'frames':console.hello()})

        thread = threading.Thread(target=pump, daemon=True)
        thread.start()
        server = ThreadingHTTPServer(('127.0.0.1',18783),
            type('PickPlaceBrowserHandler', (Handler,), {'session':session}))
        print('SIM-only pick/place fixture: http://127.0.0.1:18783', flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            stopped.set(); thread.join(2); console.disconnect(); server.server_close()
            if console.connection:console.connection.join(3)


if __name__ == '__main__':
    main()
