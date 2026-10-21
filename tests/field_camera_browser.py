"""Isolated SIM + temporary settings for field/camera-one browser validation."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
from http.server import ThreadingHTTPServer

from host.webapp import Console
from host.webserver import Handler, Session


def main():
    """Run a real browser with synthetic video; never open serial/camera devices."""
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)
        console = Console(path/'nav.json', path/'aux.json', path/'flow.json')
        console.connect('SIM', True)
        session = Session(console)
        # A synthetic NX raw-frame source exercises the real recognition API.
        # Manual browser photos remain a separate flow and cannot reach it.
        from types import SimpleNamespace
        import copy
        import time
        from host.camera_vision import detect
        class SyntheticNxHub:
            def __init__(self):
                self.config = None
                self.frame_id = 0
            def snapshot(self):
                return {'state':'ready','width':640,'height':480,'device':'/dev/synthetic','message':'Synthetic NX ready'}
            def start_tracking(self, config):
                self.config = copy.deepcopy(config)
            def stop_tracking(self):
                self.config = None
            def tracking_sample(self, after_monotonic, after_frame_id=0):
                import cv2
                import numpy as np
                image = np.full((480,640,3),255,np.uint8)
                cv2.circle(image,(250,210),45,(85,0,255),-1)
                self.frame_id += 1
                report = detect(image,self.config,time.time())
                report.update(frame_id=self.frame_id,captured_monotonic=time.monotonic())
                return report
            def tune(self, config):
                import base64
                import cv2
                import numpy as np
                from host.vision_tuning import preview
                image = np.full((480,640,3),255,np.uint8)
                cv2.rectangle(image,(180,130),(300,230),(85,0,255),-1)
                report, rendered, mask = preview(image,config,time.time())
                def encoded(frame):
                    return 'data:image/jpeg;base64,'+base64.b64encode(cv2.imencode('.jpg',frame)[1]).decode()
                return dict(report=report,image=encoded(rendered),mask=encoded(mask))
            def stop(self): pass
            def capture(self): raise AssertionError('NX recognition must not request a photo')
        original_listing = session.cameras.listing
        session.cameras.listing = lambda slot: ({'supported':True,'binding':None,'bound_device':None,'devices':[
            {'id':'/dev/synthetic','label':'Synthetic NX','resolutions':['640x480']}]} if slot=='1' else original_listing(slot))
        session.cameras.hubs['1'] = SyntheticNxHub()
        console.camera_alignment.hub = session.cameras.hubs['1']
        handler = type('FieldCameraHandler', (Handler,), {'session': session})
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
        server.daemon_threads = True
        serving = threading.Thread(target=server.serve_forever, daemon=True)
        pumping = threading.Thread(target=session.run, daemon=True)
        serving.start(); pumping.start()
        try:
            environment = dict(os.environ, NAV_URL='http://127.0.0.1:{}/'.format(server.server_port))
            script = os.environ.get('CAMERA_BROWSER_TEST', str(Path(__file__).with_suffix('.cjs')))
            return subprocess.run(['node', script], cwd=root, env=environment).returncode
        finally:
            session.stopping.set(); pumping.join(timeout=2)
            console.disconnect(); session.cameras.close()
            server.shutdown(); server.server_close(); serving.join(timeout=2)


if __name__ == '__main__':
    sys.exit(main())
