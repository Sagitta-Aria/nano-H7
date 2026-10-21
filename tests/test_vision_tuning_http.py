"""Real HTTP routes for tuning persistently, previewing and rejecting unsafe writes."""
import copy
import json
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from host.webapp import Console
from host.webserver import Handler, Session


class TuningHttpTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory()
        root=Path(self.directory.name)
        self.console=Console(root/'nav.json', root/'aux.json', root/'flow.json')
        self.session=Session(self.console)
        self.session.join('writer')
        self.hub=Mock()
        self.hub.tune.return_value=dict(report={'found':False}, image='image', mask='mask')
        self.session.cameras.hubs['1']=self.hub
        self.server=ThreadingHTTPServer(('127.0.0.1',0),type('TuningHandler',(Handler,),{'session':self.session}))
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown(); self.server.server_close(); self.thread.join(2)
        self.directory.cleanup()

    def call(self, action, payload=None, token='writer'):
        data=None if payload is None else json.dumps(payload).encode()
        request=urllib.request.Request('http://127.0.0.1:{}/api/camera/vision/{}'.format(self.server.server_port,action),data=data)
        request.add_header('X-Session',token)
        try:
            with urllib.request.urlopen(request,timeout=3) as response:
                return response.status,json.load(response)
        except urllib.error.HTTPError as error:
            return error.code,json.load(error)

    def test_save_reload_and_invalid_save_preserve_previous_parameters(self):
        _,state=self.call('state'); config=state['config']
        config['detector']['colors']['black']['min_area']=12000
        config['detector']['ocr'] = {'confidence': .55, 'confidence_one': .25}
        status,saved=self.call('config',{'config':config})
        self.assertEqual(status,200)
        self.assertEqual(self.call('state')[1]['config'],saved['config'])
        self.assertEqual(saved['config']['detector']['ocr'], config['detector']['ocr'])
        config['detector']['ring']['canny_low']=255
        self.assertEqual(self.call('config',{'config':config})[0],400)
        self.assertEqual(self.call('state')[1]['config'],saved['config'])
        self.assertEqual(self.call('defaults')[1]['detector']['colors']['black']['min_area'],9034)
        self.hub.tune.assert_not_called()

    def test_preview_does_not_persist_and_releases_busy_flag_on_error(self):
        _,state=self.call('state'); config=copy.deepcopy(state['config'])
        config['detector']['ring']['min_area']=2000
        self.assertEqual(self.call('tune',{'config':config})[0],200)
        self.assertEqual(self.call('state')[1]['config'],state['config'])
        self.hub.tune.assert_called_once_with(config)
        self.hub.tune.side_effect=ValueError('synthetic timeout')
        self.assertEqual(self.call('tune',{'config':config})[0],400)
        self.assertFalse(self.console.camera_alignment.capturing)
        self.assertFalse(self.console.camera_alignment.tuning)
        self.assertIsNone(self.console.connection)

    def test_readonly_and_running_task_reject_tuning(self):
        config=self.call('state')[1]['config']
        for action in ('config','tune'):
            self.assertEqual(self.call(action,{'config':config},token='observer')[0],409)
        with patch.object(self.console,'task',SimpleNamespace(active=True,uses_task_code=False)):
            for action in ('config','tune'):
                self.assertEqual(self.call(action,{'config':config})[0],400)
        self.hub.tune.assert_not_called()

    def test_debug_crop_reaches_worker_without_saving_or_allowing_invalid_bounds(self):
        config=self.call('state')[1]['config']
        config['kind']='block'
        self.assertEqual(self.call('tune',dict(config=config,preview_roi_pct=[15,15,70,70]))[0],200)
        self.hub.tune.assert_called_once_with(dict(config,preview_roi_pct=[15,15,70,70]))
        self.assertNotIn('preview_roi_pct',self.call('state')[1]['config'])
        self.assertEqual(self.call('tune',dict(config=config,preview_roi_pct=[40,15,70,70]))[0],400)
        self.assertFalse(self.console.camera_alignment.capturing)


if __name__ == '__main__':
    unittest.main()
