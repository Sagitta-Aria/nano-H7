"""Real QR pixels, dedicated worker isolation and stale task-result checks."""
import importlib.util
import subprocess
import sys
import time
import unittest
from unittest.mock import patch

from host.camera import CameraHub
from host.camera_qr import TaskQR


@unittest.skipUnless(importlib.util.find_spec('cv2'), 'OpenCV lives in the NX vision environment')
class QRTests(unittest.TestCase):
    def setUp(self):
        import cv2
        self.cv2 = cv2
        self.decoder = TaskQR()

    def pixels(self, text):
        """Render a real QR matrix with a quiet border at camera-like scale."""
        qr = self.cv2.QRCodeEncoder_create().encode(text)
        qr = self.cv2.copyMakeBorder(qr, 4, 4, 4, 4, self.cv2.BORDER_CONSTANT, value=255)
        return self.cv2.cvtColor(self.cv2.resize(qr, None, fx=8, fy=8, interpolation=self.cv2.INTER_NEAREST), self.cv2.COLOR_GRAY2BGR)

    def test_original_group_parser_and_leading_zeros(self):
        result = self.decoder.process(self.pixels('012+321+456+054'), 10)
        self.assertEqual(result['code'], '012+321+456+054')
        self.assertEqual(result['groups'], ['012', '321', '456', '054'])
        self.assertEqual(result['digits'], [0, 1, 2, 3, 2, 1, 4, 5, 6, 0, 5, 4])
        self.assertEqual(result['last_seen_at'], 10)

    def test_legacy_task_codes(self):
        for text in ('123321', '1,2,3,3,2,1'):
            self.assertEqual(self.decoder.process(self.pixels(text), 10)['code'], '123+321')

    def test_invalid_and_missing_frames_do_not_become_current_task(self):
        self.decoder.process(self.pixels('123+321'), 10)
        invalid = self.decoder.process(self.pixels('https://example.com'), 11)
        self.assertEqual(invalid['status'], 'invalid')
        self.assertIsNone(invalid['code'])
        blank = self.pixels('123+321')
        blank[:] = 255
        lost = self.decoder.process(blank, 12)
        self.assertEqual(lost['status'], 'searching')
        self.assertIsNone(lost['code'])
        self.assertEqual(lost['last_code'], '123+321')
        self.assertEqual(lost['last_seen_at'], 10)

    def test_live_worker_without_shutter_or_tracking(self):
        # Actual worker, raw acquisition, OpenCV decode and pipe protocol; only
        # the physical V4L2 driver is replaced with generated camera frames.
        script = '''
import cv2,time
from host.camera import worker
qr=cv2.QRCodeEncoder_create().encode('012+321+456+054')
qr=cv2.copyMakeBorder(qr,4,4,4,4,cv2.BORDER_CONSTANT,value=255)
frame=cv2.cvtColor(cv2.resize(qr,(480,480),interpolation=cv2.INTER_NEAREST),cv2.COLOR_GRAY2BGR)
class Capture:
    def isOpened(self):return True
    def set(self,*args):return True
    def get(self,key):return 30 if key==cv2.CAP_PROP_FPS else 480
    def grab(self):return True
    def read(self):time.sleep(.03);return True,frame
    def release(self):pass
cv2.VideoCapture=lambda *args:Capture()
worker('/dev/video0',480,480,False,qr=True)
'''
        hub = CameraHub(qr=True)
        self.addCleanup(hub.stop)
        popen = subprocess.Popen
        found = [dict(id='/dev/video0', label='QR', resolutions=['480x480'], mjpeg=False)]
        with patch('host.camera.devices', return_value=found), patch('host.camera.subprocess.Popen',
                side_effect=lambda args, **kw: popen([sys.executable, '-u', '-c', script], **kw)):
            hub.start('/dev/video0', '480x480')
        deadline = time.monotonic() + 10
        while not hub.snapshot().get('qr', {}).get('code') and time.monotonic() < deadline:
            time.sleep(.05)
        result = hub.snapshot()
        self.assertEqual(result['qr']['code'], '012+321+456+054', result)
        self.assertEqual(hub.sequence, 0)
        self.assertIsNone(hub.image())
        self.assertIsNone(hub.tracking_id)
        with self.assertRaisesRegex(ValueError, '仅用于二维码'):
            hub.start_tracking({'kind': 'ring'})
        hub.capture()
        self.assertEqual(hub.sequence, 1)
        self.assertEqual(hub.snapshot()['qr']['last_code'], '012+321+456+054')
        hub.stop()
        self.assertNotIn('qr', hub.snapshot())


class QRStateTests(unittest.TestCase):
    def test_stale_or_disconnected_result_cannot_claim_current_detection(self):
        hub = CameraHub(qr=True)
        for state, captured_at in [('ready', time.time()-4), ('error', time.time())]:
            hub.frame = dict(state=state, qr=dict(status='valid', code='012+321',
                groups=['012', '321'], digits=[0, 1, 2, 3, 2, 1], captured_at=captured_at,
                last_code='012+321'))
            snapshot = hub.snapshot()
            self.assertEqual(snapshot['qr']['status'], 'stale')
            self.assertIsNone(snapshot['qr']['code'])
            self.assertEqual(snapshot['qr']['last_code'], '012+321')
            self.assertEqual(hub.frame['qr']['status'], 'valid')


if __name__ == '__main__':
    unittest.main()
