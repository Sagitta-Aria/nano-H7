"""Real pipe/process checks for on-demand capture, without a physical camera."""
import json
import subprocess
import sys
import time
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from host.camera import CameraHub, CameraSlots, csi_identity

WORKER = """
import sys,json
print(json.dumps({'length':0,'width':640,'height':480}),flush=True)
for command in sys.stdin.buffer:
    data=b'one-photo'
    sys.stdout.buffer.write(json.dumps({'length':len(data),'width':640,'height':480}).encode()+b'\\n'+data)
    sys.stdout.buffer.flush()
"""


class CameraTests(unittest.TestCase):
    def setUp(self):
        self.hub = CameraHub()
        actual_popen = subprocess.Popen
        self.device_patch = patch('host.camera.devices', return_value=[{
            'id':'/dev/video0','label':'USB Camera','resolutions':['640x480','1280x720'],'mjpeg':True}])
        self.process_patch = patch('host.camera.subprocess.Popen', side_effect=lambda command, **kwargs:
                                  actual_popen([sys.executable,'-u','-c',WORKER], **kwargs))
        self.device_mock = self.device_patch.start()
        self.process_mock = self.process_patch.start()

    def tearDown(self):
        self.hub.stop()
        self.device_patch.stop()
        self.process_patch.stop()

    def open(self):
        self.hub.start('/dev/video0','640x480')
        deadline = time.monotonic() + 3
        while self.hub.snapshot()['state'] == 'opening' and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertEqual(self.hub.snapshot()['state'], 'ready')

    def test_idle_never_produces_photos(self):
        self.open()
        time.sleep(.05)
        self.assertIsNone(self.hub.image())
        self.assertEqual(self.hub.sequence, 0)

    def test_shutter_produces_one_frame_and_then_stays_idle(self):
        self.open()
        state = self.hub.capture()
        self.assertEqual(state['sequence'], 1)
        self.assertEqual(self.hub.image(), b'one-photo')
        self.assertGreater(state['taken_at'], 0)
        time.sleep(.05)
        self.assertEqual(self.hub.sequence, 1)
        self.assertEqual(self.hub.capture()['sequence'], 2)

    def test_unsupported_mode_keeps_device_open(self):
        self.open()
        process = self.hub.process
        with self.assertRaisesRegex(ValueError, '分辨率'):
            self.hub.start('/dev/video0','3840x2160')
        self.assertIs(self.hub.process, process)
        self.assertEqual(self.hub.capture()['sequence'], 1)

    def test_missing_device_and_closed_shutter_are_refused(self):
        with self.assertRaises(ValueError):
            self.hub.start('/dev/video99','640x480')
        with self.assertRaises(ValueError):
            self.hub.capture()

    def test_stop_releases_process_and_photo(self):
        self.open()
        process = self.hub.process
        self.hub.capture()
        self.hub.stop()
        self.assertIsNotNone(process.poll())
        self.assertEqual(self.hub.snapshot()['state'], 'idle')
        self.assertIsNone(self.hub.image())
        with self.assertRaises(ValueError):
            self.hub.capture()


class CameraSlotTests(CameraTests):
    def setUp(self):
        super().setUp()
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / 'cameras.json'
        self.slots = CameraSlots(self.path)
        self.found = [dict(id='/dev/video0', label='USB Camera', stable_id='/dev/v4l/by-id/USB-A',
                           aliases=['/dev/v4l/by-id/USB-A'], physical_id='usb-A',
                           resolutions=['640x480'], mjpeg=True, kind='usb'),
                      dict(id='/dev/video2', label='vi-output, imx219 9-0010',
                           stable_id='csi:/sensor/port0', aliases=['csi:/sensor/port0'],
                           physical_id='csi:/sensor/port0', resolutions=['640x480'],
                           mjpeg=False, kind='csi', sensor_id=0)]
        self.device_mock.return_value = self.found

    def tearDown(self):
        self.slots.close()
        super().tearDown()
        self.directory.cleanup()

    def test_binding_survives_restart_and_video_renumbering(self):
        self.slots.bind('2', '/dev/video2')
        restored = CameraSlots(self.path)
        self.found[1]['id'] = '/dev/video8'
        self.assertEqual(restored.listing('2')['bound_device'], '/dev/video8')
        restored.start('2', None, '640x480')
        self.assertEqual(restored.hubs['2'].snapshot()['device'], '/dev/video8')
        command = self.process_mock.call_args.args[0]
        self.assertIn('--sensor-id', command)
        restored.close()

    def test_missing_bound_sensor_never_opens_replacement(self):
        self.slots.bind('2', '/dev/video2')
        self.found.pop()
        self.found[0]['id'] = '/dev/video2'
        with self.assertRaisesRegex(ValueError, '未连接'):
            self.slots.start('2', '/dev/video2', '640x480')
        self.assertFalse(self.process_mock.called)
        self.assertIsNone(self.slots.listing('2')['bound_device'])

    def test_same_camera_cannot_belong_to_both_slots(self):
        self.slots.bind('2', '/dev/video2')
        with self.assertRaises(ValueError):
            self.slots.bind('1', '/dev/video2')
        with self.assertRaises(ValueError):
            self.slots.start('1', '/dev/video2', '640x480')

    def test_capture_and_stop_are_independent(self):
        for slot, device in (('1', '/dev/video0'), ('2', '/dev/video2')):
            self.slots.start(slot, device, '640x480')
            deadline = time.monotonic() + 3
            while self.slots.hubs[slot].snapshot()['state'] == 'opening' and time.monotonic() < deadline:
                time.sleep(.01)
        self.slots.hubs['2'].capture()
        self.assertIsNone(self.slots.hubs['1'].image())
        self.slots.stop('1')
        self.assertEqual(self.slots.hubs['2'].snapshot()['state'], 'ready')
        self.assertEqual(self.slots.hubs['2'].capture()['sequence'], 2)

    def test_unbound_camera_cannot_be_opened_twice(self):
        self.slots.start('1', '/dev/video0', '640x480')
        with self.assertRaises(ValueError):
            self.slots.start('2', '/dev/video0', '640x480')

    def test_csi_identity_uses_port_path(self):
        root = Path(self.directory.name) / 'modules'
        driver = root / 'module0' / 'drivernode0'
        driver.mkdir(parents=True)
        (driver / 'devname').write_bytes(b'imx219 9-0010\0')
        (driver / 'proc-device-tree').write_bytes(b'/proc/device-tree/sensor@10\0')
        identity = csi_identity('vi-output, imx219 9-0010', root)
        self.assertEqual(identity['stable_id'], 'csi:/proc/device-tree/sensor@10')
        self.assertIsNone(csi_identity('USB Camera', root))


if __name__ == '__main__':
    unittest.main()
