"""Standalone camera alignment against the real text protocol and SIM motion.

Images are synthetic and camera acquisition is controlled; no physical device
or serial port is opened. The controller itself and all ACK guards are real.
"""
import base64
import copy
import importlib.util
import io
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch, Mock

from host.camera_vision import DEFAULT as PRODUCTION_DEFAULT, VisionSettings, validated, detect
DEFAULT = copy.deepcopy(PRODUCTION_DEFAULT)
DEFAULT['detector']['colors']['red'].update(min_area=500, ranges=[[[0,80,46],[10,255,255]],[[170,80,46],[179,255,255]]])
DEFAULT['detector']['ring']['min_area'] = 500
from host.vision_alignment import validate_tracking, VisualPID
from host.protocol import encode, decode, Reply
from host.simulator import SimulatedBoard
from host.webapp import Console
from tests.test_mission import Link


class ConfigTests(unittest.TestCase):
    def test_existing_pid_settings_gain_mount_angle_without_overwriting_file(self):
        config = copy.deepcopy(DEFAULT)
        config['tracking'].pop('camera_yaw_deg')
        config['tracking'].update(kp=5, accel=3000)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'camera.json'
            path.write_text(json.dumps(config),encoding='utf-8')
            before = path.read_bytes()
            settings = VisionSettings(path)
            self.assertEqual(settings.snapshot()['tracking']['camera_yaw_deg'],90)
            self.assertEqual(settings.snapshot()['tracking']['kp'],5)
            self.assertEqual(settings.snapshot()['tracking']['accel'],3000)
            self.assertEqual(settings.snapshot()['target_offset_px'],[0,30])
            self.assertEqual(path.read_bytes(),before)
            saved = settings.snapshot(); saved['tracking']['camera_yaw_deg'] = -90
            settings.save(saved)
            self.assertEqual(VisionSettings(path).snapshot()['tracking']['camera_yaw_deg'],-90)

    def test_settings_and_decision_boundaries(self):
        for key, value in [('target_offset_px',[True,0]), ('kind',[]), ('min_area',float('nan'))]:
            c=copy.deepcopy(DEFAULT); c[key]=value
            with self.assertRaises(ValueError): validated(c)
        c=copy.deepcopy(DEFAULT['tracking'])
        for key,value in [('sps',0),('deadzone_px',-1),('reverse_x',1)]:
            invalid=dict(c);invalid[key]=value
            with self.assertRaises(ValueError):validate_tracking(invalid)

    def test_previous_camera_settings_load_without_confirmation_or_overwriting_file(self):
        old={'kind':'ring','target':'red','reference_offset_px':[0,30], 'min_area':600,'black_max':130,
             'calibration':{'verified':False,'matrix':[[0,0],[0,0]],'sps':150,'tolerance_px':5}}
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'camera.json';path.write_text(json.dumps(old),encoding='utf-8')
            settings=VisionSettings(path)
            self.assertEqual(settings.snapshot()['tracking']['sps'],150)
            self.assertEqual(settings.snapshot()['tracking']['deadzone_px'],5)
            self.assertEqual(settings.snapshot()['target_offset_px'],[0,30])
            self.assertEqual(json.loads(path.read_text(encoding='utf-8')),old)


@unittest.skipUnless(importlib.util.find_spec('cv2') and importlib.util.find_spec('numpy'), 'OpenCV/numpy unavailable')
class DetectionTests(unittest.TestCase):
    def test_b_blue_target_ignores_red_inside_and_blue_outside_crop(self):
        import cv2
        import numpy as np
        frame = np.full((480, 640, 3), 255, np.uint8)
        config = dict(copy.deepcopy(DEFAULT), target='blue', pickup_roi_pct=[35,35,30,30])
        # Match the calibrated blue HSV range and its minimum contour area.
        blue = tuple(int(v) for v in cv2.cvtColor(np.uint8([[[110,200,220]]]), cv2.COLOR_HSV2BGR)[0,0])
        cv2.rectangle(frame, (230, 180), (300, 290), (0, 0, 255), -1)
        cv2.rectangle(frame, (10, 10), (200, 200), blue, -1)
        self.assertFalse(detect(frame.copy(), config, 1.)['found'])
        cv2.circle(frame, (360, 250), 34, blue, -1)
        result = detect(frame.copy(), config, 2.)
        self.assertTrue(result['found'])
        self.assertEqual(result['target'], 'blue')
        self.assertLessEqual(max(abs(a-b) for a,b in zip(result['center'], [360,250])), 1)

    def test_b_crop_excludes_larger_distractor_and_returns_native_coordinates(self):
        import cv2
        import numpy as np
        frame = np.full((480, 640, 3), 255, np.uint8)
        cv2.rectangle(frame, (10, 10), (200, 200), (0, 0, 255), -1)
        cv2.rectangle(frame, (10, 350), (40, 380), (0, 0, 255), -1)
        config = dict(copy.deepcopy(DEFAULT), pickup_roi_pct=[35,35,30,30])
        negative = detect(frame.copy(), config, 1.)
        self.assertFalse(negative['found'])
        self.assertEqual(negative['roi'], [224,168,192,144])
        cv2.circle(frame, (350, 260), 24, (0, 0, 255), -1)
        annotated = frame.copy()
        result = detect(annotated, config, 2.)
        self.assertTrue(result['found'])
        self.assertLessEqual(max(abs(a-b) for a,b in zip(result['center'], [350,260])), 1)
        self.assertEqual(result['reference'], [320,240])
        self.assertEqual(result['control_point'], [350,290])
        self.assertTrue(np.array_equal(annotated[168,224], [255,255,0]))
        # Standalone recognition retains its full-frame search.
        full = detect(frame.copy(), copy.deepcopy(DEFAULT), 3.)
        self.assertLess(full['center'][0], 200)

    def test_live_digit_and_color_search_full_frame_despite_old_roi_environment(self):
        import itertools
        import sys
        import cv2
        import numpy as np
        source = importlib.import_module('工创赛视觉识别.色块识别.color_block_center')
        ring_source = importlib.import_module('工创赛视觉识别.黑色圆环识别与数字识别.black_ring')
        tools_path = str(Path(__file__).resolve().parents[1]/'tools')
        args = SimpleNamespace(mode='color',black_max=140,min_area=200,
                               confidence=.6,confidence_one=.3,interval=.03)
        def load_digit(runtime):
            runtime.detect_ring = ring_source.find_smallest_ring
            runtime.extract = lambda frame,ring:None
            runtime.target = Mock(score=0.,latency=0.)
            runtime.target.value.return_value = None
        with patch.object(sys,'path',[tools_path]+sys.path), patch.dict(sys.modules,{'color_block_center':source}), patch.dict('os.environ',{'NX_VISION_ROI_SIZE':'360','NX_VISION_ROI_BOTTOM_TRIM':'30'}):
            from nx_vision_live import VisionModes
            timestamps = itertools.count(1.,.02)
            with patch.object(VisionModes,'_load_digit',load_digit), patch('nx_vision_live.time.monotonic',side_effect=lambda:next(timestamps)), patch('host.vision_tuning.defaults',return_value=DEFAULT['detector']):
                runtime = VisionModes(args)
                for mode in ('color','digit'):
                    runtime.switch(mode)
                    self.assertEqual((runtime.roi_size,runtime.bottom_trim),(0,0))
                    for center in ((55,70),(585,70),(320,330)):
                        with self.subTest(mode=mode,center=center):
                            frame = np.full((360,640,3),255,np.uint8)
                            cv2.circle(frame,center,22,(0,0,255) if mode=='color' else (0,0,0),-1 if mode=='color' else 5)
                            runtime.process(frame)
                            report = runtime.report(frame)
                            self.assertEqual(report['roi'],(0,0,640,360))
                            self.assertIsNotNone(report['center'])
                            self.assertLessEqual(max(abs(a-b) for a,b in zip(report['center'],center)),1)
                            config = copy.deepcopy(DEFAULT)
                            config['kind'] = 'block' if mode=='color' else 'ring'
                            self.assertTrue(detect(frame.copy(),config,1.)['found'])
                runtime.close()

    def test_nx_monitor_recognizes_without_shutter_or_browser(self):
        import cv2
        import numpy as np
        from host.camera import worker
        frame=np.full((240,320,3),255,np.uint8)
        capture=Mock();capture.isOpened.return_value=True;capture.grab.return_value=True
        capture.get.side_effect=lambda key:{3:320,4:240,5:30}.get(key,0)
        def read():
            time.sleep(.02)
            return True,frame.copy()
        capture.read.side_effect=read
        displayed=threading.Event();gui_threads=[]
        def show(*args):
            gui_threads.append(threading.get_ident())
            if len(gui_threads)>=2:displayed.set()
        class Input:
            def __iter__(self):
                if not displayed.wait(3):raise RuntimeError('local monitor never displayed raw frames')
                return iter(())
        output=io.BytesIO()
        monitor=Mock();monitor.draw.side_effect=lambda raw,*args:raw
        with patch('host.camera_monitor.RecognitionMonitor',return_value=monitor),patch.dict('os.environ',{'DISPLAY':':0'}),patch('host.camera.present_local_window'),patch('cv2.VideoCapture',return_value=capture),patch('sys.stdin',SimpleNamespace(buffer=Input())),patch('sys.stdout',SimpleNamespace(buffer=output)),patch('cv2.namedWindow'),patch('cv2.setMouseCallback'),patch('cv2.setWindowProperty'),patch('cv2.imshow',side_effect=show),patch('cv2.waitKey',return_value=-1),patch('cv2.destroyAllWindows'),patch('cv2.imencode',side_effect=AssertionError('local display must not JPEG encode')):
            worker('/dev/synthetic',320,240,False,display=True)
        self.assertGreaterEqual(len(gui_threads),2)
        self.assertEqual(set(gui_threads),{threading.get_ident()})
        self.assertEqual(len(output.getvalue().splitlines()),1)
        capture.release.assert_called_once()
        self.assertGreater(monitor.process.call_count,0)

    def test_monitor_restores_digit_telemetry_and_uses_native_alignment_pixels(self):
        import cv2
        import numpy as np
        from host.camera_monitor import RecognitionMonitor
        monitor=RecognitionMonitor()
        monitor.runtime=Mock()
        monitor.runtime.select.return_value=dict(center=[496,352], digit='1', confidence=.94)
        monitor.runtime.observations=[]
        raw=np.zeros((720,1280,3),np.uint8)
        monitor.process(raw,DEFAULT)
        self.assertEqual(monitor.report['center'],[496,352])
        self.assertEqual(monitor.report['control_point'],[496,382])
        self.assertEqual([monitor.report['error_dx'],monitor.report['error_dy']],[-144,22])
        monitor.controls.compose=lambda image,mode:image
        monitor.controls.take_mode=lambda:None
        with patch('cv2.putText',wraps=cv2.putText) as text, patch('cv2.rectangle') as rectangle:
            monitor.draw(raw)
            labels=' '.join(call.args[1] for call in text.call_args_list)
        rectangle.assert_not_called()
        self.assertIn('Digit: 1  conf 0.94',labels)
        self.assertIn('Error X=-144  Y=22',labels)
        alignment=dict(monitor.report, center=[640,330], control_point=[640,360],
                       error_dx=0,error_dy=0,target='ring')
        with patch('cv2.putText',wraps=cv2.putText) as text:
            monitor.draw(raw,alignment,True)
            labels=' '.join(call.args[1] for call in text.call_args_list)
        self.assertIn('ALIGN / ring',labels)
        self.assertIn('Error X=0  Y=0',labels)
        self.assertNotIn('Digit:',labels)
        # Ending alignment/preview restores the existing live recognition display.
        with patch('cv2.putText',wraps=cv2.putText) as text:
            monitor.draw(raw,None,False)
            self.assertTrue(any('Digit: 1' in call.args[1] for call in text.call_args_list))
        monitor.report['captured_monotonic']-=2
        with patch('cv2.putText',wraps=cv2.putText) as text:
            monitor.draw(raw)
            self.assertTrue(any('Error X=--' in call.args[1] for call in text.call_args_list))

    def test_ring_masks_use_updated_neutral_black_definition(self):
        import cv2
        import numpy as np
        module = importlib.import_module('工创赛视觉识别.黑色圆环识别与数字识别.black_ring')
        frame = np.random.RandomState(42).randint(0, 256, (240, 320, 3)).astype(np.uint8)
        cv2.circle(frame, (160, 120), 45, (0, 0, 0), 8)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        channels = frame.astype(np.int16)
        neutral = (channels.max(axis=2)-channels.min(axis=2) <= 65) | (channels.max(axis=2) <= 60)
        for black_max in (1, 60, 140, 255):
            ceiling = (gray <= black_max) & neutral
            otsu, _ = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            global_mask = np.uint8(ceiling & (gray <= min(black_max, otsu)))*255
            local_mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                             cv2.THRESH_BINARY_INV, 41, 7)
            local_mask[~ceiling] = 0
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            legacy = [cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel) for m in (global_mask, local_mask)]
            for actual, expected in zip(module.make_masks(frame, black_max), legacy):
                np.testing.assert_array_equal(actual, expected)

    def test_worker_recognizes_continuous_raw_frames_without_shutter_or_jpeg(self):
        import cv2
        import numpy as np
        from host.camera import worker
        frame=np.full((240,320,3),255,np.uint8);cv2.circle(frame,(160,90),30,(0,0,255),-1)
        capture=Mock();capture.isOpened.return_value=True;capture.grab.return_value=True
        capture.get.side_effect=lambda key:{3:320,4:240,5:30}.get(key,0)
        def read():
            time.sleep(.01)
            return True,frame.copy()
        capture.read.side_effect=read
        ready=threading.Event()
        class Output(io.BytesIO):
            def write(self,data):
                result=super().write(data)
                if b'vision_only' in data:ready.set()
                return result
        class Input:
            def __iter__(self):
                yield ('TRACK '+json.dumps({'stream_id':7,'config':DEFAULT})+'\n').encode('ascii')
                if not ready.wait(3):raise RuntimeError('recognition never ran without SNAP')
                yield b'UNTRACK 7\n'
        output=Output()
        with patch('cv2.VideoCapture',return_value=capture),patch('sys.stdin',SimpleNamespace(buffer=Input())),patch('sys.stdout',SimpleNamespace(buffer=output)),patch('cv2.imencode',side_effect=AssertionError('recognition must not encode JPEG')) as encoder:
            worker('/dev/synthetic',320,240,False)
        output.seek(0);headers=[json.loads(line) for line in output.readlines()]
        detection=next(h for h in headers if h.get('vision_only'))
        self.assertTrue(detection['vision']['found'])
        self.assertEqual(detection['vision']['control_point'],[160,120])
        self.assertGreater(detection['vision']['frame_id'],0)
        self.assertEqual(detection['length'],0)
        encoder.assert_not_called();capture.release.assert_called_once()

    def test_updated_contour_selection_accepts_smallest_non_solid_shape(self):
        import cv2
        import numpy as np
        image = np.full((480,640,3),255,np.uint8)
        cv2.circle(image,(150,180),25,(0,0,255),5)
        cv2.circle(image,(400,300),45,(0,0,255),-1)
        report=detect(image,DEFAULT,123.)
        self.assertTrue(report['found']); self.assertEqual(report['center'],[150,180])

    def test_manual_shutter_does_not_run_recognition(self):
        import cv2
        import numpy as np
        from host.camera import worker
        frame = np.full((480,640,3),255,np.uint8); cv2.circle(frame,(200,180),45,(0,0,255),-1)
        capture = Mock()
        capture.isOpened.return_value=True; capture.grab.return_value=True
        capture.get.side_effect=lambda key:{3:640,4:480,5:30}.get(key,0)
        capture.read.side_effect=lambda:(True,frame.copy())
        incoming=io.BytesIO(b'SNAP\nSNAP\n')
        outgoing=io.BytesIO()
        with patch('cv2.VideoCapture',return_value=capture), patch('sys.stdin',SimpleNamespace(buffer=incoming)), patch('sys.stdout',SimpleNamespace(buffer=outgoing)), patch('host.camera_vision.detect',side_effect=AssertionError('photo must never detect')) as detector:
            worker('/dev/synthetic',640,480,False)
        outgoing.seek(0)
        ready=json.loads(outgoing.readline());self.assertEqual(ready['length'],0)
        detection=json.loads(outgoing.readline());jpeg=outgoing.read(detection['length'])
        self.assertTrue(jpeg.startswith(b'\xff\xd8'))
        self.assertIsNone(detection['vision'])
        self.assertGreater(detection['captured_at'],0)
        plain=json.loads(outgoing.readline());self.assertIsNone(plain['vision'])
        outgoing.read(plain['length']);self.assertEqual(outgoing.read(),b'')
        capture.release.assert_called_once()
        detector.assert_not_called()

    def test_requested_color_reference_and_ring_center(self):
        import cv2
        import numpy as np
        image = np.full((480,640,3),255,np.uint8)
        cv2.circle(image,(200,180),45,(0,0,255),-1)
        cv2.circle(image,(420,300),25,(255,0,0),-1)
        report=detect(image,DEFAULT,123.)
        self.assertTrue(report['found'])
        self.assertEqual(report['center'],[200,180])
        self.assertEqual(report['reference'],[320,240])
        self.assertEqual(report['control_point'],[200,210])
        self.assertEqual([report['error_dx'],report['error_dy']],[-120,-30])
        ring=np.full((480,640,3),255,np.uint8)
        cv2.circle(ring,(350,230),55,(0,0,0),8)
        config=copy.deepcopy(DEFAULT);config['kind']='ring'
        report=detect(ring,config,124.)
        self.assertTrue(report['found']);self.assertEqual(report['target'],'ring')
        self.assertLessEqual(abs(report['center'][0]-350),1)
        self.assertLessEqual(abs(report['center'][1]-230),1)
        centered=np.full((480,640,3),255,np.uint8)
        cv2.circle(centered,(320,210),55,(0,0,0),8)
        report=detect(centered,config,125.)
        self.assertEqual(VisualPID(config['tracking']).update(report['error_dx'],report['error_dy'],125.),(0,0))

    def test_nx_recognition_preview_has_no_photo_or_motion_dependency(self):
        with tempfile.TemporaryDirectory() as directory:
            console=Console(navigation_path=Path(directory)/'nav.json')
            hub=SimpleNamespace(snapshot=lambda:{'state':'ready'}, start_tracking=Mock(),stop_tracking=Mock(),
                                tracking_sample=Mock(return_value={'found':True,'center':[160,120]}))
            console.camera_alignment.hub=hub
            console.camera_alignment.recognize()
            deadline=time.monotonic()+10
            while console.camera_alignment.capturing and time.monotonic()<deadline:
                console.tick();time.sleep(.01)
            self.assertEqual(console.camera_alignment.state,'preview',console.camera_alignment.error)
            self.assertTrue(console.camera_alignment.vision['found'])
            self.assertFalse(console.armed)
            self.assertIsNone(console.connection)
            hub.start_tracking.assert_called_once();hub.stop_tracking.assert_called_once()
            self.assertIsNone(console.camera_alignment.preview_image)
            with self.assertRaisesRegex(ValueError,'网页照片仅供查看'):
                console.camera_alignment.recognize('data:image/jpeg;base64,AA==')
