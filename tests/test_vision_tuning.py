"""Updated detectors, persisted tuning and camera-worker diagnostics without hardware."""
import copy
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None

from host.camera_vision import DEFAULT, VisionSettings, detect, validated
from host.vision_tuning import defaults, validate, preview


@unittest.skipUnless(cv2 is not None, 'OpenCV/numpy unavailable')
class TuningTests(unittest.TestCase):
    def test_debug_crop_uses_actual_target_thresholds_and_native_coordinates(self):
        from host.vision_tuning import legacy_settings
        config = copy.deepcopy(DEFAULT)
        config['target'] = 'red'
        config['detector'] = legacy_settings(500, 140)
        config['preview_roi_pct'] = [25,25,50,50]
        frame = np.full((480,640,3),255,np.uint8)
        cv2.rectangle(frame,(10,10),(65,65),(0,0,255),-1)
        cv2.rectangle(frame,(200,130),(280,220),(0,0,0),-1)
        report, _, _ = preview(frame.copy(), config, 1.)
        self.assertFalse(report['found'])
        cv2.rectangle(frame,(320,200),(380,260),(0,0,255),-1)
        report, image, mask = preview(frame.copy(), config, 2.)
        self.assertEqual(report['target'], 'red')
        self.assertEqual(report['center'], [350,230])
        self.assertEqual(report['roi'], [160,120,320,240])
        self.assertEqual(report['color_settings'], config['detector']['colors']['red'])
        self.assertEqual(mask.shape, (240,320))
        self.assertTrue(np.array_equal(image[120,160], [255,255,0]))
        config['detector']['colors']['red']['ranges'] = [[[12,80,46],[12,255,255]]]
        self.assertFalse(preview(frame.copy(), config, 3.)[0]['found'])
        with self.assertRaises(ValueError): detect(frame, config, 4.)

    def test_imported_calibration_and_invalid_values(self):
        settings = defaults()
        self.assertEqual(settings['ocr'], {'confidence': .6, 'confidence_one': .3})
        self.assertEqual([settings['colors'][c]['min_area'] for c in
                          ('red','yellow','blue','green','black','light_blue')],
                         [3081,2037,3133,2037,9034,1984])
        self.assertEqual(settings['colors']['red']['ranges'][0][0][0], 12)
        self.assertTrue(all(len(item['ranges']) == 1 for item in settings['colors'].values()))
        self.assertEqual(validate(settings), settings)
        invalid = []
        for key, value in [('canny_low',120),('detect_width',159),('nearby_distance',True),
                           ('min_coverage',float('nan')),('shape_tolerance',.31),('min_area',0)]:
            item=defaults(); item['ring'][key]=value; invalid.append(item)
        item=defaults(); item['colors']['red']['ranges'][0][0][0]=180; invalid.append(item)
        item=defaults(); item['colors']['blue']['ranges'][0][0][1]=255; invalid.append(item)
        for item in invalid:
            with self.assertRaises(ValueError): validate(item)

    def test_digit_confidence_parameters_validate_and_migrate(self):
        settings = defaults()
        legacy = copy.deepcopy(settings)
        legacy.pop('ocr')
        self.assertEqual(validate(legacy)['ocr'], settings['ocr'])
        self.assertNotIn('ocr', legacy)
        settings['ocr']['confidence_one'] = .22
        self.assertEqual(validate(settings)['ocr']['confidence_one'], .22)
        for key, value in (('confidence', 0), ('confidence_one', 1.01),
                           ('confidence', float('nan'))):
            invalid = defaults(); invalid['ocr'][key] = value
            with self.assertRaises(ValueError): validate(invalid)

    def test_ocr_uses_saved_thresholds(self):
        from host.ring_perception import RingOCR
        reader = object.__new__(RingOCR)
        reader.thresholds = {'confidence': .41, 'confidence_one': .19}
        recognition = Mock(return_value=('1', .22, 1))
        reader.algorithm = SimpleNamespace(extract_ring_interior=lambda frame, ring: 'crop',
                                          recognize=recognition)
        reader.engine = object()
        reader.observations = []
        result = reader.select(None, [dict(id=1, center=(1, 1))], None)
        self.assertEqual(result['digit'], '1')
        recognition.assert_called_once_with(reader.engine, 'crop', .41, .19)

    def test_saved_thresholds_reach_control_and_clear_preview_votes(self):
        from host.ring_perception import PreviewOCR
        config = copy.deepcopy(DEFAULT)
        config['kind'] = 'ring'
        config['detector']['ocr'].update(confidence=.55, confidence_one=.25)
        reader = Mock()
        reader.observations = []
        reader.select.return_value = None
        detect(np.full((240,320,3),255,np.uint8), config, 1, ocr=reader,
               ring_detector=lambda *args: ([], None))
        reader.configure.assert_called_once_with(config['detector']['ocr'])
        preview = object.__new__(PreviewOCR)
        preview.reader = Mock()
        preview.reader.thresholds = {'confidence': .6, 'confidence_one': .3}
        preview.worker = Mock(args=SimpleNamespace(confidence=.6, confidence_one=.3))
        old_worker = preview.worker
        preview.observations = [dict(digit='1')]
        preview.configure(config['detector']['ocr'])
        old_worker.close.assert_called_once()
        preview.reader.configure.assert_called_once_with(config['detector']['ocr'])
        self.assertEqual(old_worker.args.confidence_one, .25)
        self.assertEqual(preview.observations, [])
        self.assertIsNot(preview.worker, old_worker)

    def test_old_second_interval_is_ignored_in_preview_detection_and_save(self):
        """An old config must not detect pixels matching only its second interval."""
        for name in ('red', 'blue', 'light_blue'):
            with self.subTest(color=name), tempfile.TemporaryDirectory() as folder:
                config = copy.deepcopy(DEFAULT)
                config['target'] = name
                config['detector']['colors'][name].update(min_area=500, ranges=[
                    [[40, 100, 100], [80, 255, 255]],
                    [[0, 100, 100], [10, 255, 255]],
                ])
                original = copy.deepcopy(config)
                path = Path(folder) / 'camera.json'
                path.write_text(json.dumps(config), encoding='utf-8')
                before = path.read_bytes()
                store = VisionSettings(path)
                self.assertEqual(path.read_bytes(), before)
                normalized = store.snapshot()
                self.assertEqual(normalized['detector']['colors'][name]['ranges'],
                                 config['detector']['colors'][name]['ranges'][:1])
                frame = np.full((240, 320, 3), 255, np.uint8)
                cv2.rectangle(frame, (40, 40), (140, 140), (0, 0, 255), -1)
                self.assertFalse(detect(frame.copy(), config, 1)['found'])
                report, _, mask = preview(frame.copy(), config, 1)
                self.assertFalse(report['found'])
                self.assertEqual(cv2.countNonZero(mask), 0)
                cv2.rectangle(frame, (40, 40), (140, 140), (0, 255, 0), -1)
                self.assertTrue(detect(frame, config, 2)['found'])
                store.save(config)
                self.assertEqual(json.loads(path.read_text(encoding='utf-8')), normalized)
                self.assertEqual(VisionSettings(path).snapshot(), normalized)
                self.assertEqual(config, original)

    def test_migration_atomic_save_roundtrip_and_rejected_save(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'camera.json'; old=copy.deepcopy(DEFAULT); old.pop('detector')
            old.update(min_area=678,black_max=123); old['tracking']['kp']=7
            path.write_text(json.dumps(old),encoding='utf-8'); original=path.read_bytes()
            store=VisionSettings(path); value=store.snapshot()
            self.assertEqual(path.read_bytes(),original)
            self.assertEqual(value['detector']['ring']['min_area'],678)
            self.assertEqual(value['detector']['colors']['red']['min_area'],678)
            self.assertEqual(value['detector']['colors']['red']['ranges'][0],[[0,80,46],[10,255,255]])
            self.assertEqual(value['tracking']['kp'],7)
            value['detector']['colors']['red']['min_area']=1234
            store.save(value); saved=path.read_bytes()
            self.assertEqual(VisionSettings(path).snapshot(),value)
            value['detector']['ring']['canny_low']=255
            with self.assertRaises(ValueError): store.save(value)
            self.assertEqual(path.read_bytes(),saved)

    def test_color_threshold_changes_actual_detection_and_preview(self):
        frame=np.full((480,640,3),255,np.uint8)
        cv2.rectangle(frame,(200,170),(310,240),(0,0,255),-1)
        config=copy.deepcopy(DEFAULT)
        self.assertFalse(detect(frame.copy(),config,1)['found'])  # Supplied red H12 excludes H0.
        config['detector']['colors']['red']['ranges'][0][0][0]=0
        config['detector']['colors']['red']['ranges'][0][1][0]=10
        report, annotated, mask=preview(frame.copy(),config,2)
        actual=detect(frame.copy(),config,2)
        self.assertEqual(report.pop('color_settings'), config['detector']['colors']['red'])
        self.assertEqual(report,actual)
        self.assertEqual(report['center'],[255,205])
        self.assertEqual(mask.shape,(480,640)); self.assertEqual(annotated.shape,frame.shape)
        self.assertGreater(cv2.countNonZero(mask),7000)
        config['detector']['colors']['red']['min_area']=10000
        self.assertFalse(detect(frame.copy(),config,3)['found'])

    def test_black_selects_largest_and_other_color_smallest_without_roundness(self):
        config=copy.deepcopy(DEFAULT); config['target']='black'
        frame=np.full((480,640,3),255,np.uint8)
        cv2.rectangle(frame,(20,20),(150,100),(0,0,0),-1)
        cv2.rectangle(frame,(300,200),(500,350),(0,0,0),-1)
        self.assertEqual(detect(frame.copy(),config,1)['center'],[400,275])
        config['target']='yellow'; frame[:]=255
        cv2.rectangle(frame,(20,20),(150,100),(0,255,255),-1)
        cv2.rectangle(frame,(300,200),(500,350),(0,255,255),-1)
        self.assertEqual(detect(frame,config,1)['center'],[85,60])

    def test_ring_distance_area_scaling_and_solid_rejection(self):
        config=copy.deepcopy(DEFAULT); config['kind']='ring'
        config['detector']['ring'].update(min_area=500,detect_width=0)
        frame=np.full((480,640,3),255,np.uint8)
        cv2.circle(frame,(200,200),25,(0,0,0),5)
        cv2.circle(frame,(290,200),35,(0,0,0),5)
        self.assertEqual(len(detect(frame.copy(),config,1)['rings']),1)
        config['detector']['ring']['nearby_distance']=0
        self.assertEqual(len(detect(frame.copy(),config,1)['rings']),2)
        large=np.full((720,1280,3),255,np.uint8)
        for center in ((320,360),(920,360)):
            cv2.circle(large,center,145,(0,0,0),16)
            cv2.circle(large,center,105,(0,0,0),10)
        config['detector']['ring'].update(min_area=50000,detect_width=960)
        report=detect(large.copy(),config,1)
        self.assertEqual(report['rings'],[[320,360],[920,360]])
        config['detector']['ring']['detect_width']=0
        self.assertEqual(detect(large,config,1)['rings'],report['rings'])
        blank=np.full((720,1280,3),255,np.uint8); cv2.circle(blank,(640,360),160,(0,0,0),-1)
        self.assertFalse(detect(blank,config,1)['found'])

    def test_diagnostic_worker_uses_existing_camera_and_keeps_photo_channel_separate(self):
        from host.camera import worker
        frame=np.full((480,640,3),255,np.uint8)
        cv2.rectangle(frame,(150,150),(300,250),(0,255,255),-1)
        config=copy.deepcopy(DEFAULT); config['target']='yellow'
        capture=Mock(); capture.isOpened.return_value=True;capture.grab.return_value=True
        capture.get.side_effect=lambda key:{3:640,4:480,5:30}.get(key,0)
        capture.read.side_effect=lambda:(True,frame.copy())
        command=('TUNE '+json.dumps(dict(request_id=41,config=config))+'\nSNAP\n').encode()
        output=io.BytesIO()
        with patch('cv2.VideoCapture',return_value=capture) as opening, \
             patch('sys.stdin',SimpleNamespace(buffer=io.BytesIO(command))), \
             patch('sys.stdout',SimpleNamespace(buffer=output)):
            worker('/dev/synthetic',640,480,True)
        opening.assert_called_once(); output.seek(0)
        self.assertEqual(json.loads(output.readline())['length'],0)
        header=json.loads(output.readline()); self.assertEqual(header['request_id'],41)
        result=json.loads(output.read(header['length']))
        self.assertTrue(result['report']['found'])
        self.assertTrue(result['image'].startswith('data:image/jpeg;base64,'))
        self.assertTrue(result['mask'].startswith('data:image/jpeg;base64,'))
        photo=json.loads(output.readline()); self.assertIsNone(photo['vision'])
        self.assertTrue(output.read(photo['length']).startswith(b'\xff\xd8'))
        self.assertEqual(capture.set.call_args_list[-1].args[0],cv2.CAP_PROP_FOURCC)

    def test_new_engine_gpu_adapter_preserves_onnx_call_contract(self):
        import importlib.util
        spec=importlib.util.spec_from_file_location('trt_adapter_test',Path(__file__).resolve().parents[1]/'tools/nx_vision_trt.py')
        module=importlib.util.module_from_spec(spec)
        with patch.dict('sys.modules',{'tensorrt':Mock()}): spec.loader.exec_module(module)
        accelerate=module.accelerate
        session=Mock(return_value=[np.array([123])])
        engine=SimpleNamespace(session=object())
        with patch.object(module,'TensorRTSession',return_value=session):
            self.assertIs(accelerate(engine),engine)
        tensor=np.zeros((1,3,48,32),np.float32)
        self.assertEqual(engine.session.run(None,{'input':tensor})[0][0],123)
        self.assertIs(session.call_args.args[0],tensor)
        self.assertIs(engine.nx_trt_session,session)


if __name__ == '__main__':
    unittest.main()
