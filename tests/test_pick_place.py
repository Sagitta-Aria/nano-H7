"""Execute real pick/place orchestration against the deterministic serial board.

These tests never access COM ports or motors. They verify command order, pulse
completion, ownership, cancellation and the future vision handshake.
"""
import copy
import json
import queue
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

from host.pick_place import DEFAULT, PickPlaceSettings, cycle_steps, validated
from host.protocol import encode, decode, Reply
from host.simulator import SimulatedBoard
from host.webapp import Console
from host.webserver import Handler, Session


class Link:
    """Queue-only fake transport preserving the real STOP cancellation contract."""
    def __init__(self):
        self.commands = queue.Queue()
        self.sent = []

    def submit(self, body, tag=None):
        self.sent.append(body)
        if body == 'STOP':
            while not self.commands.empty():self.commands.get_nowait()
        self.commands.put((body, tag))
        return True


class PickPlaceTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.clock = [10.]
        for target, base in [('time.monotonic', 0), ('time.time', 1700000000)]:
            mocked = patch(target, side_effect=lambda base=base:base+self.clock[0])
            mocked.start(); self.addCleanup(mocked.stop)
        root = Path(self.directory.name)
        self.console = Console(root/'nav.json', root/'aux.json', root/'flow.json')
        self.console.connection = self.link = Link()
        self.module = self.console.pick_place
        self.board = SimulatedBoard(clock=lambda:self.clock[0])
        self.sequence = 0
        self.console.device['_port'] = 'SIM'
        for body in ['HELLO', 'LIFT INIT', 'LIFT DIR 1', 'AUX ZERO 6',
                     'AUX CONFIG 5 0 10000', 'AUX CONFIG 6 -2000 0', 'ARM 1',
                     'MOTION STAT', 'AUX STAT 5', 'AUX STAT 6']:
            reply = self.send(body)
            self.assertTrue(reply.ok, (body, reply.body))
            self.console._reply(body, reply)
        self.flush()
        self.link.sent.clear()

    def send(self, body):
        self.sequence += 1
        self.board.receive(encode(self.sequence+1000000, 'PING'))
        return decode(self.board.receive(encode(self.sequence, body)))

    def flush(self):
        for _ in range(100):
            if self.link.commands.empty():return
            body, tag = self.link.commands.get_nowait()
            self.console._reply(body, self.send(body), tag)
        self.fail('unbounded queue')

    def step(self, dt=.05):
        self.clock[0] += dt
        self.send('PING')
        self.console.tick()
        self.flush()

    def until(self, predicate, count=7000):
        for _ in range(count):
            if predicate():return
            if self.module.state == 'fault':self.fail(self.module.error)
            self.step()
        self.fail(str(self.module.snapshot()))

    def start(self, mode='pick', **kwargs):
        ok, message = self.console.pick_place_action(mode, **kwargs)
        self.assertTrue(ok, message)

    def actions(self):
        return [body for body in self.link.sent if body.startswith(('AUX GOTO ', 'SERVO ')) and body != 'SERVO STAT']

    def test_pick_full_default_pulses_correct_channels_and_parallel_transfer(self):
        self.start(item=1)
        self.until(lambda:self.module.index == 4)
        # Both commands have been submitted in the same stage before either
        # target completes. Later release is withheld until both have finished.
        self.assertEqual(self.actions()[-2:], ['AUX GOTO 6 -1750 300 1000', 'SERVO 1 270'])
        self.until(lambda:self.module.state == 'done')
        self.assertEqual(self.actions(), ['SERVO 2 120', 'AUX GOTO 5 9200 300 1000',
            'SERVO 2 75', 'AUX GOTO 5 0 300 1000', 'AUX GOTO 6 -1750 300 1000',
            'SERVO 1 270', 'SERVO 2 120', 'AUX GOTO 6 0 300 1000'])
        self.assertEqual([self.send('AUX STAT '+a).fields['pos'] for a in ('5','6')], ['0','0'])

    def test_one_click_restores_saved_range_and_arms_from_live_nx_state(self):
        # NX had a valid zero but M6's saved range was never applied. Starting
        # from disarmed must handle that setup without another operator button.
        self.console._reply('ARM 0', self.send('ARM 0'))
        self.send('LIFT DIR -1')
        self.send('AUX CONFIG 5 0 9400')
        self.board.rotation_configured = False
        self.board.aux_ranges[1] = [0, 0]
        saved = self.console.aux_settings.snapshot()
        saved['5']['max'] = 9400
        saved['6'].update(min=-2950, max=0, presets=[0, -2950])
        self.console.aux_settings.save(saved)
        self.module.save(dict(DEFAULT, lift_sps=1000, rotation_sps=1000, s1_closed=80))
        self.link.sent.clear()
        self.start()
        self.until(lambda:self.module.state == 'done')
        self.assertIn('AUX CONFIG 6 -2950 0', self.link.sent)
        self.assertLess(self.link.sent.index('AUX CONFIG 6 -2950 0'), self.link.sent.index('ARM 1'))
        self.assertLess(self.link.sent.index('ARM 1'), self.link.sent.index('SERVO 2 120'))
        self.assertIn('AUX GOTO 5 9200 1000 1000', self.actions())
        self.assertIn('SERVO 2 80', self.actions())
        self.assertEqual(self.send('LIFT STAT').fields['down_sign'], '-1')
        self.assertFalse(any(body.startswith(('AUX ZERO', 'LIFT INIT', 'LIFT DIR')) for body in self.link.sent))
        self.assertEqual(self.console.aux_settings.snapshot(), saved)
        self.assertEqual([self.send('AUX STAT '+a).fields['pos'] for a in ('5','6')], ['0','0'])

    def test_place_independent_depths_and_fixed_three_retrieval_angles(self):
        config = copy.deepcopy(DEFAULT)
        config.update(place_take_down=123, place_drop_down=456)
        self.module.save(config)
        for item, angle in [(1,0), (2,135), (3,270)]:
            self.link.sent.clear()
            self.start('place', item=item, task_code='156+231+516+123')
            self.until(lambda:self.module.state == 'done')
            self.assertEqual(self.actions(), ['SERVO 2 120', 'AUX GOTO 6 -1750 300 1000',
                'SERVO 1 '+str(angle), 'AUX GOTO 5 123 300 1000', 'SERVO 2 75',
                'AUX GOTO 5 0 300 1000', 'AUX GOTO 6 0 300 1000',
                'AUX GOTO 5 456 300 1000', 'SERVO 2 120', 'AUX GOTO 5 0 300 1000'])
            self.assertEqual(self.module.context['color_id'], [6,5,1][item-1])
            self.assertEqual(self.module.context['target'], [1,3,2][item-1])

    def test_place_waits_half_second_after_verified_top_before_rotating_home(self):
        """Rotation must remain still through the extra top-position dwell."""
        self.module.save(dict(DEFAULT, place_take_down=123, place_drop_down=456))
        self.start('place')
        self.until(lambda:self.module.snapshot()['label'] == '最高点稳定等待 0.5 秒')
        self.assertEqual(self.send('AUX STAT 5').fields['pos'], '0')
        self.assertIsNone(self.module.axis_move)
        before = self.actions()
        self.assertEqual(before[-1], 'AUX GOTO 5 0 300 1000')
        self.assertNotIn('AUX GOTO 6 0 300 1000', before)
        self.step(.499)
        self.assertEqual(self.actions(), before)
        self.step(.002)
        self.assertEqual(self.actions()[-1], 'AUX GOTO 6 0 300 1000')
        self.until(lambda:self.module.state == 'done')

    def test_stop_during_place_top_wait_prevents_rotation_home(self):
        """Cancelling the dwell must retire the remaining mechanism actions."""
        self.module.save(dict(DEFAULT, place_take_down=123, place_drop_down=456))
        self.start('place')
        self.until(lambda:self.module.snapshot()['label'] == '最高点稳定等待 0.5 秒')
        before = self.actions()
        self.assertTrue(self.console.submit('STOP')[0])
        self.flush()
        self.step(1.)
        self.assertEqual(self.module.state, 'stopped')
        self.assertEqual(self.actions(), before)

    def test_pick_colors_four_to_six_and_repeats_do_not_select_angles(self):
        for item in (1,2,3):
            self.start(item=item, task_code='446+231+516+123')
            self.assertEqual(self.module.context['color_id'], [4,4,6][item-1])
            self.assertEqual(self.module.steps[4]['angle'], [270,135,0][item-1])
            self.module.stop(send=False)

    def test_b_complete_pick_requires_camera_resources_before_moving(self):
        self.assertIsNone(self.console.task_cameras)
        ok, message = self.console.pick_place_action('pick', profile='b_pickup')
        self.assertFalse(ok)
        self.assertIn('摄像头1', message)
        self.assertFalse(self.module.active)
        self.assertEqual(self.actions(), [])

    def test_b_profile_rejects_non_pick_or_vision_gated_starts(self):
        for mode, options in (('place', {}), ('pick', {'require_vision':True})):
            ok, _ = self.console.pick_place_action(mode, profile='b_pickup', **options)
            self.assertFalse(ok)
            self.assertFalse(self.module.active)
        self.assertFalse(self.console.pick_place_action('pick', profile='unknown')[0])

    def test_fresh_limits_and_zero_checked_before_first_servo_command(self):
        self.send('AUX CONFIG 5 0 8000')
        self.start()
        for _ in range(100):
            self.step()
            if self.module.state == 'fault':break
        self.assertEqual(self.module.state, 'fault')
        self.assertIn('9200', self.module.error)
        self.assertEqual(self.actions(), [])
        self.assertEqual(self.send('AUX STAT 5').fields['max'], '8000')

    def test_stop_invalidates_queued_commands_and_delayed_ack(self):
        self.start()
        self.module.tick()
        body, tag = self.link.commands.get_nowait()
        reply = self.send(body)
        self.assertTrue(self.console.submit('STOP')[0])
        self.module.reply(body, reply, tag)
        for _ in range(4):self.step()
        self.assertEqual(self.module.state, 'stopped')
        self.assertEqual(self.actions(), [])

    def test_manual_controls_and_other_owners_cannot_interrupt_cycle(self):
        self.start(require_vision=True)
        for body in ('SERVO 1 0','AUX GOTO 5 5 300 1000','ARM 1','MOVE 1 5 300 1000'):
            self.assertFalse(self.console.submit(body)[0], body)
        self.assertFalse(self.console.jog('CHASSIS JOG FWD 300 1000')[0])
        self.assertFalse(self.console.navigation_action('start')[0])
        self.assertFalse(self.console.task_action('start')[0])
        with self.assertRaises(ValueError):self.console.camera_alignment.start()
        with self.assertRaises(ValueError):self.module.save(DEFAULT)
        self.assertTrue(self.console.submit('AUX HALT 5')[0])
        self.assertFalse(self.module.active)
        self.assertIn('STOP', self.link.sent)

    def test_vision_mailbox_checks_job_target_freshness_without_moving(self):
        self.start(item=2, batch=2, task_code='156+231+516+123', require_vision=True)
        request = self.module.snapshot()['request']
        sample = dict(request, captured_at=time.time(), ready=True)
        self.assertEqual(sample['color_id'], 1)
        for change in ({'job_id':'old'}, {'color_id':6}, {'captured_at':time.time()-10}, {'ready':False}):
            with self.assertRaises(ValueError):self.console.pick_place_vision(dict(sample, **change))
        self.console.pick_place_vision(sample)
        self.assertEqual(self.link.sent, [])
        with self.assertRaises(ValueError):self.console.pick_place_vision(sample)
        self.until(lambda:self.module.state == 'done')
        self.assertIn('SERVO 1 135', self.actions())

    def test_missing_ack_and_vision_timeouts_cancel_without_resume(self):
        self.start(require_vision=True)
        self.clock[0] += 31
        self.module.tick()
        self.assertEqual(self.module.state, 'fault')
        self.assertIn('视觉', self.module.error)
        self.start()
        self.module.tick()
        self.clock[0] += 4
        self.module.tick()
        self.assertEqual(self.module.state, 'fault')
        self.assertIn('应答超时', self.module.error)

    def test_changed_move_id_cannot_release_gripper(self):
        self.start()
        self.until(lambda:self.module.axis_move and self.module.axis_move.get('id'))
        self.clock[0] += .2
        self.module.tick()
        body, tag = self.link.commands.get_nowait()
        self.module.reply(body, Reply(99, 'OK cmd=AXIS axis=5 active=0 remaining=0 move_id=999'), tag)
        self.assertEqual(self.module.state, 'fault')
        self.assertNotIn('SERVO 2 75', self.actions())

    def test_disconnect_cancels_and_noop_axes_do_not_wait_for_fake_move_id(self):
        config = copy.deepcopy(DEFAULT)
        config.update(pick_down=0, pick_rotation=0)
        self.module.save(config)
        self.start()
        self.until(lambda:self.module.state == 'done')
        self.assertFalse(any(body.startswith('AUX GOTO') for body in self.actions()))
        self.start()
        self.console._absorb('disconnected', 'SIM')
        self.assertEqual(self.module.state, 'fault')

    def test_start_requires_fresh_zero_and_direction_not_cached_values(self):
        self.board.axes[4]['pos'] = 1
        self.start()
        for _ in range(80):
            self.step()
            if self.module.state == 'fault':break
        self.assertIn('原点 0', self.module.error)
        self.assertEqual(self.actions(), [])

    def test_fault_disarm_and_stage_timeout_never_release_after_closing(self):
        config = dict(DEFAULT, step_timeout_ms=1000)
        self.module.save(config)
        self.start()
        for _ in range(100):
            self.step()
            if self.module.state == 'fault':break
        self.assertEqual(self.module.state, 'fault')
        self.assertIn('步骤执行超时', self.module.error)
        self.assertNotIn('SERVO 2 75', self.actions())

    def test_motion_fault_cancels_and_rejects_old_vision_result(self):
        self.start(require_vision=True, task_code='156+231+516+123')
        sample = dict(self.module.request, ready=True, captured_at=time.time())
        self.board.inject_motion_fault()
        self.console._reply('MOTION STAT', self.send('MOTION STAT'))
        self.assertEqual(self.module.state, 'fault')
        self.assertIn('运动故障', self.module.error)
        with self.assertRaises(ValueError):self.module.vision(sample)
        self.assertEqual(self.actions(), [])

    def test_malformed_start_inputs_do_not_claim_motion(self):
        for options in ({'item':True}, {'batch':True}, {'item':4}, {'task_code':False}, {'require_vision':'yes'}):
            ok, _ = self.console.pick_place_action('pick', **options)
            self.assertFalse(ok)
            self.assertFalse(self.module.active)
        self.assertEqual(self.link.sent, [])


class PickPlaceConfigTests(unittest.TestCase):
    def test_complete_typed_parameters_persist_and_invalid_values_preserve_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'pick_place.json'
            settings = PickPlaceSettings(path)
            saved = settings.save(dict(DEFAULT, pick_down=9300))
            self.assertEqual(PickPlaceSettings(path).snapshot(), saved)
            for key, value in [('pick_down',True),('s1_open',181),('s2_slot_1',271),('lift_sps',0),('place_rotation',-1000001)]:
                with self.assertRaises(ValueError):settings.save(dict(saved, **{key:value}))
            self.assertEqual(PickPlaceSettings(path).snapshot(), saved)
            with self.assertRaises(ValueError):validated({'pick_down':9200})


class PickPlaceHttpTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        root = Path(self.directory.name)
        self.console = Console(root/'nav.json', root/'aux.json', root/'flow.json')
        self.session = Session(self.console)
        self.session.join('writer'); self.session.join('reader')
        handler = type('PickPlaceHandler', (Handler,), {'session':self.session})
        self.server = ThreadingHTTPServer(('127.0.0.1',0), handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown(); self.server.server_close(); self.thread.join(2)
        self.directory.cleanup()

    def call(self, route, payload=None, token='writer'):
        request = urllib.request.Request('http://127.0.0.1:{}/api/pick-place/{}'.format(self.server.server_port, route),
            data=None if payload is None else json.dumps(payload).encode(), headers={'X-Session':token})
        try:
            with urllib.request.urlopen(request, timeout=3) as response:return response.status, json.load(response)
        except urllib.error.HTTPError as error:return error.code, json.load(error)

    def test_state_save_validation_permissions_and_offline_start(self):
        self.assertEqual(self.call('state')[1]['config'], DEFAULT)
        config = dict(DEFAULT, place_drop_down=8500)
        self.assertFalse(self.call('action', {'action':'pick'})[1]['ok'])
        self.assertEqual(self.call('config', {'config':config}, 'reader')[0], 409)
        self.assertEqual(self.call('config', {'config':config})[1]['config'], config)
        self.assertEqual(self.call('config', {'config':{}})[0], 400)
        self.assertEqual(self.call('state')[1]['config'], config)
        self.assertIsNone(self.call('request')[1]['request'])
        self.assertEqual(self.call('vision', {'sample':{}})[0], 400)


if __name__ == '__main__':
    unittest.main()
