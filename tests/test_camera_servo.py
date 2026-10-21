"""NX visual velocity servo through the real protocol and simulated wheel plant."""
import copy
import queue
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
from host.camera_vision import DEFAULT
from host.protocol import encode, decode, Reply
from host.simulator import SimulatedBoard
from host.webapp import Console
from tests.test_mission import Link


class VelocityLink(Link):
    def __init__(self):
        super().__init__()
        self.velocity = None

    def set_camera_velocity(self, body, tag):
        if body != self.velocity:
            self.velocity = body
            self.submit(body,tag)

    def clear_camera_velocity(self):
        self.velocity = None


class ServoTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.clock = [10.]
        p = patch('time.monotonic',side_effect=lambda:self.clock[0]);p.start();self.addCleanup(p.stop)
        self.c = Console(navigation_path=Path(self.temp.name)/'nav.json')
        self.c.connection = self.link = VelocityLink()
        self.board = SimulatedBoard(clock=lambda:self.clock[0]); self.seq = 0
        self.c.device['_port'] = 'SIM'
        self.c._reply('HELLO',self.send('HELLO'))
        self.c.submit('ARM 1'); self.flush()
        self.a = self.c.camera_alignment
        self.a.hub = SimpleNamespace(snapshot=lambda:{'state':'ready'},start_tracking=Mock(),stop_tracking=Mock())
        self.a.settings.value = copy.deepcopy(DEFAULT)
        self.a.settings.value['tracking']['camera_yaw_deg'] = 0
        def capture(image=None):
            self.a.capturing = True
            self.capture_generation = self.a.generation
        self.a.capture = capture
        self.clock[0] += .05; self.send('PING')

    def send(self, body):
        self.seq += 1
        if body != 'PING': self.board.receive(encode(self.seq+1000000,'PING'))
        return decode(self.board.receive(encode(self.seq,body)))

    def flush(self):
        for _ in range(500):
            if self.link.commands.empty(): return
            body,tag = self.link.commands.get_nowait()
            self.c._reply(body,self.send(body),tag)
        self.fail('unbounded command queue')

    def step(self, dt=.05):
        self.clock[0] += dt
        self.send('PING'); self.c.tick(); self.flush()

    def until(self, condition, limit=100):
        for _ in range(limit):
            if condition(): return
            self.assertNotEqual(self.a.state,'fault',self.a.error)
            self.step()
        self.fail(self.a.snapshot())

    def start(self):
        self.a.start(); self.flush(); self.until(lambda:self.a.state=='tracking')
        self.assertEqual(self.a.state,'tracking',self.a.error)
        self.c.last_client = 0

    def test_recognition_waits_for_physical_s1_170_ack_and_settle(self):
        self.c.pick_place.settings.value['s1_settle_ms'] = 600
        self.a.start()
        body,tag = self.link.commands.get_nowait()
        self.assertEqual(body, 'CHASSIS STAT')
        self.c._reply(body, self.send(body), tag)
        body,tag = self.link.commands.get_nowait()
        self.assertEqual(body, 'SERVO 2 170')
        self.step(.5)  # No ACK: elapsed time alone must not start recognition.
        self.a.hub.start_tracking.assert_not_called()
        self.assertNotIn('TURN ZERO', self.link.sent)
        self.c._reply(body, self.send(body), tag)
        self.step(.55)
        self.a.hub.start_tracking.assert_not_called()
        self.assertNotIn('TURN ZERO', self.link.sent)
        self.step(.1)
        self.assertEqual(self.a.state, 'tracking', self.a.error)
        self.a.hub.start_tracking.assert_called_once()

    def test_s1_opening_routes_legacy_physical_channel_and_scale(self):
        self.c.device.update(servo1_pin='PH6', servo2_pin='PH9', servo1_travel='2700')
        self.a.start(); self.flush()
        self.assertIn('SERVO 1 255', self.link.sent)
        self.a.hub.start_tracking.assert_not_called()

    def test_stop_or_rejected_s1_ack_cannot_start_recognition(self):
        self.a.start()
        body,tag = self.link.commands.get_nowait()
        self.c._reply(body, self.send(body), tag)
        body,tag = self.link.commands.get_nowait()
        self.assertEqual(body, 'SERVO 2 170')
        self.a.reply(body, Reply(1, 'ERR code=REJECTED'), tag)
        self.flush(); self.step(1)
        self.assertEqual(self.a.state, 'fault')
        self.a.hub.start_tracking.assert_not_called()
        self.assertNotIn('TURN ZERO', self.link.sent)

    def test_stop_during_s1_settle_cancels_recognition(self):
        self.a.start(); self.flush()
        self.c.submit('STOP'); self.flush(); self.step(1)
        self.assertEqual(self.a.state, 'stopped')
        self.a.hub.start_tracking.assert_not_called()
        self.assertNotIn('TURN ZERO', self.link.sent)

    def sample(self, dx=0, dy=0, **fields):
        report = dict(kind='block',target='red',found=True,center=[320+dx,210+dy],
                      control_point=[320+dx,240+dy],reference=[320,240],
                      captured_at=time.time(),captured_monotonic=time.monotonic(),
                      frame_id=self.a.last_frame_id+1,error_dx=dx,error_dy=dy)
        report.update(fields)
        self.a.results.put((self.capture_generation,report,None,None)); self.step()

    def test_both_axes_drive_continuously_at_substantial_speed_without_browser(self):
        self.start();self.sample(-144,22)
        self.assertLessEqual(self.a.vx,-1000)
        self.assertLessEqual(self.a.vy,-200)
        before=[a['pos'] for a in self.board.axes[:4]]
        for _ in range(8):self.step()
        self.assertGreater(max(abs(a['pos']-p) for a,p in zip(self.board.axes,before)),100)
        self.assertTrue(any(b.startswith('WHEELS ') for b in self.link.sent))

    def test_standalone_color_alignment_keeps_elapsed_time_limit(self):
        """Only B's station owner can opt out of the visual elapsed limit."""
        self.start()
        self.a.started = self.clock[0]-61
        self.sample(dx=100)
        self.assertEqual(self.a.state, 'fault')
        self.assertIn('60', self.a.error)
        self.assertFalse(any(b.startswith(('CHASSIS STRAIGHT','TURN MOVE','NAV FIX','NAV RESET','NAV START')) for b in self.link.sent))

    def test_left_90_mount_rotates_pid_translation_before_wheel_mixing(self):
        self.a.settings.value['tracking']['camera_yaw_deg'] = 90
        self.start(); self.sample(100,0)
        self.assertEqual((self.a.vx,self.a.vy),(0,1000))
        self.assertEqual(self.a.wheels,[-1000,1000,-1000,1000])
        # Positive image Y asks camera backward, therefore body right at +90.
        self.sample(0,100)
        self.assertEqual((self.a.vx,self.a.vy),(1000,0))
        self.assertEqual(self.a.heading_error,0)
        self.assertEqual(self.a.yaw,0)

    def test_entering_deadzone_brakes_and_confirms_actual_idle(self):
        self.start();self.sample(140,0)
        self.sample(15,0)
        self.assertLess(self.a.vx,1400)
        for _ in range(3):self.sample()
        self.until(lambda:self.a.state=='done')
        self.assertTrue(self.board.armed)
        self.assertFalse(any(a['active'] for a in self.board.axes[:4]))
        self.assertIsNone(self.link.velocity)
        self.a.results.put((self.capture_generation,None,None,'stream canceled after completion'))
        self.step()
        self.assertEqual(self.a.state,'done')

    def test_predictive_stop_outside_pixel_tolerance_cannot_finish_alignment(self):
        """A braking output is not evidence that the image target is centered."""
        self.start()
        self.a.pid.update = Mock(return_value=(0,0))
        for _ in range(4): self.sample(20,0)
        for _ in range(12): self.step()
        self.assertEqual(self.a.stable,0)
        self.assertNotEqual(self.a.state,'done')
        self.assertFalse(any(self.a.wheels))

    def test_idle_confirmation_is_not_starved_by_odd_camera_frame_cadence(self):
        """Replay queries that always happen on an odd image sequence number."""
        self.start()
        for _ in range(24):
            self.a.sequence = 0  # Next consumed frame is odd at every query.
            self.sample(0,0)
            if self.a.state == 'done': break
        self.assertEqual(self.a.state,'done')

    def test_reversal_brakes_before_opposite_wheel_targets(self):
        self.start();self.sample(100,0)
        for _ in range(5):self.step()
        self.sample(-100,0);self.step()
        self.assertEqual(self.a.state,'braking')
        self.assertEqual(self.a.wheels,[0]*4)
        self.until(lambda:self.a.state=='tracking')
        self.step(.1)
        self.assertLess(self.a.vx,0)
        self.assertEqual(self.a.error,'')
        self.assertIn('WHEELS 0 0 0 0 6000',self.link.sent)

    def test_delayed_idle_reply_cannot_erase_running_wheel_directions(self):
        self.start()
        self.sample(0,0)
        # Query while idle, but delay delivery until a new image starts motion.
        self.assertIsNone(self.a.pending)
        self.a.send('CHASSIS STAT','wheel_status')
        body,tag=self.link.commands.get_nowait()
        idle=self.send(body)
        self.sample(100,0)
        self.assertTrue(any(axis['active'] for axis in self.board.axes[:4]))
        self.c._reply(body,idle,tag)
        self.sample(-100,0)
        self.assertEqual(self.a.state,'braking',self.a.error)
        self.assertTrue(self.a.active,self.a.error)

    def test_idle_reply_for_earlier_zero_cannot_release_later_braking(self):
        self.start(); self.sample(0,0)
        self.a.send('CHASSIS STAT','wheel_status')
        body,tag=self.link.commands.get_nowait(); idle=self.send(body)
        self.sample(100,0)
        self.sample(-100,0)
        self.assertEqual(self.a.state,'braking')
        self.c._reply(body,idle,tag)
        self.assertEqual(self.a.state,'braking')
        self.assertTrue(any(self.a.polarities))

    def test_old_zero_ack_cannot_authorize_new_stop_query(self):
        self.start(); self.sample(100,0)
        old_revision=self.a.velocity_revision
        self.sample(-100,0)
        self.a.zero_ack_revision=None
        self.a.reply('WHEELS 0 0 0 0 6000',Reply(1,'OK cmd=WHEELS state=accepted'),
                     ('camera_velocity',self.a.generation,old_revision))
        self.assertIsNone(self.a.zero_ack_revision)

    def test_idle_query_requires_board_acknowledged_zero_velocity(self):
        self.start(); self.sample(100,0)
        for _ in range(5): self.step()
        self.a.vx=-1000
        self.clock[0]+=.15
        # The producer has queued zero, but the serial worker has not sent it.
        self.a.tick()
        self.assertEqual(self.a.state,'braking')
        self.assertTrue(self.a.pending is None or self.a.pending[1]!='wheel_status')

    def test_same_direction_restart_waits_for_zero_stop_tail_to_finish(self):
        self.start();self.sample(100,0)
        for _ in range(5):self.step()
        self.sample(0,0)
        self.sample(100,0)
        self.assertEqual(self.a.state,'braking')
        self.assertEqual(self.a.wheels,[0]*4)
        self.until(lambda:self.a.state=='tracking')
        self.step(.1)
        self.assertGreater(self.a.vx,0)
        self.assertEqual(self.a.error,'')

    def assert_plant_converges(self, angle):
        import math
        self.a.settings.value['tracking']['camera_yaw_deg'] = angle
        self.start(); error=[144.,-60.]
        positions=[a['pos'] for a in self.board.axes[:4]]
        for index in range(1000):
            if index%6 == 0:self.sample(*error)
            else:self.step()
            current=[a['pos'] for a in self.board.axes[:4]]
            physical=[(v-p)*(-1 if self.board.invert & (1<<i) else 1) for i,(v,p) in enumerate(zip(current,positions))]
            positions=current
            right=(-physical[0]+physical[1]+physical[2]-physical[3])/4
            forward=sum(physical)/4
            # Only the synthetic plant knows this displacement scale.
            theta = math.radians(angle)
            camera_right = math.cos(theta)*right+math.sin(theta)*forward
            camera_forward = -math.sin(theta)*right+math.cos(theta)*forward
            error[0]-=camera_right*.025;error[1]+=camera_forward*.025
            if self.a.state=='done':break
            self.assertNotEqual(self.a.state,'fault',self.a.error)
        self.assertEqual(self.a.state,'done',self.a.snapshot())
        self.assertLessEqual(max(map(abs,error)),5)

    def test_real_wheel_feedback_plant_converges_both_axes(self):
        self.assert_plant_converges(0)

    def test_left_90_camera_plant_converges_both_axes(self):
        self.assert_plant_converges(90)

    def test_right_90_camera_plant_converges_both_axes(self):
        self.assert_plant_converges(-90)

    def test_gyro_error_is_mixed_into_translation_without_separate_turn(self):
        self.start();self.sample(100,0)
        self.board.navigation.absolute_heading+=2.
        for _ in range(6):self.step()
        self.assertLess(self.a.yaw,0)
        self.assertNotEqual(self.a.wheels[0],self.a.wheels[2])

    def test_missing_or_wrong_target_stops_and_cancels_lease(self):
        self.start();self.sample(100,0)
        self.sample(found=False);self.flush()
        self.assertEqual(self.a.state,'fault')
        self.assertIsNone(self.link.velocity)
        self.assertFalse(self.board.armed)

    def test_stale_result_waits_for_new_frame_and_stall_stops(self):
        self.start();self.sample(100,0,captured_monotonic=self.clock[0]-1)
        self.assertEqual(self.a.state,'tracking')
        self.assertEqual(self.a.iterations,0)
        self.sample(100,0)
        for _ in range(18):self.step()
        self.assertEqual(self.a.wheels,[0]*4)
        for _ in range(50):
            if self.a.state=='fault':break
            self.step()
        self.assertEqual(self.a.state,'fault')
        self.assertFalse(self.board.armed)

    def test_stop_invalidates_delayed_camera_result_and_rejects_manual_motion(self):
        self.start();self.sample(100,0)
        self.assertFalse(self.c.submit('WHEELS 100 100 100 100 6000')[0])
        self.c.submit('STOP');self.flush()
        self.sample(100,0)
        self.assertEqual(self.a.state,'stopped')
        self.assertIsNone(self.link.velocity)
        self.assertFalse(self.board.armed)

    def test_changed_gyro_session_or_rejected_velocity_stops(self):
        self.start();self.board.turn.zero=None
        self.step(.15)
        self.assertEqual(self.a.state,'fault')
        self.assertFalse(self.board.armed)

    def test_late_ack_from_previous_run_cannot_change_new_run(self):
        self.start()
        self.a.reply('WHEELS 1 1 1 1 6000',Reply(1,'ERR code=OLD'),('camera_velocity',self.a.generation-1))
        self.assertEqual(self.a.state,'tracking')

    def test_rejected_velocity_cancels_run_and_lease(self):
        self.start();self.sample(100,0)
        self.a.reply('WHEELS 1 1 1 1 6000',Reply(1,'ERR code=REJECTED'),('camera_velocity',self.a.generation))
        self.flush()
        self.assertEqual(self.a.state,'fault')
        self.assertIsNone(self.link.velocity)
        self.assertFalse(self.board.armed)
        self.assertEqual(self.a.fault_context['command'],'WHEELS 1 1 1 1 6000')
        path=Path(self.temp.name)/'logs'/'camera_alignment_faults.jsonl'
        import json
        record=json.loads(path.read_text(encoding='utf-8').splitlines()[-1])
        self.assertTrue(record['armed'])  # Context must precede automatic STOP.
        self.assertEqual(record['reply'],'ERR code=REJECTED')

    def test_held_navigation_and_active_axes_cannot_start(self):
        self.c.navigation={'state':'waiting','held':'1'}
        with self.assertRaises(ValueError):self.a.start()
        self.c.navigation={};self.c.axes={'1':{'active':'1'}}
        with self.assertRaises(ValueError):self.a.start()

    def test_status_ack_timeout_stops_without_starting_motion(self):
        self.a.start();self.clock[0]+=2.1;self.a.tick();self.flush()
        self.assertEqual(self.a.state,'fault')
        self.assertFalse(self.board.armed)


class TransportLeaseTests(unittest.TestCase):
    def test_acknowledged_zero_is_not_repeated_during_firmware_stop_tail(self):
        from host.simulator import SimulatedSerial
        from host.transport import Connection
        class StopTailSerial(SimulatedSerial):
            """Mirror AXF-reproduced stopping rejection through the real worker."""
            def __init__(self):
                super().__init__()
                self.zero_count=0; self.last_zero=0.; self.commands=[]
                original=self.board.dispatch
                def dispatch(sequence,body):
                    self.commands.append(body)
                    if body.startswith('WHEELS 0 0 0 0 '):
                        self.zero_count+=1
                        if self.last_zero and time.monotonic()-self.last_zero < .3:
                            return 'ERR code=DISARMED_BUSY_OR_REVERSE'
                        self.last_zero=time.monotonic()
                    return original(sequence,body)
                self.board.dispatch=dispatch
        events=queue.Queue(); connection=Connection('SIM',events)
        def wait_for(predicate):
            end=time.monotonic()+5
            while not predicate() and time.monotonic()<end: time.sleep(.005)
            self.assertTrue(predicate())
        def renew(body,revision,seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                connection.set_camera_velocity(body,('camera_velocity',1,revision))
                time.sleep(.02)
        with patch('host.transport.SimulatedSerial',StopTailSerial):
            connection.start()
            try:
                wait_for(lambda:connection.serial and any(c=='SERVO STAT' for c in connection.serial.commands))
                connection.submit('ARM 1');wait_for(lambda:connection.serial.board.armed)
                renew('WHEELS 500 -500 500 -500 2000',1,.15)
                renew('WHEELS 0 0 0 0 2000',2,.45)
                self.assertEqual(connection.serial.zero_count,1)
                self.assertTrue(connection.serial.board.armed)
                # A later target change must still send its own stop request.
                renew('WHEELS 500 -500 500 -500 2000',3,.15)
                renew('WHEELS 0 0 0 0 2000',4,.45)
                self.assertEqual(connection.serial.zero_count,2)
                self.assertTrue(connection.serial.board.armed)
                # Suppression never renews the owner's lease behind its back.
                wait_for(lambda:not connection.serial.board.armed)
                self.assertIn('STOP',connection.serial.commands)
            finally:
                connection.close();connection.join(2)

    def test_serial_heartbeat_cannot_keep_frozen_visual_controller_moving(self):
        with tempfile.TemporaryDirectory() as directory:
            c=Console(navigation_path=Path(directory)/'nav.json')
            c.connect('SIM')
            def wait_for(condition):
                deadline=time.monotonic()+5
                while not condition() and time.monotonic()<deadline:
                    c.drain();c.tick();time.sleep(.01)
                self.assertTrue(condition())
            try:
                wait_for(lambda:c.device.get('turn')=='1')
                c.submit('ARM 1');wait_for(lambda:c.armed)
                c.connection.set_camera_velocity('WHEELS 1000 1000 1000 1000 6000',('camera_velocity',999))
                wait_for(lambda:any(a['active'] for a in c.connection.serial.board.axes[:4]))
                wait_for(lambda:not c.armed)
                self.assertFalse(any(a['active'] for a in c.connection.serial.board.axes[:4]))
                self.assertIsNone(c.connection.current_camera_velocity())
            finally:c.disconnect()


if __name__=='__main__':unittest.main()
