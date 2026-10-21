"""Exercise real Console ACK dispatch and lease clocks without physical motion."""
import unittest
from unittest.mock import patch

from host.protocol import Reply
from host.transport import Connection
from host.webapp import Console
from tests import test_camera_servo as servo_harness


class PauseHostTests(unittest.TestCase):
    """Use real executors, with the existing camera test's deterministic link."""
    setUp = servo_harness.ServoTests.setUp
    send = servo_harness.ServoTests.send
    flush = servo_harness.ServoTests.flush
    step = servo_harness.ServoTests.step
    until = servo_harness.ServoTests.until
    start = servo_harness.ServoTests.start

    def pause_clock(self):
        """Expose the worker clock without starting a serial thread."""
        self.link.imu_pause_snapshot = lambda: (self.hold, self.elapsed)
        self.hold, self.elapsed = False, 0.

    def test_long_pause_freezes_budgets_and_buffers_success_exactly_once(self):
        self.start(); self.pause_clock()
        self.a.send('TURN STAT', 'heading')
        body, tag = self.link.commands.get_nowait()
        before = self.a.pending
        started = self.a.started
        self.hold = True
        self.c._reply(body, Reply(1, 'OK cmd=TURN_STAT imu=0 paused=1'), tag)
        self.assertEqual(len(self.c.imu_pause.replies), 1)
        self.clock[0] += 120; self.elapsed = 120.
        self.c.tick()
        self.assertTrue(self.a.active, self.a.error)
        self.assertEqual(self.a.started, started+120)
        self.assertEqual(self.a.pending[-1], before[-1]+120)
        self.assertEqual(self.link.commands.qsize(), 0)
        self.hold = False
        self.c.tick()
        # Cached imu=0 must only release its tag, never fault or unlock motion.
        self.assertTrue(self.a.active, self.a.error)
        self.assertEqual(self.c.imu_pause.replies, [])
        self.assertTrue(self.a.resume_heading_required)
        new_body, new_tag = self.link.commands.get_nowait()
        self.assertEqual(new_body, 'TURN STAT')
        self.assertNotEqual(new_tag, tag)
        self.c._reply(new_body, Reply(2, 'OK cmd=TURN_STAT imu=1 zero=1 active=0 fault=none error_cd=12 tolerance_cd=50'), new_tag)
        self.assertFalse(self.a.resume_heading_required)

    def test_stop_during_pause_clears_buffer_and_cannot_restart(self):
        self.start(); self.pause_clock()
        self.a.send('TURN STAT', 'heading')
        body, tag = self.link.commands.get_nowait()
        self.hold = True
        self.c._reply(body, self.send(body), tag)
        self.assertTrue(self.c.imu_pause.replies)
        self.c.submit('STOP'); self.flush()
        self.assertFalse(self.a.active)
        self.assertEqual(self.c.imu_pause.replies, [])
        self.hold = False; self.elapsed = 10.; self.clock[0] += 10
        self.c.tick()
        self.assertFalse(self.a.active)
        self.assertFalse(self.c.armed)

    def test_action_ack_is_delivered_once_without_resending_servo(self):
        self.pause_clock(); self.a.start()
        self.flush()  # Wait for the recognition-claw settle interval.
        self.a.pending = None
        self.a.send('SERVO 2 170', 'claw_open')
        body, tag = self.link.commands.get_nowait()
        self.hold = True
        self.c._reply(body, Reply(1, 'OK cmd=SERVO'), tag)
        self.clock[0] += 120; self.elapsed = 120
        self.c.tick()
        self.assertIsNotNone(self.a.pending)
        self.hold = False
        self.c.tick()
        self.assertIsNone(self.a.pending)
        self.assertGreater(self.a.claw_until, self.clock[0])
        self.assertEqual(self.c.imu_pause.replies, [])
        self.assertFalse(any(body.startswith('SERVO ') for body, _ in list(self.link.commands.queue)))

    def test_complete_worker_pause_between_console_ticks_still_requires_fresh_heading(self):
        self.start(); self.pause_clock()
        self.c.imu_pause.sync()
        self.a.send('TURN STAT', 'heading')
        _, old_tag = self.link.commands.get_nowait()
        before = self.a.capture_token
        self.clock[0] += 120; self.elapsed = 120.
        self.c.imu_pause.sync()
        self.assertTrue(self.a.resume_heading_required)
        self.assertEqual(self.a.resume_old_heading_tag, old_tag)
        self.assertGreater(self.a.capture_token, before)


class WorkerPauseClockTests(unittest.TestCase):
    def test_clock_tracks_whole_pause_even_when_console_misses_transition(self):
        import queue
        with patch('time.monotonic', return_value=10.) as clock:
            link = Connection('unused', queue.Queue())
            link.absorb_imu_pause({'paused': '1'})
            clock.return_value = 130.
            self.assertEqual(link.imu_pause_snapshot(), (True, 120.))
            link.absorb_imu_pause({'paused': '0'})
            clock.return_value = 200.
            self.assertEqual(link.imu_pause_snapshot(), (False, 120.))
            link.absorb_imu_pause({'paused': '1'})
            clock.return_value = 201.
            link.absorb_imu_pause({'armed': '0'})
            self.assertEqual(link.imu_pause_snapshot(), (False, 121.))

