"""Deterministic handoff budgets; these are protocol/ARM tests, not floor timings."""
import queue
import threading
import unittest
from unittest.mock import patch
from host.transport import Connection, EventQueue
from host.simulator import SimulatedSerial
from tests import test_mission as mission_fixture
from tests import test_ring_sequence as ring_fixture


class HandoffTests(unittest.TestCase):
    setUp = mission_fixture.MissionTests.setUp
    send = mission_fixture.MissionTests.send
    flush = mission_fixture.MissionTests.flush
    step = mission_fixture.MissionTests.step
    until = mission_fixture.MissionTests.until

    def test_route_upload_does_not_enqueue_debug_telemetry(self):
        self.assertTrue(self.console.navigation_action('start')[0])
        before = len(self.link.sent)
        self.console.tick()
        background = self.link.sent[before:]
        self.assertFalse(any(body.startswith(('AXIS ', 'FLOW ', 'AUX STAT', 'SERVO STAT', 'LIFT STAT', 'TURN STAT'))
                             for body in background), background)

    def test_ready_start_drains_setup_acks_without_timer_ticks(self):
        self.assertTrue(self.console.competition.start()[0])
        self.console.tick(); self.flush()
        self.assertEqual(self.console.competition.state, 'running')
        self.assertTrue(self.console.navigator.active)

    def test_saved_confirmed_tuning_is_not_reapplied_at_every_start(self):
        self.console.submit('ARM 0'); self.flush()
        self.console.submit('TURN CONFIG 3456 35 321'); self.flush()
        before = len(self.link.sent)
        self.assertTrue(self.console.competition.start()[0])
        self.until(lambda: self.console.navigator.active)
        self.assertNotIn('TURN CONFIG 3456 35 321', self.link.sent[before:])

    def test_race_arrival_has_no_saved_demo_dwell(self):
        config = self.console.mission.snapshot()
        config.update(dwell_ms=3000, route=[dict(station=1, hold=0)])
        self.console.save_navigation(config)
        self.assertTrue(self.console.competition.start()[0])
        arrival = []
        original = self.console._reply
        def observe(command, reply, tag=None):
            if (tag and tag[0] == 'navrunner' and tag[3] == 'complete'
                    and reply.fields.get('state') == 'waiting'):
                arrival.append(self.clock[0])
            return original(command, reply, tag)
        with patch.object(self.console, '_reply', side_effect=observe):
            self.until(lambda: self.console.navigator.state == 'done')
        self.assertEqual(len(arrival), 1)
        elapsed = self.clock[0] - arrival[0]
        print('race arrival -> done (simulation): {:.3f}s'.format(elapsed))
        self.assertLessEqual(elapsed, .151)
        self.assertEqual(self.console.mission.snapshot()['dwell_ms'], 3000)


class RingLatencyTests(unittest.TestCase):
    setUp = ring_fixture.SequenceTests.setUp
    send = ring_fixture.SequenceTests.send
    flush = ring_fixture.SequenceTests.flush
    step = ring_fixture.SequenceTests.step
    until = ring_fixture.SequenceTests.until
    start = ring_fixture.SequenceTests.start
    sample = ring_fixture.SequenceTests.sample
    align = ring_fixture.SequenceTests.align
    calibrated_one = ring_fixture.SequenceTests.calibrated_one

    def test_position_completion_starts_next_spacing_on_next_tick(self):
        self.calibrated_one()
        started = self.clock[0]
        self.until(lambda: self.a.ring_sequence.phase == 'spacing')
        elapsed = self.clock[0] - started
        print('ring position -> next motion (simulation): {:.3f}s'.format(elapsed))
        self.assertLessEqual(elapsed, .051)


class SerialLatencyTests(unittest.TestCase):
    def test_control_overtakes_debug_reads_and_stop_retires_all(self):
        connection = Connection('SIM', queue.Queue())
        for command in ('FLOW CONFIG', 'AXIS 1', 'SERVO STAT'):
            connection.submit(command, tag=('telemetry',))
        connection.submit('NAV STATIONS 5', tag=('navigation', 1))
        self.assertEqual(connection.commands.get_nowait()[2], 'NAV STATIONS 5')
        connection.submit('NAV POINT 0 0 0', tag=('navigation', 1))
        connection.submit('STOP')
        self.assertEqual(connection.commands.get_nowait()[2], 'STOP')
        self.assertTrue(connection.commands.empty())

    def test_queue_events_and_commands_wake_their_consumers(self):
        wakeup = threading.Event()
        events = EventQueue(wakeup)
        events.put(('reply', object()))
        self.assertTrue(wakeup.is_set())
        connection = Connection('SIM', events)
        connection.submit('NAV EXEC', tag=('navrunner', 1, 1, 'complete'))
        self.assertTrue(connection.wakeup.is_set())

    def test_fragmented_reply_preserves_frame_and_tag(self):
        class FragmentedSerial(SimulatedSerial):
            def read(self, size=256):
                return super().read(min(size, 3))
        events = queue.Queue()
        connection = Connection('SIM', events)
        connection.serial = FragmentedSerial()
        tag = ('navigation', 123)
        reply = connection.exchange('HELLO', tag)
        self.assertTrue(reply.ok)
        self.assertEqual(reply.fields['protocol'], '1')
        self.assertEqual(events.get_nowait(), ('reply', ('HELLO', reply, tag)))

    def test_short_replies_do_not_wait_for_256_bytes(self):
        class TimedSerial(SimulatedSerial):
            """Charge the pyserial timeout when a read asks for absent bytes."""
            padding = 0.

            @property
            def in_waiting(self):
                return len(self.buffer)

            def read(self, size=256):
                if size > len(self.buffer): self.padding += .02
                return super().read(size)

        stream = TimedSerial()
        connection = Connection('SIM', queue.Queue())
        connection.serial = stream
        self.assertTrue(connection.exchange('PING').ok)
        print('short serial reply timeout padding: {:.3f}s'.format(stream.padding))
        self.assertEqual(stream.padding, 0.)


if __name__ == '__main__':
    unittest.main()
