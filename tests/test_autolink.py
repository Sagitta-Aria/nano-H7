"""Exercise real serial workers through hotplug, pairing and wrong-board refusal.

Synthetic transports use the actual protocol and server pump lifecycle; no
physical UARTs are opened and no physical movement is requested by these tests.
"""
import queue
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from host.autolink import AutoDevices
from host.imu import ImuHub, candidate_ports
from host.protocol import decode
from host.simulator import SimulatedSerial
from host.transport import Connection
from host.webapp import Console
from tests.test_imu import UsbFixture, wait

UID = '1234567890ABCDEF12345678'


class RobotSerial(SimulatedSerial):
    """Valid H743-shaped protocol fixture with configurable identity."""
    def __init__(self, board='H743_CB_V1.5', uid=UID):
        super().__init__()
        self.writes = []
        original = self.board.dispatch

        def dispatch(sequence, body):
            response = original(sequence, body)
            return response.replace('board=SIMULATOR', 'board=' + board).replace('uid=SIMULATED', 'uid=' + uid)
        self.board.dispatch = dispatch

    def write(self, data):
        self.writes.append(decode(data).body)
        return super().write(data)


class ImuSerial(UsbFixture):
    """Make unplug fail the actual IMU worker read instead of merely flagging it."""
    def read_until(self, delimiter, size):
        if self.closed:
            raise OSError('device unplugged')
        return super().read_until(delimiter, size)


class AutoLinkTests(unittest.TestCase):
    def test_h743_rejects_wrong_board_and_uid_without_initialization_or_stop(self):
        for board, uid, expected in [('OTHER', UID, None), ('H743_CB_V1.5', 'F' * 24, UID)]:
            stream = RobotSerial(board, uid)
            events = queue.Queue()
            worker = Connection('USB_TEST', events, expected_board='H743_CB_V1.5', expected_uid=expected)
            with patch('serial.Serial', return_value=stream), patch('host.imu.usb_ports', return_value=[]):
                worker.start()
                worker.join(2.)
            self.assertFalse(worker.is_alive())
            self.assertFalse(worker.identified)
            self.assertTrue(set(stream.writes).issubset({'HELLO', 'INFO'}))
            self.assertNotIn('connected', [event[0] for event in list(events.queue)])
            self.assertNotIn('reply', [event[0] for event in list(events.queue)])

    def test_h723_rejects_uid_before_publishing_samples(self):
        candidate = {'id': 'IMU_TEST', 'serial_number': 'USB123'}
        hub = ImuHub(port_provider=lambda: [candidate], serial_factory=lambda p: ImuSerial())
        self.addCleanup(hub.close)
        hub.expected_uid = 'F' * 24
        hub.connect('IMU_TEST')
        self.assertTrue(wait(lambda: hub.state == 'error'))
        self.assertIsNone(hub.latest)
        self.assertEqual(hub.identity, {})

    def test_two_links_retry_forever_reconnect_renumbered_and_persist_pairing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            console = Console(root / 'nav.json', root / 'aux.json')
            ports, robots, imus = [], [], []
            path = root / 'links.json'
            manager = AutoDevices(console, path=path, port_provider=lambda: list(ports), interval=.02)
            console.autolink = manager

            def robot_factory(*args, **kwargs):
                stream = RobotSerial()
                robots.append(stream)
                return stream

            def imu_factory(port):
                stream = ImuSerial()
                imus.append(stream)
                return stream

            console.imu = ImuHub(port_provider=candidate_ports, serial_factory=imu_factory)

            def pump():
                console.drain()
                manager.tick()
                return (console.device.get('_serial') == 'ROBOT2' and console.imu.state == 'streaming'
                        and console.imu.port == 'IMU2')

            def attach(suffix):
                ports[:] = [SimpleNamespace(device='ROBOT' + suffix, vid=0x1A86, pid=0x7523,
                                            description='CH340', serial_number=None),
                            SimpleNamespace(device='IMU' + suffix, vid=0x0483, pid=0x5740,
                                            description='ST', serial_number='USB123'),
                            SimpleNamespace(device='JLINK', vid=0x1366, pid=0x0105,
                                            description='debug', serial_number='debug')]

            try:
                with patch('host.imu.usb_ports', side_effect=lambda: list(ports)), patch('serial.Serial', side_effect=robot_factory):
                    for i in range(100):
                        manager.tick(now=float(i))
                    self.assertEqual(robots, [])
                    attach('1')
                    self.assertTrue(wait(lambda: (pump() or (console.device.get('_serial') == 'ROBOT1'
                                         and console.imu.state == 'streaming')), 3.))
                    # Run another cycle to persist identities after the actual handshakes.
                    time.sleep(.03)
                    pump()
                    self.assertEqual(manager.config['h743_uid'], UID)
                    self.assertEqual(manager.config['h723_uid'], UID)
                    ports.clear()
                    robots[-1].close()
                    imus[-1].close()
                    self.assertTrue(wait(lambda: (pump() or (not console.connection and console.imu.state == 'error')), 2.))
                    attach('2')
                    self.assertTrue(wait(pump, 3.))
                    self.assertFalse(console.armed)
                    self.assertFalse(console.device.get('_ack'))
                    for stream in robots:
                        allowed = {'HELLO', 'INFO', 'STOP', 'LIFT INIT', 'PING',
                                   'SERVO 1 180', 'SERVO 2 0', 'SERVO STAT'}
                        self.assertTrue(all(body in allowed or body.startswith('NAV IMU ')
                                            for body in stream.writes))
                        self.assertFalse(stream.board.armed)
                        self.assertEqual(stream.board.servo_steps, [360, 0])
                        self.assertEqual(stream.writes.count('SERVO 1 180'), 1)
                        self.assertEqual(stream.writes.count('SERVO 2 0'), 1)
                    self.assertEqual(len(robots), 2)
                    self.assertEqual(len(imus), 2)
                    manager.set_enabled(False)
                    self.assertFalse(AutoDevices(console, path=path).snapshot()['enabled'])
                    self.assertEqual(AutoDevices(console, path=path).config['h743_uid'], UID)
            finally:
                manager.set_enabled(False)
                if console.connection:
                    console.connection.close()
                    console.connection.join(2.)
                console.imu.close()

    def test_pause_does_not_open_or_disconnect_devices(self):
        with tempfile.TemporaryDirectory() as directory:
            console = Console(Path(directory) / 'nav.json', Path(directory) / 'aux.json')
            manager = AutoDevices(console, Path(directory) / 'links.json')
            manager.set_enabled(False)
            with patch.object(console, 'connect') as connect, patch.object(console.imu, 'close') as close:
                manager.tick()
            connect.assert_not_called()
            close.assert_not_called()


if __name__ == '__main__':
    unittest.main()
