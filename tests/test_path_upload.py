"""Full-route transaction safety, shared by demo and actual ARM image tests."""
import copy
import tempfile
import unittest
from pathlib import Path

from host.navigation import DEFAULT, MissionConfig, mission_waypoints, path_checksum
from host.protocol import encode, decode
from host.simulator import SimulatedBoard
from host.webapp import Console


class RouteUploadCases:
    """Protocol scenarios run identically on the MCU image and the demo."""
    def configure_upload(self):
        self.config = copy.deepcopy(DEFAULT)
        self.config['grid'] = {'x': [0,14,28], 'y': [0,14,28]}
        coords = ([0,0],[0,14],[14,0],[14,28],[28,14])
        self.config['stations'] = [{'name': name, 'x': c[0], 'y': c[1], 'heading': 0}
                                   for name, c in zip('OABCD', coords)]
        self.entries = mission_waypoints(self.config, (0,0))
        self.send('ARM 0')
        self.assertTrue(self.send('NAV RESET 0 0').ok)
        for axis in ('X','Y'): self.send('NAV GRID {} 0 14 28'.format(axis))
        self.declare_tables()
        self.send('NAV IMU 0 1 1')
        self.send('PING'); self.send('ARM 1')

    def declare_tables(self, coords=None, route=(1,2,3,4,1)):
        """Counts precede rows; both tables must be complete before BEGIN."""
        stations = self.config['stations'] if coords is None else [
            {'x': c[0], 'y': c[1]} for c in coords]
        self.assertTrue(self.send('NAV STATIONS {}'.format(len(stations))).ok)
        for i, s in enumerate(stations):
            self.assertTrue(self.send('NAV POINT {} {} {}'.format(i, s['x'], s['y'])).ok)
            self.assertTrue(self.send('NAV ATTITUDE {} 0'.format(i)).ok)
        self.assertTrue(self.send('NAV LEGS {}'.format(len(route))).ok)
        for i, station in enumerate(route):
            self.assertTrue(self.send('NAV LEG {} {} 0'.format(i, station)).ok)

    def stage(self, entries=None, ident=7):
        entries = self.entries if entries is None else entries
        self.assertTrue(self.send('NAV BEGIN {} {} 0 0'.format(ident,len(entries))).ok)
        for i, p in enumerate(entries):
            self.assertTrue(self.send('NAV WP {} {} {} {} {}'.format(ident,i,*p)).ok)

    def commit(self, entries=None, ident=7):
        return self.send('NAV COMMIT {} {}'.format(ident,path_checksum(self.entries if entries is None else entries)))

    def test_requires_complete_committed_path_and_explicit_start(self):
        self.assertFalse(self.send('NAV START').ok)
        self.stage()
        self.assertFalse(self.send('NAV START 7').ok)
        self.assertTrue(self.commit().ok)
        self.assertEqual(self.nav()['route_ready'], '1')
        self.assertFalse(self.active())
        self.assertFalse(self.send('NAV START 8').ok)
        self.assertTrue(self.send('NAV START 7').ok)
        self.assertFalse(self.send('NAV BEGIN 9 5 0 0').ok)
        self.assertFalse(self.commit().ok)
        self.send('NAV STOP')
        self.assertFalse(self.send('NAV START 7').ok)

    def test_capacity_origin_order_wrong_id_and_incomplete(self):
        for command in ('NAV BEGIN 7 4 0 0', 'NAV BEGIN 7 65 0 0', 'NAV BEGIN 7 5 0 1',
                        'NAV BEGIN 7 5 1 1'):
            self.assertFalse(self.send(command).ok)
        self.send('NAV BEGIN 7 {} 0 0'.format(len(self.entries)))
        self.assertFalse(self.send('NAV WP 7 1 0 14 0').ok)
        self.assertFalse(self.send('NAV WP 8 0 0 14 0').ok)
        self.assertTrue(self.send('NAV WP 7 0 0 14 0').ok)
        self.assertFalse(self.commit().ok)
        self.assertFalse(self.send('NAV START').ok)
        self.assertFalse(self.active())

    def test_checksum_and_replay_does_not_append_twice(self):
        self.send('NAV BEGIN 7 {} 0 0'.format(len(self.entries)))
        body = 'NAV WP 7 0 {} {} {}'.format(*self.entries[0])
        self.assertTrue(self.send(body, sequence=9000).ok)
        self.assertTrue(self.send(body, sequence=9000).ok)
        for i,p in enumerate(self.entries[1:],1): self.assertTrue(self.send('NAV WP 7 {} {} {} {}'.format(i,*p)).ok)
        self.assertFalse(self.send('NAV COMMIT 7 {}'.format(path_checksum(self.entries)^1)).ok)
        self.assertFalse(self.send('NAV START').ok)
        self.assertTrue(self.commit().ok)

    def test_rejects_diagonal_interior_missing_arrivals_and_leg_jumps(self):
        bad_routes = []
        for first in ((14,14,0), (7,7,0), (0,28,0), (0,14,1)):
            entries = list(self.entries); entries[0] = first; bad_routes.append(entries)
        entries = list(self.entries); entries[-1] = (28,14,4); bad_routes.append(entries)
        for entries in bad_routes:
            self.stage(entries)
            self.assertFalse(self.commit(entries).ok, entries)
            self.assertFalse(self.send('NAV START').ok)
            self.assertFalse(self.active())

    def test_stop_and_config_changes_cancel_staged_and_committed_routes(self):
        for command in ('NAV STOP', 'STOP', 'ARM 0', 'NAV GRID X 0 14 28',
                        'NAV POINT 1 0 14', 'NAV ATTITUDE 1 0', 'NAV CONFIG 500 2000 3000'):
            self.setUp()
            self.stage(); self.send(command)
            self.assertFalse(self.commit().ok, command)
            self.send('PING'); self.send('ARM 1')
            self.stage(); self.assertTrue(self.commit().ok)
            self.send(command)
            self.send('PING'); self.send('ARM 1')
            self.assertFalse(self.send('NAV START 7').ok, command)

    def test_upload_timeout_and_imu_session_change_cancel(self):
        self.stage(); self.advance(5000)
        self.assertFalse(self.commit().ok)
        self.setUp(); self.stage(); self.send('NAV IMU 0 1 2')
        self.assertFalse(self.commit().ok)
        self.setUp(); self.stage(); self.assertTrue(self.commit().ok)
        self.send('NAV IMU 0 1 2')
        self.assertFalse(self.send('NAV START 7').ok)

    def test_uploaded_legal_detour_is_executed_without_shortest_path_search(self):
        entries = [(14,0,0), (14,14,0), (0,14,0)] + self.entries[1:]
        self.stage(entries); self.assertTrue(self.commit(entries).ok)
        self.assertTrue(self.send('NAV START 7').ok)
        status = self.nav()
        self.assertEqual(status['planner'], 'host_grid')
        self.assertEqual((status['wx'],status['wy'],status['target_cd']), ('14','0','9000'))
        self.assertEqual(status['segments'], '3')
        self.assertEqual(status['path_id'], '7')

    def test_coincident_arrivals_preserve_all_five_legs(self):
        for i in range(5): self.send('NAV POINT {} 0 0'.format(i))
        entries = [(0,0,i) for i in range(5)]
        self.stage(entries); self.assertTrue(self.commit(entries).ok)
        self.assertTrue(self.send('NAV START 7').ok)
        self.assertEqual(self.nav()['state'], 'orienting')
        self.assertEqual(self.nav()['segments'], '0')

    def test_maximum_route_capacity_and_extreme_report(self):
        for i in range(5): self.send('NAV POINT {} 0 0'.format(i))
        entries = [(14 if i % 2 == 0 else 0,0,0) for i in range(60)] + [(0,0,i) for i in range(1,5)]
        self.stage(entries, 2147483647)
        self.assertTrue(self.commit(entries,2147483647).ok)
        self.assertTrue(self.send('NAV START 2147483647',sequence=4294967295).ok)
        self.assertEqual(self.nav()['path_id'], '2147483647')
        self.assertEqual(self.nav()['segments'], '60')


class DemoUploadTests(RouteUploadCases, unittest.TestCase):
    def setUp(self):
        self.now, self.sequence = 0., 100
        self.board = SimulatedBoard(clock=lambda:self.now)
        self.configure_upload()

    def send(self, body, sequence=None):
        self.sequence += 1
        return decode(self.board.receive(encode(sequence or self.sequence,body)))

    def nav(self):
        return self.send('NAV STAT').fields

    def active(self):
        return any(a['active'] for a in self.board.axes[:4])

    def advance(self, milliseconds):
        self.now += milliseconds/1000
        self.board.update()


class HostPlanningUploadTests(unittest.TestCase):
    def test_fresh_pose_is_used_and_failed_ack_never_reaches_start(self):
        class Worker:
            def __init__(self): self.sent=[]
            def submit(self, body, tag=None): self.sent.append((body,tag)); return True
        with tempfile.TemporaryDirectory() as directory:
            console = Console(Path(directory)/'nav.json')
            console.connection=Worker(); console.armed=True
            console.device={'nav_heading':'1','nav_grid':'1','nav_path':'1','_port':'SIM'}
            console.navigation={'ready':'1','state':'idle','x':'0','y':'0'}
            self.assertTrue(console.navigation_action('start')[0])
            command,tag=console.connection.sent[-1]
            self.assertEqual(command,'NAV STAT')
            console._reply(command,decode(encode(1,'OK cmd=NAV_STAT ready=1 state=idle x=0 y=7000 stations=5 held=0')),tag)
            self.assertIn('NAV BEGIN 2 7 0 7000',console.nav_upload)
            command,tag=console.connection.sent[-1]
            console._reply(command,decode(encode(2,'ERR code=NAV_BUSY')),tag)
            self.assertFalse(console.nav_pending)
            self.assertFalse(console.nav_upload)
            self.assertFalse(any(body.startswith('NAV START') for body,_ in console.connection.sent))

    def test_off_grid_fresh_pose_aborts_before_configuration(self):
        class Worker:
            def __init__(self): self.sent=[]
            def submit(self,body,tag=None): self.sent.append((body,tag)); return True
        with tempfile.TemporaryDirectory() as directory:
            console=Console(Path(directory)/'nav.json'); console.connection=Worker(); console.armed=True
            console.device={'nav_heading':'1','nav_grid':'1','nav_path':'1','_port':'SIM'}
            console.navigation={'ready':'1','state':'idle'}
            console.navigation_action('start')
            command,tag=console.connection.sent[-1]
            console._reply(command,decode(encode(1,'OK cmd=NAV_STAT ready=1 state=idle x=1 y=1 stations=5 held=0')),tag)
            self.assertFalse(console.nav_pending)
            self.assertEqual(len(console.connection.sent),1)


if __name__ == '__main__': unittest.main()
