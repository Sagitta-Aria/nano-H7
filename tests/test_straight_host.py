"""Protocol/API/demo ownership checks; safe to run on Nano with no real I/O."""
import unittest
import queue
from unittest.mock import Mock
from host.protocol import encode, decode
from host.simulator import SimulatedBoard
from host.straight import straight_command
from host.transport import Connection
from host.webapp import Console


class StraightHostTests(unittest.TestCase):
    def test_protocol_validation(self):
        self.assertEqual(straight_command('BACK',100,500,2000),'CHASSIS STRAIGHT BACK 100 500 2000')
        for args in (('CW',100,500,2000),('FWD',True,500,2000),('FWD',1.2,500,2000),
                     ('FWD',0,500,2000),('FWD',100,5001,2000),('FWD',100,500,0)):
            with self.assertRaises(ValueError): straight_command(*args)

    def test_web_rejects_old_firmware_and_bad_direction_without_queueing(self):
        console=Console(); console.connection=Mock(); console.device={'chassis':'1'}
        self.assertFalse(console.submit('CHASSIS STRAIGHT FWD 100 500 2000')[0])
        console.connection.submit.assert_not_called()
        console.device['straight']='1'
        self.assertFalse(console.submit('CHASSIS STRAIGHT CW 100 500 2000')[0])
        self.assertFalse(console.submit('CHASSIS STRAIGHT FWD 100.5 500 2000')[0])
        console.connection.submit.assert_not_called()
        self.assertTrue(console.submit('CHASSIS STRAIGHT FWD 100 500 2000')[0])
        console.connection.submit.assert_called_with('CHASSIS STRAIGHT FWD 100 500 2000')

    def test_web_cruise_upload_excludes_manual_straight(self):
        console=Console(); console.connection=Mock(); console.device={'straight':'1'}
        console.nav_pending=True
        self.assertFalse(console.submit('CHASSIS STRAIGHT FWD 100 500 2000')[0])
        console.connection.submit.assert_not_called()
        self.assertTrue(console.submit('CHASSIS HALT')[0])

    def test_stop_and_disarm_cancel_queued_assisted_motion(self):
        for stop in ('STOP','CHASSIS HALT','NAV STOP','ARM 0'):
            connection=Connection('SIM',queue.Queue())
            connection.submit('CHASSIS STRAIGHT FWD 100 500 2000')
            connection.submit(stop)
            self.assertEqual(connection.commands.qsize(),1)
            self.assertEqual(connection.commands.get_nowait()[2],stop)


class StraightDemoTests(unittest.TestCase):
    def setUp(self):
        self.now,self.seq=0.,0
        self.board=SimulatedBoard(clock=lambda:self.now)
        self.send('NAV IMU 17950 1 1'); self.send('PING'); self.send('ARM 1')

    def send(self,body):
        self.seq+=1
        return decode(self.board.receive(encode(self.seq,body)))

    def advance(self,angle,ms=100,valid=1,epoch=1):
        self.now+=ms/1000
        self.board.heartbeat=self.now
        self.board.navigation.feedback(angle/100,valid,epoch,self.now)
        self.board.update()

    def test_actual_differential_wheel_motion_and_exact_mean_distance(self):
        self.assertTrue(self.send('CHASSIS STRAIGHT FWD 100 500 2000').ok)
        for _ in range(20):
            self.advance(-17950)
            if self.board.chassis_state=='done': break
        status=self.send('CHASSIS STAT').fields
        self.assertEqual((status['state'],status['remaining']),('done','0'))
        pulses=[a['pos']*(-1 if i in (0,2) else 1) for i,a in enumerate(self.board.axes[:4])]
        self.assertEqual(sum(pulses),400)
        self.assertGreater(pulses[1],pulses[0])
        self.assertEqual([a['pos'] for a in self.board.axes[4:]], [0]*3)

    def test_deadband_equal_profiles_and_backward_correction(self):
        self.send('CHASSIS STRAIGHT FWD 10000 500 2000')
        for _ in range(4): self.advance(17960)
        self.assertEqual([abs(a['velocity']) for a in self.board.axes[:4]],[500]*4)
        self.send('CHASSIS HALT')
        self.send('CHASSIS STRAIGHT BACK 10000 500 2000')
        for _ in range(4): self.advance(17860)
        speeds=[a['velocity'] for a in self.board.axes[:4]]
        self.assertGreater(abs(speeds[1]),abs(speeds[0]))
        self.assertGreater(speeds[0],0); self.assertLess(speeds[1],0)

    def test_invalid_session_gap_and_stale_feedback_stop_without_resuming(self):
        for reason in ('imu_invalid','imu_changed','imu_stale'):
            self.setUp()
            self.send('CHASSIS STRAIGHT FWD 1000 500 2000')
            self.advance(17950,ms=301 if reason=='imu_stale' else 100,
                         valid=reason!='imu_invalid', epoch=2 if reason=='imu_changed' else 1)
            self.assertEqual(self.send('CHASSIS STAT').fields['fault'],reason)
            self.advance(17950,epoch=2 if reason=='imu_changed' else 1)
            self.assertEqual(self.send('CHASSIS STAT').fields['state'],'fault')
            self.assertFalse(any(a['active'] for a in self.board.axes[:4]))

    def test_large_error_and_motor_ownership(self):
        self.send('CHASSIS STRAIGHT FWD 1000 500 2000')
        self.assertFalse(self.send('TURN MOVE REL 1000 500 2000').ok)
        self.assertFalse(self.send('WHEELS 100 100 100 100 2000').ok)
        self.advance(17349)
        self.assertEqual(self.send('CHASSIS STAT').fields['fault'],'heading_deviation')


if __name__=='__main__': unittest.main()
