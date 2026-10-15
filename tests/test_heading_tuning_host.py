"""Gyro parameter API/transport regressions without accessing real devices."""
import unittest
from unittest.mock import Mock
from host import turn
from host.webapp import Console
from host.simulator import SimulatedBoard
from host.protocol import encode, decode


class HeadingTuningHostTests(unittest.TestCase):
    def test_command_keeps_exact_hundredths_and_rejects_bad_tolerance(self):
        self.assertEqual(turn.tuning_command('80.25','0.50'),'TURN CONFIG 8025 50')
        for tolerance in (True,0,'0.001',10.01,'NaN','Infinity'):
            with self.assertRaises(ValueError): turn.tuning_command(80,tolerance)

    def test_api_submits_both_settings_and_requires_new_capability(self):
        c=Console(); c.connection=Mock(); c.device={'turn':'1','turn_tuning':'1','turn_precision':'1'}
        self.assertTrue(c.turn_action('configure',kp='80',tolerance='0.5')[0])
        c.connection.submit.assert_called_with('TURN CONFIG 8000 50')
        c.device.pop('turn_precision'); c.connection.submit.reset_mock()
        self.assertFalse(c.turn_action('configure',kp=80,tolerance=.5)[0])
        c.connection.submit.assert_not_called()

    def test_simulator_shared_precision_and_legacy_gain(self):
        b=SimulatedBoard(); seq=0
        def send(body):
            nonlocal seq
            seq+=1; return decode(b.receive(encode(seq,body)))
        self.assertTrue(send('TURN CONFIG 8000 50').ok)
        self.assertEqual(b.turn.tolerance_cd,50)
        self.assertTrue(send('TURN CONFIG 4000').ok)
        self.assertEqual(send('TURN STAT').fields['tolerance_cd'],'50')

    def test_large_and_zero_gain_encode_exactly_with_wire_boundary_only(self):
        for value,expected in (('0',0),('200.01',20001),('1000.50',100050),('21474836.47',2147483647)):
            self.assertEqual(turn.gain_command(value),f'TURN CONFIG {expected}')
        for bad in (True,-1,'20.001','NaN','Infinity','21474836.48','1e999999999'):
            with self.assertRaises(ValueError): turn.gain_command(bad)

    def test_api_queues_large_gain_without_clamping_and_sim_accepts_it(self):
        console=Console(); console.connection=Mock(); console.device={'turn':'1','turn_tuning':'1','turn_precision':'1'}
        self.assertTrue(console.turn_action('configure',kp='1000.50',tolerance='0.20')[0])
        console.connection.submit.assert_called_with('TURN CONFIG 100050 20')
        board=SimulatedBoard()
        reply=decode(board.receive(encode(1,'TURN CONFIG 2147483647 20')))
        self.assertTrue(reply.ok); self.assertEqual(reply.fields['kp_centi'],'2147483647')
        self.assertTrue(decode(board.receive(encode(2,'TURN CONFIG 0 20'))).ok)
        self.assertEqual(board.turn.kp,0)


if __name__ == '__main__': unittest.main()
