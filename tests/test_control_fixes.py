"""Regression checks at actual command/output and saved/live setting seams."""
import copy
import tempfile
import unittest
from pathlib import Path
from tests import test_chassis as harness
from host.protocol import encode, decode
from host.webapp import Console


class ArmControlFixTests(unittest.TestCase):
    setUp = harness.ArmChassisTests.setUp
    request = harness.ArmChassisTests.request
    group_edge = harness.ArmChassisTests.group_edge

    def test_servo_270_command_reports_ph9_and_writes_ccr2(self):
        self.arm.call('servo_init')
        fields=self.request('SERVO 1 270').fields
        self.assertEqual(fields.get('pin'),'PH9')
        self.assertEqual(self.arm.read32(0x40001838),2500)
        self.assertEqual(self.arm.read32(0x40001834),1500)
        fields=self.request('SERVO 2 180').fields
        self.assertEqual(fields.get('pin'),'PH6')
        self.assertEqual(self.arm.read32(0x40001834),2500)

    def test_idle_armed_aux_limits_take_effect_in_move_layer(self):
        self.assertTrue(self.request('AUX CONFIG 5 0 20').ok)
        self.assertEqual(self.request('AUX STAT 5').fields['max'],'20')
        self.assertFalse(self.request('AUX STEP 5 21 300 1000').ok)
        self.assertFalse(self.request('MOVE 5 21 300 1000').ok)
        self.assertTrue(self.request('AUX STEP 5 20 300 1000').ok)
        self.assertFalse(self.request('AUX CONFIG 5 0 30').ok)

    def test_turn_02_degree_band_and_configured_p_reach_wheel_speed(self):
        self.assertTrue(self.request('TURN CONFIG 4000').ok)
        self.request('NAV IMU 0 1 1')
        self.assertTrue(self.request('TURN MOVE REL 1000 5000 2000').ok)
        self.arm.call('turn_tick',0)
        # 10 degrees * P40 = 400 step/s; old fixed P8 would yield 80.
        self.assertEqual(self.arm.call('turn_control_speed',1000,5000),400)
        for _ in range(400): self.arm.call('motion_tick')
        self.assertEqual([abs(self.arm.status(i)[1]) for i in range(4)],[400]*4)
        self.assertFalse(self.request('TURN CONFIG 2000').ok)
        self.request('TURN STOP')
        self.request('TURN MOVE REL 0 500 2000')
        for now in (0,100,200,300):
            self.arm.write32(self.arm.symbols['xTickCount'],now)
            self.arm.call('navigation_imu',21,1,1,now)
            self.arm.call('turn_tick',now)
        self.assertEqual(self.request('TURN STAT').fields['state'],'turning')

    def test_turn_completes_at_020_but_not_021_degrees(self):
        self.request('NAV IMU 0 1 1'); self.request('TURN MOVE REL 0 500 2000')
        for now in (0,100,200,300):
            self.arm.write32(self.arm.symbols['xTickCount'],now)
            self.arm.call('navigation_imu',20,1,1,now)
            self.arm.call('turn_tick',now)
        fields=self.request('TURN STAT').fields
        self.assertEqual(fields['state'],'done')
        self.assertEqual(fields['tolerance_cd'],'20')

    def test_configured_p_uses_requested_speed_above_old_600_cap(self):
        self.request('TURN CONFIG 4000'); self.request('NAV IMU 0 1 1')
        self.request('TURN MOVE REL 5000 1500 20000'); self.arm.call('turn_tick',0)
        for _ in range(150): self.arm.call('motion_tick')
        self.assertEqual([abs(self.arm.status(i)[1]) for i in range(4)],[1500]*4)


if __name__=='__main__': unittest.main()
