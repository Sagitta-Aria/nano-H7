"""Execute shared gyro tuning through the compiled firmware command path."""
import unittest
from tests import test_chassis as harness
from tests import test_navigation as nav_harness


class HeadingTuningArmTests(unittest.TestCase):
    setUp = harness.ArmChassisTests.setUp
    request = harness.ArmChassisTests.request

    def test_joint_config_and_legacy_p_preserve_tolerance(self):
        reply = self.request('TURN CONFIG 8000 50')
        self.assertTrue(reply.ok)
        self.assertEqual((reply.fields['kp_centi'], reply.fields['tolerance_cd']), ('8000', '50'))
        self.assertTrue(self.request('TURN CONFIG 10000').ok)
        self.assertEqual(self.request('TURN STAT').fields['tolerance_cd'], '50')
        for body in ('TURN CONFIG -1 50', 'TURN CONFIG 8000 0', 'TURN CONFIG 8000 1001'):
            self.assertFalse(self.request(body).ok)
        self.assertEqual(self.request('TURN STAT').fields['tolerance_cd'], '50')

    def test_configured_precision_changes_actual_completion(self):
        for tolerance, error, expected in ((50,50,'done'),(50,51,'turning'),(10,11,'turning')):
            self.setUp(); self.request('TURN CONFIG 8000 {}'.format(tolerance))
            self.request('NAV IMU 0 1 1'); self.request('TURN MOVE REL 0 1000 5000')
            for now in (0,100,200,300):
                self.arm.write32(self.arm.symbols['xTickCount'],now)
                self.arm.call('navigation_imu',error,1,1,now)
                self.arm.call('turn_tick',now)
            self.assertEqual(self.request('TURN STAT').fields['state'],expected)
            if expected == 'turning':
                self.assertFalse(self.request('TURN CONFIG 2000 100').ok)

    def test_increased_p_changes_low_error_wheel_response(self):
        self.request('TURN CONFIG 8000 20'); self.request('NAV IMU 0 1 1')
        self.request('TURN MOVE REL 100 5000 5000'); self.arm.call('turn_tick',0)
        for _ in range(100): self.arm.call('motion_tick')
        self.assertEqual([abs(self.arm.status(i)[1]) for i in range(4)],[80]*4)

    def test_large_gains_are_applied_and_saturate_without_integer_wrap(self):
        for kp in (20001,100000,2147483647):
            with self.subTest(kp=kp):
                self.assertTrue(self.request(f'TURN CONFIG {kp} 20').ok)
                self.assertEqual(self.request('TURN STAT').fields['kp_centi'],str(kp))
                for error,limit in ((21,5000),(100,777),(18000,5000)):
                    expected=min(limit,max(1,(error*kp+5000)//10000))
                    self.assertEqual(self.arm.call('turn_control_speed',error,limit),expected)
        self.assertFalse(self.request('TURN CONFIG 2147483648 20').ok)
        self.assertEqual(self.request('TURN STAT').fields['kp_centi'],'2147483647')

    def test_zero_gain_removes_floor_and_does_not_fault_or_claim_arrival(self):
        self.assertTrue(self.request('TURN CONFIG 0 20').ok)
        self.assertEqual(self.arm.call('turn_control_speed',100,5000),0)
        self.request('NAV IMU 0 1 1'); self.request('TURN MOVE REL 9000 500 2000')
        self.arm.call('turn_tick',0)
        self.assertEqual(self.arm.call('motion_chassis_active'),0)
        status=self.request('TURN STAT').fields
        self.assertEqual((status['state'],status['fault']),('turning','none'))
        self.arm.call('turn_tick',300)
        self.assertEqual(self.request('TURN STAT').fields['fault'],'imu_stale')


class NavigationTuningArmTests(unittest.TestCase):
    setUp = harness.ArmChassisTests.setUp
    request = harness.ArmChassisTests.request
    configure = nav_harness.ArmNavigationTests.configure
    start = nav_harness.ArmNavigationTests.start
    upload = nav_harness.ArmNavigationTests.upload
    nav = nav_harness.ArmNavigationTests.nav
    tick = nav_harness.ArmNavigationTests.tick

    def test_cruise_uses_same_precision_and_rejects_active_edits(self):
        for tolerance, active in ((50,1),(150,0)):
            self.setUp()
            self.configure([(0,0)]*5, [0,100,0,0,0], initial=0)
            self.assertTrue(self.request('TURN CONFIG 8000 {}'.format(tolerance)).ok)
            self.assertTrue(self.start().ok); self.tick(1)
            self.assertEqual(bool(self.arm.call('motion_chassis_active')),bool(active))
            self.assertFalse(self.request('TURN CONFIG 2000 20').ok)

    def test_zero_gain_stops_cruise_turning_output_but_keeps_feedback_guard(self):
        self.configure([(0,0)]*5,[0,9000,0,0,0],initial=0)
        self.assertTrue(self.request('TURN CONFIG 0 20').ok)
        self.start(); self.tick(1)
        self.assertEqual(self.arm.call('motion_chassis_active'),0)
        self.assertEqual(self.nav()['state'],'orienting')
        self.tick(301,feedback=False)
        self.assertEqual(self.nav()['fault'],'imu_stale')


if __name__ == '__main__': unittest.main()
