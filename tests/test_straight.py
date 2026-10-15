"""Exercise actual ARM wheel output and finite assisted distance, no real car."""
import unittest
from tests import test_chassis as chassis_harness
from tests import test_navigation as nav_harness
from tests.test_arm import TIM8


class StraightArmTests(unittest.TestCase):
    setUp = chassis_harness.ArmChassisTests.setUp
    request = chassis_harness.ArmChassisTests.request
    group_edge = chassis_harness.ArmChassisTests.group_edge

    def feedback(self, angle=0, now=0, valid=1, epoch=1):
        self.arm.write32(self.arm.symbols['xTickCount'], now)
        self.arm.call('navigation_imu', angle, valid, epoch, now)

    def ramp(self, angle, duration=400, epoch=1):
        for now in range(1, duration+1):
            if now % 100 == 0:
                self.feedback(angle, now, epoch=epoch)
            self.arm.call('chassis_tick', now)
            self.arm.call('motion_tick')

    def test_feature_and_max_sequence_frame_fit(self):
        reply = self.request('HELLO', sequence=4294967295)
        self.assertTrue(reply.ok)
        self.assertEqual(reply.fields['straight'], '1')
        self.assertEqual(reply.fields['fw'], '0.2.18')
        self.assertLessEqual(len(self.arm.transmitted), 384)

    def test_requires_fresh_yaw_and_translation_only(self):
        self.assertFalse(self.request('CHASSIS STRAIGHT FWD 10 500 2000').ok)
        self.feedback()
        for direction in ('CW', 'CCW'):
            self.assertFalse(self.request(f'CHASSIS STRAIGHT {direction} 10 500 2000').ok)
        self.feedback(valid=0)
        self.assertFalse(self.request('CHASSIS STRAIGHT FWD 10 500 2000').ok)
        self.feedback(now=10)
        self.arm.write32(self.arm.symbols['xTickCount'], 310)
        self.assertFalse(self.request('CHASSIS STRAIGHT FWD 10 500 2000').ok)
        self.assertEqual(self.arm.call('motion_chassis_active'), 0)

    def test_zero_p_disables_yaw_mix_while_preserving_translation_and_stale_stop(self):
        self.assertTrue(self.request('TURN CONFIG 0 20').ok)
        self.feedback()
        self.request('CHASSIS STRAIGHT FWD 10000 500 2000')
        self.feedback(-100); self.ramp(-100)
        self.assertEqual([abs(self.arm.status(i)[1]) for i in range(4)],[500]*4)
        self.assertEqual(self.request('CHASSIS STAT').fields['state'],'moving')
        self.arm.call('chassis_tick',700)
        self.assertEqual(self.request('CHASSIS STAT').fields['fault'],'imu_stale')

    def test_yaw_mix_forward_backward_lateral_and_all_inversion_masks(self):
        matrix = {'FWD':(1,1,1,1),'BACK':(-1,-1,-1,-1),'LEFT':(1,-1,-1,1),'RIGHT':(-1,1,1,-1)}
        for mask in (0, 5, 10, 15):
            for direction, translation in matrix.items():
                with self.subTest(mask=mask, direction=direction):
                    self.setUp()
                    self.request(f'CHASSIS CONFIG {mask}')
                    self.feedback()
                    self.assertTrue(self.request(f'CHASSIS STRAIGHT {direction} 10000 500 2000').ok)
                    self.feedback(-100)
                    self.ramp(-100)
                    # P=20, +1 degree => +20 yaw step/s; base=480.
                    expected = [(s*480+(1 if i%2 == 0 else -1)*20)*(-1 if mask & (1<<i) else 1)
                                for i,s in enumerate(translation)]
                    self.assertEqual([self.arm.status(i)[1] for i in range(4)], expected)
                    self.assertEqual(self.request('CHASSIS STAT').fields['state'], 'moving')

    def test_wrap_deadband_gain_speed_cap_and_no_direction_reversal(self):
        self.request('TURN CONFIG 8000 20')
        self.feedback(17950)
        self.request('CHASSIS STRAIGHT FWD 10000 500 2000')
        self.feedback(-17950)
        self.ramp(-17950)
        self.assertEqual([self.arm.status(i)[1] for i in range(4)], [-340,500,-340,500])
        self.assertEqual(self.request('CHASSIS STAT').fields['error_cd'], '-100')
        self.assertFalse(self.request('TURN CONFIG 2000 20').ok)
        # Cross the correction sign while STEP is high; DIR remains unchanged.
        self.group_edge()
        before = [self.arm.status(i)[0] for i in range(4)]
        self.feedback(17850, 401)
        self.arm.call('chassis_tick', 401)
        self.arm.call('motion_tick')
        self.assertEqual([self.arm.status(i)[0] for i in range(4)], before)
        self.assertTrue(all(self.arm.status(i)[1]*(-1 if i in (0,2) else 1)>0 for i in range(4)))
        self.group_edge()
        self.feedback(17940, 402)
        for now in range(402, 702):
            if now%100 == 0: self.feedback(17940, now)
            self.arm.call('chassis_tick', now); self.arm.call('motion_tick')
        self.assertEqual([abs(self.arm.status(i)[1]) for i in range(4)], [500]*4)

    def test_distance_counts_mean_pulses_instead_of_equal_wheel_endpoints(self):
        self.feedback()
        self.request('CHASSIS STRAIGHT FWD 10 500 2000')
        # Differential pulses are retained, not cancelled at individual endpoints.
        for axis, count in ((0,20),(1,5),(2,10),(3,5)):
            for _ in range(count): self.arm.edge(axis); self.arm.edge(axis)
        self.arm.call('chassis_tick', 1)
        self.assertEqual([self.arm.status(i)[0] for i in range(4)], [-20,5,-10,5])
        self.assertEqual(self.arm.call('motion_chassis_active'), 0)
        result = self.request('CHASSIS STAT').fields
        self.assertEqual((result['state'],result['remaining']), ('done','0'))
        self.assertEqual([self.arm.status(i)[0] for i in range(4,7)], [0]*3)

    def test_natural_completion_and_halt_preserve_final_high_pulses(self):
        self.feedback()
        self.request('CHASSIS STRAIGHT FWD 1 500 2000')
        self.group_edge()  # all four last pulses high
        self.assertEqual(self.arm.call('motion_chassis_active'), 15)
        self.assertFalse(self.request('CHASSIS STRAIGHT BACK 1 500 2000').ok)
        self.group_edge()  # complete highs
        self.arm.call('chassis_tick', 1)
        self.assertEqual(self.request('CHASSIS STAT').fields['state'], 'done')
        self.request('CHASSIS STRAIGHT FWD 10 500 2000')
        self.request('MOVE 7 10 100 2000')
        self.group_edge()
        self.request('CHASSIS HALT')
        self.assertEqual(self.arm.call('motion_chassis_active'), 15)
        self.group_edge()
        self.assertEqual(self.arm.call('motion_chassis_active'), 0)
        self.assertEqual(self.arm.status(6)[4], 1)
        self.assertEqual(self.arm.call('motion_armed'), 1)
        self.assertNotEqual(self.request('CHASSIS STAT').fields['remaining'], '0')

    def test_feedback_failures_latch_even_if_good_data_arrives_before_tick(self):
        for failure in ('invalid','epoch','gap','stale','deviation'):
            with self.subTest(failure=failure):
                self.setUp(); self.feedback()
                self.request('CHASSIS STRAIGHT FWD 1000 500 2000')
                if failure == 'invalid': self.feedback(valid=0, now=1)
                elif failure == 'epoch': self.feedback(epoch=2, now=1)
                elif failure == 'gap': self.feedback(now=300)
                elif failure == 'stale': self.arm.call('chassis_tick',300)
                else: self.feedback(501,1); self.arm.call('chassis_tick',1)
                self.feedback(now=301, epoch=2 if failure == 'epoch' else 1)
                self.arm.call('chassis_tick',301)
                result = self.request('CHASSIS STAT').fields
                self.assertEqual(result['state'],'fault')
                self.assertEqual(self.arm.call('motion_chassis_active'),0)

    def test_busy_stop_disarm_and_duplicate_id(self):
        self.feedback()
        self.request('CHASSIS STRAIGHT FWD 10 500 2000',sequence=1000)
        for cmd in ('CHASSIS JOG FWD 500 2000','CHASSIS MOVE FWD 10 500 2000',
                    'TURN MOVE REL 9000 500 2000','WHEELS 100 100 100 100 2000','MOVE 1 10 100 2000'):
            self.assertFalse(self.request(cmd).ok)
        self.group_edge(); self.group_edge()
        remaining = self.request('CHASSIS STAT').fields['remaining']
        self.assertTrue(self.request('CHASSIS STRAIGHT FWD 10 500 2000',sequence=1000).ok)
        self.assertEqual(self.request('CHASSIS STAT').fields['remaining'],remaining)
        self.request('ARM 0'); self.arm.call('chassis_tick',1)
        self.assertEqual(self.arm.call('motion_chassis_active'),0)
        self.assertEqual(self.request('CHASSIS STAT').fields['state'],'stopped')

    def test_independent_timer_edges_accelerate_brake_and_keep_yaw_pulses(self):
        self.feedback()
        self.request('CHASSIS STRAIGHT FWD 500 500 2000')
        cursor, peak, late_speed = 0, 0, None
        for now in range(1,5001):
            # Process independent compare times, including simultaneous flags.
            # This is register/ISR execution, not a common-edge shortcut.
            end = now*50
            while self.arm.call('motion_chassis_active'):
                deltas = {i:(self.arm.read32(TIM8+0x34+4*i)-cursor)&0xffff
                          for i in range(4) if self.arm.status(i)[4]}
                delta = min(deltas.values())
                if delta > end-cursor: break
                cursor += delta
                self.arm.write32(TIM8+0x24,cursor & 0xffff)
                self.arm.write32(TIM8+0x10,sum(2<<i for i,d in deltas.items() if d==delta))
                self.arm.call('TIM8_CC_IRQHandler')
            cursor=end; self.arm.write32(TIM8+0x24,cursor & 0xffff)
            if now%100==0: self.feedback(-100 if now<600 else 0,now)
            self.arm.call('chassis_tick',now); self.arm.call('motion_tick')
            speeds=[abs(self.arm.status(i)[1]) for i in range(4)]
            peak=max(peak,*speeds)
            if self.arm.status(0)[2] in range(1,20): late_speed=max(speeds)
            self.assertEqual(self.arm.call('motion_faults'),0)
            if not self.arm.call('motion_chassis_active'): break
        else: self.fail('assisted finite motion did not finish')
        pulses=[self.arm.status(i)[0]*(-1 if i in (0,2) else 1) for i in range(4)]
        self.assertEqual(sum(pulses),2000)
        self.assertGreater(pulses[0],pulses[1])
        self.assertGreater(peak,400); self.assertLessEqual(peak,500)
        self.assertIsNotNone(late_speed); self.assertLess(late_speed,peak)
        self.assertEqual(self.request('CHASSIS STAT').fields['state'],'done')


class CruiseStraightArmTests(unittest.TestCase):
    setUp = nav_harness.ArmNavigationTests.setUp
    request = nav_harness.ArmNavigationTests.request
    group_edge = nav_harness.ArmNavigationTests.group_edge
    configure = nav_harness.ArmNavigationTests.configure
    upload = nav_harness.ArmNavigationTests.upload
    start = nav_harness.ArmNavigationTests.start
    nav = nav_harness.ArmNavigationTests.nav
    tick = nav_harness.ArmNavigationTests.tick
    align = nav_harness.ArmNavigationTests.align

    def test_cruise_corrects_small_drift_in_existing_map_frame(self):
        self.configure()
        self.start(); self.align(0)
        self.assertEqual(self.nav()['state'],'moving')
        for now in range(self.now+1,self.now+301):
            self.tick(now,100); self.arm.call('motion_tick')
        velocities = [self.arm.status(i)[1] for i in range(4)]
        self.assertGreater(abs(velocities[1]),abs(velocities[0]))
        self.assertEqual(self.nav()['target_cd'],'0')
        self.assertEqual(self.nav()['state'],'moving')
        self.tick(self.now+1,501)
        self.assertEqual(self.nav()['fault'],'heading_deviation')

    def test_cruise_does_not_recapture_residual_alignment_error(self):
        self.configure(initial=17950)
        self.start(); self.align(19)  # within the shared 0.2-degree completion band
        self.assertEqual(self.nav()['state'],'moving')
        for now in range(self.now+1,self.now+301):
            self.tick(now,100); self.arm.call('motion_tick')
        velocities = [abs(self.arm.status(i)[1]) for i in range(4)]
        self.assertEqual(velocities[1]-velocities[0],40) # map error=1 degree, not 0.81


if __name__ == '__main__': unittest.main()
