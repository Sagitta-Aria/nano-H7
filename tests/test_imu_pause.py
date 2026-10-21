"""Exercise resumable yaw loss in the actual compiled ARM image, without hardware."""
import unittest
import struct

from tests import test_navigation as nav_harness, test_turn as turn_harness, test_chassis as chassis_harness
from tests.test_arm import SCRATCH, TIM8


class ImuPauseArmTests(unittest.TestCase):
    setUp = nav_harness.ArmNavigationTests.setUp
    request = nav_harness.ArmNavigationTests.request
    configure = nav_harness.ArmNavigationTests.configure
    upload = nav_harness.ArmNavigationTests.upload
    start = nav_harness.ArmNavigationTests.start
    nav = nav_harness.ArmNavigationTests.nav
    tick = nav_harness.ArmNavigationTests.tick
    align = nav_harness.ArmNavigationTests.align
    group_edge = nav_harness.ArmNavigationTests.group_edge
    finish_forward = nav_harness.ArmNavigationTests.finish_forward
    def test_navigation_keeps_route_and_exact_motion_on_timeout(self):
        self.configure(); self.start(); self.align(0)
        self.group_edge(); self.group_edge()
        before = [self.arm.status(i) for i in range(4)]
        self.tick(self.now+300, feedback=False)
        paused = self.nav()
        self.assertEqual(paused.get('paused'), '1')
        self.assertEqual(paused['fault'], 'none')
        self.assertEqual(paused['ready'], '1')
        self.assertEqual([self.arm.status(i)[0] for i in range(4)], [s[0] for s in before])
        for _ in range(5): self.group_edge()
        self.assertEqual([self.arm.status(i)[0] for i in range(4)], [s[0] for s in before])
        # A long pause must not consume the 60-second action budget.
        self.tick(self.now+120000, feedback=False)
        for _ in range(3): self.tick(self.now+60)
        self.assertEqual(self.nav().get('paused'), '0')
        self.assertEqual(self.nav()['move_id'], paused['move_id'])
        self.finish_forward(0)
        self.assertNotEqual(self.nav()['state'], 'fault', self.nav())


class StandalonePauseArmTests(unittest.TestCase):
    setUp = turn_harness.ArmTurnTests.setUp
    request = turn_harness.ArmTurnTests.request
    feed = turn_harness.ArmTurnTests.feed
    group_edge = turn_harness.ArmTurnTests.group_edge

    def test_relative_turn_keeps_original_target_and_id(self):
        self.feed(0, 0)
        self.request('TURN MOVE REL 500 500 2000')
        before = self.feed(10, 100)
        self.request('NAV IMU 100 0 1')
        paused = self.request('TURN STAT').fields
        self.assertEqual(paused.get('paused'), '1')
        self.assertEqual(paused['fault'], 'none')
        for t in (120010, 120070, 120130): self.feed(t, 100)
        resumed = self.request('TURN STAT').fields
        self.assertEqual(resumed.get('paused'), '0')
        self.assertEqual(resumed['target_cd'], '500')
        self.assertEqual(resumed['move_id'], before['move_id'])
        self.assertEqual(resumed['error_cd'], '400')

    def test_visual_band_is_per_command_and_smaller_than_shared_band(self):
        self.feed(0, 0)
        self.request('TURN CONFIG 20000 50 100')
        self.assertTrue(self.request('TURN MOVE REL 23 2000 4000 10').ok)
        self.feed(1, 0)
        self.assertTrue(self.arm.call('motion_chassis_active'))
        status = self.request('TURN STAT').fields
        self.assertEqual(status['tolerance_cd'], '50')
        self.assertEqual(status['task_tolerance_cd'], '10')


class PulseHoldArmTests(unittest.TestCase):
    """Check actual compiled IRQ accounting; MMIO does not prove pin timing."""
    setUp = chassis_harness.ArmChassisTests.setUp
    request = chassis_harness.ArmChassisTests.request
    group_edge = chassis_harness.ArmChassisTests.group_edge

    def pending_edge(self):
        """Model hardware compare having fired before the CPU enters its IRQ."""
        self.arm.write32(TIM8+0x24, self.arm.read32(TIM8+0x34))
        self.arm.write32(TIM8+0x10, 2)

    def test_pending_rise_and_fall_keep_exact_remainder_and_aux_keeps_running(self):
        for pending_fall in (False, True):
            with self.subTest(pending_fall=pending_fall):
                self.setUp()
                self.assertTrue(self.request('MOVE 1 4 100 2000').ok)
                self.assertTrue(self.request('MOVE 5 3 100 2000').ok)
                move_id = self.arm.status(0)[3]
                if pending_fall:
                    self.arm.edge(0)
                self.pending_edge()
                self.arm.call('motion_chassis_hold', 1)
                self.assertEqual(self.arm.status(0)[0:3], (1, 0, 3))
                if not pending_fall:
                    self.arm.edge(0)  # Finish the one outstanding STEP high.
                self.assertEqual(self.arm.read32(TIM8+0x0c) & 2, 0)
                before = self.arm.status(0)
                for _ in range(6):
                    self.arm.edge(0)
                    self.arm.edge(4)
                self.assertEqual(self.arm.status(0), before)
                self.assertEqual(self.arm.status(4)[0], 3)
                self.arm.call('motion_chassis_hold', 0)
                for _ in range(6): self.arm.edge(0)
                self.assertEqual(self.arm.status(0), (4, 0, 0, move_id, 0))
                self.assertEqual(self.arm.call('motion_faults'), 0)

    def test_unequal_vector_resume_preserves_launch_ratio_and_pulse_budget(self):
        steps = (60, 20, 20, 60)
        self.arm.uc.mem_write(SCRATCH, struct.pack('<4i', *steps))
        self.arm.uc.mem_write(SCRATCH+64, struct.pack('<4i', 1, -1, 1, -1))
        self.assertEqual(self.arm.call('motion_vector_start', SCRATCH, SCRATCH+64, 100, 2000, 779), 1)
        launch_speeds = [self.arm.status(i)[1] for i in range(4)]
        self.arm.call('motion_chassis_hold', 1)
        before = [self.arm.status(i) for i in range(4)]
        self.assertEqual(self.arm.read32(TIM8+0x0c) & 0x1e, 0)
        for _ in range(6): self.group_edge()
        self.assertEqual([self.arm.status(i) for i in range(4)], before)
        self.arm.call('motion_chassis_hold', 0)
        self.assertEqual([self.arm.status(i)[1] for i in range(4)], launch_speeds)
        positions = self.finish_vector()
        # Compare against an uninterrupted launch using the actual ISR. The
        # vector endpoint contract is a weighted projection, not four separate
        # exact endpoints; timer quantization is identical in the reference.
        self.setUp()
        self.arm.uc.mem_write(SCRATCH, struct.pack('<4i', *steps))
        self.arm.uc.mem_write(SCRATCH+64, struct.pack('<4i', 1, -1, 1, -1))
        self.assertEqual(self.arm.call('motion_vector_start', SCRATCH, SCRATCH+64, 100, 2000, 779), 1)
        self.assertEqual(positions, self.finish_vector())
        self.assertGreaterEqual(sum(p*s for p,s in zip(positions, steps)), sum(s*s for s in steps))
        self.assertLess(sum(p*s for p,s in zip(positions, steps)), sum(s*s for s in steps)+max(steps))
        self.assertEqual([self.arm.status(i)[3] for i in range(4)], [779]*4)
        self.assertEqual(self.arm.call('motion_chassis_active'), 0)

    def finish_vector(self):
        """Execute independent enabled compares and the 1 kHz planner to idle."""
        cursor = self.arm.read32(TIM8+0x24)
        for now in range(1, 5001):
            end = cursor+50
            while self.arm.call('motion_chassis_active'):
                enabled = self.arm.read32(TIM8+0x0c)
                deltas = {i:(self.arm.read32(TIM8+0x34+4*i)-cursor)&0xffff
                          for i in range(4) if enabled & (2<<i)}
                if not deltas: break
                delta = min(deltas.values())
                if delta > end-cursor: break
                cursor += delta
                self.arm.write32(TIM8+0x24, cursor & 0xffff)
                self.arm.write32(TIM8+0x10, sum(2<<i for i,d in deltas.items() if d==delta))
                self.arm.call('TIM8_CC_IRQHandler')
            cursor = end
            self.arm.write32(TIM8+0x24, cursor & 0xffff)
            self.arm.call('motion_tick')
            self.assertEqual(self.arm.call('motion_faults'), 0)
            if not self.arm.call('motion_chassis_active'): break
        else: self.fail('resumed vector did not finish')
        return [self.arm.status(i)[0] for i in range(4)]

    def test_stop_while_held_cannot_revive_reserved_motion(self):
        self.request('MOVE 1 4 100 2000')
        self.arm.call('motion_chassis_hold', 1)
        self.request('STOP')
        self.arm.call('motion_chassis_hold', 0)
        for _ in range(8): self.arm.edge(0)
        self.assertEqual(self.arm.status(0)[0], 0)
        self.assertEqual(self.arm.call('motion_armed'), 0)
        self.assertEqual(self.arm.call('motion_chassis_active'), 0)

    def test_max_sequence_status_frames_fit_wire_buffer(self):
        self.arm.uc.mem_map(0x1ff1e000, 0x1000)
        self.arm.uc.mem_write(0x1ff1e800, struct.pack('<3I', 1, 2, 3))
        self.arm.uc.mem_write(0x1ff1e880, struct.pack('<H', 2048))
        for i, body in enumerate(('HELLO', 'INFO', 'NAV STAT', 'TURN STAT', 'CHASSIS STAT')):
            with self.subTest(body=body):
                reply = self.request(body, 0xffffffff-i)
                self.assertTrue(reply.ok, reply.body)
                self.assertLessEqual(len(self.arm.transmitted), 384)
                if body == 'INFO':
                    self.assertEqual(reply.fields['chassis_max'], '5000')
                    self.assertEqual(reply.fields['aux_max'], '3000')


