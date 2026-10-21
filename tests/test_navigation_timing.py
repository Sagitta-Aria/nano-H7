"""Check real ARM report critical sections; instruction counts are not MCU time."""
import struct
import unittest

from tests import test_navigation as nav_harness
from tests.test_arm import SCRATCH
from unicorn import UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_PRIMASK


class NavigationReportTimingTests(unittest.TestCase):
    def setUp(self):
        """Reach the B-to-C cruise path without driving physical hardware."""
        self.fixture = nav_harness.ArmNavigationTests()
        self.fixture.setUp()
        self.fixture.route = [3]
        self.fixture.configure(
            points=[(0, 0), (0, 14400), (14400, 0), (14400, 28800), (28800, 14400)],
            initial=0,
        )
        for command in ('ARM 0', 'NAV RESET 14400 0',
                        'NAV CONFIG 3000 5000 3000', 'ARM 1'):
            self.assertTrue(self.fixture.request(command).ok, command)
        self.assertTrue(self.fixture.start(sequence=3384779511).ok)
        self.assertEqual(self.fixture.align(0)['state'], 'moving')
        self.arm = self.fixture.arm
        for _ in range(700):
            self.arm.call('motion_tick')
        self.arm.uc.mem_write(self.arm.symbols['x'], struct.pack('<f', 14340.5))
        self.arm.uc.mem_write(self.arm.symbols['y'], struct.pack('<f', 8572.25))

    def observe_report(self, name, after_unlock=None):
        """Observe the compiled formatter and optional task preemption at unlock."""
        arm = self.arm
        formatter = arm.symbols['__0snprintf$8'] & ~1
        stats = {'formatter_masks': [], 'longest_masked': 0,
                 'masked': 0, 'run': 0, 'released': False}

        def observe(uc, address, size, user):
            masked = uc.reg_read(UC_ARM_REG_PRIMASK)
            if address == formatter:
                stats['formatter_masks'].append(masked)
            if masked:
                stats['masked'] += 1
                stats['run'] += 1
                stats['longest_masked'] = max(stats['longest_masked'], stats['run'])
            else:
                if stats['run'] and not stats['released']:
                    stats['released'] = True
                    if after_unlock:
                        after_unlock()
                stats['run'] = 0

        arm.uc.reg_write(UC_ARM_REG_PRIMASK, 0)
        hook = arm.uc.hook_add(UC_HOOK_CODE, observe)
        # Existing translated blocks predate this hook; invalidate them so all
        # executed instructions, including MicroLIB formatting, are observed.
        arm.uc.ctl_remove_cache(0x08000000, 0x08200000)
        try:
            args = (SCRATCH + 8192, 384)
            if name == 'navigation_report':
                args += (self.fixture.now,)
            arm.call(name, *args)
        finally:
            arm.uc.hook_del(hook)
        text = bytes(arm.uc.mem_read(SCRATCH + 8192, 384)).split(b'\0')[0].decode()
        self.assertEqual(arm.uc.reg_read(UC_ARM_REG_PRIMASK), 0)
        return text, stats

    def test_live_reports_format_with_step_interrupts_enabled(self):
        for name in ('navigation_report', 'navigation_align_report',
                     'navigation_inset_report'):
            with self.subTest(report=name):
                _, stats = self.observe_report(name)
                self.assertEqual(stats['formatter_masks'], [0],
                                 'status formatting must allow STEP IRQs')
                # Bound instruction work rather than claiming cycle-accurate
                # timing. This catches a long formatter reentering the lock.
                self.assertLess(stats['longest_masked'], 4000)

    def test_navigation_report_keeps_one_snapshot_after_interrupts_resume(self):
        expected, _ = self.observe_report('navigation_report')
        self.assertIn('x=14341 y=8572', expected)

        def concurrent_update():
            # Model another task changing position/configuration immediately
            # after the report releases its lock, before formatting starts.
            self.arm.uc.mem_write(self.arm.symbols['x'], struct.pack('<f', -999.5))
            self.arm.uc.mem_write(self.arm.symbols['y'], struct.pack('<f', -888.5))
            self.arm.uc.mem_write(self.arm.symbols['points'] + 3 * 8,
                                  struct.pack('<ii', 777, 666))

        actual, stats = self.observe_report('navigation_report', concurrent_update)
        self.assertTrue(stats['released'])
        self.assertEqual(stats['formatter_masks'], [0])
        self.assertEqual(actual, expected, 'all fields must use the same snapshot')


if __name__ == '__main__':
    unittest.main()
