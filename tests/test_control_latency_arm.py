"""Fresh-feedback completion budgets against the compiled H743 image."""
import unittest
from tests import test_turn_inset as arm_fixture


class ArmLatencyTests(unittest.TestCase):
    setUp = arm_fixture.GenericArmTests.setUp
    request = arm_fixture.GenericArmTests.request
    configure = arm_fixture.GenericArmTests.configure
    upload = arm_fixture.GenericArmTests.upload
    nav = arm_fixture.GenericArmTests.nav
    tick = arm_fixture.GenericArmTests.tick
    group_edge = arm_fixture.GenericArmTests.group_edge
    prepare = arm_fixture.GenericArmTests.prepare
    execute = arm_fixture.GenericArmTests.execute

    def test_turn_finishes_after_three_fresh_stationary_samples(self):
        self.prepare(); self.execute(0)
        started = self.now
        for _ in range(2):
            self.tick(self.now + 50, 0)
            self.assertEqual(self.nav()['state'], 'turning')
        self.tick(self.now + 50, 0)
        print('aligned turn completion (compiled ARM): {}ms'.format(self.now - started))
        self.assertEqual(self.nav()['state'], 'checkpoint')

