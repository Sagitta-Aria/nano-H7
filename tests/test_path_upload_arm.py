"""Execute route transaction scenarios on the built Cortex-M7 AXF."""
import unittest
from tests import test_chassis as chassis_harness
from tests.test_path_upload import RouteUploadCases


class ArmUploadTests(RouteUploadCases, unittest.TestCase):
    request = send = chassis_harness.ArmChassisTests.request

    def setUp(self):
        chassis_harness.ArmChassisTests.setUp(self)
        self.configure_upload()

    def nav(self):
        return self.send('NAV STAT').fields

    def active(self):
        return self.arm.call('motion_chassis_active')

    def advance(self, milliseconds):
        self.arm.write32(self.arm.symbols['xTickCount'],milliseconds)
        self.arm.call('navigation_tick',milliseconds)


if __name__ == '__main__': unittest.main()
