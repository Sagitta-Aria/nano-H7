"""Execute mechanism commands while navigation is suspended in the compiled AXF."""
import unittest
from host.pick_place import DEFAULT, cycle_steps
from tests import test_mission_arm as fixture
from tests import test_auxiliary as aux_fixture


class MaterialHandoffArmTests(unittest.TestCase):
    setUp = fixture.ArmMissionTests.setUp
    request = fixture.ArmMissionTests.request
    group_edge = fixture.ArmMissionTests.group_edge
    configure = fixture.ArmMissionTests.configure
    upload = fixture.ArmMissionTests.upload
    nav = fixture.ArmMissionTests.nav
    tick = fixture.ArmMissionTests.tick
    align = fixture.ArmMissionTests.align
    finish_forward = fixture.ArmMissionTests.finish_forward
    held = fixture.ArmMissionTests.held
    external_move = fixture.ArmMissionTests.external_move
    finish_axis = aux_fixture.ArmAuxiliaryTests.finish

    def test_pick_place_during_suspend_retains_pose_and_restores_navigation(self):
        self.arm.call('servo_init')
        for body in ('ARM 0','LIFT DIR -1','AUX ZERO 6','AUX CONFIG 6 -100 0'):
            self.assertTrue(self.request(body).ok, body)
        self.held()
        self.assertTrue(self.request('NAV SUSPEND 42 0').ok)
        config = dict(DEFAULT, pick_down=20, place_take_down=20, place_drop_down=30,
                      pick_rotation=-10, place_rotation=-10)
        for mode, item in (('pick',3),('place',1)):
            for step in cycle_steps(config, mode, item):
                if 'axis' in step:
                    body = 'AUX GOTO {} {} 300 1000'.format(step['axis'],step['target'])
                    self.assertTrue(self.request(body).ok,body)
                    self.finish_axis(step['axis'])
                    if step['axis'] == 5:
                        self.assertEqual(self.arm.status(4)[0], -step['target'])
                if 'servo' in step:
                    body = 'SERVO {} {}'.format(3-step['servo'], step['angle'])
                    self.assertTrue(self.request(body).ok,body)
                self.assertEqual(self.nav()['state'], 'suspended')
            for axis in (5,6):
                fields = self.request('AUX STAT '+str(axis)).fields
                self.assertEqual((fields['pos'],fields['active']),('0','0'))
        self.external_move('FWD',7)
        self.assertTrue(self.request('NAV POSE 42 0 0 14').ok)
        self.assertTrue(self.request('NAV RESTORE 42 0 0 14').ok)
        self.assertEqual((self.nav()['state'],self.nav()['ready']),('stopped','1'))


if __name__ == '__main__':unittest.main()
