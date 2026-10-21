"""Held-leg corrections on the compiled ARM image; no board is flashed."""
import unittest
import copy
from host.navigation import DEFAULT, mission_waypoints, path_checksum
from tests import test_navigation as harness


class ArmMissionTests(unittest.TestCase):
    setUp = harness.ArmNavigationTests.setUp
    request = harness.ArmNavigationTests.request
    group_edge = harness.ArmNavigationTests.group_edge
    configure = harness.ArmNavigationTests.configure
    upload = harness.ArmNavigationTests.upload
    nav = harness.ArmNavigationTests.nav
    tick = harness.ArmNavigationTests.tick
    align = harness.ArmNavigationTests.align
    finish_forward = harness.ArmNavigationTests.finish_forward

    def held(self,heading=0):
        """Reach the first held arrival with real compiled motion completion."""
        self.route=[1,0]
        self.configure(attitudes=[0,heading,0,0,0])
        self.assertTrue(self.request('NAV LEG 0 1 1').ok)
        self.assertTrue(self.upload().ok)
        self.assertTrue(self.request('NAV START 42').ok)
        self.align(0);self.finish_forward(0)
        self.assertEqual(self.align(heading)['held'],'1')

    def positions(self):
        return [self.arm.status(i)[0] for i in range(4)]

    def test_lateral_alignment_fix_does_not_retrace_and_next_leg_departs(self):
        self.held()
        before=self.positions()
        self.assertTrue(self.request('NAV NUDGE 42 0 LEFT 7 100 1000').ok)
        self.assertEqual(self.nav()['state'],'nudging')
        self.assertFalse(self.request('NAV GO 42 0').ok)
        self.assertFalse(self.request('NAV FIX 42 0').ok)
        self.finish_forward(0)
        self.assertEqual(self.align(0)['held'],'1')
        moved=self.positions()
        self.assertEqual([a-b for a,b in zip(moved,before)],[-7,-7,7,7])
        self.assertEqual((self.nav()['x'],self.nav()['y']),('-7','14'))
        self.assertFalse(self.request('NAV GO 42 0').ok)
        self.assertFalse(self.request('NAV FIX 43 0').ok)
        self.assertFalse(self.request('NAV FIX 42 1').ok)
        self.assertTrue(self.request('NAV FIX 42 0').ok)
        self.assertEqual(self.positions(),moved,'external pose correction never moves wheels')
        self.assertEqual((self.nav()['x'],self.nav()['y']),('0','14'))
        self.assertEqual(self.request('NAV ALIGN').fields['pending'],'0')
        self.assertTrue(self.request('NAV GO 42 0').ok)
        self.assertFalse(self.request('NAV GO 42 0').ok)
        for _ in range(31):self.tick(self.now+100)
        self.assertEqual(self.nav()['leg'],'2')
        self.assertEqual(self.nav()['target_cd'],'18000')

    def test_body_forward_at_ninety_degrees_changes_world_x(self):
        self.held(9000)
        self.assertTrue(self.request('NAV NUDGE 42 0 FWD 7 100 1000').ok)
        self.finish_forward(9000);self.align(9000)
        self.assertEqual((self.nav()['x'],self.nav()['y']),('7','14'))

    def external_move(self, direction, count, relative=0):
        """Complete actual finite STEP output, retaining external ownership."""
        self.assertTrue(self.request('CHASSIS STRAIGHT {} {} 100 1000'.format(direction,count)).ok)
        for _ in range(count*2+4):self.group_edge()
        self.arm.call('chassis_tick',self.now)
        self.tick(self.now+1,relative)
        self.assertEqual(self.request('CHASSIS STAT').fields['state'],'done')

    def test_external_localization_is_generic_and_preserves_actual_yaw(self):
        self.held()
        self.assertEqual(self.request('NAV ALIGN').fields['suspend'],'1')
        # Existing WAITING owns the wheels: direct composition is rejected.
        self.assertFalse(self.request('CHASSIS STRAIGHT BACK 7 100 1000').ok)
        for command in ('NAV SUSPEND 43 0','NAV SUSPEND 42 1','NAV POSE 42 0 0 14'):
            self.assertFalse(self.request(command).ok,command)
        self.assertTrue(self.request('NAV SUSPEND 42 0').ok)
        self.external_move('BACK',7)
        self.assertEqual((self.nav()['x'],self.nav()['y']),('0','14'))
        self.assertTrue(self.request('CHASSIS HALT').ok)
        self.assertEqual(self.nav()['state'],'suspended')
        self.assertFalse(self.request('CHASSIS CONFIG 0').ok)
        self.assertFalse(self.request('NAV POINT 1 0 14').ok)
        # External pose need not be a station or lie on a lane. It changes XY
        # only; a real 10-degree visual turn must remain visible on the map.
        self.tick(self.now+1,1000)
        self.assertTrue(self.request('NAV POSE 42 0 5 9').ok)
        self.assertEqual(self.nav()['heading_cd'],'1000')
        self.external_move('RIGHT',7,1000)
        self.assertEqual((self.nav()['x'],self.nav()['y']),('12','8'))
        self.assertFalse(self.request('NAV RESTORE 42 0 12 8').ok,'off lane')
        self.assertFalse(self.request('NAV RESTORE 43 0 7 14').ok)
        self.assertTrue(self.request('NAV RESTORE 42 0 7 14').ok)
        self.assertEqual(self.nav()['state'],'stopped')
        self.assertEqual(self.nav()['heading_cd'],'1000')
        self.assertEqual((self.nav()['x'],self.nav()['y']),('7','14'))
        self.assertEqual(self.nav()['ready'],'1')
        self.assertFalse(self.request('NAV POSE 42 0 0 14').ok)
        self.assertFalse(self.request('NAV START 42').ok,'old upload revoked')
        self.assertFalse(self.request('NAV WORK 42 0 BEGIN').ok,'withdrawn business interface')

    def test_external_stop_and_stale_imu_revoke_handoff_and_uncertain_xy(self):
        self.held()
        self.assertTrue(self.request('NAV SUSPEND 42 0').ok)
        self.tick(self.now+300,feedback=False)
        self.assertEqual(self.nav()['fault'],'imu_stale')
        self.assertEqual(self.nav()['ready'],'0')
        self.assertFalse(self.request('NAV POSE 42 0 0 14').ok)
        for command in ('STOP','ARM 0','NAV STOP','NAV OBSTACLE 1'):
            self.setUp();self.held()
            self.assertTrue(self.request('NAV SUSPEND 42 0').ok)
            self.assertTrue(self.request(command).ok)
            self.assertEqual(self.nav()['ready'],'0',command)
            self.assertFalse(self.request('NAV RESTORE 42 0 7 14').ok,command)

    def test_nine_and_thirtytwo_leg_uploads_reach_past_old_fifth_leg_boundary(self):
        for count in (9,32):
            self.setUp();self.configure()
            self.assertTrue(self.request('NAV LEGS {}'.format(count)).ok)
            config=copy.deepcopy(DEFAULT)
            config['stations']=[{'name':name,'x':p[0],'y':p[1],'heading':0}
                                for name,p in zip('OABCD',self.points)]
            config['grid']=self.grid
            config['route']=[{'station':1 if i%2==0 else 0,'hold':1} for i in range(count)]
            for i,leg in enumerate(config['route']):
                self.assertTrue(self.request('NAV LEG {} {} 1'.format(i,leg['station'])).ok)
            entries=mission_waypoints(config,(0,0))
            self.assertTrue(self.request('NAV BEGIN 42 {} 0 0'.format(len(entries))).ok)
            for i,p in enumerate(entries):
                self.assertTrue(self.request('NAV WP 42 {} {} {} {}'.format(i,*p)).ok,(i,p))
            self.assertTrue(self.request('NAV COMMIT 42 {}'.format(path_checksum(entries))).ok)
            self.assertTrue(self.request('NAV START 42').ok)
            self.assertEqual(self.nav()['legs'],str(count))

    def test_held_guards_budget_stale_imu_stop_and_timeout(self):
        self.held()
        for body in ('NAV NUDGE 43 0 LEFT 1 100 1000','NAV NUDGE 42 1 LEFT 1 100 1000',
                     'NAV NUDGE 42 0 CW 1 100 1000','NAV NUDGE 42 0 LEFT 1001 100 1000'):
            self.assertFalse(self.request(body).ok,body)
        self.assertTrue(self.request('NAV NUDGE 42 0 LEFT 1000 100 1000').ok)
        for _ in range(2000):self.group_edge()
        self.tick(self.now+1);self.align(0)
        self.assertFalse(self.request('NAV NUDGE 42 0 RIGHT 1 100 1000').ok)
        self.assertTrue(self.request('NAV FIX 42 0').ok)
        self.assertTrue(self.request('NAV NUDGE 42 0 RIGHT 10 100 1000').ok)
        self.tick(self.now+300,feedback=False)
        self.assertEqual(self.nav()['fault'],'imu_stale')
        self.assertFalse(self.request('NAV GO 42 0').ok)
        self.assertFalse(self.request('NAV FIX 42 0').ok)
        self.setUp();self.held()
        self.assertTrue(self.request('NAV NUDGE 42 0 RIGHT 10 100 1000').ok)
        for _ in range(600):self.tick(self.now+100)
        self.assertEqual(self.nav()['fault'],'micro_timeout')
        self.assertEqual(self.arm.call('motion_chassis_active'),0)


if __name__=='__main__':unittest.main()
