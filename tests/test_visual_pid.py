"""Visual speed feedback and wheel mixing without physical hardware."""
import unittest
from host.vision_alignment import TRACKING_DEFAULT, VisualPID, camera_to_body, mix_wheels, validate_tracking


class VisualPIDTests(unittest.TestCase):
    def test_left_90_mount_maps_all_four_camera_directions(self):
        for camera, body in [((0,1000),(-1000,0)), ((0,-1000),(1000,0)),
                             ((1000,0),(0,1000)), ((-1000,0),(0,-1000))]:
            with self.subTest(camera=camera):
                self.assertEqual(camera_to_body(*camera,90),body)
        self.assertEqual(camera_to_body(300,-700,0),(300,-700))
        self.assertEqual(camera_to_body(300,-700,-90),(-700,-300))

    def test_installation_angle_validation_and_oblique_rotation(self):
        for angle in (True, None, float('nan'), float('inf'), -181, 181):
            with self.subTest(angle=angle), self.assertRaises(ValueError):
                validate_tracking(dict(TRACKING_DEFAULT,camera_yaw_deg=angle))
        self.assertEqual(camera_to_body(1000,0,45),(707,707))
        self.assertEqual(camera_to_body(0,0,90),(0,0))

    def test_large_error_commands_real_speed_and_both_axes(self):
        loop = VisualPID(TRACKING_DEFAULT)
        vx, vy = loop.update(-144, 23, 1.)
        self.assertLessEqual(vx, -1000)
        self.assertLessEqual(vy, -200)
        self.assertTrue(all(abs(v) <= 2000 for v in mix_wheels(vx, vy, 0, 5, 2000)))

    def test_approaching_slows_and_deadzone_stops(self):
        loop = VisualPID(TRACKING_DEFAULT)
        far = loop.update(140, 0, 1.)[0]
        near = loop.update(15, 0, 1.35)[0]
        self.assertGreater(far, near)
        self.assertGreaterEqual(near, 0)
        self.assertLess(near, 300)
        self.assertEqual(loop.update(3, -3, 1.7), (0, 0))

    def test_direction_crossing_and_reversal_switches(self):
        config = dict(TRACKING_DEFAULT, reverse_x=True, reverse_y=True)
        loop = VisualPID(config)
        self.assertLess(loop.update(50, -50, 1.)[0], 0)
        vx, vy = loop.update(-50, 50, 1.35)
        self.assertGreater(vx, 0)
        self.assertGreater(vy, 0)

    def test_mixing_matches_existing_body_directions_and_inversion(self):
        self.assertEqual(mix_wheels(1000, 0, 0, 0, 2000), [-1000,1000,1000,-1000])
        self.assertEqual(mix_wheels(0, 1000, 0, 0, 2000), [1000]*4)
        self.assertEqual(mix_wheels(0, 0, 200, 5, 2000), [-200,-200,-200,-200])
        self.assertEqual(max(map(abs,mix_wheels(2000,2000,500,5,2000))),2000)

    def test_integral_is_bounded_and_cleared_at_target(self):
        loop = VisualPID(dict(TRACKING_DEFAULT, ki=5, kd=0))
        for index in range(100):
            self.assertLessEqual(loop.update(500, 0, 1+index*.35)[0], 2000)
        self.assertEqual(loop.update(0,0,40.),(0,0))
        self.assertEqual(loop.integral,[0.,0.])

    def test_fast_approach_brakes_before_delayed_frame_reaches_deadzone(self):
        """Measured image closing speed plus latency predicts an impending crossing."""
        loop = VisualPID(TRACKING_DEFAULT)
        self.assertEqual(loop.update(80, 0, 1.)[0], 800)
        self.assertEqual(loop.update(15, 0, 1.1, latency=.1)[0], 0)

    def test_near_target_floor_tapers_instead_of_forcing_300_steps(self):
        """Small residual error must not be driven at the distant approach floor."""
        loop = VisualPID(TRACKING_DEFAULT)
        speed = loop.update(6, 0, 1.)[0]
        self.assertGreater(speed, 0)
        self.assertLess(speed, 100)

    def test_stale_and_duplicate_capture_times_do_not_integrate(self):
        """Time-based damping must reject observations without a new timestamp."""
        loop = VisualPID(TRACKING_DEFAULT)
        loop.update(80,0,1.)
        for timestamp in (1., .9):
            with self.assertRaises(ValueError): loop.update(20,0,timestamp)

    def test_delayed_accelerating_plant_settles_without_endless_reversal(self):
        """Close the loop on independent pixel/pulse scales and camera delays.

        This synthetic plant proves convergence for these cases, not real
        friction, STEP timing or camera accuracy. It never clamps at the goal.
        """
        for scale in (.05, .2, .5):
            for delay in (.05, .15, .25):
                with self.subTest(scale=scale, delay=delay):
                    loop = VisualPID(TRACKING_DEFAULT)
                    error, velocity, command = 140., 0., 0.
                    history, stable = [], 0
                    for tick in range(1500):
                        history.append(error)
                        if tick % 10 == 0 and tick >= round(delay/.01):
                            frame = tick-round(delay/.01)
                            observed = history[frame]
                            command = loop.update(observed, 0, frame*.01, latency=delay)[0]
                            stable = stable+1 if abs(observed) <= 5 and abs(velocity) < 1 else 0
                            if stable >= 3:
                                break
                        velocity += max(-60, min(60, command-velocity))
                        error -= velocity*scale*.01
                    self.assertGreaterEqual(stable, 3)
                    self.assertLessEqual(abs(error), 5)


if __name__ == '__main__':
    unittest.main()
