import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from arrival_policy import refinement_stage, heading_profile, should_capture_pose


class ArrivalPolicyTests(unittest.TestCase):
    def test_capture_only_short_aligned_approach(self):
        self.assertTrue(should_capture_pose(.15,math.radians(10)))
        self.assertFalse(should_capture_pose(.4,0))
        self.assertFalse(should_capture_pose(.15,math.pi/2))

    def test_measured_5_degree_error_remains_a_task_to_finish(self):
        stage=refinement_stage(.0074,math.radians(5.7),position_tolerance=.05,yaw_tolerance=math.radians(3))
        self.assertEqual(stage,'heading')
        # Repeated measurements never turn an unfinished waypoint into success
        # or failure just because a retry counter was consumed.
        for _ in range(10):
            self.assertEqual(refinement_stage(.0074,math.radians(5.7),
                position_tolerance=.05,yaw_tolerance=math.radians(3)),'heading')

    def test_position_first_and_strict_completion(self):
        self.assertEqual(refinement_stage(.08,.01,position_tolerance=.05,yaw_tolerance=.052),'position')
        self.assertEqual(refinement_stage(.02,.02,position_tolerance=.05,yaw_tolerance=.052),'complete')

    def test_small_turn_does_not_reuse_large_turn_speed(self):
        small=heading_profile(math.radians(6));large=heading_profile(math.pi/2)
        self.assertLess(small['max_vel_theta'],large['max_vel_theta'])
        self.assertLess(small['min_speed_theta'],small['max_vel_theta'])

    def test_overshoot_reduces_next_turn_authority(self):
        previous=heading_profile(.4)
        next_profile=heading_profile(-.25,.4,previous['max_vel_theta'])
        self.assertLess(next_profile['max_vel_theta'],heading_profile(-.25)['max_vel_theta'])
        self.assertGreater(next_profile['max_vel_theta'],0)
