import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from arrival_policy import (FAS_STATION_POSITION_TOLERANCE, FAS_STATION_YAW_TOLERANCE, SpinBrake,
                            fas_heading_needs_brake, refinement_stage,
                            heading_profile, in_place_heading_profile, heading_needs_brake)


class ArrivalPolicyTests(unittest.TestCase):
    def test_station_accepts_inside_four_centimeters_and_four_degrees(self):
        limits={'position_tolerance':.04,'yaw_tolerance':math.radians(4)}
        self.assertEqual(refinement_stage(.039,math.radians(3.8),**limits),'complete')
        self.assertEqual(refinement_stage(.15,0.,**limits),'position')
        self.assertEqual(refinement_stage(.039,math.radians(4.1),**limits),'heading')

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

    def test_fas_keeps_position_when_only_heading_is_outside_tolerance(self):
        limits={'position_tolerance':FAS_STATION_POSITION_TOLERANCE,
                'yaw_tolerance':FAS_STATION_YAW_TOLERANCE}
        self.assertEqual(refinement_stage(.0423,math.radians(.37),**limits),'complete')
        self.assertEqual(refinement_stage(.0172,math.radians(3.56),**limits),'complete')
        self.assertEqual(refinement_stage(.0526,math.radians(4.36),**limits),'heading')
        self.assertEqual(refinement_stage(.056,0.,**limits),'position')

    def test_small_turn_does_not_reuse_large_turn_speed(self):
        small=heading_profile(math.radians(6));large=heading_profile(math.pi/2)
        self.assertLess(small['max_vel_theta'],large['max_vel_theta'])
        self.assertLess(small['min_speed_theta'],small['max_vel_theta'])

    def test_overshoot_reduces_next_turn_authority(self):
        previous=heading_profile(.4)
        next_profile=heading_profile(-.25,.4,previous['max_vel_theta'])
        self.assertLess(next_profile['max_vel_theta'],heading_profile(-.25)['max_vel_theta'])
        self.assertGreater(next_profile['max_vel_theta'],0)

    def test_small_heading_command_clears_dead_zone_and_brakes_on_feedback(self):
        profile=in_place_heading_profile(math.radians(7))
        self.assertGreater(profile['min_speed_theta'],.10)
        self.assertLess(profile['min_speed_theta'],profile['max_vel_theta'])
        tolerance=math.radians(3)
        self.assertFalse(heading_needs_brake(math.radians(4),0.,tolerance))
        self.assertFalse(heading_needs_brake(math.radians(8),.18,tolerance))
        self.assertTrue(heading_needs_brake(math.radians(2.5),.18,tolerance))
        self.assertTrue(heading_needs_brake(math.radians(-1),.18,tolerance))

    def test_fas_brakes_between_pose_updates_only_with_fresh_feedback(self):
        error=math.radians(3.6)
        rate=.18
        tolerance=FAS_STATION_YAW_TOLERANCE
        self.assertFalse(heading_needs_brake(error,rate,tolerance))
        self.assertTrue(fas_heading_needs_brake(error,rate,tolerance,.10))
        self.assertFalse(fas_heading_needs_brake(error,rate,tolerance,.25))
        self.assertFalse(fas_heading_needs_brake(error,0.,tolerance,.10))
        self.assertTrue(fas_heading_needs_brake(-error,-rate,tolerance,.10))

    def test_gyro_brakes_small_turn_during_breakaway_not_after_the_goal(self):
        for sign in (-1.,1.):
            brake=SpinBrake(sign*math.radians(3.5),0.,.005)
            self.assertFalse(brake.observe(.05,.005))
            self.assertFalse(brake.observe(.10,.005+sign*.020))
            self.assertTrue(brake.observe(.15,.005+sign*.060))
            self.assertLess(brake.progress,math.radians(.5))

    def test_gyro_gap_brakes_a_pending_small_turn(self):
        brake=SpinBrake(math.radians(3.5),0.,0.)
        self.assertFalse(brake.observe(.25,.02))
        self.assertFalse(brake.moving)
        self.assertTrue(brake.observe(.50,.06))
