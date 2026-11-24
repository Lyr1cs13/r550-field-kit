import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from guarded_retreat import GuardedRetreat, RouteRetreat, rear_is_clear


class GuardedRetreatTests(unittest.TestCase):
    rear = [(-1.5, .04*i) for i in range(-6, 7)]
    front_buffer = (.16, -.09)

    def test_front_buffer_allows_only_clear_rear_and_body_envelope(self):
        points=[*self.rear, self.front_buffer]
        self.assertTrue(rear_is_clear(points))
        self.assertFalse(rear_is_clear([self.front_buffer]))
        self.assertFalse(rear_is_clear([*points, (-.40, .12)]))
        self.assertFalse(rear_is_clear([*points, (.13, -.08)]))
        self.assertTrue(rear_is_clear([*points, (.12, -.169)]))
        self.assertFalse(rear_is_clear([*points, (-.11, -.169)]))

    def test_vehicle_specific_echo_keeps_adjacent_rear_obstacles(self):
        echo=(-.153,-.089,.012)
        isolated=[*self.rear,(-.152,-.088)]
        self.assertFalse(rear_is_clear(isolated))
        self.assertTrue(rear_is_clear(isolated,echo))
        self.assertFalse(rear_is_clear([*isolated,(-.21,-.11)],echo))
        self.assertTrue(rear_is_clear([*isolated,(-.154,-.09),(-.1625,-.0924)],echo))
        self.assertFalse(rear_is_clear([*isolated,(-.154,-.09),(-.1625,-.0924),(-.147,-.088)],echo))

    def test_twenty_centimetre_reverse_latches_stop(self):
        r=GuardedRetreat()
        odom=(0., 0., math.pi/2, 0., 0.)
        self.assertEqual(r.tick(0., self.rear, odom), -.05)
        self.assertEqual(r.tick(2., self.rear, (0., -.09, math.pi/2, -.05, 0.)), -.05)
        self.assertEqual(r.tick(4., self.rear, (0., -.201, math.pi/2, -.04, 0.)), 0.)
        self.assertEqual(r.status, 'done')
        self.assertEqual(r.tick(5., self.rear, odom), 0.)

    def test_continuous_rear_check_and_stall_stop(self):
        r=GuardedRetreat()
        still=(0.,0.,0.,0.,0.)
        r.tick(0.,self.rear,still)
        self.assertEqual(r.tick(.4,[*self.rear,(-.25,0.)],still),0.)
        self.assertEqual(r.status,'blocked')
        r.reset();r.tick(0.,self.rear,still)
        self.assertEqual(r.tick(2.1,self.rear,still),0.)
        self.assertEqual(r.status,'stalled')

    def test_route_recovery_only_allows_a_second_segment_after_verified_reverse_progress(self):
        history=RouteRetreat()
        start=(0.,0.,math.pi/2)
        self.assertTrue(history.available(start))
        history.record(start)
        self.assertFalse(history.available(start))
        self.assertFalse(history.available((0.,-.10,math.pi/2)))
        self.assertFalse(history.available((.10,-.20,math.pi/2)))
        second=(.01,-.20,math.pi/2)
        self.assertTrue(history.available(second))
        history.record(second)
        self.assertFalse(history.available((.01,-.40,math.pi/2)))
        self.assertTrue(history.available((.01,-.60,math.pi/2)))
