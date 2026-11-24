import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'demo'))

from corridor_route import CorridorRoute, CurveLeg, EscapeLeg, Leg, rollout_turn, tangent_lane_shift
from map_grid import MapGrid


class CorridorRouteTests(unittest.TestCase):
    def setUp(self):
        self.grid = MapGrid(ROOT / 'scenes/demo-room')
        self.settings = {'schema': 1, 'map_id': self.grid.map_id,
                         'lanes': [[0., -6., -3.], [1., -6., -3.]],
                         'links': [-6., -3.], 'clearance_m': .27}
        self.profile = CorridorRoute(self.grid, self.settings)

    def test_forward_and_reverse_keep_station_heading_on_one_aisle(self):
        start = (0., -4.5, math.pi/2)
        for y, reverse in ((-3.8, False), (-5.2, True)):
            path = self.profile.routes(start, {'x': 0., 'y': y, 'yaw': start[2]})[0]
            self.assertEqual(len(path), 1)
            self.assertEqual(path[0].start, start[:2])
            self.assertEqual(path[0].end, (0., y))
            self.assertEqual(path[0].reverse, reverse)
            self.assertAlmostEqual(path[0].yaw, start[2])

    def test_local_recovery_uses_nearby_same_aisle_positions_only(self):
        target = {'x':0., 'y':-4.65, 'yaw':math.pi/2}
        self.assertTrue(self.profile.short_approach((.12,-4.4,-math.pi/2),target))
        self.assertFalse(self.profile.short_approach((1.,-4.4,0.),target))
        self.assertFalse(self.profile.short_approach((0.,-5.6,0.),target))

    def test_aligned_station_behind_stays_in_reverse_preference(self):
        target = {'x':0.,'y':-5.2,'yaw':math.pi/2}
        self.assertTrue(self.profile.reverse_preferred((.045,-4.,math.radians(78)),target))
        self.assertFalse(self.profile.reverse_preferred((.045,-4.,-math.pi/2),target))
        self.assertFalse(self.profile.reverse_preferred((.3,-4.,math.pi/2),target))
        self.assertFalse(self.profile.reverse_preferred((0.,-5.4,math.pi/2),target))
        self.assertTrue(self.profile.motion_routes((.045,-4.,math.radians(78)),target)[0][0].reverse)

    def test_opposite_aisle_stays_on_one_of_the_explicit_connectors(self):
        start = (0., -4.5, math.pi/2)
        goal = {'x': 1., 'y': -4.5, 'yaw': start[2]}
        routes = self.profile.routes(start, goal)
        self.assertEqual(len(routes), 2)
        for path in routes:
            self.assertEqual(len(path), 3)
            self.assertIn(path[1].start[1], self.settings['links'])
            self.assertEqual(path[1].end[1], path[1].start[1])
            self.assertEqual(path[2].end, (goal['x'], goal['y']))
            self.assertTrue(all(self.profile.line_clear(leg.start, leg.end) for leg in path))

    def test_resume_from_either_connector_without_returning_to_its_entrance(self):
        for link_y in self.settings['links']:
            for x, destination in ((.46, 1.), (.54, 0.), (-.036, 1.)):
                for heading in (0., math.pi):
                    with self.subTest(link=link_y, x=x, destination=destination, heading=heading):
                        start = (x, link_y+.005, heading)
                        goal = {'x': destination, 'y': -4.5, 'yaw': math.pi/2}
                        path = self.profile.routes(start, goal)[0]
                        self.assertEqual([leg.end for leg in path], [(destination, link_y), (destination, -4.5)])
                        self.assertEqual(path[0].start, start[:2])
                        self.assertEqual(path[0].reverse, (destination-x)*math.cos(heading) < 0.)

    def test_connector_rejoin_checks_the_surveyed_strip_and_entire_clearance(self):
        goal = {'x': 1., 'y': -4.5, 'yaw': math.pi/2}
        self.assertFalse(self.profile.routes((.48, -3.25, 0.), goal, entry_offset=.10))
        start = (.48, -3.03, 0.)
        self.assertTrue(self.profile.routes(start, goal, entry_offset=.10))
        with patch.object(self.profile, 'clearance_at', side_effect=lambda p: .18 if .70 < p[0] < .80 else .34):
            self.assertFalse(self.profile.routes(start, goal, entry_offset=.10))

    def test_resume_from_connector_margin_rejoins_before_the_long_leg(self):
        start = (.48, -3.07, 0.)
        goal = {'x': 1., 'y': -4.5, 'yaw': math.pi/2}
        with patch.object(self.profile, 'clearance_at', side_effect=lambda p: .32-abs(p[1]+3.) if .40 < p[0] < .70 else .36):
            path = self.profile.routes(start, goal, entry_offset=.10)[0]
            self.assertLess(path[0].length, .22)
            self.assertEqual(path[0].end[1], -3.)
            self.assertGreater(path[0].end[0], start[0])
            self.assertTrue(all(self.profile.line_clear(leg.start, leg.end) for leg in path[1:]))

    def test_unknown_start_and_mismatched_scene_get_no_reference(self):
        with self.assertRaisesRegex(ValueError, '地图不匹配'):
            CorridorRoute(self.grid, {**self.settings, 'map_id': 'other-map'})
        self.assertEqual(self.profile.routes((-5., -4., 0.), {'x': 1., 'y': -4., 'yaw': 0.}), [])
        self.assertTrue(self.profile.routes((.07,-4.,0.), {'x':0.,'y':-3.6,'yaw':math.pi/2}))

    def test_rejoin_from_smaller_clearance_must_be_short_and_move_inward(self):
        with patch.object(self.profile, 'clearance_at', side_effect=lambda point: .25+point[0]):
            self.assertTrue(self.profile.entry_clear((0., -4.), (.12, -4.)))
            self.assertFalse(self.profile.entry_clear((0., -4.), (.24, -4.)))
            self.assertFalse(self.profile.entry_clear((0., -4.), (-.02, -4.)))
        with patch.object(self.profile, 'clearance_at', side_effect=lambda point: .25-point[0] if point[0] < .07 else .36):
            self.assertFalse(self.profile.entry_clear((0., -4.), (.12, -4.)))

    def test_escape_backs_out_before_turning_for_a_sampling_station(self):
        start=(.45,-4.4,0.)
        goal={'x':0.,'y':-5.2,'yaw':math.pi/2}
        with patch.object(self.profile,'clearance_at',side_effect=lambda p: .34-.4*p[0]):
            leg, *following = self.profile.motion_routes(start,goal)[0]
            self.assertIsInstance(leg, EscapeLeg)
            self.assertTrue(leg.reverse)
            self.assertEqual(leg.yaw,start[2])
            self.assertEqual(following[0].start,leg.end)
            self.assertFalse(self.profile.escape_clear(EscapeLeg(start[:2],(.7,-4.4),0.,False)))

    def test_native_replan_checks_between_sparse_waypoints(self):
        with patch.object(self.profile,'clearance_at',side_effect=lambda p: .19 if 0.32<p[0]<0.58 and -4.6<p[1]<-4.2 else .34):
            self.assertTrue(self.profile.native_path_clear([(0.,-4.4),(0.,-4.),(1.,-4.),(1.,-4.4)]))
            self.assertFalse(self.profile.native_path_clear([(0.,-4.4),(1.,-4.4)]))

    def test_return_guard_keeps_clearance_until_the_dock_approach(self):
        dock=(1.,-3.2)
        with patch.object(self.profile,'clearance_at',side_effect=lambda p: .17 if p[0]>.32 else .32):
            self.assertFalse(self.profile.native_path_clear([(0.,-4.6),(1.,-4.6),dock],dock=dock))
            self.assertTrue(self.profile.native_path_clear([(0.,-4.6),(0.,-3.2),(.18,-3.2),dock],dock=dock))
            self.assertFalse(self.profile.native_path_clear([(0.,-4.6),(0.,-3.2),(.18,-3.2),dock]))

    def test_withdrawal_from_planning_edge_still_checks_occupied_cells(self):
        ix,iy = self.grid.cell_xy(.6,-4.5)
        x,y = self.grid.world(ix,iy)
        mask = bytearray(self.grid.mask)
        for j in range(iy-4,iy+5):
            mask[(self.grid.height-1-j)*self.grid.width+ix] = 0
        self.grid.mask = bytes(mask)
        profile = CorridorRoute(self.grid,self.settings)
        start = (x-self.grid.resolution/2-.133,y)
        retreat = EscapeLeg(start,(start[0]-.22,y),0.,True)
        self.assertFalse(profile.footprint_clear(start,0.))
        self.assertTrue(profile.escape_clear(retreat))
        self.grid.cells[iy*self.grid.width+ix] = 100
        self.assertFalse(profile.escape_clear(retreat))

    def test_short_reverse_can_only_leave_a_virtual_boundary_through_free_raw_cells(self):
        ix,iy = self.grid.cell_xy(.6,-4.5)
        x,y = self.grid.world(ix,iy)
        mask = bytearray(self.grid.mask)
        for row in range(iy-4,iy+5):
            mask[(self.grid.height-1-row)*self.grid.width+ix] = 0
        self.grid.mask = bytes(mask)
        profile = CorridorRoute(self.grid,self.settings)
        path = profile.outward_retreat((x,y,0.),.20)
        self.assertAlmostEqual(path[-1][0],x-.20)
        self.assertIsNone(profile.outward_retreat((x-.05,y,math.pi),.20))
        self.grid.cells[iy*self.grid.width+ix-4] = 100
        self.assertIsNone(profile.outward_retreat((x,y,0.),.20))

    def test_lookahead_does_not_change_the_stop_point_or_pass_a_wall(self):
        leg = Leg((0., -4.), (0., -4.5), math.pi/2, True)
        self.assertAlmostEqual(leg.progress((.02, -4.48)), .48)
        self.assertAlmostEqual(leg.deviation((.02, -4.48)), .02)
        with patch.object(self.profile, 'clearance_at', side_effect=lambda point: .20 if point[1] < -4.63 else .35):
            self.assertEqual(self.profile.lookahead_end(leg), (0., -4.62))
        with patch.object(self.profile, 'clearance_at', side_effect=lambda point: .20 if point[1] < -4.51 else .35):
            self.assertEqual(self.profile.lookahead_end(leg), leg.end)

    def test_same_aisle_rejoin_does_not_move_past_the_stop(self):
        start = (-.075, -4.51, math.pi/2)
        goal = {'x': 0., 'y': -4.50, 'yaw': math.pi/2}
        routes = self.profile.routes(start, goal)
        self.assertEqual(len(routes), 1)
        self.assertEqual(len(routes[0]), 1)
        self.assertEqual(routes[0][0].start, start[:2])
        self.assertEqual(routes[0][0].end, (goal['x'], goal['y']))

    def test_measured_rollout_leaves_the_correct_sign_and_never_reverses_the_turn(self):
        for sign in (-1, 1):
            requested = sign * math.radians(90)
            turn, residual = rollout_turn(requested, math.radians(10))
            self.assertAlmostEqual(turn + residual, requested)
            self.assertAlmostEqual(math.degrees(turn), sign * 80.)
            small, _ = rollout_turn(sign * math.radians(8), math.radians(10))
            self.assertAlmostEqual(math.degrees(small), sign * 4.)

    def test_side_entry_turns_while_following_one_tangent_curve(self):
        start = (.2, -4.8, math.pi)
        goal = {'x': 0., 'y': -4.3, 'yaw': math.pi/2}
        first, = self.profile.motion_routes(start, goal)[0]
        self.assertIsInstance(first, CurveLeg)
        self.assertFalse(first.reverse)
        self.assertEqual(first.start, start[:2])
        self.assertEqual(first.end, (goal['x'], goal['y']))
        self.assertAlmostEqual(first.yaw, start[2])
        self.assertAlmostEqual(first.end_yaw, goal['yaw'])
        self.assertAlmostEqual(first.progress(first.end), first.length)
        self.assertAlmostEqual(first.deviation(first.poses[3]), 0.)
        with patch.object(self.profile, 'clearance_at', side_effect=lambda p: .2 if -4.77 < p[1] < -4.61 else .35):
            self.assertIsNone(self.profile.station_curve(start, goal, False))

    def test_near_lateral_miss_backs_out_and_rejoins_without_a_pivot(self):
        start = (.046, -4.49, math.pi/2)
        goal = {'x': 0., 'y': -4.50, 'yaw': math.pi/2}
        backing, curve = self.profile.motion_routes(start, goal)[0]
        self.assertTrue(backing.reverse)
        self.assertFalse(curve.reverse)
        self.assertAlmostEqual(curve.yaw, backing.yaw)
        self.assertAlmostEqual(curve.length, curve.progress(curve.end))
        self.assertEqual(curve.end, (goal['x'], goal['y']))

    def test_parallel_offset_joins_continuously_with_whole_path_clearance(self):
        start = (.40,-4.75,math.radians(88))
        goal = {'x':0.,'y':-4.23,'yaw':math.pi/2}
        shift = tangent_lane_shift(start,goal,False)
        self.assertAlmostEqual(shift.yaw,start[2])
        self.assertAlmostEqual(shift.end_yaw,goal['yaw'])
        self.assertEqual(shift.end,(goal['x'],goal['y']))
        self.assertTrue(all(a[1] < b[1] for a,b in zip(shift.poses,shift.poses[1:])))
        self.assertEqual(self.profile.station_curve(start,goal,False),shift)
        with patch.object(self.profile,'clearance_at',side_effect=lambda p: .20 if -4.65<p[1]<-4.50 else .35):
            self.assertIsNone(self.profile.station_curve(start,goal,False))
