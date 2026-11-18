import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from directed_route import departure_leg, fas_axis_leg, straight_approach_speed, straight_leg, uniform_route
from motion_health import LocalizationGuard, VelocityLease, OemLease, cautious_twist, swept_footprint_clear
from endurance import Endurance


class DirectedTests(unittest.TestCase):
    def test_departure_is_forward_from_actual_heading_and_never_corrects_by_reversing(self):
        goal=dict(x=.30,y=0.,yaw=0.)
        leg=departure_leg([.01,.015,.04],[0.,0.,0.],goal)
        self.assertGreater(leg['distance'],0.)
        self.assertAlmostEqual(leg['poses'][-1]['x'],.30)
        self.assertTrue(all(p['yaw']==.04 for p in leg['poses']))
        self.assertTrue(all(b['x']>a['x'] for a,b in zip(leg['poses'],leg['poses'][1:])))
        self.assertEqual(departure_leg([.33,0.,0.],[0.,0.,0.],goal)['distance'],0.)
        with self.assertRaises(ValueError):
            departure_leg([0.,0.,math.pi],[0.,0.,0.],goal)

    def test_straight_geometry_ignores_final_heading_and_slows_before_the_goal(self):
        goal=dict(x=2.,y=0.,yaw=math.pi/2)
        leg=straight_leg([0.,0.,0.],goal,[[0.,0.],[1.,.125],[2.,0.]])
        self.assertIsNotNone(leg)
        self.assertTrue(all(p['yaw']==0. for p in leg['poses']))
        self.assertEqual(straight_approach_speed(.7),.12)
        self.assertLess(straight_approach_speed(.25),.10)
        self.assertLess(straight_approach_speed(.07),straight_approach_speed(.25))

    def test_reverse_does_not_request_half_turn(self):
        leg=straight_leg([2.,0.,0.],dict(x=0.,y=0.,yaw=0.),[[2.,0.],[1.,0.],[0.,0.]])
        self.assertTrue(leg['reverse'])
        self.assertTrue(all(abs(p['yaw'])<1e-8 for p in leg['poses']))
        self.assertEqual(leg['poses'][-1]['x'],0.)

    def test_fas_station_uses_both_travel_signs_with_same_final_heading(self):
        yaw=-math.pi/2
        starts=(4.125,-3.0,yaw)
        for ty,reverse in ((-3.32,False),(-2.68,True)):
            with self.subTest(reverse=reverse):
                goal=dict(x=4.125,y=ty,yaw=yaw)
                leg=fas_axis_leg(starts,goal)
                self.assertEqual(leg['reverse'],reverse)
                self.assertTrue(all(abs(p['yaw']-yaw)<1e-8 for p in leg['poses']))
                self.assertEqual(leg['poses'][-1]['y'],ty)
        self.assertEqual(straight_approach_speed(.32,reverse=True),.08)
        self.assertLess(straight_approach_speed(.15,reverse=True),.08)

    def test_fas_station_uses_native_planning_for_side_and_unaligned_legs(self):
        yaw=-math.pi/2
        self.assertIsNone(fas_axis_leg((2.,-7.02,yaw),dict(x=2.32,y=-7.02,yaw=yaw)))
        self.assertIsNone(fas_axis_leg((4.125,-3.,0.),dict(x=4.125,y=-2.68,yaw=yaw)))
        self.assertIsNone(fas_axis_leg((4.125,-3.,yaw),dict(x=4.125,y=-2.98,yaw=yaw)))

    def test_forward_and_reject_detour_lateral_or_misaligned(self):
        goal=dict(x=2.,y=0.,yaw=0.)
        self.assertFalse(straight_leg([0.,0.,0.],goal,[[0.,0.],[2.,0.]])['reverse'])
        self.assertIsNone(straight_leg([0.,0.,0.],goal,[[0.,0.],[1.,.2],[2.,0.]]))
        self.assertIsNone(straight_leg([0.,0.,math.pi/2],goal,[[0.,0.],[2.,0.]]))
        self.assertIsNone(straight_leg([0.,0.,0.],dict(x=0.,y=2.,yaw=0.),[[0.,0.],[0.,2.]]))

    def test_fifty_point_closed_ring_shared_yaw_no_duplicate_end(self):
        points=uniform_route([[0.,0.],[5.,0.],[5.,2.],[0.,2.],[0.,0.]],50,math.pi/3)
        self.assertEqual(len(points),50)
        self.assertEqual(len({p['yaw'] for p in points}),1)
        self.assertEqual(len({(p['x'],p['y']) for p in points}),50)


class HealthTests(unittest.TestCase):
    def test_hard_stop_checks_the_swept_body_without_blocking_clear_rotation(self):
        self.assertTrue(swept_footprint_clear((0.,0.,.18),[(.149,-.147)]))
        self.assertFalse(swept_footprint_clear((0.,0.,.18),[(.13,-.125)]))
        self.assertFalse(swept_footprint_clear((.12,0.,0.),[(.19,.06)]))
        self.assertFalse(swept_footprint_clear((-.12,0.,0.),[(-.21,.04)]))
        self.assertTrue(swept_footprint_clear((.06,0.,0.),[(.19,.06)]))

    def test_only_obstacles_in_the_commanded_corridor_slow_the_robot(self):
        cruise=(.12,0.,0.)
        self.assertEqual(cautious_twist(cruise,[(.27,.19)],.12,.05),cruise)
        nearer=cautious_twist(cruise,[(.21,.01)],.12,.05)[0]
        farther=cautious_twist(cruise,[(.39,.01)],.12,.05)[0]
        self.assertTrue(.05<nearer<farther<.12)
        self.assertAlmostEqual(cautious_twist((-.12,0.,0.),[(-.21,.01)],-.12,.05)[0],-nearer)

    def test_caution_preserves_curvature_and_ramps_after_clearance(self):
        bent=(.12,0.,.12)
        reduced=cautious_twist(bent,[(.32,.19)],.12,.05)
        self.assertAlmostEqual(reduced[0],reduced[2]);self.assertLess(reduced[0],.12)
        self.assertAlmostEqual(cautious_twist(bent,[],.05,.05)[0],.06)
        self.assertEqual(cautious_twist((0.,0.,.14),[],0.,.05),(0.,0.,.14))

    def test_oem_supervision_loss_revokes_once_but_never_enables(self):
        guard=OemLease();self.assertFalse(guard.expired(100.))
        guard.receive(100.,True);self.assertFalse(guard.expired(100.4))
        self.assertTrue(guard.expired(100.6));self.assertFalse(guard.expired(101.))
        guard.receive(102.,True);guard.receive(102.2,False)
        self.assertFalse(guard.expired(110.))

    def test_command_and_permission_both_expire(self):
        gate=VelocityLease()
        gate.receive_command(10.,(.1,0.,0.))
        self.assertEqual(gate.output(10.),(0.,0.,0.))
        gate.receive_permit(10.,True)
        self.assertEqual(gate.output(10.1),(.1,0.,0.))
        self.assertEqual(gate.output(10.31),(0.,0.,0.))
        gate.receive_command(10.4,(.2,0.,0.))
        self.assertEqual(gate.output(10.4),(0.,0.,0.))
        gate.receive_permit(10.4,True)
        gate.receive_command(10.4,(math.nan,0.,0.))
        self.assertEqual(gate.output(10.4),(0.,0.,0.))

    def test_fault_latches_and_does_not_self_clear_after_false_recovery(self):
        guard=LocalizationGuard()
        for t in [0.,.2,.4,.7]:
            guard.update(t,moving=True,covariance=[.001]*3,scan_fraction=.1,scan_count=100)
        self.assertIsNotNone(guard.fault)
        guard.update(2.,moving=False,covariance=[.001]*3,scan_fraction=.9,scan_count=100)
        self.assertIsNotNone(guard.fault)
        guard.reset();self.assertIsNone(guard.fault)

    def test_brief_pedestrian_occlusion_does_not_latch(self):
        guard=LocalizationGuard()
        guard.update(0.,moving=True,covariance=[.001]*3,scan_fraction=.1,scan_count=100)
        guard.update(.4,moving=True,covariance=[.001]*3,scan_fraction=.8,scan_count=100)
        self.assertIsNone(guard.fault)

    def test_recovery_requires_fresh_stable_stationary_scan_solution(self):
        guard=LocalizationGuard();guard.fault='test fault'
        values=dict(stationary=True,covariance=[.0004,.0004,.0001],scan_fraction=.9,
                    scan_count=100,pose=[0.,0.,0.],fresh=True)
        self.assertFalse(guard.recover(0.,**values))
        values['fresh']=False;self.assertFalse(guard.recover(2.,**values))
        values['fresh']=True;self.assertFalse(guard.recover(3.,**values))
        self.assertFalse(guard.recover(5.9,**values))
        self.assertTrue(guard.recover(6.1,**values));self.assertIsNone(guard.fault)

    def test_slip_recovery_must_match_observed_location_and_repeated_slip_holds(self):
        guard=LocalizationGuard()
        guard.trip_slip([1.,2.,0.],repeated=False)
        values=dict(stationary=True,covariance=[.0002,.0002,.0001],scan_fraction=.95,scan_count=120,
                    pose=[1.45,2.,0.],fresh=True)
        for now in (0.,3.,6.):self.assertFalse(guard.recover(now,**values))
        values['pose']=[1.01,2.,.01]
        self.assertFalse(guard.recover(7.,**values))
        self.assertTrue(guard.recover(10.1,**values))
        guard.trip_slip([1.,2.,0.],repeated=True)
        self.assertFalse(guard.recover(20.,**values))
        self.assertFalse(guard.recover(30.,**values))


class EnduranceTests(unittest.TestCase):
    def test_requires_calibrated_threshold(self):
        with self.assertRaises(ValueError):Endurance(dict(enabled=True,low_voltage=10.,resume_voltage=None))

    def test_charge_voltage_alone_cannot_restart(self):
        e=Endurance(dict(enabled=True,low_voltage=10.,resume_voltage=12.,low_hold_seconds=20.,charge_hold_seconds=120.))
        e.observe(0.,{'voltage':9.9});e.observe(21.,{'voltage':9.9})
        self.assertTrue(e.pending)
        e.resuming=True
        health=dict(voltage=12.3,charging_stable=False,stationary=True,ready=True,localization_confirmed=True)
        self.assertFalse(e.observe(200.,health))
        health['charging_stable']=True
        self.assertFalse(e.observe(201.,health));self.assertFalse(e.observe(320.,health))
        self.assertTrue(e.observe(321.,health))
        health['voltage']=None
        self.assertFalse(e.observe(322.,health))
        e.cancel();self.assertFalse(e.resuming)

    def test_repeat_only_and_minimum_charge_before_departure(self):
        e=Endurance(dict(enabled=True,repeat_only=True,low_voltage=10.8,resume_voltage=12.2,
                         low_hold_seconds=8.,charge_hold_seconds=10.,min_charge_seconds=30.))
        for now in (0.,9.):e.observe(now,{'voltage':10.6})
        self.assertFalse(e.pending)
        e.observe(10.,{'voltage':10.6},repeat=True)
        self.assertTrue(e.pending)
        e.resuming=True
        health=dict(voltage=12.3,charging_stable=True,stationary=True,ready=True,
                    localization_confirmed=True,dock_localization={'status':'applying'})
        self.assertFalse(e.observe(15.,health,repeat=True))
        self.assertFalse(e.observe(30.,health,repeat=True))
        health['dock_localization']['status']='confirmed'
        self.assertFalse(e.observe(32.,health,repeat=True))
        self.assertFalse(e.observe(44.,health,repeat=True))
        self.assertTrue(e.observe(45.,health,repeat=True))


class MissionRecoveryTests(unittest.TestCase):
    def test_restart_loads_remaining_points_without_motion_and_rejects_new_map(self):
        import json,tempfile
        from test_demo import ControlledBackend, Clock
        from mission import Mission
        from map_grid import MapGrid
        grid=MapGrid(Path(__file__).resolve().parents[2]/'scenes/demo-room')
        with tempfile.TemporaryDirectory() as directory:
            backend=ControlledBackend();clock=Clock();m=Mission(grid,backend,directory,clock)
            m.save_points(uniform_route([[.4,-2.],[4.4,-2.]],3,0.,0.));m.start()
            backend.ops[m.operation]['status']='succeeded';m.tick();m.tick()
            fresh=ControlledBackend();restarted=Mission(grid,fresh,directory,clock)
            restarted.load_checkpoint()
            self.assertEqual([p['id'] for p in restarted.points],['fixed-002','fixed-003'])
            self.assertEqual(fresh.calls,[])
            path=Path(directory)/'mission_checkpoint.json';data=json.loads(path.read_text(encoding='utf-8'));data['map_id']='different'
            path.write_text(json.dumps(data),encoding='utf-8')
            with self.assertRaises(ValueError):restarted.load_checkpoint()

    def test_fifty_points_keep_order_across_sensor_pause_and_recharge(self):
        import tempfile
        from test_demo import ControlledBackend, Clock
        from mission import Mission
        from map_grid import MapGrid
        grid=MapGrid(Path(__file__).resolve().parents[2]/'scenes/demo-room')
        backend=ControlledBackend();clock=Clock()
        original=backend.snapshot
        backend.snapshot=lambda:{**original(),'charging_stable':backend.charging}
        with tempfile.TemporaryDirectory() as directory:
            m=Mission(grid,backend,directory,clock)
            m.endurance=Endurance(dict(enabled=True,low_voltage=10.,resume_voltage=12.,low_hold_seconds=20.,charge_hold_seconds=120.))
            points=uniform_route([[.4,-2.],[4.4,-2.]],50,0.,wait=0.)
            m.save_points(points);m.start()
            for i in range(50):
                self.assertEqual(m.index,i)
                if i in (9,24,39):
                    backend.ready=False;m.tick();old=m.operation
                    backend.ops[old]['status']='canceled';m.tick()
                    backend.ready=True;m.tick();clock.advance(3.1);m.tick()
                    self.assertEqual(m.index,i);self.assertEqual(m.phase,'navigating')
                if i==25:
                    backend.voltage=9.8;m.tick();clock.advance(21.);m.tick()
                    backend.ops[m.operation]['status']='canceled';m.tick()
                    self.assertEqual(m.phase,'returning')
                    backend.ops[m.operation]['status']='succeeded';m.tick()
                    self.assertEqual(m.phase,'docking')
                    backend.charging=True;backend.voltage=12.3;m.tick()
                    clock.advance(121.);m.tick()
                    self.assertEqual(m.phase,'departing')
                    backend.charging=False;backend.ops[m.operation]['status']='succeeded';m.tick()
                    self.assertEqual(m.phase,'navigating');self.assertEqual(m.index,i)
                backend.ops[m.operation]['status']='succeeded';m.tick();m.tick()
            self.assertEqual(m.phase,'completed')
            self.assertEqual(m.index,50)
            self.assertFalse((Path(directory)/'mission_checkpoint.json').exists())


if __name__=='__main__':unittest.main()
