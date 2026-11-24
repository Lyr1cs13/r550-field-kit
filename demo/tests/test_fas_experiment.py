from contextlib import redirect_stdout
from collections import Counter
from http.client import HTTPResponse, IncompleteRead
import importlib.util
import io
import json
import math
from pathlib import Path
from random import Random
import sqlite3
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import patch
import uuid

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from aisle_clearance import AisleClearance
from directed_route import uniform_route
from fas_car import CorridorLease, FasCar, WS63_CONNECTED_IDS, WS63_IDS, obstruction_clear
from fas_experiment import (C5Sample, Journal, PROTOCOL, PairObservation, RecoveryWait, ScanEvidence, Stations, aggregate,
                            capture_request, evaluate, move_batches, parallel_bounds, phase_state, position_rounds,
                            scan_confirmed, search, station_ids, stopped_station_arrival, sweep_pairs)
from guarded_retreat import slip_obstruction
from map_grid import MapGrid


LINKS = station_ids({'sle':WS63_IDS,'c5':{'A':'02:00:00:00:01:AA','B':'0200000001bb'}})
RECEIVERS = ('sle/main','c5/main')
CODES = list(range(4096))


def receipt(request, sle=(-55.,-61.), c5=(-64.,-70.), n=30):
    output = dict(request,status='complete',raw_ref='station/run/record')
    output['data'] = {key:{'epoch':8,'rssi_dbm':dict(zip(('A','B'),values)),'samples':{'A':n,'B':n}}
                      for key,values in zip(RECEIVERS,(sle,c5))}
    return output


class AisleClearanceTests(TestCase):
    def test_keeps_away_from_keepout_edge_and_allows_only_outward_rejoin(self):
        grid = MapGrid(Path(__file__).resolve().parents[2] / 'scenes/demo-room')
        ix, iy = grid.cell_xy(0., -4.5)
        mask = bytearray(grid.mask)
        for j in range(iy-4, iy+5):
            mask[(grid.height-1-j)*grid.width+ix] = 0
        grid.mask = bytes(mask)
        x, y = grid.world(ix, iy)
        aisle = AisleClearance(grid.info(), grid.png(), .26)

        near, center = (x+.20, y), (x+.47, y)
        self.assertAlmostEqual(aisle.distance(near), .20-grid.resolution/2)
        self.assertFalse(aisle.allows([near, center]))
        self.assertTrue(aisle.allows([near, center], rejoin=True))
        self.assertFalse(aisle.allows([near, (x+.14,y), center], rejoin=True))
        self.assertFalse(aisle.allows([(x+.47,y-.4), near, (x+.47,y+.4)]))
        self.assertTrue(aisle.allows([(x+.47,y-.4), center, (x+.47,y+.4)]))


class ConnectedTransmitterTests(TestCase):
    def test_formal_capture_restores_only_a_fresh_measured_stationary_reference(self):
        with TemporaryDirectory() as directory:
            config = Path(directory)/'fas_robot.json'
            config.write_text(json.dumps({
                'car':'A', 'map_id':'scene', 'ws63_mode':'connected',
                'ws63_device':'/dev/serial/by-path/ws63', 'c5_device':'/dev/serial/by-path/c5',
                'c5_mac':LINKS['c5']['A'],
            }), encoding='utf-8')
            measured = [.25, .30, .08]
            health = {'stationary':True, 'ready':True, 'localization_confirmed':True,
                      'pose':measured, 'pose_age_s':.04, 'scan_map_agreement':.95,
                      'localization_covariance':(.001,.001,.002)}
            mission = SimpleNamespace(grid=SimpleNamespace(map_id='scene'), phase='idle')
            backend = SimpleNamespace(snapshot=lambda:health)
            car = FasCar(backend, mission, config)
            car.tick()
            self.assertFalse(car.pose_checkpoint.exists())
            car.lease.owner = uuid.uuid4().hex
            with patch('fas_car.time.time', return_value=1000.):
                car.tick()
                snapshot = json.loads(car.pose_checkpoint.read_text())
            self.assertEqual(snapshot['pose'], measured)
            for stamp, map_id, expect in ((1010., 'scene', True), (1200., 'scene', False), (1010., 'old', False)):
                with self.subTest(stamp=stamp, map_id=map_id):
                    car.pose_checkpoint.write_text(json.dumps({**snapshot, 'map_id':map_id}))
                    restarted = SimpleNamespace(fas_saved_pose=None, confirmed=False)
                    with patch('fas_car.time.time', return_value=stamp):
                        new_car = FasCar(restarted, mission, config)
                    self.assertEqual(restarted.fas_saved_pose, (tuple(measured), 1000.) if expect else None)
                    self.assertFalse(restarted.confirmed)
                    self.assertFalse(new_car.lease.live)

    def test_connected_mode_uses_base_managed_transmitters(self):
        with TemporaryDirectory() as directory:
            config = Path(directory)/'fas_robot.json'
            config.write_text(json.dumps({
                'car':'B', 'map_id':'map', 'ws63_mode':'connected',
                'ws63_device':'/dev/serial/by-path/ws63',
                'c5_device':'/dev/serial/by-path/c5',
                'c5_mac':LINKS['c5']['B'],
            }),encoding='utf-8')
            mission = SimpleNamespace(grid=SimpleNamespace(map_id='map'))
            with patch('fas_car.Ws63Transmitter') as transmitter:
                car = FasCar(SimpleNamespace(),mission,config)
                transmitter.assert_not_called()
            self.assertEqual(car.links, {'sle':WS63_CONNECTED_IDS['B'],'c5':LINKS['c5']['B']})
            car.disarm_radio()
            with patch.object(Path,'is_char_device',return_value=True), patch.object(Path,'samefile',return_value=True):
                self.assertFalse(car.usb_ready())
            with patch.object(Path,'is_char_device',return_value=True), patch.object(Path,'samefile',return_value=False):
                self.assertTrue(car.usb_ready())


class FasSceneTests(TestCase):
    def test_stale_config_keeps_single_car_available_and_retains_claim_validation(self):
        import server
        with TemporaryDirectory() as directory:
            root = Path(directory)
            state = root / 'state'
            state.mkdir()
            (state / 'fas_robot.json').write_text(json.dumps({
                'car': 'A', 'map_id': 'original-map', 'motion_only': True,
            }), encoding='utf-8')
            grid = SimpleNamespace(directory=root, map_id='new-map', png=lambda: b'')
            backend = SimpleNamespace(preview_route=lambda: None)
            mission = SimpleNamespace(grid=grid)
            with patch.object(server, 'HERE', root / 'demo'), patch.object(server, 'Mission', return_value=mission):
                self.assertIsNone(server.App(grid, backend, root).fas)
                (state / 'fas_robot_claim.json').write_text(json.dumps({
                    'owner': 'test-run', 'map_id': 'original-map', 'motion_only': True,
                }), encoding='utf-8')
                with self.assertRaisesRegex(ValueError, '地图版本'):
                    server.App(grid, backend, root)
                grid.map_id = 'original-map'
                self.assertEqual(server.App(grid, backend, root).fas.lease.owner, 'test-run')


class FasProtocolTests(TestCase):
    def setUp(self):
        self.request = capture_request(uuid.uuid4().hex,uuid.uuid4().hex,5,{},phase_state(5,CODES),LINKS)

    def test_delta_is_signed_and_separate_by_receiver(self):
        exact = receipt(self.request)
        self.assertEqual(evaluate(self.request,exact,RECEIVERS),dict.fromkeys(RECEIVERS,6.))
        reverse = receipt(self.request,sle=(-61.,-55.),c5=(-44.,-60.))
        self.assertEqual(evaluate(self.request,reverse,RECEIVERS),{'sle/main':-6.,'c5/main':16.})
        with self.assertRaises(ValueError):
            evaluate(self.request,receipt(self.request,n=29),RECEIVERS)

    def test_stale_or_wrong_phase_epoch_does_not_match(self):
        self.assertEqual(phase_state(4095,CODES),{'sle':[63,63],'c5':4095})
        for key,wrong in [('trial_id','old'),('state_id',4),('radios',{'sle':[0,4],'c5':5}),
                          ('tx',{'sle':{'A':WS63_IDS['B'],'B':WS63_IDS['A']}}),('status','incomplete')]:
            with self.subTest(key=key),self.assertRaises(ValueError):
                evaluate(self.request,{**receipt(self.request),key:wrong},RECEIVERS)

    def test_c5_filters_mac_epoch_and_full_24_bit_state(self):
        lines = ['D,9,02:00:00:00:01:aa,-42,100,0x065001,{"seq":1,"csi":[1,2]}',
                 'D,9,02:00:00:00:01:bb,-52,101,0x065001,{"seq":4}',
                 'D,9,02:00:00:00:01:aa,-85,101,0x066001,{"seq":2}',
                 'D,8,02:00:00:00:01:bb,-88,99,0x065001,{"seq":3}',
                 'D,9,02:00:00:00:01:cc,-70,103,0x065001,{"seq":5}']
        rows = [C5Sample.parse(line) for line in lines+[lines[0]]]
        result = aggregate(rows,9,0x065001,LINKS['c5'])
        self.assertEqual(result['samples'],{'A':1,'B':1})
        self.assertEqual(result['rssi_dbm'],{'A':-42,'B':-52})


class FasMotionTests(TestCase):
    def test_non_overlapping_routes_can_drive_in_parallel(self):
        starts = {'A':(0.,0.),'B':(3.,0.)}
        ends = {'A':(0.,2.),'B':(3.,2.)}
        paths={car:[starts[car],ends[car]] for car in starts}
        self.assertEqual(move_batches(starts,ends,paths),(('A','B'),))
        self.assertEqual(move_batches(starts,ends,paths,exclusive=True),(('A',),('B',)))

    def test_replanning_room_preserves_gap_on_the_separating_axis(self):
        paths={'A':[(0.,0.),(0.,2.)],'B':[(1.5,0.),(1.5,2.)]}
        bounds=parallel_bounds(paths)
        self.assertAlmostEqual(bounds['B'][0]-bounds['A'][1],.10)
        self.assertLess(bounds['A'][2],-.60)
        self.assertAlmostEqual(bounds['B'][3],2.9)
        self.assertIsNone(parallel_bounds({**paths,'B':[(1.1,0.),(1.1,2.)]}))

    def test_head_on_swap_is_never_scheduled(self):
        starts = {'A':(0.,0.),'B':(3.,0.)}
        ends = {'A':starts['B'],'B':starts['A']}
        paths={car:[starts[car],ends[car]] for car in starts}
        self.assertIsNone(move_batches(starts,ends,paths))

    def test_nearby_parallel_docks_depart_one_at_a_time(self):
        starts={'A':(0.,0.),'B':(.65,0.)}
        ends={'A':(0.,2.),'B':(.65,3.)}
        paths={car:[starts[car],ends[car]] for car in starts}
        self.assertEqual(move_batches(starts,ends,paths),(('A',),('B',)))
        paths['A']=[starts['A'],(.65,.25),ends['A']]
        self.assertEqual(move_batches(starts,ends,paths),(('B',),('A',)))

    def test_peer_can_move_aside_first_but_not_leave_overlap_to_timing(self):
        starts = {'A':(0.,0.),'B':(2.5,0.)}
        ends = {'A':(2.5,0.),'B':(2.5,2.)}
        paths={car:[starts[car],ends[car]] for car in starts}
        self.assertEqual(move_batches(starts,ends,paths),(('B',),('A',)))

    def test_expired_lease_and_unsafe_envelope_reject_motor_authority(self):
        now = [0.]
        lease = CorridorLease(lambda:now[0]);lease.owner='run'
        h={'ready':True,'localization_confirmed':True,'pose':(0.,0.,0.),
           'pose_age_s':.03,'localization_covariance':(.001,.001,.002),'scan_map_agreement':.91}
        key=uuid.uuid4().hex
        lease.reserve(key,'move',(-.5,2.5,-.5,.5))
        self.assertTrue(lease.allows('move',h,((2.,0.),)))
        self.assertFalse(lease.allows('return',h))
        self.assertFalse(lease.allows('move',h,((2.2,0.),)))
        self.assertFalse(lease.allows('move',{**h,'pose_age_s':.5}))
        peer=[0.,.65]
        lease.reserve(key,'move',(-.5,2.5,-.5,1.5),peer)
        self.assertTrue(lease.allows('move',h,((0.,0.),(2.,0.))))
        self.assertFalse(lease.allows('move',h,((0.,0.),(0.,1.4))))
        now[0]=1.1
        self.assertFalse(lease.allows('move',h))
        with self.assertRaises(ValueError):
            lease.renew(key)

    def test_uncertainty_uses_remaining_corridor_and_peer_clearance(self):
        lease = CorridorLease(lambda:0.);lease.owner='run'
        key = uuid.uuid4().hex
        health={'ready':True,'localization_confirmed':True,'pose':(0.,0.,0.),'pose_age_s':.04,
                'localization_covariance':(.071**2,.002,.003),'scan_map_agreement':.95}
        lease.reserve(key,'move',(-.5,2.5,-.5,.5))
        self.assertTrue(lease.allows('move',health,((2.,0.),)))
        self.assertFalse(lease.allows('move',health,((2.1,0.),)))
        lease.reserve(key,'move',(-.5,2.5,-.5,1.5),[0.,.63],.07)
        self.assertFalse(lease.allows('move',health))
        too_uncertain={**health,'localization_covariance':(.14**2,.002,.003)}
        self.assertFalse(lease.allows('move',too_uncertain))

    def test_bad_scan_closes_motor_gate_during_bounded_hold(self):
        now=[0.]
        lease=CorridorLease(lambda:now[0]);lease.owner='run'
        h={'ready':True,'localization_confirmed':True,'pose':(0.,0.,0.),
           'pose_age_s':.03,'localization_covariance':(.001,.001,.002),'scan_map_agreement':.91}
        unsynced={**h,'scan_map_agreement':.79}
        epoch=uuid.uuid4().hex
        lease.reserve(epoch,'move',(-.5,2.5,-.5,.5))
        now[0]=.5
        lease.supervise(epoch,unsynced,())
        self.assertFalse(lease.allows('move',unsynced))
        self.assertEqual(lease.until,1.5)
        lease.supervise(epoch,h,())
        self.assertTrue(lease.allows('move',h))
        self.assertEqual(lease.until,1.5)
        now[0]=1.5
        with self.assertRaisesRegex(ValueError,'已过期'):
            lease.supervise(epoch,unsynced,())
        # Usable supervisor traffic cannot keep a bad scan on hold forever.
        lease.reserve(epoch,'move',(-.5,2.5,-.5,.5))
        for stamp in (2.,2.6,3.2,3.8,4.4):
            now[0]=stamp
            lease.supervise(epoch,unsynced,())
            self.assertFalse(lease.allows('move',unsynced))
        now[0]=4.6
        with self.assertRaisesRegex(ValueError,'收回行驶许可'):
            lease.supervise(epoch,unsynced,())
        self.assertFalse(lease.live)

    def test_pose_uncertainty_can_recover_without_discarding_the_goal(self):
        now=[0.]
        lease=CorridorLease(lambda:now[0]);lease.owner='run'
        health={'ready':True,'localization_confirmed':True,'pose':(0.,0.,0.),
                'pose_age_s':.03,'localization_covariance':(.001,.001,.002),'scan_map_agreement':.91}
        uncertain={**health,'localization_covariance':(.14**2,.002,.003)}
        epoch=uuid.uuid4().hex
        lease.reserve(epoch,'move',(-.6,2.5,-.6,.6))
        now[0]=.5
        lease.supervise(epoch,uncertain,())
        self.assertTrue(lease.live)
        self.assertFalse(lease.allows('move',uncertain))
        now[0]=.8
        lease.supervise(epoch,health,())
        self.assertTrue(lease.allows('move',health))

    def test_independent_structural_match_can_verify_weak_raw_scan_but_requires_fresh_pose(self):
        lease=CorridorLease(lambda:0.);lease.owner='run'
        lease.reserve(uuid.uuid4().hex,'move',(-.5,2.5,-.5,.5))
        h={'ready':True,'localization_confirmed':True,'pose':(0.,0.,0.),
           'pose_age_s':.03,'localization_covariance':(.001,.001,.002),
           'scan_map_agreement':.79,'scan_structure_verified':True}
        self.assertTrue(lease.allows('move',h))
        changed={**h,'scan_map_agreement':.53,'scan_window_agreement':.55}
        self.assertTrue(lease.allows('move',changed))
        self.assertFalse(lease.allows('move',{**changed,'scan_structure_verified':False}))
        self.assertFalse(lease.allows('move',{**h,'scan_map_agreement':.92,'scan_window_agreement':None,
                                             'scan_structure_verified':False}))
        self.assertFalse(lease.allows('move',{**h,'pose_age_s':.5}))
        self.assertFalse(lease.allows('move',h,((2.2,0.),)))

    def test_structure_window_absorbs_one_noisy_frame_but_expires_on_pose_jump(self):
        evidence=ScanEvidence()
        for stamp,value,verified in ((1.,.72,True),(1.1,.73,False),(1.2,.71,True),(1.3,.69,False)):
            evidence.add(stamp,value,verified,(0.,0.,0.))
        score,structure=evidence.latest(1.31)
        self.assertTrue(scan_confirmed({'scan_map_agreement':.69,'scan_window_agreement':score,
                                        'scan_structure_verified':structure}))
        self.assertTrue(scan_confirmed({'scan_map_agreement':.54,'scan_window_agreement':score,
                                        'scan_structure_verified':structure}))
        evidence.add(1.4,.71,False,(.09,0.,0.))
        self.assertEqual(evidence.latest(1.41),(None,False))
        self.assertEqual(evidence.latest(1.85),(None,False))

    def test_return_recovers_transient_scan_hold_before_native_docking(self):
        lease=CorridorLease(lambda:0.);lease.owner='run'
        epoch=uuid.uuid4().hex
        lease.reserve(epoch,'return',(-.6,2.,-.6,.6))
        h={'ready':True,'localization_confirmed':True,'pose':(0.,0.,0.),'pose_age_s':.03,
           'localization_covariance':(.001,.001,.002),'scan_map_agreement':.72}
        lease.supervise(epoch,h,())
        self.assertFalse(lease.allows('return',h))
        lease.supervise(epoch,{**h,'scan_map_agreement':.90},())
        self.assertIsNone(lease.scan_hold_since)
        with self.assertRaisesRegex(ValueError,'收回行驶许可'):
            lease.supervise(epoch,h,(),docking=True)
        self.assertFalse(lease.live)

    def test_expired_travel_can_only_confirm_an_already_stopped_pose(self):
        arrival={'kind':'waypoint','status':'heading_settling','position_error':.02,'yaw_error_deg':2.}
        health={'stationary':True,'ready':True,'localization_confirmed':True,'pose_age_s':.08,'arrival':arrival}
        self.assertTrue(stopped_station_arrival(health))
        self.assertFalse(stopped_station_arrival({**health,'stationary':False}))
        self.assertFalse(stopped_station_arrival({**health,'pose_age_s':.6}))
        self.assertTrue(stopped_station_arrival({**health,'arrival':{**arrival,'yaw_error_deg':3.5}}))
        self.assertFalse(stopped_station_arrival({**health,'arrival':{**arrival,'yaw_error_deg':4.3}}))

    def test_native_dock_can_renew_with_local_feedback_inside_the_reserved_corridor(self):
        lease=CorridorLease(lambda:0.);lease.owner='run'
        epoch=uuid.uuid4().hex
        lease.reserve(epoch,'return',(-.75,2.,-.75,.75),[1.6,0.])
        h={'ready':True,'localization_confirmed':True,'pose':(0.,0.,0.),'pose_age_s':.04,
           'localization_covariance':(.001,.001,.002),'scan_window_agreement':None,
           'scan_structure_verified':False,'fas_local_docking':True}
        lease.supervise(epoch,h,(),docking=True)
        self.assertTrue(lease.allows('return',h))
        self.assertFalse(lease.allows('return',{**h,'fas_local_docking':False}))
        self.assertFalse(lease.allows('return',{**h,'pose_age_s':.55}))
        self.assertFalse(lease.allows('return',{**h,'pose':(1.2,0.,0.)}))

    def test_dock_staging_still_requires_fresh_pose_and_safe_route(self):
        lease=CorridorLease(lambda:0.);lease.owner='run'
        lease.reserve(uuid.uuid4().hex,'return',(-.75,2.,-.75,.75),[1.6,0.])
        h={'ready':True,'localization_confirmed':True,'pose':(0.,0.,0.),'pose_age_s':.04,
           'localization_covariance':(.0034,.0008,.005),'scan_window_agreement':.75,
           'fas_dock_staging':True}
        self.assertTrue(lease.allows('return',h))
        self.assertFalse(lease.allows('return',{**h,'fas_dock_staging':False}))
        self.assertFalse(lease.allows('return',{**h,'pose_age_s':.55}))
        self.assertFalse(lease.allows('return',h,((1.2,0.),)))
        lease.reserve(uuid.uuid4().hex,'move',(-.75,2.,-.75,.75),[1.6,0.])
        self.assertFalse(lease.allows('move',h))

    def test_anchored_rotation_keeps_corridor_and_peer_checks(self):
        lease=CorridorLease(lambda:0.);lease.owner='run'
        lease.reserve(uuid.uuid4().hex,'move',(-.7,2.,-.7,.7),[1.5,0.])
        health={'ready':True,'localization_confirmed':True,'pose':(0.,0.,0.),'pose_age_s':.58,
                'localization_covariance':(.002,.002,.003),'scan_map_agreement':.5,'fas_local_rotation':True}
        self.assertTrue(lease.allows('move',health,((0.,0.),)))
        self.assertFalse(lease.allows('move',{**health,'fas_local_rotation':False}))
        self.assertFalse(lease.allows('move',{**health,'pose_age_s':.8}))
        self.assertFalse(lease.allows('move',{**health,'pose':(1.05,0.,0.)}))

    def test_occupied_slip_site_can_only_be_left_with_a_checked_retreat(self):
        blockage=slip_obstruction((0.,0.,0.),(0.,0.,0.),(.11,0.,.02))
        self.assertEqual(blockage['direction'],'front')
        h={'ready':True,'localization_confirmed':True,'pose':(0.,0.,0.),'pose_age_s':.03,
           'localization_covariance':(.001,.001,.002),'scan_map_agreement':.91,'fas_obstruction':blockage}
        path=((0.,0.),(-.2,0.))
        lease=CorridorLease(lambda:0.);lease.owner='run'
        lease.reserve(uuid.uuid4().hex,'move',(-.7,.7,-.6,.6))
        self.assertFalse(lease.allows('move',h,path))
        lease.reserve(uuid.uuid4().hex,'retreat',(-.7,.7,-.6,.6),[-.65,0.])
        self.assertFalse(lease.allows('retreat',h,path))
        lease.reserve(uuid.uuid4().hex,'retreat',(-.7,.7,-.6,.6))
        self.assertTrue(lease.allows('retreat',h,path))
        self.assertFalse(obstruction_clear(h,((-.2,0.),(.7,0.))))
        self.assertTrue(obstruction_clear(h,((-.2,0.),(-.9,0.))))
        self.assertEqual(slip_obstruction((0.,0.,0.),(0.,0.,0.),(-.11,0.,0.))['direction'],'rear')
        self.assertEqual(slip_obstruction((0.,0.,0.),(0.,0.,0.),(0.,0.,.16))['direction'],'unknown')


class MotionSimulationTests(TestCase):
    @classmethod
    def setUpClass(cls):
        tool = Path(__file__).resolve().parents[2]/'tools/fas_experiment.py'
        spec = importlib.util.spec_from_file_location('fas_host_test',tool)
        cls.host = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.host)

    def driver(self, starts, points):
        settings = {'cars':{'A':'http://127.0.0.1:18890','B':'http://127.0.0.1:18892'}}
        stations = SimpleNamespace(points={name:{'id':name,'x':x,'y':0.,'yaw':0.} for name,x in points.items()})
        driver = self.host.Live(settings,Path('.'),stations,None,{},(),motion_only=True)
        self.addCleanup(driver.pool.shutdown)
        driver.states = {car:{'health':{'pose':[x,0.,0.],'localization_covariance':(.002,.002,.002)}}
                         for car,x in zip(('A','B'),starts)}
        return driver

    def test_parallel_preference_checks_the_whole_native_path(self):
        driver = self.driver((0.,2.),{'a-short':-.3,'b-short':2.3,'a-clear':-.4,'b-clear':2.4})
        first, second = ('a-short','b-short'),('a-clear','b-clear')
        def preview(car, point):
            start = driver.states[car]['health']['pose'][:2]
            path = [start,[point['x'],0.]]
            if point['id']=='a-short':
                path.insert(1,[1.1,0.])
            return path
        driver.preview = preview
        self.assertEqual(driver.order_motion_pairs([first,second],2),[second,first])

    def test_shared_start_prefers_a_single_safe_separation_move(self):
        driver = self.driver((1.,2.),{'a-far':.2,'b-far':2.5,'a-near':.4,'b-ready':2.})
        first, second = ('a-far','b-far'),('a-near','b-ready')
        driver.preview = lambda *_: self.fail('overlapping initial reservations cannot be parallel')
        self.assertEqual(driver.order_motion_pairs([first,second],2),[second,first])

    def test_motion_during_simulated_measurement_invalidates_window(self):
        driver = self.driver((0.,2.),{'a':0.,'b':2.})
        driver.window_s = 2.
        for car,state in driver.states.items():
            state['fas'] = {'operator_hold':False,'lease_live':False}
            state['health'].update(ready=True,localization_confirmed=True,stationary=car=='A')
        driver.sync = lambda:driver.states
        before = {car:{'pose':s['health']['pose']} for car,s in driver.states.items()}
        with redirect_stdout(io.StringIO()), self.assertRaises(self.host.RouteRetry):
            driver.simulate_window(('a','b'),before)

    def test_lease_timeout_at_a_verified_stop_does_not_repeat_the_drive(self):
        for compact in (True,False):
            with self.subTest(compact=compact):
                driver = self.driver((0.,2.),{'a':.3,'b':2.})
                driver.compact_status['A'] = compact
                driver.journal = SimpleNamespace(run_id='run')
                driver.stations.map_id = 'a'*64
                for car,state in driver.states.items():
                    state.update(phase='idle',map={'map_id':driver.stations.map_id},
                                 fas={'car':car,'protocol':PROTOCOL,'tx':{},'owner':'run','operator_hold':False,
                                      'motion_only':True,'energy':{'low_voltage':10.,'low_hold_seconds':20.}})
                    state['health'].update(ready=True,localization_confirmed=True,stationary=True,
                                           dock_calibrated=True,pose_age_s=.05,voltage=11.8)
                a,b=driver.states['A'],driver.states['B']
                stopped={**a,'phase':'completed','health':{**a['health'],'pose':[.301,0.,.02],
                         'arrival':{'kind':'waypoint','status':'succeeded','position_error':.001,'yaw_error_deg':1.2}}}
                driver.car_state=lambda car:stopped if car=='A' else b
                sent=[]
                def command(car,action,**_):
                    sent.append((car,action))
                    if action=='fas_renew':
                        raise ValueError('行驶许可已过期，请停稳后重新预约')
                    return {}
                driver.command=command
                driver.stop=lambda cars:sent.append((cars[0],'stop'))
                point={**driver.stations.points['a'],'name':'A'}
                with redirect_stdout(io.StringIO()):
                    driver.drive_batch(('A',),{'A':[(0.,0.),(.3,0.)]},{'A':point},{'A':0.})
                self.assertEqual(sent,[('A','fas_reserve'),('A','fas_drive'),('A','fas_renew'),('A','stop')])

    def test_retry_moves_only_the_car_that_has_not_reached_its_station(self):
        driver = self.driver((.050,2.),{'a':0.,'b':2.4})
        driver.positions=lambda:('a','b')
        driver.wait_parked=lambda:None
        driver.measured=lambda pair:pair
        calls=[]
        def preview(car, point):
            calls.append(('plan',car))
            return [(2.,0.),(point['x'],0.)]
        driver.preview=preview
        driver.drive_batch=lambda cars,*_:calls.append(('drive',cars))
        self.assertEqual(driver.place_once(('a','b')),('a','b'))
        self.assertEqual(calls,[('plan','B'),('drive',('B',))])

    def test_return_replans_when_the_controller_oscillates_without_travel(self):
        driver=self.driver((0.,3.),{'a':1.5,'b':3.})
        driver.command=lambda car,action,**kwargs:None
        stops=[]
        driver.stop=stops.append
        for car,state in driver.states.items():
            state.update(phase='returning' if car=='A' else 'idle',fas={'operator_hold':False},dock=(1.5,0.,0.))
            state['health'].update(ready=True,localization_confirmed=True,stationary=car=='B',charging_stable=False)

        def sync(**kwargs):
            health=driver.states['A']['health']
            health.update(pose=(-health['pose'][0] or .022,0.,0.),arrival={'status':'navigating'})
            return driver.states

        driver.sync=sync
        times=iter(range(0,45,3))
        with patch.object(self.host.time,'monotonic',side_effect=lambda:next(times)), patch.object(self.host.time,'sleep'):
            with self.assertRaisesRegex(self.host.RouteRetry,'无有效位移'):
                driver.drive_batch(('A',),{'A':[(0.,0.),(1.5,0.)]},{'A':None},{},returning=True)
        self.assertEqual(stops,[('A',)])

    def test_control_release_uses_parked_state_after_stop_and_release_replies_drop(self):
        driver = self.driver((0.,2.),{'a':0.,'b':2.4})
        driver.journal = SimpleNamespace(run_id='run')
        a = {'phase':'navigating','health':{'stationary':False},'fas':{'owner':'run','lease_live':True}}
        b = {'fas':{'owner':None}}
        driver.car_state = lambda car: a if car=='A' else b
        commands = []
        def command(car, action, **kwargs):
            commands.append((car,action))
            if action=='fas_stop':
                a.update(phase='idle',health={'stationary':True},fas={'owner':'run','lease_live':False})
            else:
                a['fas']['owner'] = None
            raise TimeoutError('reply dropped')
        driver.command = command
        now = [0.]
        clock = SimpleNamespace(monotonic=lambda:now[0],sleep=lambda delay:now.__setitem__(0,now[0]+delay))
        with patch.object(self.host,'time',clock),redirect_stdout(io.StringIO()):
            self.assertEqual(driver.release_available(),0)
        self.assertEqual(commands,[('A','fas_stop'),('A','fas_release')])
        self.assertFalse(driver.claimed)

    def test_charge_timer_uses_vehicle_evidence_after_a_host_link_drop(self):
        driver = self.driver((0.,2.),{'a':0.,'b':2.})
        driver.wait_parked=lambda:None
        driver.recover_obstruction=lambda:None
        driver.low_cars=lambda:[]
        driver.charge_timeout=10.
        start=100.
        now=[start]
        config={'resume_voltage':12.2,'charge_hold_seconds':2.,'min_charge_seconds':60.}
        driver.states['A'].update(fas={'energy':config},health={'charging_stable':True,'stationary':True,
            'voltage':12.3,'charging_elapsed_s':59.,'localization_confirmed':True,'dock_localization':{'status':'confirmed'}})
        driver.states['B']['health']['stationary']=True
        def sync():
            if now[0]==101.:
                raise OSError('temporary host link failure')
            driver.states['A']['health']['charging_elapsed_s']=59.+now[0]-start
            return driver.states
        driver.sync=sync
        with TemporaryDirectory() as directory:
            journal=Journal(Path(directory)/'charge.sqlite',{'motion_only':True})
            try:
                journal.set_charging(['A'])
                driver.journal=journal
                clock=SimpleNamespace(monotonic=lambda:now[0],sleep=lambda seconds:now.__setitem__(0,now[0]+seconds))
                with patch.object(self.host,'time',clock),redirect_stdout(io.StringIO()):
                    driver.service_energy()
                self.assertEqual(journal.charging(),[])
                self.assertEqual(now[0],104.)
            finally:
                journal.close()

    def test_charge_plateau_rejects_rising_voltage_and_resets_after_contact_or_telemetry_loss(self):
        window=self.host.ChargePlateau()
        health={'stationary':True,'charging_stable':True,'localization_confirmed':True,
                'dock_localization':{'status':'confirmed'}}

        def observe(now,voltage):
            health.update(charging_elapsed_s=now,voltage=voltage)
            window.observe(now,health,11.4)

        for now in range(0,241,5):
            observe(now,11.5+now*.000055)
        self.assertFalse(window.ready)
        for now in range(245,365,5):
            observe(now,11.54+(.005 if now%10 else -.005))
        self.assertFalse(window.ready)
        for now in range(365,486,5):
            observe(now,11.54+(.005 if now%10 else -.005))
        self.assertTrue(window.ready)
        observe(500,11.54)
        self.assertFalse(window.ready)
        for now in range(505,740,5):
            observe(now,11.54)
        self.assertFalse(window.ready)
        observe(740,11.54)
        self.assertTrue(window.ready)
        observe(741,11.565)
        self.assertFalse(window.ready)
        health['charging_stable']=False
        observe(746,11.54)
        self.assertFalse(window.ready)
        self.assertFalse(window.samples)

    def test_near_full_requires_taper_and_can_retain_a_verified_charge_at_the_same_dock(self):
        window=self.host.ChargePlateau(near_full=12.1)
        health={'stationary':True,'charging_stable':True,'localization_confirmed':True,
                'dock_localization':{'status':'confirmed'},'current':1.54,
                'pose':[1.,2.,0.], 'pose_age_s':.1}
        def step(t, voltage):
            health.update(charging_elapsed_s=300.+t,voltage=voltage)
            window.observe(t,health,11.4)
        for t in range(0,361,5):
            step(t,11.46)
        self.assertFalse(window.ready)
        health['current']=.42
        for t in range(365,486,5):
            step(t,12.12)
        self.assertTrue(window.ready)
        health['charging_stable']=False
        step(486,12.03)
        self.assertTrue(window.retained)
        health['pose'][0]+=.08
        step(487,12.03)
        self.assertFalse(window.ready)

    def test_lower_charge_plateau_requires_sustained_current_taper(self):
        window=self.host.ChargePlateau(near_full=12.1)
        health={'stationary':True,'charging_stable':True,'localization_confirmed':True,
                'dock_localization':{'status':'confirmed'},'current':.42,'voltage':11.49}
        for t in range(0,361,5):
            health['charging_elapsed_s']=t
            window.observe(t,health,11.4)
        self.assertFalse(window.ready)
        for t in range(365,426,5):
            health.update(current=1.55,charging_elapsed_s=t)
            window.observe(t,health,11.4)
        for t in range(430,796,5):
            health.update(current=.42,charging_elapsed_s=t)
            window.observe(t,health,11.4)
        self.assertTrue(window.ready)
        self.assertIn('平台',window.reason)

    def test_bulk_evidence_survives_a_link_gap_or_restart_only_for_the_same_onboard_charge(self):
        health={'stationary':True,'charging_stable':True,'localization_confirmed':True,
                'dock_localization':{'status':'confirmed'},'pose':[0.,0.,0.],'voltage':11.67}
        def observe(window, stamp, elapsed, current, *, clock_offset=0.):
            health.update(current=current,charging_elapsed_s=elapsed)
            window.observe(stamp+clock_offset,health,11.4,12.2,wall_stamp=100000.+stamp)

        with TemporaryDirectory() as directory:
            path=Path(directory)/'charge.sqlite'
            first=Journal(path,{'order':'pair-first'})
            try:
                live=self.host.ChargePlateau(12.1,first.bulk_charge_start('A'))
                for t in range(0,331,5):
                    observe(live,t,600.+t,1.52 if t<135 else .42)
                self.assertIsNotNone(live.bulk_charge_start)
                self.assertFalse(live.ready)
                first.set_bulk_charge_start('A',live.bulk_charge_start)
            finally:
                first.close()
            reopened=Journal(path,{'order':'pair-first'})
            try:
                self.assertIsNone(reopened.bulk_charge_start('B'))
                restored=self.host.ChargePlateau(12.1,reopened.bulk_charge_start('A'))
                reseated=self.host.ChargePlateau(12.1,reopened.bulk_charge_start('A'))
                for t in range(355,716,5):
                    observe(live,t,600.+t,.42)
                    observe(restored,t,600.+t,.42,clock_offset=-355.)
                    observe(reseated,t,t-355.,.42,clock_offset=-355.)
                    self.assertEqual((live.ready,restored.ready),(t>=715,t>=715))
                    self.assertFalse(reseated.ready)
                self.assertIsNone(reseated.bulk_charge_start)
                reopened.set_bulk_charge_start('A',None)
                self.assertIsNone(reopened.bulk_charge_start('A'))
            finally:
                reopened.close()

    def test_long_charge_plateau_uses_same_car_sustained_bulk_and_continuous_tail(self):
        start=100000.
        with TemporaryDirectory() as directory:
            path=Path(directory)/'energy.sqlite'
            db=sqlite3.connect(path)
            try:
                db.execute('CREATE TABLE samples(car,stamp,mode,charge_start,voltage,current,PRIMARY KEY(car,stamp))')
                for c in 'AB':
                    rows=[]
                    for t in range(0,3601,5):
                        voltage=11.67+.002*(t%10==0)
                        if c=='B':
                            voltage+=.00004*t
                        amps=1.56 if t<1600 else .93
                        rows.append((c,start+t,'charging',start,voltage,amps))
                    db.executemany('INSERT INTO samples VALUES (?,?,?,?,?,?)',rows)
                db.commit()
                def read(car,origin,floor):
                    return self.host.recorded_charge_plateau(path,car,origin,floor,now=start+3600.)

                self.assertIsNotNone(read('A',start,11.4))
                self.assertIsNone(read('B',start,11.4))
                self.assertIsNone(read('A',start+60.,11.4))
                self.assertIsNone(read('A',start,11.8))
                health=dict(stationary=True,charging_stable=True,localization_confirmed=True,
                            dock_localization={'status':'confirmed'},voltage=11.67,current=.93)
                window=self.host.ChargePlateau(12.1,start,lambda origin,floor:read('A',origin,floor))
                for t in range(0,361,5):
                    health['charging_elapsed_s']=3240.+t
                    window.observe(t,health,11.4,wall_stamp=start+3240.+t)
                    self.assertEqual(window.ready,t==360)
                self.assertIn('24',window.reason)
                db.execute("UPDATE samples SET current=1.40 WHERE car='A' AND stamp>=?",(start+2000,))
                db.commit()
                self.assertIsNone(read('A',start,11.4))
                db.execute("UPDATE samples SET current=.93 WHERE car='A' AND stamp>=?",(start+2000,))
                db.execute("UPDATE samples SET mode='unknown' WHERE car='A' AND stamp=?",(start+3500,))
                db.commit()
                self.assertIsNone(read('A',start,11.4))
                db.execute("UPDATE samples SET mode='charging' WHERE car='A' AND stamp=?",(start+3500,))
                db.execute("UPDATE samples SET current=.97 WHERE car='A' AND stamp<?",(start+2000,))
                db.execute("UPDATE samples SET current=1.62 WHERE car='A' AND stamp=?",(start+300,))
                db.commit()
                self.assertIsNone(read('A',start,11.4))
            finally:
                db.close()

    def test_short_return_needs_a_recent_clear_plan_and_keeps_fast_low_voltage_trigger(self):
        driver=self.driver((0.,2.),{'a':0.,'b':2.})
        driver.near_return_voltage_by_car={'A':10.3}
        for car,state in driver.states.items():
            state['fas']={'energy':{'low_voltage':10.4,'resume_voltage':11.4,'low_hold_seconds':20.}}
            state['health'].update(voltage=10.36 if car=='A' else 10.7,stationary=True)
        driver.short_return_checks['A']=(100.,{'A':(0.,0.),'B':(2.,0.)},True)
        now=[100.]
        with patch.object(self.host.time,'monotonic',side_effect=lambda:now[0]):
            self.assertEqual(driver.low_cars(),[])
            now[0]=121.
            self.assertEqual(driver.low_cars(),[])
            driver.states['B']['health']['pose'][0]=2.1
            self.assertEqual(driver.low_cars(),[])
            now[0]=141.
            self.assertEqual(driver.low_cars(),['A'])
            now[0]=142.
            driver.states['A']['health']['voltage']=10.48
            self.assertEqual(driver.low_cars(),[])
            now[0]=143.
            driver.states['A']['health']['voltage']=10.19
            self.assertEqual(driver.low_cars(),[])
            now[0]=146.
            self.assertEqual(driver.low_cars(),['A'])

    def test_contact_release_at_start_does_not_requeue_a_qualified_departure(self):
        driver=self.driver((0.,2.),{'a':0.,'b':2.})
        driver.motion_only=False
        driver.charge_plateaus={'A':self.host.ChargePlateau(12.1)}
        for car,state in driver.states.items():
            state.update(phase='departing' if car=='A' else 'completed',dock=[0.,0.,0.],
                         fas={'lease_live':car=='A', 'energy':{'low_voltage':10.4,'resume_voltage':11.4,
                                                              'low_hold_seconds':20.}})
            state['health'].update(stationary=True,charging_stable=False,voltage=11.7)
        a=driver.states['A']
        a['health']['pose']=[.03,0.,0.]
        self.assertEqual(driver.low_cars(),[])
        a['health']['charging_stable']=True
        self.assertEqual(driver.low_cars(),[])
        a['fas']['lease_live']=False
        self.assertEqual(driver.low_cars(),['A'])
        a['fas']['lease_live']=True
        a['health'].update(charging_stable=False,voltage=10.15)
        self.assertEqual(driver.low_cars(),['A'])
        a['health']['voltage']=11.7
        a['phase']='idle'
        self.assertEqual(driver.low_cars(),['A'])

    def test_early_launch_requires_both_confirmed_docks_and_keeps_return_margin(self):
        driver=self.driver((0.,2.),{'a':0.,'b':2.})
        driver.early_launch_voltage=11.25
        for car,state in driver.states.items():
            state['fas']={'energy':{'low_voltage':10.4,'resume_voltage':11.4}}
            state['health'].update(voltage=11.29 if car=='A' else 11.86,stationary=True,
                ready=True,charging_stable=True,localization_confirmed=True,
                dock_localization={'status':'confirmed'},pose_age_s=.1,scan_window_agreement=.92)
        driver.check_early_launch()
        self.assertEqual(driver.initial_departure,{'A':11.25,'B':11.25})
        driver.states['B']['health']['charging_stable']=False
        with self.assertRaisesRegex(ValueError,'B 车未达到'):
            driver.check_early_launch()
        driver.early_launch_voltage=10.8
        with self.assertRaisesRegex(ValueError,'0.70 V'):
            driver.check_early_launch()

    def test_joint_charge_waits_for_both_and_rechecks_after_a_contact_drop(self):
        driver = self.driver((0.,2.),{'a':0.,'b':2.})
        driver.motion_only=False
        driver.charge_plateaus={car:self.host.ChargePlateau() for car in ('A','B')}
        driver.wait_parked=lambda:None
        driver.recover_obstruction=lambda:None
        driver.low_cars=lambda:['A']
        docking=[]
        def dock(car, *, allow_yield=None):
            docking.append((car,allow_yield))
            if len(docking)==1:
                raise self.host.DockRouteConflict('B blocks A')
        driver.ensure_charging_dock=dock
        batches=[]

        def queue(todo, manual, *, joint):
            self.assertTrue(joint)
            self.assertFalse(manual)
            self.assertEqual(driver.journal.charging(),['A','B'])
            batches.append(tuple(todo))
            for car in todo:
                driver.charge_plateaus[car].ready=True
                driver.states[car]['health']['charging_elapsed_s']=120.
                driver.states[car]['fas']={'energy':{'min_charge_seconds':120.}}

        def sync():
            if len(batches)==1:
                driver.charge_plateaus['A'].ready=False
            return driver.states

        driver.service_charge_queue=queue
        driver.sync=sync
        with TemporaryDirectory() as directory:
            journal=Journal(Path(directory)/'charge.sqlite',{'motion_only':False})
            try:
                driver.journal=journal
                driver.service_energy()
                self.assertEqual(docking,[('A',None),('B',None),('A',True),('A',None)])
                self.assertEqual(batches,[('A','B'),('A',)])
                self.assertTrue(driver.dual_departure_pending)
                self.assertEqual(journal.charging(),['A','B'])
            finally:
                journal.close()

    def test_joint_charge_detects_peer_contact_loss_while_first_car_still_charges(self):
        driver=self.driver((0.,2.),{'a':0.,'b':2.})
        driver.charge_timeout=120.
        driver.charge_plateaus={car:self.host.ChargePlateau(12.1) for car in ('A','B')}
        driver.ensure_charging_dock=lambda car:None
        driver.journal=SimpleNamespace(set_activity=lambda *args:None)
        for state in driver.states.values():
            state.update(fas={'energy':{'resume_voltage':11.4,'charge_hold_seconds':15.,'min_charge_seconds':300.}},
                         health={'charging_stable':True,'stationary':True,'voltage':11.,
                                 'localization_confirmed':True,'dock_localization':{'status':'confirmed'}})
        now=[0.]
        def sync():
            if now[0]==10.:
                raise OSError('missing telemetry')
            # Lost contact at t=1, brief recontact at 8; a qualified full peer can stop charging.
            driver.states['B']['health']['charging_stable']=now[0] in (0.,8.)
            driver.charge_plateaus['B'].retained=12.<=now[0]<15.
        driver.sync=sync
        clock=SimpleNamespace(monotonic=lambda:now[0],sleep=lambda dt:now.__setitem__(0,now[0]+dt))
        with patch.object(self.host,'time',clock),self.assertRaisesRegex(RecoveryWait,'B 车充电中断'):
            driver.service_charge_queue(['A','B'],False,joint=True)
        self.assertEqual(now[0],27.)

    def test_live_capture_recovers_link_or_station_drift_without_losing_the_pair(self):
        driver = self.driver((0.,2.),{'a':0.,'b':2.})
        driver.motion_only=False
        events=[]
        driver.check_pause=lambda:None
        driver.stop=lambda:events.append('stop')
        driver.off=lambda:events.append('radio-off')
        driver.wait_retry=lambda delay:events.append(delay)
        driver.recover_capture_pose=lambda:None
        driver.wait_parked=lambda:events.append('both-parked')
        results=iter((self.host.VehicleLinkInterrupted('车辆通信暂时中断'), None, None))
        pose_checks=iter((self.host.RouteRetry('停靠位置需要复核'), None))
        with TemporaryDirectory() as directory:
            journal=Journal(Path(directory)/'capture.sqlite',{'motion_only':False})
            try:
                driver.journal=journal
                with journal.connection:
                    journal.connection.execute("INSERT INTO pair_sweeps VALUES ('a','b','a','b','active')")

                def collect():
                    pair=tuple(journal.connection.execute("SELECT a,b FROM pair_sweeps WHERE status='active'").fetchone())
                    driver.place_once=lambda pair, **kwargs: next_pose(pair)
                    reached=driver.place(pair)
                    fault=next(pose_checks)
                    if fault:
                        raise fault
                    return reached

                def next_pose(pair):
                    events.append(pair)
                    result=next(results)
                    if result:
                        raise result
                    return 'resumed'

                with redirect_stdout(io.StringIO()):
                    self.assertEqual(self.host.continuous_capture(driver,collect),'resumed')
                self.assertEqual(events,[('a','b'),'stop','radio-off',2.,'both-parked',
                                         ('a','b'),'stop','radio-off',4.,'both-parked',('a','b')])
            finally:
                journal.close()

    def test_capture_timeout_confirms_close_before_requesting_retry(self):
        events = []
        base = SimpleNamespace(post=lambda *_: events.append('open'),
                               cancel=lambda _: events.append('close'))
        driver = SimpleNamespace(base=base, wait_parked=lambda: None, parallel=lambda *_: None,
                                 off=lambda: events.append('radio-off'))
        request = {'cars': {'A': {'point_id': 'a'}, 'B': {'point_id': 'b'}}, 'window_s': 1.}
        with patch.object(self.host.time, 'monotonic', side_effect=[0., 25.]):
            with self.assertRaisesRegex(RecoveryWait, '基站采样超时'):
                self.host.Live.capture(driver, request)
        self.assertEqual(events, ['open', 'close', 'radio-off'])

    def test_wait_for_dock_fit_observes_peer_before_another_return(self):
        driver = self.driver((0., 2.), {'a': 0., 'b': 2.})
        driver.motion_only = False
        for state in driver.states.values():
            state.update(phase='idle', fas={'lease_live': False})
            state['health']['stationary'] = True
        phase = driver.states['A']['health']
        phase.update(charging_stable=True, dock_localization={'status': 'applying'})
        checks = []

        def poll():
            checks.append(phase['dock_localization']['status'])
            if len(checks) == 2:
                phase['dock_localization']['status'] = 'confirmed'

        driver.sync = poll
        with patch.object(self.host, 'fleet_pose_ready', return_value=True), patch.object(self.host.time, 'sleep'):
            driver.wait_parked(wait_for_dock_fit=True)
        self.assertEqual(checks, ['applying', 'applying'])

    def test_base_busy_retries_while_protocol_conflicts_still_stop(self):
        base = self.host.Base.__new__(self.host.Base)
        base.headers = {}
        for reason, exception in (('基站已在采样', RecoveryWait), ('协议或 A/B 发端身份不匹配', ValueError)):
            with self.subTest(reason=reason):
                def reject(*_args, **_kw):
                    raise ValueError(reason)
                base.api = SimpleNamespace(fetch=reject)
                with self.assertRaises(exception) as failure:
                    base.post('/fas/capture', {})
                self.assertIs(type(failure.exception), exception)

    def test_truncated_http_payload_is_recoverable_while_complete_invalid_json_is_rejected(self):
        endpoint=self.host.Endpoint('http://127.0.0.1:5100')
        for length,error in ((40,OSError),(5,ValueError)):
            with self.subTest(length=length):
                wire=io.BytesIO(f'HTTP/1.1 200 OK\r\nContent-Length: {length}\r\n\r\n'.encode()+b'{"x":')
                response=HTTPResponse(SimpleNamespace(makefile=lambda *_:wire))
                response.begin()
                with patch.object(self.host,'urlopen',return_value=response),self.assertRaises(error) as caught:
                    endpoint.fetch('/api/fas/state')
                if error is OSError:
                    self.assertIsInstance(caught.exception.__cause__,IncompleteRead)

    def test_live_recovery_only_reclaims_after_stationary_alignment_verification(self):
        driver=self.driver((0.,2.),{'a':0.,'b':2.})
        driver.journal=SimpleNamespace(run_id='run',pending=lambda:None,set_activity=lambda *_:None)
        driver.sync=lambda:driver.states
        driver.car_state=lambda car:driver.states[car]
        events=[]
        for car,state in driver.states.items():
            state.update(phase='idle',fas={'owner':'run','lease_live':False,'recovery':'available'},control_token='token')
            state['health'].update(ready=True,stationary=True,map_matches=True,pose_age_s=.1,
                                   localization_confirmed=car=='B',scan_map_agreement=.9 if car=='B' else .68)

        def command(car, action, **_):
            events.append((car,action))
            driver.states[car]['fas']['owner']=None if action=='fas_release' else 'run'

        driver.command=command
        driver.cars['A']=SimpleNamespace(fetch=lambda path,data,headers,timeout:events.append(('A',data['action'])))
        driver.verify_capture_alignment=lambda car,pose:events.append((car,'verified'))
        driver.recover_capture_pose()
        self.assertEqual(events,[('A','fas_release'),('A','align_pose'),('A','verified'),('A','fas_claim')])

    def test_unattended_recovers_network_localization_and_charge_across_batches(self):
        stations=Stations({'map_id':'a'*64,'points':uniform_route([(0.,0.),(8.,0.)],50,0.)})
        with TemporaryDirectory() as directory:
            journal=Journal(Path(directory)/'motion.sqlite',{'motion_only':True})
            try:
                driver=Driver(stations)
                driver.journal=journal
                resumes=[];stops=[];waits=[];charge=[]
                drive=driver.place
                failures=iter((OSError('车端掉线'),RecoveryWait('等待匹配'),self.host.ChargeNeeded(['B'])))
                def start(*args):
                    resumes.append(args[-1])
                def place(pair):
                    error=next(failures,None)
                    if error:
                        raise error
                    return drive(pair)
                def service():
                    if journal.charging():
                        charge.append(journal.charging())
                        if len(charge)==1:
                            raise RecoveryWait('触点暂时断开')
                        journal.set_charging([])
                def pause():
                    if len(journal.report())>=3:
                        raise self.host.PauseRequested
                def retry(delay):
                    self.assertIsNone(journal.state(0))
                    waits.append(delay)
                driver.start=start
                driver.place=place
                driver.service_energy=service
                driver.check_pause=pause
                driver.stop=lambda:stops.append(True)
                driver.wait_retry=retry
                driver.order_motion_pairs=lambda ranked,_:ranked
                with redirect_stdout(io.StringIO()), self.assertRaises(self.host.PauseRequested):
                    self.host.unattended_motion(driver,start=0,count=1,max_pairs=2,recharge_after=None,
                                                window_s=2.,resume_operator=True)
                self.assertEqual(resumes,[True])
                self.assertEqual(waits,[2.,4.,2.])
                self.assertEqual(charge,[['B'],['B']])
                self.assertEqual(len(stops),4)
                self.assertEqual([row['state_id'] for row in journal.report()],[0,1,2])
                self.assertEqual(journal.charging(),[])
            finally:
                journal.close()


class Driver:
    def __init__(self, stations):
        self.stations = stations
        self.pair = ('fixed-001','fixed-050')
        self.bad = set()
        self.interrupt = None
        self.calls = []

    def positions(self):
        return self.pair

    def place(self,pair):
        self.pair = pair
        return {k:{'point_id':v,'pose':[self.stations.points[v][name] for name in ('x','y','yaw')]}
                for k,v in zip(('A','B'),pair)}

    def service_energy(self):
        pass

    def reconcile(self,_):
        return None

    def capture(self, request):
        state=request['state_id']
        self.calls.append(state)
        if self.interrupt==state:
            raise ConnectionError('station disconnected')
        if state in self.bad:
            return receipt(request,sle=(-70,-60))
        return receipt(request)


class FasSearchTests(TestCase):
    def setUp(self):
        self.tmp=TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path=Path(self.tmp.name)/'search.sqlite'
        self.stations=Stations({'map_id':'a'*64,
            'points':uniform_route([(0.,0.),(5.,0.),(5.,3.),(0.,3.)],50,math.pi/2)})
        self.driver=Driver(self.stations)

    def run_range(self,journal,count=4):
        with redirect_stdout(io.StringIO()):
            return search(journal,self.stations,CODES,LINKS,RECEIVERS,self.driver,count=count,max_pairs=2)

    def test_restart_keeps_completed_states_and_replays_only_unverified_trial(self):
        self.driver.bad={1}
        self.driver.interrupt=2
        first=Journal(self.path,{'scene':self.stations.map_id})
        with self.assertRaises(ConnectionError):
            self.run_range(first)
        self.assertEqual(self.driver.calls,[0,1,1,2])
        self.assertEqual(first.pending()['state_id'],2)
        with self.assertRaises(ValueError):
            Journal(self.path,{'scene':self.stations.map_id})
        first.close()
        second=Journal(self.path,{'scene':self.stations.map_id})
        self.addCleanup(second.close)
        self.driver.interrupt=None
        records=self.run_range(second)
        self.assertEqual(self.driver.calls,[0,1,1,2,2,3])
        self.assertEqual([r['status'] for r in records],['pass','budget_exhausted','pass','pass'])
        self.assertEqual(records[2]['pair'], records[3]['pair'])
        self.assertEqual(second.connection.execute("SELECT COUNT(*) FROM trials WHERE status='invalid'").fetchone()[0],1)

    def test_paused_charge_state_is_durable(self):
        first=Journal(self.path,{'scene':self.stations.map_id})
        first.set_charging(['B'])
        first.close()
        second=Journal(self.path,{'scene':self.stations.map_id})
        self.addCleanup(second.close)
        self.assertEqual(second.charging(),['B'])
        self.assertEqual(second.report(),[])

    def test_pair_selected_for_travel_survives_link_loss_before_arrival(self):
        config={'order':'pair-first'}
        route=self.stations.pairs
        pair=route[4]
        self.stations.pair_candidates=lambda *_args,**_kwargs: iter((pair,))
        journal=Journal(self.path,config)
        try:
            place=self.driver.place
            def disconnected(_):
                raise ConnectionError('vehicle link lost')
            self.driver.place=disconnected
            with self.assertRaises(ConnectionError):
                sweep_pairs(journal,self.stations,CODES,LINKS,RECEIVERS,self.driver,count=1,max_pairs=1)
            self.assertEqual(journal.positioning_pair(),pair)
            self.assertEqual(journal.report(),[])
        finally:
            journal.close()
        self.driver.place=place
        self.stations.pair_candidates=lambda *_args,**_kwargs: iter((route[-1],))
        journal=Journal(self.path,config)
        self.addCleanup(journal.close)
        with redirect_stdout(io.StringIO()):
            history=sweep_pairs(journal,self.stations,CODES,LINKS,RECEIVERS,self.driver,count=1,max_pairs=1)
        self.assertEqual(tuple(history[0]['pair']),pair)
        self.assertIsNone(journal.positioning_pair())

    def test_pair_sweep_resumes_at_missing_state_and_stops_at_first_joint_hit(self):
        self.driver.bad={0}
        self.driver.interrupt=2
        capture=self.driver.capture
        timing=[]
        def alternating(request):
            timing.append((request['window_s'],request.get('c5_window_s'),request.get('parallel_radios',False)))
            result=capture(request)
            if request['state_id']==1:
                return receipt(request,c5=(-68,-62))
            return result
        self.driver.capture=alternating
        config={'order':'pair-first'}
        journal=Journal(self.path,config)
        try:
            with redirect_stdout(io.StringIO()), self.assertRaises(ConnectionError):
                sweep_pairs(journal,self.stations,CODES,LINKS,RECEIVERS,self.driver,count=4,max_pairs=2)
            pair=journal.report()[0]['pair']
            self.assertEqual(journal.report()[0]['hits'],[])
            self.assertEqual(journal.report()[0]['tested_states'],2)
        finally:
            journal.close()
        self.driver.interrupt=None
        journal=Journal(self.path,config)
        self.addCleanup(journal.close)
        with redirect_stdout(io.StringIO()):
            records=sweep_pairs(journal,self.stations,CODES,LINKS,RECEIVERS,self.driver,count=4,max_pairs=2,
                                window_s=1.4,c5_window_s=.8,parallel_radios=True)
        self.assertEqual(self.driver.calls,[0,1,2,2,0,1,2])
        self.assertEqual(timing,[(1.,None,False)]*4+[(1.4,.8,True)]*3)
        self.assertEqual(records[0]['pair'],pair)
        self.assertEqual(len({frozenset(row['pair']) for row in records}),2)
        self.assertTrue(all(row['status']=='hit' and row['hits']==[2] for row in records))

    def test_pair_draw_shuffles_unvisited_pairs_and_car_assignments(self):
        eligible = [pair for pair in self.stations.pairs if pair[0] < pair[1]]
        visited = {eligible[0]}
        with patch('fas_experiment.random.shuffle', side_effect=Random(7).shuffle):
            candidates = list(self.stations.pair_candidates(visited))
        draws = [tuple(sorted(pair)) for pair in candidates[::2]]
        self.assertCountEqual(draws, eligible[1:])
        self.assertNotEqual(draws, eligible[1:])
        self.assertTrue(any(a > b for a,b in candidates[::2]))
        self.assertEqual([pair[::-1] for pair in candidates[::2]], candidates[1::2])

    def test_evidence_sampling_avoids_recent_anchors_and_stays_random(self):
        coords={'a':(0.,0.),'b':(.25,0.),'c':(2.,0.),'d':(3.,0.),
                'e':(5.,0.),'f':(6.,0.),'g':(8.,0.),'h':(9.,0.)}
        stations=Stations({'map_id':'a'*64,'points':[
            {'id':key,'x':x,'y':y,'yaw':0.} for key,(x,y) in coords.items()]})
        observations=[PairObservation(('a','e'),'hit',7.),PairObservation(('a','f'),'hit',8.)]
        choices=set()
        for seed in range(12):
            rng=Random(seed)
            with patch.multiple('fas_experiment.random', shuffle=rng.shuffle,expovariate=rng.expovariate,random=rng.random):
                picks=stations.pair_candidates({('a','e'),('a','f')},minimum_spacing=.9,observations=observations)
                pair=next(picks)
                self.assertTrue(set(pair).isdisjoint('abef'))
                self.assertEqual(next(picks),pair[::-1])
                choices.add(tuple(sorted(pair)))
        self.assertGreater(len(choices),1)

    def test_local_signal_prior_uses_only_matching_direction_and_verified_results(self):
        coords={'a':(0.,0.),'b':(.5,0.),'c':(3.,0.),'d':(3.5,0.)}
        distances={(a,b):math.dist(first,second) for a,first in coords.items() for b,second in coords.items()}
        self.assertGreater(Stations.signal_weight(('b','d'),[PairObservation(('a','c'),'hit',6.2)],distances),1.)
        self.assertLess(Stations.signal_weight(('b','d'),[PairObservation(('a','c'),'no_hit',.2)],distances),1.)
        self.assertEqual(Stations.signal_weight(('d','b'),[PairObservation(('a','c'),'no_hit',.2)],distances),1.)
        self.assertEqual(Stations.signal_weight(('b','d'),[PairObservation(('a','c'),'incomplete',None)],distances),1.)

    def test_pair_evidence_uses_the_same_state_in_both_radios(self):
        journal=Journal(self.path,{'order':'pair-first'})
        self.addCleanup(journal.close)
        pair=('a','b')
        with journal.connection:
            journal.connection.execute("INSERT INTO pair_sweeps VALUES ('a','b','a','b','no_hit')")
        for state,values in enumerate(((8.,0.),(0.,8.),(3.,4.))):
            journal.start(pair,state,{'trial_id':str(state)})
            journal.finish(journal.pending(),{},dict(zip(RECEIVERS,values)),mark_state=False)
        self.assertEqual(journal.completed_pairs(RECEIVERS),[PairObservation(pair,'no_hit',3.)])

    def test_pair_without_joint_hit_tests_every_state(self):
        self.driver.bad=set(range(4))
        journal=Journal(self.path,{'order':'pair-first'})
        self.addCleanup(journal.close)
        with redirect_stdout(io.StringIO()):
            rows=sweep_pairs(journal,self.stations,CODES,LINKS,RECEIVERS,self.driver,count=4,max_pairs=1)
        self.assertEqual(self.driver.calls,[0,1,2,3])
        self.assertEqual((rows[0]['status'],rows[0]['tested_states'],rows[0]['hits']),('no_hit',4,[]))

    def test_coverage_prioritizes_wide_pairs_and_rejects_near_duplicates_in_either_order(self):
        coords={'a':(0.,0.),'b':(0.,2.),'c':(0.,2.25),'d':(3.,0.),'e':(0.,4.)}
        stations=Stations({'map_id':'a'*64,'points':[
            {'id':key,'x':x,'y':y,'yaw':0.} for key,(x,y) in coords.items()]})
        choices=list(stations.pair_candidates({('b','d')},minimum_spacing=.9,preferred_span=2.4))
        self.assertNotIn(('c','d'),choices)
        self.assertNotIn(('d','c'),choices)
        spans=[math.dist(coords[a],coords[b]) for a,b in choices]
        self.assertGreaterEqual(spans[0],2.4)
        self.assertLess(spans[-1],2.4)

    def test_pair_sweep_extends_only_c5_for_one_underfilled_state(self):
        windows=[]

        def capture(request):
            windows.append((request['state_id'],request['window_s'],request['c5_window_s']))
            if windows[-1]==(0,1.4,.8):
                return {**receipt(request), 'status':'incomplete',
                        'error':'C5 当前状态的 A/B 有效样本不足'}
            return receipt(request,sle=(-70,-60) if request['state_id']==0 else (-62,-68))

        self.driver.capture=capture
        journal=Journal(self.path,{'order':'pair-first'})
        self.addCleanup(journal.close)
        with redirect_stdout(io.StringIO()):
            rows=sweep_pairs(journal,self.stations,CODES,LINKS,RECEIVERS,self.driver,count=2,max_pairs=1,
                             window_s=1.4,c5_window_s=.8,parallel_radios=True)
        self.assertEqual(windows,[(0,1.4,.8),(0,1.4,5.),(1,1.4,.8)])
        self.assertEqual(rows[0]['hits'],[1])

    def test_persistent_missing_frames_do_not_become_a_no_hit_or_stop_other_pairs(self):
        first_pair=None
        def capture(request):
            nonlocal first_pair
            pair=tuple(request['cars'][car]['point_id'] for car in ('A','B'))
            if first_pair is None:
                first_pair=pair
            if pair==first_pair:
                return {**receipt(request),'status':'incomplete','error':'C5 当前状态的 A/B 有效样本不足'}
            return receipt(request,sle=(-60,-68),c5=(-60,-68))

        self.driver.capture=capture
        journal=Journal(self.path,{'order':'pair-first'})
        self.addCleanup(journal.close)
        with redirect_stdout(io.StringIO()):
            rows=sweep_pairs(journal,self.stations,CODES,LINKS,RECEIVERS,self.driver,count=4,max_pairs=2,
                             window_s=1.4,c5_window_s=.8,defer_underfilled=True)
        self.assertEqual([row['status'] for row in rows],['incomplete','hit'])
        self.assertEqual(rows[0]['tested_states'],0)
        self.assertEqual(journal.connection.execute("SELECT COUNT(*) FROM trials WHERE status='invalid'").fetchone()[0],6)

    def test_motion_round_retains_pair_through_interruption_and_queues_one_charge(self):
        journal=Journal(self.path,{'motion_only':True})
        self.addCleanup(journal.close)
        original=self.driver.place

        def interrupted(_):
            raise ConnectionError('link lost')

        self.driver.place=interrupted
        with redirect_stdout(io.StringIO()), self.assertRaises(ConnectionError):
            position_rounds(journal,self.stations,self.driver,start=0,count=3,max_pairs=2,recharge_after=1)
        pending=journal.pending()
        self.driver.place=original
        with redirect_stdout(io.StringIO()):
            position_rounds(journal,self.stations,self.driver,start=0,count=3,max_pairs=2,recharge_after=1)
        rows=journal.report()
        self.assertEqual([r['status'] for r in rows],['arrived']*3)
        self.assertEqual(rows[0]['trial_id'],pending['id'])
        self.assertNotEqual(rows[0]['pair'],rows[1]['pair'])
        self.assertEqual(journal.charging(),['A','B'])
        journal.set_charging([])
        journal.queue_charge_check()
        self.assertEqual(journal.charging(),[])

    def test_resuming_motion_prefers_untried_pairs_to_failed_pairs(self):
        journal=Journal(self.path,{'motion_only':True})
        self.addCleanup(journal.close)
        failed=self.stations.motion_ranked(self.driver.positions(),None,Counter())[0]
        journal.start(failed,0,{'trial_id':uuid.uuid4().hex})
        journal.abandon(journal.pending(),'first pair was blocked')
        with redirect_stdout(io.StringIO()):
            position_rounds(journal,self.stations,self.driver,start=0,count=1,max_pairs=1)
        self.assertNotEqual(tuple(journal.report()[0]['pair']),failed)

    def test_all_50_stations_have_same_orientation(self):
        self.assertEqual(len(self.stations.points),50)
        points=[dict(p) for p in self.stations.points.values()]
        points[3]['yaw']+=.001
        with self.assertRaises(ValueError):
            Stations({'map_id':'a'*64,'points':points})
