from contextlib import redirect_stdout
from collections import Counter
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import patch
import uuid

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from directed_route import uniform_route
from fas_car import CorridorLease, WS63_IDS, obstruction_clear
from fas_experiment import (C5Sample, Journal, PROTOCOL, RecoveryWait, ScanEvidence, Stations, aggregate,
                            capture_request, evaluate, move_batches, parallel_bounds, phase_state, position_rounds,
                            scan_confirmed, search, station_ids, stopped_station_arrival)
from guarded_retreat import slip_obstruction


LINKS = station_ids({'sle':WS63_IDS,'c5':{'A':'02:00:00:00:01:AA','B':'0200000001bb'}})
RECEIVERS = ('sle/main','c5/main')
CODES = list(range(4096))


def receipt(request, sle=(-55.,-61.), c5=(-64.,-70.), n=30):
    output = dict(request,status='complete',raw_ref='station/run/record')
    output['data'] = {key:{'epoch':8,'rssi_dbm':dict(zip(('A','B'),values)),'samples':{'A':n,'B':n}}
                      for key,values in zip(RECEIVERS,(sle,c5))}
    return output


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
        self.assertFalse(stopped_station_arrival({**health,'arrival':{**arrival,'yaw_error_deg':3.5}}))

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
