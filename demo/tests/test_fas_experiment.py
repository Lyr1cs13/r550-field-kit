from contextlib import redirect_stdout
import io
import json
import math
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from unittest import TestCase
import uuid

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from directed_route import uniform_route
from fas_car import CorridorLease, WS63_IDS
from fas_experiment import (C5Sample, Journal, PROTOCOL, Stations, aggregate, capture_request,
                            evaluate, move_batches, phase_state, search, station_ids)


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

    def test_head_on_swap_is_never_scheduled(self):
        starts = {'A':(0.,0.),'B':(3.,0.)}
        ends = {'A':starts['B'],'B':starts['A']}
        paths={car:[starts[car],ends[car]] for car in starts}
        self.assertIsNone(move_batches(starts,ends,paths))

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
        now[0]=1.1
        self.assertFalse(lease.allows('move',h))
        with self.assertRaises(ValueError):
            lease.renew(key)


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

    def test_all_50_stations_have_same_orientation(self):
        self.assertEqual(len(self.stations.points),50)
        points=[dict(p) for p in self.stations.points.values()]
        points[3]['yaw']+=.001
        with self.assertRaises(ValueError):
            Stations({'map_id':'a'*64,'points':points})
