from pathlib import Path
import importlib.util
import sys
import threading
from types import SimpleNamespace
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from fleet_panel import Fleet, pair_status
from fleet_zone import FleetZone


class FleetTests(unittest.TestCase):
    def test_onboard_check_rejects_stale_pose_and_escape_route(self):
        zone = FleetZone('a','map',(0.,3.,0.,3.))
        pose = [1.,1.,0.]
        covariance = [.001,.001,.001]
        self.assertEqual(zone.motion_reason(pose,.1,covariance,.9,[[1.5,1.5]]),'')
        self.assertTrue(zone.motion_reason(pose,.41,covariance,.9,[]))
        self.assertTrue(zone.motion_reason([.15,1.,0.],.1,covariance,.9,[]))
        self.assertTrue(zone.motion_reason(pose,.1,covariance,.9,[[3.2,1.]]))

    def test_simultaneous_motion_requires_two_disjoint_verified_zones(self):
        def state(identity,bounds):
            return {'vehicle_connected':True,'map':{'map_id':'same'},
                    'health':{'ready':True,'localization_confirmed':True,
                              'fleet':{'robot_id':identity,'bounds':bounds,'motion_reason':''}}}

        a = state('a',[0.,2.,0.,2.])
        b = state('b',[2.5,4.5,0.,2.])
        self.assertEqual(pair_status([a,b]),'')
        self.assertTrue(pair_status([a,{**b,'vehicle_connected':False}]))
        self.assertTrue(pair_status([a,{**b,'map':{'map_id':'other'}}]))
        self.assertTrue(pair_status([a,state('a',[2.5,4.5,0.,2.])]))
        self.assertTrue(pair_status([a,state('b',[2.1,4.1,0.,2.])]))

    def test_solo_motion_works_with_peer_offline_or_stopped(self):
        fleet=Fleet()
        a=SimpleNamespace(last_state=None,motion_unknown=False)
        b=SimpleNamespace(last_state=None,motion_unknown=False)
        fleet.panels=[a,b]
        stopped={'vehicle_connected':True,'phase':'idle','health':{'stationary':True}}
        offline={'vehicle_connected':False,'phase':'disconnected','health':{}}
        active={**stopped,'phase':'navigating'}
        self.assertEqual(fleet.drive_reason(0,[stopped,offline]),'')
        self.assertEqual(fleet.drive_reason(1,[offline,stopped]),'')
        self.assertEqual(fleet.drive_reason(0,[stopped,stopped]),'')
        self.assertTrue(fleet.drive_reason(0,[stopped,active]))
        b.last_state=active
        self.assertTrue(fleet.drive_reason(0,[stopped,offline]))
        b.last_state={**active,'health':{'fleet':{'active':True}}}
        self.assertEqual(fleet.drive_reason(0,[stopped,offline]),'')

    def test_pair_holds_until_both_vehicles_are_armed_and_leased(self):
        def state(identity, bounds, active, leased):
            return {'vehicle_connected':True,'phase':'idle','map':{'map_id':'same'},
                    'health':{'ready':True,'stationary':True,'localization_confirmed':True,
                              'fleet':{'robot_id':identity,'bounds':bounds,'active':active,
                                       'leased':leased,'motion_reason':''}}}
        fleet=Fleet()
        fleet.paired=True
        states=[state('a',[0.,2.,0.,2.],True,True),state('b',[2.5,4.5,0.,2.],False,False)]
        self.assertEqual(fleet.pair_start_reason(states),'')
        self.assertTrue(fleet.drive_reason(0,states))
        states[1]['health']['fleet'].update(active=True,leased=True)
        self.assertEqual(fleet.drive_reason(0,states),'')
        self.assertEqual(fleet.drive_reason(1,states),'')

    @unittest.skipUnless(importlib.util.find_spec('rclpy'), 'Requires ROS message types')
    def test_onboard_velocity_permission_expires_with_fleet_lease(self):
        from ros_backend import RosBackend
        commands=[]
        health={'ready':True,'fleet':{'leased':True,'motion_reason':''}}
        backend=SimpleNamespace(lock=threading.RLock(),active='task',ops={'task':{'status':'navigating'}},
            snapshot=lambda:health,confirmed=True,localization_guard=SimpleNamespace(reason=''),
            fleet_zone=True,motion_permit=SimpleNamespace(publish=lambda msg:commands.append(msg.data)),
            publish_spin_limit=lambda limit:None,docking_since=None,oem_lease=SimpleNamespace(publish=lambda msg:None),
            retreat_permit=SimpleNamespace(publish=lambda msg:None))
        RosBackend.publish_motion_permit(backend)
        health['fleet']['leased']=False
        RosBackend.publish_motion_permit(backend)
        self.assertEqual(commands,[True,False])

    @unittest.skipUnless(importlib.util.find_spec('rclpy'), 'Requires ROS message types')
    def test_profiled_spin_obeys_peer_reservation_and_imu_freshness(self):
        import time
        from ros_backend import RosBackend
        caps=[];permits=[];allowed=[True]
        sink=SimpleNamespace(publish=lambda msg:None)
        op=dict(status='spinning',kind='waypoint',spin_brake=SimpleNamespace(signed_limit=-.14))
        backend=SimpleNamespace(lock=threading.RLock(),active='point',ops={'point':op},path=[[0.,0.]],
            snapshot=lambda:dict(ready=True),confirmed=True,localization_guard=SimpleNamespace(reason=''),
            fleet_zone=None,fas_lease=SimpleNamespace(owner='pair',allows=lambda *args:allowed[0]),
            docking_since=None,retreat_permit=sink,oem_lease=sink,gyro=(time.monotonic(),0.,0.),
            motion_permit=SimpleNamespace(publish=lambda msg:permits.append(msg.data)),publish_spin_limit=caps.append)
        RosBackend.publish_motion_permit(backend)
        self.assertEqual((permits,caps),([],[-.14]))
        allowed[0]=False
        RosBackend.publish_motion_permit(backend)
        self.assertEqual((permits[-1],caps[-1]),(False,0.))
        allowed[0]=True;backend.gyro=(time.monotonic()-.3,0.,0.)
        RosBackend.publish_motion_permit(backend)
        self.assertEqual((permits[-1],caps[-1]),(False,0.))
