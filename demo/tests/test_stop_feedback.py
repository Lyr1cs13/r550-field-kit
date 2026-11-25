"""Output-owner regression checks. Publishers below only append to Python lists."""
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'Requires ROS message types')
class StopFeedbackTests(unittest.TestCase):
    def test_wheel_messages_cannot_refresh_brake_evidence_by_repeating_an_old_stamp(self):
        from motion_gate import MotionGate
        from nav_msgs.msg import Odometry
        clock=SimpleNamespace(nanoseconds=10_000_000_000)
        gate=SimpleNamespace(wheel_stamp_ns=None,last_retreat_odom=-math.inf,retreat_odom=None,
                             get_clock=lambda:SimpleNamespace(now=lambda:clock))
        wheel=Odometry()
        wheel.header.stamp.sec=10
        with patch('motion_gate.time.monotonic',side_effect=[1.,1.10,1.18,1.60,1.65]):
            MotionGate.odom(gate,wheel)
            for stamp,expected in ((10_100_000_000,1.),(10_180_000_000,1.),(10_600_000_000,1.)):
                clock.nanoseconds=stamp
                MotionGate.odom(gate,wheel)
                self.assertEqual(gate.last_retreat_odom,expected)
            wheel.header.stamp.nanosec=640_000_000
            clock.nanoseconds=10_650_000_000
            MotionGate.odom(gate,wheel)
            self.assertAlmostEqual(gate.last_retreat_odom,1.64)

    def test_blocked_reference_replans_the_same_goal_with_native_navigation(self):
        from corridor_ros import CorridorBackend
        from ros_backend import RosBackend
        backend=CorridorBackend.__new__(CorridorBackend)
        target={'x':.25,'y':-.6,'yaw':1.57}
        operation={'goal':target,'kind':'waypoint','corridor':True}
        backend.corridors=SimpleNamespace(includes=lambda _point: True)
        backend.ops={'move':operation}
        backend.retry_arrival=Mock()
        backend.path=[(0.,0.)]
        backend.pose=[0.,0.,0.]
        backend.corridor_retry(operation,'blocked')
        self.assertIs(operation['goal'],target)
        self.assertFalse(backend.guided(target,'move'))
        self.assertTrue(backend.guided({**target,'x':.5},'move'))
        with patch.object(RosBackend,'plan_navigation',autospec=True) as native:
            backend.plan_navigation('move')
        native.assert_called_once_with(backend,'move')

    def test_return_on_a_reference_aisle_uses_native_planner(self):
        from corridor_ros import CorridorBackend
        from ros_backend import RosBackend
        backend=CorridorBackend.__new__(CorridorBackend)
        goal={'x':.5,'y':-4.5,'yaw':0.}
        backend.corridors=SimpleNamespace(includes=lambda point: point==goal)
        backend.ops={'return':{'kind':'return','goal':goal}}
        self.assertTrue(backend.guided(goal,'waypoint'))
        with patch.object(RosBackend,'plan_navigation',autospec=True) as native:
            backend.plan_navigation('return')
        native.assert_called_once_with(backend,'return')
        self.assertFalse(backend.ops['return'].get('corridor'))

    def test_delayed_tick_keeps_distinct_scan_time_evidence(self):
        from ros_backend import RosBackend
        from fas_experiment import ScanEvidence
        from sensor_msgs.msg import LaserScan
        import threading
        scans=[]
        for offset in (660_000_000,740_000_000,820_000_000,900_000_000):
            scan=LaserScan()
            scan.header.stamp.sec=999
            scan.header.stamp.nanosec=offset
            scan.header.frame_id='laser'
            scan.range_max=12.
            scan.ranges=[2.] * 24
            scans.append(scan)
        transform=SimpleNamespace(translation=SimpleNamespace(x=0.,y=0.),
                                  rotation=SimpleNamespace(w=1.,x=0.,y=0.,z=0.))
        frames=ScanEvidence()
        bridge=SimpleNamespace(lock=threading.RLock(),pending_scans=scans,fas_lease=True,
            fas_scan_stamp=None,fas_scans=frames,last_scan=1000.,
            tf=SimpleNamespace(lookup_transform=lambda *_:SimpleNamespace(transform=transform)),
            scan_wall_agreement=SimpleNamespace(score=lambda points:(len(points),1.)),
            map_matches=True)
        with patch('ros_backend.time.time',return_value=1000.), patch('ros_backend.time.monotonic',return_value=1000.):
            RosBackend.update_laser(bridge)
            self.assertEqual(len(frames.frames),3)
            self.assertEqual(frames.latest(1000.),(1.,False))
            RosBackend.update_laser(bridge)
            self.assertEqual(len(frames.frames),3)

    def test_cancel_keeps_braking_and_latches_wheels_that_keep_turning(self):
        from motion_gate import MotionGate
        from motion_health import NativeStopFeedback, OemLease, StopFeedback
        with tempfile.TemporaryDirectory() as directory:
            hold = Path(directory) / 'motion-hold'
            gate = SimpleNamespace(alive=Mock(), output=Mock(), ir_disable=Mock(), retreat_status=Mock(),
                lease=Mock(), oem=OemLease(), disable_burst=0, retreat_identity='',
                retreat_lease=-math.inf, last_scan=0., last_tick=0., last_motion=0.,
                retreat=SimpleNamespace(status='idle'), last_linear=0., hold_file=hold, departure_order=None,
                stop_feedback=StopFeedback(), native_stop=NativeStopFeedback(), get_logger=Mock())
            gate.lease.output.return_value=(0.,0.,0.)
            for i in range(25):
                now=1.+i*.05
                gate.last_retreat_odom=now
                gate.retreat_odom=(0.,0.,0.,.10,0.)
                with patch('motion_gate.time.monotonic',return_value=now):
                    MotionGate.tick(gate)
                self.assertEqual(hold.exists(),now-1.>=.65)
            self.assertEqual(gate.output.publish.call_count,25)
            self.assertTrue(all(c.args[0].linear.x==0 for c in gate.output.publish.call_args_list))
            gate.oem.receive(2.3,True)
            gate.lease.output.return_value=(.1,0.,0.)
            with patch('motion_gate.time.monotonic',return_value=2.3):
                MotionGate.tick(gate)
            self.assertEqual(gate.output.publish.call_args.args[0].linear.x,0.)
            self.assertEqual(gate.ir_disable.publish.call_args.args[0].data,0)

    def test_brake_review_blocks_new_drive_and_requires_fresh_permission_after_rest(self):
        from motion_gate import MotionGate
        from motion_health import NativeStopFeedback, OemLease, StopFeedback, VelocityLease
        with tempfile.TemporaryDirectory() as directory:
            gate=SimpleNamespace(alive=Mock(),output=Mock(),ir_disable=Mock(),retreat_status=Mock(),
                lease=VelocityLease(),oem=OemLease(),disable_burst=0,retreat_identity='',
                retreat_lease=-math.inf,last_scan=0.,last_tick=0.,last_motion=0.,
                retreat=SimpleNamespace(status='idle'),last_linear=.045,hold_file=Path(directory)/'hold',
                departure_order=None,stop_feedback=StopFeedback(),native_stop=NativeStopFeedback(),
                get_logger=Mock(),scan_in_base=lambda:[(1.,0.)]*12,dock_motion_clear=lambda _:True,
                turn_clear_for_braking=lambda _:True,travel_clear_for_braking=lambda *_:True,dock_departure=-math.inf)
            gate.stop_feedback.still_moving(0.,(.045,0.,.08),(0.,0.,0.,.038,.068))
            for i in range(1,25):
                t=i*.05
                gate.last_scan=gate.last_retreat_odom=t
                moving=t<.85
                gate.retreat_odom=(.038*min(t,.85),0.,.068*min(t,.85),.038 if moving else 0.,.068 if moving else 0.)
                if i==16:
                    self.assertTrue(gate.stop_feedback.reviewing)
                    gate.lease.receive_command(t,(.045,0.,0.))
                    gate.lease.receive_permit(t,True)
                with patch('motion_gate.time.monotonic',return_value=t):
                    MotionGate.tick(gate)
                self.assertFalse(gate.hold_file.exists())
                self.assertEqual(gate.output.publish.call_args.args[0].linear.x,0.)
            self.assertFalse(gate.stop_feedback.reviewing)
            self.assertFalse(gate.lease.permitted)

    def test_dock_contact_inhibits_navigation_without_persistent_lock(self):
        from ros_backend import RosBackend
        import threading
        for kind, status in [('waypoint','navigating'), ('survey','heading'), ('retreat','retreating')]:
            for pose, contact in [([4.,0.,0.],True), ([.1,.05,0.],False), ([1.,0.,0.],False)]:
                with self.subTest(kind=kind,pose=pose), tempfile.TemporaryDirectory() as folder:
                    unsafe = contact
                    bridge = SimpleNamespace(lock=threading.RLock(), active='task',
                        ops={'task':dict(kind=kind,status=status)}, pose=pose,
                        dock_calibrated=True, dock_pose=[0.,0.,0.], docking_since=None,
                        snapshot=lambda:dict(ready=True,charge_contact_live=contact),
                        confirmed=True, localization_guard=SimpleNamespace(reason=''),
                        fleet_zone=None, get_logger=Mock(), motion_permit=Mock(),
                        publish_spin_limit=Mock(), retreat_permit=Mock(), oem_lease=Mock())
                    with patch('ros_backend.__file__',str(Path(folder)/'demo/ros_backend.py')):
                        RosBackend.publish_motion_permit(bridge)
                    self.assertFalse((Path(folder)/'state/motion-hold').exists())
                    self.assertEqual(bridge.motion_permit.publish.call_args.args[0].data,
                                     not unsafe and kind!='retreat')
                    self.assertEqual(bridge.retreat_permit.publish.call_args.args[0].data,
                                     'task' if not unsafe and kind=='retreat' else '')

    def test_oem_coast_stays_at_zero_without_latching_after_verified_stop(self):
        from motion_gate import MotionGate
        from motion_health import NativeStopFeedback, OemLease, StopFeedback
        with tempfile.TemporaryDirectory() as directory:
            hold=Path(directory)/'motion-hold'
            gate=SimpleNamespace(alive=Mock(),output=Mock(),ir_disable=Mock(),retreat_status=Mock(),
                lease=Mock(),oem=OemLease(),disable_burst=0,retreat_identity='',retreat_lease=-math.inf,
                last_scan=0.,last_tick=0.,last_motion=0.,retreat=SimpleNamespace(status='idle'),
                last_linear=0.,hold_file=hold,departure_order=None,stop_feedback=StopFeedback(),
                native_stop=NativeStopFeedback(),get_logger=Mock(),scan_in_base=lambda:[(1.,0.)]*12,
                dock_motion_clear=lambda _:True,dock_departure=-math.inf)
            gate.lease.output.return_value=(.05,0.,0.)
            gate.oem.receive(0.,True)
            with patch('motion_gate.time.monotonic',return_value=.01):
                MotionGate.oem_state(gate,SimpleNamespace(data=False))
            for i in range(22):
                now=(i+1)*.05
                gate.last_retreat_odom=gate.last_scan=now
                gate.retreat_odom=(-.047*min(now,.8),0.,0.,-.047 if now<.8 else 0.,0.)
                with patch('motion_gate.time.monotonic',return_value=now):
                    MotionGate.tick(gate)
                if not gate.native_stop.active:
                    break
            self.assertFalse(gate.native_stop.active)
            self.assertFalse(hold.exists())
            self.assertTrue(all(c.args[0].linear.x==0. for c in gate.output.publish.call_args_list))

    def test_explicit_oem_release_starts_only_one_stop_burst(self):
        from motion_gate import MotionGate, OEM_STOP_CYCLES
        from motion_health import NativeStopFeedback, OemLease
        gate=SimpleNamespace(oem=OemLease(),last_motion=0.,disable_burst=0,native_stop=NativeStopFeedback())
        MotionGate.oem_state(gate,SimpleNamespace(data=True))
        self.assertEqual(gate.disable_burst,0)
        MotionGate.oem_state(gate,SimpleNamespace(data=False))
        self.assertEqual(gate.disable_burst,OEM_STOP_CYCLES)
        self.assertTrue(gate.native_stop.active)
        gate.disable_burst-=1
        MotionGate.oem_state(gate,SimpleNamespace(data=False))
        self.assertEqual(gate.disable_burst,OEM_STOP_CYCLES-1)

    def test_native_mode_without_a_lease_restarts_ir_stop_burst(self):
        from motion_gate import MotionGate, OEM_STOP_CYCLES
        from motion_health import NativeStopFeedback, OemLease
        gate=SimpleNamespace(oem=OemLease(),disable_burst=0,native_stop=NativeStopFeedback())
        MotionGate.firmware_oem_state(gate,SimpleNamespace(data=True))
        self.assertEqual(gate.disable_burst,OEM_STOP_CYCLES)
        gate.oem.receive(1.,True)
        gate.disable_burst=0
        MotionGate.firmware_oem_state(gate,SimpleNamespace(data=True))
        self.assertEqual(gate.disable_burst,0)

    def test_retreat_preview_rejects_an_invisible_dock_ahead_of_reverse_path(self):
        from ros_backend import RosBackend
        import threading
        import time
        bridge = SimpleNamespace(lock=threading.RLock(), fas_obstruction=None, fleet_zone=None,
            confirmed=True, snapshot=lambda:dict(ready=True,stationary=True,
                charge_contact_live=False,voltage=12.), charging_stable=False, docking_since=None,
            scan_agreement=.95, last_retreat_status=time.monotonic(), active=None,
            pose=[.20,0.,0.], grid=Mock(), dock_calibrated=True, dock_pose=[0.,0.,0.])
        with self.assertRaisesRegex(ValueError,'碰到充电桩'):
            RosBackend.retreat_path(bridge)
        bridge.pose=[.32,0.,0.]
        self.assertAlmostEqual(RosBackend.retreat_path(bridge)[-1][0],.12)
        bridge.pose=[1.,0.,0.]
        self.assertAlmostEqual(RosBackend.retreat_path(bridge)[-1][0],.8)
