"""Exercise the real bridge update with fake publishers; never creates a ROS node."""
import importlib.util
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


@unittest.skipUnless(importlib.util.find_spec('rclpy'),'Requires installed ROS message types')
class BridgeContactTests(unittest.TestCase):

    def test_terminal_acceptance_uses_stopped_pose_and_preserves_strict_boundary(self):
        import math
        from ros_backend import RosBackend
        for distance,angle,accepted in ((.045,.02,True),(.051,.02,False),(.02,.06,False)):
            s,ir,motion=self.fixture();now=time.monotonic();stamp=time.time();cancels=[]
            s.docking_since=None;s.stationary_since=now-1;s.joint_terminal=True;s.precision_arrival=True
            s.arrival={'final_position_tolerance':.05};s.active='point'
            op={'status':'heading','started':now,'timeout':30.,'goal':{'x':0.,'y':0.,'yaw':0.},
                'yaw_tolerance':.05235987756,'handle':SimpleNamespace(cancel_goal_async=lambda:cancels.append(True))}
            s.ops={'point':op}
            transform=SimpleNamespace(translation=SimpleNamespace(x=distance,y=0.),
                rotation=SimpleNamespace(x=0.,y=0.,z=math.sin(angle/2),w=math.cos(angle/2)))
            message=SimpleNamespace(transform=transform,header=SimpleNamespace(
                stamp=SimpleNamespace(sec=int(stamp),nanosec=int((stamp%1)*1e9))))
            s.tf.lookup_transform=lambda *args:message
            RosBackend.update(s)
            self.assertEqual(bool(cancels),accepted)
            self.assertEqual(op['status'],'canceling' if accepted else 'heading')
            if accepted:self.assertEqual(op['settle_after_cancel'],'heading_settling')

    def fixture(self):
        from ros_backend import RosBackend
        from charge_confirmation import ChargeConfirmation,NativeDockingWatchdog
        from geometry_msgs.msg import Twist
        now=time.monotonic();ir=[];motion=[]
        def missing(*args):raise RuntimeError('No transform needed in this test')
        s=SimpleNamespace(lock=threading.RLock(),update_laser=lambda:None,
            charging=False,last_charge=now,current=.18,voltage=11.8,
            last_numeric={'current':now,'voltage':now},last_wheel=now,stationary_since=None,
            charge_confirmation=ChargeConfirmation(),native_docking_watchdog=NativeDockingWatchdog(),
            last_scan=now,last_ir=now,ir_count=3,pose=None,dock_pose=[0.,0.,0.],wheel_distance=0.,
            native_docking_stage="native_capture",charge_motion_since=None,
            tf=SimpleNamespace(lookup_transform=missing),initial_request=None,active=None,
            docking_since=now-1,docking_ir_lost_since=None,
            ir=SimpleNamespace(publish=lambda m:ir.append(m.data)),
            cmd=SimpleNamespace(publish=lambda m:motion.append((m.linear.x,m.angular.z))),
            update_dock_localization=lambda now:None)
        s.native_docking_watchdog.reset(now,0.)
        s.update_oem_docking=lambda stamp,current:RosBackend.update_oem_docking(s,stamp,current)
        s.zero_motion=lambda:s.cmd.publish(Twist())
        s.set_oem_ir=lambda enabled,now:RosBackend.set_oem_ir(s,enabled,now)
        return s,ir,motion





    def test_oem_enable_is_not_restarted_on_every_update(self):
        from ros_backend import RosBackend
        s,ir,motion=self.fixture()
        RosBackend.set_oem_ir(s,True,10.)
        RosBackend.set_oem_ir(s,True,10.05)
        RosBackend.set_oem_ir(s,True,10.15)
        RosBackend.set_oem_ir(s,True,11.)
        self.assertEqual(ir,[1]*10);self.assertEqual(len(motion),2)
        RosBackend.set_oem_ir(s,False,11.1);self.assertEqual(ir[-1],0)


    def test_native_capture_watchdog_stops_motion_without_declaring_success(self):
        from ros_backend import RosBackend
        from charge_confirmation import NativeDockingWatchdog
        from std_msgs.msg import Int8
        s,ir,motion=self.fixture();now=time.monotonic()
        s.current=0.;s.pose=[0.,0.,0.];s.dock_pose=[0.,0.,0.]
        s.last_scan=now;s.last_ir=now;s.ir_count=3;s.wheel_distance=0.
        s.native_docking_watchdog=NativeDockingWatchdog()
        s.native_docking_watchdog.reset(now-21,0.)
        s.native_docking_watchdog.update(now-21,travel=0.,near=True,fresh=True)
        def stop():
            s.docking_since=None;s.ir.publish(Int8(data=0));s.zero_motion()
        s.stop=stop;RosBackend.update(s)
        self.assertIsNone(s.docking_since);self.assertTrue(s.docking_error)
        self.assertEqual(ir,[0]);self.assertEqual(motion,[(0.,0.)])
        self.assertFalse(s.charging_stable)


    def test_confirmation_finishes_only_after_stopping(self):
        from ros_backend import RosBackend
        s,ir,motion=self.fixture();now=time.monotonic()
        s.stationary_since=now-1;s.charge_confirmation.since=now-3
        RosBackend.update(s)
        self.assertTrue(s.charging_stable);self.assertIsNone(s.docking_since)
        self.assertEqual(ir,[0]);self.assertEqual(motion,[(0.,0.)])

    def test_first_current_does_not_cut_off_native_state_machine(self):
        from ros_backend import RosBackend
        s,ir,motion=self.fixture();RosBackend.update(s)
        self.assertFalse(s.charging_stable);self.assertEqual(ir,[1]*10)
        self.assertEqual(s.native_docking_stage,'charging_observed')
        s.current=0.;RosBackend.update(s)
        self.assertEqual(ir,[1]*10)  # Contact bounce never sends A0 or restarts A1.

    def test_persistent_charged_wheel_motion_is_bounded(self):
        from ros_backend import RosBackend
        s,ir,motion=self.fixture();now=time.monotonic();s.charge_motion_since=now-3
        s.stop=lambda:(setattr(s,'docking_since',None),s.set_oem_ir(False,now),s.zero_motion())
        RosBackend.update(s)
        self.assertIsNone(s.docking_since);self.assertTrue(s.docking_error);self.assertEqual(ir,[0])

    def test_near_dock_raw_contact_flag_does_not_start_custom_reseat(self):
        from ros_backend import RosBackend
        s,ir,motion=self.fixture();s.dock_localization=SimpleNamespace(arm=lambda:None)
        s.charging_contact=True;RosBackend.begin_docking(s)
        self.assertEqual(s.native_docking_stage,'native_capture')
        self.assertEqual(ir,[])
