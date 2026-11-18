"""No hardware: pending goals must not spin through motion actions."""
import importlib.util
from pathlib import Path
import sys
import time
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'Requires ROS message types')
class RecoveryTests(unittest.TestCase):
    def test_return_blockage_waits_without_exhausting_retry_budget(self):
        from ros_backend import RosBackend
        backend=SimpleNamespace(pose=[1.,2.,0.],zero_motion=lambda:None)
        op={'sequential':False,'kind':'return','arrival_retries':0}
        for _ in range(30):RosBackend.retry_arrival(backend,op,'blocked return')
        self.assertEqual(op['status'],'recovery_wait')
        self.assertEqual(op['arrival_retries'],30)

    def test_late_directed_validation_cannot_restart_canceled_task(self):
        import threading
        from ros_backend import RosBackend
        op={'cancel':True,'status':'validating_leg'}
        backend=SimpleNamespace(lock=threading.RLock(),ops={'test':op})
        RosBackend.directed_validated(backend,None,'test')
        self.assertEqual(op['status'],'canceled')

    def test_invalid_straight_leg_falls_back_to_native_navigation(self):
        import threading
        from ros_backend import RosBackend
        calls=[];op={'cancel':False,'directed_path':'candidate'}
        backend=SimpleNamespace(lock=threading.RLock(),ops={'test':op},send_navigation=lambda identity:calls.append(identity))
        future=SimpleNamespace(result=lambda:SimpleNamespace(is_valid=False))
        RosBackend.directed_validated(backend,future,'test')
        self.assertEqual(calls,['test']);self.assertNotIn('directed_path',op)

    def test_fas_rejected_straight_leg_plans_fresh_before_motion(self):
        import threading
        from ros_backend import RosBackend
        calls=[]
        op={'cancel':False,'directed_path':'candidate','fas_station':True}
        backend=SimpleNamespace(lock=threading.RLock(),ops={'test':op},plan_navigation=calls.append)
        future=SimpleNamespace(result=lambda:SimpleNamespace(is_valid=False))
        RosBackend.directed_validated(backend,future,'test')
        self.assertEqual(calls,['test'])
        self.assertNotIn('directed_path',op)

    def test_blocked_goal_backoff_preserves_pending_task_and_stops(self):
        from ros_backend import RosBackend
        stops=[]
        backend=SimpleNamespace(pose=[1.,2.,0.],zero_motion=lambda:stops.append(True))
        op={'sequential':True,'arrival_retries':0}
        delays=[]
        for _ in range(12):
            RosBackend.retry_arrival(backend,op,'blocked')
            delays.append(op['retry_at']-time.monotonic())
            self.assertEqual(op['status'],'recovery_wait')
        self.assertGreater(delays[-1],9.)
        self.assertLessEqual(delays[-1],10.)
        self.assertEqual(len(stops),12)
        backend.pose=[1.1,2.,0.]
        RosBackend.retry_arrival(backend,op,'retry after movement')
        self.assertEqual(op['retry_streak'],0)

    def test_joint_terminal_keeps_real_position_and_yaw_in_completion_check(self):
        from ros_backend import RosBackend
        calls=[]
        backend=SimpleNamespace(pose=[.08,0.,0.],stationary_since=time.monotonic()-2,
            zero_motion=lambda:None,arrival={'final_position_tolerance':.05},joint_terminal=True,
            retry_arrival=lambda *args:calls.append('replan'),
            begin_waypoint_heading=lambda op:calls.append('terminal'),
            complete_waypoint=lambda *args:calls.append('complete'))
        op={'goal':{'x':0.,'y':0.,'yaw':0.},'yaw_tolerance':.052}
        RosBackend.finish_waypoint_position(backend,op,time.monotonic())
        self.assertEqual(calls,['terminal'])
        backend.pose=[.01,0.,.1]
        RosBackend.finish_waypoint_position(backend,op,time.monotonic())
        self.assertEqual(calls[-1],'terminal')
        backend.pose=[.01,0.,.01]
        RosBackend.finish_waypoint_position(backend,op,time.monotonic())
        self.assertEqual(calls[-1],'complete')

    def test_terminal_releases_translation_when_position_is_in_tolerance(self):
        from ros_backend import RosBackend
        requests=[]
        def configure(request):
            requests.append([parameter.name for parameter in request.parameters])
            return SimpleNamespace(add_done_callback=lambda callback:None)

        backend=SimpleNamespace(pose=[.03,0.,.1],active='point',arrival={'final_position_tolerance':.02},
            joint_terminal=True,follow=SimpleNamespace(server_is_ready=lambda:True),
            controller_parameters=SimpleNamespace(call_async=configure))
        op={'goal':{'x':0.,'y':0.,'yaw':0.},'yaw_tolerance':.0175}
        RosBackend.begin_waypoint_heading(backend,op)
        self.assertEqual(op['heading_controller'],'PointPose')
        self.assertFalse(RosBackend.heading_needs_settle(backend,op))

        backend.pose=[.01,0.,.1]
        self.assertTrue(RosBackend.heading_needs_settle(backend,op))
        RosBackend.begin_waypoint_heading(backend,op)
        self.assertEqual(op['heading_controller'],'PointHeading')
        self.assertEqual(requests,[['PointPose.wz_max'],['PointHeading.max_vel_theta','PointHeading.min_speed_theta',
                                                        'point_heading_checker.yaw_goal_tolerance']])
        self.assertFalse(RosBackend.heading_needs_settle(backend,op))
        backend.pose=[.01,0.,.01]
        self.assertTrue(RosBackend.heading_needs_settle(backend,op))

    def test_stopped_rssi_station_accepts_bounded_drift_after_turn(self):
        import math
        from arrival_policy import RSSI_STATION_POSITION_TOLERANCE, RSSI_STATION_YAW_TOLERANCE
        from ros_backend import RosBackend
        calls=[]
        backend=SimpleNamespace(pose=[.113,0.,math.radians(1.4)],stationary_since=time.monotonic()-2,
            zero_motion=lambda:None,arrival={'final_position_tolerance':.04},joint_terminal=True,
            retry_arrival=lambda *args:calls.append('replan'),
            begin_waypoint_heading=lambda op:calls.append('heading'),
            complete_waypoint=lambda *args:calls.append('complete'))
        op={'goal':{'x':0.,'y':0.,'yaw':0.},'yaw_tolerance':RSSI_STATION_YAW_TOLERANCE,
            'stop_xy_tolerance':RSSI_STATION_POSITION_TOLERANCE}
        RosBackend.finish_waypoint_position(backend,op,time.monotonic())
        self.assertEqual(calls,['complete'])
        backend.pose=[.113,0.,math.radians(3.5)]
        RosBackend.finish_waypoint_position(backend,op,time.monotonic())
        self.assertEqual(calls[-1],'heading')
        backend.pose=[.121,0.,0.]
        RosBackend.finish_waypoint_position(backend,op,time.monotonic())
        self.assertEqual(calls[-1],'replan')
