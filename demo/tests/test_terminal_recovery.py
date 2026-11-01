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
