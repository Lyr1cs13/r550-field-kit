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
    def test_large_terminal_turn_spends_at_most_one_second_on_xy_margin(self):
        from ros_backend import RosBackend
        canceled=[]
        backend=SimpleNamespace(arrival={},zero_motion=lambda:None)
        op=dict(sequential=True,kind='waypoint',controller_id='FixedHeading',fas_station=True,
                xy_tolerance=.04,stop_xy_tolerance=.055,turn_approach_radius=.022,
                handle=SimpleNamespace(cancel_goal_async=lambda:canceled.append(True)))
        stop=lambda distance,now:RosBackend.stop_at_waypoint(backend,op,distance,now)
        self.assertFalse(stop(.039,10.))
        self.assertFalse(stop(.042,10.4))
        self.assertFalse(stop(.033,11.))
        self.assertFalse(stop(.031,11.9))
        self.assertTrue(stop(.031,12.))
        self.assertEqual(canceled,[True])
        self.assertEqual(op['settle_after_cancel'],'position_settling')
        self.assertTrue(stop(.021,13.))

    def test_rejected_spin_racing_a_timeout_drops_its_old_gyro_limit(self):
        import threading
        from concurrent.futures import Future
        from ros_backend import RosBackend
        op=dict(cancel=True,spin_resume='heading_settling',settle_after_cancel='heading_settling',
                spin_yaw=.12,spin_brake=object(),spin_anchor=(0.,0.))
        backend=SimpleNamespace(lock=threading.Lock(),ops={'a':op},zero_motion=lambda:None,
                                finish_native_cancellation=lambda op:RosBackend.finish_native_cancellation(None,op))
        future=Future()
        future.set_result(SimpleNamespace(accepted=False))
        RosBackend.spin_accepted(backend,future,'a')
        self.assertEqual(op['status'],'heading_settling')
        self.assertFalse(op['cancel'])
        for key in ('spin_brake','spin_anchor','spin_yaw','spin_resume'):
            self.assertNotIn(key,op)

    def test_fas_keeps_heading_after_three_attempts_and_reopens_xy_on_measured_drift(self):
        import math
        from ros_backend import RosBackend
        calls=[]
        wall=time.time()
        clock=time.monotonic()
        backend=SimpleNamespace(pose=[.052,0.,.12],stationary_since=clock-1.,confirmed=True,
            localization_guard=SimpleNamespace(reason=''),arrival={},joint_terminal=True,zero_motion=lambda:None,
            request_stationary_amcl=lambda now:None,retry_arrival=lambda *args:calls.append('xy'),
            begin_waypoint_heading=lambda op:calls.append('yaw'),complete_waypoint=lambda *a:calls.append('complete'))
        op=dict(goal=dict(x=0.,y=0.,yaw=0.),fas_station=True,stop_xy_tolerance=.055,yaw_tolerance=math.radians(3),
                heading_attempts=3,settle_corrections=3)
        def observe(i):
            backend.pose_stamp=wall-.35+i*.07
            backend.last_wheel=backend.last_scan=clock+i*.1
            RosBackend.finish_waypoint_position(backend,op,clock+i*.1)
        for i in range(5):
            observe(i)
        self.assertEqual(calls,['yaw'])
        backend.pose[0]=.062
        wall=time.time()-.35
        for i in range(5,10):
            observe(i)
        self.assertEqual(calls,['yaw','xy'])

    def test_fitted_pose_waits_for_new_amcl_and_sustained_scan_before_resume(self):
        from concurrent.futures import Future
        from ros_backend import RosBackend
        fit=[1.,2.,.3]
        future=Future()
        future.set_result(fit)
        h={'ready':True,'pose_age_s':.02,'scan_map_agreement':.91,
           'localization_covariance':(.002,.002,.003)}
        applied=[]
        backend=SimpleNamespace(fas_localization={'owner':'run','wheels':fit,'deadline':30.,'future':future,
                                                'good_since':None},
            fas_still=lambda *args:True,encoders=fit,confirmed=False,pose=fit,last_amcl=0.,last_scan=10.,last_wheel=10.,
            initial_request=None,fas_scan_stamp=None,fas_scans=SimpleNamespace(frames=[]),snapshot=lambda:h,
            request_stationary_amcl=lambda now:None,
            initialize_pose=lambda pose,**kw:applied.append(pose),
            get_logger=lambda:SimpleNamespace(info=lambda msg:None))
        backend.cancel_fas_localization=lambda:RosBackend.cancel_fas_localization(backend)
        RosBackend.update_fas_localization(backend,10.)
        self.assertEqual(applied,[fit])
        backend.fas_scan_stamp=backend.fas_localization['scan_after']+.1
        backend.last_scan=backend.last_wheel=11.
        RosBackend.update_fas_localization(backend,11.)
        self.assertFalse(backend.confirmed)
        backend.last_amcl=11.
        backend.last_scan=backend.last_wheel=12.
        RosBackend.update_fas_localization(backend,12.)
        self.assertFalse(backend.confirmed)
        backend.last_scan=backend.last_wheel=12.4
        backend.fas_scan_stamp+=.1
        RosBackend.update_fas_localization(backend,12.4)
        self.assertFalse(backend.confirmed)
        backend.last_scan=backend.last_wheel=12.8
        backend.fas_scan_stamp+=.1
        RosBackend.update_fas_localization(backend,12.8)
        self.assertTrue(backend.confirmed)
        self.assertIsNone(backend.fas_localization)

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
        backend.replan_fas_line=lambda entry,identity:RosBackend.replan_fas_line(backend,entry,identity)
        future=SimpleNamespace(result=lambda:SimpleNamespace(is_valid=False))
        RosBackend.directed_validated(backend,future,'test')
        self.assertEqual(calls,['test'])
        self.assertTrue(op['direct_failed'])
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

    def test_fas_first_heading_uses_native_spin_and_keeps_stop_verification(self):
        import math
        import threading
        from ros_backend import RosBackend
        issued=[]

        def send(goal):
            issued.append(goal)
            return SimpleNamespace(add_done_callback=lambda callback:None)

        op={'goal':{'x':0.,'y':0.,'yaw':-.7},'yaw_tolerance':math.radians(3),'heading_attempts':0,'cancel':False}
        backend=SimpleNamespace(pose=[.02,0.,.15],active='point',joint_terminal=True,
            arrival={'final_position_tolerance':.055},ops={'point':op},lock=threading.RLock(),
            spin_action=SimpleNamespace(server_is_ready=lambda:True,send_goal_async=send),zero_motion=lambda:None)
        backend.start_fas_spin=lambda entry,error,resume:RosBackend.start_fas_spin(backend,entry,error,resume)
        backend.finish_native_cancellation=lambda entry:RosBackend.finish_native_cancellation(backend,entry)
        op['fas_station']=True
        RosBackend.begin_waypoint_heading(backend,op)
        self.assertAlmostEqual(issued[0].target_yaw,-.85)
        self.assertEqual(backend.path,[[.02,0.]])
        self.assertEqual(op['heading_controller'],'Spin')
        op.update(cancel=True,settle_after_cancel='heading_settling')
        RosBackend.spin_done(backend,SimpleNamespace(result=lambda:SimpleNamespace(status=5)),'point')
        self.assertEqual(op['status'],'heading_settling')
        self.assertFalse(op['cancel'])

    def test_short_approach_revalidates_from_the_stopped_pose_after_spin(self):
        import math
        from ros_backend import RosBackend
        calls=[]
        op={'status':'line_alignment','line_yaw':0.}
        backend=SimpleNamespace(pose=[0.,0.,-math.pi/2],stationary_since=2.,active='point',zero_motion=lambda:None,
            send_fas_line=lambda identity:calls.append((identity,list(backend.pose))) or True,
            start_fas_spin=lambda entry,error,resume:calls.append((error,resume)))
        RosBackend.update_line_alignment(backend,op,2.2)
        self.assertEqual(calls,[])
        RosBackend.update_line_alignment(backend,op,3.)
        self.assertEqual(calls,[(math.pi/2,'line_settling')])
        self.assertTrue(op['line_turn_used'])
        op['status']='line_settling'
        backend.pose=[.025,.01,.03]
        RosBackend.update_line_alignment(backend,op,3.)
        self.assertEqual(calls[-1],('point',[.025,.01,.03]))
