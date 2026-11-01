"""VM-only ROS interface test: native map_server, synthetic sensors/action servers.

Does NOT test actual AMCL accuracy, native controller tracking, or physical docking.
Refuses the robot architecture and any non-isolated ROS domain.
"""
import json
import math
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import tempfile
import threading
import time

if os.environ.get('ROS_DOMAIN_ID') != '89' or os.environ.get('ROS_LOCALHOST_ONLY') != '1' or platform.machine() != 'x86_64':
    raise RuntimeError('Run on the x86 VM only, ROS_DOMAIN_ID=89, ROS_LOCALHOST_ONLY=1')
ROOT = Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose, FollowPath, Spin
from rcl_interfaces.srv import SetParametersAtomically
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, UInt8, Int8
from tf2_ros import TransformBroadcaster
from map_grid import MapGrid
from mission import Mission
from ros_backend import RosBackend


class Fixture(Node):
    def __init__(self):
        super().__init__('demo_protocol_fixture')
        self.group = ReentrantCallbackGroup();self.pose = [0.,0.,0.]
        self.charging = True;self.nav_calls = 0;self.plan_calls = 0;self.spin_calls=0;self.speed_limits=[]
        self.goal_limits=[];self.speed_refresh_batches=[]
        self.heading_calls=0;self.heading_selections=[];self.position_tree_calls=0
        self.heading_hold=False;self.heading_drift=0.;self.wheel_rate=0.
        self.position_offset=0.;self.heading_yaw_error=0.;self.nav_abort=False
        self.reject_goal_parameters=False;self.goal_parameter_delay=0.
        self.auto_charge=True
        self.broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,'/amcl_pose',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.scan_pub = self.create_publisher(LaserScan,'/scan',10)
        self.charge_pub = self.create_publisher(Bool,'/robot_charging_flag',10)
        self.ir_pub = self.create_publisher(UInt8,'/robot_red_flag',10)
        self.voltage_pub = self.create_publisher(Float32,'/PowerVoltage',10)
        self.current_pub = self.create_publisher(Float32,'/robot_charging_current',10)
        self.wheel_pub = self.create_publisher(Odometry,'/odom',10)
        for name in ('amcl','controller_server','planner_server','bt_navigator','velocity_smoother'):
            self.create_service(GetState,f'/{name}/get_state',self.active,callback_group=self.group)
        self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.initialize,10,callback_group=self.group)
        self.create_subscription(Int8,'/robot_recharge_flag',self.charge,10,callback_group=self.group)
        self.create_timer(.05,self.publish,callback_group=self.group)
        self.planner = ActionServer(self,ComputePathToPose,'/compute_path_to_pose',self.plan,callback_group=self.group)
        self.navigator = ActionServer(self,NavigateToPose,'/navigate_to_pose',self.navigate,
                                     cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
        self.spinner=ActionServer(self,Spin,'/spin',self.spin,
                                 cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
        self.follower=ActionServer(self,FollowPath,'/follow_path',self.heading,
                                  cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
        self.create_service(SetParametersAtomically,'/controller_server/set_parameters_atomically',self.set_speed,callback_group=self.group)

    def set_speed(self,request,response):
        values={p.name:p.value.double_value for p in request.parameters}
        if 'goal_checker.xy_goal_tolerance' in values or 'point_position_checker.xy_goal_tolerance' in values:
            time.sleep(self.goal_parameter_delay)
            if self.reject_goal_parameters:
                response.result.successful=False;response.result.reason='fixture rejects target setup';return response
            self.goal_limits.append([values.get('goal_checker.xy_goal_tolerance',values.get('point_position_checker.xy_goal_tolerance')),
                                     values.get('goal_checker.yaw_goal_tolerance')])
        for prefix in ('FollowPath','PointPosition'):
            if prefix+'.max_speed_xy' in values:
                self.speed_refresh_batches.append(prefix+'.min_speed_theta' in values)
        self.speed_limits.extend(p.value.double_value for p in request.parameters if p.name.endswith('.max_vel_x'))
        self.speed_limits.extend(p.value.double_value for p in request.parameters if p.name.endswith('.desired_linear_vel'))
        response.result.successful=True;return response

    def spin(self,handle):
        self.spin_calls+=1;start=self.pose[2];angle=handle.request.target_yaw
        steps=max(1,int(abs(angle)/.03))
        for i in range(1,steps+1):
            if handle.is_cancel_requested:handle.canceled();return Spin.Result()
            self.pose[2]=start+angle*i/steps;time.sleep(.02)
        self.publish();handle.succeed();return Spin.Result()

    def heading(self,handle):
        self.heading_calls+=1
        self.heading_selections.append([handle.request.controller_id,handle.request.goal_checker_id])
        while self.heading_hold:
            if handle.is_cancel_requested:handle.canceled();return FollowPath.Result()
            time.sleep(.02)
        p=handle.request.path.poses[-1].pose
        target=2*math.atan2(p.orientation.z,p.orientation.w)
        start=self.pose[2];turn=math.atan2(math.sin(target-start),math.cos(target-start))
        steps=max(1,math.ceil(abs(turn)/.035))
        for i in range(1,steps+1):
            if handle.is_cancel_requested:handle.canceled();return FollowPath.Result()
            self.pose[2]=start+turn*i/steps;time.sleep(.025)
        self.pose[0]+=self.heading_drift
        self.pose[2]+=self.heading_yaw_error
        self.publish();handle.succeed();return FollowPath.Result()

    def active(self,request,response):
        response.current_state.id = 3;response.current_state.label = 'active';return response

    def initialize(self,message):
        p=message.pose.pose;self.pose=[p.position.x,p.position.y,2*math.atan2(p.orientation.z,p.orientation.w)]

    def charge(self,message):
        if message.data == 1 and self.auto_charge:
            self.charging=True;self.pose=[0.,0.,0.]

    def publish(self):
        stamp=self.get_clock().now().to_msg()
        tf=TransformStamped();tf.header.stamp=stamp;tf.header.frame_id='map';tf.child_frame_id='base_footprint'
        tf.transform.translation.x,tf.transform.translation.y=self.pose[:2]
        tf.transform.rotation.z=math.sin(self.pose[2]/2);tf.transform.rotation.w=math.cos(self.pose[2]/2)
        laser=TransformStamped();laser.header.stamp=stamp;laser.header.frame_id='base_footprint';laser.child_frame_id='laser'
        laser.transform.translation.x=.03163;laser.transform.rotation.w=1.
        self.broadcaster.sendTransform([tf,laser])
        pose=PoseWithCovarianceStamped();pose.header.stamp=stamp;pose.header.frame_id='map'
        pose.pose.pose.position.x,pose.pose.pose.position.y=self.pose[:2];pose.pose.pose.orientation=tf.transform.rotation
        pose.pose.covariance[0]=.0025;pose.pose.covariance[7]=.0025;pose.pose.covariance[35]=.01
        self.pose_pub.publish(pose)
        scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='laser';scan.angle_min=-math.pi
        scan.angle_increment=2*math.pi/360;scan.range_min=.05;scan.range_max=10.;scan.ranges=[2.]*360
        self.scan_pub.publish(scan);self.charge_pub.publish(Bool(data=self.charging));self.ir_pub.publish(UInt8(data=2))
        self.voltage_pub.publish(Float32(data=12.));self.current_pub.publish(Float32(data=1.68 if self.charging else 0.))
        wheel=Odometry();wheel.twist.twist.angular.z=self.wheel_rate;self.wheel_pub.publish(wheel)

    def plan(self,handle):
        self.plan_calls+=1;result=ComputePathToPose.Result()
        result.path.header.frame_id='map';result.path.poses=[handle.request.goal,handle.request.goal]
        handle.succeed();return result

    def navigate(self,handle):
        self.nav_calls+=1;self.charging=False;p=handle.request.pose.pose
        goal=[p.position.x,p.position.y,2*math.atan2(p.orientation.z,p.orientation.w)];start=list(self.pose)
        if handle.request.behavior_tree.endswith('waypoint_position.xml'):
            self.position_tree_calls+=1;goal[2]=start[2];goal[0]+=self.position_offset
        elif self.position_only:goal[2]=start[2]
        turn=math.atan2(math.sin(goal[2]-start[2]),math.cos(goal[2]-start[2]))
        steps=max(1,math.ceil(max(math.hypot(goal[0]-start[0],goal[1]-start[1])/.15,abs(turn)/.3)/.05))
        for i in range(1,steps+1):
            if handle.is_cancel_requested:
                handle.canceled();return NavigateToPose.Result()
            self.pose=[start[0]+(goal[0]-start[0])*i/steps,start[1]+(goal[1]-start[1])*i/steps,start[2]+turn*i/steps]
            time.sleep(.05)
        self.pose=goal;self.publish();time.sleep(.15)
        if self.nav_abort:handle.abort()
        else:handle.succeed()
        return NavigateToPose.Result()


def main():
    process = subprocess.Popen([str(Path(get_package_prefix('nav2_map_server'))/'lib/nav2_map_server/map_server'),
        '--ros-args','-r','__node:=map_server','-p','yaml_filename:='+str(ROOT/'assets/map.yaml'),
        '-p','use_sim_time:=false'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    rclpy.init();n=Node('map_test_setup');executor=None;backend=None;fixture=None;thread=None
    try:
        c=n.create_client(ChangeState,'/map_server/change_state')
        if not c.wait_for_service(timeout_sec=8):raise RuntimeError('Native map server unavailable')
        for transition in (1,3):  # configure, activate
            req=ChangeState.Request();req.transition.id=transition
            f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5)
            if not f.done() or not f.result().success:raise RuntimeError('Native map lifecycle failed')
        grid=MapGrid(ROOT/'assets');dock=json.loads((ROOT/'assets/dock_start.json').read_text())
        backend=RosBackend(grid,dock);fixture=Fixture();fixture.position_only=backend.staged_arrival;executor=MultiThreadedExecutor(num_threads=4)
        executor.add_node(backend);executor.add_node(fixture)
        thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
        deadline=time.monotonic()+12
        while time.monotonic()<deadline and not backend.snapshot()['ready']:time.sleep(.1)
        if not backend.snapshot()['ready']:raise RuntimeError(str(backend.snapshot()))
        backend.initialize_pose(backend.dock_pose,known_dock=True)
        time.sleep(.5)
        backend.confirm_localization()
        with tempfile.TemporaryDirectory() as directory:
            mission=Mission(grid,backend,directory)
            mission.save_points([{'id':'a','name':'A','x':.25,'y':-.5,'yaw':-1.57,'wait':.3},
                                 {'id':'b','name':'B','x':.25,'y':-1.,'yaw':.5,'wait':.2}])
            mission.start(auto_return=True);deadline=time.monotonic()+40
            while time.monotonic()<deadline and mission.phase not in ('docked','failed'):
                mission.tick();time.sleep(.05)
            proof={'result':mission.phase,'native_map_matches':backend.map_matches,
                   'unknown_cells':grid.cells.count(-1),'plan_requests':fixture.plan_calls,
                   'navigate_requests':fixture.nav_calls,'departure_included':True,'charging_feedback':backend.snapshot()['charging'],
                   'native_spin_requests':fixture.spin_calls,'native_speed_limits':fixture.speed_limits,
                   'native_heading_requests':fixture.heading_calls,'native_heading_selections':fixture.heading_selections,
                   'position_only_tree_requests':fixture.position_tree_calls,
                   'native_goal_tolerances':fixture.goal_limits,
                   'dwb_speed_cache_refresh_batches':fixture.speed_refresh_batches,
                   'scope':'VM isolated ROS protocol + native map loading; synthetic sensors/action servers, no robot',
                   'events':list(mission.events)}
            (ROOT/'ros_protocol_validation.json').write_text(json.dumps(proof,ensure_ascii=False,indent=2))
            print(json.dumps(proof,ensure_ascii=False,indent=2),flush=True)
            if mission.phase!='docked' or not backend.map_matches or fixture.nav_calls!=4:
                raise RuntimeError('ROS protocol rehearsal failed')
            if backend.staged_arrival and (fixture.spin_calls<2 or .05 not in fixture.speed_limits):
                raise RuntimeError('Staged native arrival or adaptive speed was not exercised')
            if backend.precision_arrival:
                waypoint_limit=[backend.position_xy_tolerance if backend.sequential_waypoints else backend.goal_xy_tolerance,
                                None if backend.sequential_waypoints else backend.goal_yaw_tolerance]
                expected=[[.05,math.radians(8)],waypoint_limit,waypoint_limit,[.08,math.radians(10)]]
                if fixture.goal_limits!=expected or not all(fixture.speed_refresh_batches):
                    raise RuntimeError('Transit/measurement goal separation or DWB speed refresh failed')
                if backend.sequential_waypoints and (fixture.heading_calls!=2 or fixture.position_tree_calls!=2 or
                        fixture.spin_calls!=0 or fixture.heading_selections!=[['PointHeading','point_heading_checker']]*2):
                    raise RuntimeError('Waypoint position/heading must be separate; transit/return must not use terminal heading')
                fixture.reject_goal_parameters=True;nav_before=fixture.nav_calls
                identity=backend.navigate({'x':.25,'y':-.5,'yaw':-1.57},'waypoint')
                deadline=time.monotonic()+5
                while time.monotonic()<deadline and backend.operation(identity)['status'] not in ('failed','succeeded'):time.sleep(.02)
                if backend.operation(identity)['status']!='failed' or fixture.nav_calls!=nav_before:
                    raise RuntimeError('Rejected target setup must not start motion navigation')
                fixture.reject_goal_parameters=False;fixture.goal_parameter_delay=.5
                identity=backend.navigate({'x':.25,'y':-.5,'yaw':-1.57},'waypoint')
                deadline=time.monotonic()+5
                while time.monotonic()<deadline and backend.ops[identity]['status']!='configuring':time.sleep(.01)
                backend.cancel(identity);time.sleep(.8)
                if backend.operation(identity)['status']!='canceled' or fixture.nav_calls!=nav_before:
                    raise RuntimeError('Cancellation during setup must suppress navigation')
                proof.update(rejected_setup_prevents_navigation=True,canceled_setup_prevents_navigation=True)
                fixture.auto_charge=False
                with backend.lock:
                    now=time.monotonic();backend.charging=False;backend.last_charge=now
                    backend.begin_docking();backend.docking_since=now-20
                    backend.ir_count=0;backend.last_ir=now;backend.docking_last_valid_ir=now-.2
                    backend.update()
                    if backend.docking_error or backend.docking_since is None:
                        raise RuntimeError('Brief IR gap late in docking must not cause timeout')
                    backend.docking_ir_lost_since=now-8.2;backend.docking_last_valid_ir=now-10
                    backend.update()
                    if not backend.docking_error or backend.docking_since is not None:
                        raise RuntimeError('Continuous IR loss must stop docking')
                proof.update(brief_late_ir_gap_tolerated=True,continuous_ir_loss_stops=True)
                if backend.sequential_waypoints:
                    import yaml
                    import xml.etree.ElementTree as ET
                    cfg=yaml.safe_load((ROOT/'navigation.yaml').read_text())['controller_server']['ros__parameters']
                    transit=ET.parse(ROOT/'assets/native_transit.xml').find('.//FollowPath')
                    point=ET.parse(ROOT/'assets/waypoint_position.xml').find('.//FollowPath')
                    for tree_path in ('native_transit.xml','waypoint_position.xml'):
                        initial=ET.parse(ROOT/'assets'/tree_path).find('.//Sequence[@name="FreshGoalPathFirst"]')
                        if initial is None or initial[0].tag!='ComputePathToPose' or initial[-1].tag!='PipelineSequence':
                            raise RuntimeError('Every new goal must wait for its first fresh path before following')
                    if not ET.parse(ROOT/'assets/waypoint_position.xml').findall('.//SmoothPath[@check_for_collisions="true"]'):
                        raise RuntimeError('Waypoint smoothing must use the native collision check')
                    tree=ET.parse(ROOT/'assets/waypoint_position.xml')
                    if (not tree.findall('.//RecoveryNode') or
                            not tree.findall('.//FollowPath[@controller_id="PointRecovery"]') or
                            'ObstacleFootprint' not in cfg['PointRecovery']['critics'] or
                            'RotateToGoal' in cfg['PointRecovery']['critics']):
                        raise RuntimeError('Blocked path recovery must retain footprint collision checking and separate final heading')
                    if (transit.get('goal_checker_id')!='goal_checker' or transit.get('controller_id')!='FollowPath' or
                            point.get('goal_checker_id')!='point_position_checker' or point.get('controller_id')!='PointPosition'):
                        raise RuntimeError('Every BT must explicitly select its native controller and checker')
                    if (cfg['PointHeading']['min_vel_x']!=0 or cfg['PointHeading']['max_vel_x']!=0 or
                            'ObstacleFootprint' not in cfg['PointHeading']['critics'] or
                            'RegulatedPurePursuitController' not in cfg['PointPosition']['plugin'] or
                            not cfg['PointPosition']['use_collision_detection']):
                        raise RuntimeError('Position/heading controller separation is not enforced')
                    fixture.goal_parameter_delay=0.;fixture.charging=False
                    def wait_for(test,seconds=8):
                        end=time.monotonic()+seconds
                        while time.monotonic()<end:
                            if test():return
                            time.sleep(.02)
                        raise RuntimeError('Timed out waiting for staged controller test')
                    time.sleep(.2);h_before=fixture.heading_calls;nav_before=fixture.nav_calls
                    identity=backend.navigate({'x':fixture.pose[0],'y':fixture.pose[1],'yaw':fixture.pose[2]},'waypoint')
                    wait_for(lambda:backend.operation(identity)['status'] in ('succeeded','failed'))
                    if backend.operation(identity)['status']!='succeeded' or fixture.heading_calls!=h_before or fixture.nav_calls!=nav_before:
                        raise RuntimeError('Already aligned waypoint must skip heading motion')
                    fixture.heading_hold=True
                    identity=backend.navigate({'x':.25,'y':-.5,'yaw':1.},'waypoint')
                    wait_for(lambda:backend.operation(identity)['status']=='heading')
                    backend.cancel(identity)
                    wait_for(lambda:backend.operation(identity)['status']=='canceled')
                    fixture.heading_hold=False;fixture.heading_drift=.045
                    identity=backend.navigate({'x':.25,'y':-.5,'yaw':-.5},'waypoint')
                    wait_for(lambda:backend.operation(identity)['status'] in ('succeeded','failed'))
                    if backend.operation(identity)['status']!='succeeded' or backend.operation(identity)['quality']!='normal':
                        raise RuntimeError('A stopped 4.5 cm point must not abort the route')
                    fixture.heading_drift=.065;nav_before=fixture.nav_calls
                    identity=backend.navigate({'x':.25,'y':-.65,'yaw':.5},'waypoint')
                    wait_for(lambda:backend.operation(identity)['status'] in ('succeeded','failed'))
                    if backend.operation(identity)['status']!='succeeded' or fixture.nav_calls!=nav_before+2:
                        raise RuntimeError('One position recovery must resolve terminal rotation drift')
                    fixture.heading_drift=0.;fixture.position_offset=.06;fixture.heading_yaw_error=math.radians(4)
                    backend.arrival['allow_near_arrival']=True  # Exercise optional mode, disabled on the car.
                    mission.save_points([{'id':'near','name':'Near','x':.25,'y':-.5,'yaw':-.5,'wait':.1}])
                    mission.start();wait_for(lambda: (mission.tick() or mission.phase in ('completed','failed')))
                    if mission.phase!='completed' or backend.operation(mission.backend.active)['quality']!='near':
                        raise RuntimeError('Usable near arrival must finish its dwell instead of failing the route')
                    if not any('按可用精度继续任务' in event['text'] for event in mission.events):
                        raise RuntimeError('Near arrival must disclose actual error in mission events')
                    backend.arrival['allow_near_arrival']=False
                    fixture.position_offset=.10;fixture.heading_yaw_error=0.;nav_before=fixture.nav_calls
                    identity=backend.navigate({'x':.25,'y':-.5,'yaw':.5},'waypoint')
                    wait_for(lambda:backend.operation(identity)['status']=='failed')
                    if '仍未到目标附近' not in backend.operation(identity)['error'] or fixture.nav_calls!=nav_before+2:
                        raise RuntimeError('A persistent large position error must fail after one bounded recovery')
                    fixture.position_offset=0.;fixture.nav_abort=True
                    identity=backend.navigate({'x':fixture.pose[0]+.08,'y':fixture.pose[1],'yaw':fixture.pose[2]},'waypoint')
                    wait_for(lambda:backend.operation(identity)['status'] in ('succeeded','failed'))
                    if backend.operation(identity)['status']!='succeeded':
                        raise RuntimeError('Native abort at a verified usable stopped pose must not fail arrival')
                    fixture.nav_abort=False;fixture.heading_hold=True
                    target=fixture.pose[2]-math.radians(4)
                    identity=backend.navigate({'x':fixture.pose[0],'y':fixture.pose[1],'yaw':target},'waypoint')
                    wait_for(lambda:backend.operation(identity)['status']=='heading')
                    fixture.pose[2]=target+math.radians(2);time.sleep(.2)
                    with backend.lock:backend.ops[identity]['started']=time.monotonic()-31
                    wait_for(lambda:backend.operation(identity)['status'] in ('succeeded','failed'))
                    if backend.operation(identity)['status']!='succeeded' or backend.operation(identity)['quality']!='normal':
                        raise RuntimeError('Heading deadline at verified accurate pose must settle and continue')
                    target=fixture.pose[2]-math.radians(4)
                    identity=backend.navigate({'x':fixture.pose[0],'y':fixture.pose[1],'yaw':target},'waypoint')
                    wait_for(lambda:backend.operation(identity)['status']=='heading')
                    with backend.lock:
                        backend.ops[identity]['started']=time.monotonic()-31
                        backend.update();backend.cancel(identity)
                    wait_for(lambda:backend.operation(identity)['status']=='canceled')
                    if backend.ops[identity].get('quality'):
                        raise RuntimeError('User cancellation during deadline recovery must never become arrival')
                    identity=backend.navigate({'x':fixture.pose[0],'y':fixture.pose[1],'yaw':1.},'waypoint')
                    wait_for(lambda:backend.operation(identity)['status']=='heading')
                    fixture.wheel_rate=5.
                    wait_for(lambda:backend.operation(identity)['status']=='failed')
                    fixture.wheel_rate=0.;fixture.heading_hold=False
                    if '转动量超出限额' not in backend.operation(identity)['error']:
                        raise RuntimeError('Excessive physical rotation must cancel heading instead of looping')
                    proof.update(heading_controller_translation_disabled=True,already_aligned_skips_heading=True,
                                 heading_cancel_acknowledged=True,ordinary_45mm_arrival_accepted=True,
                                 terminal_translation_recovered_once=True,persistent_large_error_stops=True,
                                 near_arrival_disclosed_and_dwell_completed=True,native_abort_at_verified_pose_accepted=True,
                                 heading_deadline_at_usable_pose_continues=True,user_stop_overrides_deadline_recovery=True,
                                 already_at_point_skips_navigation=True,
                                 fresh_goal_path_before_controller=True,
                                 native_waypoint_smoothing_collision_checked=True,
                                 production_near_arrival_disabled=not backend.arrival.get('allow_near_arrival',False),
                                 excessive_rotation_canceled=True)
                    nav_before=fixture.nav_calls;heading_before=fixture.heading_calls;reference=list(backend.dock_pose)
                    with backend.lock:
                        fixture.pose=[reference[0]+.08,reference[1]-.04,reference[2]+.08]
                        fixture.charging=True;backend.confirmed=False;backend.dock_localization.arm()
                    wait_for(lambda:backend.snapshot()['dock_localization']['status']=='confirmed',seconds=14)
                    if (not backend.confirmed or fixture.nav_calls!=nav_before or fixture.heading_calls!=heading_before
                            or backend.dock_pose!=reference or math.dist(fixture.pose[:2],reference[:2])>.001):
                        raise RuntimeError('Dock auto localization must initialize AMCL, confirm, and preserve the dock without motion')
                    proof.update(stable_charging_auto_initializes_amcl=True,
                                 dock_auto_confirmation_without_motion=True,dock_reference_not_overwritten=True)
                (ROOT/'ros_protocol_validation.json').write_text(json.dumps(proof,ensure_ascii=False,indent=2))
    finally:
        if backend:backend.stop()
        if executor:executor.shutdown(timeout_sec=2)
        if thread:thread.join(timeout=2)
        if backend:backend.destroy_node()
        if fixture:fixture.destroy_node()
        n.destroy_node();rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait()


if __name__=='__main__':main()
