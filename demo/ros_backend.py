"""Bridge to native Nav2/AMCL and the factory STM32 IR docking mode."""
import math
from collections import deque
from pathlib import Path
import threading
import time
import uuid
import json
import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.action import ComputePathToPose, NavigateToPose, FollowPath, Spin
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, UInt8, Int8
from tf2_ros import Buffer, TransformListener
from dock_localization import DockLocalization, localization_inputs_ready
from arrival_policy import refinement_stage, heading_profile, should_capture_pose
from charge_confirmation import ChargeConfirmation, NativeDockingWatchdog


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


from survey_ros import SurveyPathMixin

class RosBackend(SurveyPathMixin, Node):
    mode = 'robot'
    def __init__(self, grid, dock):
        super().__init__('lab_demo_bridge')
        config=yaml.safe_load((Path(__file__).parent/'navigation.yaml').read_text())
        goal_config=config['controller_server']['ros__parameters']['goal_checker']
        self.goal_xy_tolerance=goal_config['xy_goal_tolerance']
        self.goal_yaw_tolerance=goal_config['yaw_goal_tolerance']
        self.position_xy_tolerance=config['controller_server']['ros__parameters'].get('point_position_checker',goal_config)['xy_goal_tolerance']
        point_controller=config['controller_server']['ros__parameters'].get('PointPosition',{})
        self.position_is_rpp=any('RegulatedPurePursuitController' in point_controller.get(k,'')
                                 for k in ('plugin','primary_controller'))
        self.minimum_angular_speed=config['controller_server']['ros__parameters']['FollowPath'].get('min_speed_theta',0.)
        self.smoothing='velocity_smoother' in config
        self.arrival=config.get('demo_arrival',{}).get('ros__parameters',{})
        self.precision_arrival=self.arrival.get('enabled',False)
        self.joint_terminal=self.arrival.get('joint_terminal_controller',False)
        self.docking_timeout=float(self.arrival.get('docking_timeout',90.))
        self.staged_arrival=self.precision_arrival and self.arrival.get('final_spin',True)
        self.sequential_waypoints=self.precision_arrival and self.arrival.get('waypoint_position_then_heading',False)
        self.native_behavior_tree=(str(Path(get_package_share_directory('nav2_bt_navigator'))/'behavior_trees/navigate_w_replanning_only_if_path_becomes_invalid.xml')
                                   if self.precision_arrival else '')
        if self.sequential_waypoints:
            self.native_behavior_tree=str(Path(__file__).parent/'assets/native_transit.xml')
        self.speed_limit=None;self.speed_limits={};self.speed_update=None
        info_path=Path(__file__).parent/'factory_config_diff.json'
        self.profile=json.loads(info_path.read_text()).get('profile','factory') if info_path.exists() else 'factory'
        self.grid = grid;self.lock = threading.RLock()
        self.dock_calibrated=dock.get('calibrated',True)
        home = dock['robot_pose'];self.dock_pose = [home['x'], home['y'], home['yaw']]
        self.dock_localization=DockLocalization(self.dock_pose);self.last_amcl=0.
        self.departure_distance = float(dock.get('departure_distance',.30))
        d = dock['charger_approach']
        self.dock_approach = {'x': d['p_x'], 'y': d['p_y'], 'yaw': 2*math.atan2(d['orien_z'], d['orien_w'])}
        self.pose = None;self.confirmed = False;self.last_scan = 0;self.laser = [];self.pending_scans = deque(maxlen=6)
        self.amcl_seen = False;self.covariance = None;self.map_matches = False;self.known_dock_start=False
        self.charging = None;self.last_charge = 0;self.voltage = None;self.current = None
        self.last_numeric={};self.charge_confirmation=ChargeConfirmation();self.charging_stable=False
        self.native_docking_watchdog=NativeDockingWatchdog()
        self.native_docking_stage='native_capture';self.charge_motion_since=None
        self.ir_count = 0;self.last_ir = 0;self.docking_since = None;self.docking_error = None
        self.docking_ir_lost_since=None;self.docking_last_valid_ir=None
        self.ops = {};self.active = None;self.path = [];self.initial_request = None
        self.last_wheel = 0;self.stationary_since = None;self.wheel_distance=0.
        self.tf = Buffer();self.listener = TransformListener(self.tf, self)
        self.plan = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.follow = ActionClient(self, FollowPath, '/follow_path')
        self.spin_action = ActionClient(self, Spin, '/spin')
        self.controller_parameters=self.create_client(SetParametersAtomically,'/controller_server/set_parameters_atomically')
        self.survey_planner_parameters=self.create_client(SetParametersAtomically,'/planner_server/set_parameters_atomically')
        self.survey_plan_desired=None;self.survey_plan_applied=None;self.survey_plan_future=None;self.survey_plan_retry_at=0.

        self.cmd = self.create_publisher(Twist, '/cmd_vel', 1)
        self.raw_cmd=self.create_publisher(Twist,'/nav_cmd_vel',1) if self.smoothing else None
        self.ir = self.create_publisher(Int8, '/robot_recharge_flag', 10)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.map_received, q)
        self.create_subscription(NavPath, '/plan', self.path_received, 10)
        self.create_subscription(NavPath, '/plan_smoothed', self.path_received, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_received, q)
        self.create_subscription(LaserScan, '/scan', self.scan_received, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.wheel_received, qos_profile_sensor_data)
        self.create_subscription(Bool, '/robot_charging_flag', self.charge_received, qos_profile_sensor_data)
        self.create_subscription(UInt8, '/robot_red_flag', self.ir_received, qos_profile_sensor_data)
        self.create_subscription(Float32, '/PowerVoltage', lambda m: self.numeric_received('voltage',m), qos_profile_sensor_data)
        self.create_subscription(Float32, '/robot_charging_current', lambda m: self.numeric_received('current',m), qos_profile_sensor_data)
        self.lifecycle = {};self.inflight = set()
        self.mapping = False
        self.lifecycle_clients = {name: self.create_client(GetState, f'/{name}/get_state')
                        for name in ('amcl','map_server','controller_server','planner_server','bt_navigator')+ (('velocity_smoother',) if self.smoothing else ())}
        self.create_timer(2.0, self.probe)
        self.create_timer(.1, self.update)

    def map_received(self, m):
        good = (m.info.width == self.grid.width and m.info.height == self.grid.height
                and abs(m.info.resolution-self.grid.resolution) < 1e-6
                and abs(m.info.origin.position.x-self.grid.origin[0]) < 1e-5
                and abs(m.info.origin.position.y-self.grid.origin[1]) < 1e-5
                and abs(yaw(m.info.origin.orientation)) < 1e-5
                and list(m.data) == self.grid.cells)
        with self.lock:
            self.map_matches = good
            if not good:
                self.confirmed = False

    def amcl_received(self, m):
        with self.lock:
            self.amcl_seen = True
            self.last_amcl=time.monotonic()
            c = m.pose.covariance
            self.covariance = [float(c[i]) if math.isfinite(c[i]) else None for i in (0,7,35)]

    def path_received(self, m):
        with self.lock:
            if m.header.frame_id.lstrip('/') != 'map' or not m.poses or not self.active:
                return
            op=self.ops[self.active]
            if op['status'] not in ('planning','navigating'):
                return
            end=m.poses[-1].pose.position;goal=op['goal']
            if math.hypot(end.x-goal['x'],end.y-goal['y'])<.4:
                self.path=[[p.pose.position.x,p.pose.position.y] for p in m.poses
                           if math.isfinite(p.pose.position.x) and math.isfinite(p.pose.position.y)]

    def charge_received(self, m):
        with self.lock:
            self.charging = bool(m.data);self.last_charge = time.monotonic()

    def numeric_received(self, name, m):
        with self.lock:
            setattr(self,name,float(m.data) if math.isfinite(m.data) else None)
            self.last_numeric[name]=time.monotonic()

    def wheel_received(self, m):
        with self.lock:
            now=time.monotonic();dt=min(.2,max(0.,now-self.last_wheel));self.last_wheel=now
            t=m.twist.twist
            if math.isfinite(t.linear.x) and math.isfinite(t.linear.y):
                self.wheel_distance+=math.hypot(t.linear.x,t.linear.y)*dt
            if self.active:
                op=self.ops[self.active]
                if op['status'] in ('heading','heading_wait','heading_settling') and math.isfinite(t.angular.z):
                    op['heading_travel']=op.get('heading_travel',0.)+abs(t.angular.z)*dt
            stopped=all(math.isfinite(v) and abs(v)<limit for v,limit in
                        ((t.linear.x,.02),(t.linear.y,.02),(t.angular.z,.05)))
            self.stationary_since=(self.stationary_since or now) if stopped else None

    def ir_received(self, m):
        with self.lock:
            self.ir_count = int(m.data);self.last_ir = time.monotonic()

    def scan_received(self, m):
        with self.lock:
            self.last_scan = time.monotonic()
            self.pending_scans.append(m)

    def update_laser(self):
        # TF can arrive just after a scan. Retry its original timestamp rather
        # than drawing it against a newer robot pose or clearing every frame.
        with self.lock:
            if not self.pending_scans:
                return
            try:
                t = None
                for m in reversed(self.pending_scans):
                    try:
                        t = self.tf.lookup_transform('map', m.header.frame_id, Time.from_msg(m.header.stamp)).transform
                        break
                    except Exception:
                        continue
                if t is None:
                    raise ValueError('waiting for scan-time TF')
                angle = yaw(t.rotation);points = []
                for i in range(0, len(m.ranges), max(1, len(m.ranges)//320)):
                    distance = m.ranges[i]
                    if math.isfinite(distance) and m.range_min <= distance <= min(m.range_max, 12):
                        a = angle+m.angle_min+i*m.angle_increment
                        points.append([t.translation.x+distance*math.cos(a), t.translation.y+distance*math.sin(a)])
                self.laser = points  # Display only; native localization/navigation gets the full scan.
            except Exception:
                if time.monotonic()-self.last_scan > 2:
                    self.laser = [];self.pending_scans.clear()

    def probe(self):
        self.mapping = any('slam_toolbox' in name for name in self.get_node_names())
        for name, c in self.lifecycle_clients.items():
            if c.service_is_ready() and name not in self.inflight:
                self.inflight.add(name)
                c.call_async(GetState.Request()).add_done_callback(lambda f, k=name: self.lifecycle_result(f,k))

    def lifecycle_result(self, future, name):
        with self.lock:
            self.inflight.discard(name)
            try:
                self.lifecycle[name] = (future.result().current_state.id, time.monotonic())
            except Exception:
                self.lifecycle[name] = (0, time.monotonic())

    def update(self):
        with self.lock:
            self.update_laser()
            now = time.monotonic()
            charging_flag=bool(self.charging and now-self.last_charge<2)
            charge_current=self.current if now-self.last_numeric.get('current',0)<2 else None
            self.charging_stable=self.charge_confirmation.update(now,
                charging=charging_flag,
                stationary=bool(now-self.last_wheel<2 and self.stationary_since is not None and now-self.stationary_since>=.5),
                current=charge_current,
                voltage=self.voltage if now-self.last_numeric.get('voltage',0)<2 else None)
            try:
                m = self.tf.lookup_transform('map', 'base_footprint', Time())
                t = m.transform
                if abs(time.time()-m.header.stamp.sec-m.header.stamp.nanosec/1e9) > 2:
                    raise ValueError('stale transform')
                pose = [t.translation.x, t.translation.y, yaw(t.rotation)]
                if not all(math.isfinite(v) for v in pose):
                    raise ValueError('nonfinite transform')
                self.pose = pose
            except Exception:
                self.pose = None
            if self.initial_request:
                message, remaining = self.initial_request
                message.header.stamp = self.get_clock().now().to_msg();self.initial.publish(message)
                self.initial_request = (message, remaining-1) if remaining > 1 else None
            if self.active:
                op = self.ops[self.active]
                self.survey_route_tick(op)
                if op['status'] in ('succeeded','failed','canceled'):op.setdefault('finished_at',now)
                if op['status'] == 'canceling':
                    self.zero_motion()
                elif (self.joint_terminal and op['status'] in ('heading','heading_wait') and self.pose
                      and self.stationary_since is not None and now-self.stationary_since>=.7
                      and refinement_stage(math.hypot(self.pose[0]-op['goal']['x'],self.pose[1]-op['goal']['y']),
                          math.atan2(math.sin(op['goal']['yaw']-self.pose[2]),math.cos(op['goal']['yaw']-self.pose[2])),
                          position_tolerance=self.arrival.get('final_position_tolerance',.05),
                          yaw_tolerance=op['yaw_tolerance'])=='complete'):
                    # Apply the same stopped-pose acceptance throughout, rather
                    # than waiting for a stricter inner checker to time out.
                    op.update(cancel=True,status='canceling',settle_after_cancel='heading_settling')
                    if op['handle']:op['handle'].cancel_goal_async()
                    self.zero_motion()
                elif (not self.joint_terminal and op['status'] in ('heading','heading_wait')
                      and op.get('heading_travel',0.)>op['heading_budget']):
                    op.update(cancel=True,status='canceling',settle_after_cancel='heading_settling')
                    if op['handle']:op['handle'].cancel_goal_async()
                    self.zero_motion()
                elif op['status'] not in ('succeeded','canceled','failed') and now-op['started'] > op['timeout']:
                    if op['sequential'] and op['status'] in ('navigating','heading','heading_wait'):
                        # End the native action before checking usable arrival.
                        # An elapsed refinement timer is not a user stop.
                        stage='position_settling' if op['status']=='navigating' else 'heading_settling'
                        op.update(cancel=True,status='canceling',settle_after_cancel=stage)
                        if op['handle']:op['handle'].cancel_goal_async()
                        self.zero_motion()
                    else:
                        op['error'] = '导航等待超时，已请求停车';self.cancel(self.active)
                elif op['status']=='position_settling':
                    self.finish_waypoint_position(op,now)
                elif op['status']=='heading_settling':
                    self.finish_waypoint_heading(op,now)
                elif op['status']=='recovery_wait' and now>=op['retry_at']:
                    self.send_arrival_retry(op)
                elif self.precision_arrival and op['status'] in ('nav_settling','spin_settling'):
                    self.finish_arrival(op,now)
                elif self.precision_arrival and op['status']=='navigating' and self.pose:
                    distance=math.hypot(self.pose[0]-op['goal']['x'],self.pose[1]-op['goal']['y'])
                    angle=math.atan2(math.sin(op['goal']['yaw']-self.pose[2]),math.cos(op['goal']['yaw']-self.pose[2]))
                    if op['sequential'] and self.joint_terminal and should_capture_pose(distance,angle):
                        op.update(cancel=True,status='canceling',settle_after_cancel='position_settling')
                        if op['handle']:op['handle'].cancel_goal_async()
                        self.zero_motion()
                        return
                    speed=(self.arrival['creep_speed'] if distance<self.arrival['creep_distance'] else
                           self.arrival['approach_speed'] if distance<self.arrival['approach_distance'] else self.arrival['nominal_speed'])
                    # Return is a transit leg; do not apply the waypoint creep
                    # or terminal precision loop before the OEM IR handoff.
                    if op.get('route_path') and not op.get('route_rejoining'):speed=op.get('track_speed',.08)
                    if op['kind']=='return':speed=.10 if distance<.35 else self.arrival['nominal_speed']
                    self.request_speed(speed,op['controller_id'])
            if self.docking_since is not None:
                self.update_oem_docking(now,charge_current)
            self.update_dock_localization(now)

    def update_oem_docking(self,now,current):
        # The STM32 owns all IR steering and contact compression. A first
        # current/flag frame never resets or stops its state machine.
        if self.charging_stable:
            self.native_docking_stage='complete'
            self.set_oem_ir(False,now);self.cmd.publish(Twist());self.docking_since=None
            self.docking_ir_lost_since=None;return
        fresh=now-self.last_wheel<1 and now-self.last_scan<1 and now-self.last_ir<1
        near=bool(self.pose and math.hypot(self.pose[0]-self.dock_pose[0],self.pose[1]-self.dock_pose[1])<=.18)
        error=self.native_docking_watchdog.update(now,travel=self.wheel_distance,near=near,fresh=fresh)
        if error:self.docking_error=error;self.stop();return
        electrical=current is not None and current>=self.charge_confirmation.minimum_current
        self.native_docking_stage='charging_observed' if electrical else 'native_capture'
        stopped=self.stationary_since is not None and now-self.stationary_since>=.5
        # Guard an abnormal continuously spinning chassis after sustained
        # electrical connection; do not interrupt normal initial contact.
        if electrical and not stopped:
            if self.charge_motion_since is None:self.charge_motion_since=now
            if now-self.charge_motion_since>=2.5:
                self.docking_error='已持续有充电电流但车轮仍未停，已停止原生对接'
                self.stop();return
        else:self.charge_motion_since=None
        if self.ir_count>0 and now-self.last_ir<=2:
            self.docking_last_valid_ir=now;self.docking_ir_lost_since=None
            self.set_oem_ir(True,now)
        else:
            if self.docking_ir_lost_since is None:self.docking_ir_lost_since=now
            grace=self.docking_last_valid_ir is not None and now-self.docking_last_valid_ir<2
            self.set_oem_ir(grace,now)
            if now-self.docking_ir_lost_since>8:
                self.docking_error='桩前持续丢失红外，已停止原生对接';self.stop()


    def update_dock_localization(self,now):
        if not self.dock_calibrated:return
        busy=(self.docking_since is not None or
              bool(self.active and self.ops[self.active]['status'] not in ('succeeded','failed','canceled')))
        health=self.snapshot()
        action=self.dock_localization.step(now,
            charging=self.charging_stable,
            stationary=bool(now-self.last_wheel<2 and self.stationary_since is not None and now-self.stationary_since>=2),
            sensors_ready=localization_inputs_ready(now,map_matches=self.map_matches,
                mapping=self.mapping,last_scan=self.last_scan,lifecycle=self.lifecycle),
            busy=busy,pose=self.pose,amcl_updated=bool(health['ready'] and self.last_amcl>(self.dock_localization.started or now)),
            pose_request_pending=self.initial_request is not None)
        if action=='initialize':
            self.initialize_pose(self.dock_pose,known_dock=True,automatic=True)
        elif action=='confirm' and health['ready']:
            self.confirmed=True
            self.get_logger().info('已按保存的充电桩位自动校准 AMCL 定位；充电桩坐标未修改')

    def snapshot(self):
        with self.lock:
            now = time.monotonic();reason = ''
            arrival=None
            if self.active:
                op=self.ops[self.active];g=op['goal'];p=self.pose
                arrival={'kind':op['kind'],'status':op['status'],
                         'quality':op.get('quality'),
                         'heading_attempts':op['heading_attempts'],'position_retries':op['arrival_retries'],
                         'elapsed':(op.get('finished_at') or now)-op.get('created',op['started']),
                         'position_error':math.hypot(p[0]-g['x'],p[1]-g['y']) if p else None,
                         'yaw_error_deg':math.degrees(abs(math.atan2(math.sin(g['yaw']-p[2]),math.cos(g['yaw']-p[2])))) if p else None}
            states = all(self.lifecycle.get(k,(0,0))[0] == 3 and now-self.lifecycle[k][1] < 8 for k in self.lifecycle_clients)
            if self.mapping:
                reason = '建图进程仍在运行，请先关闭建图，再启动演示'
            elif not self.map_matches:
                reason = '实际导航地图与演示版本不一致或尚未加载'
            elif not states or not self.nav.server_is_ready() or not self.plan.server_is_ready():
                reason = '正在等待导航与定位启动'
            elif now-self.last_scan > 2:
                reason = '雷达数据中断，不能开始导航'
            elif now-self.last_wheel > 2:
                reason = '底盘运动反馈中断，不能开始导航'
            elif self.pose is None or not self.amcl_seen:
                reason = '请在地图上设置当前位置和车头朝向'
            return {'mode': 'robot', 'ready': not reason, 'readiness': reason or '导航与雷达已就绪',
                    'localization_confirmed': self.confirmed, 'pose': self.pose,
                    'dock_position_confirmed':self.known_dock_start and self.pose is not None and math.hypot(self.pose[0]-self.dock_pose[0],self.pose[1]-self.dock_pose[1])<.15 and abs(math.atan2(math.sin(self.pose[2]-self.dock_pose[2]),math.cos(self.pose[2]-self.dock_pose[2])))<.25,
                    'charging': self.charging_stable or (self.charging if now-self.last_charge < 2 else None),
                    'charging_feedback_flag': self.charging if now-self.last_charge < 2 else None,
                    'charging_stable': self.charging_stable,
                    'docking_contact_stage': self.native_docking_stage if self.docking_since is not None else None,
                    'voltage': self.voltage, 'current': self.current,
                    'ir': self.ir_count if now-self.last_ir < 2 else 0,
                    'stationary': now-self.last_wheel<2 and self.stationary_since is not None and now-self.stationary_since>=.5,
                    'path': self.path, 'laser': self.laser, 'map_matches': self.map_matches,
                    'localization_covariance': self.covariance,'navigation_profile':self.profile,'arrival':arrival,
                    'dock_localization':self.dock_localization.snapshot()}

    def tick(self):
        pass

    def initialize_pose(self, pose, known_dock=False,automatic=False):
        with self.lock:
            if not automatic:
                if known_dock:self.dock_localization.manual_dock(time.monotonic())
                else:self.dock_localization.suppress()
            message = PoseWithCovarianceStamped();message.header.frame_id = 'map'
            message.pose.pose.position.x, message.pose.pose.position.y = pose[0], pose[1]
            message.pose.pose.orientation.z = math.sin(pose[2]/2);message.pose.pose.orientation.w = math.cos(pose[2]/2)
            message.pose.covariance[0] = (.05 if known_dock else .5)**2
            message.pose.covariance[7] = (.05 if known_dock else .5)**2
            message.pose.covariance[35] = math.radians(5 if known_dock else 15)**2
            self.confirmed = False;self.amcl_seen = False;self.initial_request = (message, 3);self.known_dock_start=known_dock

    def confirm_localization(self):
        with self.lock:
            health = self.snapshot()
            if not health['ready']:
                raise ValueError(health['readiness'])
            self.confirmed = True

    def stamped(self, point):
        message = PoseStamped();message.header.frame_id = 'map';message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x, message.pose.position.y = point['x'], point['y']
        message.pose.orientation.z = math.sin(point['yaw']/2);message.pose.orientation.w = math.cos(point['yaw']/2)
        return message

    def set_survey_planning(self,enabled):
        if not enabled:self.restore_survey_controller()
        with self.lock:
            self.survey_plan_desired=bool(enabled)
            if self.survey_plan_future or self.survey_plan_applied==self.survey_plan_desired:return
            if time.monotonic()<self.survey_plan_retry_at or not self.survey_planner_parameters.service_is_ready():return
            desired=self.survey_plan_desired;request=SetParametersAtomically.Request()
            request.parameters=[Parameter('WaypointGrid.tolerance',value=.35 if desired else 0.).to_parameter_msg()]
            self.survey_plan_future=self.survey_planner_parameters.call_async(request)
            def done(f):
                with self.lock:
                    try:
                        result=f.result().result
                        if result.successful:self.survey_plan_applied=desired
                        else:self.get_logger().error('采集规划参数未应用：'+result.reason)
                    except Exception as exc:self.get_logger().error('采集规划参数服务异常：'+str(exc))
                    self.survey_plan_future=None;self.survey_plan_retry_at=time.monotonic()+1.
            self.survey_plan_future.add_done_callback(done)

    def navigate(self, point, kind):
        if kind=='survey' and point.get('survey_path'):return self.navigate_survey_path(point)
        with self.lock:
            if self.active and self.ops[self.active]['status'] not in ('succeeded','failed','canceled'):
                raise ValueError('旧导航尚未停止，不能同时发送两个目标')
            identity = uuid.uuid4().hex
            op = {'status': 'planning', 'goal': dict(point), 'kind': kind,
                  'started': time.monotonic(), 'timeout': 20, 'cancel': False, 'handle': None,
                  'heading_attempts':0,'arrival_retries':0}
            op['created']=op['started']
            op['sequential']=self.sequential_waypoints and kind in ('waypoint','survey')
            op['controller_id']='PointPosition' if op['sequential'] else 'FollowPath'
            op['xy_tolerance'] = (.05 if kind=='departure' else .08 if kind=='return' else self.goal_xy_tolerance) if self.precision_arrival else self.goal_xy_tolerance
            if op['sequential']:op['xy_tolerance']=self.position_xy_tolerance
            op['yaw_tolerance'] = (math.radians(8) if kind=='departure' else math.radians(10) if kind=='return' else self.goal_yaw_tolerance) if self.precision_arrival else self.goal_yaw_tolerance
            if kind=='survey':op['xy_tolerance']=min(.22,point.get('survey_pass_radius',.5));op['yaw_tolerance']=math.pi
            self.ops[identity] = op;self.active = identity;self.path = []
            self.dock_localization.arm()
            if op['sequential'] and self.pose and self.stationary_since and time.monotonic()-self.stationary_since>=.7:
                distance=math.hypot(self.pose[0]-point['x'],self.pose[1]-point['y'])
                if distance<=self.arrival.get('final_position_tolerance',.05):
                    op.update(status='position_settling',timeout=15.,already_at_position=True)
                    return identity
            goal = ComputePathToPose.Goal();goal.goal = self.stamped(point)
            goal.planner_id = 'WaypointGrid' if op['sequential'] else 'GridBased';goal.use_start = False
            self.plan.send_goal_async(goal).add_done_callback(lambda f: self.plan_accepted(f,identity))
            return identity

    def plan_accepted(self, future, identity):
        with self.lock:
            op = self.ops[identity]
            try:
                handle = future.result()
                if not handle.accepted:
                    raise ValueError('原生规划器拒绝目标')
                if op['cancel']:
                    handle.cancel_goal_async();op['status'] = 'canceled';return
                handle.get_result_async().add_done_callback(lambda f: self.plan_done(f,identity))
            except Exception as e:
                op['status'] = 'failed';op['error'] = str(e)

    def plan_done(self, future, identity):
        with self.lock:
            op = self.ops[identity]
            if op['cancel']:
                op['status'] = 'canceled';return
            try:
                result = future.result()
                if result.status != 4 or not result.result.path.poses:
                    if op['sequential']:
                        self.retry_arrival(op,'目标或通路被实时障碍占用，等待可通行后自动继续');return
                    raise ValueError('目标没有可通行路线，未发送运动导航')
                self.path = [[p.pose.position.x,p.pose.position.y] for p in result.result.path.poses]
                distance = sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(self.path,self.path[1:]))
                op['timeout'] = max(60,min(600,distance/.08+60));op['started'] = time.monotonic()
                if self.precision_arrival:
                    op['status']='configuring'
                    request=SetParametersAtomically.Request()
                    values=({'point_position_checker.xy_goal_tolerance':op['xy_tolerance']} if op['sequential'] else
                            {'goal_checker.xy_goal_tolerance':op['xy_tolerance'],
                             'goal_checker.yaw_goal_tolerance':op['yaw_tolerance']})
                    request.parameters=[Parameter(k,value=float(v)).to_parameter_msg() for k,v in values.items()]
                    self.controller_parameters.call_async(request).add_done_callback(lambda f:self.goal_configured(f,identity))
                else:
                    self.send_navigation(identity)
            except Exception as e:
                op['status'] = 'failed';op['error'] = str(e)

    def goal_configured(self,future,identity):
        with self.lock:
            op=self.ops[identity]
            if op['cancel']:
                op['status']='canceled';return
            try:
                result=future.result().result
                if not result.successful:raise ValueError('原生目标检查参数设置失败：'+result.reason)
                self.send_navigation(identity)
            except Exception as e:op.update(status='failed',error=str(e))

    def send_navigation(self,identity):
        op=self.ops[identity];op['status']='navigating'
        goal=NavigateToPose.Goal();goal.pose=self.stamped(op['goal'])
        goal.behavior_tree=(str(Path(__file__).parent/'assets/waypoint_position.xml')
                            if op['sequential'] else self.native_behavior_tree)
        self.nav.send_goal_async(goal).add_done_callback(lambda f:self.nav_accepted(f,identity))

    def nav_accepted(self, future, identity):
        with self.lock:
            op = self.ops[identity]
            try:
                handle = future.result()
                if not handle.accepted:
                    raise ValueError('原生导航拒绝目标')
                op['handle'] = handle
                handle.get_result_async().add_done_callback(lambda f: self.nav_done(f,identity))
                if op['cancel']:
                    handle.cancel_goal_async()
            except Exception as e:
                op['status'] = 'failed';op['error'] = str(e)

    def nav_done(self, future, identity):
        with self.lock:
            op = self.ops[identity]
            try:
                result = future.result()
                if op['cancel']:
                    self.finish_native_cancellation(op)
                elif result.status == 4:
                    if op.get('route_rejoining'):
                        self.finish_survey_rejoin(identity);return
                    p = self.pose;g = op['goal']
                    if op['sequential']:
                        op.update(status='position_settling',handle=None,started=time.monotonic(),timeout=15.)
                        self.zero_motion();return
                    if self.precision_arrival and op['kind']=='waypoint':
                        op.update(status='nav_settling',handle=None,started=time.monotonic(),timeout=40.)
                        self.zero_motion()
                        return
                    if p is None or math.hypot(p[0]-g['x'],p[1]-g['y']) > op['xy_tolerance']+.02:
                        raise ValueError('导航报告到达，但地图位置不符，请检查定位')
                    turn = abs(math.atan2(math.sin(p[2]-g['yaw']),math.cos(p[2]-g['yaw'])))
                    if turn > op['yaw_tolerance']+.02:
                        raise ValueError('到达朝向不符，请检查定位')
                    op['status'] = 'succeeded'
                else:
                    if op['sequential']:
                        # A native progress failure near the goal is not by
                        # itself proof the sampling point is unusable. Stop,
                        # inspect real pose, and permit one bounded recovery.
                        op.update(status='position_settling',handle=None,started=time.monotonic(),timeout=15.)
                        self.zero_motion();return
                    raise ValueError('导航未能完成，已停止当前任务')
            except Exception as e:
                op['status'] = 'failed';op['error'] = str(e)
            if self.active == identity:
                self.path = []

    def operation(self, identity):
        with self.lock:
            op = self.ops[identity]
            return {'status': op['status'], 'error': op.get('error'),
                    'quality':op.get('quality'),'position_error':op.get('final_position_error'),
                    'yaw_error_deg':op.get('final_yaw_error_deg'),
                    'stage':('waiting' if op['status']=='recovery_wait' else
                             'terminal' if self.joint_terminal and op['status'] in ('heading_configuring','heading_wait','heading','heading_settling') else
                             'heading' if op['status'] in ('heading_configuring','heading_wait','heading','heading_settling') else 'position'),
                    'detail':op.get('recovery_message')}

    def request_speed(self,speed,controller_id='FollowPath'):
        cap=getattr(self,'survey_speed_limit',None)
        if cap is not None:speed=min(speed,cap)
        # Use DWB's own dynamic limits, preserving angular authority at low vx.
        last=self.speed_limits.get(controller_id)
        if (last is not None and abs(speed-last)<.001) or self.speed_update is not None:
            return
        if not self.controller_parameters.service_is_ready():
            return
        request=SetParametersAtomically.Request()
        if controller_id=='PointPosition' and self.position_is_rpp:
            request.parameters=[Parameter(controller_id+'.desired_linear_vel',value=float(speed)).to_parameter_msg()]
        else:
            request.parameters=[Parameter(k,value=float(speed)).to_parameter_msg()
                                for k in (controller_id+'.max_vel_x',controller_id+'.max_speed_xy')]
        # This bundled DWB version refreshes max_speed_xy_sq_ in the
        # min_speed_theta branch, not in max_speed_xy's branch. Reapply its
        # configured value last in the same atomic request; no source patch.
        if not (controller_id=='PointPosition' and self.position_is_rpp):
            request.parameters.append(Parameter(controller_id+'.min_speed_theta',value=float(self.minimum_angular_speed)).to_parameter_msg())
        self.speed_update=self.controller_parameters.call_async(request)
        def done(future):
            with self.lock:
                try:
                    result=future.result().result
                    if result.successful:self.speed_limit=speed;self.speed_limits[controller_id]=speed
                    else:self.get_logger().error('原生限速更新失败：'+result.reason)
                except Exception as e:self.get_logger().error('原生限速服务异常：'+str(e))
                self.speed_update=None
        self.speed_update.add_done_callback(done)

    def finish_waypoint_position(self,op,now):
        if op.get('kind')=='survey':
            self.zero_motion()
            if not self.pose or not self.stationary_since or now-self.stationary_since<.7:return
            distance=math.hypot(self.pose[0]-op['goal']['x'],self.pose[1]-op['goal']['y'])
            if distance<=op['goal'].get('survey_pass_radius',.5):
                op.update(status='succeeded',quality='transit',finished_at=now,actual_pose=list(self.pose))
            else:self.retry_arrival(op,'引导点尚未到达，继续通道导航')
            return
        self.zero_motion()
        if not self.pose or not self.stationary_since or now-self.stationary_since<.7:return
        goal=op['goal'];distance=math.hypot(self.pose[0]-goal['x'],self.pose[1]-goal['y'])
        error=math.atan2(math.sin(goal['yaw']-self.pose[2]),math.cos(goal['yaw']-self.pose[2]))
        stage=refinement_stage(distance,error,
            position_tolerance=self.arrival.get('final_position_tolerance',.05),yaw_tolerance=op['yaw_tolerance'])
        if stage=='position' and not (self.joint_terminal and distance<=.20):
            self.retry_arrival(op,'等待目标和路线可通行，继续修正位置');return
        if stage=='complete':
            self.complete_waypoint(op,now,distance,error);return
        self.begin_waypoint_heading(op)

    def begin_waypoint_heading(self,op):
        if not self.follow.server_is_ready():
            op.update(status='failed',error='原生朝向跟踪动作未就绪');self.zero_motion();return
        error=math.atan2(math.sin(op['goal']['yaw']-self.pose[2]),math.cos(op['goal']['yaw']-self.pose[2]))
        profile=heading_profile(error,op.get('previous_heading_error'),op.get('previous_heading_limit'))
        if self.joint_terminal and math.hypot(self.pose[0]-op['goal']['x'],self.pose[1]-op['goal']['y'])>.04:
            profile['max_vel_theta']=.20  # Permit a small corrective arc for lateral error.
        op.update(previous_heading_error=error,previous_heading_limit=profile['max_vel_theta'],
                  status='heading_configuring',handle=None,started=time.monotonic(),
                  timeout=float(self.arrival.get('heading_timeout',30.)),
                  heading_travel=0.,heading_budget=abs(error)+math.radians(45))
        request=SetParametersAtomically.Request()
        request.parameters=([Parameter('PointPose.wz_max',value=float(profile['max_vel_theta'])).to_parameter_msg()]
                            if self.joint_terminal else
                            [Parameter('PointHeading.'+k,value=float(v)).to_parameter_msg() for k,v in profile.items()])
        self.controller_parameters.call_async(request).add_done_callback(
            lambda f,identity=self.active:self.heading_configured(f,identity))

    def heading_configured(self,future,identity):
        with self.lock:
            op=self.ops[identity]
            if op['cancel']:op['status']='canceled';return
            try:
                result=future.result().result
                if not result.successful:raise ValueError('朝向速度配置失败：'+result.reason)
                self.send_waypoint_heading(op,identity)
            except Exception as e:op.update(status='failed',error=str(e));self.zero_motion()

    def send_waypoint_heading(self,op,identity):
        op['heading_attempts']+=1;op['status']='heading_wait'
        goal=FollowPath.Goal();goal.controller_id='PointPose' if self.joint_terminal else 'PointHeading'
        goal.goal_checker_id='point_pose_checker' if self.joint_terminal else 'point_heading_checker'
        path=NavPath();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
        start={'x':self.pose[0],'y':self.pose[1],'yaw':self.pose[2]}
        path.poses=[self.stamped(start),self.stamped(op['goal'])];goal.path=path
        self.path=[[start['x'],start['y']],[op['goal']['x'],op['goal']['y']]]
        self.follow.send_goal_async(goal).add_done_callback(lambda f:self.heading_accepted(f,identity))

    def heading_accepted(self,future,identity):
        with self.lock:
            op=self.ops[identity]
            try:
                handle=future.result()
                if not handle.accepted:raise ValueError('原生朝向跟踪拒绝目标')
                op['handle']=handle
                handle.get_result_async().add_done_callback(lambda f:self.heading_done(f,identity))
                if op['cancel']:handle.cancel_goal_async()
                else:op['status']='heading'
            except Exception as e:op.update(status='failed',error=str(e));self.zero_motion()

    def heading_done(self,future,identity):
        with self.lock:
            op=self.ops[identity]
            try:
                result=future.result()
                if op['cancel']:self.finish_native_cancellation(op)
                else:
                    # Re-evaluate the stopped pose even after native progress
                    # failure; never turn a user cancellation into recovery.
                    op.update(status='heading_settling',handle=None,native_heading_status=result.status)
            except Exception as e:op.update(status='failed',error=str(e))
            self.zero_motion()

    def finish_native_cancellation(self,op):
        stage=op.pop('settle_after_cancel',None)
        if stage:
            op.update(status=stage,cancel=False,handle=None,started=time.monotonic(),timeout=15.)
        else:
            op['status']='failed' if op.get('error') else 'canceled'

    def finish_waypoint_heading(self,op,now):
        self.zero_motion()
        if not self.pose or not self.stationary_since or now-self.stationary_since<.7:return
        goal=op['goal'];distance=math.hypot(self.pose[0]-goal['x'],self.pose[1]-goal['y'])
        error=math.atan2(math.sin(goal['yaw']-self.pose[2]),math.cos(goal['yaw']-self.pose[2]))
        self.finish_waypoint_position(op,now)

    def near_waypoint(self,distance,error):
        return (self.arrival.get('allow_near_arrival',False) and distance<=self.arrival.get('near_position_tolerance',.08) and
                abs(error)<=self.arrival.get('near_yaw_tolerance',math.radians(5)))

    def complete_waypoint(self,op,now,distance,error):
        quality=('normal' if distance<=self.arrival.get('final_position_tolerance',.05) and abs(error)<=op['yaw_tolerance'] else 'near')
        op.update(status='succeeded',finished_at=now,quality=quality,
                  final_position_error=distance,final_yaw_error_deg=math.degrees(abs(error)),
                  actual_pose=list(self.pose))
        self.path=[]

    def finish_arrival(self,op,now):
        self.zero_motion()
        if now-op['started']>op['timeout']:
            op.update(status='failed',error='停靠校验超时，已停止');return
        if not self.pose or not self.stationary_since or now-self.stationary_since<.7:
            return
        point=op['goal'];distance=math.hypot(self.pose[0]-point['x'],self.pose[1]-point['y'])
        if distance>op['xy_tolerance']+.015:
            self.retry_arrival(op,'停稳后位置超出精确门限，未开始停留');return
        angle=math.atan2(math.sin(point['yaw']-self.pose[2]),math.cos(point['yaw']-self.pose[2]))
        if abs(angle)<=op['yaw_tolerance']:
            op['status']='succeeded';self.path=[];return
        if not self.staged_arrival:
            self.retry_arrival(op,'原生到点后朝向仍超差，未开始停留');return
        if op['heading_attempts']>=2:
            op.update(status='failed',error='最终朝向两次调整后仍超差，已停止');return
        if not self.spin_action.server_is_ready():
            op.update(status='failed',error='原生 Spin 动作未就绪');return
        op['heading_attempts']+=1;op['status']='spin_wait'
        goal=Spin.Goal();goal.target_yaw=float(angle)
        goal.time_allowance.sec=int(max(10,abs(angle)/.2+6))
        identity=self.active
        self.spin_action.send_goal_async(goal).add_done_callback(lambda f:self.spin_accepted(f,identity))

    def retry_arrival(self,op,message):
        if op.get('kind')=='survey' and op['goal'].get('survey_optional') and op.get('arrival_retries',0)>=2:
            self.zero_motion();op.update(status='succeeded',quality='skipped_optional',finished_at=time.monotonic(),actual_pose=list(self.pose) if self.pose else None)
            return
        if op.get('kind')=='survey' and self.pose and math.hypot(self.pose[0]-op['goal']['x'],self.pose[1]-op['goal']['y'])<=op['goal'].get('survey_pass_radius',.5):
            op.update(status='position_settling',handle=None,started=time.monotonic(),timeout=15.)
            self.zero_motion();return
        if op['sequential']:
            op['arrival_retries']+=1
            previous=op.get('retry_pose');p=self.pose
            progressed=bool(previous and p and math.hypot(p[0]-previous[0],p[1]-previous[1])>.03)
            streak=0 if progressed else op.get('retry_streak',0)+1
            delay=min(10.,1.5**min(streak,6))
            op.update(status='recovery_wait',retry_at=time.monotonic()+delay,
                      started=time.monotonic(),timeout=delay+20.,handle=None,
                      retry_streak=streak,retry_pose=list(p) if p else None,recovery_message=message)
            self.zero_motion();return
        # One bounded native replan after a failed stopped-pose verification.
        if op['arrival_retries']>=1:
            op.update(status='failed',error=message);return
        op['arrival_retries']+=1
        op.update(status='navigating',started=time.monotonic(),timeout=40.,handle=None)
        self.send_arrival_retry(op)

    def send_arrival_retry(self,op):
        if op['sequential']:
            # Check a fresh native path before starting another motion action.
            # A blocked sampling point remains pending, without repeated BT clears.
            op.update(status='planning',started=time.monotonic(),timeout=20.,handle=None)
            goal=ComputePathToPose.Goal();goal.goal=self.stamped(op['goal'])
            goal.planner_id='WaypointGrid';goal.use_start=False
            identity=self.active
            self.plan.send_goal_async(goal).add_done_callback(lambda f:self.plan_accepted(f,identity))
            return
        op.update(status='navigating',started=time.monotonic(),timeout=60.,handle=None)
        self.request_speed(self.arrival['creep_speed'],op['controller_id'])
        goal=NavigateToPose.Goal();goal.pose=self.stamped(op['goal'])
        goal.behavior_tree=self.native_behavior_tree
        identity=self.active
        self.nav.send_goal_async(goal).add_done_callback(lambda f:self.nav_accepted(f,identity))

    def spin_accepted(self,future,identity):
        with self.lock:
            op=self.ops[identity]
            try:
                handle=future.result()
                if not handle.accepted:raise ValueError('原生最终定向被拒绝')
                op['handle']=handle
                handle.get_result_async().add_done_callback(lambda f:self.spin_done(f,identity))
                if op['cancel']:handle.cancel_goal_async()
            except Exception as e:op.update(status='failed',error=str(e));self.zero_motion()

    def spin_done(self,future,identity):
        with self.lock:
            op=self.ops[identity]
            try:
                result=future.result()
                if op['cancel']:op['status']='canceled'
                elif result.status!=4:raise ValueError('原生最终定向未完成，已停止')
                else:op.update(status='spin_settling',handle=None)
            except Exception as e:op.update(status='failed',error=str(e))
            self.zero_motion()

    def cancel(self, identity):
        with self.lock:
            op = self.ops[identity]
            if op['status'] in ('succeeded','canceled','failed'):
                return
            op.pop('settle_after_cancel',None)  # A user stop always suppresses recovery.
            op['cancel'] = True
            if op['status'] in ('planning','configuring','heading_configuring','recovery_wait','nav_settling','spin_settling','position_settling','heading_settling'):
                op['status'] = 'canceled'  # Planning never commands wheels; callbacks suppress navigation.
            else:
                op['status'] = 'canceling'
                if op['handle']:
                    op['handle'].cancel_goal_async()
            self.zero_motion()

    def zero_motion(self):
        if self.raw_cmd is not None:
            self.raw_cmd.publish(Twist())
        self.cmd.publish(Twist())

    def set_oem_ir(self,enabled,now):
        # Match auto_recharge_ros2.Pub_Recharger_Flag: one enable burst,
        # followed by zero-velocity frames. Do not continuously restart A1.
        if not enabled:
            self.oem_ir_enabled=False
            self.ir.publish(Int8(data=0))
            return
        if not getattr(self,'oem_ir_enabled',False):
            self.oem_ir_enabled=True;self.oem_ir_started=now;self.oem_ir_last_zero=now-.1
            for _ in range(10):self.ir.publish(Int8(data=1))
        if now-self.oem_ir_started<=.5 and now-self.oem_ir_last_zero>=.095:
            self.cmd.publish(Twist());self.oem_ir_last_zero=now


    def begin_docking(self):
        with self.lock:
            self.dock_localization.arm()
            self.oem_ir_enabled=False;self.charge_motion_since=None
            self.native_docking_stage='native_capture'
            self.native_docking_watchdog.reset(time.monotonic(),self.wheel_distance)
            self.docking_error = None;self.docking_since = time.monotonic()
            self.docking_ir_lost_since=None;self.docking_last_valid_ir=None


    def stop(self):
        with self.lock:
            if not self.context.ok():
                return  # ROS already handles process shutdown; the HTTP stop precedes it.
            if self.active:
                self.cancel(self.active)
            self.docking_since = None;self.set_oem_ir(False,time.monotonic());self.zero_motion();self.path = []
            self.docking_ir_lost_since=None;self.docking_last_valid_ir=None

