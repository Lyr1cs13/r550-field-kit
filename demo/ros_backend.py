"""Bridge to native Nav2/AMCL and the factory STM32 IR docking mode."""
import math
import os
from collections import deque
from concurrent.futures import Future
from pathlib import Path
from statistics import median
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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TwistStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.action import ComputePathToPose, NavigateToPose, FollowPath, Spin
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float32, UInt8, Int8, String
from tf2_ros import Buffer, TransformListener
from dock_localization import (DOCK_MATCH_RADIUS, DockLocalization, localization_inputs_ready,
                               matches_dock, staging_scan_supported)
from arrival_policy import (FAS_STATION_POSITION_TOLERANCE, FAS_STATION_YAW_TOLERANCE,
                            RSSI_STATION_POSITION_TOLERANCE, RSSI_STATION_YAW_TOLERANCE,
                            StationCheck, refinement_stage, heading_profile, in_place_heading_profile,
                            heading_needs_brake, fas_heading_needs_brake)
from charge_confirmation import ChargeConfirmation, NativeDockingWatchdog
from directed_route import (DEPARTURE_STOP_MARGIN, DOCK_STAGING_DISTANCE, angle_error, departure_goal,
                            departure_leg, dock_footprint_intersects, dock_offset, dock_staging_needed, fas_station_leg,
                            line_poses, straight_approach_speed, straight_leg)
from turn_approach import TurnApproach, approach_radius
from nav2_msgs.srv import IsPathValid
from motion_health import DEPARTURE_SPEED, SPIN_CLEARANCE_RADIUS, LocalizationGuard, scan_points_in_base
from fleet_zone import FleetZone
from fas_car import SCAN_UNSTABLE
from fas_experiment import (MAX_FLEET_SIGMA, MIN_SCAN_AGREEMENT,
                            ScanEvidence, position_sigma, scan_confirmed, yaw_error)
from slip_monitor import SlipMonitor
from spin_control import ProfiledSpin
from guarded_retreat import RETREAT_DISTANCE, slip_obstruction
from std_srvs.srv import Empty


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def station_pivot(op):
    return bool(op.get('station_pivot') or op.get('fas_station'))


def record_stopped_turn(op,pose):
    origin=op.pop('turn_origin',None)
    if origin is not None:
        op['last_turn'].update(before=list(origin),after=list(pose),
                               centre_shift_m=math.dist(origin[:2],pose[:2]),
                               map_turn_deg=math.degrees(math.remainder(pose[2]-origin[2],2*math.pi)))
        if 'turn_started_at' in op:
            op['last_turn']['seconds']=time.monotonic()-op.pop('turn_started_at')
        if op['last_turn'].get('stage')=='heading_settling':
            op.setdefault('first_terminal_turn',dict(op['last_turn'],approach_aim=op.get('turn_aim')))
    return origin


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
        margin_checker=config['controller_server']['ros__parameters'].get('fas_turn_position_checker')
        self.turn_approach_radius=margin_checker['xy_goal_tolerance'] if margin_checker else None
        self.turn_approach=TurnApproach()
        point_controller=config['controller_server']['ros__parameters'].get('PointPosition',{})
        self.directed_lookahead=config['controller_server']['ros__parameters']['FixedHeading']['lookahead_dist']
        self.corridor_bend_lookahead=config['controller_server']['ros__parameters'].get('CorridorBend',{}).get('lookahead_dist',.16)
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
        zone_path=os.environ.get('R550_FLEET_ZONE')
        self.fleet_config=FleetZone.load(zone_path,grid.map_id) if zone_path else None
        self.fleet_latch=Path(zone_path).with_suffix('.paired') if zone_path else None
        self.fleet_zone=self.fleet_config if self.fleet_latch and self.fleet_latch.exists() else None
        self.fleet_lease_until=0.
        self.pose_stamp=0.
        self.localization_guard=LocalizationGuard()
        self.slip_monitor=SlipMonitor()
        self.encoders=None;self.slip_site=None
        self.scan_agreement=None;self.scan_known_count=0
        self.fas_scan_model=None;self.fas_scan_stamp=None;self.fas_scans=ScanEvidence()
        self.fas_localization=None;self.fas_trusted_pose=None;self.fas_saved_pose=None
        self.fas_obstruction=None
        import numpy as np
        from scipy.ndimage import distance_transform_edt
        self.wall_distances=distance_transform_edt(np.asarray(grid.cells).reshape(grid.height,grid.width)<65)*grid.resolution
        self.dock_calibrated=dock.get('calibrated',True)
        home = dock['robot_pose'];self.dock_pose = [home['x'], home['y'], home['yaw']]
        self.dock_localization=DockLocalization(self.dock_pose);self.last_amcl=0.
        self.dock_scan_job=None
        self.dock_scan_pose=None
        self.departure_distance = max(DOCK_STAGING_DISTANCE,float(dock.get('departure_distance',.30)))
        d = dock['charger_approach']
        self.dock_approach = {'x': d['p_x'], 'y': d['p_y'], 'yaw': 2*math.atan2(d['orien_z'], d['orien_w'])}
        self.pose = None;self.confirmed = False;self.last_scan = 0;self.laser = [];self.pending_scans = deque(maxlen=6)
        self.amcl_seen = False;self.covariance = None;self.map_matches = False;self.known_dock_start=False
        self.charging = None;self.last_charge = 0;self.voltage = None;self.current = None
        self.last_numeric={};self.charge_confirmation=ChargeConfirmation();self.charging_stable=False
        self.last_charge_contact=-math.inf
        self.native_docking_watchdog=NativeDockingWatchdog()
        self.native_docking_stage='native_capture';self.charge_motion_since=None
        self.ir_count = 0;self.last_ir = 0;self.docking_since = None;self.docking_error = None
        self.docking_ir_lost_since=None;self.docking_last_valid_ir=None
        self.ops = {};self.active = None;self.path = [];self.initial_request = None
        self.track = deque(maxlen=450)
        self.last_wheel = 0;self.stationary_since = None;self.wheel_distance=0.;self.wheel_yaw_rate=0.
        self.gyro=None
        self.rest_gyro=deque(maxlen=24)
        self.tf = Buffer();self.listener = TransformListener(self.tf, self)
        self.plan = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.follow = ActionClient(self, FollowPath, '/follow_path')
        self.leg_validator = self.create_client(IsPathValid, '/is_path_valid')
        self.nomotion_update=self.create_client(Empty,'/request_nomotion_update')
        self.nomotion_future=None;self.nomotion_at=0.
        self.spin_action = ActionClient(self, Spin, '/spin')
        self.controller_parameters=self.create_client(SetParametersAtomically,'/controller_server/set_parameters_atomically')
        self.survey_planner_parameters=self.create_client(SetParametersAtomically,'/planner_server/set_parameters_atomically')
        self.survey_plan_desired=None;self.survey_plan_applied=None;self.survey_plan_future=None;self.survey_plan_retry_at=0.

        self.cmd = self.create_publisher(Twist, '/collision_cmd_vel', 1)
        self.motion_permit=self.create_publisher(Bool,'/navigation_motion_permit',1)
        self.spin_limit_pub=self.create_publisher(TwistStamped,'/navigation_spin_limit',1)
        self.departure_cmd=self.create_publisher(String,'/navigation_departure_cmd',1)
        self.retreat_permit=self.create_publisher(String,'/navigation_retreat_permit',1)
        self.last_retreat_status=0.
        self.create_subscription(String,'/navigation_retreat_status',self.retreat_status_received,1)
        self.oem_lease=self.create_publisher(Bool,'/navigation_oem_active',1)
        self.last_gate=0.
        self.create_subscription(Bool,'/navigation_gate_alive',self.gate_received,1)
        self.raw_cmd=self.create_publisher(Twist,'/nav_cmd_vel',1) if self.smoothing else None
        self.ir = self.create_publisher(Int8, '/robot_recharge_flag', 10)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
        current_sensor=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(OccupancyGrid, '/map', self.map_received, q)
        self.create_subscription(NavPath, '/plan', self.path_received, 10)
        self.create_subscription(NavPath, '/plan_smoothed', self.path_received, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_received, q)
        self.create_subscription(LaserScan, '/scan', self.scan_received, current_sensor)
        self.create_subscription(Odometry, '/odom', self.wheel_received, current_sensor)
        self.create_subscription(Imu, '/imu/data_raw', self.gyro_received, current_sensor)
        self.create_subscription(Bool, '/robot_charging_flag', self.charge_received, qos_profile_sensor_data)
        self.create_subscription(UInt8, '/robot_red_flag', self.ir_received, qos_profile_sensor_data)
        self.create_subscription(Float32, '/PowerVoltage', lambda m: self.numeric_received('voltage',m), qos_profile_sensor_data)
        self.create_subscription(Float32, '/robot_charging_current', lambda m: self.numeric_received('current',m), qos_profile_sensor_data)
        self.lifecycle = {};self.inflight = set()
        self.mapping = False
        self.lifecycle_clients = {name: self.create_client(GetState, f'/{name}/get_state')
                        for name in ('amcl','map_server','controller_server','planner_server','bt_navigator','collision_monitor')+ (('velocity_smoother',) if self.smoothing else ())}
        self.create_timer(2.0, self.probe)
        self.create_timer(.1, self.update)
        self.create_timer(.05, self.publish_motion_permit)

    def gate_received(self,message):
        if message.data:self.last_gate=time.monotonic()

    def retreat_status_received(self,message):
        with self.lock:
            self.last_retreat_status=time.monotonic()
            identity,_,state=message.data.partition(':')
            if not identity or identity!=self.active:
                return
            op=self.ops[identity]
            if op['status']!='retreating':
                return
            if state=='done':
                op.update(status='succeeded',finished_at=self.last_retreat_status)
            elif state not in ('idle','backing'):
                detail={'blocked':'后方或车身净空不足，或传感器未就绪','moving':'车轮未停稳',
                        'stalled':'短退未取得有效进展','deviated':'短退时发生偏移'}
                op.update(status='failed',error=detail.get(state,'短退许可已结束'))

    def publish_motion_permit(self):
        with self.lock:
            op=self.ops[self.active] if self.active else {}
            status=op.get('status')
            health=self.snapshot()
            permitted=(status in ('navigating','heading','heading_wait','spinning','spin_wait','clearing_dock')
                       and health['ready'] and self.confirmed and not self.localization_guard.reason)
            if status=='spin_wait' and op.get('spin_resume'):
                permitted=False
            if op.get('spin_brake') and time.monotonic()-self.gyro[0]>.16:
                permitted=False
            returning=op.get('kind')=='return' or op.get('goal',{}).get('dock_retry_exit',False)
            # The seated robot pose is a physical reference even when the low
            # charging rail is invisible to the horizontal laser.
            ordinary = op.get('kind') in ('waypoint','survey','retreat')
            active_travel = status in ('navigating','heading','heading_wait','spinning','spin_wait','retreating')
            dock_blocked = False
            if ordinary and active_travel:
                dock_blocked = health['charge_contact_live']
                if dock_blocked:
                    hold = Path(__file__).resolve().parent.parent/'state'/'motion-hold'
                    if not hold.exists():
                        hold.parent.mkdir(parents=True,exist_ok=True)
                        hold.touch()
                        self.get_logger().error('普通行驶收到充电接触反馈，锁定底盘')
                    permitted=False
            if returning and health['charge_contact_live']:
                permitted=False
            if self.fleet_zone:
                permitted=permitted and health['fleet']['leased'] and not health['fleet']['motion_reason']
            fas = getattr(self,'fas_lease',None)
            if fas and fas.owner:
                kind = 'return' if returning else 'move'
                permitted = permitted and fas.allows(kind,health,self.path)
            if status=='clearing_dock':
                speed=op['departure_speed']
                if (permitted and speed and self.docking_since is None and health['pose_age_s']<=.35
                        and time.monotonic()-self.last_wheel<=.25):
                    contact=None
                    if op['departure_seated']:
                        contact=(*dock_offset(self.pose,self.dock_pose),angle_error(self.pose[2],self.dock_pose[2]))
                    stamp=self.get_clock().now().nanoseconds*1e-9
                    self.departure_cmd.publish(String(data=json.dumps([stamp,speed,contact])))
                else:
                    self.departure_cmd.publish(String())
                permitted=False
            profile=op.get('spin_brake')
            if permitted and profile and status=='spinning':
                self.publish_spin_limit(profile.signed_limit)
            else:
                self.motion_permit.publish(Bool(data=bool(permitted and not profile)))
                self.publish_spin_limit(0.)
            retreat=(status=='retreating' and not dock_blocked and health['ready'] and self.confirmed and not self.fleet_zone
                     and not self.localization_guard.reason and self.docking_since is None)
            if fas and fas.owner:
                retreat=retreat and fas.allows('retreat',health,self.path)
            self.retreat_permit.publish(String(data=self.active if retreat else ''))
            oem = self.docking_since is not None and self.fleet_zone is None
            if fas and fas.owner:
                oem = oem and fas.allows('return',health)
            self.oem_lease.publish(Bool(data=oem))

    def publish_spin_limit(self, limit):
        message=TwistStamped()
        message.header.stamp=self.get_clock().now().to_msg()
        message.twist.angular.z=float(limit)
        self.spin_limit_pub.publish(message)

    def retreat_path(self, *, fas=False):
        with self.lock:
            health=self.snapshot()
            obstruction=self.fas_obstruction
            if fas and (not obstruction or obstruction['direction']!='front' or obstruction['attempted']):
                raise ValueError('当前没有可自动短退的前向空转')
            if (self.fleet_zone or not self.confirmed or not health['ready'] or not health['stationary']
                    or health['charge_contact_live'] or self.charging_stable or self.docking_since is not None
                    or not (scan_confirmed(health) if fas else (self.scan_agreement or 0.)>=.80)
                    or health['voltage'] is None or health['voltage']<10.65
                    or time.monotonic()-self.last_retreat_status>.35):
                raise ValueError('短退需要停车、有效定位和车尾保护')
            if self.active and self.ops[self.active]['status'] not in ('succeeded','failed','canceled'):
                raise ValueError('旧导航尚未停止')
            x,y,heading=self.pose
            path=[[x,y]]
            for distance in (.05,.10,.15,RETREAT_DISTANCE):
                point=[x-distance*math.cos(heading),y-distance*math.sin(heading)]
                if self.dock_calibrated and dock_footprint_intersects((*point,heading),self.dock_pose):
                    raise ValueError('短退时车身会碰到充电桩')
                self.grid.validate(*point,clearance=.14)
                path.append(point)
            return path

    def retreat(self, *, fas=False):
        with self.lock:
            path=self.retreat_path(fas=fas)
            if fas:
                if not self.fas_lease.allows('retreat',self.snapshot(),path):
                    raise ValueError('短退的走廊或停车车辆净空未通过复核')
                self.fas_obstruction['attempted']=True
            target=dict(zip(('x','y','yaw'),(*path[-1],self.pose[2])))
            identity=uuid.uuid4().hex
            self.ops[identity]={'kind':'retreat','goal':target,'status':'retreating','controller_id':'RearGuard',
                                'started':time.monotonic(),'timeout':9.,'cancel':False,'handle':None,
                                'heading_attempts':0,'arrival_retries':0}
            self.active=identity;self.path=path
            return identity

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
            if (op['status']!='navigating' or op.get('explicit_path') or
                    (op.get('route_path') and not op.get('route_rejoining'))):
                return
            stamp=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
            if stamp<op.get('path_stamp',0):
                return
            end=m.poses[-1].pose.position;goal=op['goal']
            if math.hypot(end.x-goal['x'],end.y-goal['y'])<=max(.08,op.get('xy_tolerance',0.)+.045):
                op['path_stamp']=stamp
                self.path=[[p.pose.position.x,p.pose.position.y] for p in m.poses
                           if math.isfinite(p.pose.position.x) and math.isfinite(p.pose.position.y)]

    def charge_received(self, m):
        with self.lock:
            self.charging = bool(m.data);self.last_charge = time.monotonic()
            if self.charging:self.last_charge_contact=self.last_charge

    def numeric_received(self, name, m):
        with self.lock:
            setattr(self,name,float(m.data) if math.isfinite(m.data) else None)
            self.last_numeric[name]=time.monotonic()
            if name=='current' and math.isfinite(m.data) and m.data>=self.charge_confirmation.minimum_current:
                self.last_charge_contact=self.last_numeric[name]

    def wheel_received(self, m):
        with self.lock:
            now=time.monotonic();dt=min(.2,max(0.,now-self.last_wheel));self.last_wheel=now
            t=m.twist.twist
            self.wheel_yaw_rate=t.angular.z if math.isfinite(t.angular.z) else 0.
            p=m.pose.pose
            encoders=(p.position.x,p.position.y,yaw(p.orientation))
            self.encoders=encoders if all(math.isfinite(v) for v in encoders) else None
            if math.isfinite(t.linear.x) and math.isfinite(t.linear.y):
                self.wheel_distance+=math.hypot(t.linear.x,t.linear.y)*dt
            if self.active:
                op=self.ops[self.active]
                if op['status'] in ('heading','heading_wait','spinning','heading_settling') and math.isfinite(t.angular.z):
                    op['heading_travel']=op.get('heading_travel',0.)+abs(t.angular.z)*dt
            stopped=all(math.isfinite(v) and abs(v)<limit for v,limit in
                        ((t.linear.x,.02),(t.linear.y,.02),(t.angular.z,.05)))
            self.stationary_since=(self.stationary_since or now) if stopped else None
            if self.active:
                op=self.ops[self.active]
                if (station_pivot(op) and op['status'] in ('heading','heading_wait','spinning') and self.pose
                        and self.should_brake_heading(op,now)):
                    self.settle_waypoint_heading(op)

    def ir_received(self, m):
        with self.lock:
            self.ir_count = int(m.data);self.last_ir = time.monotonic()

    def gyro_received(self,m):
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        age=self.get_clock().now().nanoseconds*1e-9-stamp
        rate=m.angular_velocity.z
        if not -.05<=age<=.20 or not math.isfinite(rate) or abs(rate)>2.:
            return
        with self.lock:
            if self.gyro and stamp<=self.gyro[1]:
                return
            self.gyro=(time.monotonic(),stamp,rate)
            op=self.ops[self.active] if self.active else None
            if self.stationary_since is None:
                self.rest_gyro.clear()
            elif abs(rate)<.04 and (not op or op['status'] not in ('spin_wait','spinning','canceling')):
                self.rest_gyro.append((stamp,rate))
            if op and op['status']=='spinning' and op.get('spin_brake') and op['spin_brake'].observe(stamp,rate):
                self.settle_waypoint_heading(op)

    def scan_received(self, m):
        with self.lock:
            stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            if abs(self.get_clock().now().nanoseconds*1e-9-stamp)>.5:return
            self.last_scan = time.monotonic()
            self.pending_scans.append(m)

    def update_laser(self):
        # Keep the original scan-time transform. After a delayed timer, retain
        # several independent frames instead of dropping all but the newest.
        with self.lock:
            if not self.pending_scans:
                return
            fas=getattr(self,'fas_lease',None)
            matched=[]
            now=time.time()
            for m in reversed(self.pending_scans):
                stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
                if now-stamp>.80 or (self.fas_scan_stamp is not None and stamp<=self.fas_scan_stamp):
                    continue
                try:
                    t=self.tf.lookup_transform('map',m.header.frame_id,Time.from_msg(m.header.stamp)).transform
                except Exception:
                    continue
                matched.append((stamp,m,t))
                if len(matched)==(3 if fas else 1):
                    break
            if not matched and time.monotonic()-self.last_scan>2.:
                self.laser=[];self.pending_scans.clear()
            for stamp,m,t in reversed(matched):
                angle = yaw(t.rotation);points = []
                for i in range(0, len(m.ranges), max(1, len(m.ranges)//320)):
                    distance = m.ranges[i]
                    if math.isfinite(distance) and m.range_min <= distance <= min(m.range_max, 12):
                        a = angle+m.angle_min+i*m.angle_increment
                        points.append([t.translation.x+distance*math.cos(a), t.translation.y+distance*math.sin(a)])
                self.laser = points  # Display only; native localization/navigation gets the full scan.
                known=[]
                for px,py in points:
                    ix,iy=self.grid.cell_xy(px,py)
                    if self.grid.value(ix,iy)>=0:
                        known.append(self.wall_distances[iy,ix]<=.15)
                self.scan_known_count=len(known)
                self.scan_agreement=sum(known)/len(known) if known else None
                self.fas_scan_stamp=stamp
                if fas:
                    verified=False
                    if self.map_matches and len(points)>=75 and (self.scan_agreement or 0.)<MIN_SCAN_AGREEMENT:
                        from stationary_alignment import StationaryAligner
                        if self.fas_scan_model is None:
                            self.fas_scan_model=StationaryAligner(self.grid)
                        verified=self.fas_scan_model.verifies_tracking(points,(t.translation.x,t.translation.y))
                    self.fas_scans.add(stamp,self.scan_agreement or 0.,verified,
                                       (t.translation.x,t.translation.y,angle))

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

    def detect_wheel_slip(self,now,fixed_edges=False):
        op=self.ops[self.active] if self.active else None
        # Spin already checks its swept footprint and uses angular feedback for braking.
        # A translation reference must not span a stopped turn or its wheel deadband.
        translating = op and (op['status'] in ('navigating','retreating','clearing_dock')
                              or (op['status']=='heading' and op.get('heading_controller')=='PointPose'))
        if (not translating
                or not self.confirmed or not self.pose
                or self.encoders is None or now-self.last_wheel>.30 or now-self.last_scan>.35
                or not self.pending_scans):
            self.slip_monitor.reset()
            return
        fas=getattr(self,'fas_lease',None)
        if self.slip_monitor.reference is None and not fixed_edges and (self.scan_agreement or 0.)<.75:
            return
        scan=self.pending_scans[-1]
        stamp=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
        anchor=self.slip_monitor.observe(stamp,self.encoders,self.pose,scan)
        if anchor is None:
            return
        previous=self.slip_site
        repeated=previous is not None and math.dist(anchor[:2],previous[:2])<.20
        self.slip_site=anchor
        if (fas and fas.owner and fas.recovery_enabled and op['status']!='heading'
                and (not repeated or self.fas_obstruction is None)):
            self.fas_obstruction=slip_obstruction(anchor,self.slip_monitor.reference[1],self.encoders)
        self.localization_guard.trip_slip(anchor,repeated=repeated)
        self.get_logger().warning(self.localization_guard.fault)

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
                self.pose_stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
                if abs(time.time()-self.pose_stamp) > 2:
                    raise ValueError('stale transform')
                pose = [t.translation.x, t.translation.y, yaw(t.rotation)]
                if not all(math.isfinite(v) for v in pose):
                    raise ValueError('nonfinite transform')
                self.pose = pose
                self.pose_observed_at = now
            except Exception:
                self.pose = None
            moving=bool(self.active and self.ops[self.active]['status'] not in ('succeeded','failed','canceled'))
            fas=getattr(self,'fas_lease',None)
            fixed_edges=bool(fas and fas.owner and self.fas_scans.latest(time.time())[1])
            had_fault=bool(self.localization_guard.fault)
            if not had_fault:self.detect_wheel_slip(now,fixed_edges)
            self.localization_guard.update(now,moving=moving,covariance=self.covariance,
                scan_fraction=self.scan_agreement,scan_count=self.scan_known_count,fixed_edges=fixed_edges)
            if self.localization_guard.fault:
                if not had_fault:self.guard_was_confirmed=self.confirmed
                self.confirmed=False
                if self.stationary_since is not None:
                    self.request_stationary_amcl(now)
                recovered=self.localization_guard.recover(now,
                    stationary=self.stationary_since is not None and now-self.stationary_since>=.7,
                    covariance=self.covariance,scan_fraction=self.scan_agreement,scan_count=self.scan_known_count,
                    pose=self.pose,fresh=now-self.last_scan<.3 and now-self.last_wheel<.3 and now-self.last_amcl<1.5,
                    fixed_edges=fixed_edges)
                if recovered:self.confirmed=getattr(self,'guard_was_confirmed',False)
            if self.initial_request:
                message, remaining = self.initial_request
                message.header.stamp = self.get_clock().now().to_msg();self.initial.publish(message)
                self.initial_request = (message, remaining-1) if remaining > 1 else None
            if fas:
                self.update_fas_localization(now)
            if self.active:
                op = self.ops[self.active]
                self.survey_route_tick(op)
                if op['status'] in ('succeeded','failed','canceled'):op.setdefault('finished_at',now)
                if op['status'] == 'canceling':
                    self.zero_motion()
                elif ((op['status'] in ('heading','heading_wait') or
                       (op.get('spin_resume') and op['status']=='spinning')) and self.pose and
                      ((self.stationary_since is not None and now-self.stationary_since>=.7
                        and self.heading_needs_settle(op)) or self.should_brake_heading(op,now))):
                    self.settle_waypoint_heading(op)
                elif (op['status']=='spinning' and op.get('spin_brake') and op['spin_brake'].moving
                      and now-self.gyro[0]>.20):
                    self.settle_waypoint_heading(op)
                elif (op.get('heading_controller') in ('PointHeading','Spin')
                      and op['status'] in ('heading','heading_wait','spinning')
                      and op.get('heading_travel',0.)>op['heading_budget']):
                    self.settle_waypoint_heading(op)
                elif op['status']=='clearing_dock':
                    self.update_departure(op,now)
                elif op['status'] not in ('succeeded','canceled','failed') and now-op['started'] > op['timeout']:
                    if station_pivot(op) and op['status'] in ('line_alignment','line_settling','position_settling','heading_settling'):
                        op.update(started=now,timeout=15.,recovery_message='停车等待新定位，保留当前采集点')
                        self.zero_motion()
                    elif op.get('kind')=='return' and op['status']=='navigating':
                        op.update(cancel=True,status='canceling',settle_after_cancel='transit_recovery')
                        if op['handle']:op['handle'].cancel_goal_async()
                        self.zero_motion()
                    elif op['sequential'] and op['status'] in ('navigating','heading','heading_wait','spinning','spin_wait'):
                        # End the native action before checking usable arrival.
                        # An elapsed refinement timer is not a user stop.
                        stage=op.get('spin_resume') or ('position_settling' if op['status']=='navigating' else 'heading_settling')
                        op.update(cancel=True,status='canceling',settle_after_cancel=stage)
                        if op['handle']:op['handle'].cancel_goal_async()
                        self.zero_motion()
                    else:
                        op['error'] = '导航等待超时，已请求停车';self.cancel(self.active)
                elif op['status']=='position_settling':
                    self.finish_waypoint_position(op,now)
                elif op['status']=='heading_settling':
                    self.finish_waypoint_heading(op,now)
                elif op['status'] in ('line_alignment','line_settling'):
                    self.update_line_alignment(op,now)
                elif op['status']=='recovery_wait' and now>=op['retry_at']:
                    self.send_arrival_retry(op)
                elif self.precision_arrival and op['status'] in ('nav_settling','spin_settling'):
                    self.finish_arrival(op,now)
                elif self.precision_arrival and op['status']=='navigating' and self.pose:
                    distance=math.hypot(self.pose[0]-op['goal']['x'],self.pose[1]-op['goal']['y'])
                    if self.stop_at_waypoint(op,distance,now):
                        return
                    speed=(self.arrival['creep_speed'] if distance<self.arrival['creep_distance'] else
                           self.arrival['approach_speed'] if distance<self.arrival['approach_distance'] else self.arrival['nominal_speed'])
                    if op['controller_id']=='FixedHeading':
                        speed=straight_approach_speed(distance,reverse=bool(op.get('reverse')))
                    # Return is a transit leg; do not apply the waypoint creep
                    # or terminal precision loop before the OEM IR handoff.
                    if op.get('route_path') and not op.get('route_rejoining'):speed=op.get('track_speed',.08)
                    if op['kind']=='return':speed=.10 if distance<.35 else self.arrival['nominal_speed']
                    self.request_speed(speed,op['controller_id'])
                if (self.pose and op['status'] in ('navigating','position_settling','heading','spinning','heading_settling')
                        and not self.localization_guard.fault
                        and (not self.track or math.dist(self.track[-1],self.pose[:2])>=.02)):
                    self.track.append(list(self.pose[:2]))
            if self.docking_since is not None:
                self.update_oem_docking(now,charge_current)
            if (fas and fas.owner and not moving and self.docking_since is None and self.confirmed
                    and self.stationary_since is not None and now-self.stationary_since>=1.
                    and now-self.last_scan<.5 and now-self.last_wheel<.5
                    and (fixed_edges or (self.scan_agreement or 0.)>=MIN_SCAN_AGREEMENT)
                    and self.map_matches and not self.mapping
                    and self.covariance and all(v is not None for v in self.covariance[:2])
                    and max(self.covariance[:2])>.08**2):
                self.request_stationary_amcl(now)
            self.update_dock_localization(now)

    def update_oem_docking(self,now,current):
        fas = getattr(self,'fas_lease',None)
        lease_error=fas.motion_error('return',self.snapshot()) if fas and fas.owner else None
        if lease_error:
            self.docking_error=f'双车回充已停车：{lease_error}'
            self.stop()
            return
        if self.pose is not None and abs(dock_offset(self.pose,self.dock_pose)[1])>.28:
            self.docking_error='已偏离本车桩前红外对接区，停车复核'
            self.stop()
            return
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
        if (Path(__file__).resolve().parent.parent/'state'/'motion-hold').exists():
            self.confirmed=False
            return
        if not self.dock_calibrated:return
        busy=(self.docking_since is not None or
              bool(self.active and self.ops[self.active]['status'] not in ('succeeded','failed','canceled')))
        health=self.snapshot()
        stationary=bool(now-self.last_wheel<2 and self.stationary_since is not None and now-self.stationary_since>=2)
        sensors_ready=localization_inputs_ready(now,map_matches=self.map_matches,
            mapping=self.mapping,last_scan=self.last_scan,lifecycle=self.lifecycle)
        if self.dock_scan_job and self.dock_scan_job[2].done():
            self.finish_dock_scan(stationary and sensors_ready and not busy)
        applying=self.dock_localization.status=='applying'
        pending=self.initial_request is not None or self.dock_scan_job is not None
        scan_started=self.dock_scan_job is not None or self.dock_scan_pose is not None
        if applying and not pending and not busy and stationary and sensors_ready and (self.charging_stable or scan_started):
            self.request_stationary_amcl(now)
        fit=self.dock_scan_pose
        agrees_with_fit=(fit is None or (self.pose and math.dist(fit[:2],self.pose[:2])<=.075
                          and abs(math.remainder(self.pose[2]-fit[2],2*math.pi))<=math.radians(5)))
        fas=getattr(self,'fas_lease',None)
        scan_ready=scan_confirmed(health) if fas and fas.owner else (self.scan_agreement or 0.)>=MIN_SCAN_AGREEMENT
        amcl_updated=bool(health['ready'] and scan_ready and agrees_with_fit
                          and self.last_amcl>(self.dock_localization.started or now))
        action=self.dock_localization.step(now,
            charging=self.charging_stable,
            stationary=stationary,sensors_ready=sensors_ready,
            busy=busy,pose=self.pose,amcl_updated=amcl_updated,pose_request_pending=pending,
            scan_pending=self.dock_scan_job is not None,scan_started=scan_started)
        if action=='initialize':
            self.start_dock_scan()
        elif action=='confirm' and health['ready']:
            self.confirmed=True
            self.get_logger().info('已用贴桩后的静止雷达和 AMCL 确认定位')
        elif applying and self.dock_localization.status=='attention':
            self.get_logger().warning(
                f'贴桩定位复核超时（导航={health["ready"]}，传感器={sensors_ready}，'
                f'停车={stationary}，地图匹配={self.scan_agreement}，'
                f'拟合进行中={self.dock_scan_job is not None}，新定位={amcl_updated}）')

    def request_stationary_amcl(self,now):
        if (now-self.nomotion_at>=1. and self.nomotion_update.service_is_ready()
                and (self.nomotion_future is None or self.nomotion_future.done())):
            self.nomotion_future=self.nomotion_update.call_async(Empty.Request())
            self.nomotion_at=now

    def start_dock_scan(self):
        try:
            scans,sensor=self.stationary_alignment_inputs()
        except ValueError:
            self.dock_localization.arm()
            return
        future=Future()
        self.dock_scan_job=(self.dock_localization.started,self.wheel_distance,future)
        threading.Thread(target=self.solve_dock_scan,args=(scans,sensor,future),daemon=True).start()

    def solve_dock_scan(self,scans,sensor,future):
        try:
            from stationary_alignment import StationaryAligner
            result=StationaryAligner(self.grid).fit(self.dock_pose,scans,sensor,radius=.22,heading_degrees=12.,
                                                    use_fixed_edges=True)
            future.set_result(result)
        except Exception as error:
            future.set_exception(error)

    def finish_dock_scan(self,eligible):
        started,wheel_distance,future=self.dock_scan_job
        self.dock_scan_job=None
        if self.dock_localization.status!='applying' or self.dock_localization.started!=started:
            return
        if not eligible or abs(self.wheel_distance-wheel_distance)>.01:
            self.dock_localization.arm()
            return
        try:
            result=future.result()
            if not matches_dock(result['pose'],self.dock_pose):
                raise ValueError('静止雷达位姿偏离本车桩位')
        except Exception as error:
            self.dock_localization.status='attention'
            self.get_logger().warn(f'贴桩定位仍待确认：{error}')
            return
        self.initialize_pose(result['pose'],known_dock=True,automatic=True,precise=True)
        self.dock_scan_pose=result['pose']
        self.dock_localization.started=time.monotonic()
        elapsed=self.dock_localization.started-started
        self.get_logger().info(f'贴桩静止雷达匹配 {result["wall_support"]:.0%}，用时 {elapsed:.1f} 秒，等待 AMCL 复核')

    def local_rotation_ready(self,now):
        op=self.ops[self.active] if self.active else None
        if not op or op['status']!='spinning' or not op.get('spin_anchor') or self.pose is None or self.encoders is None:
            return False
        wheels,position=op['spin_anchor']
        return (now-op['started']<=8. and now-self.last_scan<=.35 and now-self.last_wheel<=.18
                and self.gyro is not None and now-self.gyro[0]<=.16
                and math.dist(wheels,self.encoders[:2])<=.035 and math.dist(position,self.pose[:2])<=.075)

    def snapshot(self):
        with self.lock:
            now = time.monotonic();reason = ''
            arrival=None
            if self.active:
                op=self.ops[self.active];g=op['goal'];p=self.pose
                arrival={'kind':op['kind'],'status':op['status'],
                         'quality':op.get('quality'),'controller':op.get('controller_id'),
                         'heading_controller':op.get('heading_controller'),
                         'spin_resume':op.get('spin_resume'),
                         'yaw_tolerance_deg':math.degrees(op['yaw_tolerance']) if 'yaw_tolerance' in op else None,
                         'position_tolerance_m':op.get('stop_xy_tolerance',op.get('xy_tolerance')),
                         'heading_attempts':op['heading_attempts'],'position_retries':op['arrival_retries'],
                         'last_turn':op.get('last_turn'),
                         'first_terminal_turn':op.get('first_terminal_turn') if op['heading_attempts']>1 else None,
                         'approach_aim':op.get('turn_aim'),
                         'travel_direction':op.get('travel_direction'),
                         'heading_refinement_seconds':((op.get('finished_at') or now)-op['first_heading_at']
                                                       if 'first_heading_at' in op else 0.),
                         'elapsed':(op.get('finished_at') or now)-op.get('created',op['started']),
                         'position_error':math.hypot(p[0]-g['x'],p[1]-g['y']) if p else None,
                         'yaw_error_deg':math.degrees(abs(math.atan2(math.sin(g['yaw']-p[2]),math.cos(g['yaw']-p[2])))) if p else None}
            states = all(self.lifecycle.get(k,(0,0))[0] == 3 and now-self.lifecycle[k][1] < 8 for k in self.lifecycle_clients)
            if (Path(__file__).resolve().parent.parent/'state'/'motion-hold').exists():
                reason = '停车锁定：请复核实车、定位和桩位'
            elif self.mapping:
                reason = '建图进程仍在运行，请先关闭建图，再启动演示'
            elif not self.map_matches:
                reason = '实际导航地图与演示版本不一致或尚未加载'
            elif not states or not self.nav.server_is_ready() or not self.plan.server_is_ready():
                reason = '正在等待导航与定位启动'
            elif now-self.last_scan > 2:
                reason = '雷达数据中断，不能开始导航'
            elif now-self.last_wheel > 2:
                reason = '底盘运动反馈中断，不能开始导航'
            elif now-self.last_gate > .5:
                reason = '速度保护节点心跳中断，保持停车'
            elif self.pose is None or not self.amcl_seen:
                reason = '请在地图上设置当前位置和车头朝向'
            elif self.localization_guard.fault:
                reason = self.localization_guard.fault
            fleet=None
            if self.fleet_config:
                status=self.ops[self.active]['status'] if self.active else ''
                route=self.path if self.fleet_zone and status=='navigating' else ()
                fleet={'robot_id':self.fleet_config.robot_id,'bounds':self.fleet_config.bounds,
                       'active':self.fleet_zone is not None,'leased':now<self.fleet_lease_until,
                       'motion_reason':self.fleet_motion_reason(route)}
            feedback_fresh=now-self.last_charge<2
            current_fresh=now-self.last_numeric.get('current',0)<2 and self.current is not None
            contact=bool(now-self.last_charge_contact<1.5 or (feedback_fresh and self.charging) or
                         (current_fresh and self.current>=self.charge_confirmation.minimum_current))
            scan_window,structure=self.fas_scans.latest(time.time())
            # Handoff already checked the stopped approach. During IR control the
            # nearby dock changes scan similarity but live sensor and pose checks still apply.
            local_docking=bool(self.docking_since is not None and self.pose
                and now-self.last_scan<=.35 and now-self.last_wheel<=.30 and now-self.last_ir<=.6
                and -.14<=dock_offset(self.pose,self.dock_pose)[0]<=.90
                and abs(dock_offset(self.pose,self.dock_pose)[1])<=.28)
            local_staging=bool(self.dock_calibrated and arrival and arrival['kind']=='return'
                and arrival['status'] not in ('canceled','failed')
                and now-self.last_scan<=.35 and now-self.last_wheel<=.30
                and staging_scan_supported(self.pose,(self.dock_approach['x'],self.dock_approach['y']),
                                           self.dock_pose,scan_window,self.covariance))
            return {'mode': 'robot', 'ready': not reason, 'readiness': reason or '导航与雷达已就绪',
                    'localization_confirmed': self.confirmed, 'pose': self.pose,
                    'pose_age_s':abs(time.time()-self.pose_stamp) if self.pose is not None else None,
                    'dock_calibrated':self.dock_calibrated,'fleet':fleet,
                    'dock_position_confirmed':self.known_dock_start and self.pose is not None and math.hypot(self.pose[0]-self.dock_pose[0],self.pose[1]-self.dock_pose[1])<.15 and abs(math.atan2(math.sin(self.pose[2]-self.dock_pose[2]),math.cos(self.pose[2]-self.dock_pose[2])))<.25,
                    'dock_departure_required':bool(self.dock_calibrated and self.pose
                        and (dock_footprint_intersects(self.pose,self.dock_pose) or (
                            self.known_dock_start and dock_staging_needed(self.pose,self.dock_pose,self.departure_distance)))),
                    'charging': self.charging_stable or (self.charging if now-self.last_charge < 2 else None),
                    'charging_feedback_flag': self.charging if now-self.last_charge < 2 else None,
                    'charge_contact_live':contact,'charge_telemetry_fresh':current_fresh and feedback_fresh,
                    'charging_stable': self.charging_stable,
                    'charging_elapsed_s': (now-self.charge_confirmation.since
                                           if self.charging_stable and self.charge_confirmation.since is not None else None),
                    'docking_contact_stage': self.native_docking_stage if self.docking_since is not None else None,
                    'voltage': self.voltage if now-self.last_numeric.get('voltage',0)<2 else None, 'current': self.current,
                    'motion_fault':self.localization_guard.fault,
                    'scan_map_agreement':self.scan_agreement,
                    'scan_window_agreement':scan_window,'scan_structure_verified':structure,
                    'fas_local_rotation':self.local_rotation_ready(now),
                    'fas_dock_staging':local_staging,
                    'fas_local_docking':local_docking,
                    'fas_localization_active':self.fas_localization is not None,
                    'fas_obstruction':self.fas_obstruction,
                    'ir': self.ir_count if now-self.last_ir < 2 else 0,
                    'stationary': now-self.last_wheel<.3 and self.stationary_since is not None and now-self.stationary_since>=.5,
                    'path': self.path, 'track':list(self.track), 'laser': self.laser, 'map_matches': self.map_matches,
                    'localization_covariance': self.covariance,'navigation_profile':self.profile,'arrival':arrival,
                    'dock_localization':self.dock_localization.snapshot()}

    def tick(self):
        pass

    def fleet_motion_reason(self,path):
        return self.fleet_config.motion_reason(self.pose,abs(time.time()-self.pose_stamp),
                                              self.covariance,self.scan_agreement,path)

    def set_fleet_mode(self, active):
        with self.lock:
            if not self.fleet_config:
                raise ValueError('当前入口未配置车辆运行区')
            health=self.snapshot()
            if not health['stationary'] or self.docking_since is not None:
                raise ValueError('车辆停稳后才能切换运行模式')
            if self.active and self.ops[self.active]['status'] not in ('succeeded','failed','canceled'):
                raise ValueError('请先结束当前导航任务')
            if active:
                if not health['ready'] or not health['localization_confirmed']:
                    raise ValueError('请先核对车辆定位')
                reason=self.fleet_motion_reason(())
                if reason:
                    raise ValueError(reason)
            try:
                if active:
                    self.fleet_latch.touch()
                else:
                    self.fleet_latch.unlink(missing_ok=True)
            except OSError as error:
                raise ValueError('无法保存车辆运行模式') from error
            self.fleet_zone=self.fleet_config if active else None
            self.fleet_lease_until=0.

    def renew_fleet_lease(self):
        with self.lock:
            if not self.fleet_zone or self.fleet_zone.bounds is None:
                raise ValueError('车端双车运行区尚未配置')
            self.fleet_lease_until=time.monotonic()+1.

    def stationary_alignment_inputs(self):
        scans=[{'angle_min':m.angle_min,'angle_increment':m.angle_increment,'range_min':m.range_min,
                'range_max':m.range_max,'ranges':list(m.ranges)} for m in self.pending_scans]
        if len(scans)<4:
            raise ValueError('等待连续的静止雷达帧')
        try:
            sensor=self.tf.lookup_transform('base_footprint',self.pending_scans[-1].header.frame_id,Time()).transform
        except Exception as exc:
            raise ValueError('暂未取得雷达的车体坐标') from exc
        return scans,(sensor.translation.x,sensor.translation.y,yaw(sensor.rotation))

    def fas_localization_reference(self):
        if self.localization_guard.anchor is not None:
            return self.localization_guard.anchor
        if self.confirmed and self.pose is not None and abs(time.time()-self.pose_stamp)<=.45:
            return self.pose
        if self.fas_saved_pose and self.amcl_seen and self.pose is not None and abs(time.time()-self.pose_stamp)<=.45:
            reference,stamp=self.fas_saved_pose
            if (0.<=time.time()-stamp<120. and math.dist(reference[:2],self.pose[:2])<=.18
                    and yaw_error(reference[2],self.pose[2])<=math.radians(12)):
                return reference
        return self.fas_trusted_pose

    def fas_localization_state(self):
        if self.fas_localization:
            return 'verifying' if 'fit' in self.fas_localization else 'fitting'
        return 'available' if self.fas_localization_reference() is not None else None

    def fas_still(self, now, owner):
        return (self.fas_lease.owner==owner and not self.fas_lease.live and self.docking_since is None
                and not self.mapping and self.map_matches and self.encoders is not None
                and self.stationary_since is not None and now-self.stationary_since>=.8
                and (not self.active or self.ops[self.active]['status'] in ('succeeded','failed','canceled')))

    def start_fas_localization(self, owner):
        with self.lock:
            if self.fas_localization:
                return
            now=time.monotonic()
            reference=self.fas_localization_reference()
            if (reference is None or not self.fas_still(now,owner) or now-self.last_scan>=.35
                    or now-self.last_wheel>=.35 or self.initial_request or self.dock_scan_job):
                raise ValueError('等待停稳、鲜活雷达和可信的局部定位参考')
            scans,sensor=self.stationary_alignment_inputs()
            if self.fas_trusted_pose is None:
                self.fas_trusted_pose=tuple(reference)
            future=Future()
            self.fas_localization={'owner':owner,'wheels':self.encoders,'deadline':now+14.,
                                   'future':future}
            threading.Thread(target=self.solve_fas_scan,args=(tuple(reference),scans,sensor,future),daemon=True).start()

    def solve_fas_scan(self, reference, scans, sensor, future):
        try:
            from stationary_alignment import StationaryAligner
            model=StationaryAligner(self.grid)
            fit=model.fit(reference,scans,sensor,radius=.17,heading_degrees=10.,use_fixed_edges=True)
            if ((not fit['fixed_edges'] and fit['wall_support']<.64) or math.dist(fit['pose'][:2],reference[:2])>.22
                    or yaw_error(fit['pose'][2],reference[2])>math.radians(10)):
                raise ValueError('固定墙线支持不足或结果偏离局部参考')
            future.set_result(fit['pose'])
        except Exception as error:
            future.set_exception(error)

    def cancel_fas_localization(self):
        self.fas_localization=None

    def update_fas_localization(self, now):
        job=self.fas_localization
        if job is None:
            if (self.confirmed and not self.localization_guard.reason and self.pose is not None
                    and abs(time.time()-self.pose_stamp)<=.45 and scan_confirmed(self.snapshot())):
                self.fas_trusted_pose=tuple(self.pose)
            return
        if (now>=job['deadline'] or not self.fas_still(now,job['owner'])
                or math.dist(job['wheels'][:2],self.encoders[:2])>.012
                or yaw_error(job['wheels'][2],self.encoders[2])>math.radians(1.5)):
            self.cancel_fas_localization()
            return
        if now-self.last_scan>=.35 or now-self.last_wheel>=.35:
            return  # Stationary verification can wait; no motion is permitted during this job.
        if 'fit' not in job:
            if not job['future'].done():
                return
            try:
                fit=job['future'].result()
            except Exception as error:
                self.get_logger().warning(f'双车静止定位待复核：{error}')
                self.cancel_fas_localization()
                return
            self.initialize_pose(fit,automatic=True,precise=True)
            job.update(fit=fit,applied=now,scan_after=time.time(),deadline=now+12.,good_samples=[])
            return
        self.request_stationary_amcl(now)
        health=self.snapshot()
        sigma=position_sigma(health)
        good=(not self.initial_request and self.last_amcl>job['applied'] and health['ready']
              and self.fas_scan_stamp is not None and self.fas_scan_stamp>job['scan_after']
              and scan_confirmed(health) and health['pose_age_s']<=.45 and sigma is not None
              and sigma<=MAX_FLEET_SIGMA and math.dist(self.pose[:2],job['fit'][:2])<=.075
              and yaw_error(self.pose[2],job['fit'][2])<=math.radians(5))
        samples=job['good_samples']
        samples[:]=[(stamp,observed) for stamp,observed in samples if now-observed<=1.2]
        if not good or (samples and samples[-1][0]==self.fas_scan_stamp):
            return
        samples.append((self.fas_scan_stamp,now))
        if len(samples)>=3 and now-samples[0][1]>=.55:
            self.fas_trusted_pose=tuple(self.pose)
            self.confirmed=True
            self.cancel_fas_localization()
            self.get_logger().info('双车静止定位已通过连续雷达和 AMCL 复核')

    def refine_stationary_pose(self, hint=None):
        from stationary_alignment import StationaryAligner
        with self.lock:
            health=self.snapshot()
            if not health['stationary'] or self.mapping or not self.map_matches or time.monotonic()-self.last_scan>.5:
                raise ValueError('静止雷达精校需要停稳、底图一致且雷达实时在线')
            if hint is None and not self.pose:
                raise ValueError('请先在地图给出大致位置与朝向')
            reference=list(self.pose if hint is None else hint)
            wheel_distance=self.wheel_distance
            scans,sensor=self.stationary_alignment_inputs()
            fas=getattr(self,'fas_lease',None)
        result=StationaryAligner(self.grid).fit(reference,scans,sensor,use_fixed_edges=bool(fas and fas.recovery_enabled))
        with self.lock:
            health=self.snapshot()
            if not health['stationary'] or abs(self.wheel_distance-wheel_distance)>.01:
                raise ValueError('扫描期间小车位置变化，请停稳再校准')
            pose=result['pose']
            known_dock=(health.get('charging_stable') and self.dock_calibrated
                        and math.dist(pose[:2],self.dock_pose[:2])<=DOCK_MATCH_RADIUS
                        and abs(math.remainder(pose[2]-self.dock_pose[2],2*math.pi))<math.radians(5))
            self.initialize_pose(pose,known_dock=known_dock,precise=True)
        return result

    def initialize_pose(self, pose, known_dock=False,automatic=False,precise=False):
        with self.lock:
            self.dock_scan_pose=None
            if not automatic:
                if known_dock:self.dock_localization.manual_dock(time.monotonic())
                else:self.dock_localization.suppress()
                self.slip_site=None
                self.fas_trusted_pose=None
                self.fas_saved_pose=None
                self.fas_obstruction=None
            self.slip_monitor.reset()
            message = PoseWithCovarianceStamped();message.header.frame_id = 'map'
            message.pose.pose.position.x, message.pose.pose.position.y = pose[0], pose[1]
            message.pose.pose.orientation.z = math.sin(pose[2]/2);message.pose.pose.orientation.w = math.cos(pose[2]/2)
            spread=.05 if known_dock else .06 if precise else .5
            message.pose.covariance[0] = spread**2
            message.pose.covariance[7] = spread**2
            message.pose.covariance[35] = math.radians(5 if known_dock or precise else 15)**2
            self.confirmed = False;self.amcl_seen = False;self.initial_request = (message, 3);self.known_dock_start=known_dock
            self.fas_scans.frames.clear()
            self.localization_guard.reset()

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

    def fas_line(self, pose, point, xy_limit=FAS_STATION_POSITION_TOLERANCE):
        if not self.leg_validator.service_is_ready():
            return None
        leg=fas_station_leg(pose,point,min_distance=xy_limit)
        if not leg:
            return None
        poses=leg['poses']
        try:
            for p in poses:
                self.grid.validate(p['x'],p['y'],clearance=leg['clearance'])
        except ValueError:
            return None
        aim=self.turn_approach.aim(pose[:2],point,leg['reverse'],xy_limit,self.turn_approach_radius)
        if aim is not None:
            travel=math.atan2(aim['y']-pose[1],aim['x']-pose[0])
            body=math.remainder(travel+(math.pi if leg['reverse'] else 0.),math.tau)
            adjusted=line_poses(pose,aim,body,.04)
            try:
                for p in adjusted:
                    self.grid.validate(p['x'],p['y'],clearance=leg['clearance'])
            except ValueError:
                pass  # The nominal line above is still available for live validation.
            else:
                poses=adjusted
        route=NavPath()
        route.header.frame_id='map'
        route.header.stamp=self.get_clock().now().to_msg()
        route.poses=[self.stamped(p) for p in poses]
        return route,leg['reverse']

    def preview_route(self, point, kind, health):
        """Request a native planner path without sending a navigation action."""
        def result(future):
            complete = threading.Event()
            future.add_done_callback(lambda _:complete.set())
            if not complete.wait(3.):
                raise ValueError('原生路径预览超时')
            return future.result()

        pose = list(health['pose'])
        path = [pose[:2]]
        if kind == 'move' and (health['charging'] or health['dock_position_confirmed']
                               or health.get('dock_departure_required')):
            end=departure_goal(self.dock_pose,self.departure_distance)
            clear=self.dock_clearance_leg(pose,end)
            if clear['poses']:
                final = clear['poses'][-1]
                pose[:2] = final['x'], final['y']
                path.extend([[p['x'],p['y']] for p in clear['poses']])
        if math.dist(pose[:2], (point['x'],point['y'])) > .04:
            candidate=self.fas_line(pose,point) if kind=='move' and self.sequential_waypoints else None
            if candidate:
                line, _=candidate
                request=IsPathValid.Request()
                request.path=line
                if result(self.leg_validator.call_async(request)).is_valid:
                    return path+self.plan_coordinates(line)
            goal = ComputePathToPose.Goal()
            goal.start = self.stamped(dict(zip(('x','y','yaw'),pose)))
            goal.goal = self.stamped(point)
            goal.use_start = True
            goal.planner_id = 'WaypointGrid' if kind == 'move' else 'GridBased'
            handle = result(self.plan.send_goal_async(goal))
            if not handle.accepted:
                raise ValueError('Nav2 拒绝路径预览')
            response = result(handle.get_result_async())
            if response.status != 4 or not response.result.path.poses:
                raise ValueError('当前没有可走的原生导航路线')
            path.extend(self.plan_coordinates(response.result.path))
        path.append([point['x'],point['y']])
        if kind == 'return':
            path.append(list(self.dock_pose[:2]))
        return path

    @staticmethod
    def plan_coordinates(path):
        return [[p.pose.position.x,p.pose.position.y] for p in path.poses]

    def send_fas_line(self, identity):
        op=self.ops[identity]
        if not station_pivot(op) or op.get('direct_failed') or (self.fleet_zone and not op.get('fas_station')):
            return False
        result=self.fas_line(self.pose,op['goal'],op['stop_xy_tolerance'])
        if result is None:
            return False
        line,reverse=result
        path=self.plan_coordinates(line)
        if self.fleet_zone and self.fleet_motion_reason(path):
            return False
        fas=getattr(self,'fas_lease',None)
        if fas and fas.owner and not fas.allows('move',self.snapshot(),path):
            return False
        op.update(directed_path=line,reverse=reverse,started=time.monotonic(),timeout=20.)
        self.send_navigation(identity)
        return True

    def plan_navigation(self,identity):
        op=self.ops[identity]
        op.update(status='planning',started=time.monotonic(),timeout=20.,handle=None)
        goal=ComputePathToPose.Goal()
        goal.goal=self.stamped(op['goal'])
        goal.planner_id='WaypointGrid' if op['sequential'] else 'GridBased'
        goal.use_start=False
        self.plan.send_goal_async(goal).add_done_callback(lambda f:self.plan_accepted(f,identity))

    def navigate(self, point, kind):
        if self.fleet_zone:
            reason=self.fleet_motion_reason(((point['x'],point['y']),))
            if reason:raise ValueError(reason)
            if kind=='return' or point.get('survey_path'):
                raise ValueError('双车模式仅允许各自运行区内的普通导航点')
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
            op['yaw_tolerance'] = (math.radians(8) if kind=='departure' else math.radians(10) if kind=='return' else self.goal_yaw_tolerance) if self.precision_arrival else self.goal_yaw_tolerance
            if op['sequential']:
                op['xy_tolerance']=self.position_xy_tolerance
                op['yaw_tolerance']=self.arrival.get('final_yaw_tolerance',self.goal_yaw_tolerance)
                if kind=='waypoint' and point.get('fas_station'):
                    op['fas_station']=True
                    op['stop_xy_tolerance']=FAS_STATION_POSITION_TOLERANCE
                    op['yaw_tolerance']=FAS_STATION_YAW_TOLERANCE
                elif kind=='waypoint' and point.get('rssi_station'):
                    op['stop_xy_tolerance']=RSSI_STATION_POSITION_TOLERANCE
                    op['strict_yaw_tolerance']=op['yaw_tolerance']
                    op['yaw_tolerance']=RSSI_STATION_YAW_TOLERANCE
                elif kind=='waypoint' and point.get('return_via'):
                    op['stop_xy_tolerance']=.12
                    op['yaw_tolerance']=math.pi
                elif kind=='waypoint':
                    op['station_pivot']=True
                    op['stop_xy_tolerance']=self.arrival.get('final_position_tolerance',.05)
            if kind=='survey':op['xy_tolerance']=min(.22,point.get('survey_pass_radius',.5));op['yaw_tolerance']=math.pi
            self.ops[identity] = op;self.active = identity;self.path = [];self.track.clear()
            if kind=='departure':
                self.send_departure(op)
                return identity
            self.dock_localization.arm()
            if op['sequential'] and self.pose and self.stationary_since and time.monotonic()-self.stationary_since>=.7:
                distance=math.hypot(self.pose[0]-point['x'],self.pose[1]-point['y'])
                if distance<=op.get('stop_xy_tolerance',self.arrival.get('final_position_tolerance',.05)):
                    op.update(status='position_settling',timeout=15.,already_at_position=True,travel_direction='stationary')
                    return identity
            if not self.send_fas_line(identity):
                self.plan_navigation(identity)
            return identity

    def dock_clearance_leg(self,pose,point):
        staging=not point.get('dock_retry_exit',False)
        leg=departure_leg(pose,self.dock_pose,point,staging=staging)
        try:
            for sample in leg['poses']:
                self.grid.validate(sample['x'],sample['y'],clearance=.14)
            if staging:
                target=leg['poses'][-1] if leg['poses'] else {'x':pose[0],'y':pose[1]}
                self.grid.validate(target['x'],target['y'],clearance=.22)
        except ValueError as error:
            raise ValueError('离桩直线或桩前转向区域超出地图净空') from error
        return leg

    def send_departure(self,op):
        op['controller_id']='GuardedDeparture'
        try:
            health=self.snapshot()
            if not self.dock_calibrated or not health['stationary'] or self.encoders is None:
                raise ValueError('离桩需要已标定的桩位和底盘停稳反馈')
            if not self.departure_cmd.get_subscription_count():
                raise ValueError('离桩速度保护未就绪')
            leg=self.dock_clearance_leg(self.pose,op['goal'])
        except ValueError as exc:
            op.update(status='failed',error=str(exc))
            self.zero_motion()
            return
        retry=op['goal'].get('dock_retry_exit',False)
        target=leg['poses'][-1] if leg['poses'] else dict(zip(('x','y','yaw'),self.pose))
        self.path=[[p['x'],p['y']] for p in leg['poses']]
        op.update(status='clearing_dock',explicit_path=True,departure_target=(target['x'],target['y']),
                  departure_origin=tuple(self.pose),departure_odom=self.encoders,
                  departure_seated=bool(health['charge_contact_live'] or health['dock_position_confirmed'] or retry),
                  departure_length=leg['distance'],departure_speed=0.,departure_reached=False,
                  started=time.monotonic(),timeout=max(32.,leg['distance']/.045+12.))

    def update_departure(self,op,now):
        op['departure_speed']=0.
        if now-op['started']>op['timeout']:
            self.end_departure(op,op.get('recovery_message') or '直行离桩超时，已停车')
            return
        if (self.pose is None or abs(time.time()-self.pose_stamp)>.35 or self.encoders is None
                or now-self.last_wheel>.25 or now-self.last_scan>.30):
            op['recovery_message']='等待实时雷达与底盘定位后离桩'
            return
        travel,side=dock_offset(self.pose,op['departure_origin'])
        wheel,wheel_side=dock_offset(self.encoders,op['departure_odom'])
        turn=abs(angle_error(self.pose[2],op['departure_origin'][2]))
        wheel_turn=abs(angle_error(self.encoders[2],op['departure_odom'][2]))
        if (min(travel,wheel)<-.04 or abs(travel-wheel)>.10 or max(abs(side),abs(wheel_side))>.055
                or max(turn,wheel_turn)>math.radians(9)):
            self.end_departure(op,'离桩时车身偏离直线或轮速与地图位移不一致，已停车复核')
            return
        remaining=op['departure_length']-max(travel,wheel)
        if remaining>DEPARTURE_STOP_MARGIN and not op['departure_reached']:
            limit=.06 if op['goal'].get('dock_retry_exit') else DEPARTURE_SPEED
            op['departure_speed']=min(limit,max(.035,.55*remaining))
            op['recovery_message']='离桩尚未完成，请核对前方和两侧净空'
            return
        op['departure_reached']=True
        op['recovery_message']='等待离桩停稳并确认充电接触已解除'
        if self.stationary_since is None or now-self.stationary_since<.5 or self.snapshot()['charge_contact_live']:
            return
        if (op['departure_length']-min(travel,wheel)>.05
                or math.dist(self.pose[:2],op['departure_target'])>.06):
            self.end_departure(op,'离桩停稳后，实测位移没有达到桩前净空位置')
            return
        if not op['goal'].get('dock_retry_exit') and not self.departure_turn_clear():
            op['recovery_message']='已离桩，等待车身四角的转向空间清空'
            return
        self.end_departure(op)

    def departure_turn_clear(self):
        scans=list(self.pending_scans)[-2:]
        if len(scans)<2 or len({(s.header.stamp.sec,s.header.stamp.nanosec) for s in scans})<2:
            return False
        now=self.get_clock().now().nanoseconds*1e-9
        for scan in scans:
            if not -.05<=now-scan.header.stamp.sec-scan.header.stamp.nanosec*1e-9<=.30:
                return False
            try:
                t=self.tf.lookup_transform('base_footprint',scan.header.frame_id,Time()).transform
            except Exception:
                return False
            points=scan_points_in_base(scan,t.translation.x,t.translation.y,yaw(t.rotation))
            if len(points)<10 or any(x*x+y*y<=SPIN_CLEARANCE_RADIUS**2 for x,y in points):
                return False
        return True

    def end_departure(self,op,error=None):
        self.departure_cmd.publish(String())
        self.zero_motion()
        self.path=[]
        op.update(status='failed' if error else 'succeeded',error=error,recovery_message=None)
        if not error:
            op['actual_pose']=list(self.pose)
            self.known_dock_start=False
            self.get_logger().info('低速直行离桩已停稳，地图与底盘位移复核通过')

    def plan_accepted(self, future, identity):
        with self.lock:
            op = self.ops[identity]
            try:
                handle = future.result()
                if not handle.accepted and op.get('fas_station') and not op['cancel']:
                    self.retry_arrival(op,'规划服务暂未接受，保留当前采集点');return
                if not handle.accepted:
                    raise ValueError('原生规划器拒绝目标')
                if op['cancel']:
                    handle.cancel_goal_async();op['status'] = 'canceled';return
                handle.get_result_async().add_done_callback(lambda f: self.plan_done(f,identity))
            except Exception as e:
                op['status'] = 'failed';op['error'] = str(e)

    @staticmethod
    def aligned_approach(pose, goal, planned):
        # Join two short heading-aligned straights with a cubic; the original
        # planner corridor, static clearance and live costmap must all allow it.
        if not pose or len(planned) < 2:
            return None
        start = complex(*pose[:2])
        end = complex(goal['x'], goal['y'])
        chord = end - start
        length = abs(chord)
        if not .55 <= length <= 2.8:
            return None
        forward = complex(math.cos(pose[2]), math.sin(pose[2]))
        final = complex(math.cos(goal['yaw']), math.sin(goal['yaw']))
        reverse = (chord * final.conjugate()).real < 0.
        sign = -1 if reverse else 1
        if min((chord * axis.conjugate()).real * sign / length for axis in (forward, final)) < .22:
            return None
        lead = min(.32, length * .21)
        shoulder = min(.54, length * .34)
        p1 = start + sign * shoulder * forward
        p3 = end - sign * lead * final
        p2 = p3 - sign * shoulder * final
        samples = []
        n = math.ceil((abs(p1-start)+abs(p2-p1)+abs(p3-p2))/.035)
        for i in range(n+1):
            t = i/n
            u = 1-t
            pos = u**3*start + 3*u*u*t*p1 + 3*u*t*t*p2 + t**3*p3
            tangent = sign * (u*u*(p1-start)+2*u*t*(p2-p1)+t*t*(p3-p2))
            if abs(tangent) < .025:
                return None
            samples.append((pos, math.atan2(tangent.imag, tangent.real)))
        n = math.ceil(lead/.035)
        samples.extend((p3+(end-p3)*i/n, goal['yaw']) for i in range(1,n+1))
        reference = [complex(*p[:2]) for p in planned]
        if any(min(abs(pos-ref) for ref in reference) > .42 for pos,_ in samples):
            return None
        return dict(reverse=reverse, clearance=.25,
                    poses=[dict(x=pos.real,y=pos.imag,yaw=yaw) for pos,yaw in samples])

    def plan_done(self, future, identity):
        with self.lock:
            op = self.ops[identity]
            if op['cancel']:
                op['status'] = 'canceled';return
            try:
                result = future.result()
                if result.status != 4 or not result.result.path.poses:
                    if op['sequential'] or op.get('kind')=='return':
                        self.retry_arrival(op,'目标或通路被实时障碍占用，等待可通行后自动继续');return
                    raise ValueError('目标没有可通行路线，未发送运动导航')
                self.path = self.plan_coordinates(result.result.path)
                fas = getattr(self,'fas_lease',None)
                if fas and fas.owner:
                    reason=fas.motion_error('return' if op['kind']=='return' else 'move',self.snapshot(),self.path)
                    if reason and reason!=SCAN_UNSTABLE:
                        op.update(status='failed',error=reason)
                        self.path = []
                        return
                if self.fleet_zone:
                    reason=self.fleet_motion_reason(self.path)
                    if reason:
                        op.update(status='failed',error=reason)
                        self.path=[]
                        return
                op.pop('directed_path',None);op.pop('explicit_path',None)
                leg = (straight_leg(self.pose,op['goal'],self.path)
                       if op['kind']=='waypoint' and not op.get('fas_station') and not op.get('direct_failed') else None)
                if (not leg and op['kind']=='waypoint' and op['sequential'] and not self.fleet_zone
                        and not (fas and fas.owner) and not op.get('arrival_retries')
                        and 'stop_xy_tolerance' not in op):
                    leg=self.aligned_approach(self.pose,op['goal'],self.path)
                op['curved_reverse']=bool(leg and leg.get('reverse') and 'clearance' in leg)
                if leg and self.leg_validator.service_is_ready():
                    try:
                        for p in leg['poses']:
                            self.grid.validate(p['x'],p['y'],clearance=leg.get('clearance',.18))
                    except ValueError:
                        pass
                    else:
                        candidate=NavPath();candidate.header.frame_id='map';candidate.header.stamp=self.get_clock().now().to_msg()
                        candidate.poses=[self.stamped(p) for p in leg['poses']]
                        op['directed_path']=candidate;op['reverse']=leg['reverse']
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
        for key in ('turn_approach_radius','turn_margin_entered','turn_aim','explicit_path'):
            op.pop(key,None)
        if op.get('directed_path') is not None:
            op['status']='validating_leg'
            request=IsPathValid.Request();request.path=op['directed_path']
            self.leg_validator.call_async(request).add_done_callback(lambda f:self.directed_validated(f,identity))
            return
        op['controller_id']='PointPosition' if op['sequential'] else 'FollowPath'
        op['travel_direction']='native'
        op['path_stamp']=self.get_clock().now().nanoseconds
        goal=NavigateToPose.Goal();goal.pose=self.stamped(op['goal'])
        goal.behavior_tree=(str(Path(__file__).parent/'assets/waypoint_position.xml')
                            if op['sequential'] else self.native_behavior_tree)
        self.nav.send_goal_async(goal).add_done_callback(lambda f:self.nav_accepted(f,identity))

    def directed_validated(self,future,identity):
        with self.lock:
            op=self.ops[identity]
            if op['cancel']:op['status']='canceled';return
            try:
                if not future.result().is_valid:
                    op.pop('directed_path',None)
                    if station_pivot(op):
                        self.replan_fas_line(op,identity)
                    else:
                        self.send_navigation(identity)
                    return
                if not station_pivot(op):
                    self.follow_directed(identity)
                    return
                if not self.pose:
                    raise ValueError('直行前等待定位')
                approach=yaw(op['directed_path'].poses[0].pose.orientation)
                if yaw_error(approach,self.pose[2])>math.radians(15):
                    if op.get('line_turn_used') or not self.spin_action.server_is_ready():
                        self.replan_fas_line(op,identity)
                    else:
                        self.path=self.plan_coordinates(op['directed_path'])
                        op.update(status='line_alignment',line_yaw=approach,started=time.monotonic(),timeout=8.)
                    return
                distance=math.dist(self.pose[:2],(op['goal']['x'],op['goal']['y']))
                endpoint=op['directed_path'].poses[-1].pose
                angle=yaw_error(yaw(endpoint.orientation),op['goal']['yaw'])
                outer=min(op['xy_tolerance'],op['stop_xy_tolerance'])
                op['turn_approach_radius']=approach_radius(angle,op['yaw_tolerance'],outer,self.turn_approach_radius)
                op['turn_aim']=(endpoint.position.x,endpoint.position.y)
                speed=straight_approach_speed(distance,reverse=op['reverse'])
                request=SetParametersAtomically.Request()
                request.parameters=[Parameter('FixedHeading.desired_linear_vel',value=speed).to_parameter_msg()]
                op.update(status='configuring',timeout=max(20.,distance/speed+20.))
                self.controller_parameters.call_async(request).add_done_callback(
                    lambda f:self.directed_speed_ready(f,identity,speed))
            except Exception as exc:
                if station_pivot(op):
                    self.retry_arrival(op,'等待直线路线复核：'+str(exc))
                else:
                    op.update(status='failed',error='直线路线检查失败：'+str(exc));self.zero_motion()

    def replan_fas_line(self,op,identity):
        op.pop('directed_path',None)
        op['direct_failed']=True
        self.plan_navigation(identity)

    def update_line_alignment(self,op,now):
        self.zero_motion()
        if not self.pose or self.stationary_since is None or now-self.stationary_since<.7:
            return
        if op['status']=='line_alignment':
            error=math.remainder(op['line_yaw']-self.pose[2],2*math.pi)
            op['line_turn_used']=True
            self.start_fas_spin(op,error,'line_settling')
        else:
            record_stopped_turn(op,self.pose)
            if not self.send_fas_line(self.active):
                self.replan_fas_line(op,self.active)

    def directed_speed_ready(self,future,identity,speed):
        with self.lock:
            op=self.ops[identity]
            if op['cancel']:
                op['status']='canceled'
                return
            try:
                response=future.result().result
                if not response.successful:
                    raise ValueError(response.reason)
                self.speed_limits['FixedHeading']=speed
                self.follow_directed(identity)
            except Exception as exc:
                self.retry_arrival(op,'等待直行速度配置：'+str(exc))

    def follow_directed(self,identity):
        op=self.ops[identity]
        # A long straight-line lookahead cuts across a short reverse bend.
        lookahead=min(.22,self.directed_lookahead) if op.get('curved_reverse') else self.directed_lookahead
        request=SetParametersAtomically.Request()
        request.parameters=[Parameter('FixedHeading.lookahead_dist',value=float(lookahead)).to_parameter_msg()]
        op.update(status='configuring',started=time.monotonic())
        self.controller_parameters.call_async(request).add_done_callback(
            lambda future:self.directed_lookahead_ready(future,identity))

    def directed_lookahead_ready(self,future,identity):
        with self.lock:
            op=self.ops[identity]
            if op['cancel']:
                self.finish_native_cancellation(op)
                return
            if self.active!=identity or op['status']!='configuring':
                return
            try:
                result=future.result().result
                if not result.successful:
                    raise ValueError(result.reason)
            except Exception as exc:
                self.retry_arrival(op,'等待路径前视距离配置：'+str(exc))
                return
            self.send_directed_path(identity)

    def send_directed_path(self,identity):
        op=self.ops[identity]
        goal=FollowPath.Goal()
        goal.path=op['directed_path']
        goal.controller_id='FixedHeading'
        goal.goal_checker_id='fas_turn_position_checker' if op.get('turn_approach_radius') else 'directed_position_checker'
        op.update(status='navigating',controller_id='FixedHeading',explicit_path=True,started=time.monotonic(),
                  travel_direction='reverse' if op.get('reverse') else 'forward')
        self.path=self.plan_coordinates(goal.path)
        self.follow.send_goal_async(goal).add_done_callback(lambda f:self.nav_accepted(f,identity))

    def nav_accepted(self, future, identity):
        with self.lock:
            op = self.ops[identity]
            try:
                handle = future.result()
                if not handle.accepted and station_pivot(op) and not op['cancel']:
                    self.retry_arrival(op,'跟踪服务暂未接受，保留当前采集点');return
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
                        if (station_pivot(op) and op.get('explicit_path') and p and
                                math.dist(p[:2],(g['x'],g['y']))>op['stop_xy_tolerance']):
                            op['direct_failed']=True
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
                    if op.get('kind')=='return':
                        op['handle']=None
                        self.retry_arrival(op,'通路受阻，保持停车并等待重新规划');return
                    if op['sequential']:
                        # A native progress failure near the goal is not by
                        # itself proof the sampling point is unusable. Stop,
                        # inspect real pose, and permit one bounded recovery.
                        if station_pivot(op) and op.get('explicit_path'):
                            op['line_aborts']=op.get('line_aborts',0)+1
                            op['direct_failed']=op['line_aborts']>=2
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
                             'approach' if op.get('spin_resume')=='line_settling' or op['status']=='line_alignment' else
                             'terminal' if op.get('heading_controller')=='PointPose' and op['status'] in ('heading_configuring','heading_wait','heading','heading_settling') else
                             'heading' if op['status'] in ('heading_configuring','heading_wait','heading','heading_settling',
                                                            'spin_wait','spinning') else 'position'),
                    'detail':op.get('recovery_message')}

    def request_speed(self,speed,controller_id='FollowPath'):
        fixed_pursuit = controller_id in ('FixedHeading', 'CorridorTrack', 'CorridorBend')
        pursuit = fixed_pursuit or (controller_id == 'PointPosition' and self.position_is_rpp)
        if fixed_pursuit:speed=min(speed,.12)
        cap=getattr(self,'survey_speed_limit',None)
        if cap is not None:speed=min(speed,cap)
        # Use DWB's own dynamic limits, preserving angular authority at low vx.
        last=self.speed_limits.get(controller_id)
        if (last is not None and abs(speed-last)<.001) or self.speed_update is not None:
            return
        if not self.controller_parameters.service_is_ready():
            return
        request=SetParametersAtomically.Request()
        if pursuit:
            request.parameters=[Parameter(controller_id+'.desired_linear_vel',value=float(speed)).to_parameter_msg()]
        else:
            request.parameters=[Parameter(k,value=float(speed)).to_parameter_msg()
                                for k in (controller_id+'.max_vel_x',controller_id+'.max_speed_xy')]
        # This bundled DWB version refreshes max_speed_xy_sq_ in the
        # min_speed_theta branch, not in max_speed_xy's branch. Reapply its
        # configured value last in the same atomic request; no source patch.
        if not pursuit:
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

    def stop_at_waypoint(self,op,distance,now):
        if not op['sequential'] or op['kind']!='waypoint':
            return False
        stop_distance=self.arrival.get('final_position_tolerance',.05)
        if station_pivot(op):
            stop_distance=min(op['xy_tolerance'],op['stop_xy_tolerance'])
        if op['controller_id']=='FixedHeading' and op.get('turn_approach_radius'):
            aim=op.get('turn_aim')
            aim_distance=math.dist(self.pose[:2],aim) if aim is not None else distance
            if distance>op['stop_xy_tolerance']:
                return False
            if aim_distance>op['turn_approach_radius']:
                if distance>stop_distance or now-op.setdefault('turn_margin_entered',now)<1.:
                    return False
        elif distance>stop_distance:
            return False
        op.update(cancel=True,status='canceling',settle_after_cancel='position_settling')
        if op['handle']:op['handle'].cancel_goal_async()
        self.zero_motion()
        return True

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
        if not self.pose or not self.stationary_since or now-self.stationary_since<.7:
            op.pop('station_check',None)
            return
        goal=op['goal'];distance=math.hypot(self.pose[0]-goal['x'],self.pose[1]-goal['y'])
        error=math.atan2(math.sin(goal['yaw']-self.pose[2]),math.cos(goal['yaw']-self.pose[2]))
        stage=refinement_stage(distance,error,
            position_tolerance=op.get('stop_xy_tolerance',self.arrival.get('final_position_tolerance',.05)),yaw_tolerance=op['yaw_tolerance'])
        if station_pivot(op):
            if (not self.confirmed or self.localization_guard.reason or now-self.last_wheel>.30
                    or now-self.last_scan>.35 or not -.05<=time.time()-self.pose_stamp<=.45):
                op.pop('station_check',None)
                return
            if ('turn_bias' in op and self.gyro and now-self.gyro[0]<=.20
                    and abs(self.gyro[2]-op['turn_bias'])>.03):
                op.pop('station_check',None)
                return
            if 'station_check' not in op:
                op['station_check']=StationCheck()
                if distance>op['stop_xy_tolerance'] or op['heading_attempts']:
                    self.request_stationary_amcl(now)
            stage=op['station_check'].observe(self.pose_stamp,self.last_scan,distance,error,
                                               op['stop_xy_tolerance'],op['yaw_tolerance'])
            if stage=='wait' or (stage=='heading' and now<op.get('heading_retry_at',0.)):
                return
            if stage=='complete':
                health=self.snapshot()
                if not health['ready'] or (op.get('fas_station') and not scan_confirmed(health)):
                    self.request_stationary_amcl(now)
                    return
            origin=record_stopped_turn(op,self.pose)
            if origin is not None and op.pop('learn_terminal_turn',False) and scan_confirmed(self.snapshot()):
                station=(round(goal['x'],2),round(goal['y'],2))
                self.turn_approach.observe(station,origin,self.pose,math.radians(op['last_turn']['requested_deg']),
                                           op['yaw_tolerance'],op['stop_xy_tolerance'])
            op.pop('station_check',None)
        if stage=='position' and (station_pivot(op) or not self.joint_terminal or distance>.12 or op.get('joint_adjusted')):
            self.retry_arrival(op,'等待目标和路线可通行，继续修正位置');return
        if stage=='complete':
            self.complete_waypoint(op,now,distance,error);return
        if not station_pivot(op) and op.get('settle_corrections',0)>=1+self.arrival.get('heading_retries',1):
            self.retry_arrival(op,'停车复核后重新规划到当前采集点');return
        self.begin_waypoint_heading(op)

    def begin_waypoint_heading(self,op):
        error=math.atan2(math.sin(op['goal']['yaw']-self.pose[2]),math.cos(op['goal']['yaw']-self.pose[2]))
        distance=math.hypot(self.pose[0]-op['goal']['x'],self.pose[1]-op['goal']['y'])
        joint=self.joint_terminal and distance>op.get('stop_xy_tolerance',self.arrival.get('final_position_tolerance',.05))
        if station_pivot(op):
            now=time.monotonic()
            if not self.spin_action.server_is_ready():
                op.update(status='heading_settling',heading_retry_at=now+1.,started=now,timeout=15.,
                          recovery_message='等待原地转向服务，保留当前点')
                return
            op.setdefault('first_heading_at',now)
            op['settle_corrections']=op.get('settle_corrections',0)+1
            op['heading_attempts']+=1
            self.path=[list(self.pose[:2])]
            self.start_fas_spin(op,error,'heading_settling')
            return
        if not self.follow.server_is_ready():
            op.update(status='failed',error='原生朝向跟踪动作未就绪');self.zero_motion();return
        speed_profile = heading_profile if joint else in_place_heading_profile
        profile=speed_profile(error,op.get('previous_heading_error'),op.get('previous_heading_limit'))
        if joint and distance>.04:
            profile['max_vel_theta']=.20  # Permit a small corrective arc for lateral error.
        timeout=12. if joint else max(9.,abs(error)/.10+5.)
        if joint:op['joint_adjusted']=True
        op.update(previous_heading_error=error,previous_heading_limit=profile['max_vel_theta'],
                  heading_controller='PointPose' if joint else 'PointHeading',
                  status='heading_configuring',handle=None,started=time.monotonic(),
                  timeout=min(timeout,float(self.arrival.get('heading_timeout',30.))),
                  settle_corrections=op.get('settle_corrections',0)+1,
                  heading_travel=0.,heading_budget=abs(error)+math.radians(45))
        request=SetParametersAtomically.Request()
        if joint:
            request.parameters=[Parameter('PointPose.wz_max',value=float(profile['max_vel_theta'])).to_parameter_msg()]
        else:
            request.parameters=[Parameter('PointHeading.'+k,value=float(v)).to_parameter_msg() for k,v in profile.items()]
            request.parameters.append(Parameter('point_heading_checker.yaw_goal_tolerance',
                                                value=op['yaw_tolerance']*.5).to_parameter_msg())
        self.controller_parameters.call_async(request).add_done_callback(
            lambda f,identity=self.active:self.heading_configured(f,identity))

    def start_fas_spin(self,op,error,resume):
        # The native behavior commands angular velocity only and checks the swept footprint.
        now=time.monotonic()
        gyro=getattr(self,'gyro',None)
        feedback=gyro and now-gyro[0]<=.15 and abs(gyro[2])<.04
        duration=math.ceil(max(8.,abs(error)/.16+5.))
        op.update(status='spin_wait',started=now,timeout=duration+2.,handle=None,
                  spin_resume=resume,spin_yaw=math.remainder(self.pose[2]+error,2*math.pi),
                  heading_controller='Spin',heading_travel=0.,heading_budget=abs(error)+math.radians(12),
                  turn_origin=tuple(self.pose),turn_started_at=now,learn_terminal_turn=False,
                  last_turn={'requested_deg':math.degrees(error),'stage':resume})
        op.pop('turn_bias',None)
        target=error
        if feedback:
            rates=[value for stamp,value in self.rest_gyro if 0.<=gyro[1]-stamp<=.65]
            bias=median(rates) if len(rates)>=3 else gyro[2]
            op['turn_bias']=bias
            op['spin_brake']=ProfiledSpin(error,gyro[1],bias)
            target=op['spin_brake'].native_target
            health=self.snapshot()
            if self.encoders is not None and health['stationary'] and health['ready'] and self.confirmed and scan_confirmed(health):
                op['spin_anchor']=(tuple(self.encoders[:2]),tuple(self.pose[:2]))
                op['learn_terminal_turn']=(resume=='heading_settling' and op['heading_attempts']==1)
        goal=Spin.Goal();goal.target_yaw=float(target);goal.time_allowance.sec=duration
        identity=self.active
        self.spin_action.send_goal_async(goal).add_done_callback(lambda f:self.spin_accepted(f,identity))

    def settle_waypoint_heading(self,op):
        brake=op.get('spin_brake')
        if brake:
            op['last_turn']['gyro_at_release_deg']=math.degrees(brake.direction*brake.progress)
        op.update(cancel=True,status='canceling',settle_after_cancel=op.get('spin_resume','heading_settling'))
        if op.get('spin_resume'):
            self.motion_permit.publish(Bool(data=False))
            self.publish_spin_limit(0.)
        if op['handle']:
            op['handle'].cancel_goal_async()
        self.zero_motion()

    def heading_needs_settle(self,op):
        if op.get('spin_resume')=='line_settling':
            return yaw_error(op['spin_yaw'],self.pose[2])<=math.radians(3)
        goal=op['goal']
        distance=math.hypot(self.pose[0]-goal['x'],self.pose[1]-goal['y'])
        error=math.atan2(math.sin(goal['yaw']-self.pose[2]),math.cos(goal['yaw']-self.pose[2]))
        stage=refinement_stage(distance,error,
            position_tolerance=op.get('stop_xy_tolerance',self.arrival.get('final_position_tolerance',.05)),yaw_tolerance=op['yaw_tolerance'])
        # Release joint translation as soon as the stopped position is usable.
        return stage=='complete' or (stage=='heading' and op.get('heading_controller')=='PointPose')

    def should_brake_heading(self, op, now):
        if op.get('heading_controller') not in ('PointHeading','Spin') or now-self.last_wheel>.18:
            return False
        if op.get('spin_brake'):
            return False
        error=math.remainder(op.get('spin_yaw',op['goal']['yaw'])-self.pose[2],2*math.pi)
        if station_pivot(op):
            return fas_heading_needs_brake(error,self.wheel_yaw_rate,op['yaw_tolerance'],
                                           now-self.pose_observed_at)
        return heading_needs_brake(error,self.wheel_yaw_rate,op['yaw_tolerance'])

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
        op.setdefault('first_heading_at',time.monotonic())
        op['heading_attempts']+=1;op['status']='heading_wait'
        goal=FollowPath.Goal();goal.controller_id=op['heading_controller']
        goal.goal_checker_id='point_pose_checker' if goal.controller_id=='PointPose' else 'point_heading_checker'
        path=NavPath();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
        start={'x':self.pose[0],'y':self.pose[1],'yaw':self.pose[2]}
        target=op['goal'] if goal.controller_id=='PointPose' else {**start,'yaw':op['goal']['yaw']}
        path.poses=[self.stamped(start),self.stamped(target)];goal.path=path
        self.path=[[start['x'],start['y']],[target['x'],target['y']]]
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
                    op.update(status='heading_settling',handle=None,native_heading_status=result.status,started=time.monotonic(),timeout=15.)
            except Exception as e:op.update(status='failed',error=str(e))
            self.zero_motion()

    def finish_native_cancellation(self,op):
        stage=op.pop('settle_after_cancel',None)
        if stage=='transit_recovery':
            op.update(cancel=False,handle=None)
            self.retry_arrival(op,'行驶受阻，停车后重试同一目标');return
        if stage:
            op.update(status=stage,cancel=False,handle=None,started=time.monotonic(),timeout=15.)
        else:
            op['status']='failed' if op.get('error') else 'canceled'

    def finish_waypoint_heading(self,op,now):
        self.finish_waypoint_position(op,now)

    def near_waypoint(self,distance,error):
        return (self.arrival.get('allow_near_arrival',False) and distance<=self.arrival.get('near_position_tolerance',.08) and
                abs(error)<=self.arrival.get('near_yaw_tolerance',math.radians(5)))

    def complete_waypoint(self,op,now,distance,error):
        strict_yaw = op.get('strict_yaw_tolerance', op['yaw_tolerance'])
        quality=('normal' if distance<=self.arrival.get('final_position_tolerance',.05) and abs(error)<=strict_yaw else 'near')
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
        if op['sequential'] or op.get('kind')=='return':
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
        if op['sequential'] or op.get('kind')=='return':
            # Check a fresh native path before starting another motion action.
            # A blocked sampling point remains pending, without repeated BT clears.
            identity=self.active
            op.update(settle_corrections=0,joint_adjusted=False)
            if station_pivot(op):
                op.pop('line_turn_used',None)
                op.pop('directed_path',None)
            if not self.send_fas_line(identity):
                self.plan_navigation(identity)
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
                if not handle.accepted:
                    resume=op.pop('spin_resume',None)
                    self.zero_motion()
                    op.pop('spin_brake',None)
                    op.pop('spin_anchor',None)
                    op.pop('spin_yaw',None)
                    if op['cancel']:
                        self.finish_native_cancellation(op)
                    elif resume:
                        now=time.monotonic()
                        op.update(status=resume,started=now,timeout=15.,heading_retry_at=now+1.,
                                  recovery_message='原地转向暂未接受，停车复核当前点')
                    else:
                        raise ValueError('原生最终定向被拒绝')
                    return
                op['handle']=handle
                handle.get_result_async().add_done_callback(lambda f:self.spin_done(f,identity))
                if op['cancel']:handle.cancel_goal_async()
                elif op.get('spin_resume'):
                    if op.get('spin_brake') and self.gyro:
                        op['spin_brake'].start=op['spin_brake'].stamp=self.gyro[1]
                    op['status']='spinning'
            except Exception as e:op.update(status='failed',error=str(e));self.zero_motion()

    def spin_done(self,future,identity):
        with self.lock:
            op=self.ops[identity]
            try:
                result=future.result()
                resume=op.pop('spin_resume',None)
                if op.get('spin_brake'):
                    self.publish_spin_limit(0.)
                op.pop('spin_yaw',None)
                op.pop('spin_brake',None)
                op.pop('spin_anchor',None)
                if op['cancel']:self.finish_native_cancellation(op)
                elif resume:
                    op.update(status=resume,handle=None,started=time.monotonic(),timeout=15.)
                elif result.status!=4:raise ValueError('原生最终定向未完成，已停止')
                else:op.update(status='spin_settling',handle=None)
            except Exception as e:op.update(status='failed',error=str(e))
            if op['status']=='heading_settling':
                # A blocked turn keeps its XY target latched. Slow repeated attempts
                # down without sending the vehicle off on a fresh translation route.
                attempts=op.get('settle_corrections',0)
                op['heading_retry_at']=time.monotonic()+min(6.,.5 * max(0,min(12,attempts-1)))
            self.zero_motion()

    def cancel(self, identity):
        with self.lock:
            op = self.ops[identity]
            if op['status'] in ('succeeded','canceled','failed'):
                return
            op.pop('settle_after_cancel',None)  # A user stop always suppresses recovery.
            op['cancel'] = True
            if op['status']=='clearing_dock':
                op['status']='failed' if op.get('error') else 'canceled'
                self.departure_cmd.publish(String())
            elif op['status']=='retreating':
                op['status']='failed' if op.get('error') else 'canceled'
                self.retreat_permit.publish(String())
            elif op['status'] in ('planning','validating_leg','configuring','heading_configuring','line_alignment',
                                'line_settling','recovery_wait','nav_settling','spin_settling','position_settling','heading_settling'):
                op['status'] = 'canceled'  # Planning never commands wheels; callbacks suppress navigation.
            else:
                op['status'] = 'canceling'
                if op['handle']:
                    op['handle'].cancel_goal_async()
            self.zero_motion()

    def zero_motion(self):
        if self.active and self.ops[self.active].get('spin_brake'):
            self.publish_spin_limit(0.)
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
            if getattr(self,'fleet_zone',None):
                raise ValueError('双车模式暂未配置独立回充区')
            fas = getattr(self,'fas_lease',None)
            if fas and fas.owner and not fas.allows('return',self.snapshot()):
                raise ValueError('等待双车回充走廊的专用许可')
            self.oem_lease.publish(Bool(data=True))
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

