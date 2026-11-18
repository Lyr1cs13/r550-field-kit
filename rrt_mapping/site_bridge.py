"""Live SLAM adapter. Manual and Nav2 commands share the final velocity gate."""
import json
import math
import time
from collections import deque
from uuid import uuid4

import numpy as np
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import FollowPath
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import ClearQueue, Pause
from std_msgs.msg import Bool, Int8
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from frontier_tasks import FrontierTasks, NativeDockingWatchdog
from motion_gate import yaw
from site_core import GridFrame, ManualDrive, angle_delta, dock_record, home_pose, inside
from slip_monitor import SlipMonitor


class SiteBridge(FrontierTasks):
    def __init__(self, state, context):
        super().__init__(state=state,context=context)
        self.manual = ManualDrive()
        self.manual_mode = False
        self.drive_session = None
        self.pose_map = None
        self.correction = None
        self.last_correction = -math.inf
        self.lifecycle = {}
        self.frame = None
        self.scan_msg = None
        self.scan_queue = deque(maxlen=12)
        self.last_scan = self.last_gate = -math.inf
        self.scan_xy = np.empty((0,2))
        self.scan_match = (0,0.)
        self.verified = False
        self.fault = ''
        self.writing = True
        self.live_path = []
        self.nav_twist = Twist()
        self.nav_stamp = -math.inf
        self.wheel_pose = None
        # A range-limit ring is tied to the sensor and carries no motion information.
        self.slip = SlipMonitor(range_ratio=.985)
        self.oem_sent = False
        self.ir_lost = None
        self.frontier_key = None
        self.frontier_idle = None
        self.capture_started = self.leaving_started = None
        self.permit = self.create_publisher(Bool,'/navigation_motion_permit',1)
        self.oem = self.create_publisher(Bool,'/navigation_oem_active',1)
        self.departure = self.create_publisher(Bool,'/site_dock_departure',1)
        self.tf_nav = self.create_publisher(TFMessage,'/tf',10)
        self.tf_slam = self.create_publisher(TFMessage,'/site_slam_tf',10)
        self.pause_client = self.create_client(Pause,'/slam_toolbox/pause_new_measurements')
        self.param_client = self.create_client(GetParameters,'/slam_toolbox/get_parameters')
        self.clear_client = self.create_client(ClearQueue,'/slam_toolbox/clear_queue')
        self.lifecycle_clients={name:self.create_client(GetState,f'/{name}/get_state')
                                for name in ('controller_server','planner_server')}
        self.create_subscription(Bool,'/navigation_gate_alive',
                                 lambda m: setattr(self,'last_gate',time.monotonic()),1)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/map',self.receive_map,latched)
        self.create_subscription(LaserScan,'/scan',self.receive_scan,qos_profile_sensor_data)
        self.create_subscription(TFMessage,'/tf',self.relay_odom,10)
        self.create_subscription(TFMessage,'/site_slam_tf',self.relay_correction,10)
        self.create_subscription(Twist,'/site_nav_cmd_vel',self.receive_nav,1)
        self.create_subscription(Path,'/plan',self.receive_path,1)
        self.create_timer(.05,self.guard)
        self.create_timer(.4,self.project_scan)
        self.create_timer(.75,self.check_navigation)

    def age(self, stamp):
        return (self.get_clock().now().nanoseconds-Time.from_msg(stamp).nanoseconds)*1e-9

    def relay_odom(self,msg):
        # SLAM receives the live odometry TF without receiving its relayed correction back.
        ordinary=[t for t in msg.transforms if t.header.frame_id!='map']
        if ordinary:
            self.tf_slam.publish(TFMessage(transforms=ordinary))

    def relay_correction(self,msg):
        for t in msg.transforms:
            if t.header.frame_id=='map' and t.child_frame_id=='odom_combined':
                self.correction=t.transform
                self.last_correction=time.monotonic()

    def publish_correction(self,now):
        # Humble 2.6.x keeps publishing the frozen correction with the last
        # scan's old stamp while writes are paused. Supply that same correction
        # on the live TF stream only while SLAM is still publishing it.
        if self.correction is not None and now-self.last_correction<=.30:
            t=TransformStamped()
            t.header.frame_id='map'
            t.header.stamp=self.get_clock().now().to_msg()
            t.child_frame_id='odom_combined'
            t.transform=self.correction
            self.tf_nav.publish(TFMessage(transforms=[t]))

    def receive_map(self,msg):
        m=msg.info
        if (msg.header.frame_id.lstrip('/')=='map' and m.width and m.height and
                m.width*m.height<=4_000_000 and m.resolution>0 and abs(yaw(m.origin.orientation))<1e-5):
            self.frame=GridFrame(msg.data,m.width,m.height,m.resolution,
                                 (m.origin.position.x,m.origin.position.y))

    def receive_scan(self,msg):
        ranges=np.asarray(msg.ranges)
        count=((msg.range_min<=ranges)&(ranges<=msg.range_max)&np.isfinite(ranges)).sum()
        if count>=10 and -.10<=self.age(msg.header.stamp)<=.40:
            self.last_scan=time.monotonic()
            self.scan_msg=msg
            self.scan_queue.append((self.last_scan,msg))

    def project_scan(self):
        now=time.monotonic()
        if self.correction is None or now-self.last_scan>.45:
            self.scan_match=(0,0.)
            self.scan_xy=np.empty((0,2))
            return
        for seen,msg in reversed(self.scan_queue):
            if not .10<=now-seen<=.60:
                continue
            try:
                t=self.tf.lookup_transform('odom_combined',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
                break
            except Exception:
                continue
        else:
            self.scan_match=(0,0.)
            return
        stride=max(1,len(msg.ranges)//500)
        ranges=np.asarray(msg.ranges[::stride])
        x,y,heading=self.in_map(t)
        angles=heading+msg.angle_min+np.arange(len(ranges))*stride*msg.angle_increment
        use=np.isfinite(ranges)&(ranges>=max(.24,msg.range_min))&(ranges<=min(6.,msg.range_max))
        self.scan_xy=np.column_stack((x+ranges[use]*np.cos(angles[use]),
                                      y+ranges[use]*np.sin(angles[use])))
        self.scan_match=self.frame.match(self.scan_xy) if self.frame else (0,0.)

    def wheel_feedback(self,msg):
        p=msg.pose.pose
        enc=(p.position.x,p.position.y,yaw(p.orientation))
        v=msg.twist.twist
        if all(math.isfinite(a) for a in (*enc,v.linear.x,v.angular.z)) and -.10<=self.age(msg.header.stamp)<=.45:
            self.wheel_pose=enc
            super().wheel_feedback(msg)

    def charge_feedback(self,msg):
        super().charge_feedback(msg)
        self.got('charge',msg.data)

    def receive_nav(self,msg):
        self.nav_twist,self.nav_stamp=msg,time.monotonic()

    def receive_path(self,msg):
        if self.mode in ('explore','return') and self.active:
            path=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
            if path:
                self.live_path=path

    def in_map(self,transform):
        correction=self.correction
        angle=yaw(correction.rotation)
        c,s=math.cos(angle),math.sin(angle)
        x,y=transform.translation.x,transform.translation.y
        heading=yaw(transform.rotation)+angle
        return (correction.translation.x+c*x-s*y,correction.translation.y+s*x+c*y,
                angle_delta(heading,0.))

    def read_pose(self):
        if self.correction is None:
            return None
        try:
            t=self.tf.lookup_transform('odom_combined','base_footprint',Time())
            pose=self.in_map(t.transform)
            if -.8<=self.age(t.header.stamp)<=.55 and all(math.isfinite(v) for v in pose):
                return pose
        except Exception:
            pass
        return None

    def check_navigation(self):
        def record(name,future):
            try:
                self.lifecycle[name]=future.result().current_state.id,time.monotonic()
            except Exception:
                self.lifecycle.pop(name,None)
        for name,client in self.lifecycle_clients.items():
            if client.service_is_ready():
                client.call_async(GetState.Request()).add_done_callback(lambda future,n=name:record(n,future))

    def nav_ready(self):
        now=time.monotonic()
        return (self.nav.server_is_ready() and self.plan.server_is_ready() and
                all(self.lifecycle.get(name,(0,0))[0]==3 and now-self.lifecycle[name][1]<2.
                    for name in self.lifecycle_clients))

    def stationary(self,now):
        return (self.stationary_since is not None and now-self.stationary_since>=.7
                and now-self.last_wheel<=.35)

    def fresh(self,now):
        return (self.frame is not None and self.pose_map is not None and
                now-self.last_scan<=.40 and now-self.last_wheel<=.35 and now-self.last_gate<=.25
                and now-self.last_correction<=.30)

    def aligned_at_dock(self):
        home=home_pose(self.home)
        p=self.pose_map
        known,agreement=self.scan_match
        return (self.fresh(time.monotonic()) and self.stationary(time.monotonic()) and
                math.dist(p[:2],home[:2])<=.13 and abs(angle_delta(p[2],home[2]))<=.15
                and known>=30 and agreement>=.65)

    def command(self,mode,response):
        self.manual_mode=False
        self.drive_session=None
        self.manual.clear()
        self.live_path=[]
        self.nav_stamp=-math.inf
        self.permit.publish(Bool(data=False))
        self.departure.publish(Bool(data=False))
        self.oem.publish(Bool(data=False))
        self.disable_ir()
        return super().command(mode,response)

    def disable_ir(self):
        if self.oem_sent:
            for _ in range(4):
                self.ir.publish(Int8(data=0))
            self.oem_sent=False

    def halt(self, phase='pause'):
        self.command('pause',Trigger.Response())
        self.phase=phase

    def freeze_on_fault(self,message):
        self.fault=message
        self.halt('localization_fault')
        if self.param_client.service_is_ready():
            f=self.param_client.call_async(GetParameters.Request(names=['paused_new_measurements']))
            def freeze(done):
                try:
                    paused=done.result().values[0].bool_value
                    if not paused and self.pause_client.service_is_ready():
                        self.pause_client.call_async(Pause.Request())
                    if self.clear_client.service_is_ready():
                        self.clear_client.call_async(ClearQueue.Request())
                    self.writing=False
                except Exception:
                    self.get_logger().error('Could not freeze SLAM after a motion fault')
            f.add_done_callback(freeze)

    def set_dock(self):
        self.home=dock_record(self.pose_map)
        p=self.output.parent/'dock_start.json'
        temp=p.with_suffix('.tmp')
        temp.write_text(json.dumps(self.home,indent=2),encoding='utf-8')
        temp.replace(p)
        self.verified=True

    def near_seated(self):
        h=home_pose(self.home)
        return (self.pose_map is not None and math.dist(self.pose_map[:2],h[:2])<.23 and
                abs(angle_delta(self.pose_map[2],h[2]))<.17)

    def begin(self, mode):
        now=time.monotonic()
        if not self.verified or self.fault or not self.fresh(now):
            raise ValueError('请先在桩位完成定位校准，等待雷达和底盘就绪')
        if mode=='explore':
            if not self.rectangle or not inside(self.rectangle,self.pose_map,.20):
                raise ValueError('探索范围需包含小车当前位置和车身余量')
        if mode!='manual' and (self.busy or not self.nav_ready()):
            raise ValueError('等待上一任务停止及建图导航就绪')
        self.command('pause' if mode=='manual' else mode,Trigger.Response())
        self.slip.reset()
        if mode=='manual':
            self.manual_mode=True
            self.drive_session=uuid4().hex
            self.phase='manual'
        elif mode=='explore':
            self.points=[None]*5
            self.frontier_key=None
            self.frontier_idle=None
            if self.near_seated():
                self.phase='leaving'
                self.leaving_started=(now,self.wheel_travel)
        elif self.dock_confirmation.confirmed and self.near_seated():
            self.halt('docked')
        elif self.can_capture() and (self.red or self.near_seated()):
            self.phase='capture'
            self.capture_started=now

    def can_capture(self):
        if self.pose_map is None:
            return False
        hx,hy,yaw0=home_pose(self.home)
        x,y,heading=self.pose_map
        along=(x-hx)*math.cos(yaw0)+(y-hy)*math.sin(yaw0)
        lateral=-(x-hx)*math.sin(yaw0)+(y-hy)*math.cos(yaw0)
        return -.11<=along<=.84 and abs(lateral)<=.34 and abs(angle_delta(heading,yaw0))<.60

    def accept_plan(self,path,goal,kind):
        coords=[(v.pose.position.x,v.pose.position.y) for v in path.poses] if path else []
        if coords and (kind!='explore' or all(inside(self.rectangle,p,.10) for p in coords)):
            self.live_path=coords
            return True
        if kind=='return':
            self.halt('return_unreachable')
        else:
            p=goal.pose.position
            self.blocked.append((p.x,p.y,time.monotonic()+120.))
            self.failures+=1
            self.phase='waiting_for_frontiers'
        return False

    def plan_done(self,future,generation,p,kind):
        if generation!=self.generation:
            self.busy=False
            return
        try:
            outcome=future.result()
            path=outcome.result.path if outcome.status==GoalStatus.STATUS_SUCCEEDED else None
        except Exception:
            path=None
        if not self.accept_plan(path,p,kind):
            self.busy=False
            return
        self.phase='navigating_'+kind
        goal=FollowPath.Goal(path=path,controller_id='FollowPath',goal_checker_id='goal_checker')
        self.nav.send_goal_async(goal).add_done_callback(lambda f:self.nav_accepted(f,generation,kind))

    def nav_done(self,future,generation,kind):
        if kind=='return' and generation==self.generation:
            self.active=None
            self.busy=False
            try:
                success=future.result().status==GoalStatus.STATUS_SUCCEEDED
            except Exception:
                success=False
            if success:
                self.phase='capture'
                self.capture_started=time.monotonic()
                self.live_path=[]
            else:
                self.halt('return_unreachable')
        else:
            super().nav_done(future,generation,kind)

    def tick(self):
        if self.phase in ('leaving','capture','docking'):
            return
        if self.mode=='return' and self.phase in ('plan_error','nav_error'):
            self.halt('return_unreachable')
            return
        if self.mode=='explore' and self.pose_map is not None and self.frame is not None and not self.busy:
            key=(self.frame.id,tuple(self.rectangle))
            if key!=self.frontier_key:
                self.frontier_key=key
                for point in self.frame.frontier_candidates(self.rectangle,self.pose_map):
                    self.offer(*point)
            if not self.proposals:
                self.phase='waiting_for_frontiers'
                now=time.monotonic()
                self.frontier_idle=self.frontier_idle or now
                if now-self.frontier_idle>10. and self.nav.server_is_ready():
                    self.halt('no_frontiers')
            else:
                self.frontier_idle=None
        super().tick()

    def update_speed(self,value):
        pass  # The panel's native controller and final gate have fixed low limits.

    def sentry_docking(self,now,stopped):
        if self.dock_confirmation.confirmed:
            self.halt('docked')
            return
        ir_recent=now-self.telemetry.get('ir',(-math.inf,0))[0]<=.5
        if self.phase=='capture':
            if stopped and ir_recent and self.red and self.can_capture():
                self.dock_watchdog=NativeDockingWatchdog()
                self.dock_watchdog.reset(now,self.wheel_travel)
                self.ir_lost=self.charge_moving_since=None
                self.phase='docking'
                self.oem.publish(Bool(data=True))
                self.oem_sent=True
                for _ in range(10):
                    self.ir.publish(Int8(data=1))
            elif now-self.capture_started>8.:
                self.halt('no_ir')
            return
        near=math.dist(self.pose_map[:2],home_pose(self.home)[:2])<.18
        error=self.dock_watchdog.update(now,travel=self.wheel_travel,near=near,
                                        fresh=now-self.telemetry.get('ir',(-math.inf,0))[0]<2)
        electrical=self.telemetry.get('current',(-math.inf,0.))
        if now-electrical[0]<.6 and electrical[1]>=.1 and not stopped:
            self.charge_moving_since=self.charge_moving_since or now
        else:
            self.charge_moving_since=None
        self.ir_lost=None if ir_recent and self.red else (self.ir_lost or now)
        if (error or (self.ir_lost is not None and now-self.ir_lost>8.) or
                (self.charge_moving_since is not None and now-self.charge_moving_since>2.5)):
            self.halt('dock_failed')

    def guard(self):
        now=time.monotonic()
        self.publish_correction(now)
        self.pose_map=self.read_pose()
        stopped=self.stationary(now)
        timestamp,current=self.telemetry.get('current',(-math.inf,None))
        self.dock_confirmation.update(now,charging=self.charging and now-self.telemetry.get('charge',(-math.inf,0))[0]<1.,
                                      stationary=stopped,current=current if now-timestamp<.7 else None,voltage=self.voltage)
        usable=self.verified and not self.fault and self.fresh(now)
        active=self.manual_mode or self.mode in ('explore','return')
        if active and not usable:
            self.halt('sensors_lost')
        if self.mode=='explore' and usable and not inside(self.rectangle,self.pose_map,.14):
            self.halt('outside_region')
        if self.phase in ('capture','docking') and usable:
            self.sentry_docking(now,stopped)
        v=Twist()
        if usable and self.manual_mode and not self.busy:
            v.linear.x,v.angular.z=self.manual.output(now)
        elif usable and self.phase=='leaving':
            t,travel=self.leaving_started
            if now-t>9.:
                self.halt('departure_blocked')
            elif self.wheel_travel-travel>=.28 or math.dist(self.pose_map[:2],home_pose(self.home)[:2])>=.30:
                self.phase='waiting_for_frontiers'
            else:
                v.linear.x=.08
        elif usable and self.active and self.mode in ('explore','return') and now-self.nav_stamp<.22:
            if self.mode=='explore' and not all(inside(self.rectangle,p,.10) for p in self.live_path):
                self.halt('outside_region')
            elif self.live_path:
                v.linear.x=max(-.08,min(.10,self.nav_twist.linear.x))
                v.angular.z=max(-.30,min(.30,self.nav_twist.angular.z))
        moving=self.phase!='docking' and usable and (abs(v.linear.x)+abs(v.angular.z)>.02 or not stopped)
        if moving and self.scan_msg is not None:
            if self.slip.observe(self.last_scan,self.wheel_pose,self.pose_map,self.scan_msg) is not None:
                self.freeze_on_fault('轮程与雷达不一致，已停车。请放回桩位校准')
                v=Twist()
        self.oem.publish(Bool(data=self.phase=='docking' and usable))
        self.departure.publish(Bool(data=usable and self.near_seated() and v.linear.x>0 and abs(v.angular.z)<.05))
        self.permit.publish(Bool(data=usable and bool(v.linear.x or v.angular.z)))
        self.cmd.publish(v)

    def view(self):
        now=time.monotonic()
        return dict(phase=self.phase, map=self.frame.view() if self.frame else None, pose=self.pose_map,
                    dock=home_pose(self.home) if self.verified else None,
                    radar=self.scan_xy[::2].round(3).tolist(), path=self.live_path[::2],
                    stationary=self.stationary(now), ready=self.fresh(now) and self.verified and not self.fault,
                    nav_ready=self.nav_ready(),
                    writing=self.writing, charging=self.dock_confirmation.confirmed,
                    fault=self.fault, bounds=self.rectangle, map_match=self.scan_match,
                    drive_session=self.drive_session, targets=self.successes)
