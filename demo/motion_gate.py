"""Final ROS navigation velocity owner, downstream of collision checking.

STM32 IR docking remains a separate OEM hardware mode with its own watchdog.
"""
import json
import time
import math
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int8, String
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from motion_health import (DEPARTURE_SPEED, NativeStopFeedback, StopFeedback, VelocityLease, OemLease, cautious_twist,
                           departure_twist, scan_points_in_base, swept_dock_clear, swept_footprint_clear)
from guarded_retreat import GuardedRetreat


OEM_STOP_CYCLES = 10


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class MotionGate(Node):
    def __init__(self):
        super().__init__('navigation_motion_gate')
        self.declare_parameter('site_mapping',False)
        record_path = self.declare_parameter('dock_record','').value
        record = json.loads(Path(record_path).read_text(encoding='utf-8')) if record_path else {}
        self.dock_pose = None
        if record.get('calibrated',bool(record)):
            pose = record['robot_pose']
            self.dock_pose = tuple(float(pose[k]) for k in ('x','y','yaw'))
            if not all(map(math.isfinite,self.dock_pose)):
                raise ValueError('充电桩位姿无效')
        self.lease = VelocityLease()
        self.last_motion=time.monotonic()
        self.last_tick=self.last_motion;self.last_linear=0.
        self.hold_file=Path(__file__).resolve().parent.parent/'state'/'motion-hold'
        self.stop_feedback=StopFeedback()
        self.native_stop=NativeStopFeedback()
        self.last_scan=-math.inf
        self.oem=OemLease();self.disable_burst=0
        config=Path(__file__).resolve().parent.parent/'config/vehicle.yaml'
        vehicle=yaml.safe_load(config.read_text(encoding='utf-8')) if config.exists() else {}
        echo=vehicle.get('antenna_filter',{}).get('retreat_echo')
        self.retreat=GuardedRetreat(echo);self.retreat_lease=-math.inf;self.retreat_identity=''
        self.retreat_scan=None;self.retreat_odom=None;self.last_retreat_odom=-math.inf
        self.wheel_stamp_ns=None
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.output = self.create_publisher(Twist, '/cmd_vel', 1)
        self.alive = self.create_publisher(Bool, '/navigation_gate_alive', 1)
        self.ir_disable = self.create_publisher(Int8, '/robot_recharge_flag', 1)
        self.retreat_status=self.create_publisher(String, '/navigation_retreat_status', 1)
        self.create_subscription(Twist, '/guard_cmd_vel', self.command, 1)
        self.create_subscription(Bool, '/navigation_motion_permit', self.permit, 1)
        self.create_subscription(TwistStamped, '/navigation_spin_limit', self.spin_limit, 1)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(String, '/navigation_retreat_permit', self.permit_retreat, 1)
        self.create_subscription(Bool, '/navigation_oem_active',self.oem_state,1)
        self.create_subscription(Bool, '/robot_charging_mode',self.firmware_oem_state,qos_profile_sensor_data)
        self.dock_departure=-math.inf
        self.dock_departure_stamp=-math.inf
        self.departure_order=None
        if self.get_parameter('site_mapping').value:
            self.create_subscription(Bool,'/site_dock_departure',self.departure,1)
        else:
            self.create_subscription(String,'/navigation_departure_cmd',self.departure_command,1)
        self.create_timer(.05, self.tick)

    def departure(self,msg):
        self.dock_departure=time.monotonic() if msg.data else -math.inf

    def departure_command(self,msg):
        if not msg.data:
            self.departure_order=None
            return
        try:
            stamp,speed,contact=json.loads(msg.data)
        except (ValueError,TypeError):
            return
        numbers=(stamp,speed,*(contact if isinstance(contact,list) else ()))
        if (any(type(v) not in (float,int) or not math.isfinite(v) for v in numbers)
                or not self.dock_departure_stamp<stamp or not 0.<speed<=DEPARTURE_SPEED
                or (contact is not None and (not isinstance(contact,list) or len(contact)!=3))
                or not -.05<=self.get_clock().now().nanoseconds*1e-9-stamp<=.18):
            return
        self.dock_departure_stamp=stamp
        self.departure_order=(time.monotonic(),speed,contact)

    def command(self, msg):
        self.lease.receive_command(time.monotonic(), (msg.linear.x, msg.linear.y, msg.angular.z))

    def permit(self, msg):
        self.lease.receive_permit(time.monotonic(), msg.data, self.get_clock().now().nanoseconds*1e-9)

    def spin_limit(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        self.lease.receive_spin_limit(time.monotonic(), stamp, age, msg.twist.angular.z)

    def permit_retreat(self, msg):
        if msg.data and msg.data!=self.retreat_identity:
            self.retreat_identity=msg.data;self.retreat.reset()
        self.retreat_lease=time.monotonic() if msg.data else -math.inf

    def odom(self, msg):
        now=time.monotonic()
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age=(self.get_clock().now().nanoseconds-stamp)*1e-9
        if not -.05<=age<=.25 or stamp==self.wheel_stamp_ns:
            return
        if self.wheel_stamp_ns is not None and stamp<self.wheel_stamp_ns and now-self.last_retreat_odom<=.25:
            return
        p=msg.pose.pose;t=msg.twist.twist
        values=(p.position.x,p.position.y,yaw(p.orientation),t.linear.x,t.angular.z)
        self.retreat_odom=values if all(math.isfinite(v) for v in values) else None
        self.wheel_stamp_ns=stamp
        self.last_retreat_odom=now-max(age,0.)

    def oem_state(self,msg):
        now=time.monotonic()
        if msg.data and not self.oem.active:self.last_motion=now
        if self.oem.receive(now,msg.data):
            self.disable_burst=OEM_STOP_CYCLES
            self.native_stop.start()

    def firmware_oem_state(self,msg):
        if msg.data and not self.oem.active:
            self.disable_burst=OEM_STOP_CYCLES
            self.native_stop.start()

    def scan(self, msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        age=self.get_clock().now().nanoseconds*1e-9-stamp
        valid=sum(math.isfinite(r) and msg.range_min<=r<=msg.range_max for r in msg.ranges)
        if -.05<=age<=.4 and valid>=10:
            self.last_scan=time.monotonic();self.retreat_scan=msg

    def scan_in_base(self):
        msg=self.retreat_scan
        if msg is None:
            return ()
        try:
            t=self.tf.lookup_transform('base_footprint', msg.header.frame_id, Time()).transform
        except Exception:
            return ()
        return scan_points_in_base(msg,t.translation.x,t.translation.y,yaw(t.rotation))

    def dock_motion_clear(self, velocity):
        if self.dock_pose is None:
            return True
        try:
            transform = self.tf.lookup_transform('map','base_footprint',Time())
        except Exception:
            return False
        stamp = transform.header.stamp
        age = self.get_clock().now().nanoseconds*1e-9-(stamp.sec+stamp.nanosec*1e-9)
        if not -.35<=age<=.4:
            return False
        t = transform.transform
        pose = (t.translation.x,t.translation.y,yaw(t.rotation))
        return swept_dock_clear(velocity,pose,self.dock_pose)

    def tick(self):
        self.alive.publish(Bool(data=True))
        now=time.monotonic();v = self.lease.output(now)
        dt=min(.12,now-self.last_tick);self.last_tick=now
        reviewing=self.stop_feedback.reviewing
        if reviewing:v=(0.,0.,0.)
        retreating=not reviewing and now-self.retreat_lease<=.20
        order=self.departure_order
        departing=not reviewing and order is not None and now-order[0]<=.18
        fresh=now-self.last_retreat_odom<=.25 and self.retreat_odom is not None
        if retreating:
            if self.oem.active or self.disable_burst or self.native_stop.active or any(v) or departing:
                self.retreat.status='canceled';v=(0.,0.,0.)
            else:
                points=self.scan_in_base() if now-self.last_scan<=.30 else ()
                odom=self.retreat_odom if now-self.last_retreat_odom<=.15 else None
                v=(self.retreat.tick(now,points,odom),0.,0.)
        elif self.retreat_identity and self.retreat.status in ('idle','backing'):
            self.retreat.status='canceled'
        self.retreat_status.publish(String(data=f'{self.retreat_identity}:{self.retreat.status}'))
        if self.oem.expired(now):
            self.disable_burst=OEM_STOP_CYCLES
            self.native_stop.start()
        disabling_oem=bool(self.disable_burst)
        if disabling_oem:
            self.ir_disable.publish(Int8(data=0));self.disable_burst-=1
            self.last_motion=now;v=(0.,0.,0.)
        if self.oem.active or self.native_stop.active:v=(0.,0.,0.)
        if now-self.last_scan>.4:v=(0.,0.,0.)
        if departing:
            if (not any(v) and not self.oem.active and not disabling_oem and not self.native_stop.active and not retreating
                    and not self.hold_file.exists() and now-self.last_scan<=.25 and fresh
                    and self.retreat_odom[3]>=-.01 and abs(self.retreat_odom[4])<=.10):
                v=departure_twist(order[1],self.scan_in_base(),order[2],self.last_linear,dt)
            else:
                v=(0.,0.,0.)
        elif any(v) and not retreating:
            points=self.scan_in_base()
            if not points:
                v=(0.,0.,0.)
            else:
                if now-self.dock_departure<=.20 and v[0]>0 and abs(v[2])<=.05:
                    # The seated rear contact is already touching the dock.
                    # During a commanded straight departure its clearance only increases.
                    points=[(x,y) for x,y in points if not (-.24<=x<=-.125 and abs(y)<=.105)]
                v=cautious_twist(v,points,self.last_linear,dt)
                if not swept_footprint_clear(v,points):
                    v=(0.,0.,0.)
        if any(v) and not departing and not self.dock_motion_clear(v):
            v=(0.,0.,0.)
        # An accepted cancellation says nothing about the wheels. Keep braking
        # until chassis feedback settles, even if the velocity source is gone.
        native_braking=self.native_stop.active
        feedback=self.retreat_odom if fresh and (reviewing or native_braking or not self.oem.active) else None
        if self.hold_file.exists() or reviewing:
            v=(0.,0.,0.)
        if native_braking and not reviewing:
            if not disabling_oem:
                self.ir_disable.publish(Int8(data=0))
            v=(0.,0.,0.)
            persistent=self.native_stop.update(now,feedback)
            self.stop_feedback.still_moving(now,v,None)
        else:
            persistent=self.stop_feedback.requires_lock(
                now,v,feedback,self.last_retreat_odom,lambda: self.turn_clear_for_braking(now),
                lambda: self.travel_clear_for_braking(now,feedback))
        if reviewing or self.stop_feedback.reviewing:
            v=(0.,0.,0.)
            self.lease.receive_permit(now,False)
            self.lease.receive_command(now,v)
            self.departure_order=None
            self.retreat_lease=-math.inf
            self.ir_disable.publish(Int8(data=0))
            if self.oem.receive(now,False):
                self.native_stop.start()
            if not reviewing:
                self.get_logger().info('小幅刹停余动，保持零速并复核轮速')
            elif not self.stop_feedback.reviewing and not self.hold_file.exists():
                self.get_logger().info('刹停复核已停稳，等待新的行驶许可')
        self.last_linear=v[0]
        if persistent and not self.hold_file.exists():
            self.hold_file.parent.mkdir(parents=True,exist_ok=True)
            self.hold_file.touch()
            self.get_logger().error(f'零速指令后车轮持续运动，已锁定底盘：v={feedback[3]:.3f}, w={feedback[4]:.3f}')
        if self.hold_file.exists():
            self.ir_disable.publish(Int8(data=0))
            self.output.publish(Twist())
            self.last_linear=0.
            return
        if any(v):self.last_motion=now
        elif self.oem.active and not native_braking and now-self.last_motion>.5:
            return  # Do not continuously send velocity frames during OEM IR docking.
        msg = Twist();msg.linear.x, msg.linear.y, msg.angular.z = v
        self.output.publish(msg)

    def turn_clear_for_braking(self, now):
        points=self.scan_in_base() if now-self.last_scan<=.25 else ()
        return len(points)>=10 and swept_footprint_clear((0.,0.,.20),points)

    def travel_clear_for_braking(self, now, feedback):
        points=self.scan_in_base() if now-self.last_scan<=.25 else ()
        velocity=(feedback[3],0.,feedback[4])
        return len(points)>=10 and swept_footprint_clear(velocity,points) and self.dock_motion_clear(velocity)


def main():
    rclpy.init();node = MotionGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():node.output.publish(Twist())
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    main()
