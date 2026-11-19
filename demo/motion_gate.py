"""Final ROS navigation velocity owner, downstream of collision checking.

STM32 IR docking remains a separate OEM hardware mode with its own watchdog.
"""
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
from motion_health import VelocityLease, OemLease, cautious_twist, swept_footprint_clear
from guarded_retreat import GuardedRetreat


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class MotionGate(Node):
    def __init__(self):
        super().__init__('navigation_motion_gate')
        self.declare_parameter('site_mapping',False)
        self.lease = VelocityLease()
        self.last_motion=time.monotonic()
        self.last_tick=self.last_motion;self.last_linear=0.
        self.hold_file=Path(__file__).resolve().parent.parent/'state'/'motion-hold'
        self.uncommanded_since=None
        self.last_scan=-math.inf
        self.oem=OemLease();self.disable_burst=0
        config=Path(__file__).resolve().parent.parent/'config/vehicle.yaml'
        vehicle=yaml.safe_load(config.read_text(encoding='utf-8')) if config.exists() else {}
        echo=vehicle.get('antenna_filter',{}).get('retreat_echo')
        self.retreat=GuardedRetreat(echo);self.retreat_lease=-math.inf;self.retreat_identity=''
        self.retreat_scan=None;self.retreat_odom=None;self.last_retreat_odom=-math.inf
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
        self.dock_departure=-math.inf
        if self.get_parameter('site_mapping').value:
            self.create_subscription(Bool,'/site_dock_departure',self.departure,1)
        self.create_timer(.05, self.tick)

    def departure(self,msg):
        self.dock_departure=time.monotonic() if msg.data else -math.inf

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
        p=msg.pose.pose;t=msg.twist.twist
        values=(p.position.x,p.position.y,yaw(p.orientation),t.linear.x,t.angular.z)
        self.retreat_odom=values if all(math.isfinite(v) for v in values) else None
        self.last_retreat_odom=time.monotonic()

    def oem_state(self,msg):
        now=time.monotonic()
        if msg.data and not self.oem.active:self.last_motion=now
        self.oem.receive(now,msg.data)

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
        heading=yaw(t.rotation)+msg.angle_min
        return [(t.translation.x+r*math.cos(heading+i*msg.angle_increment),
                 t.translation.y+r*math.sin(heading+i*msg.angle_increment))
                for i,r in enumerate(msg.ranges) if math.isfinite(r) and msg.range_min<=r<=msg.range_max]

    def tick(self):
        self.alive.publish(Bool(data=True))
        now=time.monotonic();v = self.lease.output(now)
        dt=min(.12,now-self.last_tick);self.last_tick=now
        retreating=now-self.retreat_lease<=.20
        if retreating:
            if self.oem.active or self.disable_burst or any(v):
                self.retreat.status='canceled';v=(0.,0.,0.)
            else:
                points=self.scan_in_base() if now-self.last_scan<=.30 else ()
                odom=self.retreat_odom if now-self.last_retreat_odom<=.15 else None
                v=(self.retreat.tick(now,points,odom),0.,0.)
        elif self.retreat_identity and self.retreat.status in ('idle','backing'):
            self.retreat.status='canceled'
        self.retreat_status.publish(String(data=f'{self.retreat_identity}:{self.retreat.status}'))
        if self.oem.expired(now):self.disable_burst=10
        if self.disable_burst:
            self.ir_disable.publish(Int8(data=0));self.disable_burst-=1
            self.last_motion=now;v=(0.,0.,0.)
        if self.oem.active:v=(0.,0.,0.)
        if now-self.last_scan>.4:v=(0.,0.,0.)
        if any(v) and not retreating:
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
        self.last_linear=v[0]
        # An accepted cancellation says nothing about the wheels. Keep braking
        # until chassis feedback settles, even if the velocity source is gone.
        fresh=now-self.last_retreat_odom<=.25 and self.retreat_odom is not None
        moving=fresh and (abs(self.retreat_odom[3])>.025 or abs(self.retreat_odom[4])>.10)
        if not any(v) and not self.oem.active and moving:
            if self.uncommanded_since is None:self.uncommanded_since=now
            if now-self.uncommanded_since>=.65 and not self.hold_file.exists():
                self.hold_file.parent.mkdir(parents=True,exist_ok=True)
                self.hold_file.touch()
                self.get_logger().error('零速指令后车轮持续运动，已锁定底盘')
        else:
            self.uncommanded_since=None
        if self.hold_file.exists():
            self.ir_disable.publish(Int8(data=0))
            self.output.publish(Twist())
            self.last_linear=0.
            return
        if any(v):self.last_motion=now
        elif self.oem.active and now-self.last_motion>.5:
            return  # Do not continuously send velocity frames during OEM IR docking.
        msg = Twist();msg.linear.x, msg.linear.y, msg.angular.z = v
        self.output.publish(msg)


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
