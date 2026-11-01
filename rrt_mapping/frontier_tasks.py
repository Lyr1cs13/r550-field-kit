"""Task scheduling only; no scan, IMU, TF or map publication/modification.

Factory RRT supplies frontiers. Factory Nav2 plans, drives and collision-checks
recoveries. The STM32 factory IR mode performs final docking.
"""
import json
import math
import time
from pathlib import Path

import sys
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'demo'))
from charge_confirmation import ChargeConfirmation, NativeDockingWatchdog

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist, PointStamped, PolygonStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from std_msgs.msg import Bool, Float32, UInt8, Int8, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from wheeltec_rrt_msg.msg import PointArray


class FrontierTasks(Node):
    def __init__(self):
        super().__init__('rrt_frontier_tasks')
        self.declare_parameter('low_voltage_return', 10.0)
        self.declare_parameter('restore_session', False)
        self.declare_parameter('start_paused', False)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.plan = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.speed_client = self.create_client(SetParametersAtomically, '/controller_server/set_parameters_atomically')
        self.speed_limit, self.speed_pending = 0.12, False
        self.clears = [self.create_client(ClearEntireCostmap, topic) for topic in (
            '/local_costmap/clear_entirely_local_costmap',
            '/global_costmap/clear_entirely_global_costmap')]
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 1)
        self.ir = self.create_publisher(Int8, '/robot_recharge_flag', 10)
        self.status = self.create_publisher(String, '/rrt_task_status', 10)
        self.create_subscription(PoseStamped, '/assigned_frontier_goal', self.proposal, 10)
        self.create_subscription(PointArray, '/filtered_goal_points', self.frontiers, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.clicked, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap, 1)
        self.create_subscription(PolygonStamped, '/local_costmap/published_footprint',
                                 lambda m: setattr(self, 'footprint', m), 1)
        self.create_subscription(Bool, '/robot_charging_flag', self.charge_feedback, qos_profile_sensor_data)
        self.create_subscription(UInt8, '/robot_red_flag', lambda m:(setattr(self,'red',int(m.data)),self.got('ir',int(m.data))), qos_profile_sensor_data)
        self.create_subscription(Float32, '/PowerVoltage', lambda m: setattr(self, 'voltage', float(m.data)), qos_profile_sensor_data)
        self.create_service(Trigger, '/rrt_pause', lambda r, s: self.command('pause', s))
        self.create_service(Trigger, '/rrt_resume', lambda r, s: self.command('explore', s))
        self.create_service(Trigger, '/rrt_return_to_dock', lambda r, s: self.command('return', s))
        self.create_service(Trigger, '/rrt_return_via_corridors', self.return_via_corridors)
        self.mode, self.phase = 'explore', 'waiting_for_five_points'
        self.active = None
        self.busy = False
        self.generation = 0
        self.target = None
        self.target_kind = None
        self.route, self.route_index = [], 0
        self.proposals = []
        self.blocked = []
        self.cm = None
        self.footprint = None
        self.points = []
        self.rectangle = None
        self.charging, self.red, self.voltage = False, 0, None
        self.dock_confirmation=ChargeConfirmation();self.dock_watchdog=None;self.telemetry={};self.wheel_travel=0.;self.last_wheel=0.;self.stationary_since=None;self.charge_moving_since=None
        self.create_subscription(Odometry,'/odom',self.wheel_feedback,qos_profile_sensor_data)
        self.create_subscription(Float32,'/robot_charging_current',lambda m:self.got('current',float(m.data)),qos_profile_sensor_data)
        self.low_since = None
        self.defer_until = 0.0
        self.navigation_since = 0.0
        self.docking_since = 0.0
        self.successes, self.failures = 0, 0
        self.last_log = None
        self.home = json.loads((Path(__file__).resolve().parent.parent/'state/mapping/dock_start.json').read_text())
        self.output = Path(__file__).resolve().parent.parent/'state/mapping/task_status.json'
        # Restore task state only when restarting this adapter in the same live
        # SLAM session. Never apply an old boundary to a newly started map.
        if self.get_parameter('restore_session').value:
            saved = json.loads((self.output.parent / 'boundary.json').read_text())
            self.points = [tuple(p) for p in saved['points'][:4]] + [(0.0, 0.0)]
            self.rectangle = tuple(saved['rectangle'])
            if self.output.exists():
                prior = json.loads(self.output.read_text())
                self.successes = prior.get('completed_goals', 0)
                self.failures = prior.get('failed_or_skipped', 0)
            self.phase = 'waiting_for_frontiers'
        if self.get_parameter('start_paused').value:
            self.mode, self.phase = 'pause', 'pause'
        self.create_timer(1.0, self.tick)

    def clicked(self, msg):
        if msg.header.frame_id.lstrip('/') != 'map' or len(self.points) >= 5:
            return
        self.points.append((msg.point.x, msg.point.y))
        if len(self.points) == 4:
            a, c = self.points[0], self.points[2]
            self.rectangle = (min(a[0], c[0]), max(a[0], c[0]), min(a[1], c[1]), max(a[1], c[1]))
            (self.output.parent / 'boundary.json').write_text(json.dumps({'points': self.points, 'rectangle': self.rectangle}))
        if len(self.points) == 5:
            self.phase = 'waiting_for_frontiers'

    def costmap(self, msg):
        self.cm = msg

    def got(self,key,value):self.telemetry[key]=(time.monotonic(),value)

    def wheel_feedback(self,msg):
        now=time.monotonic();twist=msg.twist.twist;dt=min(.2,max(0.,now-self.last_wheel));self.last_wheel=now
        self.wheel_travel+=abs(twist.linear.x)*dt
        self.stationary_since=(self.stationary_since or now) if abs(twist.linear.x)<.02 and abs(twist.angular.z)<.05 else None

    def finish_docking(self,status):
        self.ir.publish(Int8(data=0));self.cmd.publish(Twist());self.mode='pause';self.phase=status;self.dock_watchdog=None

    def charge_feedback(self, msg):
        self.charging = bool(msg.data)

    def proposal(self, msg):
        if self.mode != 'explore':
            return
        if msg.header.frame_id.lstrip('/') == 'map':
            self.offer(msg.pose.position.x, msg.pose.position.y, first=True)

    def frontiers(self, msg):
        if self.mode != 'explore':
            return
        for p in msg.points:
            self.offer(p.x, p.y)

    def offer(self, x, y, first=False):
        if not math.isfinite(x) or not math.isfinite(y):
            return
        if self.rectangle:
            a, b, c, d = self.rectangle
            if not (a <= x <= b and c <= y <= d):
                return
        for i, p in enumerate(self.proposals):
            if math.hypot(x-p[0], y-p[1]) < 0.25:
                if first:
                    self.proposals.insert(0, self.proposals.pop(i))
                return
        if first:
            self.proposals.insert(0, (x, y))
        else:
            self.proposals.append((x, y))
        self.proposals = self.proposals[:100]

    def blocked_goal(self, p):
        now = time.monotonic()
        self.blocked = [v for v in self.blocked if v[2] > now]
        return any(math.hypot(p[0]-v[0], p[1]-v[1]) < 0.35 for v in self.blocked)

    def cell(self, x, y):
        if self.cm is None:
            return None
        m = self.cm
        ix = math.floor((x-m.info.origin.position.x)/m.info.resolution)
        iy = math.floor((y-m.info.origin.position.y)/m.info.resolution)
        if not (0 <= ix < m.info.width and 0 <= iy < m.info.height):
            return -1
        return int(m.data[iy*m.info.width+ix])

    def pose(self, x, y, z=0.0, w=1.0):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = float(z), float(w)
        return p

    def footprint_needs_recovery(self):
        # Check the actual native footprint against the global planning map.
        # A corner can touch a lethal grid cell while the center is only in
        # ordinary inflation. Do not skip every goal in that situation: let
        # native Nav2 recover using its live local footprint collision checker.
        if self.footprint is None or self.cm is None:
            return False
        if self.footprint.header.frame_id.lstrip('/') != self.cm.header.frame_id.lstrip('/'):
            return False
        points = list(self.footprint.polygon.points)
        if not points:
            return False
        for a, b in zip(points, points[1:] + points[:1]):
            count = max(1, math.ceil(math.hypot(a.x-b.x, a.y-b.y) / (self.cm.info.resolution/2)))
            for i in range(count+1):
                f = i/count
                if self.cell(a.x+(b.x-a.x)*f, a.y+(b.y-a.y)*f) == 100:
                    return True
        return False

    def safe_frontier_approach(self, point):
        # Relocate the stopping target only. Paths may still traverse ordinary
        # inflation; sensor data and the costmap itself are never altered.
        if self.cm is None:
            return None
        cost = self.cell(*point)
        if cost is not None and 0 <= cost <= 45:
            return point
        m = self.cm
        best = None
        for dx in range(-7, 8):
            for dy in range(-7, 8):
                x, y = point[0]+dx*m.info.resolution, point[1]+dy*m.info.resolution
                shift = math.hypot(x-point[0], y-point[1])
                if shift > 0.35:
                    continue
                if self.rectangle:
                    a, b, c, d = self.rectangle
                    if not (a <= x <= b and c <= y <= d):
                        continue
                c = self.cell(x, y)
                if c is not None and 0 <= c <= 45:
                    score = shift+0.001*c
                    if best is None or score < best[0]:
                        best = (score, (x, y))
        return best[1] if best else None

    def command(self, mode, response):
        self.mode = mode
        self.generation += 1
        self.proposals.clear()
        if mode == 'explore':
            self.blocked.clear()
        if self.active:
            self.active.cancel_goal_async()
        if self.phase == 'docking':
            self.ir.publish(Int8(data=0))
        self.phase = 'canceling' if self.busy else mode
        if not self.busy:
            self.cmd.publish(Twist())
        response.success, response.message = True, 'requested ' + mode
        return response

    def return_via_corridors(self, request, response):
        try:
            route = json.loads((self.output.parent / 'return_route.json').read_text())['waypoints']
            if not route or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in route):
                raise ValueError('Invalid route')
            self.route, self.route_index = route, 0
            return self.command('route', response)
        except Exception as e:
            response.success, response.message = False, str(e)
            return response

    def clear_stale(self):
        for service in self.clears:
            if service.service_is_ready():
                service.call_async(ClearEntireCostmap.Request())

    def start(self, p, kind):
        self.target, self.target_kind = p, kind
        self.busy = True
        self.navigation_since = time.monotonic()
        self.phase = 'planning_' + kind
        generation = self.generation
        goal = ComputePathToPose.Goal()
        goal.goal = p
        goal.planner_id = 'GridBased'
        goal.use_start = False
        future = self.plan.send_goal_async(goal)
        future.add_done_callback(lambda f: self.plan_accepted(f, generation, p, kind))

    def plan_accepted(self, future, generation, p, kind):
        try:
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError('plan rejected')
            handle.get_result_async().add_done_callback(lambda f: self.plan_done(f, generation, p, kind))
        except Exception as e:
            self.busy = False
            self.phase = 'plan_error'
            self.get_logger().error(str(e))

    def plan_done(self, future, generation, p, kind):
        if generation != self.generation:
            self.busy = False
            return
        try:
            result = future.result()
            valid = result.status == GoalStatus.STATUS_SUCCEEDED and len(result.result.path.poses) > 1
        except Exception:
            valid = False
        # A start in inscribed space needs Nav2's collision-checked recovery,
        # not repeated rejection of every frontier during preflight.
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time())
            start_cost = self.cell(t.transform.translation.x, t.transform.translation.y)
        except Exception:
            start_cost = None
        if not valid and kind == 'explore' and start_cost not in (99, 100) and not self.footprint_needs_recovery():
            self.blocked.append((p.pose.position.x, p.pose.position.y, time.monotonic()+90))
            self.busy = False
            self.failures += 1
            self.phase = 'skipping_unreachable_frontier'
            self.defer_until = time.monotonic()+2
            return
        self.phase = 'navigating_' + kind
        goal = NavigateToPose.Goal()
        p.header.stamp = self.get_clock().now().to_msg()
        goal.pose = p
        tree = 'navigation_recovery_south.xml' if kind in ('explore', 'route') else 'dock_navigation.xml'
        goal.behavior_tree = str(Path(__file__).resolve().with_name(tree))
        self.nav.send_goal_async(goal).add_done_callback(lambda f: self.nav_accepted(f, generation, kind))

    def nav_accepted(self, future, generation, kind):
        try:
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError('navigation rejected')
            self.active = handle
            if generation != self.generation:
                handle.cancel_goal_async()
            handle.get_result_async().add_done_callback(lambda f: self.nav_done(f, generation, kind))
        except Exception as e:
            self.busy = False
            self.phase = 'nav_error'
            self.get_logger().error(str(e))

    def nav_done(self, future, generation, kind):
        self.active = None
        self.busy = False
        if generation != self.generation:
            return
        try:
            success = future.result().status == GoalStatus.STATUS_SUCCEEDED
        except Exception:
            success = False
        if not success:
            self.failures += 1
            self.clear_stale()
            self.defer_until = time.monotonic()+3
            if kind in ('return', 'route'):
                self.mode, self.phase = 'pause', 'return_failed' if kind == 'return' else 'corridor_route_failed'
            else:
                self.blocked.append((self.target.pose.position.x, self.target.pose.position.y, time.monotonic()+90))
                self.phase = 'recovery_failed_try_next_frontier'
            return
        self.successes += 1
        if kind == 'route':
            self.route_index += 1
            self.phase = 'corridor_waypoint_reached'
            self.defer_until = time.monotonic()+(4 if self.route_index == 4 else 1)
            return
        if kind == 'return':
            if self.red > 0:
                self.phase = 'docking'
                self.docking_since = time.monotonic()
            else:
                self.mode, self.phase = 'pause', 'arrived_no_ir_signal'
        else:
            self.blocked.append((self.target.pose.position.x, self.target.pose.position.y,
                                 time.monotonic()+120))
            self.phase = 'waiting_for_frontiers'
            self.defer_until = time.monotonic()+2

    def tick(self):
        now = time.monotonic()
        try:
            tf = self.tf.lookup_transform('map', 'base_footprint', Time())
            q = tf.transform.rotation
            body = {'x': tf.transform.translation.x, 'y': tf.transform.translation.y,
                    'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}
        except Exception:
            body = None
        desired_speed = 0.10 if self.mode == 'return' else 0.12
        # The threshold is excluded by native KeepoutFilter; no speed boost.
        self.update_speed(desired_speed)
        state = {'mode': self.mode, 'phase': self.phase, 'points': len(self.points),
                 'completed_goals': self.successes, 'failed_or_skipped': self.failures,
                 'pending': len(self.proposals), 'voltage': self.voltage,
                 'low_voltage_return': self.get_parameter('low_voltage_return').value,
                 'speed_limit': self.speed_limit,
                 'target': [self.target.pose.position.x, self.target.pose.position.y] if self.target else None,
                 'route_index': self.route_index, 'route_points': len(self.route),
                 'charging': self.charging, 'ir': self.red, 'robot': body,
                 'start_cost': self.cell(body['x'], body['y']) if body else None}
        self.status.publish(String(data=json.dumps(state)))
        self.output.write_text(json.dumps(state, indent=2))
        stamp = (self.mode, self.phase, self.successes, self.failures)
        if stamp != self.last_log:
            self.get_logger().info(json.dumps(state))
            self.last_log = stamp
        low_voltage = self.get_parameter('low_voltage_return').value
        if self.voltage is not None and self.voltage < low_voltage and not self.charging:
            self.low_since = self.low_since or now
            if now-self.low_since > 20 and self.mode == 'explore':
                self.command('return', Trigger.Response())
                self.get_logger().warn(f'Voltage below {low_voltage:.1f} V for 20 s: requesting dock return.')
        else:
            self.low_since = None
        if self.busy:
            if self.active and now-self.navigation_since > 300:
                self.active.cancel_goal_async()
                self.phase = 'timeout_canceling'
            return
        if self.mode == 'pause' or now < self.defer_until:
            return
        if self.phase == 'docking':
            stopped=self.stationary_since is not None and now-self.stationary_since>=.5
            current=self.telemetry.get('current',(0.,0.));current_value=current[1] if now-current[0]<2 else None
            if self.dock_confirmation.update(now,charging=self.charging,stationary=stopped,current=current_value,voltage=self.voltage):
                self.finish_docking('docked');return
            if self.dock_watchdog is None:
                self.dock_watchdog=NativeDockingWatchdog();self.dock_watchdog.reset(now,self.wheel_travel)
            near=bool(body and math.hypot(body['x']-self.home['robot_pose']['x'],body['y']-self.home['robot_pose']['y'])<.18)
            fresh=now-self.last_wheel<2 and now-self.telemetry.get('ir',(0.,0))[0]<2
            error=self.dock_watchdog.update(now,travel=self.wheel_travel,near=near,fresh=fresh)
            electrical=current_value is not None and current_value>=.1
            self.charge_moving_since=(self.charge_moving_since or now) if electrical and not stopped else None
            if error or (self.charge_moving_since and now-self.charge_moving_since>2.5):self.finish_docking(error or 'charging_wheels_not_stopped');return
            self.ir.publish(Int8(data=1 if self.red>0 else 0));return
        if not self.nav.server_is_ready() or not self.plan.server_is_ready() or body is None or self.cm is None:
            return
        if self.mode == 'return':
            d = self.home['charger_approach']
            self.start(self.pose(d['p_x'], d['p_y'], d['orien_z'], d['orien_w']), 'return')
            return
        if self.mode == 'route':
            if self.route_index >= len(self.route):
                self.mode, self.phase = 'return', 'return'
                return
            p = self.safe_frontier_approach(self.route[self.route_index])
            if p is None:
                self.mode, self.phase = 'pause', 'corridor_target_blocked'
                return
            bearing = math.atan2(p[1]-body['y'], p[0]-body['x'])
            self.start(self.pose(*p, math.sin(bearing/2), math.cos(bearing/2)), 'route')
            return
        if len(self.points) < 5:
            return
        # Prefer nearby native frontiers to repeatedly traversing the room.
        self.proposals.sort(key=lambda p: math.hypot(p[0]-body['x'], p[1]-body['y']))
        while self.proposals:
            p = self.proposals.pop(0)
            if self.blocked_goal(p):
                continue
            p = self.safe_frontier_approach(p)
            if p is None or self.blocked_goal(p):
                continue
            if math.hypot(p[0]-body['x'], p[1]-body['y']) < 0.25:
                continue
            bearing = math.atan2(p[1]-body['y'], p[0]-body['x'])
            self.start(self.pose(*p, math.sin(bearing/2), math.cos(bearing/2)), 'explore')
            break

    def update_speed(self, value):
        if self.speed_pending or value == self.speed_limit or not self.speed_client.service_is_ready():
            return
        req = SetParametersAtomically.Request()
        req.parameters = [Parameter('FollowPath.vx_max', value=value).to_parameter_msg(),
                          Parameter('FollowPath.vx_std', value=0.10 if value > 0.12 else 0.06).to_parameter_msg()]
        self.speed_pending = True
        def done(future):
            self.speed_pending = False
            try:
                result = future.result().result
                if result.successful:
                    self.speed_limit = value
                    self.get_logger().info(f'Native controller speed limit: {value:.2f} m/s')
                else:
                    self.get_logger().error(result.reason)
            except Exception as e:
                self.get_logger().error(str(e))
        self.speed_client.call_async(req).add_done_callback(done)


def main():
    rclpy.init()
    node = FrontierTasks()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            if node.active:
                f = node.active.cancel_goal_async()
                rclpy.spin_until_future_complete(node, f, timeout_sec=2)
            node.ir.publish(Int8(data=0))
            for _ in range(5):
                node.cmd.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.05)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
