"""Track a scene's explicit aisle legs through the existing Nav2 motion chain."""
import math
from pathlib import Path as FilePath
import time

import yaml
from nav2_msgs.action import FollowPath
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import Path
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter

from arrival_policy import StationCheck
from corridor_route import CorridorRoute, CurveLeg, Leg, rollout_turn
from directed_route import angle_error, departure_goal, departure_leg, straight_approach_speed
from ros_backend import RosBackend


NO_MOTION_STATES = {
    'corridor_prepare', 'corridor_validate', 'corridor_configure', 'corridor_stopped', 'corridor_turn_done',
}
CONTROLLER = 'CorridorTrack'
BEND_CONTROLLER = 'CorridorBend'


class CorridorBackend(RosBackend):
    def __init__(self, grid, dock):
        self.corridors = CorridorRoute.load(grid)
        path = FilePath(__file__).resolve().parents[1] / 'config/vehicle.yaml'
        vehicle = yaml.safe_load(path.read_text(encoding='utf-8')) if path.is_file() else {}
        tuning = (vehicle or {}).get('corridor_drive', {})
        self.rollout_bias = {reverse: math.radians(float(tuning.get(key, 0.))) for reverse, key in
                             ((False, 'forward_rollout_yaw_deg'), (True, 'reverse_rollout_yaw_deg'))}
        if not all(0. <= value <= math.radians(15.) for value in self.rollout_bias.values()):
            raise ValueError('走廊起步转角标定应在 0 到 15 度之间')
        super().__init__(grid, dock)
        departure = float(dock.get('departure_distance', self.departure_distance))
        if .25 <= departure < self.departure_distance:
            self.departure_distance = departure

    def guided(self, point, kind):
        fallback = getattr(self,'reference_retry',None)
        return (kind in ('waypoint', 'move') and self.corridors.includes(point)
                and (not fallback or fallback[:2] != (point['x'],point['y']) or time.monotonic() >= fallback[2]))

    def preview_route(self, point, kind, health):
        if not self.guided(point, kind):
            return super().preview_route(point, kind, health)
        pose = tuple(health['pose'])
        prefix = [pose[:2]]
        if kind == 'move' and any(health.get(key) for key in
                                 ('charging', 'dock_position_confirmed', 'dock_departure_required')):
            goal = departure_goal(self.dock_pose, self.departure_distance)
            path = departure_leg(pose, self.dock_pose, goal, staging=True)
            prefix.extend((p['x'], p['y']) for p in path['poses'])
            pose = (*prefix[-1], pose[2])
        routes = self.corridors.motion_routes(pose, point)
        if not routes:
            return super().preview_route(point, kind, health)
        prefix.extend(p for leg in routes[0] for p in leg.samples()[1:])
        return prefix

    def send_fas_line(self, identity):
        op = self.ops[identity]
        if op.get('corridor_fallback') or (op['kind']=='waypoint' and self.corridors.includes(op['goal'])):
            return False
        return super().send_fas_line(identity)

    def plan_navigation(self, identity):
        op = self.ops[identity]
        if op.get('corridor_fallback') or not self.guided(op['goal'], op['kind']):
            return super().plan_navigation(identity)
        op.update(corridor=True, sequential=True, corridor_legs=None, corridor_check=None, corridor_rollout=0.)
        op.setdefault('stop_xy_tolerance',self.arrival.get('final_position_tolerance', .05))
        self.corridor_state(op, 'corridor_prepare')

    def corridor_state(self, op, state, timeout=35.):
        op.update(status=state, started=time.monotonic(), timeout=timeout, handle=None)

    def corridor_retry(self, op, reason):
        goal = op['goal']
        self.reference_retry = (goal['x'],goal['y'],time.monotonic()+30.)
        op.update(corridor=False,corridor_fallback=True,corridor_legs=None)
        self.path = []
        self.retry_arrival(op, reason+'，按实时地图重新规划')

    def corridor_lease_ok(self, op, health, path):
        if self.fleet_zone and self.fleet_motion_reason(path):
            return False
        fas = getattr(self, 'fas_lease', None)
        return not (fas and fas.owner) or fas.allows('move', health, path)

    def update(self):
        super().update()
        with self.lock:
            identity = self.active
            op = self.ops[identity] if identity else {}
            if not op.get('corridor') or op.get('cancel'):
                return
            now = time.monotonic()
            state = op['status']
            if state == 'navigating' and self.pose:
                leg = op['corridor_leg']
                if leg.progress(self.pose) >= leg.length - .025:
                    self.brake_corridor(op, 'position_settling')
                elif leg.deviation(self.pose) > .18:
                    if now - op.setdefault('corridor_off_since', now) >= .8:
                        self.brake_corridor(op, 'corridor_rejoin')
                else:
                    op.pop('corridor_off_since', None)
                return
            if state in ('corridor_validate', 'corridor_configure'):
                if now-op['started'] > 4.:
                    self.corridor_retry(op, '参考线实时复核超时，停车后重试')
                return
            if state not in NO_MOTION_STATES:
                return
            health = self.snapshot()
            if not (health['ready'] and health['localization_confirmed'] and health['stationary']
                    and self.stationary_since is not None and now-self.stationary_since >= .7
                    and self.pose is not None and health['pose_age_s'] is not None
                    and health['pose_age_s'] <= .4 and now-self.last_scan < .35):
                return
            if state == 'corridor_stopped':
                intermediate = len(op['corridor_legs']) > 1 and isinstance(op['corridor_legs'][1], CurveLeg)
                tolerance = .055 if intermediate else op['stop_xy_tolerance']
                if math.dist(self.pose[:2], op['corridor_leg'].end) > tolerance:
                    self.corridor_retry(op, '参考线停稳后的位置超差，保持当前目标')
                    return
                op['corridor_legs'] = op['corridor_legs'][1:]
                op['corridor_turns'] = 0
                op['corridor_rollout'] = 0.
            self.prepare_corridor_leg(identity, op, health)

    def brake_corridor(self, op, next_stage):
        op.update(cancel=True, status='canceling', settle_after_cancel=next_stage)
        if op['handle']:
            op['handle'].cancel_goal_async()
        self.zero_motion()

    def prepare_corridor_leg(self, identity, op, health):
        if op['corridor_legs'] is None:
            goal = op['goal']
            at_position = math.dist(self.pose[:2], (goal['x'], goal['y'])) <= op['stop_xy_tolerance']
            routes = [()] if at_position else self.corridors.motion_routes(self.pose, goal)
            available = [route for route in routes if self.corridor_lease_ok(
                op, health, [self.pose[:2]] + [p for leg in route for p in leg.samples()[1:]])]
            if not available:
                self.corridor_retry(op, '当前没有满足地图和通行许可的走廊参考线')
                return
            op.update(corridor_legs=available[0], corridor_turns=0)
        if not op['corridor_legs']:
            self.finish_corridor_goal(op)
            return

        original = op['corridor_legs'][0]
        if math.dist(self.pose[:2], original.end) <= .017:
            op['corridor_legs'] = op['corridor_legs'][1:]
            op['corridor_turns'] = 0
            op['corridor_rollout'] = 0.
            self.corridor_state(op, 'corridor_prepare')
            return
        start = tuple(self.pose[:2])
        if math.dist(start, original.start) > .095:
            self.corridor_retry(op, '转向后偏离连接点，停车复核接入路线')
            return
        if isinstance(original, CurveLeg):
            target = {'x': original.end[0], 'y': original.end[1], 'yaw': original.end_yaw}
            leg = self.corridors.station_curve(self.pose, target, original.reverse)
            if leg is None:
                self.corridor_retry(op, '停稳位置已改变，重新核对走廊曲线')
                return
        else:
            if not self.corridors.entry_clear(start, original.end):
                self.corridor_retry(op, '转向后偏离连接点，停车复核接入路线')
                return
            bearing = math.atan2(original.end[1]-start[1], original.end[0]-start[0])
            leg = Leg(start, original.end, math.remainder(bearing + (math.pi if original.reverse else 0.), math.tau),
                      original.reverse)
        remaining = [leg, *op['corridor_legs'][1:]]
        path = [start] + [p for part in remaining for p in part.samples()[1:]]
        if not self.leg_validator.service_is_ready() or not self.corridor_lease_ok(op, health, path):
            self.corridor_retry(op, '等待实时净空和通行许可')
            return
        self.path = path
        end = self.corridors.lookahead_end(leg)
        if end != leg.end and not self.corridor_lease_ok(op, health, [start, leg.end, end]):
            end = leg.end
        self.validate_corridor_leg(identity, op, leg, end)

    def validate_corridor_leg(self, identity, op, leg, end):
        route = Path()
        route.header.frame_id = 'map'
        route.header.stamp = self.get_clock().now().to_msg()
        if isinstance(leg, CurveLeg):
            poses = list(leg.poses)
            if end != leg.end:
                tail = Leg(leg.end, end, leg.end_yaw, leg.reverse)
                poses.extend((x, y, leg.end_yaw) for x, y in tail.samples()[1:])
        else:
            native = Leg(leg.start, end, leg.yaw, leg.reverse)
            poses = [(x, y, leg.yaw) for x, y in native.samples()]
        route.poses = [self.stamped({'x': x, 'y': y, 'yaw': yaw}) for x, y, yaw in poses]
        request = IsPathValid.Request()
        request.path = route
        self.corridor_state(op, 'corridor_validate')
        attempt = object()
        op.update(corridor_attempt=attempt, corridor_leg=leg, directed_path=route, corridor_lookahead=end != leg.end)
        self.leg_validator.call_async(request).add_done_callback(
            lambda future: self.corridor_validated(future, identity, attempt))

    def corridor_validated(self, future, identity, attempt):
        with self.lock:
            op = self.ops[identity]
            if (self.active != identity or op['status'] != 'corridor_validate' or
                    op.get('corridor_attempt') is not attempt):
                return
            try:
                leg = op['corridor_leg']
                if not future.result().is_valid:
                    if op['corridor_lookahead']:
                        self.validate_corridor_leg(identity, op, leg, leg.end)
                    else:
                        self.corridor_retry(op, '实时障碍占用参考线，等待或检查另一条连接线')
                    return
                error = angle_error(leg.yaw, self.pose[2])
                predicted = angle_error(leg.yaw, self.pose[2] + op['corridor_rollout'])
                if min(abs(error), abs(predicted)) > math.radians(10):
                    turn, residual = rollout_turn(error, self.rollout_bias[leg.reverse])
                    op['corridor_rollout'] = residual
                    self.corridor_spin(op, turn)
                    return
                request = SetParametersAtomically.Request()
                controller = BEND_CONTROLLER if isinstance(leg, CurveLeg) else CONTROLLER
                request.parameters = [Parameter(controller + '.desired_linear_vel',
                                                value=self.corridor_speed(leg)).to_parameter_msg()]
                if controller == BEND_CONTROLLER:
                    request.parameters.append(Parameter(controller + '.lookahead_dist',
                                                        value=self.corridor_bend_lookahead).to_parameter_msg())
                self.corridor_state(op, 'corridor_configure')
                self.controller_parameters.call_async(request).add_done_callback(
                    lambda result: self.corridor_configured(result, identity, attempt))
            except Exception as exc:
                self.corridor_retry(op, f'参考线复核未通过：{exc}')

    def corridor_spin(self, op, error):
        if op['corridor_turns'] >= 2:
            self.corridor_retry(op, '连接点转向未收敛，保持停车复核')
            return
        if not self.spin_action.server_is_ready() or not self.snapshot()['stationary']:
            self.corridor_retry(op, '等待车身停稳和转向服务')
            return
        op['corridor_turns'] += 1
        op['heading_attempts'] += 1
        op.setdefault('first_heading_at', time.monotonic())
        self.start_fas_spin(op, error, 'corridor_turn_done')

    def corridor_speed(self, leg):
        distance = leg.length - leg.progress(self.pose)
        limit = .040 if self.corridors.clearance_at(leg.start) < self.corridors.gap else (.070 if leg.reverse else .085)
        if isinstance(leg, CurveLeg):
            limit = min(.055, limit)
        return min(limit, straight_approach_speed(distance, reverse=leg.reverse))

    def snapshot(self):
        health = super().snapshot()
        op = self.ops.get(self.active, {})
        leg = op.get('corridor_leg')
        if op.get('corridor') and leg and op['status'] not in ('succeeded', 'failed', 'canceled'):
            health['corridor'] = {'stage': op['status'], 'start': leg.start, 'end': leg.end,
                                  'reverse': leg.reverse, 'remaining_legs': len(op.get('corridor_legs') or ()),
                                  'rollout_yaw_deg': math.degrees(op.get('corridor_rollout', 0.)),
                                  'deviation_m': leg.deviation(self.pose) if self.pose else None}
        return health

    def corridor_configured(self, future, identity, attempt):
        with self.lock:
            op = self.ops[identity]
            if (self.active != identity or op['status'] != 'corridor_configure' or
                    op.get('corridor_attempt') is not attempt):
                return
            try:
                result = future.result().result
                if not result.successful:
                    raise ValueError(result.reason)
                leg = op['corridor_leg']
                health = self.snapshot()
                if not (health['stationary'] and health['ready'] and health['localization_confirmed']
                        and self.pose and math.dist(self.pose[:2], leg.start) <= .045):
                    raise ValueError('发车前位姿已变化，等待重新复核')
                goal = FollowPath.Goal()
                goal.path = op['directed_path']
                goal.controller_id = BEND_CONTROLLER if isinstance(leg, CurveLeg) else CONTROLLER
                goal.goal_checker_id = 'directed_position_checker'
                self.speed_limits[goal.controller_id] = self.corridor_speed(leg)
                op.pop('corridor_off_since', None)
                self.corridor_state(op, 'navigating', leg.length/.045+25.)
                op.update(controller_id=goal.controller_id, explicit_path=True, reverse=leg.reverse,
                          travel_direction='reverse' if leg.reverse else 'forward')
                self.follow.send_goal_async(goal).add_done_callback(lambda f: self.nav_accepted(f, identity))
            except Exception as exc:
                self.corridor_retry(op, f'跟踪参考线前停止：{exc}')

    def stop_at_waypoint(self, op, distance, now):
        if op.get('corridor'):
            return False
        return super().stop_at_waypoint(op, distance, now)

    def request_speed(self, speed, controller_id='FollowPath'):
        op = self.ops.get(self.active, {})
        if (controller_id in (CONTROLLER, BEND_CONTROLLER) and op.get('corridor') and op['status'] == 'navigating'
                and self.pose is not None):
            speed = self.corridor_speed(op['corridor_leg'])
            if abs(speed - self.speed_limits.get(controller_id, 0.)) < .006:
                return
        return super().request_speed(speed, controller_id)

    def operation(self, identity):
        with self.lock:
            result = super().operation(identity)
            op = self.ops[identity]
            if op.get('corridor_legs') and op.get('spin_resume') == 'corridor_turn_done':
                result['stage'] = 'approach'
            return result

    def finish_native_cancellation(self, op):
        if op.get('corridor'):
            stage = op.get('settle_after_cancel')
            if stage in ('position_settling', 'corridor_rejoin'):
                op.pop('settle_after_cancel')
                op['cancel'] = False
                if stage == 'position_settling':
                    self.corridor_state(op, 'corridor_stopped')
                else:
                    self.corridor_retry(op, '实际轨迹持续偏离参考线，停车复核')
                return
        super().finish_native_cancellation(op)

    def heading_needs_settle(self, op):
        if op.get('corridor') and op.get('spin_resume') == 'corridor_turn_done':
            return abs(angle_error(op['spin_yaw'], self.pose[2])) <= math.radians(3)
        return super().heading_needs_settle(op)

    def nav_done(self, future, identity):
        with self.lock:
            op = self.ops[identity]
            if not op.get('corridor'):
                return super().nav_done(future, identity)
            try:
                result = future.result()
                if op['cancel']:
                    self.finish_native_cancellation(op)
                elif result.status == 4:
                    self.corridor_state(op, 'corridor_stopped')
                else:
                    self.corridor_retry(op, '跟踪因实时障碍或进度保护停止，保留目标')
            except Exception as exc:
                self.corridor_retry(op, f'路线跟踪等待恢复：{exc}')
            self.zero_motion()

    def finish_corridor_goal(self, op):
        goal = op['goal']
        distance = math.dist(self.pose[:2], (goal['x'], goal['y']))
        error = angle_error(goal['yaw'], self.pose[2])
        if op['corridor_check'] is None:
            op['corridor_check'] = StationCheck()
            self.request_stationary_amcl(time.monotonic())
        stage = op['corridor_check'].observe(
            self.pose_stamp, self.last_scan, distance, error, op['stop_xy_tolerance'], op['yaw_tolerance'])
        if stage == 'wait':
            return
        op['corridor_check'] = None
        if stage == 'position':
            self.corridor_retry(op, '最终转向后位置超差，保留目标重新对准')
        elif stage == 'heading':
            self.corridor_spin(op, error)
        else:
            self.complete_waypoint(op, time.monotonic(), distance, error)

    def cancel(self, identity):
        with self.lock:
            op = self.ops[identity]
            if op.get('corridor') and op['status'] in NO_MOTION_STATES:
                op.update(cancel=True, status='canceled')
                self.zero_motion()
                return
            super().cancel(identity)
