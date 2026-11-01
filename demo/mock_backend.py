"""Explicit offline rehearsal. Imports no ROS and cannot control a robot."""
import math
import time
import uuid


class MockBackend:
    mode = 'offline'
    def __init__(self, grid, dock, clock=time.monotonic, speed=1.0):
        self.grid, self.clock, self.speed = grid, clock, speed
        home = dock['robot_pose'];self.pose = [home['x'], home['y'], home['yaw']]
        d = dock['charger_approach']
        self.dock_pose = list(self.pose)
        self.departure_distance = float(dock.get('departure_distance',.30))
        self.dock_approach = {'x': d['p_x'], 'y': d['p_y'], 'yaw': 2*math.atan2(d['orien_z'], d['orien_w'])}
        self.ops = {};self.active = None;self.path = [];self.charging = True
        self.confirmed = False;self.docking = None;self.docking_error = None;self.last = clock()

    def navigate(self, point, kind):
        identity = uuid.uuid4().hex
        try:
            path = self.grid.offline_path(self.pose, [point['x'], point['y']])
            self.ops[identity] = {'status': 'navigating', 'goal': dict(point), 'path': path, 'cursor': 0, 'kind': kind}
            self.path = path;self.active = identity;self.charging = False
        except ValueError as e:
            self.ops[identity] = {'status': 'failed', 'error': str(e)}
        return identity

    def cancel(self, identity):
        if identity in self.ops and self.ops[identity]['status'] not in ('failed','succeeded','canceled'):
            self.ops[identity]['status'] = 'canceled'
        if identity == self.active:
            self.active = None;self.path = []

    def operation(self, identity):
        return self.ops[identity]

    def stop(self):
        if self.active:
            self.cancel(self.active)
        self.docking = None;self.path = []

    def begin_docking(self):
        self.docking = self.clock();self.docking_error = None

    def initialize_pose(self, pose, known_dock=False):
        self.pose = [float(v) for v in pose];self.confirmed = False
        self.charging = math.hypot(self.pose[0]-self.dock_pose[0],self.pose[1]-self.dock_pose[1]) < .1

    def confirm_localization(self):
        self.confirmed = True

    def tick(self):
        now = self.clock();travel = min(.5, max(0, now-self.last))*self.speed;self.last = now
        if self.active:
            op = self.ops[self.active];path = op['path']
            while travel > 0 and op['cursor'] < len(path):
                target = path[op['cursor']];dx, dy = target[0]-self.pose[0], target[1]-self.pose[1]
                distance = math.hypot(dx, dy)
                if distance > 1e-8:
                    self.pose[2] = math.atan2(dy, dx)
                if distance <= travel:
                    self.pose[:2] = target;travel -= distance;op['cursor'] += 1
                else:
                    self.pose[0] += dx*travel/distance;self.pose[1] += dy*travel/distance;travel = 0
            if op['cursor'] >= len(path):
                self.pose = [op['goal']['x'], op['goal']['y'], op['goal']['yaw']]
                op['status'] = 'succeeded';self.active = None;self.path = []
        if self.docking is not None and now-self.docking >= 2:
            self.charging = True;self.pose = list(self.dock_pose);self.docking = None

    def snapshot(self):
        return {'mode': 'offline', 'ready': True, 'readiness': '离线演练，不连接或控制小车',
                'localization_confirmed': self.confirmed, 'pose': list(self.pose),
                'charging': self.charging, 'voltage': 12.0, 'current': 1.68 if self.charging else 0,
                'ir': 2 if self.charging or self.docking is not None else 0,
                'stationary': self.active is None and self.docking is None,
                'path': self.path, 'laser': [], 'map_matches': True,
                'simulation_speed': self.speed}
