"""Geometry and input validation for the mapping panel."""
from io import BytesIO
import math
import re
from uuid import uuid4

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt


VELOCITIES = {
    'forward': (.10, 0.), 'back': (-.07, 0.),
    'left': (0., .23), 'right': (0., -.23), 'stop': (0., 0.),
}


def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def dock_record(pose):
    from math import sin, cos
    x, y, yaw = pose
    angle = yaw-math.radians(15)
    return {'calibrated': True, 'robot_pose': dict(x=x, y=y, yaw=yaw),
            'charger_approach': dict(p_x=x+.6*cos(yaw), p_y=y+.6*sin(yaw),
                                     orien_z=sin(angle/2), orien_w=cos(angle/2)),
            'departure_distance': .30}


def home_pose(dock):
    return tuple(dock['robot_pose'][axis] for axis in ('x', 'y', 'yaw'))


def inside(bounds, pose, margin=0.):
    a, b, c, d = bounds
    return a+margin <= pose[0] <= b-margin and c+margin <= pose[1] <= d-margin


def rectangle(value):
    if not isinstance(value, list) or len(value) != 4 or any(
            type(v) not in (float,int) or not math.isfinite(v) or abs(v)>1000 for v in value):
        raise ValueError('请在地图上框选探索范围')
    a, b, c, d = value
    if not (.85 <= b-a <= 60 and .85 <= d-c <= 60):
        raise ValueError('探索范围的每边需在 0.85～60 米之间')
    return tuple(value)


def site_name(value):
    if not isinstance(value, str) or not (1 <= len(value.strip()) <= 40) or not value.strip().isprintable():
        raise ValueError('请输入 1～40 字的场地名称')
    return value.strip()


def drive_owner(value):
    if not isinstance(value,str) or not re.fullmatch(r'[a-f0-9-]{8,64}',value):
        raise ValueError('遥控会话无效')
    return value


class ManualDrive:
    def __init__(self):
        self.sequences = {}
        self.clear()

    def clear(self):
        self.owner = None
        self.until = 0.
        self.velocity = (0., 0.)

    def update(self, now, owner, direction, sequence):
        if direction not in VELOCITIES or type(sequence) is not int or not 0 <= sequence < 2**53:
            raise ValueError('遥控方向无效')
        if sequence <= self.sequences.get(owner,-1):
            return
        if len(self.sequences)>=32 and owner not in self.sequences:
            self.sequences.pop(next(iter(self.sequences)))
        self.sequences[owner]=sequence
        if direction == 'stop':
            if self.owner in (None,owner):
                self.clear()
        elif self.owner not in (None, owner) and now < self.until:
            raise ValueError('另一页面正在遥控')
        else:
            self.owner = owner
            self.until = now+.30
            self.velocity = VELOCITIES[direction]

    def output(self, now):
        return self.velocity if now < self.until else (0., 0.)


class GridFrame:
    def __init__(self, data, width, height, resolution, origin):
        self.width, self.height = width, height
        self.resolution = resolution
        self.x, self.y = origin
        self.grid = np.asarray(data, dtype=np.int8).reshape(height, width)
        self.id = uuid4().hex
        self.png = None
        self.clearance = None

    def view(self):
        return dict(map_id=self.id, width=self.width, height=self.height,
                    resolution=self.resolution, origin=[self.x,self.y,0.],
                    max_x=self.x+self.width*self.resolution, max_y=self.y+self.height*self.resolution)

    def image(self):
        if self.png is None:
            shades = np.zeros((*self.grid.shape,3), dtype=np.uint8)
            shades[:]=(109,130,135)
            shades[(self.grid>=0)&(self.grid<=25)]=(244,246,242)
            shades[self.grid>=65]=(20,35,49)
            # In the displayed top view, +X is up and +Y is left.
            stream = BytesIO()
            Image.fromarray(shades[::-1,::-1].transpose(1,0,2)).save(stream,format='PNG')
            self.png = stream.getvalue()
        return self.png

    def wall_distance(self):
        if self.clearance is None:
            self.clearance=distance_transform_edt(self.grid<65)*self.resolution
        return self.clearance

    def match(self, points):
        if not len(points) or not np.any(self.grid>=65):
            return 0, 0.
        x = np.floor((points[:,0]-self.x)/self.resolution).astype(int)
        y = np.floor((points[:,1]-self.y)/self.resolution).astype(int)
        valid=(0<=x)&(x<self.width)&(0<=y)&(y<self.height)
        x,y=x[valid],y[valid]
        known=self.grid[y,x]>=0
        count=int(known.sum())
        return count, float((self.wall_distance()[y[known],x[known]]<=.13).mean()) if count else 0.

    def frontier_candidates(self, bounds, robot):
        free=(self.grid>=0)&(self.grid<=25)
        if not np.any(self.grid<0):
            return []
        edge=distance_transform_edt(self.grid>=0)*self.resolution
        usable=free&(edge>=.22)&(edge<=.63)&(self.wall_distance()>=.23)
        iy,ix=np.nonzero(usable)
        x=self.x+(ix+.5)*self.resolution
        y=self.y+(iy+.5)*self.resolution
        a,b,c,d=bounds
        valid=(a+.20<=x)&(x<=b-.20)&(c+.20<=y)&(y<=d-.20)
        x,y=x[valid],y[valid]
        distance=np.hypot(x-robot[0],y-robot[1])
        result=[]
        for i in np.argsort(distance):
            p=float(x[i]),float(y[i])
            if distance[i]>.32 and all(math.dist(p,q)>.48 for q in result):
                result.append(p)
                if len(result)>=24:
                    break
        return result
