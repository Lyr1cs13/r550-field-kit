"""Map-bound routes for aisles, with a safe turning point at every leg boundary."""
from dataclasses import dataclass, field
import itertools
import json
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from directed_route import angle_error
from motion_health import PADDED_FRONT, PADDED_REAR, PADDED_HALF_WIDTH


def rollout_turn(error, expected_rollout):
    """Leave room for a measured heading change when this chassis starts rolling."""
    residual = math.copysign(expected_rollout, error)
    command = math.copysign(max(math.radians(4), abs(error) - expected_rollout), error)
    return command, residual


@dataclass(frozen=True)
class Leg:
    start: tuple
    end: tuple
    yaw: float
    reverse: bool

    @property
    def length(self):
        return math.dist(self.start, self.end)

    @property
    def end_yaw(self):
        return self.yaw

    def samples(self, spacing=.04):
        count = max(2, math.ceil(self.length / spacing))
        x, y = self.start
        dx, dy = self.end[0] - x, self.end[1] - y
        return [(x + dx*i/count, y + dy*i/count) for i in range(count+1)]

    def progress(self, point):
        dx, dy = self.end[0] - self.start[0], self.end[1] - self.start[1]
        return ((point[0]-self.start[0])*dx + (point[1]-self.start[1])*dy) / self.length

    def deviation(self, point):
        dx, dy = self.end[0] - self.start[0], self.end[1] - self.start[1]
        t = min(1., max(0., self.progress(point)/self.length))
        return math.dist(point[:2], (self.start[0]+t*dx, self.start[1]+t*dy))


@dataclass(frozen=True)
class EscapeLeg(Leg):
    """Short move away from an edge along the current body heading."""


@dataclass(frozen=True)
class CurveLeg:
    poses: tuple
    reverse: bool
    segments: tuple = field(init=False, repr=False)
    length: float = field(init=False)

    def __post_init__(self):
        segments, distance = [], 0.
        for a, b in zip(self.poses, self.poses[1:]):
            length = math.dist(a[:2], b[:2])
            if length > 1e-6:
                segments.append((a[:2], b[:2], distance, length))
                distance += length
        object.__setattr__(self, 'segments', tuple(segments))
        object.__setattr__(self, 'length', distance)

    @property
    def start(self):
        return self.poses[0][:2]

    @property
    def end(self):
        return self.poses[-1][:2]

    @property
    def yaw(self):
        return self.poses[0][2]

    @property
    def end_yaw(self):
        return self.poses[-1][2]

    def samples(self):
        return [pose[:2] for pose in self.poses]

    def projection(self, point):
        x, y = point[:2]
        best, travelled = math.inf, 0.
        for (ax, ay), (bx, by), distance, length in self.segments:
            t = ((x-ax)*(bx-ax)+(y-ay)*(by-ay))/length**2
            t = max(0., min(1., t))
            error = math.hypot(x-ax-t*(bx-ax), y-ay-t*(by-ay))
            if error < best:
                best, travelled = error, distance+t*length
        return travelled, best

    def progress(self, point):
        return self.projection(point)[0]

    def deviation(self, point):
        return self.projection(point)[1]


def tangent_entry(pose, goal, reverse, *, minimum_run=.12):
    """A tangent circle followed by a straight run along the goal's aisle."""
    x, y, yaw = pose
    flip = math.pi if reverse else 0.
    start, end = yaw+flip, goal['yaw']+flip
    turn = angle_error(end, start)
    if not math.radians(18) <= abs(turn) <= math.radians(110):
        return None
    side = math.copysign(1., turn)
    dx, dy = goal['x']-x, goal['y']-y
    ux, uy = math.cos(end), math.sin(end)
    cx = side*(math.sin(end)-math.sin(start))
    cy = side*(math.cos(start)-math.cos(end))
    radius = (dx*uy-dy*ux)/(cx*uy-cy*ux)
    run = (dx-radius*cx)*ux+(dy-radius*cy)*uy
    if not .16 <= radius <= .75 or run < minimum_run:
        return None
    steps = max(2, math.ceil(radius*abs(turn)/.022))
    poses = [(
        x+radius*side*(math.sin(start+turn*i/steps)-math.sin(start)),
        y+radius*side*(math.cos(start)-math.cos(start+turn*i/steps)),
        math.remainder(yaw+turn*i/steps, math.tau),
    ) for i in range(steps+1)]
    steps = math.ceil(run/.032)
    tail_x, tail_y = poses[-1][:2]
    poses.extend((tail_x+i*run*ux/steps, tail_y+i*run*uy/steps, goal['yaw']) for i in range(1, steps))
    poses.append((goal['x'], goal['y'], goal['yaw']))
    return CurveLeg(tuple(poses), reverse)


def tangent_alignment(pose, goal, reverse):
    """Connect nearby parallel headings, including a small lateral correction."""
    x, y, yaw = pose
    flip = math.pi if reverse else 0.
    target_yaw = goal['yaw']
    chord = complex(goal['x']-x, goal['y']-y)
    a = complex(math.cos(yaw+flip), math.sin(yaw+flip))
    b = complex(math.cos(target_yaw+flip), math.sin(target_yaw+flip))
    along = (chord/b).real
    if (not .15 <= along <= .85 or abs((chord/b).imag) > .14
            or abs(angle_error(target_yaw, yaw)) > math.radians(22)):
        return None
    handle = along/3
    p, q, r, s = 0j, handle*a, chord-handle*b, chord
    steps = math.ceil(abs(chord)/.02)
    poses = []
    for i in range(steps+1):
        t = i/steps
        u = 1.-t
        position = u**3*p+3*u*u*t*q+3*u*t*t*r+t**3*s
        d = 3*(u*u*(q-p)+2*u*t*(r-q)+t*t*(s-r))
        dd = 6*(u*(r-2*q+p)+t*(s-2*r+q))
        if abs(d) < .08 or abs((d.conjugate()*dd).imag) > abs(d)**3/.17:
            return None
        poses.append((x+position.real, y+position.imag, math.remainder(math.atan2(d.imag, d.real)-flip, math.tau)))
    poses[-1] = (goal['x'], goal['y'], target_yaw)
    return CurveLeg(tuple(poses), reverse)


def tangent_lane_shift(pose, goal, reverse):
    """Connect parallel headings with two tangent arcs of equal radius."""
    target_yaw = goal['yaw']
    heading = angle_error(pose[2], target_yaw)
    if abs(heading) > math.radians(16):
        return None
    frame_yaw = target_yaw + (math.pi if reverse else 0.)
    frame = complex(math.cos(frame_yaw), math.sin(frame_yaw))
    origin = complex(*pose[:2])
    chord = (complex(goal['x'], goal['y'])-origin)/frame
    along, side = chord.real, chord.imag
    if not (.28 <= along <= 1.4 and .12 <= abs(side) <= .48):
        return None
    sign = math.copysign(1., side)
    projection = (along*(math.cos(heading)+1.)+side*math.sin(heading))/(2.*abs(chord))
    bend = math.atan2(side, along) + sign*math.acos(max(-1.,min(1.,projection)))
    if not max(0.,sign*heading) < sign*bend <= math.radians(89):
        return None
    span = sign*complex(2.*math.sin(bend)-math.sin(heading), math.cos(heading)+1.-2.*math.cos(bend))
    radius = abs(chord)/abs(span)
    if not .19 <= radius <= .95 or abs(radius*span-chord) > 1e-6:
        return None
    offset = 0j
    poses = [tuple(pose)]
    for start, end, direction in ((heading,bend,sign),(bend,0.,-sign)):
        steps = max(2,math.ceil(radius*abs(end-start)/.024))
        for i in range(1,steps+1):
            yaw = start+(end-start)*i/steps
            delta = radius*direction*complex(math.sin(yaw)-math.sin(start),math.cos(start)-math.cos(yaw))
            position = origin + frame*(offset+delta)
            poses.append((position.real,position.imag,math.remainder(target_yaw+yaw,math.tau)))
        offset += delta
    poses[-1] = (goal['x'],goal['y'],target_yaw)
    return CurveLeg(tuple(poses), reverse)


class CorridorRoute:
    def __init__(self, grid, settings):
        if settings.get('schema') != 1 or settings.get('map_id') != grid.map_id:
            raise ValueError('参考路线与当前地图不匹配')
        self.grid = grid
        self.lanes = [tuple(map(float, row)) for row in settings['lanes']]
        self.links = tuple(map(float, settings['links']))
        self.gap = float(settings['clearance_m'])
        if not (len(self.lanes) == 2 and 1 <= len(self.links) <= 2 and .25 <= self.gap <= .40):
            raise ValueError('无效的双走廊参考线')
        if not all(len(row) == 3 and all(math.isfinite(v) for v in row) and row[1] < row[2]
                   for row in self.lanes):
            raise ValueError('无效的走廊边界')
        if not all(math.isfinite(y) and all(lo <= y <= hi for _, lo, hi in self.lanes) for y in self.links):
            raise ValueError('换边连接点超出走廊')

        cells = np.asarray(grid.cells).reshape(grid.height, grid.width)
        mask = np.flipud(np.frombuffer(grid.mask, dtype=np.uint8).reshape(cells.shape))
        self.clearance = distance_transform_edt((cells == 0) & (mask >= 128)) * grid.resolution
        horizontal = [((x, lo), (x, hi)) for x, lo, hi in self.lanes]
        connectors = [((self.lanes[0][0], y), (self.lanes[1][0], y)) for y in self.links]
        if not all(self.line_clear(a, b) for a, b in horizontal + connectors):
            raise ValueError('参考线未满足地图及禁区净空')

    @classmethod
    def load(cls, grid):
        path = grid.directory / 'corridor_route.json'
        return cls(grid, json.loads(path.read_text(encoding='utf-8')))

    def clearance_at(self, point):
        ix, iy = self.grid.cell_xy(*point)
        return float(self.clearance[iy, ix]) if 0 <= ix < self.grid.width and 0 <= iy < self.grid.height else 0.

    def line_clear(self, start, end):
        return all(self.clearance_at(p) >= self.gap for p in Leg(start, end, 0., False).samples())

    def entry_clear(self, start, end):
        """Permit a short rejoin from just outside the corridor's comfort margin."""
        if self.line_clear(start, end):
            return True
        leg = Leg(start, end, 0., False)
        if leg.length > .22:
            return False
        margins = [self.clearance_at(point) for point in leg.samples(.025)]
        return (margins[0] >= self.gap-.035 and margins[-1] >= self.gap
                and all(b >= a for a, b in zip(margins, margins[1:])))

    def footprint_clear(self, position, heading, *, include_keepout=True):
        c, s = math.cos(heading), math.sin(heading)
        a, b = abs(c), abs(s)
        length = (PADDED_FRONT+PADDED_REAR)/2
        width = PADDED_HALF_WIDTH
        offset = (PADDED_FRONT-PADDED_REAR)/2
        px, py = position[0]+offset*c, position[1]+offset*s
        ix, iy = self.grid.cell_xy(*position)
        radius = math.ceil(.24 / self.grid.resolution)
        half = self.grid.resolution/2
        for i in range(ix-radius, ix+radius+1):
            for j in range(iy-radius, iy+radius+1):
                if self.grid.value(i,j) == 0 and (not include_keepout or not self.grid.forbidden(i,j)):
                    continue
                x, y = self.grid.world(i,j)
                dx, dy = x-px, y-py
                if (abs(dx) <= half+length*a+width*b and abs(dy) <= half+length*b+width*a
                        and abs(dx*c+dy*s) <= length+half*(a+b)
                        and abs(-dx*s+dy*c) <= width+half*(a+b)):
                    return False
        return True

    def escape_clear(self, leg):
        samples = leg.samples(.025)
        margins = [self.clearance_at(point) for point in samples]
        # A padded footprint already crossing the planning boundary can
        # withdraw, but surveyed obstacles remain checked from the first pose.
        starts_clear = self.footprint_clear(leg.start,leg.yaw)
        return (leg.length <= .57 and margins[0] > 0. and min(margins) >= margins[0]-.01
                and margins[-1] >= margins[0]+.04
                and all(self.footprint_clear(point,leg.yaw,include_keepout=starts_clear or math.dist(leg.start,point)>.035)
                        for point in samples))

    def escape_routes(self, pose, goal):
        if self.clearance_at(pose[:2]) >= self.gap:
            return []
        x,y,heading = pose
        c,s = math.cos(heading),math.sin(heading)
        for distance in (.22,.38,.55):
            for sign in (-1,1):
                end = x+sign*distance*c,y+sign*distance*s
                leg = EscapeLeg((x,y),end,heading,sign<0)
                if self.escape_clear(leg):
                    routes = self.routes((*end,heading),goal,entry_offset=.85)
                    if routes:
                        return [(leg,*route) for route in routes]
        return []

    def outward_retreat(self, pose, distance):
        x, y, heading = pose
        if (self.clearance_at((x,y)) >= self.gap
                or not any(abs(x-lane) <= .55 and lo <= y <= hi for lane,lo,hi in self.lanes)):
            return None
        end = (x-distance*math.cos(heading), y-distance*math.sin(heading))
        points = Leg((x,y),end,heading,True).samples(.025)
        margins = [self.clearance_at(p) for p in points]
        if (margins[-1] < margins[0]+.04 or any(b+.01 < a for a,b in zip(margins,margins[1:]))
                or not all(self.footprint_clear(p,heading,include_keepout=False) for p in points)):
            return None
        return [list(point) for point in points]

    def native_path_clear(self, path, *, dock=None):
        def clear(point):
            return ((dock is not None and math.dist(point[:2],dock[:2]) <= .90)
                    or self.clearance_at(point) >= self.gap)

        return (bool(path) and clear(path[0])
                and all(clear(point) for first,last in zip(path,path[1:])
                        for point in Leg(tuple(first[:2]),tuple(last[:2]),0.,False).samples()))

    def lookahead_end(self, leg):
        """Find a clear tangent extension for the controller's lookahead."""
        if isinstance(leg, CurveLeg):
            heading = leg.end_yaw + (math.pi if leg.reverse else 0.)
            dx, dy = math.cos(heading), math.sin(heading)
        else:
            dx = (leg.end[0]-leg.start[0])/leg.length
            dy = (leg.end[1]-leg.start[1])/leg.length
        for distance in (.24, .18, .12, .08):
            end = (leg.end[0]+distance*dx, leg.end[1]+distance*dy)
            if self.line_clear(leg.end,end):
                return end
        return leg.end

    def includes(self, point):
        x, y = point['x'], point['y']
        return any(abs(x-lane) <= .02 and lo-.01 <= y <= hi+.01 for lane, lo, hi in self.lanes)

    def short_approach(self, pose, goal):
        """Prefer a checked local manoeuvre when the target is already in this aisle."""
        return (pose is not None and self.includes(goal)
                and abs(pose[0]-goal['x']) <= .26
                and math.dist(pose[:2],(goal['x'],goal['y'])) <= .80)

    def reverse_preferred(self, pose, goal):
        if pose is None or not self.includes(goal) or abs(pose[0]-goal['x']) > .14:
            return False
        if abs(angle_error(pose[2],goal['yaw'])) > math.radians(35):
            return False
        return (goal['y']-pose[1])*math.sin(goal['yaw']) < -.12

    def station_curve(self, pose, goal, reverse):
        """Stay in the station's own aisle and check the entire curve before use."""
        gx, gy = goal['x'], goal['y']
        lane = next((x for x, low, high in self.lanes if abs(gx-x) <= .02 and low <= gy <= high), None)
        if lane is None or abs(pose[0]-lane) > .48 or abs(math.cos(goal['yaw'])) > .05:
            return None
        offset = abs(pose[0]-lane)
        if offset <= .28:
            curve = tangent_entry(pose, goal, reverse) or tangent_alignment(pose, goal, reverse)
            if curve and all(abs(x-lane) <= .28 and self.clearance_at((x,y)) >= self.gap for x,y,_ in curve.poses):
                return curve
        shift = tangent_lane_shift(pose, goal, reverse)
        if shift and all(abs(x-lane) <= max(.28,offset+.02)
                         and self.clearance_at((x,y)) >= self.gap for x,y,_ in shift.poses):
            return shift
        return None

    def motion_routes(self, pose, goal):
        routes = self.routes(pose, goal)
        if not self.includes(goal):
            return routes
        if not routes:
            return self.escape_routes(pose,goal)
        if not routes[0]:
            return routes
        direct = routes[0][0]
        if (len(routes[0]) == 1 and abs(angle_error(direct.yaw, pose[2])) <= math.radians(3)
                and abs(angle_error(direct.yaw, goal['yaw'])) <= math.radians(3)):
            return routes
        directions = (True,) if self.reverse_preferred(pose,goal) else (False,True)
        curves = [curve for reverse in directions if (curve := self.station_curve(pose, goal, reverse))]
        if curves:
            return [(min(curves, key=lambda curve: curve.length),), *routes]
        # A few centimetres sideways cannot be fixed by pivoting through 90 degrees
        # without moving the real sampling point. Back out a short, surveyed run.
        near = math.dist(pose[:2], (goal['x'], goal['y'])) <= .14
        if near and abs(pose[0]-goal['x']) >= .026 and abs(angle_error(pose[2], goal['yaw'])) <= math.radians(16):
            yaw = pose[2]
            for distance in (.27, .34):
                for reverse in (True, False):
                    travel = distance if reverse else -distance
                    end = (pose[0]-travel*math.cos(yaw), pose[1]-travel*math.sin(yaw))
                    if self.line_clear(pose[:2], end) and (curve := self.station_curve((*end, yaw), goal, not reverse)):
                        return [(Leg(pose[:2], end, yaw, reverse), curve), *routes]
        return routes

    def joins(self, point, lane, *, allow_offset, offset_limit=.56):
        x, lo, hi = self.lanes[lane]
        if abs(point[0]-x) > offset_limit or not lo-.60 <= point[1] <= hi+.60:
            return ()
        y = min(hi, max(lo, point[1]))
        if not allow_offset or abs(point[0]-x) <= .025:
            return ((x, y),)
        return tuple(dict.fromkeys((x, min(hi, max(lo, y+delta))) for delta in (0., -.42, .42)))

    @staticmethod
    def compact(vertices):
        points = [vertices[0]]
        for vertex in vertices[1:]:
            if math.dist(vertex, points[-1]) <= .008:
                continue
            if len(points) > 1:
                a, b = points[-2:]
                inward = math.atan2(b[1]-a[1], b[0]-a[0])
                outward = math.atan2(vertex[1]-b[1], vertex[0]-b[0])
                if abs(angle_error(inward, outward)) < .018:
                    points.pop()
            points.append(vertex)
        return points

    def routes(self, start, goal, *, entry_offset=.56):
        target = (goal['x'], goal['y'])
        if math.dist(start[:2], target) < .017:
            return [()]
        source = tuple(start[:2])
        begin = min(range(2), key=lambda i: abs(source[0]-self.lanes[i][0]))
        end = min(range(2), key=lambda i: abs(target[0]-self.lanes[i][0]))
        routes = {}
        same_lane = begin == end
        if (same_lane and all(abs(point[0]-self.lanes[begin][0]) <= .14 for point in (source, target))
                and self.entry_clear(source, target)):
            inputs = [(source, target)]
        else:
            inputs = []
            for u, v in itertools.product(self.joins(source, begin, allow_offset=not same_lane,
                                                     offset_limit=entry_offset),
                                          self.joins(target, end, allow_offset=not same_lane)):
                if same_lane:
                    inputs.append((source, u, v, target))
                else:
                    inputs.extend((source, u, (u[0], y), (v[0], y), v, target) for y in self.links)
        for vertices in inputs:
            vertices = self.compact(vertices)
            edges = tuple(zip(vertices, vertices[1:]))
            if (not edges or edges[-1][1] != target or not self.entry_clear(*edges[0])
                    or not all(self.line_clear(a, b) for a, b in edges[1:])):
                continue
            if any(math.dist(a, b) < .10 for a, b in edges[1:-1]):
                continue
            # Score both drive directions for each leg, preserving the actual start and final yaw.
            candidates = [(0., start[2], ())]
            for a, b in edges:
                bearing = math.atan2(b[1]-a[1], b[0]-a[0])
                choices = []
                for reverse in (False, True):
                    yaw = math.remainder(bearing + (math.pi if reverse else 0.), math.tau)
                    leg = Leg(a, b, yaw, reverse)
                    score, _, legs = min(candidates, key=lambda v: v[0] + 6.*abs(angle_error(yaw, v[1])))
                    prior = legs[-1].yaw if legs else start[2]
                    cost = score + .9 + leg.length/(.071 if reverse else .083) + 6.*abs(angle_error(yaw, prior))
                    choices.append((cost, yaw, legs+(leg,)))
                candidates = choices
            cost, _, legs = min(candidates, key=lambda v: v[0]+6.*abs(angle_error(goal['yaw'],v[1])))
            routes[tuple(vertices)] = (cost+6.*abs(angle_error(goal['yaw'],legs[-1].yaw)), legs)
        return [legs for _, legs in sorted(routes.values(), key=lambda item: item[0])]
