"""One short reverse segment, with a conservative laser envelope in base_footprint."""
import math
from self_return_geometry import isolated_echo_indices


RETREAT_DISTANCE = .20
RETREAT_SPEED = .05


class RouteRetreat:
    """Allow one escape, then one more only after a verified change of position."""
    def __init__(self):
        self.starts = []

    def available(self, pose, *, limit=2):
        if not self.starts or math.dist(self.starts[0][:2], pose[:2])>limit*RETREAT_DISTANCE+.15:
            return True
        x,y,heading=self.starts[-1]
        dx,dy=pose[0]-x,pose[1]-y
        c,s=math.cos(heading),math.sin(heading)
        return (len(self.starts)<limit and -.27<=dx*c+dy*s<=-.15 and abs(-dx*s+dy*c)<=.055
                and abs(math.remainder(pose[2]-heading,math.tau))<=math.radians(8))

    def record(self, pose):
        if self.starts and math.dist(self.starts[0][:2],pose[:2])>.55:
            self.starts.clear()
        self.starts.append(tuple(pose))


def slip_obstruction(anchor, odom_start, odom_end):
    dx,dy=odom_end[0]-odom_start[0],odom_end[1]-odom_start[1]
    c,s=math.cos(odom_start[2]),math.sin(odom_start[2])
    along,lateral=dx*c+dy*s,-dx*s+dy*c
    direction='unknown'
    point=None
    if (abs(along)>=.065 and abs(lateral)<=.045
            and abs(math.remainder(odom_end[2]-odom_start[2],math.tau))<=math.radians(10)):
        direction='front' if along>0. else 'rear'
        offset=math.copysign(.28,along)
        point=[anchor[0]+offset*math.cos(anchor[2]),anchor[1]+offset*math.sin(anchor[2])]
    return {'anchor':list(anchor),'point':point,'direction':direction,'attempted':False}


def rear_is_clear(points, self_echo=None):
    omitted = isolated_echo_indices(points, self_echo)
    points = [p for i,p in enumerate(points) if i not in omitted]
    rear_coverage = sum(x < -.45 and abs(y) < -x for x, y in points)
    occupied = any((-.45 <= x <= .02 and abs(y) <= .17)
                   or (.02 < x <= .145 and abs(y) <= .145) for x, y in points)
    return rear_coverage >= 8 and not occupied


class GuardedRetreat:
    def __init__(self, self_echo=None):
        self.self_echo = self_echo
        self.status = 'idle'
        self.origin = None
        self.started = 0.

    def reset(self):
        self.status = 'idle'
        self.origin = None

    def tick(self, now, points, odom):
        if self.status not in ('idle', 'backing'):
            return 0.
        if odom is None or not rear_is_clear(points, self.self_echo):
            self.status = 'blocked'
            return 0.
        x, y, yaw, v, w = odom
        if self.origin is None:
            if abs(v) > .02 or abs(w) > .05:
                self.status = 'moving'
                return 0.
            self.origin = x, y, yaw
            self.started = now
        ox, oy, heading = self.origin
        dx, dy = x-ox, y-oy
        travel = -(dx*math.cos(heading)+dy*math.sin(heading))
        lateral = -dx*math.sin(heading)+dy*math.cos(heading)
        if travel < -.02 or abs(lateral) > .035 or abs(math.remainder(yaw-heading, 2*math.pi)) > math.radians(4):
            self.status = 'deviated'
        elif travel >= RETREAT_DISTANCE:
            self.status = 'done'
        elif now-self.started >= 7. or (now-self.started >= 2. and travel < .035):
            self.status = 'stalled'
        else:
            self.status = 'backing'
        return -RETREAT_SPEED if self.status == 'backing' else 0.
