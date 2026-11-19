"""Fixed-heading straight legs; every candidate still needs live Nav2 validation."""
import math
from arrival_policy import FAS_STATION_POSITION_TOLERANCE


def angle_error(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def dock_offset(pose, dock):
    dx, dy = pose[0]-dock[0], pose[1]-dock[1]
    c, s = math.cos(dock[2]), math.sin(dock[2])
    return dx*c+dy*s, -dx*s+dy*c


def departure_leg(pose, dock, goal, spacing=.04):
    """Project the configured dock clearance onto the current forward ray."""
    if pose is None:
        raise ValueError('离桩前需要有效定位')
    along, side = dock_offset(pose, dock)
    planned, _ = dock_offset((goal['x'], goal['y']), dock)
    heading_error = angle_error(pose[2], dock[2])
    if along < -.20 or abs(side) > .15 or abs(heading_error) > math.radians(20):
        raise ValueError('离桩前请核对车尾贴合及车头朝向')
    remaining = max(0., (planned-along)/math.cos(heading_error))
    if remaining < .04:
        return {'distance':0., 'poses':[]}
    c,s = math.cos(pose[2]),math.sin(pose[2])
    count = max(1,math.ceil(remaining/spacing))
    return {'distance':remaining,'poses':[
        {'x':pose[0]+remaining*c*i/count, 'y':pose[1]+remaining*s*i/count, 'yaw':pose[2]}
        for i in range(count+1)]}


def straight_approach_speed(distance, *, reverse=False):
    return max(.035,min(.08 if reverse else .12,.042+.22*(distance-.08)))


def line_poses(pose, goal, body_heading, spacing):
    x,y = pose[:2]
    dx,dy = goal['x']-x,goal['y']-y
    count = max(2,math.ceil(math.hypot(dx,dy)/spacing))
    return [{'x':x+dx*i/count,'y':y+dy*i/count,'yaw':body_heading} for i in range(count+1)]


def fas_axis_leg(pose, goal, spacing=.04):
    """Prefer a straight approach and save larger turns for a stopped chassis."""
    if pose is None:
        return None
    dx,dy = goal['x']-pose[0],goal['y']-pose[1]
    if not .10 <= math.hypot(dx,dy) <= 12.:
        return None
    bearing = math.atan2(dy,dx)
    reverse = abs(angle_error(bearing,pose[2])) > math.pi/2
    body_heading = angle_error(bearing+(math.pi if reverse else 0.),0.)
    final_turn_limit = math.pi/2 if reverse else math.radians(12)
    if (abs(angle_error(body_heading,goal['yaw'])) > final_turn_limit+1e-9
            or abs(angle_error(body_heading,pose[2])) > math.radians(15)):
        return None
    return {'reverse':reverse,'poses':line_poses(pose,goal,body_heading,spacing)}


def fas_short_leg(pose, goal, spacing=.04, *, min_distance=.10, max_distance=.80):
    """Connect nearby points independently of the final sampling heading."""
    if pose is None:
        return None
    dx,dy=goal['x']-pose[0],goal['y']-pose[1]
    if not min_distance<=math.hypot(dx,dy)<=max_distance:
        return None
    forward=math.atan2(dy,dx)
    reverse=angle_error(forward+math.pi,0.)
    def turn_cost(heading):
        return abs(angle_error(heading,pose[2]))+abs(angle_error(goal['yaw'],heading))
    backwards=turn_cost(reverse)+math.radians(12)<turn_cost(forward)
    return {'reverse':backwards,'poses':line_poses(pose,goal,reverse if backwards else forward,spacing)}


def fas_station_leg(pose, goal):
    """Candidate only. The bridge checks every cell, then asks Nav2 to check the live path."""
    leg=fas_axis_leg(pose,goal) or fas_short_leg(pose,goal,min_distance=FAS_STATION_POSITION_TOLERANCE,max_distance=12.)
    if leg:
        distance=math.dist(pose[:2],(goal['x'],goal['y']))
        leg['clearance']=.27 if distance>.80 else .23
    return leg


def straight_leg(pose, goal, planned, spacing=.04):
    """Select forward/reverse when the initial heading and native path agree.

    A lateral leg, curved detour, or changed initial heading uses normal Nav2.
    This function is geometry, NOT an obstacle/footprint validity check.
    """
    if pose is None or len(planned) < 2:
        return None
    x, y, heading = pose
    dx, dy = goal['x']-x, goal['y']-y
    length = math.hypot(dx, dy)
    if not .20 <= length <= 12.:
        return None
    bearing = math.atan2(dy, dx)
    backwards = abs(angle_error(bearing, heading)) > math.pi/2
    body_heading = angle_error(bearing + (math.pi if backwards else 0.), 0.)
    if abs(angle_error(body_heading, heading)) > math.radians(12):
        return None
    previous = -.08
    for px, py in planned:
        along = ((px-x)*dx+(py-y)*dy)/length
        cross = abs((px-x)*dy-(py-y)*dx)/length
        if cross > .15 or along < previous-.04 or along > length+.08:
            return None
        previous = max(previous,along)
    return {'reverse':backwards,'poses':line_poses(pose,goal,body_heading,spacing)}


def uniform_route(vertices, count, heading, wait=3.):
    """Sample an explicit open polyline; closure must be explicitly provided."""
    if not isinstance(count, int) or not 2 <= count <= 100:
        raise ValueError('点位数须为2～100')
    if len(vertices) < 2 or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in vertices):
        raise ValueError('路线至少需要两个有效拐点')
    if not math.isfinite(heading) or not math.isfinite(wait) or not 0 <= wait <= 3600:
        raise ValueError('朝向或停留时间无效')
    legs = [(a, b, math.dist(a, b)) for a, b in zip(vertices, vertices[1:]) if math.dist(a, b) > 1e-6]
    total = sum(d for _, _, d in legs)
    if total < .2:
        raise ValueError('路线过短')
    closed = math.dist(vertices[0], vertices[-1]) < 1e-6
    result = []
    for i in range(count):
        distance = total*i/(count if closed else count-1)
        for a, b, length in legs:
            if distance <= length+1e-9:
                t = min(1., distance/length)
                result.append(dict(id=f'fixed-{i+1:03d}', name=f'点位{i+1}',
                                   x=a[0]+(b[0]-a[0])*t, y=a[1]+(b[1]-a[1])*t,
                                   yaw=angle_error(heading, 0.), wait=wait, enabled=True))
                break
            distance -= length
    return result
