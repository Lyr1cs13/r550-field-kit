"""Measured onboard rear-right antenna envelope in base_footprint metres.
Not an angular blind sector: farther returns on the same ray are retained.
Centre fitted to 144 stationary scans and physically confirmed onboard antenna.
"""
import math
CENTER=(-.120,-.073)
RADIUS=.027

def inside_antenna(x,y):
    return math.isfinite(x) and math.isfinite(y) and (x-CENTER[0])**2+(y-CENTER[1])**2<=RADIUS**2

def antenna_return(distance,angle,tx,ty,yaw):
    if not math.isfinite(distance):return False
    return inside_antenna(tx+distance*math.cos(angle+yaw),ty+distance*math.sin(angle+yaw))


def isolated_echo_indices(points, echo):
    """An optional second return limited to a few beams in the measured volume."""
    if echo is None:
        return ()
    cx, cy, radius = echo
    indices = tuple(i for i, (x,y) in enumerate(points) if math.hypot(x-cx,y-cy) <= radius)
    return indices if len(indices) <= 3 else ()


def antenna_echo_indices(ranges, angle, increment, sensor, limits, center, radius, secondary=None):
    """Only project ranges that could intersect either measured body envelope."""
    tx,ty=sensor
    lower,upper=limits
    near=math.hypot(center[0]-tx,center[1]-ty)
    other=math.hypot(secondary[0]-tx,secondary[1]-ty) if secondary else None
    primary_hits=[]
    secondary_hits=[]
    for index,distance in enumerate(ranges):
        if not math.isfinite(distance) or not lower<=distance<=upper:
            continue
        primary=abs(distance-near)<=radius
        alternate=secondary and abs(distance-other)<=secondary[2]
        if not (primary or alternate):
            continue
        heading=angle+index*increment
        x,y=tx+distance*math.cos(heading),ty+distance*math.sin(heading)
        if primary and (x-center[0])**2+(y-center[1])**2<=radius**2:
            primary_hits.append(index)
        if alternate and math.hypot(x-secondary[0],y-secondary[1])<=secondary[2]:
            secondary_hits.append(index)
    if len(secondary_hits)<=3:
        primary_hits.extend(secondary_hits)
    return set(primary_hits)
