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
