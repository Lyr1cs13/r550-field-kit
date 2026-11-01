"""Stopped-pose refinement policy; native controllers retain collision checking."""
import math


def should_capture_pose(distance, yaw_error):
    # A short carrot makes pure pursuit amplify centimetre-scale cross-track
    # errors. Hand the last straight segment to pose control before that gain
    # rises. Keep large final turns after the position approach.
    return distance <= .18 and abs(yaw_error) <= math.radians(25)


def refinement_stage(distance, yaw_error, *, position_tolerance, yaw_tolerance):
    if distance > position_tolerance:
        return 'position'
    if abs(yaw_error) > yaw_tolerance:
        return 'heading'
    return 'complete'


def heading_profile(error, previous_error=None, previous_limit=None):
    # Reduce authority after an observed reversal instead of repeating the
    # same overshooting command. These are native DWB limits, not open-loop turns.
    limit=max(.10,min(.20,.65*abs(error)))
    if previous_error is not None and error*previous_error < 0 and previous_limit:
        limit=min(limit,max(.07,previous_limit*.7))
    return {'max_vel_theta':limit,'min_speed_theta':min(.035,limit/3)}
