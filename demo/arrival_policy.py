"""Stopped-pose refinement policy; native controllers retain collision checking."""
import math


RSSI_STATION_POSITION_TOLERANCE = .12
RSSI_STATION_YAW_TOLERANCE = math.radians(3)
FAS_STATION_YAW_TOLERANCE = math.radians(3)


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


def in_place_heading_profile(error, previous_error=None, previous_limit=None):
    if abs(error) <= math.radians(25):
        # A small command can hold the stationary R550 below breakaway torque.
        # The measured-rate brake controls the travel once the wheels start.
        return {'max_vel_theta': .16, 'min_speed_theta': .14}
    return heading_profile(error, previous_error, previous_limit)


def heading_needs_brake(error, rate, tolerance):
    speed = abs(rate)
    if speed < .045:
        return False
    if error * rate < 0:
        return abs(error) <= 2 * tolerance
    remaining_travel = min(math.radians(3.5), speed * .14 + speed ** 2 / 2.)
    return abs(error) <= remaining_travel + .25 * tolerance


def fas_heading_needs_brake(error, rate, tolerance, pose_age):
    # Bridge the interval between map poses with fresh wheel feedback.
    if not 0. <= pose_age <= .15:
        return False
    projected_error = math.remainder(error-rate*min(pose_age,.10),2*math.pi)
    return heading_needs_brake(projected_error,rate,tolerance)
