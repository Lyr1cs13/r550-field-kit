"""Stopped-pose refinement policy; native controllers retain collision checking."""
import math


RSSI_STATION_POSITION_TOLERANCE = .12
RSSI_STATION_YAW_TOLERANCE = math.radians(3)
FAS_STATION_POSITION_TOLERANCE = .055
FAS_STATION_YAW_TOLERANCE = math.radians(4)
SMALL_SPIN_ANGLE = math.radians(12)
FAS_TURN_MARGIN_ANGLE = math.radians(45)


def refinement_stage(distance, yaw_error, *, position_tolerance, yaw_tolerance):
    if distance > position_tolerance:
        return 'position'
    if abs(yaw_error) > yaw_tolerance:
        return 'heading'
    return 'complete'


class StationCheck:
    """Classify successive stopped map poses, keeping the live acceptance limits."""
    def __init__(self):
        self.samples = []

    def observe(self, pose_stamp, scan_stamp, distance, error, xy_limit, yaw_limit):
        if self.samples and (pose_stamp <= self.samples[-1][0] or scan_stamp <= self.samples[-1][1]):
            return 'wait'
        stage = refinement_stage(distance, error, position_tolerance=xy_limit, yaw_tolerance=yaw_limit)
        yaw_side = (1 if error > 0. else -1) if stage == 'heading' else 0
        self.samples.append((pose_stamp, scan_stamp, stage, yaw_side))
        self.samples = [row for row in self.samples if pose_stamp-row[0] <= 1.5]
        if len(self.samples) < 3 or pose_stamp-self.samples[0][0] < .24:
            return 'wait'
        # Do not launch translation or reverse a turn on one flickering boundary sample.
        recent = self.samples[-3:]
        if all(row[2:] == (stage, yaw_side) for row in recent):
            return stage
        # Act on a persistent boundary crossing if the current reading has
        # support across the window or recurs among the last five paired readings.
        # Success always needs three consecutive in-bounds poses.
        if pose_stamp-self.samples[0][0] >= 1. and stage != 'complete':
            matching = sum(row[2:] == (stage, yaw_side) for row in self.samples)
            persistent = sum(row[2:] == (stage, yaw_side) for row in self.samples[-5:]) >= 2
            if 2*matching >= len(self.samples) or persistent:
                return stage
        return 'wait'


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


class SpinBrake:
    """Estimate travel during the command delay and chassis deceleration."""
    def __init__(self, error, stamp, bias):
        self.direction=math.copysign(1.,error)
        self.target=abs(error)
        self.progress=0.
        self.stamp=stamp
        self.bias=bias
        self.rate=0.
        small=self.target<=SMALL_SPIN_ANGLE
        self.command_delay=.22 if small else .16
        self.deceleration=.27 if small else .40

    def observe(self, stamp, gyro_rate):
        dt=stamp-self.stamp
        speed=self.direction*(gyro_rate-self.bias)
        if dt<=0.:
            return False
        if dt>.16:
            moving=self.moving or abs(speed)>=.035
            self.stamp,self.rate=stamp,speed
            return moving
        self.progress+=(speed+self.rate)*dt*.5
        rising=max(0.,min(1.65,(speed-self.rate)/dt))
        self.stamp,self.rate=stamp,speed
        if self.progress>=self.target:
            return True
        if speed<.025:
            return False
        brake_rate=speed+rising*self.command_delay
        tail=(speed+brake_rate)*self.command_delay*.5+brake_rate**2/(2*self.deceleration)
        return self.progress+tail>=self.target

    @property
    def moving(self):
        return abs(self.rate)>=.035 or abs(self.progress)>=math.radians(.3)
