"""Conservative localization supervision. Consistency is not absolute accuracy."""
import math
from directed_route import dock_footprint_intersects


DEPARTURE_SPEED = .07
SPIN_CLEARANCE_RADIUS = .195
PADDED_FRONT, PADDED_REAR, PADDED_HALF_WIDTH = .15, .16, .12


def scan_points_in_base(scan, x, y, heading):
    first = heading+scan.angle_min
    return [(x+r*math.cos(first+i*scan.angle_increment),
             y+r*math.sin(first+i*scan.angle_increment))
            for i,r in enumerate(scan.ranges)
            if math.isfinite(r) and scan.range_min<=r<=scan.range_max]


def departure_twist(speed, points, dock_relative, last_linear, dt):
    """Check a slow, forward-only escape from an already seated dock."""
    if dock_relative is not None:
        along,side,error=dock_relative
        if -.13<=along<=.12 and abs(side)<=.08 and abs(error)<=math.radians(12):
            c,s=math.cos(error),math.sin(error)
            # A rear return must be at the calibrated dock as well as behind
            # this car. Front and side returns remain part of the envelope.
            points=[(x,y) for x,y in points
                    if not (-.24<=x<=-.125 and abs(y)<=.105
                            and -.25<=along+c*x-s*y<=-.115 and abs(side+s*x+c*y)<=.11)]
    if not 0.<speed<=DEPARTURE_SPEED or sum(x>.22 and abs(y)<x for x,y in points)<8:
        return (0.,0.,0.)
    velocity=cautious_twist((speed,0.,0.),points,last_linear,dt)
    return velocity if swept_footprint_clear(velocity,points) else (0.,0.,0.)


def swept_footprint_clear(velocity, scan_points):
    """Check padded chassis occupancy over the near braking arc in base frame."""
    vx, _, wz = velocity
    nearby = [(x,y) for x,y in scan_points if x*x+y*y <= .32**2]
    if abs(vx) < .015 and abs(wz) > .015:
        return all(x*x+y*y > SPIN_CLEARANCE_RADIUS**2 for x,y in nearby)
    for t in (0., .16, .32, .48):
        angle=wz*t;c=math.cos(angle);s=math.sin(angle)
        dx=vx*t if abs(wz)<1e-6 else vx*s/wz
        dy=0. if abs(wz)<1e-6 else vx*(1.-c)/wz
        for x,y in nearby:
            rx=c*(x-dx)+s*(y-dy);ry=-s*(x-dx)+c*(y-dy)
            if -PADDED_REAR<=rx<=PADDED_FRONT and abs(ry)<=PADDED_HALF_WIDTH:
                return False
    return True


def swept_dock_clear(velocity, pose, dock):
    x,y,heading = pose
    vx,vy,turn = velocity
    c,s = math.cos(heading),math.sin(heading)
    for t in (0., .12, .24, .36, .48):
        angle = turn*t
        advance, bend = (t,0.) if abs(turn)<1e-6 else (math.sin(angle)/turn,(1.-math.cos(angle))/turn)
        dx,dy = vx*advance-vy*bend, vy*advance+vx*bend
        if dock_footprint_intersects((x+c*dx-s*dy,y+s*dx+c*dy,heading+angle),dock):
            return False
    return True


def cautious_twist(velocity, scan_points, last_linear, dt):
    """Slow the commanded arc when an obstacle is ahead in the swept band."""
    vx, vy, wz = velocity
    speed = abs(vx)
    if speed <= .001:
        return velocity
    direction = math.copysign(1., vx)
    curvature = wz / vx
    clearance = .48
    for x, y in scan_points:
        ahead = x*direction
        sideways = curvature*x
        if .02 < ahead < clearance and abs(sideways) < .95:
            arc_y = curvature*x*x / (1. + math.sqrt(1.-sideways*sideways))
            if abs(y-arc_y) <= .16:
                clearance = max(.18,ahead)
    limit = .05 + (.12-.05)*(clearance-.18)/(.48-.18)
    acceleration_limit = abs(last_linear)+.20*dt if vx*last_linear >= 0 else .20*dt
    scale = min(1., limit/speed, acceleration_limit/speed)
    return vx*scale, vy*scale, wz*scale


class LocalizationGuard:
    def __init__(self):
        self.fault = None
        self.bad_since = None
        self.reason = ''
        self.recovery_since = None
        self.recovery_pose = None
        self.anchor = None
        self.manual_recovery = False

    def trip_slip(self, anchor, *, repeated):
        self.anchor=anchor
        self.manual_recovery=repeated
        self.reason='轮速显示已行驶，雷达显示车身未移动'
        self.fault=self.reason+('，同一处重复空转，等待处理' if repeated else '，已停车并复核定位')

    def reset(self):
        self.fault = None
        self.bad_since = None
        self.reason = ''
        self.recovery_since = None
        self.recovery_pose = None
        self.anchor = None
        self.manual_recovery = False

    def recover(self, now, *, stationary, covariance, scan_fraction, scan_count, pose, fresh, fixed_edges=False):
        """Require a stationary, tight and sustained scan/map solution before release.

        Repeated map geometry can still be ambiguous; this cannot prove global truth.
        """
        good=(self.fault and not self.manual_recovery and stationary and fresh and pose is not None and covariance is not None
              and len(covariance)==3 and all(v is not None and math.isfinite(v) and v>=0 for v in covariance)
              and max(covariance[:2])<=.03**2 and covariance[2]<=math.radians(3)**2
              and (fixed_edges or (scan_count>=80 and scan_fraction is not None and scan_fraction>=.80)))
        if good and self.anchor is not None:
            angle=math.atan2(math.sin(pose[2]-self.anchor[2]),math.cos(pose[2]-self.anchor[2]))
            good=math.dist(pose[:2],self.anchor[:2])<=.10 and abs(angle)<=math.radians(8)
        if good and self.recovery_pose is not None:
            angle=math.atan2(math.sin(pose[2]-self.recovery_pose[2]),math.cos(pose[2]-self.recovery_pose[2]))
            good=math.dist(pose[:2],self.recovery_pose[:2])<=.02 and abs(angle)<=math.radians(1)
        if not good:
            self.recovery_since=None;self.recovery_pose=None
            return False
        if self.recovery_since is None:
            self.recovery_since=now;self.recovery_pose=list(pose)
        if now-self.recovery_since>=3.:
            self.reset();return True
        return False

    def update(self, now, *, moving, covariance, scan_fraction=None, scan_count=0, fixed_edges=False):
        if self.fault:
            return self.fault
        bad = ''
        if covariance is not None and (len(covariance)!=3 or any(
                v is None or not math.isfinite(v) or v < 0 for v in covariance)):
            bad = '定位协方差无效'
        elif covariance is not None and (max(covariance[:2]) > .15**2 or covariance[2] > math.radians(12)**2):
            bad = '定位不确定性过大'
        elif not fixed_edges and scan_count >= 40 and scan_fraction is not None and scan_fraction < .35:
            bad = '雷达与静态地图持续不一致，可能空转或定位漂移'
        self.reason = bad
        if moving and bad:
            if self.bad_since is None:
                self.bad_since = now
            if now-self.bad_since >= .6:
                self.fault = bad+'；保留任务，重新核对定位后继续'
        else:
            self.bad_since = None
        return self.fault


class StopFeedback:
    """Distinguish bounded braking after a command from unintended motion."""
    def __init__(self):
        self.last_turn = None
        self.last_travel = None
        self.since = None
        self.origin = None
        self.initial_linear_speed = 0.
        self.last_drive = None
        self.cut_rates = (0., 0.)
        self.brake_pose = None
        self.brake_distance = self.brake_yaw = 0.
        self.reviewing = False
        self.quiet_since = self.quiet_stamp = None

    def track_braking(self, odometry):
        self.brake_distance += math.dist(odometry[:2],self.brake_pose[:2])
        self.brake_yaw += abs(math.remainder(odometry[2]-self.brake_pose[2],2*math.pi))
        self.brake_pose = odometry[:3]

    def still_moving(self, now, command, odometry, turn_clear=lambda: False, travel_clear=lambda: False):
        if any(command):
            self.last_turn = (now, command[2]) if abs(command[0]) < .01 and command[2] else None
            self.last_travel = (now, command[0]) if abs(command[0]) >= .025 else None
            if abs(command[0]) >= .008 or abs(command[2]) >= .035:
                self.last_drive = (now, command[0], command[2])
        elif odometry is None or (abs(odometry[3]) <= .008 and abs(odometry[4]) <= .035):
            self.last_drive = None
        if any(command) or odometry is None or (abs(odometry[3]) <= .025 and abs(odometry[4]) <= .10):
            self.since = self.origin = None
            return False

        if self.since is None:
            self.since = now
            self.origin = odometry[:3]
            self.initial_linear_speed = abs(odometry[3])
            self.cut_rates = odometry[3:]
            self.brake_pose = self.origin
            self.brake_distance = self.brake_yaw = 0.
        self.track_braking(odometry)
        if now-self.since < .65:
            return False
        linear_speed, yaw_rate = odometry[3:]
        if (self.last_turn and now-self.last_turn[0] < 2.5
                and abs(linear_speed) <= .025 and yaw_rate*self.last_turn[1] > 0. and turn_clear()):
            return False
        if not self.last_travel:
            return True
        # A measured stop from 12 cm/s may coast about 5.5 cm. Only allow the
        # longer tail at higher approach speeds and when the wheels decelerate.
        grace_distance = min(.06, .035 + .625 * max(0., abs(self.last_travel[1]) - .08))
        slowing = abs(linear_speed) <= min(.055, self.initial_linear_speed * .65)
        return not (now-self.last_travel[0] < 1.20 and linear_speed*self.last_travel[1] > 0.
                    and slowing and abs(yaw_rate) <= .12
                    and math.dist(odometry[:2],self.origin[:2]) < grace_distance
                    and abs(math.remainder(odometry[2]-self.origin[2],2*math.pi)) < .06 and travel_clear())

    def requires_lock(self, now, command, odometry, stamp, turn_clear, travel_clear):
        """Keep a small braking tail at zero until distinct wheel frames prove rest."""
        if not self.reviewing:
            if not self.still_moving(now,command,odometry,turn_clear,travel_clear):
                return False
            if self.last_drive is None or not 0. <= self.since-self.last_drive[0] <= .30:
                return True
        if odometry is None or not 0. <= now-stamp <= .15:
            self.quiet_since = self.quiet_stamp = None
            self.reviewing = True
            return False
        if self.reviewing:
            self.track_braking(odometry)
        speed,turn = odometry[3:]
        # A near-zero translation during a pivot can have either sign. Check the
        # direction of every significant component against the last drive and cut.
        for value,requested,measured,limit in zip((speed,turn),self.last_drive[1:],self.cut_rates,(.025,.10)):
            if abs(value)>limit and (value*measured<=0. or (abs(requested)>limit and value*requested<0.)):
                return True
        if (abs(speed)>.050 or abs(turn)>.16 or self.brake_distance>=.045
                or self.brake_yaw>=math.radians(8)
                or not travel_clear()):
            return True
        if abs(speed)>.008 or abs(turn)>.035:
            self.quiet_since = self.quiet_stamp = None
            if now-self.since >= 1.50:
                return True
        elif self.quiet_stamp != stamp:
            if self.quiet_stamp is None or not 0. < stamp-self.quiet_stamp <= .16:
                self.quiet_since = stamp
            self.quiet_stamp = stamp
            if stamp-self.quiet_since >= .30:
                self.reviewing = False
                self.last_drive = self.last_turn = self.last_travel = None
                self.since = self.origin = self.quiet_since = self.quiet_stamp = None
                return False
        self.reviewing = True
        return False


class NativeStopFeedback:
    """Keep braking after STM32 handoff until fresh wheel readings settle."""
    def __init__(self):
        self.active = False
        self.origin = self.moving_since = self.still_since = None

    def start(self):
        if not self.active:
            self.active = True
            self.origin = self.moving_since = self.still_since = None

    def update(self, now, odometry):
        if not self.active:
            return False
        if odometry is None:
            self.still_since = self.moving_since = None
            return False
        x,y,yaw,speed,turn = odometry
        if self.origin is None:
            self.origin = (x,y,yaw)
        if abs(speed)<=.025 and abs(turn)<=.10:
            self.moving_since=None
            if self.still_since is None:
                self.still_since=now
            if now-self.still_since>=.20:
                self.active=False
            return False
        self.still_since=None
        if self.moving_since is None:
            self.moving_since=now
        elapsed=now-self.moving_since
        dx,dy,heading=self.origin
        return (math.hypot(x-dx,y-dy)>=.050 or abs(math.remainder(yaw-heading,2*math.pi))>=.15
                or (elapsed>=.65 and (abs(speed)>=.075 or abs(turn)>=.30))
                or elapsed>=1.5)


class VelocityLease:
    """No stale nonzero command can survive a missing permission heartbeat."""
    def __init__(self, timeout=.3):
        self.timeout = timeout
        self.command_time = self.permit_time = -math.inf
        self.command = (0., 0., 0.)
        self.permitted = False
        self.spin_limit = 0.
        self.spin_time = self.spin_started = self.spin_stamp = -math.inf

    def receive_command(self, now, command):
        self.command_time = now
        self.command = tuple(command) if all(math.isfinite(v) for v in command) else (0., 0., 0.)

    def receive_permit(self, now, permitted, ros_stamp=None):
        self.permit_time = now
        if not permitted:
            self.spin_limit = 0.
            if ros_stamp is not None:
                self.spin_stamp = max(self.spin_stamp, ros_stamp)
        elif self.spin_limit and now-self.spin_time > .16:
            self.spin_limit = 0.
            self.command_time = -math.inf
        self.permitted = bool(permitted) and not self.spin_limit

    def receive_spin_limit(self, now, stamp, age, limit):
        """A separate, expiring permit may only reduce a fresh pure Spin command."""
        if not (-.05 <= age <= .16 and math.isfinite(stamp) and stamp > self.spin_stamp):
            return
        self.spin_stamp = stamp
        if not math.isfinite(limit) or abs(limit) > .25:
            limit = 0.
        if limit:
            if self.spin_limit * limit <= 0. or now - self.spin_time > .16:
                self.spin_started = now
            self.permitted = False
        self.spin_limit, self.spin_time = limit, now

    def output(self, now):
        if self.spin_limit:
            vx, vy, wz = self.command
            if (not 0. <= now-self.spin_time <= .16 or not 0. <= now-self.command_time <= self.timeout
                    or self.command_time < self.spin_started or abs(vx)+abs(vy) > 1e-6 or wz*self.spin_limit <= 0.):
                return (0., 0., 0.)
            return (0., 0., math.copysign(min(abs(wz), abs(self.spin_limit)), wz))
        if (not self.permitted or not 0 <= now-self.permit_time <= self.timeout or
                not 0 <= now-self.command_time <= self.timeout):
            return (0., 0., 0.)
        return self.command


class OemLease:
    """Track supervision of the OEM docking mode."""
    def __init__(self):
        self.active=False;self.received=-math.inf

    def receive(self,now,active):
        active=bool(active)
        released=self.active and not active
        self.active=active;self.received=now
        return released

    def expired(self,now):
        if self.active and now-self.received>.5:
            self.active=False
            return True
        return False
