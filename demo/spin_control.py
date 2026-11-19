"""Gyro-feedback speed ceiling for a collision-checked, stationary Nav2 Spin."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TaperProfile:
    start_rate: float
    start_ramp: float
    crawl_rate: float
    delay: float
    deceleration: float


@dataclass(frozen=True)
class Calibration:
    micro_angle_deg: float
    start_rate: float
    delay: float
    deceleration: float
    taper: TaperProfile


# Fit to the training partition of the R550 field logs. SI units except the switch angle.
R550_TURN = Calibration(9.15, .165, .17, .454, TaperProfile(.175, .030, .095, .17, .400))


class ProfiledSpin:
    def __init__(self, error, stamp, bias, calibration=R550_TURN):
        self.direction = math.copysign(1., error)
        self.target = abs(error)
        self.bias = bias
        self.stamp = self.start = stamp
        self.calibration = calibration
        self.micro = self.target <= math.radians(calibration.micro_angle_deg)
        self.limit = calibration.start_rate if self.micro else calibration.taper.start_rate
        self.rate = self.progress = 0.
        self.turning = self.stopped = False

    @property
    def moving(self):
        return self.turning or abs(self.rate) >= .035

    @property
    def native_target(self):
        # Supply breakaway authority. The signed gyro target and independent travel
        # budget are still the requested turn; Nav2 checks the whole swept footprint.
        return self.direction * max(self.target, .15)

    @property
    def signed_limit(self):
        return 0. if self.stopped else self.direction * self.limit

    def observe(self, stamp, gyro_rate):
        if self.stopped:
            return True
        dt = stamp - self.stamp
        if dt <= 0.:
            return False
        speed = self.direction * (gyro_rate - self.bias)
        self.stamp = stamp
        if dt > .16:
            self.stopped = self.moving or abs(speed) >= .035
            self.rate = speed
            return self.stopped
        self.progress += .5 * (self.rate + speed) * dt
        rising = max(0., min(1.65, (speed - self.rate) / dt))
        self.rate = speed
        # Below the typical measured coast angle, a delayed, sharp breakaway may
        # already supply the entire turn before the next IMU / motor update.
        if (not self.turning and self.target <= math.radians(self.calibration.micro_angle_deg / 2)
                and stamp-self.start >= .35 and speed >= .018 and rising >= .25):
            self.stopped = True
            return True
        self.turning = self.turning or speed >= .035
        remaining = self.target - self.progress
        if remaining <= 0. or self.progress < -math.radians(1.):
            self.stopped = True
            return True
        if self.micro:
            p = self.calibration
            if self.turning:
                self.limit = max(.11, min(p.start_rate, math.sqrt(.32 * remaining)))
            brake_rate = speed + rising * p.delay
        else:
            p = self.calibration.taper
            if not self.turning:
                maximum = .205 if self.target <= math.radians(12.) else .24
                self.limit = min(maximum, p.start_rate + (stamp - self.start) * p.start_ramp)
                return False
            next_limit = max(p.crawl_rate, min(.25, p.crawl_rate + .45 * (remaining - speed * p.delay)))
            brake_rate = max(speed, min(speed + rising * p.delay, 1.45 * max(self.limit, next_limit)))
            self.limit = next_limit
        if speed >= .025:
            coast = .5 * (speed + brake_rate) * p.delay + brake_rate ** 2 / (2. * p.deceleration)
            self.stopped = remaining <= coast
        return self.stopped
