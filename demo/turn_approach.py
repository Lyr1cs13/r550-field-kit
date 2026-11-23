"""Choose a stopped handoff for one car and map, using verified terminal turns."""
import cmath
from collections import deque
import math
from statistics import median


def approach_radius(angle, yaw_tolerance, outer, inner):
    """Spend more of the XY budget on a larger turn, continuously up to 45 degrees."""
    angle = abs(math.remainder(angle, math.tau))
    if not inner or inner >= outer or angle <= yaw_tolerance:
        return None
    if yaw_tolerance >= math.pi/4:
        return inner
    scale = (math.sin(angle / 2) - math.sin(yaw_tolerance / 2)) / (
        math.sin(math.pi / 8) - math.sin(yaw_tolerance / 2))
    return outer - (outer - inner) * min(1., scale)


def _rotation(angle):
    return cmath.rect(1., angle)


def _centre(offsets):
    return complex(median(z.real for z in offsets), median(z.imag for z in offsets))


class TurnApproach:
    """Stationary map shifts are effective offsets and include localization error."""
    def __init__(self):
        self.history = deque(maxlen=32)

    def observe(self, station, before, after, requested, yaw_tolerance, xy_limit):
        if not all(math.isfinite(v) for v in (*before, *after, requested, yaw_tolerance)):
            return
        angle = math.remainder(after[2] - before[2], math.tau)
        if not math.radians(30) <= abs(angle) <= math.radians(150):
            return
        if abs(math.remainder(angle - requested, math.tau)) > yaw_tolerance:
            return
        delta = complex(after[0]-before[0], after[1]-before[1]) * _rotation(-before[2])
        previous = self.prediction(angle, xy_limit)
        if previous is not None and abs(previous-delta) > xy_limit/2:
            self.history = deque((row for row in self.history if row[1]*angle <= 0),
                                 maxlen=self.history.maxlen)
        if abs(delta) > .12:
            return
        # In the approach body frame: shift = (R(turn) - I) * effective offset.
        self.history.append((station, angle, delta / (_rotation(angle) - 1)))

    def prediction(self, angle, xy_limit):
        angle = math.remainder(angle, math.tau)
        if not math.radians(30) <= abs(angle) <= math.radians(150):
            return None
        by_station = {station: (a, offset) for station, a, offset in tuple(self.history)
                      if a*angle > 0 and abs(a-angle) <= math.radians(35)}
        if len(by_station) < 4:
            return None
        samples = list(by_station.values())
        offsets = [offset for _, offset in samples]
        # Hold each station out. Reject a directional bias if it cannot predict
        # the other observed stops more closely than aiming at their centres.
        residuals = [abs((_rotation(a)-1) * (offset-_centre(offsets[:i]+offsets[i+1:])))
                     for i, (a, offset) in enumerate(samples)]
        baseline = [abs((_rotation(a)-1)*offset) for a, offset in samples]
        if max(residuals) > xy_limit/2 or median(residuals)+.01 >= median(baseline):
            return None
        return (_rotation(angle)-1) * _centre(offsets)

    def aim(self, start_xy, goal, reverse, xy_limit, checker_radius):
        """Return a candidate endpoint on the approach side, still inside the XY budget."""
        end = complex(goal['x'], goal['y'])
        chord = end - complex(*start_xy)
        if abs(chord) < .10 or not checker_radius or checker_radius >= xy_limit:
            return None
        travel = chord/abs(chord)
        body = -travel if reverse else travel
        shift = self.prediction(goal['yaw']-cmath.phase(body), xy_limit)
        if shift is None:
            return None
        # The native checker can end the approach short of its endpoint. Centre
        # that interval, while never extending the chord beyond the real station.
        correction = -shift*body/travel + checker_radius/2
        correction = complex(min(0., correction.real), correction.imag)
        budget = min(xy_limit-checker_radius, .05*abs(chord))
        if abs(correction) <= .005 or budget <= .005:
            return None
        correction *= min(1., budget/abs(correction))
        endpoint = end + travel*correction
        return dict(goal, x=endpoint.real, y=endpoint.imag)
