#!/usr/bin/env python3
"""Replay field-derived angular dynamics without ROS or a vehicle connection."""
import argparse
from bisect import bisect_right
from collections import defaultdict, deque
from dataclasses import asdict, dataclass, replace
from itertools import product
import json
import math
from pathlib import Path
import random
from statistics import median
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'demo'))
from arrival_policy import SpinBrake
from spin_control import Calibration, ProfiledSpin, R550_TURN, TaperProfile


DT = .02
ANGLES = (3.5, 6., 12., 30., 90.)


def clamp(value, lower, upper):
    return min(upper, max(lower, value))


def quantile(values, fraction):
    values = sorted(values)
    index = fraction * (len(values) - 1)
    low = math.floor(index)
    return values[low] + (index - low) * (values[math.ceil(index)] - values[low])


@dataclass(frozen=True)
class Turn:
    trace: str
    feedback: str
    direction: int
    command: float
    breakaway_s: float
    acceleration: float
    deceleration: float
    stop_delay_s: float
    gain: float
    coast_degrees: float


def angular_area(samples):
    return sum(.5 * (w + z) * (b - a) for (a, w), (b, z) in zip(samples, samples[1:]) if 0 < b - a <= .16)


def extract_turns(path):
    with path.open(encoding='utf-8-sig') as stream:
        data = [json.loads(line) for line in stream]
    commands = [(t, w if abs(x) + abs(y) < .015 else 0.) for t, key, value in data
                if key == 'command' for x, y, w in (value,)]
    feedback = 'gyro' if any(key == 'gyro' for _, key, _ in data) else 'wheel'
    rates = [(t, value if feedback == 'gyro' else value[2]) for t, key, value in data if key == feedback]
    times = [t for t, _ in rates]
    for i, (start, initial) in enumerate(commands):
        if abs(initial) < .05 or (i > 0 and commands[i - 1][1] * initial > .0025):
            continue
        j = i + 1
        while j < len(commands) and commands[j][1] * initial > .0025:
            j += 1
        if j == len(commands) or commands[j][0] - start < .20:
            continue
        stop = commands[j][0]
        next_action = next((t for t, w in commands[j + 1:] if abs(w) >= .05), math.inf)
        sign = 1 if initial > 0 else -1
        selected = [(t, sign * w) for t, w in rates[bisect_right(times, start):bisect_right(times, min(stop + 1.45, next_action))]]
        onset = next((t for t, w in selected if w > .035), None)
        coast = [(t, w) for t, w in selected if t >= stop]
        if onset is None or onset > stop or len(coast) < 5 or max(w for _, w in coast) < .06:
            continue
        peak_at = max(range(len(coast)), key=lambda n: coast[n][1])
        peak_t, peak_rate = coast[peak_at]
        tail = coast[peak_at:]
        tail_area = angular_area(tail)
        if tail_area <= .01 or tail[-1][1] > .045 or peak_t - stop > .55:
            continue
        rise = [(z - w) / (b - a) for (a, w), (b, z) in zip(selected, selected[1:])
                if a >= onset and b <= peak_t and .02 <= b - a <= .15 and z > w]
        level = median(abs(w) for _, w in commands[i:j])
        yield Turn(path.name, feedback, sign, level, onset - start,
                   clamp(quantile(rise, .72) if rise else .50, .18, 1.6),
                   clamp(peak_rate ** 2 / (2 * tail_area), .16, .85),
                   clamp(peak_t - stop, .04, .36), clamp(peak_rate / level, .65, 2.2),
                   math.degrees(angular_area(coast)))


class AngularPlant:
    def __init__(self, profile):
        self.profile = profile
        self.rate = self.angle = self.powered = 0.
        self.broken_away = False
        self.queue = deque([0.] * max(1, round(profile.stop_delay_s / DT)))

    def step(self, command):
        self.queue.append(command)
        applied = self.queue.popleft()
        p = self.profile
        if not self.broken_away and applied >= .10:
            self.powered += DT * (applied / p.command) ** 2
            self.broken_away = self.powered >= p.breakaway_s
        target = p.gain * applied if self.broken_away else 0.
        self.rate += clamp(target - self.rate, -p.deceleration * DT, p.acceleration * DT)
        self.angle += self.rate * DT


def simulate(profile, angle, controller, calibration, rng):
    plant = AngularPlant(profile)
    target = math.radians(abs(angle))
    sign = math.copysign(1., angle)
    delay = deque([(0., 0.)] * rng.randint(4, 10))
    gyro_delay = deque([0.] * rng.randint(1, 3))
    feedback = (ProfiledSpin(sign*target, 0., 0., calibration) if controller == 'profiled' else
                SpinBrake(sign*target, 0., 0.))
    pulse = target
    if controller == 'profiled':
        pulse = abs(feedback.native_target)
    elif controller == 'deployed_gyro' and abs(angle) <= 12.:
        pulse = max(target, .14)
    angular_rate = 0.
    stopped_at = None
    stationary_since = None
    allowance = math.ceil(max(8., target / .16 + 5.))
    smoothed = next_cycle = 0.
    for tick in range(math.ceil((allowance + 2.) / DT)):
        t = (tick + 1) * DT
        delay.append((plant.angle, plant.rate))
        delayed_angle, _ = delay.popleft()
        gyro_delay.append(plant.rate + rng.gauss(0., .002))
        gyro = gyro_delay.popleft()
        if stopped_at is None:
            stop = (delayed_angle >= pulse or
                    (controller != 'native' and tick % 3 == 0 and feedback.observe(t, sign*gyro)))
            if stop or t >= allowance:
                stopped_at = t
        if t >= next_cycle:
            next_cycle += .05
            native = clamp(math.sqrt(.24 * max(0., pulse-delayed_angle)), .12, .25) if stopped_at is None else 0.
            smoothed += clamp(native-smoothed, -.8*.05, .6*.05)
            angular_rate = min(smoothed, abs(feedback.signed_limit)) if controller == 'profiled' else smoothed
            if stopped_at is not None and controller != 'native':
                angular_rate = 0.
        plant.step(angular_rate)
        if stopped_at is not None and plant.rate < .015:
            stationary_since = stationary_since if stationary_since is not None else t
            if t - stationary_since >= .55:
                break
        else:
            stationary_since = None
    settled = stationary_since is not None and t - stationary_since >= .55
    return round(math.degrees(plant.angle - target), 4), round(t, 3), plant.broken_away, settled


def scenarios(turns, angles, repetitions, seed):
    stress = random.Random(seed)
    for original in turns:
        for angle in angles:
            for _ in range(repetitions):
                profile = replace(original, breakaway_s=original.breakaway_s * stress.uniform(.70, 1.4),
                                  stop_delay_s=clamp(original.stop_delay_s + stress.uniform(-.025, .075), .03, .44),
                                  deceleration=original.deceleration * stress.uniform(.78, 1.22))
                yield profile, angle, stress.randrange(1 << 30)


def identify(train):
    micro_angle = clamp(2. * median(p.coast_degrees for p in train), 6., 12.)
    deadtime = clamp(quantile([p.stop_delay_s for p in train], .65) + .05, .12, .35)
    decelerations = [p.deceleration for p in train]
    deceleration = quantile(decelerations, .32)
    initial = Calibration(micro_angle, .165, deadtime, deceleration,
                          TaperProfile(.175, .03, .11, deadtime, deceleration))
    micro = list(scenarios(train, [angle for angle in (3.2,3.5,4.5,6.,8.5) if angle <= micro_angle], 5, 71))
    larger = list(scenarios(train, [angle for angle in ANGLES if angle > micro_angle], 4, 71))

    def loss(calibration, cases):
        score = 0.
        for profile, angle, seed in cases:
            error, seconds, moved, settled = simulate(profile, angle, 'profiled', calibration, random.Random(seed))
            score += min(abs(error) / 3., 4.) ** 2 + .12 * seconds
            if abs(error) > 3. or not moved or not settled:
                score += 5.
        return score

    # Total reaction time includes the IMU sampling delay and final gate tick.
    micro_candidates = (replace(initial, start_rate=start, delay=lag, deceleration=brake)
                        for start,lag,brake in product((.145,.165,.185),
                            (deadtime,deadtime+.05,deadtime+.10,deadtime+.15),
                            (min(decelerations),deceleration,max(decelerations))))
    fitted = min(micro_candidates, key=lambda candidate:loss(candidate, micro))
    taper_candidates = (replace(fitted, taper=TaperProfile(*settings)) for settings in product(
        (.155, .175), (.015, .03), (.095, .11, .12), (deadtime, min(.40, deadtime + .05)),
        (min(decelerations), median(decelerations), max(decelerations))))
    return min(taper_candidates, key=lambda candidate:loss(candidate, larger))


def compare(traces, seed, repetitions):
    measured = [turn for path in traces for turn in extract_turns(path)]
    by_trace = defaultdict(list)
    for turn in measured:
        by_trace[turn.trace].append(turn)
    train, holdout = [], []
    for turns in by_trace.values():
        split = max(1, len(turns) * 2 // 3) if len(turns) > 1 else 0
        train.extend(turns[:split])
        holdout.extend(turns[split:])
    if min(len(train), len(holdout)) < 2:
        raise ValueError('Need independent recorded turn episodes for identification and evaluation')
    calibration = identify(train)
    results = defaultdict(list)
    for scenario, angle, observation_seed in scenarios(holdout, ANGLES, repetitions, seed):
        for mode in ('native', 'deployed_gyro', 'profiled'):
            results[(mode, angle)].append(simulate(scenario, angle*scenario.direction, mode, R550_TURN, random.Random(observation_seed)))
    rows = []
    for (mode, angle), outcomes in results.items():
        errors = [abs(error) for error, _, _, _ in outcomes]
        rows.append({'controller': mode, 'target_deg': angle, 'trials': len(outcomes),
                     'within_3_deg': round(sum(abs(error) <= 3. and settled for error, _, _, settled in outcomes) / len(outcomes), 4),
                     'p95_error_deg': round(quantile(errors, .95), 3),
                     'median_s': round(median(t for _, t, _, _ in outcomes), 3),
                     'no_motion': sum(not moved for _, _, moved, _ in outcomes),
                     'overshot_by_more_than_3_deg': sum(error > 3. for error, _, _, _ in outcomes),
                     'short_by_more_than_3_deg': sum(error < -3. for error, _, _, _ in outcomes),
                     'not_settled': sum(not settled for _, _, _, settled in outcomes)})
    return {'seed': seed, 'identification_turns': len(train), 'held_out_turns': len(holdout),
            'fit_from_training': asdict(calibration), 'evaluated_calibration': asdict(R550_TURN),
            'field_turns': [asdict(p) for p in measured], 'metrics': rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('traces', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=550)
    parser.add_argument('--repetitions', type=int, default=16)
    args = parser.parse_args()
    if not 1 <= args.repetitions <= 200:
        parser.error('--repetitions must be 1..200')
    result = compare(args.traces, args.seed, args.repetitions)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    for row in result['metrics']:
        print(row)


if __name__ == '__main__':
    main()
