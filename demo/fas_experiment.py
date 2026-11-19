"""Ordered two-vehicle RSSI search and durable state for the fixed base host."""
from collections import Counter, deque
from dataclasses import dataclass
import hashlib
from itertools import permutations
import json
import math
from pathlib import Path
import re
import sqlite3
from statistics import median
import time
import uuid

from arrival_policy import FAS_STATION_POSITION_TOLERANCE, FAS_STATION_YAW_TOLERANCE


PROTOCOL = 'FAS_PAIR_1'
CARS = ('A', 'B')
BANDS = ('sle', 'c5')
MIN_DELTA_DB = 6.0
STATE_COUNT = 4096
MIN_STATION_SEPARATION = .95
LOCALIZATION_PAD = .60
EXCLUSIVE_LOCALIZATION_PAD = .90
PARKED_PEER_CLEARANCE = .60
MAX_FLEET_SIGMA = .13
MIN_SCAN_AGREEMENT = .80
RECOVERY_STATE = -3


class RecoveryWait(ValueError):
    """The run can continue when the missing operating condition recovers."""


def canonical(obj):
    return json.dumps(obj, sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False)


def mac(value):
    address = re.sub('[:-]', '', str(value)).lower()
    if not re.fullmatch('[0-9a-f]{12}', address):
        raise ValueError('需要六字节完整设备地址')
    return ':'.join(address[i:i+2] for i in range(0, 12, 2))


def station_ids(links):
    result = {band: {car: mac(links[band][car]) for car in CARS} for band in BANDS}
    if any(values['A'] == values['B'] for values in result.values()):
        raise ValueError('同频段两车发端地址必须唯一')
    return result


def point_xy(point):
    return point['x'], point['y']


def yaw_error(first, second):
    return abs(math.remainder(first-second, 2*math.pi))


class Stations:
    def __init__(self, route):
        self.map_id = route['map_id']
        self.points = {}
        self.order = {}
        if not re.fullmatch('[0-9a-f]{64}', self.map_id):
            raise ValueError('点位文件缺少有效 map_id')
        items = route['points']
        if len(items) != 50:
            raise ValueError('采集路线应包含 50 个点位')
        self.yaw = items[0]['yaw']
        for i, point in enumerate(items):
            name = point['id']
            values = (*point_xy(point), point['yaw'])
            if (not isinstance(name, str) or not name or name in self.points
                    or not all(type(v) in (int, float) and math.isfinite(v) for v in values)
                    or yaw_error(point['yaw'], self.yaw) > 1e-6):
                raise ValueError('点位必须唯一、坐标有效并使用同一朝向')
            self.points[name] = dict(point)
            if i:
                distance += math.dist(point_xy(items[i-1]), point_xy(point))
            else:
                distance = 0.
            self.order[name] = distance
        self.pairs = tuple((a, b) for a, b in permutations(self.points, 2)
                           if math.dist(point_xy(self.points[a]), point_xy(self.points[b])) >= MIN_STATION_SEPARATION)
        if not self.pairs:
            raise ValueError('路线没有满足两车间距的位置对')

    def ranked(self, positions, tested, preferred=None, history=None):
        """Use prior measurements and movement along the U; no RSSI distance model."""
        history = history or {}

        def rank(pair):
            distance = sum(abs(self.order[dest]-self.order[origin])
                           for dest, origin in zip(pair, positions) if origin in self.order)
            return (pair != preferred,
                    -history.get(pair, 0), distance, pair)

        return sorted((pair for pair in self.pairs if pair not in tested), key=rank)

    def motion_ranked(self, positions, last_pair, visits):
        def rank(pair):
            distance = sum(abs(self.order[dest]-self.order[origin]) for dest,origin in zip(pair,positions))
            repeated = sum(visits[dest] for dest in pair)
            stationary = sum(dest==origin for dest,origin in zip(pair,positions))
            return (distance + 1.6*repeated + .65*stationary, pair)

        return sorted((pair for pair in self.pairs if pair!=last_pair), key=rank)


def sample_box(path, pad=LOCALIZATION_PAD):
    xs, ys = zip(*[point[:2] for point in path])
    return min(xs)-pad, max(xs)+pad, min(ys)-pad, max(ys)+pad


def boxes_apart(first, second, gap=.10):
    return max(first[0]-second[1], second[0]-first[1],
               first[2]-second[3], second[2]-first[3]) >= gap


def parallel_bounds(paths):
    """Reserve room for replanning on each side of a verified divider."""
    planned = {car:sample_box(paths[car]) for car in CARS}
    gap, edge, lead, follow = max((planned[b][axis]-planned[a][axis+1],axis,a,b)
                                  for axis in (0,2) for a,b in (CARS,CARS[::-1]))
    if gap<.10:
        return None
    split = (planned[lead][edge+1]+planned[follow][edge])/2
    bounds = {car:list(sample_box(paths[car],pad=EXCLUSIVE_LOCALIZATION_PAD)) for car in CARS}
    bounds[lead][edge+1] = min(bounds[lead][edge+1],split-.05)
    bounds[follow][edge] = max(bounds[follow][edge],split+.05)
    return bounds


def position_sigma(health):
    cov = health.get('localization_covariance')
    if not cov or len(cov)<2 or any(v is None or not math.isfinite(v) or v<0 for v in cov[:2]):
        return None
    return math.sqrt(max(cov[:2]))


def scan_confirmed(health):
    score=health.get('scan_map_agreement') or 0.
    window=health.get('scan_window_agreement',score)
    return (health.get('scan_structure_verified') is True
            or (window is not None and window>=MIN_SCAN_AGREEMENT))


class ScanEvidence:
    """Brief independent scan evidence, invalidated by stale data or a pose jump."""
    def __init__(self):
        self.frames=deque(maxlen=6)

    def add(self, stamp, score, verified, sensor_pose):
        if not self.frames or stamp>self.frames[-1][0]:
            self.frames.append((stamp,score,verified,sensor_pose))

    def latest(self, now):
        recent=[row for row in self.frames if -.03<=now-row[0]<=.80]
        if len(recent)<3 or now-recent[-1][0]>.38:
            return None,False
        current=recent[-1][3]
        linked=[row for row in recent if math.dist(row[3][:2],current[:2])<=.075
                and yaw_error(row[3][2],current[2])<=math.radians(6)]
        if len(linked)<3:
            return None,False
        matches=[row for row in linked if row[2] and now-row[0]<=.65]
        score=median(row[1] for row in linked)
        return score,len(matches)>=2 and now-matches[-1][0]<=.40


def fleet_pose_ready(health):
    age, sigma = health.get('pose_age_s'), position_sigma(health)
    return bool(health.get('ready') and health.get('localization_confirmed') and health.get('pose')
                and age is not None and 0.<=age<=.45 and sigma is not None and sigma<=MAX_FLEET_SIGMA
                and scan_confirmed(health) and not health.get('fas_localization_active'))


def stopped_station_arrival(health):
    """Keep a completed physical stop observable after its motion lease ends."""
    arrival=health.get('arrival') or {}
    age=health.get('pose_age_s')
    position,heading=(arrival.get(key) for key in ('position_error','yaw_error_deg'))
    return bool(health.get('stationary') and health.get('ready') and health.get('localization_confirmed')
                and age is not None and 0.<=age<=.45 and arrival.get('kind')=='waypoint'
                and arrival.get('status') in ('position_settling','heading_settling','succeeded')
                and position is not None and 0.<=position<=FAS_STATION_POSITION_TOLERANCE
                and heading is not None and 0.<=heading<=math.degrees(FAS_STATION_YAW_TOLERANCE))


def peer_clearance(first_sigma, second_sigma, *, moving_together=False):
    minimum = .70 if moving_together else PARKED_PEER_CLEARANCE
    return max(minimum, .38+2*(first_sigma+second_sigma))


def clear_of_parked_peer(path, peer, clearance=PARKED_PEER_CLEARANCE):
    points = [point[:2] for point in path]
    if not points or any(math.dist(point,peer[:2]) < clearance for point in points):
        return False
    px,py = peer[:2]
    for (ax,ay),(bx,by) in zip(points,points[1:]):
        dx,dy = bx-ax,by-ay
        squared = dx*dx+dy*dy
        if squared:
            fraction = max(0.,min(1.,((px-ax)*dx+(py-ay)*dy)/squared))
            if math.hypot(ax+fraction*dx-px,ay+fraction*dy-py) < clearance:
                return False
    return True


def move_batches(starts, ends, paths, *, exclusive=False, clearance=PARKED_PEER_CLEARANCE):
    """Reserve the full planned corridor, including the stationary peer's footprint.

    Each returned step uses independent onboard bounds before movement starts.
    An empty tuple means already positioned. None means neither ordering is safe.
    """
    move = {car: sample_box(paths[car]) for car in paths if paths[car]}
    if not move:
        return ()
    if len(move) == 2 and not exclusive and boxes_apart(move['A'], move['B']):
        return (CARS,)
    for lead, follow in (CARS, CARS[::-1]):
        if lead not in move:
            continue
        if not clear_of_parked_peer(paths[lead], starts[follow], clearance):
            continue
        if follow not in move:
            return ((lead,),)
        if clear_of_parked_peer(paths[follow], ends[lead], clearance):
            return ((lead,), (follow,))
    return None


def phase_state(index, c5_codes):
    if type(index) is not int or not 0 <= index < STATE_COUNT:
        raise ValueError('状态编号须为 0–4095')
    return {'sle': list(divmod(index, 64)), 'c5': c5_codes[index]}


def capture_request(run_id, trial_id, index, poses, state, links, *, min_samples=30, window_s=1.):
    return {'protocol':PROTOCOL, 'run_id':run_id, 'trial_id':trial_id, 'state_id':index,
            'radios':state, 'tx':links, 'cars':poses, 'min_samples':min_samples, 'window_s':window_s}


def evaluate(request, receipt, receivers):
    """Validate fresh equal-state A/B evidence independently on each requested receiver."""
    for key in ('protocol','run_id','trial_id','state_id','radios','tx'):
        if receipt.get(key) != request[key]:
            raise ValueError('回执与请求的轮次、硬件状态或发端身份不一致')
    if receipt.get('status') != 'complete' or not receipt.get('raw_ref'):
        raise ValueError('基站未确认完整原始数据已保存')
    data = receipt['data']
    selected = {}
    for key in receivers:
        row = data.get(key)
        if not isinstance(row, dict) or type(row.get('epoch')) not in (str,int) or row['epoch'] == '':
            raise ValueError(f'{key} 缺少有效采集窗口')
        values = row.get('rssi_dbm', {})
        counts = row.get('samples', {})
        if any(type(counts.get(car)) is not int or counts[car] < request['min_samples']
               or type(values.get(car)) not in (int,float) or not -127 <= values[car] <= 0
               for car in CARS):
            raise ValueError(f'{key} 两车样本不足或 RSSI 无效')
        selected[key] = round(values['A']-values['B'], 9)
    return selected


@dataclass(frozen=True)
class C5Sample:
    epoch: int
    address: str
    rssi_dbm: int
    timestamp_ms: int
    gpio_state: int
    payload: str

    @classmethod
    def parse(cls, line):
        fields = line.strip().split(',', 6)
        if len(fields) != 7 or fields[0] != 'D':
            raise ValueError('C5 字段格式无效')
        _, epoch, address, rssi, stamp, state, payload = fields
        result = cls(int(epoch), mac(address), int(rssi), int(stamp),
                     int(state, 16 if state.lower().startswith('0x') else 10), payload)
        if (min(result.epoch,result.timestamp_ms,result.gpio_state) < 0
                or result.gpio_state > 0xffffff or not -127 <= result.rssi_dbm <= 0):
            raise ValueError('C5 数据范围无效')
        return result


def aggregate(rows, expected_epoch, expected_code, addresses):
    """Collect separately by transmitter, deduplicated by hardware sample identity."""
    samples = {car: [] for car in CARS}
    seen = set()
    by_address = {address:car for car,address in addresses.items()}
    for row in rows:
        car = by_address.get(row.address)
        key = row.address, row.timestamp_ms, row.payload
        if car and row.epoch == expected_epoch and row.gpio_state == expected_code and key not in seen:
            seen.add(key)
            samples[car].append(row.rssi_dbm)
    return {'epoch':expected_epoch, 'samples':{car:len(v) for car,v in samples.items()},
            'rssi_dbm':{car:median(v) for car,v in samples.items() if v}}


class Journal:
    def __init__(self, path, config):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = self.path.with_suffix(self.path.suffix+'.lock').open('a+b')
        try:
            self.lock.seek(0)
            if self.lock.read(1) == b'':
                self.lock.write(b'\0')
                self.lock.flush()
            self.lock.seek(0)
            import os
            if os.name == 'nt':
                import msvcrt
                msvcrt.locking(self.lock.fileno(),msvcrt.LK_NBLCK,1)
            else:
                import fcntl
                fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError as exc:
            self.lock.close()
            raise ValueError('该实验文件已有一个运行进程') from exc
        self.connection = sqlite3.connect(self.path, timeout=3.)
        self.connection.row_factory = sqlite3.Row
        self.connection.execute('PRAGMA journal_mode=WAL')
        self.connection.execute('PRAGMA synchronous=FULL')
        self.connection.executescript('''
            CREATE TABLE IF NOT EXISTS run (id TEXT PRIMARY KEY, config TEXT NOT NULL UNIQUE);
            CREATE TABLE IF NOT EXISTS states (
                state_id INTEGER PRIMARY KEY, status TEXT NOT NULL, details TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS trials (
                id TEXT PRIMARY KEY, state_id INTEGER NOT NULL, a TEXT NOT NULL, b TEXT NOT NULL,
                status TEXT NOT NULL, request TEXT NOT NULL, receipt TEXT, delta TEXT,
                created_utc REAL NOT NULL, completed_utc REAL);
            CREATE INDEX IF NOT EXISTS trials_state ON trials (state_id,status);
            CREATE UNIQUE INDEX IF NOT EXISTS one_pending ON trials (status) WHERE status='pending';
        ''')
        fingerprint = hashlib.sha256(canonical(config).encode()).hexdigest()
        with self.connection:
            existing = self.connection.execute('SELECT * FROM run').fetchone()
            if existing and existing['config'] != fingerprint:
                self.lock.close()
                raise ValueError('路线、射频配置或判据有变化，请使用新实验文件')
            self.run_id = existing['id'] if existing else uuid.uuid4().hex
            self.connection.execute('INSERT OR IGNORE INTO run VALUES (?,?)', (self.run_id,fingerprint))

    def pending(self):
        return self.connection.execute("SELECT * FROM trials WHERE status='pending'").fetchone()

    def close(self):
        self.connection.close()
        self.lock.close()

    def charging(self):
        row=self.connection.execute("SELECT details FROM states WHERE state_id=-1").fetchone()
        return json.loads(row['details']) if row else []

    def set_charging(self, cars):
        with self.connection:
            self.connection.execute("INSERT OR REPLACE INTO states VALUES (-1,'charging',?)",(canonical(cars),))

    def set_activity(self, stage, message):
        with self.connection:
            self.connection.execute("INSERT OR REPLACE INTO states VALUES (?,?,?)",
                                    (RECOVERY_STATE,stage,canonical({'message':message})))

    def motion_completed(self):
        return self.connection.execute("SELECT COUNT(*) FROM states WHERE state_id>=0 AND status='arrived'").fetchone()[0]

    def queue_charge_check(self):
        with self.connection:
            if self.state(-2) is None:
                self.complete(-2,'charge_check',{'cars':list(CARS)})
                self.connection.execute("INSERT OR REPLACE INTO states VALUES (-1,'charging',?)",(canonical(CARS),))

    def tested(self, state):
        return {(r['a'],r['b']) for r in self.connection.execute(
            "SELECT a,b FROM trials WHERE state_id=? AND status IN ('pass','miss')", (state,))}

    def motion_failures(self, state):
        failed=self.connection.execute("SELECT a,b FROM trials WHERE state_id=? AND status='invalid'",(state,))
        return Counter((row['a'],row['b']) for row in failed)

    def prior_winners(self):
        rows = self.connection.execute("SELECT a,b,COUNT(*) AS n FROM trials WHERE status='pass' GROUP BY a,b")
        return {(r['a'],r['b']):r['n'] for r in rows}

    def state(self, state_id):
        return self.connection.execute('SELECT * FROM states WHERE state_id=?', (state_id,)).fetchone()

    def start(self, pair, state, request):
        with self.connection:
            self.connection.execute('INSERT INTO trials VALUES (?,?,?,?,?,?,?,?,?,?)',
                (request['trial_id'],state,*pair,'pending',canonical(request),None,None,time.time(),None))

    def finish(self, trial, receipt, deltas):
        threshold_met = all(value >= MIN_DELTA_DB for value in deltas.values())
        status = 'pass' if threshold_met else 'miss'
        with self.connection:
            changed = self.connection.execute(
                "UPDATE trials SET status=?,receipt=?,delta=?,completed_utc=? WHERE id=? AND status='pending'",
                (status,canonical(receipt),canonical(deltas),time.time(),trial['id'])).rowcount
            if changed != 1:
                raise ValueError('当前采样请求已结束')
            if threshold_met:
                self.complete(trial['state_id'],'pass',{'trial_id':trial['id'],'pair':[trial['a'],trial['b']],'delta_db':deltas})
        return status

    def abandon(self, trial, reason, receipt=None):
        with self.connection:
            self.connection.execute("UPDATE trials SET status='invalid',receipt=?,completed_utc=? WHERE id=? AND status='pending'",
                                    (canonical({'reason':reason,'receipt':receipt}),time.time(),trial['id']))

    def finish_motion(self, trial, poses):
        with self.connection:
            changed=self.connection.execute(
                "UPDATE trials SET status='arrived',receipt=?,completed_utc=? WHERE id=? AND status='pending'",
                (canonical(poses),time.time(),trial['id'])).rowcount
            if changed!=1:
                raise ValueError('当前行驶轮次已结束')
            self.complete(trial['state_id'],'arrived',
                          {'trial_id':trial['id'],'pair':[trial['a'],trial['b']],'poses':poses})

    def complete(self, state, status, details):
        self.connection.execute('INSERT INTO states VALUES (?,?,?)', (state,status,canonical(details)))

    def report(self):
        return [{'state_id':r['state_id'],'status':r['status'],**json.loads(r['details'])}
                for r in self.connection.execute('SELECT * FROM states WHERE state_id>=0 ORDER BY state_id')]


def position_rounds(journal, stations, driver, *, start, count, max_pairs, recharge_after=None, order_pairs=None):
    """Rehearse the real trips and charge policy without any RF results."""
    completed=journal.report()
    visits=Counter(point for row in completed for point in row['pair'])
    last_pair=tuple(completed[-1]['pair']) if completed else None
    for number in range(start,start+count):
        if journal.state(number):
            continue
        if recharge_after is not None and len(completed)>=recharge_after:
            journal.queue_charge_check()
        driver.service_energy()
        pending=journal.pending()
        if pending is not None and pending['state_id']!=number:
            raise ValueError('还有未完成的位置对，请沿用原轮次范围继续')
        candidates=stations.motion_ranked(driver.positions(),last_pair,visits)
        candidates.sort(key=journal.motion_failures(number).__getitem__)
        if order_pairs is not None:
            candidates=order_pairs(candidates,max_pairs)
        tried=set()
        if pending is not None:
            candidates.insert(0,(pending['a'],pending['b']))
        for pair in candidates:
            if pair in tried:
                continue
            if len(tried)>=max_pairs:
                break
            tried.add(pair)
            if pending is None:
                trial_id=uuid.uuid4().hex
                journal.start(pair,number,{'trial_id':trial_id,'purpose':'motion_only','pair':pair})
                pending=journal.pending()
            print(f'轮次 {number+1}，一号车 {pair[0]}，二号车 {pair[1]}',flush=True)
            poses=driver.place(pair)
            if poses is not None:
                journal.finish_motion(pending,poses)
                completed.append({'pair':pair})
                visits.update(pair)
                last_pair=pair
                print(f'轮次 {number+1} 两车已停稳',flush=True)
                break
            journal.abandon(pending,'未找到能让两车到站的安全路线')
            pending=None
            print(f'轮次 {number+1} 保留，复核下一组位置',flush=True)
        if not journal.state(number):
            raise RecoveryWait(f'第 {number+1} 组暂无可安全完成的位置对')


def search(journal, stations, c5_codes, links, receivers, driver, *, start=0, count=STATE_COUNT,
           max_pairs=12, window_s=1., min_samples=30):
    """Run a signed, state-first search. An interrupted window remains retryable."""
    preferred = None
    pending = journal.pending()
    if pending:
        prior = json.loads(pending['request'])
        receipt = driver.reconcile(prior)
        if receipt is not None:
            journal.finish(pending,receipt,evaluate(prior,receipt,receivers))
        else:
            journal.abandon(pending,'中断的窗口已关闭，待重新核对车位及状态')
    for state in range(start, start+count):
        saved = journal.state(state)
        if saved:
            if saved['status']=='pass':
                preferred=tuple(json.loads(saved['details'])['pair'])
            continue
        phase = phase_state(state,c5_codes)
        tested = journal.tested(state)
        unavailable = set()
        invalid = {}
        prior_winners = journal.prior_winners()
        while len(tested)<max_pairs:
            driver.service_energy()
            candidates=stations.ranked(driver.positions(),tested|unavailable,preferred,prior_winners)
            if not candidates:
                break
            pair=candidates[0]
            poses=driver.place(pair)
            if poses is None:
                unavailable.add(pair)
                continue
            trial_id=uuid.uuid4().hex
            request=capture_request(journal.run_id,trial_id,state,poses,phase,links,
                                    min_samples=min_samples,window_s=window_s)
            journal.start(pair,state,request)
            receipt=driver.capture(request)
            try:
                deltas=evaluate(request,receipt,receivers)
            except (ValueError,KeyError) as exc:
                journal.abandon(journal.pending(),str(exc),receipt)
                invalid[pair] = invalid.get(pair,0)+1
                if invalid[pair] >= 2:
                    raise ValueError('连续出现无效采样，当前状态仍待采') from exc
                continue
            status=journal.finish(journal.pending(),receipt,deltas)
            print(f'状态 {state:04d} {pair[0]} / {pair[1]} {status} {deltas}',flush=True)
            tested.add(pair)
            if status=='pass':
                preferred=pair
                break
        if not journal.state(state):
            if not tested and unavailable:
                raise ValueError('两车当前无可安全调度的位置对，保留待采状态')
            status=('no_match' if len(tested)==len(stations.pairs)
                    else 'budget_exhausted' if len(tested)>=max_pairs else 'routes_unavailable')
            with journal.connection:
                journal.complete(state,status,{'tested_pairs':len(tested),'unavailable_pairs':len(unavailable)})
            print(f'状态 {state:04d} {status} · {len(tested)} 个有效位置对',flush=True)
    return journal.report()
