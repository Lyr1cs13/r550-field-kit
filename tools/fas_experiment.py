#!/usr/bin/env python3
"""Run a resumable FAS pair search from the base host. Offline rehearsal by default."""
import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor, wait
from contextlib import closing
from functools import partial
from http.client import IncompleteRead
import json
import math
from pathlib import Path
from statistics import median
import sqlite3
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen
import uuid

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'demo'))
from aisle_clearance import AisleClearance
from arrival_policy import FAS_STATION_POSITION_TOLERANCE, FAS_STATION_YAW_TOLERANCE
from fas_experiment import (CARS, EXCLUSIVE_LOCALIZATION_PAD, LOCALIZATION_PAD, MAX_FLEET_SIGMA,
                            PROTOCOL, STATE_COUNT, Journal, RecoveryWait, Stations, boxes_apart, canonical,
                            capture_request, clear_of_parked_peer, evaluate, fleet_pose_ready, move_batches,
                            parallel_bounds, peer_clearance, phase_state, point_xy, position_rounds, position_sigma,
                            sample_box, search, station_ids, stopped_station_arrival, sweep_pairs, yaw_error)
from fas_car import NOT_PARKED, WS63_IDS


DOCK_SETTLE_SECONDS = 30.
SHORT_RETURN_DISTANCE = 2.
SHORT_RETURN_RECHECK = 90.
PARKED_PHASES = ('idle','docked','completed','failed')
SERIAL_CLOSE_FAULT='接收板关窗未确认'


class ChargeNeeded(Exception):
    def __init__(self, cars):
        self.cars = cars


class RouteRetry(Exception):
    pass


class VehicleLinkInterrupted(RouteRetry):
    pass


class StationHeadingRetry(RouteRetry):
    pass


class DockRouteConflict(RecoveryWait):
    pass


class ReturnRequested(Exception):
    pass


class PauseRequested(Exception):
    pass


def recorded_charge_plateau(database, car, origin, floor, *, now=None):
    """Check the current dock session for a long plateau under the vehicle's normal load."""
    now=time.time() if now is None else now
    start=now-24*60
    if start-origin<60.:
        return None
    try:
        with closing(sqlite3.connect(database.resolve().as_uri()+'?mode=ro',uri=True,timeout=.25)) as db:
            peak=db.execute("""
                SELECT MAX(amps) FROM (
                    SELECT MIN(current) AS amps FROM samples
                    WHERE car=? AND stamp BETWEEN ? AND ? AND mode='charging'
                      AND ABS(charge_start-?)<=3 AND current>=0
                    GROUP BY CAST((stamp-?)/60 AS INTEGER)
                    HAVING COUNT(*)>=10 AND MAX(stamp)-MIN(stamp)>=50
                )""",(car,origin-3.,start,origin,origin)).fetchone()[0]
            rows=db.execute("""SELECT stamp,mode,charge_start,voltage,current FROM samples
                               WHERE car=? AND stamp BETWEEN ? AND ? ORDER BY stamp""",
                            (car,start,now)).fetchall()
    except (OSError,sqlite3.Error):
        return None
    if (peak is None or not math.isfinite(peak) or peak<.90 or not rows or rows[0][0]-start>8. or now-rows[-1][0]>8.
            or any(b[0]-a[0]>12. for a,b in zip(rows,rows[1:]))):
        return None
    if any(mode!='charging' or any(v is None or not math.isfinite(v) for v in (since,voltage,amps))
           or abs(since-origin)>3. or amps<0. for _,mode,since,voltage,amps in rows):
        return None
    quarters=[[] for _ in range(4)]
    for row in rows:
        quarters[min(3,int((row[0]-start)//360))].append(row)
    if any(len(group)<60 for group in quarters):
        return None
    levels=[median(row[3] for row in group) for group in quarters]
    currents=[median(row[4] for row in group) for group in quarters]
    limit=.65*peak
    if (min(levels)<floor or max(levels)-min(levels)>.012 or abs(levels[-1]-levels[0])>.006
            or max(currents)>limit or max(currents)-min(currents)>.09):
        return None
    return levels[-1],limit


class ChargePlateau:
    """Qualify a near-full charge using voltage, current, and the verified seat."""
    TAPER_WINDOW = 120.
    TAPER_CURRENT = .55

    def __init__(self, near_full=None, bulk_charge_start=None, recorded_check=None):
        self.samples = deque()
        self.near_full = near_full
        self.recorded_check = recorded_check
        self.window = 240. if near_full is None else 360.
        self.reset(bulk_charge_start)

    def reset(self, bulk_charge_start=None):
        self.samples.clear()
        self.ready = False
        self.level = None
        self.reason = None
        self.charged = None
        self.retained = False
        self.ready_elapsed = 0.
        self.bulk_since = None
        self.bulk_charge_start = bulk_charge_start
        self.last_recorded_check = float('-inf')
        self.recorded_tail = None

    def observe(self, now, health, floor, target=12.2, *, wall_stamp=None):
        voltage = health['voltage']
        elapsed = health.get('charging_elapsed_s')
        valid_elapsed = isinstance(elapsed,(int,float)) and math.isfinite(elapsed) and elapsed >= 0.
        origin = (now if wall_stamp is None else wall_stamp)-elapsed if valid_elapsed else None
        bulk = self.bulk_charge_start
        if bulk is not None and (origin is None or abs(origin-bulk)>3.
                                 or not health['stationary'] or not health['charging_stable']):
            bulk = None
        if (not health['stationary'] or not health['localization_confirmed'] or not health.get('ready',True)
                or voltage is None or not math.isfinite(voltage) or voltage<floor):
            self.reset(bulk)
            return
        if not health['charging_stable']:
            if self.preserve_completed_charge(health,voltage):
                return
            self.reset()
            return
        if health.get('dock_localization',{}).get('status')!='confirmed' or not valid_elapsed:
            self.reset(bulk)
            return
        self.retained=False
        if self.bulk_charge_start is not None and bulk is None:
            self.reset()
        if self.samples:
            last_time,_,last_elapsed,_ = self.samples[-1]
            if now-last_time > 12. or elapsed < last_elapsed:
                # A gap invalidates the voltage window. The onboard clock can still
                # identify an uninterrupted charge previously seen at high current.
                self.reset(bulk if elapsed>=last_elapsed else None)
            elif now-last_time < 5.:
                if self.level is not None and abs(voltage-self.level) > .020:
                    self.ready = False
                    self.reason = None
                return
        current=health.get('current')
        if isinstance(current,(int,float)) and math.isfinite(current) and current>=.90:
            if self.bulk_since is None:
                self.bulk_since=now
            if self.bulk_charge_start is None and now-self.bulk_since>=60.:
                self.bulk_charge_start=origin
        else:
            self.bulk_since=None
        self.samples.append((now,voltage,elapsed,current))
        while len(self.samples)>1 and now-self.samples[1][0]>=self.window:
            self.samples.popleft()
        self.ready=False
        self.reason=None
        if self.near_full is not None and now-self.samples[0][0]>=self.TAPER_WINDOW:
            recent=[row for row in self.samples if now-row[0]<=self.TAPER_WINDOW]
            if voltage>=self.near_full and median(row[1] for row in recent)>=self.near_full and self.tapered(recent):
                self.qualify(health,voltage,elapsed,'接近充足电压且电流持续回落')
                return
        if now-self.samples[0][0]>=self.window:
            segment=self.window/4
            spans=((0.,segment),(segment,2*segment),(2*segment,3*segment),(3*segment,math.inf))
            levels=[median(v for stamp,v,_,_ in self.samples if low<=now-stamp<high) for low,high in spans]
            self.level=levels[0]
            flat=max(levels)-min(levels)<=.010 and levels[0]-levels[-1]<=.006 and abs(voltage-self.level)<=.020
            if flat and (self.near_full is None or min(levels)>=target-.005 or
                         (self.bulk_charge_start is not None and self.tapered(self.samples))):
                self.qualify(health,voltage,elapsed,'达到续采下限并通过电压平台复核')
            elif flat and self.recorded_check and self.bulk_charge_start is not None:
                if now-self.last_recorded_check>=30.:
                    self.recorded_tail=self.recorded_check(self.bulk_charge_start,floor)
                    self.last_recorded_check=now
                if self.recorded_tail:
                    level,limit=self.recorded_tail
                    if (abs(voltage-level)<=.020 and isinstance(current,(int,float))
                            and 0. <= current <= limit+.06):
                        self.qualify(health,voltage,elapsed,'本轮电流已明显回落，连续 24 分钟电压稳定')

    def tapered(self, rows):
        currents=[i for _,_,_,i in rows if isinstance(i,(int,float)) and math.isfinite(i) and i>=0.]
        return (bool(currents) and len(currents)>=.85*len(rows) and median(currents)<=self.TAPER_CURRENT
                and sum(i<=self.TAPER_CURRENT+.10 for i in currents)>=.90*len(currents))

    def qualify(self, health, voltage, elapsed, reason):
        self.ready=True
        self.ready_elapsed=elapsed
        self.reason=reason
        if self.near_full is not None and health.get('pose'):
            self.charged=(tuple(health['pose']),voltage,elapsed)

    def preserve_completed_charge(self, health, voltage):
        if self.charged is None or health.get('pose_age_s') is None or health['pose_age_s']>.45:
            return False
        seat,level,elapsed=self.charged
        pose=health.get('pose')
        if (not pose or math.dist(seat[:2],pose[:2])>.06 or yaw_error(seat[2],pose[2])>math.radians(6)
                or voltage<level-.18):
            return False
        self.ready=self.retained=True
        self.ready_elapsed=elapsed
        self.reason='已确认充足，停充后仍在原桩'
        self.samples.clear()
        return True


class Endpoint:
    def __init__(self, url):
        parts = urlsplit(url)
        if (parts.scheme not in ('http','https') or not parts.hostname or parts.username
                or parts.password or parts.path not in ('','/') or parts.query or parts.fragment):
            raise ValueError('服务地址须是 HTTP(S) 主机和端口')
        self.url = url.rstrip('/')

    def fetch(self, path, value=None, headers=None, timeout=.85, missing=False):
        request = Request(self.url+path,data=canonical(value).encode() if value is not None else None,
                          headers={'Content-Type':'application/json',**(headers or {})})
        try:
            with urlopen(request,timeout=timeout) as response:
                return json.load(response)
        except IncompleteRead as exc:
            raise OSError('设备回执在传输中断开') from exc
        except HTTPError as exc:
            if missing and exc.code == 404:
                return None
            try:
                message = json.load(exc).get('error',f'HTTP {exc.code}')
            except (ValueError, AttributeError, IncompleteRead):
                message = f'HTTP {exc.code}'
            if exc.code in (502,503,504):
                raise OSError(message) from exc
            raise ValueError(message) from exc


class Base:
    def __init__(self, settings, directory):
        self.api = Endpoint(settings['address'])
        key = (directory/settings['key_file']).read_text(encoding='utf-8').strip()
        if not key or any(ch.isspace() for ch in key):
            raise ValueError('请使用采集基站分配的访问密钥')
        self.headers = {'X-FAS-Key':key}
        self.receipt_wait = False

    def get(self, path, *, missing=False):
        return self.api.fetch(path,headers=self.headers,timeout=2.,missing=missing)

    def post(self, path, data):
        try:
            return self.api.fetch(path,data,self.headers,timeout=2.)
        except ValueError as exc:
            message=str(exc)
            if path=='/fas/capture' and (message=='基站已在采样' or message.startswith(SERIAL_CLOSE_FAULT)):
                raise RecoveryWait(message) from exc
            raise

    def receipt(self, trial, *, wait=False):
        suffix = '?wait=0.4' if wait and self.receipt_wait else ''
        result = self.get('/fas/capture/'+trial+suffix,missing=True)
        if result and result.get('trial_id') != trial:
            raise ValueError('基站返回了不同的采样窗口')
        return result

    def cancel(self, request):
        identity={k:request[k] for k in ('run_id','trial_id')}
        deadline=time.monotonic()+8.
        while True:
            result=self.post('/fas/cancel',identity)
            status=result.get('status')
            if status in ('idle','canceled','complete'):
                return
            error=result.get('error') or ''
            if status=='blocked' and (error.startswith(SERIAL_CLOSE_FAULT) or error=='另一采样窗口仍在运行'):
                raise RecoveryWait(error)
            if status!='canceling':
                raise ValueError(error or '基站尚未关闭上一采样窗口')
            if time.monotonic()>=deadline:
                raise RecoveryWait('等待基站确认关闭上一采样窗口')
            time.sleep(.15)


class Live:
    def __init__(self, settings, config_dir, stations, journal, links, receivers, *, motion_only=False, pause_file=None,
                 energy_history=None):
        self.stations, self.journal, self.links = stations,journal,links
        self.cars = {car:Endpoint(settings['cars'][car]) for car in CARS}
        if len({value.url for value in self.cars.values()}) != 2:
            raise ValueError('两车必须分别使用独立的车端入口')
        self.motion_only = motion_only
        self.auto_charging=False
        self.manual_returning=False
        self.manual_controls_ready=False
        self.pause_file = pause_file
        self.base = None if motion_only else Base(settings['base'],config_dir)
        self.receivers = receivers
        self.pool = ThreadPoolExecutor(max_workers=4)
        self.states = {}
        self.compact_status = dict.fromkeys(CARS,True)
        self.started_low = {car:None for car in CARS}
        self.critical_low_since = dict.fromkeys(CARS)
        self.short_return_checks = {}
        self.initial_departure = {}
        self.early_launch_voltage = None
        plateau = settings.get('charge_until_plateau',False)
        if type(plateau) is not bool:
            raise ValueError('充电平台期设置须为布尔值')
        self.dual_departure_pending = False
        self.resume_voltage_by_car=settings.get('resume_voltage_by_car',{})
        self.return_voltage_by_car=settings.get('return_voltage_by_car',{})
        self.near_return_voltage_by_car=settings.get('near_return_voltage_by_car',{})
        near_full=settings.get('near_full_voltage_by_car',{})
        self.resume_min_charge_seconds=settings.get('resume_min_charge_seconds')
        for voltages in (self.resume_voltage_by_car,self.return_voltage_by_car,self.near_return_voltage_by_car,near_full):
            if (not isinstance(voltages,dict) or not set(voltages)<=set(CARS) or
                    any(type(v) not in (int,float) or not math.isfinite(v) for v in voltages.values())):
                raise ValueError('实验电压配置必须为按 A/B 车提供的有限数值')
        if near_full and (set(near_full)!=set(CARS) or not plateau or motion_only):
            raise ValueError('接近充足电压须在双车采集的电压平台模式下同时为 A/B 配置')
        self.charge_plateaus = {}
        if plateau and not motion_only:
            for car in CARS:
                check=partial(recorded_charge_plateau,energy_history,car) if energy_history and car in near_full else None
                self.charge_plateaus[car]=ChargePlateau(near_full.get(car),journal.bulk_charge_start(car),check)
        if self.resume_min_charge_seconds is not None and (not plateau or
                type(self.resume_min_charge_seconds) not in (int,float) or
                not math.isfinite(self.resume_min_charge_seconds) or self.resume_min_charge_seconds<300.):
            raise ValueError('实验最短充电时间须在平台模式下设为至少 300 秒')
        self.cache = {}
        self.anchor = None
        self.aisle_margin = settings.get('aisle_clearance_m')
        if self.aisle_margin is not None and (
                type(self.aisle_margin) not in (int, float) or not .22 <= self.aisle_margin <= .35):
            raise ValueError('采集路线净空设置须在 0.22 到 0.35 米之间')
        self.aisle_map = None
        self.rejoin_attempts = {}
        self.failed_placement = None
        self.claimed = set()
        self.completed = False
        self.recovery_at = dict.fromkeys(CARS,float('-inf'))
        self.recovery_claims = set()
        self.command_timeout = float(settings.get('navigation_timeout_s',150.))
        self.charge_timeout = float(settings.get('charge_timeout_s',14400.))

    def parallel(self, cars, fn):
        futures = {car:self.pool.submit(fn,car) for car in cars}
        wait(futures.values())
        return {car:job.result() for car,job in futures.items()}

    def sync(self, *, renew=(), epoch=None, check_operator=True):
        self.check_pause()
        def sample(car):
            if car in renew:
                try:
                    try:
                        status=self.command(car,'fas_renew',epoch=epoch,timeout=.40)
                    except OSError:
                        status=self.command(car,'fas_renew',epoch=epoch,timeout=.40)
                except ValueError as error:
                    if '行驶许可已过期' not in str(error):
                        raise
                    return {**self.car_state(car),'motion_lease_expired':True}
                except OSError as exc:
                    raise OSError(f'{car} 车行驶许可续期失败：{exc}') from exc
                if 'health' in status:
                    return status
            try:
                return self.car_state(car)
            except OSError as exc:
                raise OSError(f'{car} 车状态读取失败：{exc}') from exc
        self.states = self.parallel(CARS,sample)
        for car,state in self.states.items():
            health = state['health']
            fas = state.get('fas') or {}
            expected_links = {band:values[car] for band,values in self.links.items()}
            if (state['map']['map_id'] != self.stations.map_id or fas.get('car') != car
                    or fas.get('protocol') != PROTOCOL or not health['dock_calibrated']
                    or fas.get('motion_only',False)!=self.motion_only or fas['tx']!=expected_links):
                raise ValueError(f'{car} 车的试验模式、地图、桩位或发端身份未对齐')
            resume=self.resume_voltage_by_car.get(car)
            low=self.return_voltage_by_car.get(car)
            near=self.near_return_voltage_by_car.get(car)
            if resume is not None or low is not None or near is not None or self.resume_min_charge_seconds is not None:
                energy=fas['energy']
                resume=energy['resume_voltage'] if resume is None else resume
                low=energy['low_voltage'] if low is None else low
                if not energy['low_voltage'] <= low < resume <= energy['resume_voltage']:
                    raise ValueError(f'{car} 车实验回充与续采电压须在车端允许范围内')
                if near is not None and not energy['low_voltage']+.15<=near<=low:
                    raise ValueError(f'{car} 车短返程阈值须留出欠压余量且不超过普通返程阈值')
                state['fas']={**fas,'energy':{**energy,'low_voltage':low,'resume_voltage':resume}}
                if self.resume_min_charge_seconds is not None:
                    state['fas']['energy']['min_charge_seconds']=self.resume_min_charge_seconds
            owner=fas.get('owner')
            if owner not in (None,self.journal.run_id):
                raise ValueError(f'{car} 车已交给另一任务')
            if check_operator and owner==self.journal.run_id and fas.get('operator_hold'):
                raise ValueError(f'{car} 车已手动停车')
            if car in self.charge_plateaus:
                window=self.charge_plateaus[car]
                floor=state['fas']['energy']['resume_voltage']
                target=fas['energy']['resume_voltage']
                if window.near_full is not None and not floor<=window.near_full<=target:
                    raise ValueError(f'{car} 车接近充足电压须介于续采下限和车端设置之间')
                previous_bulk=window.bulk_charge_start
                window.observe(time.monotonic(),health,floor,target,wall_stamp=time.time())
                if window.bulk_charge_start!=previous_bulk:
                    self.journal.set_bulk_charge_start(car,window.bulk_charge_start)
        return self.states

    def check_pause(self):
        if self.pause_file is not None and self.pause_file.exists():
            raise PauseRequested
        if (getattr(self,'auto_charging',False) and self.pause_file and self.pause_file.with_suffix('.no-auto-charge').exists()):
            self.pause_file.touch()
            raise PauseRequested
        if (self.pause_file and getattr(self,'manual_controls_ready',False) and not self.manual_returning
                and self.pause_file.with_suffix('.return-now').exists()):
            raise ReturnRequested

    def wait_retry(self, delay):
        deadline=time.monotonic()+delay
        while time.monotonic()<deadline:
            self.check_pause()
            time.sleep(min(.25,max(0.,deadline-time.monotonic())))
        self.check_pause()

    def car_state(self, car):
        if self.compact_status[car]:
            state=self.cars[car].fetch('/api/fas/state',missing=True)
            if state is not None:
                return state
            self.compact_status[car]=False
        return self.cars[car].fetch('/api/state')

    def command(self, car, action, *, timeout=.85, **payload):
        state = self.states[car]
        request = {'action':action,'run_id':self.journal.run_id,'map_id':self.stations.map_id,**payload}
        return self.cars[car].fetch('/api/command',request,{'X-Control-Token':state['control_token']},timeout)

    def parked_command(self, car, action, *, timeout=3., **payload):
        deadline=time.monotonic()+12.
        while True:
            try:
                return self.command(car,action,timeout=timeout,**payload)
            except ValueError as exc:
                if str(exc)!=NOT_PARKED:
                    raise
                if time.monotonic()>=deadline:
                    raise RecoveryWait(f'{car} 车等待停稳后继续当前步骤') from exc
            self.wait_retry(.25)

    def start(self, preflight_state, c5_codes, min_samples, window_s, resume_operator, *,
              c5_window_s=None, parallel_radios=False):
        self.window_s = window_s
        self.sync(check_operator=not resume_operator)
        held=tuple(car for car,s in self.states.items() if s['fas']['operator_hold'])
        if not self.motion_only:
            base = self.base.get('/fas/health')
            if not base.get('ready') and (base.get('error') or '').startswith(SERIAL_CLOSE_FAULT):
                raise RecoveryWait(base['error'])
            if (base.get('protocol') != PROTOCOL or base.get('ready') is not True or base.get('tx_configured') != self.links
                    or not set(self.receivers) <= set(base.get('receivers', []))):
                raise ValueError('基站协议、接收通道或两车发端仍未就绪')
            if c5_window_s is not None and base.get('separate_c5_window') is not True:
                raise ValueError('基站版本尚不支持独立 C5 采样窗口')
            if parallel_radios and base.get('parallel_radios') is not True:
                raise ValueError('基站版本尚不支持双频并行采样')
            self.base.receipt_wait = base.get('receipt_wait') is True
        for car in CARS:
            config = self.states[car]['fas']['energy']
            if not config.get('enabled') or not config.get('resume_voltage'):
                raise ValueError(f'{car} 车需要有效的低电回充与续作阈值')
            self.claimed.add(car)
            deadline = time.monotonic()+15.
            while True:
                try:
                    self.command(car,'fas_claim',timeout=3.)
                    break
                except ValueError as exc:
                    if str(exc)!=NOT_PARKED:
                        raise
                    if time.monotonic()>=deadline:
                        raise RecoveryWait(str(exc)) from exc
                    self.wait_retry(.3)
            # An earlier process may have ended mid-motion. Revoke old grants first.
            self.command(car,'fas_stop',timeout=3.)
        pending = self.journal.pending()
        if pending and not self.motion_only:
            request = json.loads(pending['request'])
            self.base.cancel(request)
            self.journal.abandon(pending,'重启后关闭旧窗口，保留原始数据并重新测量')
        if held and resume_operator:
            self.parallel(held,lambda car:self.command(car,'fas_resume'))
        self.wait_parked()
        if self.motion_only:
            print('两车停稳预检通过，开始位置对通行验证。',flush=True)
            return
        self.parallel(CARS,lambda car:self.parked_command(car,'fas_probe'))
        test_poses = self.actual_poses(('preflight-A','preflight-B'))
        check = capture_request(self.journal.run_id,uuid.uuid4().hex,preflight_state,test_poses,
                                phase_state(preflight_state,c5_codes),self.links,
                                min_samples=min_samples,window_s=window_s,c5_window_s=c5_window_s,
                                parallel_radios=parallel_radios)
        check['purpose'] = 'preflight'
        receipt=self.capture(check)
        if (receipt.get('error') or '').startswith(SERIAL_CLOSE_FAULT):
            raise RecoveryWait(receipt['error'])
        evaluate(check,receipt,self.receivers)
        print('两车静止预检通过，基站已确认各指定通道分别收到 A、B 帧。',flush=True)

    def wait_parked(self, *, wait_for_dock_fit=False):
        started = time.monotonic()
        deadline = started+(30. if self.motion_only else 15.)
        while time.monotonic() < deadline:
            try:
                self.sync()
            except OSError:
                time.sleep(.3)
                continue
            if all(s['phase'] in PARKED_PHASES and s['health']['stationary'] and not s['fas']['lease_live']
                   for s in self.states.values()):
                fitting=any(s['health'].get('charging_stable')
                            and s['health'].get('dock_localization',{}).get('status')=='applying'
                            for s in self.states.values())
                if all(fleet_pose_ready(s['health']) for s in self.states.values()) and not (wait_for_dock_fit and fitting):
                    return
                if fitting:
                    # The stationary dock fit and subsequent AMCL check can outlast a normal stop.
                    deadline = max(deadline,started+85.)
                if self.motion_only:
                    self.relocalize_parked()
            time.sleep(.3)
        raise RecoveryWait('等待两车停稳或雷达定位复核')

    def relocalize_parked(self):
        now=time.monotonic()
        for car,s in self.states.items():
            f=s['fas']
            if (fleet_pose_ready(s['health']) or f.get('owner')!=self.journal.run_id
                    or f.get('recovery') not in ('available','waiting') or now-self.recovery_at[car]<12.):
                continue
            self.check_pause()
            self.recovery_at[car]=now
            try:
                self.command(car,'fas_relocalize',timeout=2.)
            except (OSError,ValueError):
                continue  # Parked, fresh sensing and the current claim are checked again on the car.
            self.journal.set_activity('recovering',f'{car} 车正在用静止雷达重新对齐地图')

    def recover_capture_pose(self):
        # Only the regular stationary calibration endpoint is available on the
        # deployed capture firmware. Close the radio window before its use.
        if self.journal.pending():
            return
        self.sync()
        if any(s['phase'] not in PARKED_PHASES or not s['health']['stationary'] or s['fas']['lease_live']
               for s in self.states.values()):
            return
        for car,state in self.states.items():
            health=state['health']
            if '停车锁定' in (health.get('readiness') or ''):
                raise ValueError(f'{car} 车底盘停车锁定，须核对实车和桩位')
            if fleet_pose_ready(health):
                if car in self.recovery_claims:
                    self.command(car,'fas_claim',timeout=3.)
                    self.recovery_claims.discard(car)
                continue
            now=time.monotonic()
            pose=health.get('pose')
            age=health.get('pose_age_s')
            if (not pose or age is None or not 0.<=age<=.45 or health.get('motion_fault')
                    or not health.get('map_matches') or health.get('charging_stable')
                    or state['fas'].get('dock_retry_pending') or state['fas'].get('recovery')!='available'
                    or (health.get('scan_map_agreement') or 0.)<.55 or now-self.recovery_at[car]<20.):
                continue
            self.recovery_at[car]=now
            self.recovery_claims.add(car)
            if state['fas']['owner']==self.journal.run_id:
                self.command(car,'fas_release',timeout=3.)
            state=self.states[car]=self.car_state(car)
            if state['fas']['owner'] is not None or not state['health']['stationary']:
                raise RecoveryWait(f'{car} 车暂不能独占静止定位复核')
            self.journal.set_activity('recovering',f'{car} 车停稳，正在自动雷达精校')
            endpoint=self.cars[car]
            headers={'X-Control-Token':state['control_token']}
            try:
                endpoint.fetch('/api/command',{'action':'align_pose'},headers,timeout=16.)
                self.verify_capture_alignment(car,pose)
            except ValueError as error:
                raise RecoveryWait(f'{car} 车静止雷达复核未通过：{error}') from error
            self.check_pause()
            self.command(car,'fas_claim',timeout=3.)
            self.recovery_claims.discard(car)

    def verify_capture_alignment(self, car, previous_pose):
        deadline=time.monotonic()+8.
        stable_since=None
        while time.monotonic()<deadline:
            state=self.states[car]=self.car_state(car)
            health=state['health']
            pose=health.get('pose')
            if state['fas']['owner'] is not None or not health['stationary']:
                raise ValueError('车辆控制权或静止状态已变化')
            if (fleet_pose_ready({**health,'localization_confirmed':True})
                    and math.dist(pose[:2],previous_pose[:2])<=.18
                    and yaw_error(pose[2],previous_pose[2])<=math.radians(10)):
                if stable_since is None:
                    stable_since=time.monotonic()
                if time.monotonic()-stable_since>=.6:
                    self.cars[car].fetch('/api/command',{'action':'confirm'},
                                         {'X-Control-Token':state['control_token']},timeout=3.)
                    return
            else:
                stable_since=None
            time.sleep(.25)
        raise ValueError('校准偏移或连续雷达证据未达标')

    def off(self):
        if self.motion_only:
            return
        def disable(car):
            try:
                self.command(car,'fas_radio',enabled=False,timeout=3.)
            except (OSError, ValueError, KeyError):
                # WS63 also expires its own transmit lease.
                pass
        self.parallel(tuple(self.claimed),disable)

    def stop(self, cars=None):
        def parked(car):
            try:
                self.command(car,'fas_stop')
            except (OSError, ValueError, KeyError):
                pass  # No further heartbeat. Onboard leases expire independently.
        self.parallel(tuple(self.claimed) if cars is None else cars, parked)

    def close(self):
        try:
            self.stop()
            self.off()
            if self.pause_file is not None:
                self.release_available()
            elif self.completed:
                self.wait_parked()
                for car in self.claimed:
                    self.command(car,'fas_release',timeout=3.)
        finally:
            self.pool.shutdown()

    def release_available(self):
        released, pending = [], []
        for car in CARS:
            deadline = time.monotonic()+8.
            stop_requested = False
            while time.monotonic()<deadline:
                try:
                    state = self.states[car] = self.car_state(car)
                    fas = state['fas']
                    if fas.get('owner') != self.journal.run_id:
                        if car in self.claimed:
                            self.claimed.discard(car)
                            if fas.get('owner') is None:
                                released.append(car)
                        break
                    self.claimed.add(car)
                    if state['health']['stationary'] and state['phase'] in PARKED_PHASES and not fas['lease_live']:
                        self.command(car,'fas_release',timeout=3.)
                        self.claimed.discard(car)
                        released.append(car)
                        break
                    if not stop_requested:
                        self.command(car,'fas_stop',timeout=3.)
                        stop_requested = True
                except (OSError,ValueError,KeyError):
                    # A lost reply may still have stopped or released the car.
                    pass
                time.sleep(.3)
            else:
                pending.append(car)
        print(f'已释放 {released}。待车辆连通并停稳后核对 {pending}。',flush=True)
        return 1 if pending else 0

    def positions(self):
        self.wait_parked()
        points = tuple(tuple(self.states[car]['health']['pose']) for car in CARS)
        if points != self.anchor:
            if not self.anchor or any(math.dist(a[:2],b[:2])>.025 for a,b in zip(points,self.anchor)):
                self.cache.clear()
            self.anchor = points
        return tuple(min(self.stations.points,key=lambda p:math.dist(point_xy(self.stations.points[p]),pose[:2]))
                     for pose in points)

    def low_cars(self):
        now = time.monotonic()
        ready = []
        for car,state in self.states.items():
            h, config = state['health'], state['fas']['energy']
            start_voltage=self.initial_departure.get(car)
            if start_voltage is not None and not h.get('charging_stable'):
                self.initial_departure.pop(car,None)
                start_voltage=None
            near=self.near_return_voltage_by_car.get(car)
            threshold=near if near is not None and self.short_return_available(car,now) else config['low_voltage']
            voltage=h['voltage']
            low = voltage is not None and voltage < threshold and not h.get('charging_stable')
            self.started_low[car] = (self.started_low[car] or now) if low else None
            critical=near is not None and low and voltage<near-.10
            if critical:
                if self.critical_low_since[car] is None:
                    self.critical_low_since[car]=now
            else:
                self.critical_low_since[car]=None
            if (start_voltage is not None and h['voltage'] is not None and h['voltage']>=start_voltage
                    and h.get('dock_localization',{}).get('status')=='confirmed'):
                continue
            # Contact opens before the car clears the dock. The joint charging decision
            # has already been made when the scheduler grants this departure.
            if (state.get('phase')=='departing' and state['fas'].get('lease_live')
                    and voltage is not None and voltage>=threshold):
                continue
            plateau = getattr(self,'charge_plateaus',{}).get(car)
            needs_dock_charge=bool(plateau and plateau.near_full is not None and not plateau.ready
                and (voltage is None or voltage<plateau.near_full) and h['stationary'] and h.get('pose')
                and math.dist(h['pose'][:2],state['dock'][:2])<=.14
                and yaw_error(h['pose'][2],state['dock'][2])<=math.radians(12))
            if (not self.motion_only and h.get('charging_stable') and
                    ((plateau is not None and not plateau.ready)
                     or (h['voltage'] is not None and h['voltage'] < config['resume_voltage']))):
                ready.append(car)
            elif needs_dock_charge:
                ready.append(car)
            elif low and (now-self.started_low[car] >= config['low_hold_seconds'] or
                          (critical and now-self.critical_low_since[car]>=3.)):
                ready.append(car)
        return ready

    def check_early_launch(self):
        floor=self.early_launch_voltage
        for car,state in self.states.items():
            health=state['health']
            energy=state['fas']['energy']
            voltage=health.get('voltage')
            if not energy['low_voltage']+.70<=floor<=energy['resume_voltage']:
                raise ValueError(f'{car} 车提前离桩阈值须保留至少 0.70 V 返程余量')
            if (voltage is None or not math.isfinite(voltage) or voltage<floor or not health['stationary']
                    or not health['charging_stable'] or not fleet_pose_ready(health)
                    or health.get('dock_localization',{}).get('status')!='confirmed'):
                raise ValueError(f'{car} 车未达到本次提前离桩条件')
        self.initial_departure=dict.fromkeys(CARS,floor)

    def short_return_available(self, car, now):
        checked=self.short_return_checks.get(car)
        if checked is None:
            return False
        stamp, previous, clear=checked
        return (clear and now-stamp<=SHORT_RETURN_RECHECK and all(
            state['health']['stationary'] and state['health']['pose'] is not None
            and math.dist(state['health']['pose'][:2],previous[key])<=.05 for key,state in self.states.items()))

    def check_short_return_routes(self):
        now=time.monotonic()
        for car in self.near_return_voltage_by_car:
            peer='B' if car=='A' else 'A'
            state=self.states[car]
            health=state['health']
            voltage=health['voltage']
            if (voltage is None or voltage>state['fas']['energy']['low_voltage']+.06
                    or voltage<=self.near_return_voltage_by_car[car]-.10 or health.get('charging_stable')
                    or not fleet_pose_ready(health) or not all(s['health']['stationary'] and s['health']['pose']
                                                                  for s in self.states.values())):
                continue
            starts={key:s['health']['pose'][:2] for key,s in self.states.items()}
            home=state['dock'][:2]
            if math.dist(starts[car],home)>SHORT_RETURN_DISTANCE:
                continue
            previous=self.short_return_checks.get(car)
            if (previous and now-previous[0]<=SHORT_RETURN_RECHECK and
                    all(math.dist(starts[key],pose)<=.05 for key,pose in previous[1].items())):
                continue
            self.short_return_checks[car]=(now,starts,False)
            try:
                path=self.command(car,'fas_preview',kind='return',timeout=2.)['navigation_path']
                clearance=peer_clearance(*self.position_sigmas().values())
            except (OSError,ValueError,RouteRetry):
                continue  # Unknown geometry keeps the ordinary, higher threshold.
            if (path and math.dist(path[0][:2],starts[car])<=.20 and math.dist(path[-1][:2],home)<=.95
                    and math.dist(starts[car],path[0][:2])+math.dist(path[-1][:2],home)
                    +sum(math.dist(a[:2],b[:2]) for a,b in zip(path,path[1:]))<=SHORT_RETURN_DISTANCE
                    and clear_of_parked_peer([starts[car],*path,home],starts[peer],clearance)):
                self.short_return_checks[car]=(time.monotonic(),starts,True)

    def aisle_clearance(self):
        margin = getattr(self, 'aisle_margin', None)
        if margin is not None and self.aisle_map is None:
            state = self.cars['A'].fetch('/api/state', timeout=4.)
            metadata = state['map']
            if metadata['map_id'] != self.stations.map_id:
                raise ValueError('净空地图与采集点位不匹配')
            with urlopen(self.cars['A'].url+'/map.png?map_id='+self.stations.map_id, timeout=4.) as response:
                self.aisle_map = AisleClearance(metadata, response.read(), margin)
        return getattr(self, 'aisle_map', None)

    def preview(self, car, point=None, *, rejoin=False):
        key = car, None if point is None else (point['id'], rejoin)
        if key not in self.cache:
            response = self.parked_command(car, 'fas_preview', kind='return' if point is None else 'move',
                                           point=point, timeout=7.)
            path = response['navigation_path']
            aisle = self.aisle_clearance() if point is not None else None
            if aisle:
                state = self.states[car]
                start = state['health']['pose']
                departing = not rejoin and math.dist(start[:2],state['dock'][:2]) <= .55
                if not aisle.allows([start, *path], rejoin=rejoin, departing=departing):
                    raise ValueError('当前没有可走的路线，原生预览净空不足')
            self.cache[key] = path
        return self.cache[key]

    def rejoin_corridors(self):
        aisle = self.aisle_clearance()
        if aisle is None:
            return
        for car in CARS:
            state = self.states[car]
            pose = state['health']['pose']
            if state['health']['charging_stable'] or math.dist(pose[:2],state['dock'][:2]) <= .55:
                continue
            now = time.monotonic()
            previous = self.rejoin_attempts.get(car)
            same = previous and math.dist(pose[:2],previous[0]) <= .10
            if same and now-previous[1] < 8.:
                continue
            start_margin = aisle.distance(pose)
            if start_margin >= aisle.margin:
                continue
            attempted = {point:stamp for point,stamp in previous[2].items() if now-stamp < 60.} if same else {}
            self.rejoin_attempts[car] = (pose[:2],now,attempted)
            other = 'B' if car == 'A' else 'A'
            peer_pose = self.states[other]['health']['pose']
            clearance = peer_clearance(*self.position_sigmas().values())
            choices = []
            for point in self.stations.points.values():
                target = point_xy(point)
                distance = math.dist(pose[:2], target)
                if point['id'] in attempted or not .15 <= distance <= .90 or aisle.distance(target) < aisle.margin:
                    continue
                bearing = math.atan2(target[1]-pose[1],target[0]-pose[0])
                choices.append((yaw_error(bearing,pose[2])+.5*distance,point['id'],point))
            for _, _, point in sorted(choices)[:4]:
                bearing = math.atan2(point['y']-pose[1],point['x']-pose[0])
                approach = min((bearing,bearing+math.pi),key=lambda yaw:yaw_error(yaw,pose[2]))
                waypoint = {**point,'id':f'rejoin-{point["id"]}','yaw':math.remainder(approach,2*math.pi),'wait':0.}
                try:
                    path = self.preview(car,waypoint,rejoin=True)
                except ValueError as exc:
                    if str(exc) != NOT_PARKED and '没有可走' not in str(exc) and '预览超时' not in str(exc):
                        raise
                    continue
                if not clear_of_parked_peer(path,peer_pose,clearance):
                    continue
                message=f'{car} 车先驶回走廊中线，途经 {point["name"]}'
                self.journal.set_activity('recovering',message)
                print(message,flush=True)
                try:
                    self.drive_batch((car,),{car:path},{car:waypoint},{car:waypoint['yaw']})
                except VehicleLinkInterrupted:
                    raise
                except RouteRetry as exc:
                    attempted[point['id']] = time.monotonic()
                    print(f'{car} 车停稳后重新核对接入。{exc}',flush=True)
                self.wait_parked()
                break

    def position_sigmas(self, *, allow_holding=False):
        sigmas = {car:position_sigma(state['health']) for car,state in self.states.items()}
        for car,sigma in sigmas.items():
            holding = allow_holding and self.states[car]['fas'].get('scan_hold') is True
            if sigma is None or (sigma > MAX_FLEET_SIGMA and not holding):
                raise RouteRetry(f'{car} 车定位不确定性超过走廊预约上限')
        return sigmas

    def drive_batch(self, cars, paths, endpoints, end_yaws, *, returning=False, retreating=False, charge_waiting=None):
        other = set(CARS)-set(cars)
        parked = {car:tuple(self.states[car]['health']['pose']) for car in other}
        epoch = uuid.uuid4().hex
        kind = 'retreat' if retreating else 'return' if returning else 'move'
        peer = next(iter(parked), None)
        parked_peer = list(parked[peer][:2]) if peer else None
        last_poses = {car:s['health']['pose'] for car,s in self.states.items()}
        aisle = self.aisle_clearance() if not retreating else None
        margin_floors = {}
        if aisle:
            reference_margin = .12 if returning else aisle.margin-.04
            margin_floors = {car:min(reference_margin,aisle.distance(last_poses[car]))-.015 for car in cars}
        docked_since = None
        return_progress = None
        try:
            self.check_pause()
            bounds = ({cars[0]:sample_box(paths[cars[0]],pad=EXCLUSIVE_LOCALIZATION_PAD)}
                      if peer else parallel_bounds(paths))
            if bounds is None:
                raise RouteRetry('两车预约走廊相交，需重新调度')
            sigmas = self.position_sigmas()
            clearance = peer_clearance(*sigmas.values())
            if parked_peer and not all(clear_of_parked_peer(paths[car],parked_peer,clearance) for car in cars):
                raise RouteRetry('更新后的路线与停车车辆间距不足')
            try:
                self.parallel(cars,lambda car:self.command(car,'fas_reserve',epoch=epoch,kind=kind,
                    bounds=bounds[car],
                    parked_peer=parked_peer,
                    parked_peer_sigma=sigmas[peer] if peer else None,timeout=3.))
                self.check_pause()
                self.parallel(cars,lambda car:self.command(car,'fas_drive',epoch=epoch,kind=kind,point=endpoints[car]))
            except ValueError as exc:
                if str(exc)!=NOT_PARKED and not any(
                    reason in str(exc) for reason in ('车辆预约未通过','行驶许可无效','短退需要','短退的走廊','没有可自动短退')
                ):
                    raise
                raise RouteRetry(str(exc)) from exc
            if self.motion_only and kind=='move':
                targets = '、'.join(f'{car}→{endpoints[car]["name"]}' for car in cars)
                print(f'{"双车并行" if not peer else "分段通行"} {targets}',flush=True)
            timeout = 14. if retreating else self.command_timeout
            if returning:
                length = sum(math.dist(a[:2],b[:2]) for a,b in zip(paths[cars[0]],paths[cars[0]][1:]))
                timeout = max(200.,timeout,length/.055+75.)
            deadline = time.monotonic()+timeout
            observation_only=set()
            while time.monotonic() < deadline:
                polled_at = time.monotonic()
                try:
                    states = self.sync(renew=tuple(car for car in cars if car not in observation_only),epoch=epoch)
                except ValueError as exc:
                    if '失效' in str(exc) or '许可' in str(exc):
                        raise RouteRetry(str(exc)) from exc
                    raise
                poses = {}
                for car,state in states.items():
                    h = state['health']
                    if state['fas']['operator_hold']:
                        raise ValueError('操作员已停车')
                    verifying_dock=(returning and car in cars and h['stationary'] and
                        (state['phase']=='docked' or state['fas']['dock_retry_pending']))
                    if state.get('motion_lease_expired') or car in observation_only:
                        if (kind=='move' and stopped_station_arrival(h)) or (verifying_dock and h['charging_stable']):
                            observation_only.add(car)
                        else:
                            raise RouteRetry(f'{car} 车行驶许可已过期，停稳后重新预约')
                    if not verifying_dock and (not h['ready'] or not h['localization_confirmed'] or not h.get('pose')):
                        raise RouteRetry(f'{car} 车定位暂不可用，已收回两车运动许可')
                    poses[car] = h.get('pose') or last_poses[car]
                    if car in parked and (not h['stationary'] or math.dist(parked[car][:2],poses[car][:2])>.075):
                        raise RouteRetry(f'{car} 车的停车位置变化，已收回运动许可')
                    near_dock = returning and math.dist(poses[car][:2],state['dock'][:2]) <= .90
                    if car in margin_floors and not near_dock and aisle.distance(poses[car]) < margin_floors[car]:
                        raise RouteRetry(f'{car} 车实际路线向障碍边缘偏离，已停车')
                clearance = peer_clearance(*self.position_sigmas(allow_holding=True).values(),moving_together=not parked)
                if math.dist(poses['A'][:2], poses['B'][:2]) < clearance:
                    raise RouteRetry('两车安全间距缩小，已收回运动许可')
                last_poses = poses
                if retreating:
                    car=cars[0]
                    health=states[car]['health']
                    arrival=health.get('arrival') or {}
                    if health['stationary'] and arrival.get('kind')=='retreat' and arrival.get('status')=='succeeded':
                        if (not fleet_pose_ready(health)
                                or math.dist(poses[car][:2],paths[car][-1][:2])>.07):
                            raise RouteRetry('后退结束后未复核到有效退出位移')
                        break
                if any(states[car]['phase'] in ('failed','idle') and
                       not (returning and states[car]['fas']['dock_retry_pending']) for car in cars):
                    detail='；'.join(f'{car} 车：{states[car]["message"]}' for car in cars)
                    raise RouteRetry(f'本次路线需要停稳后重新规划。{detail}')
                if returning:
                    car=cars[0]
                    dock_state=states[car]
                    health=dock_state['health']
                    arrival=health.get('arrival') or {}
                    outside_dock = math.dist(poses[car][:2],dock_state['dock'][:2]) > .90
                    if (outside_dock and health['stationary'] and arrival.get('status')=='recovery_wait'
                            and arrival.get('retry_streak',0)>=3):
                        raise RouteRetry('返程重规划没有进展，停稳复核走廊入口')
                    if outside_dock and arrival.get('status')=='navigating':
                        position = poses[car][:2]
                        if not return_progress or math.dist(position,return_progress[0])>=.085:
                            return_progress = position,polled_at
                        elif polled_at-return_progress[1]>=20.:
                            raise RouteRetry('返程连续 20 秒无有效位移，停稳后从实测位置重新规划')
                    else:
                        return_progress = None
                    stable=dock_state['phase']=='docked' and health['stationary'] and health['charging_stable']
                    docked_since=(docked_since if docked_since is not None else polled_at) if stable else None
                    if (docked_since is not None and polled_at-docked_since>=DOCK_SETTLE_SECONDS
                            and health['localization_confirmed'] and health.get('dock_localization',{}).get('status')=='confirmed'):
                        break
                elif not retreating:
                    low = [car for car in self.low_cars() if car!=charge_waiting]
                    if low:
                        raise ChargeNeeded(low)
                    if any(states[car]['health']['stationary'] and states[car]['fas'].get('route_start_blocked')
                           for car in cars):
                        raise RouteRetry('原生控制器在同一起点反复拒绝行驶')
                    if all(states[car]['health']['stationary'] and states[car]['phase']=='completed' for car in cars):
                        if any(math.dist(poses[car][:2],point_xy(endpoints[car]))>FAS_STATION_POSITION_TOLERANCE for car in cars):
                            raise RouteRetry('有车停稳后的位置需重校')
                        if any(yaw_error(poses[car][2],end_yaws[car])>FAS_STATION_YAW_TOLERANCE for car in cars):
                            raise StationHeadingRetry('位置已达标，保留原点做静止调向')
                        break
                time.sleep(max(0.,.25-(time.monotonic()-polled_at)))
            else:
                raise RouteRetry('本轮导航超时')
        except OSError as exc:
            raise VehicleLinkInterrupted(f'车辆通信暂时中断，停车后重新规划。{exc}') from exc
        finally:
            # A successful native dock is left in its actual stopped/charging state.
            if not returning or any(self.states[car]['phase']!='docked' for car in cars):
                self.stop(cars)
            self.cache.clear()

    def place(self, pair):
        exclusive=False
        guarded = getattr(self, 'aisle_margin', None) is not None
        anchor = tuple(self.states[car]['health']['pose'][:2] for car in CARS) if guarded else ()
        for attempt in range(3):
            try:
                poses = self.place_once(pair,exclusive=exclusive)
                if poses is not None:
                    self.failed_placement = None
                if poses is not None and self.motion_only:
                    return self.simulate_window(pair,poses)
                if poses is not None:
                    return poses
                break
            except (RouteRetry,OSError) as exc:
                if not self.motion_only and isinstance(exc,(VehicleLinkInterrupted,OSError)):
                    raise
                print(f'本轮停稳后重试。{exc}',flush=True)
                self.wait_parked()
                if isinstance(exc,StationHeadingRetry):
                    self.cache.clear()
                    continue
                recovered=attempt<2 and self.recover_obstruction()
                self.cache.clear()
                if attempt>=1 and not recovered:
                    break
                exclusive=(not self.motion_only or any(
                    detail in str(exc) for detail in ('预约走廊','停车车辆的净空')
                ))
        if guarded:
            self.wait_parked()
            self.rejoin_corridors()
            low = self.low_cars()
            if low:
                raise ChargeNeeded(low)
            previous, failures = self.failed_placement or (anchor, 0)
            actual = tuple(self.states[car]['health']['pose'][:2] for car in CARS)
            same = all(math.dist(a,b) < .12 for a,b in zip(previous,actual))
            self.failed_placement = (previous, failures+1) if same else (actual, 1)
            if self.failed_placement[1] >= 3:
                raise RecoveryWait('原地换点未改善通行，只在停稳复核后重试入口')
        return None

    def recover_obstruction(self):
        recovered=False
        for car in CARS:
            if not self.states[car]['fas'].get('retreat_available'):
                continue
            self.check_pause()
            try:
                path=self.command(car,'fas_preview',kind='retreat',timeout=3.)['navigation_path']
            except ValueError:
                continue  # Only the car can authorize a fresh retreat candidate.
            peer='B' if car=='A' else 'A'
            clearance=peer_clearance(*self.position_sigmas().values())
            if not clear_of_parked_peer(path,self.states[peer]['health']['pose'],clearance):
                continue
            self.journal.set_activity('recovering',f'{car} 车复核车尾和另一车净空后短退')
            print(f'{car} 车已核对车尾与另一台车的净空，申请短退。',flush=True)
            try:
                self.drive_batch((car,),{car:path},{car:None},{},retreating=True)
                recovered=True
            except RouteRetry as error:
                print(f'短退未通过复核，保留空转区域。{error}',flush=True)
            self.wait_parked()
        return recovered

    def place_once(self, pair, *, exclusive=False):
        self.positions()
        if self.charge_plateaus and not self.dual_departure_pending and self.early_launch_voltage is None:
            waiting = self.journal.charging() or self.low_cars()
            if waiting:
                raise ChargeNeeded(waiting)
        starts = {car:self.states[car]['health']['pose'] for car in CARS}
        if self.stage_dock_departures(starts,exclusive=exclusive):
            self.positions()
            starts = {car:self.states[car]['health']['pose'] for car in CARS}
        target = dict(zip(CARS, (self.stations.points[p] for p in pair)))
        if getattr(self, 'aisle_margin', None) is not None:
            destinations='、'.join(f'{car} 车 {target[car]["name"]}' for car in CARS)
            self.journal.set_activity('moving',f'前往位置对 {destinations}')
        paths = self.station_paths(starts,target)
        if paths is None or not self.dispatch_placement(starts,target,paths,exclusive=exclusive):
            return None
        poses = self.measured(pair)
        if getattr(self, 'aisle_margin', None) is not None and not self.motion_only:
            self.journal.set_activity('sampling','两车实测位姿和朝向已确认，开始双频采样')
        return poses

    def stage_dock_departures(self, starts, *, exclusive):
        aisle = self.aisle_clearance()
        if aisle is None:
            return False
        targets, paths = {}, {}
        for car in CARS:
            if not self.states[car]['health']['charging_stable']:
                continue
            start = starts[car]
            choices = []
            for point in self.stations.points.values():
                distance = math.dist(start[:2],point_xy(point))
                if not .32 <= distance <= 1.15 or aisle.distance(point_xy(point)) < aisle.margin:
                    continue
                bearing = math.atan2(point['y']-start[1],point['x']-start[0])
                turn = min(yaw_error(body,start[2])+yaw_error(point['yaw'],body)
                           for body in (bearing,bearing+math.pi))
                choices.append((distance+.15*turn,point['id'],point))
            for _, _, point in sorted(choices):
                try:
                    paths[car] = self.preview(car,point)
                except ValueError as exc:
                    if '没有可走' not in str(exc) and '预览超时' not in str(exc):
                        raise
                    continue
                targets[car] = point
                break
        if not targets:
            return False
        message='离桩后先进入各自走廊中线，再前往采集点'
        self.journal.set_activity('running',message)
        print(message,flush=True)
        return self.dispatch_placement(starts,targets,paths,exclusive=exclusive)

    def dispatch_placement(self, starts, target, paths, *, exclusive):
        ends = {car:point_xy(point) for car,point in target.items()}
        yaws = {car:point['yaw'] for car,point in target.items()}
        clearance = peer_clearance(*self.position_sigmas().values())
        batches = move_batches(starts,ends,paths,exclusive=exclusive,clearance=clearance)
        if batches is None:
            return False
        for index,batch in enumerate(batches):
            if index:
                for car in batch:
                    paths[car]=self.preview(car,target[car])
            if self.dual_departure_pending:
                self.sync()
                if self.dual_charge_waiting():
                    raise ChargeNeeded(CARS)
                self.journal.set_charging([])
                self.dual_departure_pending=False
            if self.early_launch_voltage is not None:
                self.sync()
                self.check_early_launch()
                self.journal.set_charging([])
                self.early_launch_voltage=None
            self.drive_batch(batch,paths,target,yaws)
            self.wait_parked()
        return True

    def station_paths(self, starts, target):
        def path_to(car):
            point = target[car]
            if self.at_station(starts[car],point):
                return []
            try:
                return self.preview(car,point)
            except ValueError as exc:
                if str(exc)==NOT_PARKED:
                    raise RouteRetry(str(exc)) from exc
                if not any(reason in str(exc) for reason in ('没有可走','路径预览超时')):
                    raise
                return None

        paths = self.parallel(CARS,path_to)
        return paths if all(path is not None for path in paths.values()) else None

    @staticmethod
    def at_station(pose, point):
        return (math.dist(pose[:2],point_xy(point))<=FAS_STATION_POSITION_TOLERANCE
                and yaw_error(pose[2],point['yaw'])<=FAS_STATION_YAW_TOLERANCE)

    def order_motion_pairs(self, candidates, limit):
        starts = {car:self.states[car]['health']['pose'] for car in CARS}
        can_start_together = boxes_apart(*(sample_box([starts[car]]) for car in CARS))
        clearance = peer_clearance(*self.position_sigmas().values())
        separated, remaining = [], []
        checked = 0
        # Leave room for the accepted parking error when choosing a split pair.
        room = LOCALIZATION_PAD+FAS_STATION_POSITION_TOLERANCE
        for pair in candidates:
            target = {car:self.stations.points[p] for car,p in zip(CARS,pair)}
            ends = {car:point_xy(target[car]) for car in CARS}
            if boxes_apart(*(sample_box([ends[car]],pad=room) for car in CARS)):
                separated.append(pair)
            else:
                remaining.append(pair)
            if not can_start_together or checked>=limit:
                continue
            if (any(math.dist(starts[car][:2],ends[car])<=.045 for car in CARS)
                    or not boxes_apart(*(sample_box([starts[car],ends[car]]) for car in CARS))):
                continue
            checked += 1
            try:
                paths = self.station_paths(starts,target)
            except (OSError,RouteRetry):
                checked = limit
                continue
            if paths and move_batches(starts,ends,paths,clearance=clearance)==(CARS,):
                return [pair]+[candidate for candidate in candidates if candidate!=pair]
        if not can_start_together:
            def setup_cost(pair):
                moves=sum(not self.at_station(starts[car],self.stations.points[p]) for car,p in zip(CARS,pair))
                distance=sum(math.dist(starts[car][:2],point_xy(self.stations.points[p])) for car,p in zip(CARS,pair))
                return moves,distance
            separated.sort(key=setup_cost)
        return separated+remaining

    def simulate_window(self, pair, before):
        print(f'两车已到位，模拟采集 {self.window_s:g} 秒。',flush=True)
        deadline = time.monotonic()+self.window_s
        while True:
            for car,state in self.sync().items():
                health, fas = state['health'], state['fas']
                if fas['operator_hold']:
                    raise ValueError('操作员已停车')
                if (not health['ready'] or not health['localization_confirmed'] or not health['stationary']
                        or not health['pose'] or fas['lease_live']):
                    raise RouteRetry(f'{car} 车在模拟采集期间未确认停稳')
                first, actual = before[car]['pose'], health['pose']
                if math.dist(first[:2],actual[:2])>.025 or yaw_error(first[2],actual[2])>math.radians(2):
                    raise RouteRetry(f'{car} 车在模拟采集期间位姿变化')
            low = self.low_cars()
            if low:
                raise ChargeNeeded(low)
            remaining = deadline-time.monotonic()
            if remaining<=0.:
                return self.measured(pair)
            time.sleep(min(.25,remaining))

    def measured(self, pair):
        poses = self.actual_poses(pair)
        for car, identity in zip(CARS,pair):
            point = self.stations.points[identity]
            pose = poses[car]['pose']
            if (math.dist(pose[:2],point_xy(point))>FAS_STATION_POSITION_TOLERANCE
                    or yaw_error(pose[2],point['yaw'])>FAS_STATION_YAW_TOLERANCE):
                raise RouteRetry(f'{car} 车停靠位置或朝向不符合采样要求')
        return poses

    def sweep_pose(self, pair):
        poses=self.measured(pair)
        self.check_short_return_routes()
        low=self.low_cars()
        if low:
            raise ChargeNeeded(low)
        return poses

    def actual_poses(self, pair):
        self.wait_parked()
        poses = {}
        for car, identity in zip(CARS,pair):
            pose = self.states[car]['health']['pose']
            if self.states[car]['fas']['operator_hold'] or not pose:
                raise ValueError('车辆已停止实验，待重新确认')
            poses[car] = {'point_id':identity,'pose':pose,'frame':'map','map_id':self.stations.map_id}
        return poses

    def dual_charge_waiting(self):
        waiting=[]
        for car,plateau in self.charge_plateaus.items():
            state=self.states[car]
            elapsed=max(state['health'].get('charging_elapsed_s') or 0.,plateau.ready_elapsed)
            if not plateau.ready or elapsed<state['fas']['energy'].get('min_charge_seconds',0.):
                waiting.append(car)
        return waiting

    def service_energy(self):
        self.wait_parked()
        if self.early_launch_voltage is not None:
            self.check_early_launch()
            self.journal.set_activity('running','本次提前离桩，开始位置对调度')
            return
        for _ in range(3):
            if not self.recover_obstruction():
                break
        manual=getattr(self,'manual_returning',False)
        if not manual and not self.journal.charging():
            self.check_short_return_routes()
        todo = list(CARS) if manual else list(dict.fromkeys(self.journal.charging()+self.low_cars()))
        if not todo:
            self.rejoin_corridors()
        if todo and not manual and self.motion_only and self.pause_file and self.pause_file.with_suffix('.no-auto-charge').exists():
            self.pause_file.touch()
            raise PauseRequested
        self.auto_charging=bool(todo and not manual and self.motion_only and self.pause_file)
        try:
            if todo and self.charge_plateaus and not manual:
                self.initial_departure.clear()
                self.dual_departure_pending=False
                self.journal.set_charging(list(CARS))
                deadline=time.monotonic()+self.charge_timeout
                waiting=list(CARS)
                while waiting:
                    if time.monotonic()>=deadline:
                        raise RecoveryWait('等待两车同时满足充电和电压平台条件')
                    blocked=[]
                    failures=[]
                    for car in waiting:
                        try:
                            self.ensure_charging_dock(car)
                        except DockRouteConflict:
                            blocked.append(car)
                        except RecoveryWait as exc:
                            failures.append(exc)
                    for car in blocked:
                        try:
                            self.ensure_charging_dock(car,allow_yield=True)
                        except RecoveryWait as exc:
                            failures.append(exc)
                    if failures:
                        raise failures[0]
                    self.service_charge_queue(waiting,False,joint=True)
                    self.sync()
                    waiting=self.dual_charge_waiting()
                self.dual_departure_pending=True
                self.journal.set_activity('running','两车充电与桩位校准已通过，位置对调度中')
            else:
                self.service_charge_queue(todo,manual)
        finally:
            self.auto_charging=False

    def ensure_charging_dock(self, car, *, allow_yield=None):
        self.cache.clear()
        state = self.states[car]
        allow_yield=self.motion_only if allow_yield is None else allow_yield
        print(f'{car} 车开始回充检查。电压 {state["health"]["voltage"]} V。',flush=True)
        plateau=self.charge_plateaus.get(car)
        if not state['health'].get('charging_stable') and not (plateau and plateau.retained):
            peer = self.states['B' if car == 'A' else 'A']['health']
            if peer.get('charging_stable') and peer.get('dock_localization', {}).get('status') == 'applying':
                self.journal.set_activity('charging', '等已回桩车辆完成定位，再让另一车回充')
                self.wait_parked(wait_for_dock_fit=True)
                state = self.states[car]
                if state['health'].get('charging_stable'):
                    return
            self.journal.set_activity('charging',f'{car} 车回充中')
            yielded=False
            for _ in range(3):
                starts = {k:s['health']['pose'] for k,s in self.states.items()}
                try:
                    path = {car:self.preview(car)}
                except ValueError as exc:
                    if str(exc)!=NOT_PARKED and '没有可走' not in str(exc) and '路径预览超时' not in str(exc):
                        raise
                    raise RecoveryWait(f'{car} 车暂时没有可走的回充路线') from exc
                clearance = peer_clearance(*self.position_sigmas().values())
                if move_batches(starts,{car:state['dock']},path,exclusive=True,clearance=clearance) is None:
                    if yielded or not allow_yield or not self.yield_for_charge(car,path[car],clearance):
                        raise DockRouteConflict(f'{car} 车回充路线与另一车的停车位置冲突')
                    yielded=True
                    self.cache.clear()
                    state=self.states[car]
                    continue
                try:
                    self.drive_batch((car,),path,{car:None},{},returning=True)
                    break
                except RouteRetry as error:
                    print(f'{car} 车回充停稳后复核。{error}',flush=True)
                    self.wait_parked()
                    if self.states[car]['health'].get('charging_stable'):
                        break  # The contact was confirmed even though the previous reply was lost.
            else:
                raise RecoveryWait(f'{car} 车等待回充通道或触点恢复')

    def service_charge_queue(self,todo,manual,*,joint=False):
        if not manual and not joint:self.journal.set_charging(todo)
        while todo:
            car = todo[0]
            peer = 'B' if car == 'A' else 'A'
            self.ensure_charging_dock(car)
            config = self.states[car]['fas']['energy']
            deadline = time.monotonic()+self.charge_timeout
            stable_since = ready_since = None
            interrupted_since = {}
            plateau = getattr(self,'charge_plateaus',{}).get(car)
            detail = '电压平台和桩位校准' if plateau else '续作电压和桩位校准'
            self.journal.set_activity('charging',f'{car} 车在桩上等待{detail}')
            while time.monotonic()<deadline:
                try:
                    self.sync()
                except OSError:
                    stable_since=ready_since=None
                    interrupted_since.clear()
                    time.sleep(1.)
                    continue
                health = self.states[car]['health']
                if not self.states[peer]['health']['stationary']:
                    raise RecoveryWait('充电期间另一车状态变化，等待两车停稳后复核')
                now = time.monotonic()
                for docked_car in (CARS if joint else (car,)):
                    observed = self.states[docked_car]['health']
                    if ((observed.get('charging_stable') and observed['stationary'])
                            or (joint and self.charge_plateaus[docked_car].retained)):
                        interrupted_since.pop(docked_car, None)
                    elif now - interrupted_since.setdefault(docked_car, now) >= 12.:
                        raise RecoveryWait(f'{docked_car} 车充电中断，待复核后重新回充')
                if joint and plateau.retained and plateau.ready_elapsed>=config.get('min_charge_seconds',0.):
                    break
                stable = health.get('charging_stable') and health['stationary']
                dock_confirmed=(stable and health['localization_confirmed']
                                and health.get('dock_localization',{}).get('status')=='confirmed')
                if manual and dock_confirmed and (health.get('charging_elapsed_s') or 0)>=3.:
                    break
                stable_since = (stable_since or now) if stable else None
                onboard_duration=health.get('charging_elapsed_s')
                elapsed=(onboard_duration if stable and isinstance(onboard_duration,(int,float))
                         and math.isfinite(onboard_duration) and onboard_duration>=0.
                         else now-stable_since if stable_since is not None else 0.)
                confirmed = dock_confirmed and health['voltage'] is not None and health['voltage']>=config['resume_voltage']
                ready_since = (ready_since or now) if confirmed else None
                held=now-ready_since if ready_since is not None else 0.
                if joint and plateau.ready and plateau.samples:
                    held=max(held,plateau.samples[-1][0]-plateau.samples[0][0])
                if (confirmed and held>=config['charge_hold_seconds']
                        and elapsed>=config.get('min_charge_seconds',0.) and (not plateau or plateau.ready)):
                    break
                time.sleep(1.)
            else:
                raise RecoveryWait(f'{car} 车仍等待充电或桩位校准')
            todo.pop(0)
            if not manual and not joint:self.journal.set_charging(todo)
            self.started_low[car] = None
            self.critical_low_since[car]=None
            detail=f' {plateau.reason}。' if joint else ''
            print(f'{car} 车充电与桩位校准已通过。{detail}',flush=True)
        if not joint:self.journal.set_activity('running','位置对调度中')

    def yield_for_charge(self, car, returning_path, clearance):
        peer='B' if car=='A' else 'A'
        pose=self.states[peer]['health']['pose']
        waiting=self.states[car]['health']['pose']
        if self.states[peer]['health'].get('charging_stable') or peer in self.low_cars():
            return False
        options=sorted(self.stations.points.values(),key=lambda p:math.dist(pose[:2],point_xy(p)))
        options=[p for p in options if .30<=math.dist(pose[:2],point_xy(p))<=1.8
                 and clear_of_parked_peer(returning_path,point_xy(p),clearance+.10)]
        for point in options[:4]:
            try:
                path=self.preview(peer,point)
            except ValueError as exc:
                if str(exc)!=NOT_PARKED and not any(s in str(exc) for s in ('没有可走','路径预览超时')):
                    raise
                continue
            if not clear_of_parked_peer(path,waiting,clearance):
                continue
            self.journal.set_activity('charging',f'{peer} 车让出 {car} 车的回充路线')
            self.drive_batch((peer,),{peer:path},{peer:point},{peer:point['yaw']},charge_waiting=car)
            self.wait_parked()
            return True
        return False

    def reconcile(self, request):
        result = self.base.receipt(request['trial_id'])
        if result is not None and result.get('status') == 'complete' and result.get('after_pose_verified') is True:
            return result
        self.base.cancel(request)
        return None

    def capture(self, request):
        pair = tuple(request['cars'][car]['point_id'] for car in CARS)
        self.wait_parked()
        try:
            self.parallel(CARS,lambda car:self.parked_command(car,'fas_radio',enabled=True))
            now = time.monotonic()
            self.base.post('/fas/capture',request)
            deadline = now+max(24., request['window_s']*4)
            next_lease = now+2.
            while time.monotonic()<deadline:
                self.check_pause()
                polled_at = time.monotonic()
                receipt = self.base.receipt(request['trial_id'],wait=True)
                if receipt is not None and receipt.get('status') in ('complete','incomplete'):
                    if (receipt['status']=='incomplete' and (receipt.get('error') or '').startswith(SERIAL_CLOSE_FAULT)):
                        raise RecoveryWait(receipt['error'])
                    after = self.actual_poses(pair) if request.get('purpose')=='preflight' else self.measured(pair)
                    for car in CARS:
                        first, last = request['cars'][car]['pose'],after[car]['pose']
                        if math.dist(first[:2],last[:2])>.025 or yaw_error(first[2],last[2])>math.radians(2):
                            raise RouteRetry('采样窗口内车位变化，保留当前位置对并重采')
                    return {**receipt,'after_poses':after,'after_pose_verified':True}
                if time.monotonic()>=next_lease:
                    self.wait_parked()
                    if request.get('purpose')!='preflight':
                        low = self.low_cars()
                        if low:
                            raise ChargeNeeded(low)
                    self.parallel(CARS,lambda car:self.parked_command(car,'fas_radio',enabled=True))
                    next_lease = time.monotonic()+2.
                if not self.base.receipt_wait:
                    time.sleep(.25)
                else:
                    time.sleep(max(0.,.06-(time.monotonic()-polled_at)))
            raise RecoveryWait('基站采样超时，确认关窗后从当前状态继续')
        except (Exception,KeyboardInterrupt):
            # Require close acknowledgment before the caller can initiate a charging move.
            self.base.cancel(request)
            raise
        finally:
            self.off()


class Rehearsal:
    def __init__(self, stations, receivers):
        self.stations, self.receivers = stations,receivers
        self.pair = (next(iter(stations.points)),next(reversed(stations.points)))

    def positions(self):
        return self.pair

    def place(self, pair):
        self.pair = pair
        return {car:{'point_id':identity,'pose':[ *point_xy(self.stations.points[identity]),self.stations.yaw],
                     'frame':'map','map_id':self.stations.map_id} for car,identity in zip(CARS,pair)}

    def reconcile(self, request):
        return None

    def service_energy(self):
        pass

    def capture(self, request):
        state = request['state_id']
        values = {'A': -49.-state%6, 'B': -58.+state%3}
        return {**request,'status':'complete','raw_ref':f'rehearsal/{request["trial_id"]}',
                'data':{key:{'rssi_dbm':values,'epoch':state,'samples':dict.fromkeys(CARS,request['min_samples'])}
                        for key in self.receivers}}


def unattended_motion(driver, *, start, count, max_pairs, recharge_after, window_s, resume_operator):
    """Keep the authorized run alive while it waits for a safe way to continue."""
    journal=driver.journal
    started=False
    delay=2.
    completed=journal.motion_completed()
    while True:
        try:
            driver.check_pause()
            if not started:
                journal.set_activity('recovering','核对两车在线状态和定位')
                allowed_resume,resume_operator=resume_operator,False
                driver.start(start,None,0,window_s,allowed_resume)
                started=True
                driver.manual_controls_ready=True
            if getattr(driver,'manual_returning',False):
                driver.service_energy()
                journal.set_activity('paused','两车已回充，测试已结束')
                return
            position_rounds(journal,driver.stations,driver,start=start,count=count,max_pairs=max_pairs,
                            recharge_after=recharge_after,order_pairs=driver.order_motion_pairs)
        except ReturnRequested:
            driver.manual_returning=True
            driver.stop()
        except ChargeNeeded as error:
            journal.set_charging(list(dict.fromkeys(journal.charging()+error.cars)))
            driver.stop()
            delay=2.
        except (OSError,RecoveryWait,RouteRetry) as error:
            driver.stop()
            actual=journal.motion_completed()
            if actual>completed:
                completed=actual
                delay=2.
            journal.set_activity('recovering',str(error))
            print(f'保留当前轮次，{delay:g} 秒后自动复核。{error}',flush=True)
            try:
                driver.wait_retry(delay)
            except ReturnRequested:
                driver.manual_returning=True
                driver.stop()
            delay=min(delay*2,30.)
        else:
            start+=count
            delay=2.


def continuous_capture(driver, collect):
    """Resume the same journal after transient failures in an authorized live run."""
    journal=driver.journal
    recovering=False
    delay=2.
    while True:
        attempted=time.monotonic()
        try:
            driver.check_pause()
            if recovering and not journal.pending():
                driver.recover_capture_pose()
                driver.wait_parked()
                journal.set_activity('running','两车通信和定位已恢复，继续原采集断点')
            return collect()
        except ChargeNeeded as error:
            journal.set_charging(list(dict.fromkeys(journal.charging()+list(error.cars))))
            driver.stop()
            recovering=False
            delay=2.
        except (RouteRetry,OSError,RecoveryWait) as error:
            driver.stop()
            driver.off()
            if time.monotonic()-attempted>=30.:
                delay=2.
            recovering=True
            journal.set_activity('recovering',f'{error}。{delay:g} 秒后自动复核两车并续采')
            print(f'保留当前采集断点，{delay:g} 秒后自动复核。{error}',flush=True)
            driver.wait_retry(delay)
            delay=min(delay*2,30.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route',type=Path,required=True)
    parser.add_argument('--station-count',type=int,help='Use only the first N waypoints from the route')
    parser.add_argument('--initial-departure-voltage',type=float,
                        help='Optional first departure voltage at a verified dock for a fresh live run')
    parser.add_argument('--early-launch-voltage',type=float,
                        help='One operator-authorized early departure of both cars from confirmed charging docks')
    parser.add_argument('--journal',type=Path,required=True)
    parser.add_argument('--start',type=int,default=0)
    parser.add_argument('--count',type=int,default=16)
    parser.add_argument('--max-pairs',type=int,default=12,help='Pair budget (0 tests all admissible pairs)')
    parser.add_argument('--pair-first',action='store_true',help='Test each distinct point pair until the first joint 6 dB hit')
    parser.add_argument('--min-samples',type=int,default=30)
    parser.add_argument('--window-s',type=float,default=1.)
    parser.add_argument('--c5-window-s',type=float,help='Override the C5 window in seconds for a new radio experiment')
    parser.add_argument('--parallel-radios',action='store_true',help='Run the WS63 and C5 receive windows concurrently')
    parser.add_argument('--live-config',type=Path,help='Explicitly enable physical control with this configuration')
    parser.add_argument('--energy-history',type=Path,help='Read the local energy recorder for extended charge plateaus')
    parser.add_argument('--motion-only',action='store_true',help='Visit ordered position pairs without a radio or base station')
    parser.add_argument('--unattended',action='store_true',help='Keep a live motion rehearsal running until explicitly stopped')
    parser.add_argument('--recharge-after',type=int,help='Request one real charge-and-resume after this many verified motion rounds')
    parser.add_argument('--bench',action='store_true',help='Run the radio preflight on parked cars and exit')
    parser.add_argument('--release',action='store_true',help='Stop and release cars from this journal without starting motion')
    parser.add_argument('--resume-operator',action='store_true',help='Resume after a manual stop once the site is checked')
    parser.add_argument('--pause-file',type=Path,help='Honor an operator pause and release stopped cars on exit')
    args=parser.parse_args()
    route=json.loads(args.route.read_text(encoding='utf-8'))
    if args.station_count is not None:
        if not 2<=args.station_count<=len(route['points']):
            parser.error('点位数量超出所选路线范围')
        route={**route,'points':route['points'][:args.station_count]}
    stations=Stations(route)
    if args.max_pairs==0 and not args.motion_only:
        args.max_pairs=len({tuple(sorted(pair)) for pair in stations.pairs}) if args.pair_first else len(stations.pairs)
    if not (args.start>=0 and 1<=args.count<=STATE_COUNT and (args.unattended or args.start+args.count<=STATE_COUNT)
            and 1<=args.max_pairs<=len(stations.pairs) and 5<=args.min_samples<=5000
            and .2<=args.window_s<=180.):
        parser.error('状态范围、测量窗口或搜索预算无效')
    if ((args.bench or args.release) and not args.live_config) or (args.bench and args.release):
        parser.error('静止联调与释放二选一，并提供真实设备配置')
    if args.motion_only and args.bench:
        parser.error('仅行驶模式不使用基站联调')
    if args.motion_only and args.pair_first:
        parser.error('按位置对扫描需要射频采样模式')
    if args.c5_window_s is not None and (args.motion_only or not .2<=args.c5_window_s<=5):
        parser.error('独立 C5 窗口仅用于射频采样，范围为 0.2–5 秒')
    if args.motion_only and args.parallel_radios:
        parser.error('双频并行仅用于射频采样')
    if args.initial_departure_voltage is not None and (
            not args.live_config or not args.pair_first or args.bench or args.release
            or not math.isfinite(args.initial_departure_voltage)):
        parser.error('首次离桩电压仅用于新的实车按位置对采集')
    if args.early_launch_voltage is not None and (
            not args.live_config or not args.pair_first or args.bench or args.release
            or args.initial_departure_voltage is not None or not math.isfinite(args.early_launch_voltage)
            or not args.journal.is_file()):
        parser.error('提前离桩只用于操作员已授权续采的实车位置对实验')
    if args.unattended and (not args.motion_only or not args.live_config or args.release):
        parser.error('持续运行需要两车实车仅行驶模式')
    if args.pause_file is not None and (not args.live_config or args.release):
        parser.error('暂停信号文件用于实车任务')
    if args.recharge_after is not None and (not args.motion_only or not args.live_config
            or not 1<=args.recharge_after<args.count):
        parser.error('回充验收需要实车仅行驶模式，且充后还有至少一轮')
    config=json.loads(args.live_config.read_text(encoding='utf-8')) if args.live_config else {}
    sampling=dict(window_s=args.window_s,c5_window_s=args.c5_window_s,parallel_radios=args.parallel_radios)
    if args.pair_first and 'new_pair_sampling' in config:
        profile=config['new_pair_sampling']
        if (not isinstance(profile,dict) or profile.keys()!=sampling.keys()
                or any(type(profile[key]) not in (int,float) or not .2<=profile[key]<=5
                       for key in ('window_s','c5_window_s'))
                or type(profile['parallel_radios']) is not bool):
            parser.error('新点对需要有效的双频窗口设置')
        sampling=profile
    if args.motion_only:
        receivers,links,codes=(),{},None
        record_config=dict(route=route,motion_only=True,live=bool(args.live_config),max_pairs=args.max_pairs)
    else:
        receivers=tuple(config.get('criteria',['sle/main','c5/main']))
        if not receivers or len(set(receivers)) != len(receivers) or any(
                key.split('/')[0] not in ('sle','c5') or '/' not in key or not key.split('/')[-1] for key in receivers):
            parser.error('请设置判定所用的接收通道')
        links=station_ids(config.get('tx',{'sle':WS63_IDS,'c5':{'A':'02:00:00:00:00:c1','B':'02:00:00:00:00:c2'}}))
        codes=json.loads((args.live_config.parent/config['c5_states']).read_text(encoding='utf-8')) if args.live_config else list(range(STATE_COUNT))
        if (len(codes)!=STATE_COUNT or len(set(codes))!=STATE_COUNT
                or any(type(code) is not int or not 0 <= code <= 0xffffff for code in codes)):
            parser.error('C5 必须提供 4096 个明确的原始 GPIO 状态字')
        record_config=dict(route=route,tx=links,receivers=receivers,c5_codes=codes,live=bool(args.live_config),
                           max_pairs=args.max_pairs,min_samples=args.min_samples,window_s=args.window_s)
        if args.c5_window_s is not None:
            record_config['c5_window_s']=args.c5_window_s
        if args.parallel_radios:
            record_config['parallel_radios']=True
        if args.pair_first:
            if {key.split('/')[0] for key in receivers}!={'sle','c5'}:
                parser.error('按位置对采集须同时指定 WS63 和 C5 接收链')
            record_config.update(order='pair-first-joint-hit',start=args.start,count=args.count)
        if config.get('resume_voltage_by_car'):
            record_config['resume_voltage_by_car']=config['resume_voltage_by_car']
    journal=Journal(args.journal,record_config)
    driver=(Live(config,args.live_config.parent,stations,journal,links,receivers,
                 motion_only=args.motion_only,pause_file=args.pause_file,energy_history=args.energy_history) if args.live_config
            else Rehearsal(stations,receivers))
    try:
        if args.live_config:
            if args.release:
                return driver.release_available()
            if args.unattended:
                unattended_motion(driver,start=args.start,count=args.count,max_pairs=args.max_pairs,
                                  recharge_after=args.recharge_after,window_s=args.window_s,resume_operator=args.resume_operator)
                return 0
            while True:
                try:
                    driver.start(args.start,codes,args.min_samples,resume_operator=args.resume_operator,**sampling)
                    break
                except (OSError,RecoveryWait) as error:
                    if args.bench or args.motion_only:
                        raise
                    driver.stop()
                    driver.off()
                    journal.set_activity('recovering',f'{error}。等待两车和基站重新通过预检')
                    print(f'预检暂未通过，5 秒后复核：{error}',flush=True)
                    driver.wait_retry(5.)
            if args.bench:
                driver.completed=True
                return 0
            if args.early_launch_voltage is not None:
                if not driver.charge_plateaus:
                    raise ValueError('提前离桩仅用于已启用双车自动回充的实验')
                driver.early_launch_voltage=args.early_launch_voltage
                driver.check_early_launch()
            if (args.initial_departure_voltage is not None and not driver.charge_plateaus and not journal.charging()
                    and not journal.connection.execute('SELECT 1 FROM trials LIMIT 1').fetchone()):
                for car,state in driver.sync().items():
                    energy=state['fas']['energy']
                    if not energy['low_voltage']<args.initial_departure_voltage<=energy['resume_voltage']:
                        raise ValueError('首次离桩电压须高于低电阈值且不超过续采电压')
                    driver.initial_departure[car]=args.initial_departure_voltage
            if not args.motion_only:
                collect=sweep_pairs if args.pair_first else search
                coverage={}
                if args.pair_first:
                    coverage={key:config.get(setting,0.) for key,setting in
                              (('minimum_spacing','pair_spacing_m'),('preferred_span','pair_preferred_span_m'))}
                    if any(type(value) not in (float,int) or not math.isfinite(value) or value<0.
                           for value in coverage.values()):
                        raise ValueError('位置对距离配置须为非负有限数值')
                    coverage['defer_underfilled']=True
                    coverage['use_pair_evidence']=config.get('pair_use_evidence',True)
                    if type(coverage['use_pair_evidence']) is not bool:
                        raise ValueError('基于实测的位置对抽样设置须为布尔值')
                continuous_capture(driver,lambda:collect(
                    journal,stations,codes,links,receivers,driver,start=args.start,count=args.count,
                    max_pairs=args.max_pairs,min_samples=args.min_samples,**sampling,**coverage))
                driver.completed=True
                return 0
        while True:
            try:
                if args.motion_only:
                    position_rounds(journal,stations,driver,start=args.start,count=args.count,
                                    max_pairs=args.max_pairs,recharge_after=args.recharge_after,
                                    order_pairs=driver.order_motion_pairs if args.live_config else None)
                else:
                    collect=sweep_pairs if args.pair_first else search
                    collect(journal,stations,codes,links,receivers,driver,start=args.start,count=args.count,
                            max_pairs=args.max_pairs,min_samples=args.min_samples,**sampling)
                if args.live_config:
                    driver.completed=True
                break
            except ChargeNeeded as error:
                journal.set_charging(list(dict.fromkeys(journal.charging()+error.cars)))
                driver.stop()
                driver.service_energy()
    except (PauseRequested,KeyboardInterrupt):
        if args.live_config:
            journal.set_activity('paused','已暂停 · 进度已保存')
        print(f'两车已请求停车，进度保存在 {args.journal}。',flush=True)
        return 130
    except (OSError,ValueError,KeyError,URLError,RouteRetry) as error:
        if args.live_config:
            journal.set_activity('blocked',str(error))
        print(f'实验已暂停，进度保存在 {args.journal}。{error}',file=sys.stderr)
        return 1
    finally:
        try:
            if args.live_config:
                driver.close()
        finally:
            report=args.journal.with_suffix('.json')
            report.write_text(json.dumps(journal.report(),indent=2,ensure_ascii=False),encoding='utf-8')
            journal.close()


if __name__=='__main__':
    sys.exit(main())
