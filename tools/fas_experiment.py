#!/usr/bin/env python3
"""Run a resumable FAS pair search from the base host. Offline rehearsal by default."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen
import uuid

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'demo'))
from arrival_policy import FAS_STATION_POSITION_TOLERANCE, FAS_STATION_YAW_TOLERANCE
from fas_experiment import (CARS, EXCLUSIVE_LOCALIZATION_PAD, LOCALIZATION_PAD, MAX_FLEET_SIGMA,
                            PROTOCOL, STATE_COUNT, Journal, RecoveryWait, Stations, boxes_apart, canonical,
                            capture_request, clear_of_parked_peer, evaluate, fleet_pose_ready, move_batches,
                            parallel_bounds, peer_clearance, phase_state, point_xy, position_rounds, position_sigma,
                            sample_box, search, station_ids, stopped_station_arrival, yaw_error)
from fas_car import NOT_PARKED, WS63_IDS


DOCK_SETTLE_SECONDS = 30.
PARKED_PHASES = ('idle','docked','completed','failed')


class ChargeNeeded(Exception):
    def __init__(self, cars):
        self.cars = cars


class RouteRetry(Exception):
    pass


class ReturnRequested(Exception):
    pass


class PauseRequested(Exception):
    pass


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
        except HTTPError as exc:
            if missing and exc.code == 404:
                return None
            try:
                message = json.load(exc).get('error',f'HTTP {exc.code}')
            except (ValueError, AttributeError):
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

    def get(self, path, *, missing=False):
        return self.api.fetch(path,headers=self.headers,timeout=2.,missing=missing)

    def post(self, path, data):
        return self.api.fetch(path,data,self.headers,timeout=2.)

    def receipt(self, trial):
        result = self.get('/fas/capture/'+trial,missing=True)
        if result and result.get('trial_id') != trial:
            raise ValueError('基站返回了不同的采样窗口')
        return result

    def cancel(self, request):
        result = self.post('/fas/cancel',{k:request[k] for k in ('run_id','trial_id')})
        if result.get('status') not in ('idle','canceled','complete'):
            raise ValueError('基站尚未关闭上一采样窗口')


class Live:
    def __init__(self, settings, config_dir, stations, journal, links, receivers, *, motion_only=False, pause_file=None):
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
        self.cache = {}
        self.anchor = None
        self.claimed = set()
        self.completed = False
        self.recovery_at = dict.fromkeys(CARS,float('-inf'))
        self.command_timeout = float(settings.get('navigation_timeout_s',150.))
        self.charge_timeout = float(settings.get('charge_timeout_s',14400.))

    def parallel(self, cars, fn):
        futures = {car:self.pool.submit(fn,car) for car in cars}
        return {car:job.result() for car,job in futures.items()}

    def sync(self, *, renew=(), epoch=None, check_operator=True):
        self.check_pause()
        def sample(car):
            if car in renew:
                try:
                    status=self.command(car,'fas_renew',epoch=epoch)
                except ValueError as error:
                    if '行驶许可已过期' not in str(error):
                        raise
                    return {**self.car_state(car),'motion_lease_expired':True}
                if 'health' in status:
                    return status
            return self.car_state(car)
        self.states = self.parallel(CARS,sample)
        for car,state in self.states.items():
            health = state['health']
            fas = state.get('fas') or {}
            expected_links = {band:values[car] for band,values in self.links.items()}
            if (state['map']['map_id'] != self.stations.map_id or fas.get('car') != car
                    or fas.get('protocol') != PROTOCOL or not health['dock_calibrated']
                    or fas.get('motion_only',False)!=self.motion_only or fas['tx']!=expected_links):
                raise ValueError(f'{car} 车的试验模式、地图、桩位或发端身份未对齐')
            owner=fas.get('owner')
            if owner not in (None,self.journal.run_id):
                raise ValueError(f'{car} 车已交给另一任务')
            if check_operator and owner==self.journal.run_id and fas.get('operator_hold'):
                raise ValueError(f'{car} 车已手动停车')
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

    def start(self, preflight_state, c5_codes, min_samples, window_s, resume_operator):
        self.window_s = window_s
        self.sync(check_operator=not resume_operator)
        held=tuple(car for car,s in self.states.items() if s['fas']['operator_hold'])
        if not self.motion_only:
            base = self.base.get('/fas/health')
            if (base.get('protocol') != PROTOCOL or base.get('ready') is not True or base.get('tx_configured') != self.links
                    or not set(self.receivers) <= set(base.get('receivers', []))):
                raise ValueError('基站协议、接收通道或两车发端仍未就绪')
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
        self.parallel(CARS,lambda car:self.command(car,'fas_probe',timeout=3.))
        test_poses = self.actual_poses(('preflight-A','preflight-B'))
        check = capture_request(self.journal.run_id,uuid.uuid4().hex,preflight_state,test_poses,
                                phase_state(preflight_state,c5_codes),self.links,
                                min_samples=min_samples,window_s=window_s)
        check['purpose'] = 'preflight'
        evaluate(check,self.capture(check),self.receivers)
        print('两车静止预检通过，基站已确认各指定通道分别收到 A、B 帧。',flush=True)

    def wait_parked(self):
        deadline = time.monotonic()+(30. if self.motion_only else 15.)
        while time.monotonic() < deadline:
            try:
                self.sync()
            except OSError:
                time.sleep(.3)
                continue
            if all(s['phase'] in PARKED_PHASES and s['health']['stationary'] and not s['fas']['lease_live']
                   for s in self.states.values()):
                if all(fleet_pose_ready(s['health']) for s in self.states.values()):
                    return
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

    def off(self):
        if self.motion_only:
            return
        for car in tuple(self.claimed):
            try:
                self.command(car,'fas_radio',enabled=False,timeout=3.)
            except (OSError, ValueError, KeyError):
                # WS63 also expires its own transmit lease.
                continue

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
            try:
                self.states[car] = self.cars[car].fetch('/api/state')
                if self.states[car].get('fas', {}).get('owner') != self.journal.run_id:
                    continue
                self.claimed.add(car)
                self.command(car,'fas_stop')
                deadline = time.monotonic()+8.
                while time.monotonic()<deadline:
                    self.states[car] = self.cars[car].fetch('/api/state')
                    state = self.states[car]
                    if state['health']['stationary'] and state['phase'] in ('idle','docked','completed','failed'):
                        self.command(car,'fas_release',timeout=3.)
                        self.claimed.remove(car)
                        released.append(car)
                        break
                    time.sleep(.3)
                else:
                    pending.append(car)
            except (OSError,ValueError,KeyError):
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
            low = h['voltage'] is not None and h['voltage'] < config['low_voltage']
            self.started_low[car] = (self.started_low[car] or now) if low else None
            if (not self.motion_only and h.get('charging_stable') and h['voltage'] is not None
                    and h['voltage'] < config['resume_voltage']):
                ready.append(car)
            elif low and now-self.started_low[car] >= config['low_hold_seconds']:
                ready.append(car)
        return ready

    def preview(self, car, point=None):
        key = car, None if point is None else point['id']
        if key not in self.cache:
            response = self.command(car,'fas_preview',kind='return' if point is None else 'move',
                                    point=point,timeout=7.)
            self.cache[key] = response['navigation_path']
        return self.cache[key]

    def position_sigmas(self):
        sigmas = {car:position_sigma(state['health']) for car,state in self.states.items()}
        for car,sigma in sigmas.items():
            if sigma is None or sigma > MAX_FLEET_SIGMA:
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
        docked_since = None
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
            timeout = 14. if retreating else max(200.,self.command_timeout) if returning else self.command_timeout
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
                clearance = peer_clearance(*self.position_sigmas().values(),moving_together=not parked)
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
                    health=states[cars[0]]['health']
                    stable=states[cars[0]]['phase']=='docked' and health['stationary'] and health['charging_stable']
                    docked_since=(docked_since if docked_since is not None else polled_at) if stable else None
                    if (docked_since is not None and polled_at-docked_since>=DOCK_SETTLE_SECONDS
                            and health['localization_confirmed'] and health.get('dock_localization',{}).get('status')=='confirmed'):
                        break
                elif not retreating:
                    low = [car for car in self.low_cars() if car!=charge_waiting]
                    if low:
                        raise ChargeNeeded(low)
                    if all(states[car]['health']['stationary'] and states[car]['phase']=='completed' for car in cars):
                        if any(math.dist(poses[car][:2],point_xy(endpoints[car]))>FAS_STATION_POSITION_TOLERANCE
                               or yaw_error(poses[car][2],end_yaws[car])>FAS_STATION_YAW_TOLERANCE for car in cars):
                            raise RouteRetry('有车停稳后的位置或统一朝向需重校')
                        break
                time.sleep(max(0.,.25-(time.monotonic()-polled_at)))
            else:
                raise RouteRetry('本轮导航超时')
        except OSError as exc:
            raise RouteRetry('车辆通信暂时中断，停车后重新规划') from exc
        finally:
            # A successful native dock is left in its actual stopped/charging state.
            if not returning or any(self.states[car]['phase']!='docked' for car in cars):
                self.stop(cars)
            self.cache.clear()

    def place(self, pair):
        exclusive=False
        for _ in range(2):
            try:
                poses = self.place_once(pair,exclusive=exclusive)
                if poses is not None and self.motion_only:
                    return self.simulate_window(pair,poses)
                return poses
            except (RouteRetry,OSError) as exc:
                print(f'本轮停稳后重试。{exc}',flush=True)
                self.wait_parked()
                self.recover_obstruction()
                self.cache.clear()
                exclusive=(not self.motion_only or any(
                    detail in str(exc) for detail in ('预约走廊','停车车辆的净空')
                ))
        return None

    def recover_obstruction(self):
        if not self.motion_only:
            return
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
            print(f'{car} 车空转点位已重新定位，申请一次受保护的短退。',flush=True)
            try:
                self.drive_batch((car,),{car:path},{car:None},{},retreating=True)
            except RouteRetry as error:
                print(f'短退未通过复核，保留空转区域。{error}',flush=True)
            self.wait_parked()

    def place_once(self, pair, *, exclusive=False):
        self.positions()
        target = dict(zip(CARS, (self.stations.points[p] for p in pair)))
        starts = {car:self.states[car]['health']['pose'] for car in CARS}
        ends = {car:point_xy(point) for car,point in target.items()}
        yaws = {car:point['yaw'] for car,point in target.items()}
        paths = self.station_paths(starts,target)
        if paths is None:
            return None
        clearance = peer_clearance(*self.position_sigmas().values())
        batches = move_batches(starts,ends,paths,exclusive=exclusive,clearance=clearance)
        if batches is None:
            return None
        for index,batch in enumerate(batches):
            if index:
                for car in batch:
                    paths[car]=self.preview(car,target[car])
            self.drive_batch(batch,paths,target,yaws)
            self.wait_parked()
        return self.measured(pair)

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

    def actual_poses(self, pair):
        self.wait_parked()
        poses = {}
        for car, identity in zip(CARS,pair):
            pose = self.states[car]['health']['pose']
            if self.states[car]['fas']['operator_hold'] or not pose:
                raise ValueError('车辆已停止实验，待重新确认')
            poses[car] = {'point_id':identity,'pose':pose,'frame':'map','map_id':self.stations.map_id}
        return poses

    def service_energy(self):
        self.wait_parked()
        self.recover_obstruction()
        manual=getattr(self,'manual_returning',False)
        todo = list(CARS) if manual else list(dict.fromkeys(self.journal.charging()+self.low_cars()))
        if todo and not manual and self.motion_only and self.pause_file and self.pause_file.with_suffix('.no-auto-charge').exists():
            self.pause_file.touch()
            raise PauseRequested
        self.auto_charging=bool(todo and not manual and self.motion_only and self.pause_file)
        try:
            self.service_charge_queue(todo,manual)
        finally:
            self.auto_charging=False

    def service_charge_queue(self,todo,manual):
        if not manual:self.journal.set_charging(todo)
        while todo:
            car = todo[0]
            peer = 'B' if car == 'A' else 'A'
            self.cache.clear()
            state = self.states[car]
            print(f'{car} 车开始回充检查。电压 {state["health"]["voltage"]} V。',flush=True)
            if not state['health'].get('charging_stable'):
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
                        if yielded or not self.motion_only or not self.yield_for_charge(car,path[car],clearance):
                            raise RecoveryWait('回充路线与另一车的停车位置冲突')
                        yielded=True
                        self.cache.clear()
                        state=self.states[car]
                        continue
                    try:
                        self.drive_batch((car,),path,{car:None},{},returning=True)
                        break
                    except RouteRetry:
                        self.wait_parked()
                else:
                    raise RecoveryWait(f'{car} 车等待回充通道或触点恢复')
            config = self.states[car]['fas']['energy']
            deadline = time.monotonic()+self.charge_timeout
            stable_since = ready_since = interrupted_since = None
            self.journal.set_activity('charging',f'{car} 车在桩上等待续作电压和桩位校准')
            while time.monotonic()<deadline:
                try:
                    self.sync()
                except OSError:
                    stable_since=ready_since=None
                    time.sleep(1.)
                    continue
                health = self.states[car]['health']
                if not self.states[peer]['health']['stationary']:
                    raise ValueError('充电期间另一车状态变化')
                now = time.monotonic()
                stable = health.get('charging_stable') and health['stationary']
                interrupted_since = None if stable else (interrupted_since if interrupted_since is not None else now)
                if interrupted_since is not None and now-interrupted_since>=12.:
                    raise RecoveryWait(f'{car} 车充电中断，待复核后重新回充')
                if manual and stable and health.get('charging_elapsed_s',0) is not None and (health.get('charging_elapsed_s') or 0)>=3.:break
                stable_since = (stable_since or now) if stable else None
                onboard_duration=health.get('charging_elapsed_s')
                elapsed=(onboard_duration if stable and isinstance(onboard_duration,(int,float))
                         and math.isfinite(onboard_duration) and onboard_duration>=0.
                         else now-stable_since if stable_since is not None else 0.)
                confirmed = (stable and health['voltage'] is not None and health['voltage'] >= config['resume_voltage']
                             and health['localization_confirmed'] and health.get('dock_localization',{}).get('status')=='confirmed')
                ready_since = (ready_since or now) if confirmed else None
                if (ready_since and now-ready_since>=config['charge_hold_seconds']
                        and elapsed>=config.get('min_charge_seconds',0.)):
                    break
                time.sleep(1.)
            else:
                raise RecoveryWait(f'{car} 车仍等待充电或桩位校准')
            todo.pop(0)
            if not manual:self.journal.set_charging(todo)
            self.started_low[car] = None
            print(f'{car} 车充电与桩位校准已通过，继续未完成轮次。',flush=True)
        self.journal.set_activity('running','位置对调度中')

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
            self.parallel(CARS,lambda car:self.command(car,'fas_radio',enabled=True,timeout=3.))
            now = time.monotonic()
            self.base.post('/fas/capture',request)
            deadline = now+max(12., request['window_s']*4)
            next_lease = now+2.
            while time.monotonic()<deadline:
                receipt = self.base.receipt(request['trial_id'])
                if receipt is not None and receipt.get('status') in ('complete','incomplete'):
                    after = self.actual_poses(pair) if request.get('purpose')=='preflight' else self.measured(pair)
                    for car in CARS:
                        first, last = request['cars'][car]['pose'],after[car]['pose']
                        if math.dist(first[:2],last[:2])>.025 or yaw_error(first[2],last[2])>math.radians(2):
                            raise ValueError('采样窗口内车位变化，保留数据并重采')
                    return {**receipt,'after_poses':after,'after_pose_verified':True}
                if time.monotonic()>=next_lease:
                    self.wait_parked()
                    if request.get('purpose')!='preflight':
                        low = self.low_cars()
                        if low:
                            raise ChargeNeeded(low)
                    self.parallel(CARS,lambda car:self.command(car,'fas_radio',enabled=True,timeout=3.))
                    next_lease = time.monotonic()+2.
                time.sleep(.25)
            raise ValueError('基站采样超时，保留当前状态待恢复')
        except Exception:
            try:
                self.base.cancel(request)
            except (OSError,ValueError):
                pass  # The base also owns and expires its capture lease.
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route',type=Path,required=True)
    parser.add_argument('--journal',type=Path,required=True)
    parser.add_argument('--start',type=int,default=0)
    parser.add_argument('--count',type=int,default=16)
    parser.add_argument('--max-pairs',type=int,default=12)
    parser.add_argument('--min-samples',type=int,default=30)
    parser.add_argument('--window-s',type=float,default=1.)
    parser.add_argument('--live-config',type=Path,help='Explicitly enable physical control with this configuration')
    parser.add_argument('--motion-only',action='store_true',help='Visit ordered position pairs without a radio or base station')
    parser.add_argument('--unattended',action='store_true',help='Keep a live motion rehearsal running until explicitly stopped')
    parser.add_argument('--recharge-after',type=int,help='Request one real charge-and-resume after this many verified motion rounds')
    parser.add_argument('--bench',action='store_true',help='Run the radio preflight on parked cars and exit')
    parser.add_argument('--release',action='store_true',help='Stop and release cars from this journal without starting motion')
    parser.add_argument('--resume-operator',action='store_true',help='Resume after a manual stop once the site is checked')
    parser.add_argument('--pause-file',type=Path,help='Honor a managed simulation pause and release stopped cars on exit')
    args=parser.parse_args()
    route=json.loads(args.route.read_text(encoding='utf-8'))
    stations=Stations(route)
    if not (args.start>=0 and 1<=args.count<=STATE_COUNT and (args.unattended or args.start+args.count<=STATE_COUNT)
            and 1<=args.max_pairs<=len(stations.pairs) and 5<=args.min_samples<=5000
            and .2<=args.window_s<=180.):
        parser.error('状态范围、测量窗口或搜索预算无效')
    if ((args.bench or args.release) and not args.live_config) or (args.bench and args.release):
        parser.error('静止联调与释放二选一，并提供真实设备配置')
    if args.motion_only and args.bench:
        parser.error('仅行驶模式不使用基站联调')
    if args.unattended and (not args.motion_only or not args.live_config or args.release):
        parser.error('持续运行需要两车实车仅行驶模式')
    if args.pause_file is not None and (not args.motion_only or not args.live_config or args.release):
        parser.error('暂停信号文件仅用于实车模拟采集任务')
    if args.recharge_after is not None and (not args.motion_only or not args.live_config
            or not 1<=args.recharge_after<args.count):
        parser.error('回充验收需要实车仅行驶模式，且充后还有至少一轮')
    config=json.loads(args.live_config.read_text(encoding='utf-8')) if args.live_config else {}
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
    journal=Journal(args.journal,record_config)
    driver=(Live(config,args.live_config.parent,stations,journal,links,receivers,
                 motion_only=args.motion_only,pause_file=args.pause_file) if args.live_config
            else Rehearsal(stations,receivers))
    try:
        if args.live_config:
            if args.release:
                return driver.release_available()
            if args.unattended:
                unattended_motion(driver,start=args.start,count=args.count,max_pairs=args.max_pairs,
                                  recharge_after=args.recharge_after,window_s=args.window_s,resume_operator=args.resume_operator)
                return 0
            driver.start(args.start,codes,args.min_samples,args.window_s,args.resume_operator)
            if args.bench:
                driver.completed=True
                return 0
        while True:
            try:
                if args.motion_only:
                    position_rounds(journal,stations,driver,start=args.start,count=args.count,
                                    max_pairs=args.max_pairs,recharge_after=args.recharge_after,
                                    order_pairs=driver.order_motion_pairs if args.live_config else None)
                else:
                    search(journal,stations,codes,links,receivers,driver,start=args.start,count=args.count,
                           max_pairs=args.max_pairs,min_samples=args.min_samples,window_s=args.window_s)
                if args.live_config:
                    driver.completed=True
                break
            except ChargeNeeded as error:
                journal.set_charging(list(dict.fromkeys(journal.charging()+error.cars)))
                driver.stop()
                driver.service_energy()
    except (PauseRequested,KeyboardInterrupt):
        if args.unattended:
            journal.set_activity('paused','已暂停 · 进度已保存')
        print(f'两车已请求停车，进度保存在 {args.journal}。',flush=True)
        return 130
    except (OSError,ValueError,KeyError,URLError,RouteRetry) as error:
        if args.unattended:
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
