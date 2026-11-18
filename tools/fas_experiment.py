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
from arrival_policy import FAS_STATION_YAW_TOLERANCE
from fas_experiment import (CARS, PROTOCOL, STATE_COUNT, Journal, Stations, canonical, capture_request,
                            evaluate, move_batches, phase_state, point_xy, sample_box, search, station_ids,
                            yaw_error)
from fas_car import WS63_IDS


class ChargeNeeded(Exception):
    def __init__(self, cars):
        self.cars = cars


class RouteRetry(Exception):
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
    def __init__(self, settings, config_dir, stations, journal, links, receivers):
        self.stations, self.journal, self.links = stations,journal,links
        self.cars = {car:Endpoint(settings['cars'][car]) for car in CARS}
        if len({value.url for value in self.cars.values()}) != 2:
            raise ValueError('两车必须分别使用独立的车端入口')
        self.base = Base(settings['base'],config_dir)
        self.receivers = receivers
        self.pool = ThreadPoolExecutor(max_workers=4)
        self.states = {}
        self.started_low = {car:None for car in CARS}
        self.cache = {}
        self.anchor = None
        self.claimed = set()
        self.completed = False
        self.command_timeout = float(settings.get('navigation_timeout_s',150.))
        self.charge_timeout = float(settings.get('charge_timeout_s',14400.))

    def parallel(self, cars, fn):
        futures = {car:self.pool.submit(fn,car) for car in cars}
        return {car:job.result() for car,job in futures.items()}

    def sync(self):
        self.states = self.parallel(CARS,lambda car:self.cars[car].fetch('/api/state'))
        for car,state in self.states.items():
            health = state['health']
            fas = state.get('fas') or {}
            if (state['map']['map_id'] != self.stations.map_id or fas.get('car') != car
                    or fas.get('protocol') != PROTOCOL or not health['dock_calibrated']
                    or fas['tx'] != {band:values[car] for band,values in self.links.items()}):
                raise ValueError(f'{car} 车的配置、地图、桩位或发端身份未对齐')
        return self.states

    def command(self, car, action, *, timeout=.85, **payload):
        state = self.states[car]
        request = {'action':action,'run_id':self.journal.run_id,'map_id':self.stations.map_id,**payload}
        return self.cars[car].fetch('/api/command',request,{'X-Control-Token':state['control_token']},timeout)

    def start(self, preflight_state, c5_codes, min_samples, window_s, resume_operator):
        self.sync()
        base = self.base.get('/fas/health')
        if (base.get('protocol') != PROTOCOL or base.get('ready') is not True or base.get('tx_configured') != self.links
                or not set(self.receivers) <= set(base.get('receivers', []))):
            raise ValueError('基站协议、接收通道或两车发端仍未就绪')
        for car in CARS:
            config = self.states[car]['fas']['energy']
            if not config.get('enabled') or not config.get('resume_voltage'):
                raise ValueError(f'{car} 车需要有效的低电回充与续作阈值')
            self.claimed.add(car)
            self.command(car,'fas_claim',timeout=3.)
            # An earlier process may have ended mid-motion. Revoke old grants first.
            self.command(car,'fas_stop',timeout=3.)
        pending = self.journal.pending()
        if pending:
            request = json.loads(pending['request'])
            self.base.cancel(request)
            self.journal.abandon(pending,'重启后关闭旧窗口，保留原始数据并重新测量')
        self.wait_parked()
        if any(s['fas']['operator_hold'] for s in self.states.values()):
            if not resume_operator:
                raise ValueError('操作员已停车，请核对现场后显式恢复')
            self.parallel(CARS,lambda car:self.command(car,'fas_resume'))
        self.parallel(CARS,lambda car:self.command(car,'fas_probe',timeout=3.))
        test_poses = self.actual_poses(('preflight-A','preflight-B'))
        check = capture_request(self.journal.run_id,uuid.uuid4().hex,preflight_state,test_poses,
                                phase_state(preflight_state,c5_codes),self.links,
                                min_samples=min_samples,window_s=window_s)
        check['purpose'] = 'preflight'
        evaluate(check,self.capture(check),self.receivers)
        print('两车静止预检通过，基站已确认各指定通道分别收到 A、B 帧。',flush=True)

    def wait_parked(self):
        deadline = time.monotonic()+8.
        while time.monotonic() < deadline:
            self.sync()
            if all(s['phase'] in ('idle','docked','completed','failed') and s['health']['stationary']
                   and s['health']['ready'] and s['health']['localization_confirmed'] for s in self.states.values()):
                return
            time.sleep(.3)
        raise ValueError('两车未能同时确认停稳与有效定位')

    def off(self):
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
            if self.completed:
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
            if h.get('charging_stable') and h['voltage'] is not None and h['voltage'] < config['resume_voltage']:
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

    def drive_batch(self, cars, paths, endpoints, end_yaws, *, returning=False):
        other = set(CARS)-set(cars)
        parked = {car:tuple(self.states[car]['health']['pose']) for car in other}
        epoch = uuid.uuid4().hex
        kind = 'return' if returning else 'move'
        self.parallel(cars,lambda car:self.command(
            car,'fas_reserve',epoch=epoch,kind=kind,bounds=sample_box(paths[car]),timeout=3.))
        try:
            self.parallel(cars,lambda car:self.command(car,'fas_drive',epoch=epoch,kind=kind,point=endpoints[car]))
            deadline = time.monotonic()+self.command_timeout
            while time.monotonic() < deadline:
                self.parallel(cars,lambda car:self.command(car,'fas_renew',epoch=epoch))
                states = self.sync()
                poses = {}
                for car,state in states.items():
                    h = state['health']
                    if state['fas']['operator_hold']:
                        raise ValueError('操作员已停车')
                    if not h['ready'] or not h['localization_confirmed'] or not h.get('pose'):
                        raise ValueError(f'{car} 车定位暂不可用，已收回两车运动许可')
                    poses[car] = h['pose']
                    if car in parked and (not h['stationary'] or math.dist(parked[car][:2],poses[car][:2])>.075):
                        raise ValueError(f'{car} 车的停车位置变化，已收回运动许可')
                if math.dist(poses['A'][:2], poses['B'][:2]) < .70:
                    raise ValueError('两车安全间距缩小，已收回运动许可')
                if any(states[car]['phase'] in ('failed','idle') and
                       not (returning and states[car]['fas']['dock_retry_pending']) for car in cars):
                    raise RouteRetry('本次路线需要停稳后重新规划')
                if not returning:
                    low = self.low_cars()
                    if low:
                        raise ChargeNeeded(low)
                if all(states[car]['health']['stationary'] and states[car]['phase'] ==
                       ('docked' if returning else 'completed') for car in cars):
                    if not returning and any(math.dist(poses[car][:2],point_xy(endpoints[car]))>.055
                                             or yaw_error(poses[car][2],end_yaws[car])>FAS_STATION_YAW_TOLERANCE
                                             for car in cars):
                        raise RouteRetry('有车停稳后的位置或统一朝向需重校')
                    break
                time.sleep(.18)
            else:
                raise RouteRetry('本轮导航超时')
        finally:
            # A successful native dock is left in its actual stopped/charging state.
            if not returning or any(self.states[car]['phase']!='docked' for car in cars):
                self.stop(cars)
            self.cache.clear()

    def place(self, pair):
        for _ in range(2):
            try:
                return self.place_once(pair)
            except RouteRetry:
                self.wait_parked()
                self.cache.clear()
        return None

    def place_once(self, pair):
        self.positions()
        target = dict(zip(CARS, (self.stations.points[p] for p in pair)))
        starts = {car:self.states[car]['health']['pose'] for car in CARS}
        ends = {car:point_xy(point) for car,point in target.items()}
        yaws = {car:point['yaw'] for car,point in target.items()}
        paths = {}
        for car in CARS:
            if (math.dist(starts[car][:2],ends[car])<=.045
                    and yaw_error(starts[car][2],yaws[car])<=FAS_STATION_YAW_TOLERANCE):
                paths[car] = []
            else:
                try:
                    paths[car] = self.preview(car,target[car])
                except ValueError as exc:
                    if '没有可走' not in str(exc):
                        raise
                    return None
        batches = move_batches(starts,ends,paths)
        if batches is None:
            return None
        for batch in batches:
            self.drive_batch(batch,paths,target,yaws)
            self.wait_parked()
        return self.measured(pair)

    def measured(self, pair):
        poses = self.actual_poses(pair)
        for car, identity in zip(CARS,pair):
            point = self.stations.points[identity]
            pose = poses[car]['pose']
            if math.dist(pose[:2],point_xy(point))>.055 or yaw_error(pose[2],point['yaw'])>FAS_STATION_YAW_TOLERANCE:
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
        todo = list(dict.fromkeys(self.journal.charging()+self.low_cars()))
        self.journal.set_charging(todo)
        while todo:
            car = todo[0]
            peer = 'B' if car == 'A' else 'A'
            self.cache.clear()
            state = self.states[car]
            if not state['health'].get('charging_stable'):
                for retry in range(2):
                    starts = {k:s['health']['pose'] for k,s in self.states.items()}
                    path = {car:self.preview(car)}
                    if move_batches(starts,{car:state['dock']},path,exclusive=True) is None:
                        raise ValueError('回充路线与另一车停车占用冲突，进度已保存')
                    try:
                        self.drive_batch((car,),path,{car:None},{},returning=True)
                        break
                    except RouteRetry:
                        self.wait_parked()
                else:
                    raise ValueError(f'{car} 车回充重试后仍需核对通道或触点')
            config = self.states[car]['fas']['energy']
            deadline = time.monotonic()+self.charge_timeout
            stable_since = ready_since = None
            while time.monotonic()<deadline:
                self.sync()
                health = self.states[car]['health']
                if not self.states[peer]['health']['stationary']:
                    raise ValueError('充电期间另一车状态变化')
                now = time.monotonic()
                stable = health.get('charging_stable') and health['stationary']
                stable_since = (stable_since or now) if stable else None
                confirmed = (stable and health['voltage'] is not None and health['voltage'] >= config['resume_voltage']
                             and health['localization_confirmed'] and health.get('dock_localization',{}).get('status')=='confirmed')
                ready_since = (ready_since or now) if confirmed else None
                if (ready_since and now-ready_since>=config['charge_hold_seconds']
                        and now-stable_since>=config.get('min_charge_seconds',0.)):
                    break
                time.sleep(1.)
            else:
                raise ValueError(f'{car} 车的充电或桩位校准尚未达到续采条件')
            todo.pop(0)
            self.journal.set_charging(todo)
            self.started_low[car] = None

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
    parser.add_argument('--bench',action='store_true',help='Run the radio preflight on parked cars and exit')
    parser.add_argument('--release',action='store_true',help='Stop and release cars from this journal without starting motion')
    parser.add_argument('--resume-operator',action='store_true',help='Resume after a manual stop once the site is checked')
    args=parser.parse_args()
    route=json.loads(args.route.read_text(encoding='utf-8'))
    stations=Stations(route)
    if not (0<=args.start<STATE_COUNT and 1<=args.count<=STATE_COUNT-args.start
            and 1<=args.max_pairs<=len(stations.pairs) and 5<=args.min_samples<=5000
            and .2<=args.window_s<=180.):
        parser.error('状态范围、测量窗口或搜索预算无效')
    if ((args.bench or args.release) and not args.live_config) or (args.bench and args.release):
        parser.error('静止联调与释放二选一，并提供真实设备配置')
    config=json.loads(args.live_config.read_text(encoding='utf-8')) if args.live_config else {}
    receivers=tuple(config.get('criteria',['sle/main','c5/main']))
    if not receivers or len(set(receivers)) != len(receivers) or any(
            key.split('/')[0] not in ('sle','c5') or '/' not in key or not key.split('/')[-1] for key in receivers):
        parser.error('请设置判定所用的接收通道')
    links=station_ids(config.get('tx',{'sle':WS63_IDS,'c5':{'A':'02:00:00:00:00:c1','B':'02:00:00:00:00:c2'}}))
    codes=json.loads((args.live_config.parent/config['c5_states']).read_text(encoding='utf-8')) if args.live_config else list(range(STATE_COUNT))
    if (len(codes)!=STATE_COUNT or len(set(codes))!=STATE_COUNT
            or any(type(code) is not int or not 0 <= code <= 0xffffff for code in codes)):
        parser.error('C5 必须提供 4096 个明确的原始 GPIO 状态字')
    journal=Journal(args.journal,dict(route=route,tx=links,receivers=receivers,c5_codes=codes,live=bool(args.live_config),
                                     max_pairs=args.max_pairs,min_samples=args.min_samples,window_s=args.window_s))
    driver=(Live(config,args.live_config.parent,stations,journal,links,receivers) if args.live_config
            else Rehearsal(stations,receivers))
    try:
        if args.live_config:
            if args.release:
                return driver.release_available()
            driver.start(args.start,codes,args.min_samples,args.window_s,args.resume_operator)
            if args.bench:
                driver.completed=True
                return 0
        while True:
            try:
                search(journal,stations,codes,links,receivers,driver,start=args.start,count=args.count,
                       max_pairs=args.max_pairs,min_samples=args.min_samples,window_s=args.window_s)
                if args.live_config:
                    driver.completed=True
                break
            except ChargeNeeded as error:
                journal.set_charging(list(dict.fromkeys(journal.charging()+error.cars)))
                driver.stop()
                driver.service_energy()
    except (OSError,ValueError,KeyError,URLError,RouteRetry) as error:
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
