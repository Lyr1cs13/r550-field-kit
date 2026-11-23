"""Waypoint/dwell/docking sequencing. No localization or motion-control algorithms."""
from collections import deque
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid

from c5_station import C5Station, utc
from collection import BROADCAST_MODES, BROADCAST_STATION, Collection
from servo_car import MODE as SERVO_MODE
from endurance import Endurance
from directed_route import angle_error, departure_goal, dock_footprint_intersects, dock_offset
import yaml


TERMINAL = {'succeeded', 'failed', 'canceled'}
QUIET = {'idle', 'completed', 'failed', 'docked'}
DOCK_RECHECK_SECONDS = 18.


class Mission:
    def __init__(self, grid, backend, directory, clock=time.monotonic):
        self.grid, self.backend, self.clock = grid, backend, clock
        self.directory = Path(directory);self.directory.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.points = [];self.events = deque(maxlen=100)
        self.phase = 'idle';self.message = '请添加导航点，核对定位后开始'
        self.run = [];self.index = 0;self.operation = None;self.pending = None
        self.repeat = False;self.lap = 0;self.station_retry_at = 0.;self.station_started = False
        self.until = 0;self.remaining = 0;self.auto_return = False;self.low_since = None
        self.return_via = None
        self.dock_failure_reconcile=False;self.dock_retry_count=0;self.dock_retry_pending=False
        self.dock_retry_stationary_since=None
        self._load()
        self.collection = Collection(self.directory,grid)
        self.c5 = C5Station(self.collection.directory / 'c5')
        self.c5_session = None
        self.health_pause=False;self.health_good_since=None
        self.endurance=Endurance(yaml.safe_load((Path(__file__).resolve().parent.parent/'config/endurance.yaml').read_text(encoding='utf-8')))

    def _load(self):
        p = self.directory / 'waypoints.json'
        if p.exists():
            try:
                data = json.loads(p.read_text(encoding='utf-8'))
                if data['map_id'] != self.grid.map_id:
                    self.message = '地图版本已改变，旧点位未加载，请重新核对点位'
                    return
                self.points = self.validate_points(data['points'])
            except (ValueError, KeyError, TypeError):
                self.message = '点位文件无效，请重新设置'

    def event(self, text):
        item = {'time': time.strftime('%H:%M:%S'), 'text': text}
        self.events.appendleft(item)
        with (self.directory / 'events.jsonl').open('a', encoding='utf-8') as f:
            f.write(json.dumps(item, ensure_ascii=False)+'\n')

    def validate_points(self, points):
        if not isinstance(points, list) or len(points) > 100:
            raise ValueError('最多设置 100 个导航点')
        result = [];ids = set()
        for i, point in enumerate(points):
            if not isinstance(point, dict):
                raise ValueError('点位格式错误')
            values = [float(point[k]) for k in ('x','y','yaw','wait')]
            if not all(math.isfinite(v) for v in values) or not 0 <= values[3] <= 3600:
                raise ValueError('坐标与朝向必须有效，停留时间须在 0～3600 秒内')
            self.grid.validate(values[0], values[1])
            identity = str(point.get('id') or uuid.uuid4().hex[:12])
            if identity in ids or len(identity) > 64:
                raise ValueError('点位编号重复或无效')
            ids.add(identity)
            result.append({'id': identity, 'name': str(point.get('name') or f'点位 {i+1}')[:40],
                           'x': values[0], 'y': values[1],
                           'yaw': math.atan2(math.sin(values[2]), math.cos(values[2])),
                           'wait': values[3], 'enabled': bool(point.get('enabled', True))})
            if 'survey_path' in point:
                from survey_path import checked_path
                result[-1]['survey_path']=checked_path(self.grid,point['survey_path'],values[:2])
            if 'survey_pass_radius' in point:
                radius=float(point['survey_pass_radius'])
                if not math.isfinite(radius) or not .1<=radius<=.5:raise ValueError('采集引导范围无效')
                result[-1].update(survey_pass_radius=radius,survey_optional=bool(point.get('survey_optional',False)))
        return result

    def save_points(self, points):
        with self.lock:
            if self.phase not in QUIET:
                raise ValueError('请先停止当前任务，再修改点位')
            self.points = self.validate_points(points)
            data = json.dumps({'schema': 1, 'map_id': self.grid.map_id, 'points': self.points}, ensure_ascii=False, indent=2)
            temporary = self.directory / 'waypoints.tmp'
            temporary.write_text(data, encoding='utf-8');os.replace(temporary, self.directory/'waypoints.json')
            return self.points

    def require_ready(self):
        health = self.backend.snapshot()
        if not health['ready']:
            raise ValueError(health.get('readiness') or '导航或定位尚未就绪')
        if not health.get('localization_confirmed'):
            raise ValueError('请先核对小车位置、朝向和红色雷达，并确认定位正确')
        fleet=health.get('fleet')
        if fleet and fleet.get('active',True):
            if fleet['motion_reason']:
                raise ValueError(fleet['motion_reason'])
            if not fleet['leased']:
                raise ValueError('等待双车面板确认两车运行许可')

    def start(self, identity=None, auto_return=False, start_from=0, repeat=False, *, station=None):
        with self.lock:
            if self.phase not in QUIET:
                raise ValueError('已有任务运行，请先停止或暂停')
            self.require_ready()
            health=self.backend.snapshot()
            dock_status=health.get('dock_localization',{}).get('status')
            if health.get('charging_stable') and dock_status not in (None,'confirmed','manual'):
                raise ValueError('等待贴桩后的静止雷达定位复核')
            if auto_return and getattr(self.backend,'fleet_zone',None):
                raise ValueError('双车运行区内请关闭自动回充')
            self.endurance.cancel()
            self.health_pause=False;self.health_good_since=None
            if auto_return and not getattr(self.backend,'dock_calibrated',True):raise ValueError('先标定充电桩，再启用完成后回充')
            self.dock_retry_count=0;self.dock_retry_pending=False
            self.dock_failure_reconcile=False
            chosen = ([{**self.validate_points([station])[0],'fas_station':True}] if station is not None else
                      [p for p in self.points if (p['id'] == identity if identity else p['enabled'])])
            if start_from:
                if identity or self.collection.snapshot().get('profile')!='moving_loop_demo' or not isinstance(start_from,int) or not 0<=start_from<len(chosen):raise ValueError('无效续跑引导点')
                chosen=chosen[start_from:]
            if not chosen:
                raise ValueError('请至少选择一个导航点')
            if getattr(self.backend,'fleet_zone',None):
                reason=self.backend.fleet_motion_reason([(point['x'],point['y']) for point in chosen])
                if reason:
                    raise ValueError(reason)
            if any(p['id'].startswith('north-loop-') for p in chosen) and self.collection.snapshot().get('profile')!='moving_loop_demo':
                raise ValueError('快速回环引导点请使用“广播 RSSI 快速回环”模式，避免按普通精确停靠逐点执行')
            if any(p.get('survey_path') for p in chosen) and self.collection.snapshot().get('profile')!='moving_loop_demo':raise ValueError('连续采集路线仅用于广播采集模式')
            mode = self.collection.snapshot()['mode']
            if type(repeat) is not bool or (repeat and (identity or self.c5.enabled or mode not in ('timed',BROADCAST_STATION))):
                raise ValueError('循环巡航可用于定时停留与固定基站 RSSI 定点采集')
            if self.c5.enabled:
                self._require_motion_allowed()
                if self.c5.check() != self.c5.plan:
                    raise ValueError('C5 采集方案已变更，请重新应用采集设置')
                self.collection.archive_map()
                self.require_ready()
            self.collection.prepare(chosen)
            if self.c5.enabled:
                self.c5_session = self.collection.session if mode == SERVO_MODE else uuid.uuid4().hex
            if mode in BROADCAST_MODES:
                self.backend.survey_speed_limit=.12 if self.collection.snapshot().get('profile')=='moving_loop_demo' else .14
            if hasattr(self.backend,'set_survey_planning'):
                self.backend.set_survey_planning(self.collection.snapshot().get('profile')=='moving_loop_demo')
            self.run = [dict(p) for p in chosen];self.index = 0
            self.repeat = repeat;self.lap = 1
            self.auto_return = bool(auto_return) if identity is None else False
            self.pending = None;self.low_since = None
            label='开始循环巡航' if repeat else '开始巡航'
            self.event('开始单点前往' if identity else f'{label}，共 {len(chosen)} 个点')
            health=self.backend.snapshot()
            if (health.get('charging') or health.get('dock_position_confirmed')
                    or health.get('dock_departure_required')):
                self._depart()
            else:
                self._navigate_current()

    def _require_motion_allowed(self):
        self.collection.require_motion_allowed()
        if self.c5.active:
            raise ValueError('C5 正在采集或收尾，请停止任务并等待采集端结束')

    def _depart(self):
        self._require_motion_allowed()
        if not getattr(self.backend,'dock_calibrated',True):raise ValueError('先标定桩位，再从充电桩发车')
        distance = getattr(self.backend,'departure_distance',.30)
        point = departure_goal(self.backend.dock_pose,distance)
        self.operation = self.backend.navigate(point,'departure')
        self.phase = 'departing';self.message = '先直行驶离充电桩，再前往导航点'

    def _navigate_current(self):
        self._require_motion_allowed()
        self.save_checkpoint()
        point = self.run[self.index]
        collection = self.collection.snapshot()
        moving = collection.get('profile') == 'moving_loop_demo'
        if collection['mode'] == BROADCAST_STATION:
            point = {**point, 'rssi_station': True}
        self.operation = self.backend.navigate(point, 'survey' if moving else 'waypoint')
        self.phase = 'navigating';self.message = f'正在前往 {point["name"]}'

    def _begin_arrival(self, returning):
        if not returning:
            health=self.backend.snapshot();pose=health.get('pose')
            if pose is not None:
                point=self.run[self.index]
                timing=health.get('arrival') or {}
                record=dict(time=time.time(),map_id=self.grid.map_id,point_id=point['id'],
                    target=[point['x'],point['y'],point['yaw']],measured_map_pose=list(pose),
                    position_error_m=math.hypot(pose[0]-point['x'],pose[1]-point['y']),
                    yaw_error_deg=math.degrees(abs(math.atan2(math.sin(pose[2]-point['yaw']),math.cos(pose[2]-point['yaw'])))),
                    approach_seconds=timing.get('elapsed'),heading_attempts=timing.get('heading_attempts'),
                    yaw_tolerance_deg=timing.get('yaw_tolerance_deg'),position_tolerance_m=timing.get('position_tolerance_m'),
                    position_retries=timing.get('position_retries'),controller=timing.get('controller'),
                    travel_direction=timing.get('travel_direction'),heading_refinement_seconds=timing.get('heading_refinement_seconds'),
                    covariance=health.get('localization_covariance'),scan_map_agreement=health.get('scan_map_agreement'))
                if timing.get('last_turn'):
                    record.update(last_turn=timing['last_turn'],approach_aim=timing.get('approach_aim'))
                if timing.get('first_terminal_turn'):
                    record['first_terminal_turn']=timing['first_terminal_turn']
                with (self.directory/'arrival_measurements.jsonl').open('a',encoding='utf-8') as f:
                    f.write(json.dumps(record,ensure_ascii=False)+'\n')
        now = self.clock()
        if returning:
            if getattr(self.backend,'dock_handoff_ready',None) and not self._can_handoff_dock(self.backend.snapshot()):
                if self.phase != 'settling_return':
                    self.phase='settling_return';self.until=now+10.
                if now >= self.until:
                    self._fail('回充接管前未取得有效位姿和红外，保持停车')
                else:
                    self.message='桩前已停稳，等待位姿与红外复核'
                return
            self.backend.begin_docking();self.until = now+getattr(self.backend,'docking_timeout',90.)
            self.phase = 'docking';self.message = '桩前到达，等待红外对接及充电反馈'
        else:
            p = self.run[self.index]
            if self.collection.snapshot().get('profile')=='moving_loop_demo':
                self._advance();return
            mode = self.collection.snapshot()['mode']
            collect_ws63 = mode in ('ws63', SERVO_MODE, *BROADCAST_MODES)
            if collect_ws63 or self.c5.enabled:
                if mode == SERVO_MODE or self.c5.enabled:
                    self.backend.stop()
                health = self.backend.snapshot()
                visit_id = None
                if self.c5.enabled:
                    if not all(health.get(key) for key in ('ready', 'stationary', 'localization_confirmed')):
                        raise ValueError('到点停稳及定位确认之前不能启动 C5 采集')
                    visit_id = uuid.uuid4().hex
                    self.c5.begin({
                        'visit_id': visit_id, 'session_id': self.c5_session, 'map_id': self.grid.map_id,
                        'point_id': p['id'], 'point_name': p['name'], 'arrived_utc': utc(),
                        'actual_pose': health.get('pose'),
                    })
                if mode == BROADCAST_STATION:
                    self.phase='collecting';self.station_retry_at=now;self.station_started=False
                    self.message=f'{p["name"]} 已停稳，等待固定基站 RSSI 采集'
                    self.event(self.message)
                    return
                if collect_ws63:
                    if visit_id:
                        self.collection.begin(p, health, visit_id=visit_id)
                    else:
                        self.collection.begin(p, health)
                devices = 'WS63 和 C5' if collect_ws63 and visit_id else 'C5' if visit_id else 'WS63'
                self.phase='collecting';self.message=f'{p["name"]} 已停稳，等待 {devices} 本轮采集完成'
                self.event(self.message)
                return
            self.until = now+p['wait']
            self.phase = 'dwelling';self.message = f'{p["name"]} 已停稳，停留 {p["wait"]:g} 秒'
            self.event(self.message)

    def _can_handoff_dock(self, health):
        """Use IR only in the observed stopped capture area, never mid-route."""
        pose = health.get('pose');approach = self.backend.dock_approach
        if not (health.get('ready') and health.get('localization_confirmed')
                and health.get('stationary') and health.get('ir') and pose):
            return False
        if check := getattr(self.backend,'dock_handoff_ready',None):
            return check(health)
        angle = math.atan2(math.sin(pose[2]-self.backend.dock_pose[2]),
                           math.cos(pose[2]-self.backend.dock_pose[2]))
        home=self.backend.dock_pose;dx=pose[0]-home[0];dy=pose[1]-home[1]
        forward=dx*math.cos(home[2])+dy*math.sin(home[2])
        near_home=math.hypot(dx,dy)<=.15 and forward>=-.08
        return ((math.hypot(pose[0]-approach['x'],pose[1]-approach['y']) <= .10 or near_home)
                and abs(angle) <= math.radians(30))

    def configure_collection(self, mode, device='auto', c5=False):
        with self.lock:
            if self.phase not in QUIET: raise ValueError('请先停止当前任务，再切换采集模式')
            if type(c5) is not bool or (c5 and mode not in ('timed', SERVO_MODE)):
                raise ValueError('C5 可用于单独定点采集或与 SERVO_CAR_2 同站采集')
            if c5 and self.backend.snapshot().get('mode') == 'offline':
                raise ValueError('请从实车入口开启 C5 采集')
            if mode in ('ws63',SERVO_MODE,*BROADCAST_MODES) and self.backend.snapshot().get('mode')=='offline':
                raise ValueError('离线演练不能启动真实 WS63 采集，请使用实车入口')
            selection = self.c5.selection(c5)
            self.collection.configure(mode,device)
            self.c5.configure(*selection)
            labels = {'timed':'定时停留', 'ws63':'旧版 WS63 采集', 'broadcast':'广播沿途采集',
                      BROADCAST_STATION:'固定基站 RSSI 定点采集',
                      SERVO_MODE:'旋转定点采集（SERVO_CAR_2）'}
            label = 'C5 定点采集' if c5 and mode == 'timed' else labels[mode] + (' + C5' if c5 else '')
            self.event(f'切换到{label}')

    def _advance(self):
        self.index += 1
        if self.index>=len(self.run) and self.repeat:
            self.index=0;self.lap+=1
            self.event(f'开始第 {self.lap} 轮，继续同一组点位')
        if self.index>=len(self.run):self.clear_checkpoint()
        if self.index>=len(self.run) and self.endurance.pending:
            self.endurance.cancel();self._return();return
        if self.index < len(self.run) and self.endurance.pending:
            self.begin_recharge()
        elif self.index < len(self.run): self._navigate_current()
        elif self.auto_return: self._return()
        else:
            if self.collection.broadcast:self.collection.broadcast.finish_drive()
            self.phase='completed';self.message='所有导航点与停留任务已完成';self.event(self.message)
            self.clear_checkpoint()

    def save_checkpoint(self):
        if getattr(getattr(self.backend,'fas_lease',None),'owner',None):
            return
        data={'schema':1,'map_id':self.grid.map_id,'run':self.run,'index':self.index,
              'auto_return':self.auto_return,'collection_mode':self.collection.snapshot()['mode'],
              'repeat':self.repeat,'lap':self.lap}
        path=self.directory/'mission_checkpoint.json';temporary=path.with_suffix('.tmp')
        temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8');os.replace(temporary,path)

    def clear_checkpoint(self):
        if getattr(getattr(self.backend,'fas_lease',None),'owner',None):
            return
        (self.directory/'mission_checkpoint.json').unlink(missing_ok=True)

    def load_checkpoint(self):
        with self.lock:
            if self.phase not in QUIET:raise ValueError('请先停止当前任务')
            path=self.directory/'mission_checkpoint.json'
            if not path.exists():raise ValueError('没有未完成任务记录')
            saved=json.loads(path.read_text(encoding='utf-8'))
            if saved.get('map_id')!=self.grid.map_id:raise ValueError('任务地图版本不同，不能恢复点位')
            run=self.validate_points(saved['run']);index=saved['index']
            if not isinstance(index,int) or not 0<=index<len(run):raise ValueError('任务进度记录无效')
            self.save_points(run[index:]+run[:index] if saved.get('repeat') is True else run[index:])
            self.message='已载入剩余点位；核对定位和采集设置后点击开始，不自动发车'
            self.event(self.message)

    def begin_recharge(self):
        self.save_checkpoint()
        self.endurance.pending=False;self.endurance.resuming=True
        self.dock_retry_count=0;self.dock_retry_pending=False
        self.event('低电量回充，已保存下一待完成点位；充电稳定并满足实测恢复电压后继续')
        self.return_to_dock(automatic=True)

    def _low_voltage_return(self):
        if self.c5.enabled and self.phase in ('collecting','paused_collection'):
            self._fail('电压持续过低，已停止本轮定点采集');return
        if self.collection.snapshot().get('motion_blocked'):
            self._fail('电压持续过低，旋转定点采集已停止，请先恢复未放行的轮次');return
        self.event('电压持续过低，结束任务并回充');self.return_to_dock()

    def return_to_dock(self, automatic=False, via=None):
        with self.lock:
            if via is None and (self.phase in ('returning','settling_return','docking','canceling_collection') or
                               (self.phase=='canceling' and self.pending in ('return','dock_handoff','dock_contact'))):
                return
            if getattr(self.backend,'fleet_zone',None):
                raise ValueError('双车模式暂未配置独立回充区')
            if not automatic:self.endurance.cancel();self.repeat=False
            self._require_motion_allowed()
            if self.c5.enabled and self.phase in ('collecting','paused_collection'):
                raise ValueError('请先停止当前停靠任务，再主动回充')
            if not getattr(self.backend,'dock_calibrated',True):raise ValueError('当前场景尚未标定充电桩')
            if not automatic:
                self.dock_retry_count=0;self.dock_retry_pending=False
                self.dock_failure_reconcile=False
            self.return_via=via
            collection=self.collection.snapshot()
            if collection['mode']==BROADCAST_STATION and self.phase in ('collecting','paused_collection'):
                self.collection.cancel()
            elif (self.phase in ('collecting','paused_collection','canceling_collection')
                    or collection.get('pending') or collection['stage']=='canceling'):
                self.collection.cancel();self.backend.stop()
                self.phase='canceling_collection';self.message='等待 WS63 取消当前采集，再主动回充'
                return
            health=self.backend.snapshot()
            if health.get('stationary') and health.get('charging_stable',health.get('charging')):
                self._hold_dock_contact(health);return
            if health.get('charge_contact_live'):
                self._hold_dock_contact(health);return
            self.require_ready()
            if self.operation and self.backend.operation(self.operation)['status'] not in TERMINAL:
                self.backend.cancel(self.operation);self.pending = 'return'
                self.phase = 'canceling';self.message = '正在停止当前任务，随后回充'
            else:
                self._return()

    def retreat(self, *, fas=False):
        if self.phase not in QUIET or self.c5.active:
            raise ValueError('先结束当前任务，再短退')
        self.require_ready();self._require_motion_allowed()
        self.operation=self.backend.retreat(fas=True) if fas else self.backend.retreat()
        self.phase='retreating';self.message='低速直退，监测车尾和两侧'
        self.event(self.message)

    def _return(self):
        self._require_motion_allowed()
        if hasattr(self.backend,'set_survey_planning'):self.backend.set_survey_planning(False)
        if self.collection.broadcast:self.collection.broadcast.finish_drive()
        self.backend.survey_speed_limit=None
        health=self.backend.snapshot()
        if health.get('charging_stable',health.get('charging')) or health.get('charge_contact_live'):
            self._hold_dock_contact(health);return
        pose=health.get('pose')
        if (self.return_via is None and health.get('mode')=='robot' and pose
                and dock_footprint_intersects(pose,self.backend.dock_pose)):
            try:
                self.return_via=self._unpowered_dock_exit(health)
            except ValueError as error:
                self._fail(str(error));return
        if self.return_via is None and self._can_handoff_dock(health):
            self.operation=None;self.pending=None
            self.event('已在有效红外接管区域，直接使用厂家对接')
            self._begin_arrival(True)
            return
        if self.return_via is not None:
            clearing=self.return_via.get('dock_retry_exit',False)
            self.operation=self.backend.navigate(self.return_via,'departure' if clearing else 'waypoint')
            self.message='低速向前离开触点，再重新对接' if clearing else '先回到走廊中线，再前往充电桩'
        else:
            self.operation = self.backend.navigate(self.backend.dock_approach, 'return')
            self.message = '正在导航至充电桩前方'
        self.phase = 'returning'
        self.pending = None;self.event('开始主动回充')

    def pause(self, automatic=False):
        with self.lock:
            self.health_pause=automatic;self.health_good_since=None
            if not automatic:self.endurance.cancel()
            if self.phase == 'collecting':
                self.phase='paused_collection';self.message='巡航已暂停，当前站采集继续，完成后也不离站'
            elif self.phase == 'dwelling':
                self.remaining = max(0, self.until-self.clock());self.phase = 'paused_dwell'
                self.message = '停留计时已暂停';self.event(self.message)
            elif self.phase in ('navigating','returning','departing'):
                self.pending = 'pause_return' if self.phase == 'returning' else 'pause_departure' if self.phase == 'departing' else 'pause_navigation'
                self.backend.cancel(self.operation);self.phase = 'canceling';self.message = '正在停车'
            elif self.phase == 'docking':
                self.backend.stop();self.phase = 'paused_dock';self.message = '红外对接已暂停'
            elif self.phase in ('settling','settling_return','settling_departure'):
                self.phase = 'paused_'+self.phase;self.message = '停稳确认已暂停'

    def resume(self):
        with self.lock:
            self.require_ready()
            if self.phase == 'paused_collection':
                self.phase='collecting';self.message='继续等待采集确认'
            elif self.phase == 'paused_dwell':
                self.until = self.clock()+self.remaining;self.phase = 'dwelling';self.message = '继续停留计时'
            elif self.phase == 'paused_navigation':
                self._navigate_current()
            elif self.phase == 'paused_return':
                self._return()
            elif self.phase == 'paused_departure':
                self._depart()
            elif self.phase == 'paused_dock':
                self._return()
            elif self.phase in ('paused_settling','paused_settling_return','paused_settling_departure'):
                self.phase = self.phase[len('paused_'):];self.until = self.clock()+10
                self.message = '等待底盘停稳'
            else:
                raise ValueError('当前没有可以继续的暂停任务')

    def stop(self):
        with self.lock:
            self.return_via=None
            self.health_pause=False;self.repeat=False
            self.endurance.cancel();self.clear_checkpoint()
            if hasattr(self.backend,'set_survey_planning'):self.backend.set_survey_planning(False)
            self.dock_retry_pending=False;self.dock_failure_reconcile=False
            self.backend.survey_speed_limit=None
            self.c5.cancel()
            self.collection.cancel()
            self.backend.stop()
            if self.operation and self.backend.operation(self.operation)['status'] not in TERMINAL:
                self.backend.cancel(self.operation);self.phase = 'canceling';self.pending = 'stop'
                self.message = '停止中，等待导航确认取消'
            else:
                self.operation = None;self.phase = 'idle';self.pending = None;self.message = '任务已停止'
            self.event('停止任务')

    def _fail(self, message):
        self.return_via=None
        if self.phase!='docking':self.endurance.cancel()
        if hasattr(self.backend,'set_survey_planning'):self.backend.set_survey_planning(False)
        self.dock_failure_reconcile=self.phase=='docking'
        retryable=(self.phase=='docking' and any(t in message for t in (
                   '桩旁轮程已达到保护上限但未检测到充电','对接超时，未收到充电反馈',
                   '桩旁连续对接未取得充电反馈','原生对接达到行程或时间上限但未充电')))
        self.dock_retry_pending=retryable
        self.dock_retry_stationary_since=None
        if retryable:self.until=self.clock()+30.
        self.backend.survey_speed_limit=None
        self.c5.cancel()
        self.collection.cancel()
        self.backend.stop()
        if self.operation and self.backend.operation(self.operation)['status'] not in TERMINAL:
            self.backend.cancel(self.operation);self.pending = 'fail'
            self.phase = 'canceling'
        else:
            self.phase = 'failed';self.operation = None
        self.message = message;self.event(message)

    def _hold_dock_contact(self,health):
        self.backend.stop()
        self.return_via=None
        self.dock_retry_pending=True
        self.dock_retry_stationary_since=None
        self.dock_failure_reconcile=True
        self.until=self.clock()+30.
        if self.operation and self.backend.operation(self.operation)['status'] not in TERMINAL:
            self.backend.cancel(self.operation);self.pending='dock_contact';self.phase='canceling'
        else:
            self.operation=None;self.pending=None;self.phase='failed'
        self.message='检测到充电接触，停车复核电流'
        if self.phase=='failed':self._retry_docking_if_needed(health,self.clock())

    def _unpowered_dock_exit(self,health):
        pose=health.get('pose')
        if not (health.get('ready') and health.get('localization_confirmed') and health.get('stationary')
                and health.get('charge_telemetry_fresh') and pose and (health.get('scan_map_agreement') or 0.)>=.80):
            raise ValueError('未确认充电，停轮、定位或充电反馈不足，保持停车')
        dock=self.backend.dock_pose
        along,side=dock_offset(pose,dock)
        if along>.27:
            return None
        if along<-.16 or abs(side)>.12 or abs(angle_error(pose[2],dock[2]))>math.radians(15):
            raise ValueError('未通电，车辆姿态超出可安全离桩的范围，保持停稳')
        return {**departure_goal(dock,.34),'name':'回充重试离桩','wait':0.,'dock_retry_exit':True}

    def _retry_docking_if_needed(self,health,now):
        if self.phase in ('failed','idle') and self.dock_failure_reconcile and health.get('stationary') and health.get('charging_stable',health.get('charging')):
            self.dock_retry_pending=False;self.dock_failure_reconcile=False;self.phase='docked';self.message='停车复核已稳定充电，无需再次移动';self.event(self.message);return True
        if not self.dock_retry_pending or self.phase!='failed':return False
        if not health.get('stationary') or health.get('charge_contact_live'):
            if now>=self.until:
                self.dock_retry_pending=False
                self.message='触点或停轮状态仍待确认，保持停车'
                self.event(self.message)
            self.dock_retry_stationary_since=None;return True
        if self.dock_retry_stationary_since is None:self.dock_retry_stationary_since=now
        if now-self.dock_retry_stationary_since<DOCK_RECHECK_SECONDS:return True
        if self.dock_retry_count>=1:
            self.dock_retry_pending=False
            self.message='重新贴合后仍未确认通电，已停轮并保留任务'
            self.event(self.message)
            return True
        try:
            via=self._unpowered_dock_exit(health)
        except ValueError as error:
            self.dock_retry_pending=False
            self.message=str(error)
            self.event(self.message)
            return True
        self.dock_retry_pending=False;self.dock_retry_count+=1
        self.event('贴合后持续无电流，已停轮复核，重试回充（1/1）')
        self.return_to_dock(automatic=True,via=via);return True

    def tick(self):
        with self.lock:
            self.backend.tick()
            health = self.backend.snapshot();now = self.clock()
            contact=health.get('charge_contact_live') or health.get('charging_stable',health.get('charging'))
            if contact and self.phase in ('returning','settling_return'):
                self._hold_dock_contact(health)
                return
            charge_ready=self.endurance.observe(now,health,repeat=self.repeat)
            if self.phase=='docked' and charge_ready:
                collection=self.collection.snapshot()
                if collection['mode']==BROADCAST_STATION and not (collection['connected'] and collection.get('beacon_recent')):
                    self.message='已达到续采电压，等待 WS63 接收恢复';return
                self.endurance.cancel();self.low_since=None
                self.event('充电和桩位定位复核完成，从保存的点位继续任务')
                self._depart();return
            if self.health_pause and self.phase.startswith('paused_'):
                good=health.get('ready') and health.get('localization_confirmed') and health.get('stationary')
                self.health_good_since=(self.health_good_since if self.health_good_since is not None else now) if good else None
                if self.health_good_since is not None and now-self.health_good_since>=3.:
                    self.health_pause=False;self.resume();self.event('传感器与定位复核稳定，继续原任务')
                else:
                    self.message='任务已保留，等待传感器与定位复核稳定：'+health.get('readiness','')
                return
            if self._retry_docking_if_needed(health,now):return
            self.collection.observe({**health,'survey_active':self.phase in ('departing','navigating','settling_departure','settling')})
            if self.phase in QUIET or self.phase in ('returning','docking'):
                self.backend.survey_speed_limit=None
            if self.collection.snapshot()['mode'] in BROADCAST_MODES and self.phase=='navigating':
                pose=health.get('pose')
                from scene_settings import contains
                b=self.collection.broadcast
                if pose and b and not contains(b.bounds,pose,margin=.25):
                    self._fail('已超出北区采集边界，保持停车并保留数据，请核对路线');return
            calibrated=health.get('dock_localization',{}).get('calibrations',0)
            if calibrated>getattr(self,'dock_calibrations_seen',0):
                self.dock_calibrations_seen=calibrated
                text='贴桩停稳后已用雷达复核定位'
                self.event(text)
                if self.phase in QUIET and self.phase!='failed':
                    self.message=('正在充电；' if health.get('charging') else '')+text
            if self.phase in ('navigating','returning','departing','dwelling','collecting','paused_collection','settling','settling_return','settling_departure') and not health['ready']:
                if self.phase not in ('collecting','paused_collection'):
                    self.pause(automatic=True);self.event('传感器或定位异常，停车并保留当前点位：'+health['readiness']);return
                self._fail('定位或传感器未就绪，已停止：'+health['readiness']);return
            if (self.phase in ('navigating','departing','dwelling','collecting','paused_collection','settling','settling_departure')
                    and not getattr(getattr(self.backend,'fas_lease',None),'owner',None)):
                voltage = health.get('voltage')
                threshold=self.endurance.config.get('low_voltage',10.)
                hold=self.endurance.config.get('low_hold_seconds',20.)
                can_resume=self.endurance.allows_resume(self.repeat)
                self.low_since = (self.low_since if self.low_since is not None else now) if voltage is not None and voltage < threshold else None
                if self.low_since is not None and now-self.low_since >= hold:
                    if getattr(self.backend,'fleet_zone',None):
                        self.save_checkpoint();self._fail('电量过低，已保留点位；切换到单车模式后回充');return
                    if can_resume and self.phase=='collecting':
                        status=self.collection.snapshot()
                        if (status['mode']==BROADCAST_STATION
                                and (not self.station_started or status['stage'] not in ('sampling','done'))):
                            self.begin_recharge();return
                    if can_resume and self.phase not in ('collecting','paused_collection') and not self.collection.snapshot().get('motion_blocked'):
                        self.begin_recharge();return
                    if can_resume and self.phase in ('collecting','paused_collection'):
                        self.message='低电量，当前站保存并放行后回充，剩余任务已保留'
                        # Continue processing collection below; do not start a new station.
                        self.low_since=None
                    else:
                        self._low_voltage_return();return
            if self.phase=='retreating':
                result=self.backend.operation(self.operation)
                if not health['ready'] or not health['localization_confirmed']:
                    self._fail('短退期间定位或传感器异常，已停车')
                elif result['status']=='succeeded' and health.get('stationary'):
                    self.operation=None;self.phase='idle';self.message='已完成低速短退并停稳';self.event(self.message)
                elif result['status'] in ('failed','canceled'):
                    self._fail(result.get('error') or '短退未完成，已停车')
            elif self.phase == 'canceling':
                if self.operation and self.backend.operation(self.operation)['status'] not in TERMINAL:
                    return
                if health.get('mode')=='robot' and not health.get('stationary'):
                    self.message='取消已下发，等待底盘反馈停稳'
                    return
                action = self.pending;self.pending = None;self.operation = None
                if action == 'return':
                    if health['ready']:
                        self._return()
                    else:
                        self._fail('已停车，导航未就绪，未开始回充')
                elif action == 'dock_handoff':
                    if contact:
                        self._hold_dock_contact(health)
                    elif self._can_handoff_dock(health) or getattr(self.backend,'dock_handoff_ready',None):
                        self.event('导航取消已确认，复核红外接管条件')
                        self._begin_arrival(True)
                    else:
                        self._fail('桩前接管条件已改变，保持停车，请核对位置与红外')
                elif action in ('pause_navigation','pause_return','pause_departure'):
                    self.phase = 'paused_navigation' if action == 'pause_navigation' else 'paused_return' if action == 'pause_return' else 'paused_departure'
                    self.message = '已停车，可以继续';self.event(self.message)
                elif action == 'fail':
                    self.phase = 'failed'
                elif action == 'dock_contact':
                    self.phase='failed'
                    self._retry_docking_if_needed(health,now)
                else:
                    self.phase = 'idle';self.message = '任务已停止'
            elif self.phase in ('navigating','returning','departing'):
                result = self.backend.operation(self.operation)
                if self.phase=='departing' and result.get('detail'):
                    self.message=result['detail']
                if self.phase=='navigating':
                    name=self.run[self.index]['name'];stage=result.get('stage')
                    self.message=(f'{name}：'+(result.get('detail') or '等待通路恢复，随后自动继续') if stage=='waiting' else
                                  f'{name}：正在低速收敛位置与朝向' if stage=='terminal' else
                                  '正在对准短直线' if stage=='approach' else
                                  f'{name} 位置已到，正在对正朝向' if stage=='heading' else
                                  f'正在前往 {name}，跟踪可通行路线')
                if self.phase == 'returning' and self.return_via is None and self._can_handoff_dock(health):
                    if result['status'] not in TERMINAL:
                        self.backend.cancel(self.operation);self.pending = 'dock_handoff'
                        self.phase = 'canceling';self.message = '已到红外接管区域，停止导航并等待确认'
                    else:
                        self.operation = None;self.event('已停在桩前红外接管区域')
                        self._begin_arrival(True)
                    return
                if result['status'] not in TERMINAL:
                    return
                if result['status'] != 'succeeded':
                    self._fail(result.get('error') or '目标导航失败，已停车');return
                if (self.phase=='returning' and self.return_via and self.return_via.get('dock_retry_exit')
                        and not health.get('stationary')):
                    return
                if self.phase=='navigating' and result.get('quality')=='near':
                    self.event(f'{self.run[self.index]["name"]} 已在附近停稳，位置误差 {result["position_error"]*100:.1f} cm、朝向误差 {result["yaw_error_deg"]:.1f}°，按可用精度继续任务')
                self.operation = None
                if self.phase=='returning' and self.return_via is not None:
                    self.return_via=None;self._return();return
                if self.phase == 'departing':
                    if health.get('stationary',False):
                        if health.get('charging'):
                            self._fail('驶离桩后仍反馈充电，请核对是否真正移动');return
                        self._navigate_current()
                    else:
                        self.phase = 'settling_departure';self.until = now+10;self.message = '离桩后等待底盘停稳'
                    return
                returning = self.phase == 'returning'
                if health.get('stationary',False):
                    self._begin_arrival(returning)
                else:
                    self.phase = 'settling_return' if returning else 'settling'
                    self.until = now+10;self.message = '导航已到达，等待底盘停稳'
            elif self.phase in ('settling','settling_return','settling_departure'):
                if health.get('stationary',False):
                    if self.phase == 'settling_departure':
                        if health.get('charging'):
                            self._fail('驶离桩后仍反馈充电，请核对是否真正移动')
                        else:
                            self._navigate_current()
                    else:
                        self._begin_arrival(self.phase == 'settling_return')
                elif now >= self.until:
                    self._fail('已到达但未确认底盘停稳，任务已停止')
            elif self.phase == 'dwelling' and now >= self.until:
                p = self.run[self.index];self.event(f'{p["name"]} 停留结束')
                self._advance()
            elif self.phase in ('collecting','paused_collection'):
                status=self.collection.snapshot()
                if not health.get('stationary'):
                    self._fail('采集期间底盘未停稳，本轮已作废');return
                if (status['mode'] == SERVO_MODE or self.c5.enabled) and not health.get('localization_confirmed'):
                    self._fail('采集期间定位确认失效，保持停车');return
                if (status['mode']==BROADCAST_STATION and not (self.station_started and status['stage']=='done')
                        and (status['stage']!='error' or status.get('retryable'))):
                    if self.phase=='collecting' and status['stage']!='sampling':
                        self.station_started=False
                        if status['connected'] and status.get('beacon_recent') and now>=self.station_retry_at:
                            self.station_retry_at=now+2.
                            try:
                                self.collection.begin(self.run[self.index],health)
                            except ValueError as error:
                                self.message=str(error)
                            else:
                                self.station_started=True
                                self.message=f'{self.run[self.index]["name"]} 正在采集定点 RSSI'
                        elif not status.get('beacon_recent'):
                            self.message='已停稳，等待固定基站广播恢复后重采当前点'
                    return
                if status['stage']=='error':
                    self._fail('WS63 采集未完成：'+str(status['error']));return
                if self.c5.enabled:
                    c5 = self.c5.snapshot()
                    if c5['stage'] in ('error', 'canceled'):
                        self._fail(c5.get('error') or 'C5 采集未完成');return
                    if c5['stage'] != 'done':
                        return
                if self.phase != 'collecting' or (status['mode'] != 'timed' and status['stage'] != 'done'):
                    return
                if status['mode'] == SERVO_MODE and not self.collection.consume_servo_permission():
                    self._fail('本轮移动许可未能确认，保持停车');return
                if self.c5.enabled:
                    self.c5.release()
                device = 'WS63 和 C5' if self.c5.enabled and status['mode'] == SERVO_MODE else 'C5' if self.c5.enabled else 'WS63'
                self.event(f'{self.run[self.index]["name"]} {device} 采集完成')
                self._advance()
            elif self.phase=='canceling_collection':
                status=self.collection.snapshot()
                if status['stage'] in ('canceled','done'):
                    self._return()
                elif status['stage']=='error':
                    self._fail('串口取消尚未确认，保持停车：'+str(status['error']))
            elif self.phase == 'docking':
                if health.get('charging_stable' ,health.get('charging')):
                    self.backend.stop();self.phase = 'docked';self.message = '回充完成：底盘已反馈正在充电'
                    self.event(self.message)
                elif self.backend.docking_error:
                    self._fail(self.backend.docking_error)
                elif now >= self.until:
                    self._fail('对接超时，未收到充电反馈，已停止')
                elif health.get('docking_contact_stage')=='charging_observed':
                    self.message='检测到电流，厂家程序仍在完成贴合并等待停轮'
                elif health.get('docking_contact_stage')=='verify':
                    self.message='已检测充电接触，已停车，正在确认持续充电'
                elif health.get('docking_contact_stage')=='reseat':
                    self.message='触点未压实，正在由厂家红外程序短距离补贴合'
            elif self.phase=='docked' and health.get('charging_stable') is False:
                if not hasattr(self,'charge_lost_since'):self.charge_lost_since=now
                if now-self.charge_lost_since>=8:
                    del self.charge_lost_since
                    self.event('充电接触已中断，停稳复核后重试')
                    self._hold_dock_contact(health)
            elif hasattr(self,'charge_lost_since'):
                del self.charge_lost_since

    def state(self):
        with self.lock:
            health = self.backend.snapshot()
            remaining = max(0, self.until-self.clock()) if self.phase == 'dwelling' else self.remaining if self.phase == 'paused_dwell' else None
            return {'phase': self.phase, 'message': self.message, 'points': self.points,
                    'checkpoint_available':(self.directory/'mission_checkpoint.json').exists(),
                    'recharge_resume_pending':self.endurance.resuming,
                    'recharge_resume_enabled':bool(self.endurance.config.get('enabled')),
                    'recharge_resume_repeat_only':bool(self.endurance.config.get('repeat_only')),
                    'repeat':self.repeat,'lap':self.lap,
                    'run': self.run, 'index': self.index, 'remaining': remaining,
                    'auto_return': self.auto_return, 'health': health, 'collection':self.collection.snapshot(),
                    'c5': self.c5.snapshot(), 'dock': self.backend.dock_pose,
                    'events': list(self.events), 'map': self.grid.info()}
