"""Opt-in vehicle side of the dual-radio experiment."""
import json
import math
from pathlib import Path
import re
import time

from fas_experiment import (CARS, MAX_FLEET_SIGMA, PROTOCOL, clear_of_parked_peer, fleet_pose_ready, mac,
                            peer_clearance, position_sigma, scan_confirmed, stopped_station_arrival)


LEASE_SECONDS = 1.0
SCAN_HOLD_SECONDS = 2.5
SCAN_UNSTABLE = '雷达或定位数据尚不稳定'
NOT_PARKED = '实验接管前两车须各自停稳、定位就绪并标好桩位'
PATH_KINDS = ('move','return','retreat')
WS63_IDS = {'A': '52:35:35:30:00:a1', 'B': '52:35:35:30:00:b1'}
WS63_CONNECTED_IDS = {'A': '11:22:33:44:55:61', 'B': '11:22:33:44:55:62'}


def obstruction_clear(health, path):
    blockage=health.get('fas_obstruction')
    if not blockage:
        return True
    if not path or math.dist(path[0][:2],blockage['anchor'][:2])<.12:
        return False
    return blockage['point'] is not None and clear_of_parked_peer(path,blockage['point'],.24)


class CorridorLease:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.owner = None
        self.recovery_enabled = False
        self.bounds = None
        self.parked_peer = None
        self.parked_peer_sigma = .07
        self.epoch = self.kind = None
        self.until = 0.
        self.scan_hold_since = None

    @property
    def live(self):
        return self.bounds is not None and self.clock() < self.until

    def reserve(self, epoch, kind, bounds, parked_peer=None, parked_peer_sigma=.07):
        if (kind not in PATH_KINDS or not re.fullmatch('[0-9a-f]{32}', epoch)
                or len(bounds) != 4 or not all(type(v) in (int,float) and math.isfinite(v) for v in bounds)
                or min(bounds[1]-bounds[0], bounds[3]-bounds[2]) < .95):
            raise ValueError('两车走廊预约无效')
        if parked_peer is not None and (not isinstance(parked_peer,list) or len(parked_peer)!=2
                or not all(type(value) in (int,float) and math.isfinite(value) for value in parked_peer)
                or type(parked_peer_sigma) not in (int,float) or not 0<=parked_peer_sigma<=MAX_FLEET_SIGMA):
            raise ValueError('待避让车辆的停车位置无效')
        self.epoch, self.kind, self.bounds = epoch, kind, tuple(bounds)
        self.parked_peer = tuple(parked_peer) if parked_peer is not None else None
        self.parked_peer_sigma = parked_peer_sigma
        self.until = self.clock() + LEASE_SECONDS
        self.scan_hold_since = None

    def renew(self, epoch):
        if not self.live or epoch != self.epoch:
            raise ValueError('行驶许可已过期，请停稳后重新预约')
        self.until = self.clock() + LEASE_SECONDS

    def revoke(self):
        self.until = 0.
        self.scan_hold_since = None

    def supervise(self, epoch, health, route, *, verifying_dock=False, docking=False):
        if not self.live or epoch != self.epoch:
            raise ValueError('行驶许可已过期，请停稳后重新预约')
        reason = None if verifying_dock else self.motion_error(self.kind,health,route)
        if self.kind in ('move','return') and not docking and reason == SCAN_UNSTABLE:
            # The velocity gate stays closed. A fresh supervisor heartbeat can
            # keep the reserved corridor while the observation recovers.
            now=self.clock()
            if self.scan_hold_since is None:
                self.scan_hold_since=now
            if now-self.scan_hold_since<SCAN_HOLD_SECONDS:
                self.renew(epoch)
                return
        if reason:
            self.revoke()
            raise ValueError(f'{reason}，已收回行驶许可')
        self.scan_hold_since=None
        self.renew(epoch)

    def allows(self, kind, health, route=()):
        return self.motion_error(kind,health,route) is None

    def motion_error(self, kind, health, route=()):
        if self.owner is None:
            return None
        if kind != self.kind or not self.live:
            return '走廊预约已失效'
        if not health['ready'] or not health['localization_confirmed'] or health.get('fas_localization_active'):
            return '定位尚未确认'
        pose = health.get('pose')
        age = health.get('pose_age_s')
        sigma = position_sigma(health)
        rotation=kind=='move' and health.get('fas_local_rotation') is True
        docking=kind=='return' and (health.get('fas_dock_staging') is True or health.get('fas_local_docking') is True)
        if (not pose or age is None or not 0. <= age <= (.70 if rotation else .45)
                or not (rotation or docking or scan_confirmed(health))):
            return SCAN_UNSTABLE
        if sigma is None or sigma > MAX_FLEET_SIGMA:
            return '定位不确定性超过走廊预约上限'
        if self.parked_peer is not None:
            clearance=peer_clearance(sigma,self.parked_peer_sigma)
            if (not clear_of_parked_peer([pose],self.parked_peer,clearance)
                    or (route and not clear_of_parked_peer(route,self.parked_peer,clearance))):
                return '实际路线与停车车辆的净空不足'
        if kind!='retreat' and route and not obstruction_clear(health,route):
            return '路线仍进入空转位置'
        # Include the circumscribed footprint, braking and pose uncertainty.
        margin = .29 + 2*sigma
        xmin,xmax,ymin,ymax = self.bounds
        if not all(xmin+margin <= p[0] <= xmax-margin and ymin+margin <= p[1] <= ymax-margin
                   for p in (pose,*route)):
            return '实际路线的定位余量超出预约走廊'
        return None


class Ws63Transmitter:
    def __init__(self, device, car):
        self.device = Path(device)
        self.car = car
        self.serial = None

    def open(self):
        if self.serial is not None:
            return self.serial
        import serial
        port = serial.Serial(port=None, baudrate=115200, timeout=.08, write_timeout=.25, exclusive=True)
        port.port = str(self.device)
        port.dtr = port.rts = False
        port.open()
        self.serial = port
        return port

    def send(self, name, expected='OK'):
        port = self.open()
        try:
            port.reset_input_buffer()
            port.write(f'AT+{name}\r\n'.encode())
            deadline = time.monotonic()+.5
            while time.monotonic() < deadline:
                line = port.readline(256).decode('ascii',errors='ignore').strip()
                if line == expected or (expected.startswith('+') and line.startswith(expected)):
                    return line
                if line == 'ERROR':
                    break
        except (OSError, ValueError):
            port.close()
            self.serial = None
            raise
        port.close()
        self.serial = None
        raise OSError(f'WS63 串口没有确认 {name}')

    def probe(self):
        version,car,address,ready,active = self.send('FASHELLO','+FASHELLO:')[10:].split(',')
        if (version,car,mac(address),ready) != ('1',self.car,WS63_IDS[self.car],'1') or active not in ('0','1'):
            raise ValueError('WS63 固件角色、地址或就绪状态和车号不符')
        return active == '1'

    def set(self, enabled):
        if enabled:
            self.send('FASON')
        else:
            self.send('FASOFF')

    def close(self):
        if self.serial is not None:
            self.serial.close()
            self.serial = None


class FasCar:
    def __init__(self, backend, mission, config_path):
        self.backend, self.mission = backend, mission
        settings = json.loads(config_path.read_text(encoding='utf-8'))
        self.car = settings['car']
        self.map_id = mission.grid.map_id
        if self.car not in CARS or settings['map_id'] != self.map_id:
            raise ValueError('FAS 车号或地图版本错误')
        self.motion_only = settings.get('motion_only',False)
        if type(self.motion_only) is not bool:
            raise ValueError('仅行驶模式必须明确启用或关闭')
        self.tx = self.c5 = None
        self.ws63_device = None
        self.ws63_mode = settings.get('ws63_mode','broadcast')
        if self.ws63_mode not in ('broadcast','connected'):
            raise ValueError('WS63 模式须为 broadcast 或 connected')
        self.links = {}
        if not self.motion_only:
            sle, c5 = settings['ws63_device'], settings['c5_device']
            if (sle == c5 or not all(re.fullmatch(r'/dev/serial/by-(id|path)/[^/]+', port) for port in (sle,c5))):
                raise ValueError('WS63 和 C5 需分别配置稳定的 USB 串口路径')
            self.ws63_device,self.c5 = Path(sle),Path(c5)
            if self.ws63_mode=='broadcast':
                self.tx = Ws63Transmitter(sle,self.car)
            addresses = WS63_CONNECTED_IDS if self.ws63_mode=='connected' else WS63_IDS
            self.links = {'sle':addresses[self.car],'c5':mac(settings['c5_mac'])}
        self.claim = config_path.with_name('fas_robot_claim.json')
        self.pose_checkpoint = config_path.with_name('fas_robot_pose.json')
        self.pose_checkpoint_at = 0.
        self.lease = CorridorLease()
        self.lease.recovery_enabled = self.motion_only
        self.air_until = 0.
        self.command_id = None
        self.operator_hold = False
        if self.claim.exists():
            saved = json.loads(self.claim.read_text(encoding='utf-8'))
            if saved['map_id'] != self.map_id or saved.get('motion_only',False)!=self.motion_only:
                raise ValueError('需在原地图与原试验模式下停稳并释放 FAS 任务')
            self.lease.owner = saved['owner']
            self.operator_hold = saved.get('paused',False)
        self.backend.fas_lease = self.lease
        if self.motion_only and self.pose_checkpoint.is_file():
            try:
                saved=json.loads(self.pose_checkpoint.read_text(encoding='utf-8'))
                pose,stamp=saved['pose'],float(saved['saved_at'])
                if (saved['map_id']==self.map_id and len(pose)==3
                        and all(type(v) in (float,int) and math.isfinite(v) for v in pose)
                        and 0.<=time.time()-stamp<120.):
                    self.backend.fas_saved_pose=(tuple(pose),stamp)
            except (OSError,ValueError,TypeError,KeyError):
                pass  # A missing or damaged checkpoint grants no motion.

    def state(self):
        recovery=getattr(self.backend,'fas_localization_state',None)
        blocked=getattr(self.backend,'fas_obstruction',None)
        return {'protocol':PROTOCOL,'car':self.car,'owner':self.lease.owner,
                'lease_epoch':self.lease.epoch,'lease_live':self.lease.live,
                'scan_hold':self.lease.live and self.lease.scan_hold_since is not None,
                'command_id':self.command_id,'operator_hold':self.operator_hold,
                'motion_only':self.motion_only,
                'usb':self.usb_ready(),
                'ws63_mode':self.ws63_mode,
                'tx':self.links,'energy':dict(self.mission.endurance.config),
                'dock_retry_pending':self.mission.dock_retry_pending,
                'recovery':recovery() if recovery else None,
                'retreat_available':bool(blocked and blocked['direction']=='front' and not blocked['attempted'])}

    def usb_ready(self):
        try:
            return (self.ws63_device is not None and self.ws63_device.is_char_device()
                    and self.c5.is_char_device() and not self.ws63_device.samefile(self.c5))
        except OSError:
            return False

    def require_parked(self, *, localization=True):
        from mission import QUIET
        health = self.backend.snapshot()
        if (self.mission.phase not in QUIET or not health['stationary'] or not health['dock_calibrated']
                or health.get('map_matches') is False or (localization and not fleet_pose_ready(health))
                or self.mission.endurance.resuming or self.mission.dock_retry_pending
                or (health.get('fleet') or {}).get('active')):
            raise ValueError(NOT_PARKED)
        return health

    def disarm_radio(self):
        if self.tx is None:
            return
        if self.air_until > time.monotonic() or self.tx.probe():
            self.tx.set(False)
            for _ in range(6):
                time.sleep(.08)
                if not self.tx.probe():
                    break
            else:
                raise OSError('WS63 停发尚未确认')
        self.air_until = 0.

    def tick(self):
        from mission import QUIET
        if self.lease.owner and not self.lease.live and self.mission.phase not in (*QUIET,'canceling'):
            checking_stop=(self.lease.kind=='move' and time.monotonic()-self.lease.until<3.
                           and self.mission.phase in ('navigating','settling','dwelling')
                           and stopped_station_arrival(self.backend.snapshot()))
            if not checking_stop:
                self.mission.stop()
        now=time.monotonic()
        if (self.motion_only and not self.lease.live and self.mission.phase in QUIET
                and now-self.pose_checkpoint_at>=5.):
            self.pose_checkpoint_at=now
            h=self.backend.snapshot()
            if h['stationary'] and fleet_pose_ready(h):
                try:
                    pending=self.pose_checkpoint.with_suffix('.tmp')
                    pending.write_text(json.dumps({'map_id':self.map_id,'pose':h['pose'],'saved_at':time.time()}),
                                       encoding='utf-8')
                    pending.replace(self.pose_checkpoint)
                except OSError:
                    pass  # Live safety does not depend on a checkpoint being writable.

    def save_claim(self, owner):
        pending = self.claim.with_suffix('.tmp')
        pending.write_text(json.dumps({'owner':owner,'map_id':self.map_id,'paused':self.operator_hold,
                                       'motion_only':self.motion_only}),encoding='utf-8')
        pending.replace(self.claim)

    def operator_stop(self):
        with self.backend.lock:
            self.lease.revoke()
            self.operator_hold = True
            self.cancel_relocalization()
        self.mission.stop()
        self.save_claim(self.lease.owner)

    def cancel_relocalization(self):
        cancel=getattr(self.backend,'cancel_fas_localization',None)
        if cancel:
            cancel()

    def command(self, action, data):
        owner = data['run_id']
        if not isinstance(owner,str) or not re.fullmatch('[0-9a-f]{32}',owner) or data.get('map_id') != self.map_id:
            raise ValueError('实验会话或地图无效')
        if action == 'fas_claim' and self.lease.owner is None:
            self.require_parked(localization=not self.motion_only)
            if self.mission.collection.snapshot()['mode'] != 'timed' or self.mission.c5.enabled:
                raise ValueError('请将车端旧采集模式切回定时')
            if not self.motion_only and not self.state()['usb']:
                raise ValueError('车载 WS63 / C5 未同时枚举')
            self.disarm_radio()
            with self.backend.lock:
                self.save_claim(owner)
                self.lease.owner = owner
        if self.lease.owner != owner:
            raise ValueError('本车未由该实验接管')
        if action == 'fas_claim':
            return {}
        if self.motion_only and action in ('fas_probe','fas_radio'):
            raise ValueError('仅行驶试验禁止操作射频设备')
        if action == 'fas_probe':
            self.require_parked()
            if not self.usb_ready():
                raise ValueError('两块车载射频板须分别枚举为不同的 USB 串口')
            if self.tx:
                self.tx.probe()
            return {}
        if action == 'fas_stop':
            with self.backend.lock:
                self.lease.revoke()
                self.cancel_relocalization()
            self.mission.stop()
            return {}
        if action == 'fas_resume':
            self.require_parked(localization=not self.motion_only)
            self.operator_hold = False
            self.save_claim(owner)
            return {}
        if self.operator_hold and not (action == 'fas_release' or
                                       (action == 'fas_radio' and data.get('enabled') is False)):
            raise ValueError('操作员已停车，需显式恢复实验')
        if action == 'fas_relocalize':
            self.require_parked(localization=False)
            relocalize=getattr(self.backend,'start_fas_localization',None)
            if self.lease.live or not self.motion_only or not relocalize:
                raise ValueError('静止定位复核需要撤销行驶许可')
            relocalize(owner)
            return {}
        if action == 'fas_renew':
            with self.backend.lock:
                health=self.backend.snapshot()
                verifying_dock=(self.lease.kind=='return' and health['stationary'] and
                    (self.mission.phase=='docked' or self.mission.dock_retry_pending))
                self.lease.supervise(data['epoch'],health,health.get('path',[]),verifying_dock=verifying_dock,
                                     docking=getattr(self.backend,'docking_since',None) is not None)
            return {}
        if action == 'fas_radio':
            self.require_parked()
            if type(data['enabled']) is not bool or (data['enabled'] and self.lease.live):
                raise ValueError('只有停车采样窗口可以开启发端')
            if data['enabled']:
                if not self.usb_ready():
                    raise ValueError('车载射频 USB 枚举已丢失或重复')
                if self.tx:
                    self.tx.set(True)
                    self.air_until = time.monotonic()+5.5
            else:
                self.disarm_radio()
            return {}
        if action == 'fas_release':
            from mission import QUIET
            if self.mission.phase not in QUIET or not self.backend.snapshot()['stationary']:
                raise ValueError('两车停稳后方可清理实验接管')
            if self.tx and self.tx.device.exists():
                try:
                    self.disarm_radio()
                except (OSError, ValueError):
                    if time.monotonic() < self.air_until:
                        raise
            if self.tx:
                self.tx.close()
            with self.backend.lock:
                self.lease.revoke()
                self.cancel_relocalization()
                self.claim.unlink(missing_ok=True)
                self.lease.owner = None
                self.operator_hold = False
            return {}
        health = self.require_parked()
        kind = data['kind']
        if kind not in PATH_KINDS or (kind=='retreat' and not self.motion_only):
            raise ValueError('预约类型无效')
        if action == 'fas_preview':
            if kind=='retreat':
                return {'navigation_path':self.backend.retreat_path(fas=True)}
            goal=(self.backend.dock_approach if kind=='return' else self.mission.validate_points([data['point']])[0])
            path = self.backend.preview_route(goal, kind, health)
            if not obstruction_clear(health,path):
                raise ValueError('当前没有可走的路线，避开空转位置')
            return {'navigation_path':path}
        if action == 'fas_reserve':
            self.disarm_radio()
            with self.backend.lock:
                self.lease.reserve(data['epoch'],kind,data['bounds'],data.get('parked_peer'),data.get('parked_peer_sigma',.07))
                reason=self.lease.motion_error(kind,self.backend.snapshot())
                if reason:
                    self.lease.revoke()
                    raise ValueError(f'{reason}，车辆预约未通过车端确认')
            return {}
        if action == 'fas_drive':
            if self.command_id == data['epoch']:
                return {}
            with self.backend.lock:
                if self.lease.epoch != data['epoch'] or not self.lease.allows(kind,self.backend.snapshot()):
                    raise ValueError('当前行驶许可无效')
            if kind=='retreat':
                self.mission.retreat(fas=True)
            elif kind == 'move':
                self.mission.start(station=data['point'])
            else:
                self.mission.dock_retry_count=0
                self.mission.return_to_dock(automatic=True)
            self.command_id = data['epoch']
            return {}
        raise ValueError('未知实验操作')
