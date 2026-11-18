"""Opt-in vehicle side of the dual-radio experiment."""
import json
import math
from pathlib import Path
import re
import time

from fas_experiment import CARS, PROTOCOL, mac


LEASE_SECONDS = 1.0
WS63_IDS = {'A': '52:35:35:30:00:a1', 'B': '52:35:35:30:00:b1'}


class CorridorLease:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.owner = None
        self.bounds = None
        self.epoch = self.kind = None
        self.until = 0.

    @property
    def live(self):
        return self.bounds is not None and self.clock() < self.until

    def reserve(self, epoch, kind, bounds):
        if (kind not in ('move','return') or not re.fullmatch('[0-9a-f]{32}', epoch)
                or len(bounds) != 4 or not all(type(v) in (int,float) and math.isfinite(v) for v in bounds)
                or min(bounds[1]-bounds[0], bounds[3]-bounds[2]) < .95):
            raise ValueError('两车走廊预约无效')
        self.epoch, self.kind, self.bounds = epoch, kind, tuple(bounds)
        self.until = self.clock() + LEASE_SECONDS

    def renew(self, epoch):
        if not self.live or epoch != self.epoch:
            raise ValueError('行驶许可已过期，请停稳后重新预约')
        self.until = self.clock() + LEASE_SECONDS

    def revoke(self):
        self.until = 0.

    def allows(self, kind, health, route=()):
        if self.owner is None:
            return True
        if kind != self.kind or not self.live or not health['ready'] or not health['localization_confirmed']:
            return False
        pose = health.get('pose')
        cov = health.get('localization_covariance')
        score = health.get('scan_map_agreement')
        age = health.get('pose_age_s')
        if (not pose or age is None or not 0. <= age <= .45 or not cov or len(cov)<2
                or any(v is None or not math.isfinite(v) or v < 0 for v in cov[:2])
                or score is None or score < .80):
            return False
        sigma = math.sqrt(max(cov[:2]))
        if sigma > .07:
            return False
        # Include the circumscribed footprint, braking and pose uncertainty.
        margin = .29 + 2*sigma
        xmin,xmax,ymin,ymax = self.bounds
        return all(xmin+margin <= p[0] <= xmax-margin and ymin+margin <= p[1] <= ymax-margin
                   for p in (pose,*route))


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
        sle, c5 = settings['ws63_device'], settings['c5_device']
        if (sle == c5 or not all(re.fullmatch(r'/dev/serial/by-(id|path)/[^/]+', port) for port in (sle,c5))):
            raise ValueError('WS63 和 C5 需分别配置稳定的 USB 串口路径')
        self.tx = Ws63Transmitter(sle,self.car)
        self.c5 = Path(c5)
        self.c5_mac = mac(settings['c5_mac'])
        self.claim = config_path.with_name('fas_robot_claim.json')
        self.lease = CorridorLease()
        self.air_until = 0.
        self.command_id = None
        self.operator_hold = False
        if self.claim.exists():
            saved = json.loads(self.claim.read_text(encoding='utf-8'))
            if saved['map_id'] != self.map_id:
                raise ValueError('需在原地图停稳后释放 FAS 任务')
            self.lease.owner = saved['owner']
            self.operator_hold = saved.get('paused',False)
        self.backend.fas_lease = self.lease

    def state(self):
        return {'protocol':PROTOCOL,'car':self.car,'owner':self.lease.owner,
                'lease_epoch':self.lease.epoch,'lease_live':self.lease.live,
                'command_id':self.command_id,'operator_hold':self.operator_hold,
                'usb':self.tx.device.exists() and self.c5.exists(),
                'tx':{'sle':WS63_IDS[self.car],'c5':self.c5_mac},'energy':dict(self.mission.endurance.config),
                'dock_retry_pending':self.mission.dock_retry_pending}

    def require_parked(self):
        from mission import QUIET
        health = self.backend.snapshot()
        if (self.mission.phase not in QUIET or not health['stationary'] or not health['ready']
                or not health['localization_confirmed'] or not health['dock_calibrated']
                or health.get('pose_age_s',1.) > .45 or (health.get('scan_map_agreement') or 0.) < .80
                or self.mission.endurance.resuming or self.mission.dock_retry_pending
                or (health.get('fleet') or {}).get('active')):
            raise ValueError('实验接管前两车须各自停稳、定位就绪并标好桩位')
        return health

    def disarm_radio(self):
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
            self.mission.stop()

    def save_claim(self, owner):
        pending = self.claim.with_suffix('.tmp')
        pending.write_text(json.dumps({'owner':owner,'map_id':self.map_id,'paused':self.operator_hold}),encoding='utf-8')
        pending.replace(self.claim)

    def operator_stop(self):
        with self.backend.lock:
            self.lease.revoke()
            self.operator_hold = True
        self.mission.stop()
        self.save_claim(self.lease.owner)

    def command(self, action, data):
        owner = data['run_id']
        if not isinstance(owner,str) or not re.fullmatch('[0-9a-f]{32}',owner) or data.get('map_id') != self.map_id:
            raise ValueError('实验会话或地图无效')
        if action == 'fas_claim' and self.lease.owner is None:
            self.require_parked()
            if self.mission.collection.snapshot()['mode'] != 'timed' or self.mission.c5.enabled:
                raise ValueError('请将车端旧采集模式切回定时')
            if not self.state()['usb']:
                raise ValueError('车载 WS63 / C5 未同时枚举')
            self.disarm_radio()
            with self.backend.lock:
                self.save_claim(owner)
                self.lease.owner = owner
        if self.lease.owner != owner:
            raise ValueError('本车未由该实验接管')
        if action == 'fas_claim':
            return {}
        if action == 'fas_probe':
            self.require_parked()
            self.tx.probe()
            if not self.c5.exists():
                raise ValueError('车载 C5 USB 未枚举')
            return {}
        if action == 'fas_stop':
            with self.backend.lock:
                self.lease.revoke()
            self.mission.stop()
            return {}
        if action == 'fas_resume':
            self.require_parked()
            self.operator_hold = False
            self.save_claim(owner)
            return {}
        if self.operator_hold and not (action == 'fas_release' or
                                       (action == 'fas_radio' and data.get('enabled') is False)):
            raise ValueError('操作员已停车，需显式恢复实验')
        if action == 'fas_renew':
            with self.backend.lock:
                self.lease.renew(data['epoch'])
            return {}
        if action == 'fas_radio':
            self.require_parked()
            if type(data['enabled']) is not bool or (data['enabled'] and self.lease.live):
                raise ValueError('只有停车采样窗口可以开启发端')
            if data['enabled']:
                if not self.c5.exists():
                    raise ValueError('车载 C5 USB 枚举已丢失')
                self.tx.set(True)
                self.air_until = time.monotonic()+5.5
            else:
                self.disarm_radio()
            return {}
        if action == 'fas_release':
            from mission import QUIET
            if self.mission.phase not in QUIET or not self.backend.snapshot()['stationary']:
                raise ValueError('两车停稳后方可清理实验接管')
            if self.tx.device.exists():
                try:
                    self.disarm_radio()
                except (OSError, ValueError):
                    if time.monotonic() < self.air_until:
                        raise
            self.tx.close()
            with self.backend.lock:
                self.lease.revoke()
                self.claim.unlink(missing_ok=True)
                self.lease.owner = None
                self.operator_hold = False
            return {}
        health = self.require_parked()
        kind = data['kind']
        if kind not in ('move','return'):
            raise ValueError('预约类型无效')
        if action == 'fas_preview':
            if kind == 'return':
                goal = self.backend.dock_approach
            else:
                goal = self.mission.validate_points([data['point']])[0]
            path = self.backend.preview_route(goal, kind, health)
            return {'navigation_path':path}
        if action == 'fas_reserve':
            self.disarm_radio()
            with self.backend.lock:
                self.lease.reserve(data['epoch'],kind,data['bounds'])
                if not self.lease.allows(kind,self.backend.snapshot()):
                    self.lease.revoke()
                    raise ValueError('车辆定位或走廊预约未通过车端确认')
            return {}
        if action == 'fas_drive':
            if self.command_id == data['epoch']:
                return {}
            with self.backend.lock:
                if self.lease.epoch != data['epoch'] or not self.lease.allows(kind,self.backend.snapshot()):
                    raise ValueError('当前行驶许可无效')
            if kind == 'move':
                self.mission.start(station=data['point'])
            else:
                self.mission.return_to_dock(automatic=True)
            self.command_id = data['epoch']
            return {}
        raise ValueError('未知实验操作')
