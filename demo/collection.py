"""Optional acquisition coordinator. One serial owner, no ROS or motion commands.

The mission only consumes this object's correlated completion; all serial waits
and retries are on its worker thread. Default timed missions never open a port.
"""
from datetime import datetime, timezone
import glob
import json
import math
import os
from pathlib import Path
import queue
import shutil
import threading
import time
import uuid

from rdk_link import Link, Point, ProtocolError, DeviceReset


_ports_cache=(0.,[])
_ports_cache_lock=threading.Lock()

def ws63_ports():
    global _ports_cache
    with _ports_cache_lock:
        now=time.monotonic()
        if now-_ports_cache[0]<1.:return list(_ports_cache[1])
        ports=_scan_ws63_ports();_ports_cache=(now,ports)
        return list(ports)

def _scan_ws63_ports():
    """HiHope board previously identified and tested as CH340K 1a86:7522.

    Chassis/lidar are other USB serial chips. Never send AT probes to them.
    """
    try:
        from serial.tools import list_ports
        devices={os.path.realpath(p.device) for p in list_ports.comports()
                 if (p.vid,p.pid)==(0x1a86,0x7522)}
        return [p for p in glob.glob('/dev/serial/by-id/*') if os.path.realpath(p) in devices]
    except ImportError:
        return []


class Collection:
    def __init__(self, directory, grid, serial_factory=None):
        self.directory = Path(directory)/'collection'
        self.directory.mkdir(parents=True, exist_ok=True)
        self.grid = grid; self.factory = serial_factory
        self.lock = threading.RLock(); self.commands = queue.Queue()
        self.config = {'mode':'timed', 'device':'auto'}
        self.status = {'connected':False, 'stage':'disabled', 'error':None, 'visit':None}
        self.generation = 0; self.link = None; self.stream = None; self.closed = False
        self.record = None; self.session = None; self.numbers = {}
        self.broadcast = None
        # Always start in timed mode after a process restart. Never silently
        # resume a hardware scan whose arm/MCU state may have been lost.
        threading.Thread(target=self._worker, name='ws63-serial', daemon=True).start()

    def snapshot(self):
        if self.broadcast:
            return {**self.config,**self.broadcast.snapshot(),'ports':ws63_ports()}
        with self.lock:
            return {**self.config, **self.status, 'pending':bool(self.link and self.link.pending),
                    'ports':ws63_ports()}

    def _status(self, **values):
        with self.lock: self.status.update(values)

    def configure(self, mode, device='auto'):
        if mode not in ('timed','ws63','broadcast'): raise ValueError('未知停留模式')
        if device != 'auto' and not (device.startswith('/dev/serial/by-id/') or device.startswith('/dev/ttyUSB') or device.startswith('/dev/ttyACM')):
            raise ValueError('请选择 /dev/serial/by-id/ 下的 WS63 串口')
        if self.broadcast:
            if self.broadcast.snapshot()['stage']=='sampling':raise ValueError('请先停止采集再切换模式')
            self.broadcast.close();self.broadcast=None
        with self.lock:
            if self.status['stage'] in ('sampling','submitting','canceling') or (self.link and self.link.pending):
                raise ValueError('采集尚未结束，请先停止并确认取消')
            self.generation += 1
            self.config = {'mode':mode,'device':device}
            self.status = {'connected':False,'stage':'connecting' if mode in ('ws63','broadcast') else 'disabled','error':None,'visit':None}
            self.commands.put(('configure', self.generation, None))

    def prepare(self, points):
        if self.broadcast:return self.broadcast.prepare(points)
        if self.config['mode']=='broadcast':raise ValueError('广播接收组件尚未连接就绪，请等待握手和基站数据')
        with self.lock:
            if self.config['mode'] != 'ws63': return
            if not self.status['connected'] or self.status['stage'] not in ('ready','done'):
                raise ValueError(self.status['error'] or 'WS63 尚未连接就绪；不会以定时停留替代真实采集')
            self.session = uuid.uuid4().hex
            self.numbers = {p['id']:i+1 for i,p in enumerate(points)}
            saved_map=self.directory/'maps'/self.grid.map_id
            if not saved_map.exists():
                saved_map.parent.mkdir(parents=True,exist_ok=True)
                temporary=saved_map.with_name(saved_map.name+'.tmp-'+uuid.uuid4().hex)
                shutil.copytree(self.grid.directory,temporary)
                os.replace(temporary,saved_map)
            # Full hashes, map geometry and route are persisted before departure.
            self._log('session', session=self.session, map=self.grid.info(),
                      frame_id='map', base_frame='base_footprint', units={'position':'m','yaw':'rad'},
                      points=points, saved_map=str(saved_map.relative_to(self.directory)),
                      dock_reference='dock_start.json')

    def _log(self, event, **values):
        payload = {'time_utc':datetime.now(timezone.utc).isoformat(), 'event':event, **values}
        with self.lock:
            with (self.directory/'journal.jsonl').open('a',encoding='utf-8') as f:
                f.write(json.dumps(payload,ensure_ascii=False,allow_nan=False)+'\n'); f.flush();os.fsync(f.fileno())

    def export(self, packed=False):
        if self.broadcast:return self.broadcast.export()
        with self.lock:
            p=self.directory/'journal.jsonl';data=p.read_bytes() if p.exists() else b''
            if not packed: return data
            from collection_export import bundle
            return bundle(self.directory,data)

    def begin(self, target, health):
        if self.broadcast:return self.broadcast.begin(target,health)
        if self.config['mode']=='broadcast':raise ValueError('广播接收组件尚未就绪')
        pose = health.get('pose')
        if not health.get('stationary') or not health.get('localization_confirmed') or not health.get('ready'):
            raise ValueError('到点停稳及定位确认之前不能启动采集')
        if not pose or len(pose)!=3 or not all(math.isfinite(v) for v in pose):
            raise ValueError('缺少有效的实际定位，不能使用目标坐标代替')
        with self.lock:
            if not self.status['connected'] or self.status['stage'] not in ('ready','done'):
                raise ValueError(self.status['error'] or 'WS63 串口未就绪')
            tag = int(self.grid.map_id[:8],16) & 0x7fffffff
            wire = Point.from_pose(self.numbers[target['id']],tag,*pose)
            self.record = {'session':self.session,'visit_id':uuid.uuid4().hex,'point_id':target['id'],
                           'point_number':wire.point,'map_id':self.grid.map_id,'map_tag':tag,
                           'target_pose':[target[k] for k in ('x','y','yaw')], 'actual_pose':list(pose),
                           'frame_id':'map','base_frame':'base_footprint', 'started':time.monotonic()}
            self._log('arrival', **{k:v for k,v in self.record.items() if k!='started'})
            self._status(stage='submitting', error=None, elapsed=0)
            self.commands.put(('begin',self.generation,wire))

    def cancel(self):
        if self.broadcast:return self.broadcast.cancel()
        with self.lock:
            if self.status['stage'] in ('submitting','sampling','error'):
                self._status(stage='canceling')
                self.commands.put(('cancel',self.generation,None))

    def _line_log(self, line):
        # Save complete original measurement output; scan ↔ visit binding is a
        # separate event and never overwritten by the next car position.
        if line.startswith(('[scan_csv]', '+RDKBIND:', '+RDKQUALITY:')):
            self._log('device', session=self.session, line=line)
        if line.startswith('+RDKBIND:'):
            fields = tuple(map(int,line.split(':',1)[1].split(',')))
            if len(fields)==3 and self.link and fields[:2]==(self.link.run,self.link.visit):
                self._status(scan=fields[2])

    def _connect(self):
        if self.stream: self.stream.close()
        self.stream = None; self.link = None
        if self.config['mode']=='timed': return
        if self.config['mode']=='broadcast':
            from broadcast_survey import BroadcastSurvey
            self.broadcast=BroadcastSurvey(self.directory,self.grid,self.config['device'],self.factory)
            self.broadcast.moving_demo=True
            if hasattr(self,'anchor_metadata'):self.broadcast.anchor_metadata=dict(self.anchor_metadata)
            return
        device=self.config['device']
        if device=='auto':
            ports=ws63_ports()
            if len(ports)!=1:
                raise ValueError('未找到唯一的 HiHope WS63（CH340K）串口，请接好设备或明确选择')
            device=ports[0]
        factory=self.factory
        if factory is None:
            import serial
            if os.path.realpath(device) not in {os.path.realpath(p) for p in ws63_ports()}:
                raise ValueError('所选端口不是已验证的 WS63 CH340K；不会向底盘或雷达串口发送指令')
            factory=serial.Serial
        self.stream=factory(device,115200,timeout=.1,write_timeout=1,exclusive=True)
        if hasattr(self.stream,'reset_input_buffer'):
            time.sleep(.6);self.stream.reset_input_buffer()
        self.link=Link(self.stream)
        self.link.on_line=self._line_log
        hello=self.link.command('AT+RDKHELLO')
        if '+RDKCAP:2,SLE_SCAN_GATED' not in hello:
            raise ProtocolError('请烧录真实采集门控 Client；旧串口测试固件不能用于巡航采集')
        self.link.connect()
        self._status(connected=True,stage='ready',device_resolved=device,error=None,scan=None)

    def _worker(self):
        next_poll=0
        while not self.closed:
            try:
                try: action,generation,value=self.commands.get(timeout=.1)
                except queue.Empty: action=None;generation=self.generation
                if generation!=self.generation: continue
                if action=='configure': self._connect()
                elif action=='begin':
                    self.link.submit(value)
                    self._log('wire_arrival', session=self.session,visit_id=self.record['visit_id'],
                              run=self.link.run, visit=self.link.visit, fields=self.link.last_fields)
                    self._status(stage='sampling',visit=self.link.visit)
                elif action=='cancel':
                    if self.link and self.link.pending: self.link.cancel()
                    self._log('cancel',session=self.session)
                    self._status(stage='canceled',connected=False,error='本轮已取消；重新采集前核对机械臂并重新连接')
                if self.link and self.snapshot()['stage']=='sampling' and time.monotonic()>=next_poll:
                    next_poll=time.monotonic()+.4
                    run,visit,state,result,_=self.link.state()
                    if (run,visit)!=(self.link.run,self.link.visit): raise DeviceReset('采集会话消失，请核对 MCU 是否重启')
                    if state==2:
                        self.link.pending=False
                        if result: raise ProtocolError(f'采集结果无效（状态 {result}），不会前往下一站')
                        self._log('complete',session=self.session,visit_id=self.record['visit_id'],run=run,visit=visit,
                                  scan=self.snapshot().get('scan'),elapsed=time.monotonic()-self.record['started'])
                        self._status(stage='done')
                    elif state!=1:
                        if state==3: self.link.pending=False
                        raise ProtocolError('采集被取消或状态异常')
                    else: self._status(elapsed=round(time.monotonic()-self.record['started'],1))
            except Exception as exc:
                self._status(stage='error',connected=False,error=str(exc))
                self._log('error',session=self.session,detail=str(exc))
                # Keep ownership of the port so an explicit cancel can still be
                # attempted. No silent new session after an MCU reset/disconnect.

    def observe(self,health):
        if self.broadcast:self.broadcast.observe(health)

    def close(self):
        if self.broadcast:self.broadcast.close()
        self.cancel()
        deadline=time.monotonic()+2.5
        while self.snapshot()['stage']=='canceling' and time.monotonic()<deadline:
            time.sleep(.05)
        self.closed=True
