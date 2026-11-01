"""Broadcast RSSI survey; no motion commands and no SLE connection.

Serial has one owner. Quick-demo samples use timestamped localized motion;
the optional stationary baseline retains strict dwell windows. No interpolation.
"""
import base64
from collections import deque
import csv
from datetime import datetime, timezone
import io
import json
import math
import os
from pathlib import Path
import shutil
import statistics
import threading
import time
import uuid
import zipfile


from device_clock import DeviceClock
from scene_settings import settings, contains

class BroadcastSurvey:
    minimum_seconds=6.
    minimum_samples=40
    maximum_seconds=20.

    def __init__(self,directory,grid,device,factory=None,clock=time.monotonic,autostart=True):
        self.directory=Path(directory)/'broadcast';self.directory.mkdir(parents=True,exist_ok=True)
        self.scene=settings(grid);self.bounds=self.scene['bounds'];self.bin_origin=self.scene['bin_origin']
        self.grid=grid;self.device=device;self.factory=factory;self.clock=clock
        self.lock=threading.RLock();self.history=deque(maxlen=120)
        self.stream=None;self.closed=False;self.session=None;self.visit=None;self.raw_file=None
        self.points=[];self.buffer=bytearray();self.offset=None;self.last_us=None;self.last_seq=0
        self.last_packet=0;self.last_status=0;self.drops=0;self.invalid=0;self.hello=False
        self.status={'connected':False,'stage':'connecting','error':None,'visit':None,'samples':0}
        self.worker=None
        self.moving_demo=False;self.drive_active=False;self.drive_requested=False;self.bins={};self.saved_at=0;self.inherit_session=None
        self.reconnects=0;self.epoch=0;self.last_report_us=None;self.report_times=deque(maxlen=200);self.device_clock=DeviceClock()
        if autostart:
            self.worker=threading.Thread(target=self._worker,name='broadcast-uart',daemon=True);self.worker.start()

    def snapshot(self):
        with self.lock:return {**self.status,**self.device_clock.summary(),'mode':'broadcast','pending':False,
                               'last_rssi':getattr(self,'last_rssi',None),'queue_drops':self.drops,
                               'invalid_reports':self.invalid,'measured_points':len(self.points),
                               'profile':'moving_loop_demo' if self.moving_demo else 'stationary',
                               'reconnects':self.reconnects,'radio_epoch':self.epoch,
                               'reports_last_5s':sum(self.clock()-t<5 for t in self.report_times),
                               'beacon_recent':bool(self.last_packet and self.clock()-self.last_packet<2.5)}

    def observe(self,health):
        with self.lock:
            now=self.clock()
            if self.history and now-self.history[-1]['t']<.08:return
            self.history.append({'t':now,'pose':health.get('pose'),
                'valid':bool(health.get('ready') and health.get('localization_confirmed')),
                'stationary':bool(health.get('stationary')),
                'survey_active':bool(health.get('survey_active',False)),
                'covariance':health.get('localization_covariance')})

    def prepare(self,points):
        with self.lock:
            if self.raw_file:self.raw_file.flush()
        # Rebuilding prior data must not hold the UART lock: reports/heartbeats
        # must keep draining while an existing segment is aggregated.
        inherited=None
        if self.inherit_session:
            from types import SimpleNamespace
            parent=self.directory/self.inherit_session
            meta=json.loads((parent/'session.json').read_text(encoding='utf-8'))
            if meta['map']['map_id']!=self.grid.map_id or meta.get('profile')!='moving_loop_demo':raise ValueError('续采地图或采集模式不匹配')
            inherited=SimpleNamespace(bins={},points=[],status={},session=self.inherit_session)
            with (parent/'raw.csv').open(newline='',encoding='utf-8') as f:
                for row in csv.DictReader(f):
                    if row['accepted']=='True':
                        BroadcastSurvey._drive_sample(inherited,[float(row[k]) for k in ('x_m','y_m','yaw_rad')],int(row['rssi_dbm']),float(row['rx_host_monotonic'])-float(row['estimated_uart_delay_s']))
        with self.lock:
            if any(not contains(self.bounds,[p['x'],p['y']]) for p in points):
                raise ValueError('广播采样点超出当前场景采集范围，请重新配置路线')
            if not self.status['connected'] or self.clock()-self.last_packet>2.5:
                raise ValueError('尚未连续收到固定基站 R550_BC01 的广播，请先放好并通电')
            if self.raw_file:self.raw_file.close()
            self.session=uuid.uuid4().hex;self.points=[];self.visit=None
            self.bins={};self.drive_active=self.moving_demo;self.drive_requested=self.moving_demo
            folder=self.directory/self.session;folder.mkdir()
            (self.directory/'latest.json').write_text(json.dumps({'session':self.session}),encoding='utf-8')
            shutil.copytree(self.grid.directory,folder/'map')
            manifest={'schema':1,'session':self.session,'map':self.grid.info(),'anchor_id':1,
                'frame_id':'map','base_frame':'base_footprint','units':{'position':'m','yaw':'rad','rssi':'dBm'},
                'pose_source':'localized ROS map pose; not external ground truth',
                'anchor_metadata':self.anchor_metadata,'transport':'SLE_PASSIVE_BROADCAST',
                'antenna_pose_reference':'base_footprint; antenna offset not surveyed',
                'region':dict(zip(('x_min','x_max','y_min','y_max'),self.bounds)), 'display_bounds':self.scene['display_bounds'], 'bin_origin':self.bin_origin,
                'target_points':points,'created_utc':datetime.now(timezone.utc).isoformat(),
                'minimum_seconds':self.minimum_seconds,'minimum_samples':self.minimum_samples,
                'maximum_seconds':self.maximum_seconds,'fixed_tx_power_setting_dbm':0}
            manifest.update(profile='moving_loop_demo' if self.moving_demo else 'stationary',
                            spatial_bin_m=.1,heading_policy='as driven; mixed orientations' if self.moving_demo else 'north')
            (folder/'session.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
            self.raw_file=(folder/'raw.csv').open('w',newline='',encoding='utf-8')
            self.csv=csv.writer(self.raw_file)
            self.csv.writerow(['utc','session','visit_id','point_id','rx_host_monotonic','device_us',
                'rx_sequence','anchor_id','rssi_dbm','queue_drops','x_m','y_m','yaw_rad',
                'stationary','accepted','reason','estimated_uart_delay_s','radio_epoch'])
            self.folder=folder;self.last_flush=self.clock();self.status.update(stage='ready',samples=0,error=None)
            if self.inherit_session:
                parent=self.directory/self.inherit_session
                meta=json.loads((parent/'session.json').read_text(encoding='utf-8'))
                if meta['map']['map_id']!=self.grid.map_id or meta.get('profile')!='moving_loop_demo':raise ValueError('续采地图或采集模式不匹配')
                segment=folder/'segments'/self.inherit_session;segment.mkdir(parents=True)
                for name in ('raw.csv','session.json','points.json'):
                    if (parent/name).exists():shutil.copy2(parent/name,segment/name)
                self.bins=inherited.bins;self.points=inherited.points
                for p in self.points:p['visit_id']=self.session
                self.status['samples']=inherited.status.get('samples',0)
                manifest['inherited_session']=self.inherit_session
                (folder/'session.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
                self.inherit_session=None;self._save_drive()

    anchor_metadata={'label':'固定基站1','x_m':None,'y_m':None,'height_m':None,'notes':''}

    def begin(self,target,health):
        with self.lock:
            if not health.get('stationary') or not health.get('ready') or not health.get('localization_confirmed'):
                raise ValueError('广播采集必须在实际停稳并确认定位后开始')
            pose=health.get('pose')
            if not pose or not all(math.isfinite(v) for v in pose):raise ValueError('实际位置无效')
            if not self.session or not self.status['connected']:raise ValueError('广播采集会话未就绪')
            if self.clock()-self.last_packet>2.5:raise ValueError('固定基站广播已丢失，不能开始该点')
            self.visit={'id':uuid.uuid4().hex,'target':dict(target),'pose':list(pose),
                        'started':self.clock(),'rssi':[],'poses':[],'drop_start':self.drops}
            self.status.update(stage='sampling',samples=0,elapsed=0,visit=self.visit['id'],error=None)

    def cancel(self):
        with self.lock:
            self.drive_active=False
            self.drive_requested=False
            if self.moving_demo:self._save_drive()
            if self.visit and self.status['stage']=='sampling':self._event('canceled',visit_id=self.visit['id'])
            self.visit=None;self.status.update(stage='canceled')
            if self.raw_file:self.raw_file.flush();os.fsync(self.raw_file.fileno())

    def _event(self,event,**fields):
        if not self.session:return
        with (self.folder/'events.jsonl').open('a',encoding='utf-8') as f:
            f.write(json.dumps({'event':event,'utc':datetime.now(timezone.utc).isoformat(),**fields},ensure_ascii=False)+'\n')

    def _error(self,message):
        self.drive_active=False
        self.status.update(stage='error',error=message,connected=False)
        self._event('error',detail=message)

    def handle_line(self,line,received_at=None):
        now=self.clock() if received_at is None else received_at
        if line=='+BCHELLO:1,RX,1,0,100':self.hello=True;return
        if line.startswith('+BCSTATUS:'):
            us,_,drops,invalid=map(int,line.split(':')[1].split(','))
            if self.last_us is not None and us<self.last_us:
                if self.moving_demo:self._radio_epoch('WS63 reboot detected')
                else:self._error('WS63 已重启，当前会话作废；重新连接后重新采集');return
            self.device_clock.observe(us/1e6,now)
            self.last_us=us;self.last_status=now;self.drops=drops;self.invalid=invalid;return
        if not line.startswith('+BCRSSI:'):return
        us,seq,anchor,rssi,drops=map(int,line.split(':')[1].split(','))
        if anchor!=1 or not -127<=rssi<=20:raise ValueError('广播数据字段越界')
        # STATUS can follow buffered reports; sequence is authoritative for RX reset detection.
        if self.last_seq and seq<=self.last_seq:
            if self.moving_demo and self.last_report_us is not None and us<self.last_report_us:self._radio_epoch('RX sequence and device clock restarted')
            elif self.moving_demo:
                self._event('duplicate_report_discarded',sequence=seq);return
            else:self._error('广播接收序号回退或重复，停止本会话以避免跨重启混合');return
        self.last_seq=seq;self.last_packet=now;self.last_rssi=rssi;self.drops=drops
        self.last_report_us=us;self.report_times.append(now)
        candidate=now-us/1e6;self.offset=candidate if self.offset is None else min(candidate,self.offset)
        aligned=self.device_clock.map_time(us/1e6)
        received=us/1e6+self.offset if aligned is None else min(now,aligned);delay=now-received
        # Nearest sampled pose; usable only in a stationary window and within 150 ms.
        h=min(self.history,key=lambda p:abs(p['t']-received)) if self.history else None
        reason='transit';accepted=False;v=self.visit;pose=h['pose'] if h else None
        if self.moving_demo and self.drive_active:
            if delay>.15:reason='serial_backlog'
            elif not h or abs(h['t']-received)>.15 or not h['valid'] or not pose:reason='pose_stale'
            elif not h['survey_active']:reason='not_survey_leg'
            elif not contains(self.bounds,pose):reason='outside_region'
            elif self.grid.value(*self.grid.cell_xy(*pose[:2]))!=0 or self.grid.forbidden(*self.grid.cell_xy(*pose[:2])):reason='map_not_free'
            elif self._pose_motion_bad(h):reason='turning_or_pose_jump'
            else:
                accepted=True;reason='moving_demo';self._drive_sample(pose,rssi,received)
        elif v and self.status['stage']=='sampling':
            age=now-v['started']
            if received<v['started']+1.:reason='settling'
            elif delay>.25:reason='serial_backlog'
            elif not h or abs(h['t']-received)>.15 or not h['valid'] or not pose:reason='pose_stale'
            elif not h['stationary']:reason='moving'
            elif math.hypot(pose[0]-v['pose'][0],pose[1]-v['pose'][1])>.04 or abs(math.atan2(math.sin(pose[2]-v['pose'][2]),math.cos(pose[2]-v['pose'][2])))>math.radians(3):
                reason='pose_drift';self._error('停靠期间定位发生漂移，本点未加入热力图；请核对定位后重试')
            elif not contains(self.bounds,pose):reason='outside_region'
            else:
                accepted=True;reason='stationary';v['rssi'].append(rssi);v['poses'].append(list(pose))
            self.status.update(samples=len(v['rssi']),elapsed=round(age,1))
        if self.raw_file:
            self.csv.writerow([datetime.now(timezone.utc).isoformat(),self.session,v['id'] if v else '',
                v['target']['id'] if v else '',now,us,seq,anchor,rssi,drops,
                *(pose if pose else ['','','']),bool(h and h['stationary']),accepted,reason,round(delay,5),self.epoch])

    def check_window(self):
        now=self.clock()
        if self.moving_demo:
            if self.session and now-self.saved_at>=1:self._save_drive()
            return
        if self.status['stage']!='sampling' or not self.visit:return
        v=self.visit;age=now-v['started']
        if now-self.last_packet>5.:
            self._error('5 秒未收到基站广播，本点未完成');return
        if self.drops>v['drop_start']:
            self._error('采集期间 WS63 接收队列溢出，本点未完成');return
        if age>=self.minimum_seconds+1 and len(v['rssi'])>=self.minimum_samples:
            xs=[p[0] for p in v['poses']];ys=[p[1] for p in v['poses']]
            p={'point_id':v['target']['id'],'visit_id':v['id'],'x':statistics.mean(xs),
               'y':statistics.mean(ys),'yaw':v['pose'][2],'target':v['target'],
               'median':statistics.median(v['rssi']),'mean':statistics.mean(v['rssi']),
               'std':statistics.pstdev(v['rssi']),'min':min(v['rssi']),'max':max(v['rssi']),
               'n':len(v['rssi']),'duration':age,'anchor_id':1}
            self.points.append(p)
            (self.folder/'points.json').write_text(json.dumps(self.points,ensure_ascii=False,indent=2),encoding='utf-8')
            self._event('complete',**p);self.raw_file.flush();os.fsync(self.raw_file.fileno())
            self.status.update(stage='done');self.visit=None
        elif age>=self.maximum_seconds:self._error('20 秒内有效停稳样本不足 40，未以时间到冒充采集完成')

    def _pose_motion_bad(self,h):
        previous=[p for p in self.history if .06<=h['t']-p['t']<=.5 and p['pose']]
        if not previous:return True
        p=previous[-1];dt=h['t']-p['t'];a,b=h['pose'],p['pose']
        angle=abs(math.atan2(math.sin(a[2]-b[2]),math.cos(a[2]-b[2])))
        return math.hypot(a[0]-b[0],a[1]-b[1])/dt>.5 or angle/dt>.45

    def _drive_sample(self,pose,rssi,received):
        origin=getattr(self,'bin_origin',[0.,0.])
        key=(math.floor((pose[0]-origin[0])/.1),math.floor((pose[1]-origin[1])/.1))
        values=self.bins.setdefault(key,[]);values.append((list(pose),rssi,received))
        points={p['point_id']:p for p in self.points}
        for (ix,iy),v in [(key,values)]:
            if len(v)<1:continue
            rss=[a[1] for a in v];yaw=math.atan2(sum(math.sin(a[0][2]) for a in v),sum(math.cos(a[0][2]) for a in v))
            points[f'path-bin-{ix}-{iy}']={'point_id':f'path-bin-{ix}-{iy}','visit_id':self.session,
                'x':statistics.mean(a[0][0] for a in v),'y':statistics.mean(a[0][1] for a in v),'yaw':yaw,
                'median':statistics.median(rss),'mean':statistics.mean(rss),'std':statistics.pstdev(rss),
                'min':min(rss),'max':max(rss),'n':len(v),'duration':v[-1][2]-v[0][2],
                'anchor_id':1,'profile':'moving_loop_demo','heading_policy':'mixed as driven'}
        self.points=list(points.values());self.status.update(samples=sum(len(v) for v in self.bins.values()))

    def _save_drive(self):
        if not self.session:return
        temp=self.folder/'points.tmp';temp.write_text(json.dumps(self.points,ensure_ascii=False,indent=2),encoding='utf-8')
        os.replace(temp,self.folder/'points.json');self.saved_at=self.clock()

    def finish_drive(self):
        with self.lock:
            self.drive_active=False
            self.drive_requested=False
            if self.moving_demo:
                self._save_drive()
                if self.status['stage']!='error':self.status.update(stage='done')

    def _radio_epoch(self,reason):
        self.epoch+=1;self.last_seq=0;self.last_us=None;self.last_report_us=None;self.offset=None;self.device_clock=DeviceClock()
        self._event('radio_epoch',epoch=self.epoch,reason=reason)

    def _worker(self):
        while not self.closed:
            self._connection_loop()
            if self.closed:return
            with self.lock:
                self.reconnects+=1
                self.status.update(connected=False,stage='reconnecting')
                self._event('reconnecting',attempt=self.reconnects,reason=self.status.get('error'))
            for _ in range(min(30,10*self.reconnects)):
                if self.closed:return
                time.sleep(.1)

    def _connection_loop(self):
        try:
            with self.lock:
                self.buffer.clear();self.hello=False;self.last_status=0;self.last_packet=0
            if self.factory is None:
                from collection import ws63_ports
                import serial
                ports=ws63_ports()
                device=ports[0] if self.device=='auto' and len(ports)==1 else self.device
                if device=='auto' or os.path.realpath(device) not in {os.path.realpath(p) for p in ports}:
                    raise ValueError('请连接唯一的 HiHope WS63 CH340K 接收板，或明确选择它的串口')
                factory=serial.Serial
            else:device=self.device;factory=self.factory
            self.stream=factory(device,115200,timeout=.02,write_timeout=1,exclusive=True)
            time.sleep(.6)
            if hasattr(self.stream,'reset_input_buffer'):self.stream.reset_input_buffer()
            self.stream.write(b'AT+BCHELLO\r\n');self.stream.flush();deadline=self.clock()+6
            while not self.closed:
                # Wait for only the first byte, then immediately drain available data.
                data=self.stream.read(1)
                waiting=getattr(self.stream,'in_waiting',0)
                if waiting:data+=self.stream.read(min(waiting,8192))
                received_at=self.clock()
                with self.lock:
                    self.buffer.extend(data)
                    if len(self.buffer)>16384:raise ValueError('串口行超长或波特率不匹配')
                    while b'\n' in self.buffer:
                        raw,_,remain=self.buffer.partition(b'\n');self.buffer=bytearray(remain)
                        self.handle_line(raw.decode('ascii',errors='replace').strip(),received_at=received_at)
                    if not self.hello and self.clock()>deadline:raise ValueError('接收板握手失败，请烧录本次广播 Client 固件')
                    if self.hello and not self.status['connected'] and self.status['stage'] in ('connecting','reconnecting'):
                        self.status.update(connected=True,stage='ready',device_resolved=device,error=None)
                        if self.moving_demo and self.drive_requested:self.drive_active=True
                    if self.status['connected'] and self.clock()-(self.last_status or (deadline-6))>4:
                        raise ValueError('WS63 心跳丢失，已停止采集')
                    self.check_window()
                    if self.raw_file and self.clock()-self.last_flush>1:
                        self.raw_file.flush();self.last_flush=self.clock()
        except Exception as exc:
            with self.lock:self._error(str(exc))
        finally:
            if self.stream:self.stream.close()
            self.stream=None

    def data(self):
        with self.lock:return {'session':self.session,'map_id':self.grid.map_id,'map':self.grid.info(),'points':list(self.points),
                              'profile':'moving_loop_demo' if self.moving_demo else 'stationary',
                              'status':self.snapshot(),'region':self.scene['display_bounds']}

    def export(self):
        with self.lock:
            if not self.session:raise ValueError('还没有广播采集会话')
            if self.moving_demo:self._save_drive()
            self.raw_file.flush();out=io.BytesIO()
            from survey_heatmap import html_report
            with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED) as z:
                for p in self.folder.rglob('*'):
                    if p.is_file():z.write(p,p.relative_to(self.folder).as_posix())
                z.writestr('heatmap.html',html_report(self.data(),self.grid.png()))
                f=io.StringIO();w=csv.writer(f);w.writerow(['point_id','x_m','y_m','yaw_rad','median_dbm','std_db','samples'])
                for p in self.points:w.writerow([p['point_id'],p['x'],p['y'],p['yaw'],p['median'],p['std'],p['n']])
                z.writestr('summary.csv',f.getvalue())
            return out.getvalue()

    def close(self):
        self.closed=True
        if self.worker and self.worker is not threading.current_thread():self.worker.join(timeout=2)
        with self.lock:
            if self.moving_demo:self._save_drive()
            if self.raw_file:self.raw_file.flush();self.raw_file.close();self.raw_file=None


def saved_data(directory,grid):
    root=Path(directory)/'collection'/'broadcast';pointer=root/'latest.json'
    if not pointer.exists():return {'points':[],'map':grid.info(),'status':{'stage':'disabled'}}
    session=json.loads(pointer.read_text(encoding='utf-8'))['session']
    if len(session)!=32 or any(c not in '0123456789abcdef' for c in session):raise ValueError('采集会话编号无效')
    folder=root/session;meta=json.loads((folder/'session.json').read_text(encoding='utf-8'))
    if meta['map']['map_id']!=grid.map_id:raise ValueError('已保存采集与当前地图版本不同')
    p=folder/'points.json';points=json.loads(p.read_text(encoding='utf-8')) if p.exists() else []
    return {'session':session,'map':meta['map'],'map_id':grid.map_id,'points':points,
            'profile':meta.get('profile','stationary'),
            'region':meta.get('display_bounds',[grid.origin[0],grid.max_x,grid.origin[1],grid.max_y]),'status':{'stage':'已保存，未继续采集'}}


def export_saved(directory,grid):
    data=saved_data(directory,grid)
    if not data.get('session'):raise ValueError('没有已保存的广播采集')
    root=Path(directory)/'collection'/'broadcast'/data['session'];out=io.BytesIO()
    from survey_heatmap import html_report
    with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED) as z:
        for p in root.rglob('*'):
            if p.is_file():z.write(p,p.relative_to(root).as_posix())
        z.writestr('heatmap.html',html_report(data,grid.png()))
        f=io.StringIO();w=csv.writer(f);w.writerow(['point_id','x_m','y_m','yaw_rad','median_dbm','std_db','samples'])
        for p in data['points']:w.writerow([p['point_id'],p['x'],p['y'],p['yaw'],p['median'],p['std'],p['n']])
        z.writestr('summary.csv',f.getvalue())
    return out.getvalue()
