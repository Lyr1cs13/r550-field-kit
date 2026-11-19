"""Start and pause the position-pair runner from the local fleet dashboard."""
from contextlib import closing
import json
from pathlib import Path
import sqlite3
import subprocess
import sys

from fas_experiment import CARS, RECOVERY_STATE, Stations, fleet_pose_ready


class PairSimulation:
    def __init__(self, directory, settings, addresses):
        self.route = (directory/settings['route']).resolve()
        self.stations = Stations(json.loads(self.route.read_text(encoding='utf-8')))
        self.points = [{**point,'number':index} for index,point in enumerate(self.stations.points.values(),1)]
        self.journal = (directory/settings.get('journal','position-pairs.sqlite')).resolve()
        self.pause_file = self.journal.with_suffix('.pause')
        self.endpoints = self.journal.with_suffix('.endpoints.json')
        self.log = self.journal.with_suffix('.log')
        self.settings = {'cars':dict(zip(CARS,addresses)),
                         **{key:settings[key] for key in ('navigation_timeout_s','charge_timeout_s') if key in settings}}
        self.window_s = float(settings.get('window_s',2.))
        if not .2<=self.window_s<=180.:
            raise ValueError('模拟采集时间无效')
        self.process = None

    @property
    def running(self):
        return self.process is not None and self.process.poll() is None

    def progress(self):
        if not self.journal.is_file():
            return None,0,None,None
        with closing(sqlite3.connect(self.journal.as_uri()+'?mode=ro',uri=True,timeout=.2)) as db:
            owner = db.execute('SELECT id FROM run').fetchone()
            count, = db.execute("SELECT COUNT(*) FROM states WHERE state_id>=0 AND status='arrived'").fetchone()
            target = db.execute("SELECT state_id,a,b FROM trials WHERE status='pending'").fetchone()
            stage = db.execute("SELECT status,details FROM states WHERE state_id=?",(RECOVERY_STATE,)).fetchone()
        return owner[0] if owner else None,count,target,stage

    def start_reason(self, states, owner):
        if any(not s.get('vehicle_connected') for s in states):
            return '等待两车上线'
        if any(s['map']['map_id']!=self.stations.map_id for s in states):
            return '采集点与当前地图不一致'
        for car,s in zip(CARS,states):
            f,h = s.get('fas',{}),s['health']
            if '停车锁定' in str(h.get('readiness','')):
                return f'{car} 车已锁定，等待停车验收'
            if f.get('car')!=car or f.get('motion_only') is not True:
                return '等待两台车启用模拟采集'
            if f.get('owner') not in (None,owner) or f.get('lease_live'):
                return '等待两车结束现有任务'
            if s.get('collection',{}).get('mode')!='timed' or s.get('c5',{}).get('enabled'):
                return '请先关闭两车的实采功能'
            if (s['phase'] not in ('idle','completed','failed','docked') or not h.get('stationary')
                    or not h.get('dock_calibrated') or h.get('map_matches') is False
                    or s.get('recharge_resume_pending')):
                return '等待两车停稳并确认定位'
            if not fleet_pose_ready(h) and f.get('recovery') not in ('available','waiting'):
                return f'{car} 车缺少可用于静止复核的定位参考'
        return ''

    def status(self, states):
        try:
            owner,count,target,stage = self.progress()
        except sqlite3.Error:
            return {'running':self.running,'stopping':False,'reason':'采集记录暂不可用','completed':0,'targets':{}}
        stopping = self.running and self.pause_file.exists()
        phases = [s.get('fas',{}) for s in states]
        shared = phases[0].get('lease_epoch') if len(phases)==2 else None
        parallel = bool(shared and all(f.get('lease_live') and f.get('command_id')==shared for f in phases))
        scan_holds=[car for car,f in zip(CARS,phases) if f.get('scan_hold')]
        if stopping:
            message = '正在停车'
        elif self.running:
            message = ('、'.join(scan_holds)+' 车停车复核雷达' if scan_holds else
                       '两车并行' if parallel else json.loads(stage[1])['message']
                       if stage and stage[0] in ('recovering','charging') else '位置对调度中')
        elif self.process is not None and (self.pause_file.exists() or self.process.returncode):
            message = json.loads(stage[1])['message'] if stage and stage[0]=='blocked' else '已暂停 · 进度已保存'
        else:
            message = '双车模拟采集'
        return {'running':self.running,'stopping':stopping,'auto_charge':not self.pause_file.with_suffix('.no-auto-charge').exists(),
                'return_requested':self.running and self.pause_file.with_suffix('.return-now').exists(),'completed':count,'message':message,
                'reason':self.start_reason(states,owner),'round':target[0]+1 if target else None,
                'targets':dict(zip(CARS,target[1:])) if target else {}}

    def charge_option(self, action, enabled):
        self.journal.parent.mkdir(parents=True,exist_ok=True)
        if action=='auto_charge':
            flag=self.pause_file.with_suffix('.no-auto-charge')
            if enabled:flag.unlink(missing_ok=True)
            else:flag.touch()
        elif action=='return_now' and enabled and self.running and not self.pause_file.exists():
            self.pause_file.with_suffix('.return-now').touch()
        else:
            raise ValueError('请先开启 50 点测试')

    def start(self, states):
        if self.running:
            return
        try:
            owner,count,target,_ = self.progress()
        except sqlite3.Error as error:
            raise ValueError('采集记录暂不可用') from error
        reason = self.start_reason(states,owner)
        if reason:
            raise ValueError(reason)
        # Resume the incomplete block. The worker stays active for later blocks.
        start = (target[0] if target else count)//50*50
        self.journal.parent.mkdir(parents=True,exist_ok=True)
        pending = self.endpoints.with_suffix('.tmp')
        pending.write_text(json.dumps(self.settings),encoding='utf-8')
        pending.replace(self.endpoints)
        self.pause_file.with_suffix('.return-now').unlink(missing_ok=True)
        self.pause_file.unlink(missing_ok=True)
        runner = Path(__file__).resolve().parents[1]/'tools/fas_experiment.py'
        args = [sys.executable,'-u',str(runner),'--route',str(self.route),'--journal',str(self.journal),
                '--live-config',str(self.endpoints),'--motion-only','--window-s',str(self.window_s),
                '--start',str(start),'--count','50','--pause-file',str(self.pause_file),'--resume-operator','--unattended']
        with self.log.open('ab') as log:
            self.process = subprocess.Popen(args,cwd=runner.parents[1],stdin=subprocess.DEVNULL,stdout=log,
                                            stderr=subprocess.STDOUT,start_new_session=True)

    def pause(self):
        if self.running:
            self.pause_file.touch()
