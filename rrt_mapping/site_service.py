#!/usr/bin/env python3
"""Persistent site panel. Hardware is started only by an explicit setup action."""
import argparse
from contextlib import contextmanager
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import parse_qs, urlsplit
from urllib.request import ProxyHandler, build_opener
from urllib.error import URLError
from uuid import uuid4

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parent.parent
WEB=ROOT/'demo/web'
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'demo')]
from map_grid import MapGrid, pgm
from mapping_state import prepare
import scene
from site_core import GridFrame, drive_owner, home_pose, rectangle, site_name


PARKED={'idle','docked','completed','failed'}
MOTION={'manual','explore','return'}


def atomic_json(path,value):
    p=path.with_suffix('.tmp')
    p.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
    p.replace(path)


def wait_for(check,seconds,error):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if check():
            return
        time.sleep(.10)
    raise ValueError(error)


def checkpoint_manifest(path):
    manifest=json.loads((path/'manifest.json').read_text(encoding='utf-8'))
    needed={'slam_graph.posegraph','slam_graph.data','dock_start.json','native_slam_params.yaml',
            'WHEELTEC.yaml','WHEELTEC.pgm','threshold_keepout.yaml','threshold_keepout.pgm'}
    hashes=manifest['sha256']
    if manifest['graph_prefix']!='slam_graph' or not needed <= hashes.keys():
        raise ValueError('检查点不完整')
    for name,digest in hashes.items():
        if Path(name).name!=name or hashlib.sha256((path/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('检查点校验失败')
    return manifest


def saved_frame(path):
    info=yaml.safe_load((path/'WHEELTEC.yaml').read_text(encoding='utf-8'))
    w,h,pixels=pgm(path/'WHEELTEC.pgm')
    gray=np.frombuffer(bytes(pixels),dtype=np.uint8).reshape(h,w)[::-1]
    probability=np.asarray(gray,dtype=float)/255
    if not int(info.get('negate',0)):
        probability=1-probability
    data=np.where(probability>info['occupied_thresh'],100,np.where(probability<info['free_thresh'],0,-1))
    return GridFrame(data,w,h,info['resolution'],info['origin'][:2])


class RosSession:
    def __init__(self,state):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from site_bridge import SiteBridge
        self.context=Context()
        rclpy.init(context=self.context,signal_handler_options=SignalHandlerOptions.NO)
        self.lock=threading.RLock()
        self.done=threading.Event()
        try:
            self.node=SiteBridge(state,self.context)
            self.executor=SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
        except Exception:
            self.context.try_shutdown()
            raise
        self.thread=threading.Thread(target=self.run,daemon=True)
        self.thread.start()

    def run(self):
        while not self.done.is_set():
            try:
                with self.lock:
                    self.executor.spin_once(timeout_sec=.025)
            except Exception as error:
                with self.lock:
                    self.node.fault='建图控制器异常，已停车'
                    self.node.halt('localization_fault')
                print(error,file=sys.stderr,flush=True)
                self.done.set()
            time.sleep(.0001)

    def call(self,client,request):
        if not client.wait_for_service(timeout_sec=3.):
            raise ValueError('SLAM 服务未就绪')
        with self.lock:
            future=client.call_async(request)
        wait_for(future.done,6.,'SLAM 服务响应超时')
        return future.result()

    def write(self,enabled):
        from rcl_interfaces.srv import GetParameters
        from slam_toolbox.srv import ClearQueue, Pause
        node=self.node
        result=self.call(node.param_client,GetParameters.Request(names=['paused_new_measurements']))
        if not result.values:
            raise ValueError('无法确认 SLAM 写入状态')
        if result.values[0].bool_value==enabled:
            self.call(node.pause_client,Pause.Request())
        if not enabled:
            self.call(node.clear_client,ClearQueue.Request())
        with self.lock:
            node.writing=enabled

    def close(self):
        with self.lock:
            self.node.halt()
        self.done.set()
        self.thread.join(timeout=2.)
        self.executor.shutdown(timeout_sec=1.)
        self.node.destroy_node()
        self.context.try_shutdown()


class SiteService:
    def __init__(self,root=ROOT,nav_port=8890,ros_class=RosSession):
        self.root=Path(root)
        self.base=self.root/'state/site-mapping'
        self.base.mkdir(parents=True,exist_ok=True)
        self.record=self.base/'draft.json'
        self.inhibit=self.base/'owns-robot'
        self.nav_port=nav_port
        self.ros_class=ros_class
        self.token=secrets.token_urlsafe(24)
        self.lock=threading.RLock()
        self.job=None
        self.epoch=0
        self.message=''
        self.process=self.ros=self.frozen=None
        self.browser_frame=None
        self.draft=json.loads(self.record.read_text(encoding='utf-8')) if self.record.exists() else None
        if self.draft and self.draft.get('checkpoint'):
            try:
                self.frozen=saved_frame(self.checkpoint())
            except (OSError,ValueError,KeyError):
                self.message='上次检查点不可读取'

    def nav_status(self):
        try:
            with build_opener(ProxyHandler({})).open(f'http://127.0.0.1:{self.nav_port}/api/state',timeout=.6) as r:
                return json.load(r)
        except (URLError,OSError,ValueError):
            return None

    def take_over(self):
        had_owner=self.inhibit.exists()
        self.inhibit.touch()
        try:
            nav=self.nav_status()
            if nav:
                health=nav['health']
                if (nav['phase'] not in PARKED or not health.get('stationary') or nav.get('recharge_resume_pending')
                        or (health.get('fleet') or {}).get('active')):
                    raise ValueError('请停止巡航和续采，退出双车联动并等车辆停稳')
            if nav:
                subprocess.run([str(self.root/'kit'),'stop'],cwd=self.root,check=True,
                               stdout=subprocess.DEVNULL,timeout=20)
            def stopped():
                try:
                    scene.ensure_stopped(self.nav_port)
                except ValueError:
                    return False
                return True
            wait_for(stopped,14.,'底盘或导航仍由其他程序占用，请先正常退出该程序')
        except Exception:
            if not had_owner:
                self.inhibit.unlink(missing_ok=True)
            raise

    def path(self):
        return self.base/self.draft['id']

    def checkpoint(self):
        return self.root/'maps'/'sites'/self.draft['id']/self.draft['checkpoint']

    def launch_mapping(self,checkpoint=None):
        env=dict(os.environ,R550_MAPPING_STATE=str(self.path()))
        args=['ros2','launch',str(self.root/'rrt_mapping/mapping.launch.py'),'panel:=true']
        if checkpoint:
            args.append('checkpoint:='+str(checkpoint))
        with (self.path()/'mapping.log').open('ab') as log:
            self.process=subprocess.Popen(args,cwd=self.root,env=env,stdout=log,stderr=subprocess.STDOUT,
                                          stdin=subprocess.DEVNULL,start_new_session=True)
        try:
            self.ros=self.ros_class(self.path())
        except Exception:
            self.stop_mapping()
            raise

    def stop_mapping(self):
        ros,self.ros=self.ros,None
        if ros:
            ros.close()
        process,self.process=self.process,None
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=14.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                process.wait(timeout=5.)

    @contextmanager
    def live(self):
        ros=self.ros
        if ros is None:
            raise ValueError('先建立场地或在桩位恢复检查点')
        with ros.lock:
            if ros.done.is_set():
                raise ValueError('建图控制器离线，请在桩位恢复检查点')
            yield ros.node

    def started(self,restore):
        def complete():
            if self.process.poll() is not None:
                raise ValueError('建图启动失败，请查看场地建图日志')
            with self.live() as node:
                return (node.aligned_at_dock() if restore else
                        node.fresh(time.monotonic()) and node.stationary(time.monotonic()))
        wait_for(complete,46.,'等待桩位、静止雷达和地图对齐超时。请核对贴合位置后重试')
        with self.live() as node:
            if restore:
                node.verified=True
            else:
                node.set_dock()
            if self.draft.get('bounds'):
                node.rectangle=tuple(self.draft['bounds'])
        self.ros.write(False)

    def create(self,name):
        if self.ros:
            raise ValueError('请先保存并退出当前建图')
        self.take_over()
        draft={'id':'site-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid4().hex[:6],'name':name,
               'checkpoint':None,'bounds':None}
        try:
            prepare(self.base/draft['id'])
            self.draft,self.frozen=draft,None
            atomic_json(self.record,self.draft)
            self.launch_mapping()
            self.started(False)
            self.save()
        except Exception:
            self.stop_mapping()
            raise
        self.message='桩位已标定。可遥控建图或框选自动探索范围'

    def pause(self,write=True):
        with self.live() as node:
            node.halt()
        if write:
            self.ros.write(False)

    def save(self):
        with self.live() as node:
            if not node.verified or node.fault:
                raise ValueError('先恢复可靠的桩位定位，再保存地图')
        self.pause()
        filename='checkpoint-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid4().hex[:6]
        output=self.root/'maps'/'sites'/self.draft['id']/filename
        env=dict(os.environ,R550_MAPPING_STATE=str(self.path()))
        log=self.path()/'checkpoint.log'
        with log.open('ab') as stream:
            proc=subprocess.run([sys.executable,str(self.root/'rrt_mapping/session_checkpoint.py'),'save',
                                 '--output',str(output)],env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=68.)
        if proc.returncode:
            raise ValueError('检查点保存失败，上一检查点已保留')
        checkpoint_manifest(output)
        self.frozen=saved_frame(output)
        self.draft={**self.draft,'checkpoint':filename}
        atomic_json(self.record,self.draft)
        self.message='检查点已保存，地图写入已暂停'

    def restore(self):
        if not self.draft or not self.draft.get('checkpoint'):
            if self.draft and not self.ros:
                self.create(self.draft['name'])
                return
            raise ValueError('请先建立新场地')
        path=self.checkpoint()
        checkpoint_manifest(path)
        if self.ros and not self.ros.done.is_set():
            with self.live() as node:
                if not node.stationary(time.monotonic()):
                    raise ValueError('请先停车并等底盘停稳')
            self.pause()
        if self.ros:
            self.stop_mapping()
        self.take_over()
        try:
            self.launch_mapping(path)
            self.started(True)
        except Exception:
            self.stop_mapping()
            raise
        self.message='已在桩位校准，恢复至最近检查点。可继续建图'

    def leave(self):
        if self.ros and not self.ros.done.is_set():
            self.pause()
            wait_for(lambda:self.ros.node.stationary(time.monotonic()),5.,'等待车辆停稳超时')
            with self.live() as node:
                verified=node.verified and not node.fault and node.fresh(time.monotonic())
            if verified:
                self.save()
        self.stop_mapping()
        self.inhibit.unlink(missing_ok=True)
        self.message='已结束建图。可恢复检查点、启用已保存场地或新建场地'

    def driving(self,mode,epoch):
        with self.lock:
            if epoch!=self.epoch:
                return
            with self.live() as node:
                if not node.verified or node.fault:
                    raise ValueError('需要先校准定位')
                if node.manual_mode or node.mode!='pause':
                    node.halt()
        def stopped():
            with self.live() as node:
                return not node.busy and node.stationary(time.monotonic())
        wait_for(stopped,6.,'上一任务尚未确认停车，请稍后重试')
        # Return on the current partial map without integrating docking contact/slip.
        self.ros.write(mode!='return')
        with self.lock:
            canceled=epoch!=self.epoch
            if not canceled:
                with self.live() as node:
                    node.begin(mode)
        if canceled:
            self.ros.write(False)
            return
        self.message={'manual':'按住方向键遥控，松手停车','explore':'正在探索选定区域',
                      'return':'正在使用当前地图回桩'}[mode]

    def select_bounds(self,value):
        bounds=rectangle(value)
        with self.live() as node:
            if node.busy or node.mode!='pause' or not node.stationary(time.monotonic()):
                raise ValueError('等车辆停稳后再修改探索范围')
            node.rectangle=bounds
        self.draft={**self.draft,'bounds':bounds}
        atomic_json(self.record,self.draft)
        self.message='探索范围已保存'

    def export_scene(self):
        if not self.draft or not self.draft.get('checkpoint'):
            raise ValueError('请先保存检查点')
        path=self.checkpoint()
        checkpoint_manifest(path)
        data=self.draft
        final=self.root/'scenes'/data['id']
        if final.exists():
            raise ValueError('此场地已保存。可从已保存场地启用，或另建场地')
        stage=self.root/'scenes'/('.'+data['id']+'-'+uuid4().hex[:6])
        scene.create(data['id'],path/'WHEELTEC.yaml',target=stage)
        for file in ('dock_start.json','threshold_keepout.pgm','threshold_keepout.yaml'):
            (stage/file).write_bytes((path/file).read_bytes())
        atomic_json(stage/'map_metadata.json',{'version':data['name'],'scene':data['id']})
        atomic_json(stage/'site.json',{'checkpoint':str(path.relative_to(self.root))})
        grid=MapGrid(stage)
        grid.validate(*home_pose(json.loads((stage/'dock_start.json').read_text()))[:2],0.)
        stage.rename(final)
        self.message='场地与桩位已保存，可启用巡航'
        return data['id']

    def activate(self,identity):
        available={entry['id'] for entry in self.scenes()}
        if identity not in available:
            raise ValueError('请选择已保存的场地')
        if self.ros:
            self.pause()
            wait_for(lambda:self.ros.node.stationary(time.monotonic()),5.,'请等待小车停稳')
            self.stop_mapping()
        self.take_over()
        scene.activate(identity,nav_port=self.nav_port)
        self.inhibit.unlink(missing_ok=True)
        subprocess.run([str(self.root/'kit'),'nav'],cwd=self.root,check=True,
                       stdout=subprocess.DEVNULL,timeout=30.)
        self.message='已切换巡航场地，可打开巡航面板核对定位'

    def scenes(self):
        result=[]
        for folder in sorted((self.root/'scenes').glob('*')):
            if folder.name.startswith('.') or folder.name=='demo-room' or not (folder/'map.yaml').is_file():
                continue
            try:
                dock=json.loads((folder/'dock_start.json').read_text(encoding='utf-8'))
                meta=json.loads((folder/'map_metadata.json').read_text(encoding='utf-8'))
                if dock.get('calibrated',True):
                    result.append({'id':folder.name,'name':meta.get('version',folder.name)})
            except (OSError,ValueError):
                continue
        return result

    def snapshot(self):
        with self.lock:
            ros=self.ros
            if ros and not ros.done.is_set():
                with ros.lock:
                    value,frame=ros.node.view(),ros.node.frame
            else:
                frame=self.frozen
                value={'phase':'restore' if self.draft else 'empty',
                       'map':frame.view() if frame else None,'ready':False,'stationary':False}
                if frame and self.draft:
                    try:
                        value['dock']=home_pose(json.loads((self.path()/'dock_start.json').read_text()))
                    except (OSError,ValueError,KeyError):
                        pass
            old=self.browser_frame
            self.browser_frame=frame
            if frame and (old is None or frame.id!=old.id):
                self.previous_frame=old
            return {**value,'draft':self.draft,'job':self.job,'message':self.message,'scenes':self.scenes(),
                    'live':ros is not None and not ros.done.is_set(),'nav_port':self.nav_port,'control_token':self.token}

    def image(self,identity):
        with self.lock:
            frame=next((v for v in (self.browser_frame,getattr(self,'previous_frame',None))
                        if v and v.id==identity),None)
        return frame.image() if frame else None

    def dispatch(self,data):
        action=data.get('action')
        if not isinstance(action,str):
            raise ValueError('无效操作')
        with self.lock:
            if action=='stop':
                self.epoch+=1
                if self.ros and not self.ros.done.is_set():
                    self.pause(write=False)
                self.message='已停车'
                if not self.job and self.ros and not self.ros.done.is_set():
                    self._job('pause',self.pause)
                return
            if self.job:
                raise ValueError('正在处理上一操作')
            if action=='drive':
                with self.live() as node:
                    if not node.manual_mode or data.get('session')!=node.drive_session:
                        raise ValueError('请先启用遥控建图')
                    node.manual.update(time.monotonic(),drive_owner(data.get('owner')),data.get('direction'),
                                       data.get('sequence'))
                return
            if action=='new' and data.get('seated') is True:
                self._job(action,self.create,site_name(data.get('name')))
            elif action=='restore' and data.get('seated') is True:
                self._job(action,self.restore)
            elif action=='activate' and data.get('seated') is True and isinstance(data.get('scene'),str):
                self._job(action,self.activate,data['scene'])
            elif action in MOTION:
                self._job(action,self.driving,action,self.epoch)
            elif action=='leave':
                self._job(action,self.leave)
            elif action in ('save','export','bounds'):
                if action=='bounds':
                    self.select_bounds(data.get('bounds'))
                else:
                    self._job(action,self.save if action=='save' else self.export_scene)
            else:
                raise ValueError('请核对操作，并确认车尾贴合和方向')

    def _job(self,name,fn,*args):
        self.job=name
        self.message=''
        def worker():
            try:
                fn(*args)
            except (ValueError,KeyError,TypeError,OSError,RuntimeError,subprocess.SubprocessError) as error:
                self.message=str(error)
                print(f'{name}: {error}',file=sys.stderr,flush=True)
            finally:
                with self.lock:
                    self.job=None
        threading.Thread(target=worker,daemon=True).start()


CONTENT={'/':'site.html','/site.js':'site.js','/site.css':'site.css'}


def handler(site):
    class Handler(BaseHTTPRequestHandler):
        def reply(self,value,status=200,kind='application/json; charset=utf-8'):
            data=json.dumps(value,ensure_ascii=False,allow_nan=False).encode() if isinstance(value,dict) else value
            self.send_response(status)
            self.send_header('Content-Type',kind)
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('Content-Length',str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            url=urlsplit(self.path)
            if url.path in CONTENT:
                mime={'/':'text/html','/site.js':'text/javascript','/site.css':'text/css'}
                self.reply((WEB/CONTENT[url.path]).read_bytes(),kind=mime[url.path]+'; charset=utf-8')
            elif url.path=='/api/site/state':
                self.reply(site.snapshot())
            elif url.path=='/api/site/map':
                image=site.image(parse_qs(url.query).get('id',[''])[0])
                if image is None:
                    self.reply({'error':'地图已更新'},404)
                else:
                    self.reply(image,kind='image/png')
            else:
                self.reply({'error':'页面不存在'},404)

        def do_POST(self):
            if self.path!='/api/site/command':
                return self.reply({'error':'页面不存在'},404)
            supplied=self.headers.get('X-Control-Token','')
            origin=self.headers.get('Origin')
            if (not secrets.compare_digest(supplied.encode(),site.token.encode()) or
                    (origin and origin!='http://'+self.headers.get('Host',''))):
                return self.reply({'error':'控制会话无效，请刷新页面'},403)
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=4096:
                    raise ValueError('请求大小无效')
                value=json.loads(self.rfile.read(size))
                if not isinstance(value,dict):
                    raise ValueError('无效操作')
                site.dispatch(value)
            except (ValueError,TypeError,OSError) as error:
                return self.reply({'error':str(error)},409)
            self.reply({'accepted':True})

        def log_message(self,*_):
            pass
    return Handler


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind',default='0.0.0.0')
    parser.add_argument('--port',type=int,default=8896)
    parser.add_argument('--nav-port',type=int,default=8890)
    args=parser.parse_args()
    site=SiteService(nav_port=args.nav_port)
    web=ThreadingHTTPServer((args.bind,args.port),handler(site))
    def shutdown(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,shutdown)
    print(f'Site mapping: http://{args.bind}:{args.port}/',flush=True)
    try:
        web.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        site.stop_mapping()
        web.server_close()


if __name__=='__main__':
    main()
