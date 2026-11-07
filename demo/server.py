#!/usr/bin/env python3
"""Local demo UI. Offline rehearsal and live ROS are explicit separate modes."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import secrets
import threading
import time
import traceback

from map_grid import MapGrid
from mission import Mission, QUIET

HERE = Path(__file__).resolve().parent


class App:
    def __init__(self, grid, backend, directory):
        self.grid, self.backend = grid, backend
        self.mission = Mission(grid, backend, directory)
        self.token = secrets.token_urlsafe(24);self.image = grid.png();self.running = True
        cache=grid.directory/'survey_route.json'
        self.loop_proposal=json.loads(cache.read_text(encoding='utf-8')) if cache.exists() else None
        if self.loop_proposal and self.loop_proposal.get('map_id')!=grid.map_id:self.loop_proposal=None
        self.parallel_proposal=None;self.probe_proposal=None

    def view_state(self):
        state=self.mission.state();proposal=self.loop_proposal
        if proposal and len(proposal['points'])==len(state['points']) and all(a['id']==b['id'] and a['x']==b['x'] and a['y']==b['y'] for a,b in zip(proposal['points'],state['points'])):
            state['survey_route_preview']=proposal.get('preview_path',[])
        return state

    def command(self, data):
        action = data['action'];m = self.mission
        with m.lock:
            if action == 'collection_config':
                m.configure_collection(data['mode'],str(data.get('device','auto')))
            elif action == 'survey_load_north':
                if m.phase not in QUIET:raise ValueError('请先停止任务再加载采样点')
                from survey_route import north_route
                proposal=self.parallel_proposal or self.loop_proposal or north_route(self.grid)
                self.loop_proposal=proposal
                previous=m.directory/'before_north_survey.json'
                if not previous.exists():previous.write_text(json.dumps(m.points,ensure_ascii=False),encoding='utf-8')
                m.save_points(proposal['points'])
                m.event(f'已加载 {len(m.points)} 个回环引导点，尚未发车；沿途采集，不做定点等候或统一朝向')
            elif action == 'survey_restore_points':
                p=m.directory/'before_north_survey.json'
                if not p.exists():raise ValueError('没有保存的原巡航点位')
                m.save_points(json.loads(p.read_text(encoding='utf-8')))
            elif action == 'survey_anchor':
                if m.phase not in QUIET:raise ValueError('先停止任务再记录基站位置')
                height=data.get('height_m')
                if height is not None and (not isinstance(height,(int,float)) or not math.isfinite(height) or not 0<height<5):
                    raise ValueError('天线高度需在 0～5 米之间')
                m.collection.anchor_metadata={'label':str(data.get('label','固定基站1'))[:80],
                    'notes':str(data.get('notes',''))[:500],'height_m':height,'x_m':None,'y_m':None}
                if m.collection.broadcast:m.collection.broadcast.anchor_metadata=dict(m.collection.anchor_metadata)
                m.event('已记录固定基站摆放说明；移动基站后请开始新的采集任务')
            elif action == 'save_points':
                m.save_points(data['points'])
            elif action == 'go':
                m.start(data['id'])
            elif action == 'start':
                m.start(auto_return=data.get('auto_return', True),start_from=data.get('start_from',0))
            elif action == 'survey_inherit':
                identity=str(data.get('session',''))
                if m.phase not in QUIET or not m.collection.broadcast or not m.collection.broadcast.moving_demo:raise ValueError('先停稳并连接广播采集板')
                if len(identity)!=32 or any(c not in '0123456789abcdef' for c in identity):raise ValueError('会话编号无效')
                parent=m.collection.broadcast.directory/identity
                meta=json.loads((parent/'session.json').read_text(encoding='utf-8'))
                if meta['map']['map_id']!=self.grid.map_id or meta.get('profile')!='moving_loop_demo':raise ValueError('地图或模式不匹配')
                m.collection.broadcast.inherit_session=identity
                m.collection.broadcast.anchor_metadata=dict(meta['anchor_metadata'])
                m.event('已选择同一固定基站与地图的续采记录，下一次启动将保留前段原始数据')
            elif action == 'pause':
                m.pause()
            elif action == 'resume':
                m.resume()
            elif action == 'stop':
                m.stop()
            elif action == 'return':
                m.event('用户点击主动回充')
                m.return_to_dock()
            elif action in ('initial_pose','initial_dock'):
                if m.phase not in QUIET:
                    raise ValueError('先停止任务，再修改定位')
                pose = self.backend.dock_pose if action == 'initial_dock' else [float(v) for v in data['pose']]
                if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
                    raise ValueError('定位数据无效')
                self.grid.validate(pose[0],pose[1],clearance=0)
                self.backend.initialize_pose(pose, known_dock=action=='initial_dock')
                m.event('按已保存桩位初始化定位，未修改充电桩坐标' if action=='initial_dock' else '设置当前位置，等待核对雷达与地图')
            elif action == 'confirm':
                if m.phase not in QUIET:
                    raise ValueError('请先停止任务')
                self.backend.confirm_localization();m.event('已确认位置与朝向')
            elif action == 'import':
                if data['map_id'] != self.grid.map_id:
                    raise ValueError('导入点位属于另一地图版本，不能直接使用')
                m.save_points(data['points'])
            else:
                raise ValueError('未知操作')
        return self.view_state()

    def tick(self):
        while self.running:
            try:
                self.mission.tick()
            except Exception as e:
                traceback.print_exc()
                with self.mission.lock:
                    self.mission._fail('任务执行异常：'+str(e))
            time.sleep(.05)


def handler(app):
    class Handler(BaseHTTPRequestHandler):
        def respond(self, data, code=200, content='application/json; charset=utf-8'):
            if isinstance(data, (dict,list)):
                data = json.dumps(data,ensure_ascii=False,allow_nan=False).encode('utf-8')
            self.send_response(code);self.send_header('Content-Type',content)
            self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)))
            self.end_headers();self.wfile.write(data)

        def do_GET(self):
            path = self.path.split('?',1)[0]
            if path == '/api/state':
                state = app.view_state();state['control_token'] = app.token;self.respond(state)
            elif path == '/api/collection/export':
                self.respond(app.mission.collection.export(),content='application/x-ndjson; charset=utf-8')
            elif path == '/api/collection/bundle':
                self.respond(app.mission.collection.export(packed=True),content='application/zip')
            elif path == '/api/collection/servo/export':
                self.respond(app.mission.collection.servo_store.export(packed=True), content='application/zip')
            elif path == '/api/broadcast/data':
                b=app.mission.collection.broadcast
                from broadcast_survey import saved_data
                try:data=b.data() if b and b.session else saved_data(app.mission.directory,app.grid)
                except (ValueError,KeyError,OSError) as exc:self.respond({'error':str(exc)},409);return
                self.respond(data)
            elif path == '/api/broadcast/export':
                b=app.mission.collection.broadcast
                from broadcast_survey import export_saved
                try:content=b.export() if b and b.session else export_saved(app.mission.directory,app.grid)
                except (ValueError,KeyError,OSError) as exc:self.respond({'error':str(exc)},409);return
                self.respond(content,content='application/zip')
            elif path == '/api/survey/route':
                from survey_route import north_route
                self.respond(app.parallel_proposal or app.loop_proposal or north_route(app.grid))
            elif path == '/rssi':
                from survey_heatmap import html_report
                self.respond(html_report().encode('utf-8'),content='text/html; charset=utf-8')
            elif path == '/api/export':
                self.respond({'schema':1,'map_id':app.grid.map_id,'points':app.mission.state()['points']})
            elif path == '/map.png':
                self.respond(app.image,content='image/png')
            elif path in ('/','/app.js','/style.css','/ui.js','/vendor/tabler.min.css','/vendor/icons.svg'):
                name = 'index.html' if path == '/' else path[1:]
                content = ('text/css; charset=utf-8' if name.endswith('.css') else
                           'text/javascript; charset=utf-8' if name.endswith('.js') else
                           'image/svg+xml' if name.endswith('.svg') else 'text/html; charset=utf-8')
                self.respond((HERE/'web'/name).read_bytes(),content=content)
            else:
                self.respond({'error':'不存在的页面'},404)

        def do_POST(self):
            if self.path != '/api/command':
                self.respond({'error':'不存在的接口'},404);return
            if not secrets.compare_digest(self.headers.get('X-Control-Token',''),app.token):
                self.respond({'error':'页面控制会话失效，请刷新页面'},403);return
            try:
                length = int(self.headers.get('Content-Length','0'))
                if not 0 < length <= 65536:
                    raise ValueError('请求过大或为空')
                def invalid(value):
                    raise ValueError('不能使用无穷或非数值坐标')
                data = json.loads(self.rfile.read(length).decode('utf-8'),parse_constant=invalid)
                self.respond(app.command(data))
            except (ValueError,KeyError,TypeError) as e:
                self.respond({'error':str(e)},400)
            except Exception:
                traceback.print_exc();self.respond({'error':'操作失败，请核对导航连接状态'},409)

        def log_message(self, fmt, *args):
            if len(args) > 1 and str(args[1]) not in ('200','304'):
                super().log_message(fmt,*args)
    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend',choices=['offline','ros'],default='offline')
    parser.add_argument('--host',default='127.0.0.1');parser.add_argument('--port',type=int,default=8890)
    parser.add_argument('--data',type=Path)
    args = parser.parse_args()
    grid = MapGrid(HERE/'assets');dock = json.loads((HERE/'assets/dock_start.json').read_text(encoding='utf-8'))
    if args.backend == 'offline':
        from mock_backend import MockBackend
        backend = MockBackend(grid,dock)
    else:
        import rclpy
        from ros_backend import RosBackend
        rclpy.init(args=[]);backend = RosBackend(grid,dock)
        threading.Thread(target=rclpy.spin,args=(backend,),daemon=True).start()
    directory = args.data or HERE/'data'/args.backend
    app = App(grid,backend,directory)
    web = ThreadingHTTPServer((args.host,args.port),handler(app))
    threading.Thread(target=app.tick,daemon=True).start()
    print(f'{args.backend.upper()} demo: http://{args.host}:{args.port}/',flush=True)
    try:
        web.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        app.running = False;app.mission.collection.close();backend.stop();web.server_close()


if __name__ == '__main__':
    main()
