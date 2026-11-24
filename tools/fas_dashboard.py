"""Read-only live view of the 304 acquisition journal."""
import csv
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
from pathlib import Path
import secrets
import sqlite3
import subprocess
import sys
from threading import Lock
from urllib.request import urlopen


VEHICLES = {'A':18890, 'B':18892}
WEB = Path(__file__).resolve().parents[1]/'demo/web/fas.html'


def car_get(port, path):
    with urlopen(f'http://127.0.0.1:{port}{path}', timeout=1.2) as response:
        return response.read()


def journal_state(db):
    db.row_factory = sqlite3.Row
    try:
        active = db.execute("SELECT a,b FROM pair_sweeps WHERE status='active'").fetchone()
        rows = db.execute('''
            SELECT p.a,p.b,p.status,t.state_id,t.delta
            FROM pair_sweeps p
            LEFT JOIN trials t ON t.a=p.a AND t.b=p.b AND t.status='pass'
            ORDER BY p.rowid DESC
        ''').fetchall()
        counts = dict(db.execute('SELECT status,COUNT(*) FROM pair_sweeps GROUP BY status').fetchall())
        last = db.execute('SELECT a,b,state_id,status,delta FROM trials ORDER BY rowid DESC LIMIT 1').fetchone()
        activity = db.execute('SELECT status,details FROM states WHERE state_id=-3').fetchone()
        tested = db.execute("SELECT COUNT(*) FROM trials WHERE a=? AND b=? AND status IN ('pass','miss')", tuple(active)).fetchone()[0] if active else 0
        recent = db.execute('SELECT a,b,state_id,status,delta FROM trials ORDER BY rowid DESC LIMIT 24').fetchall()
    except sqlite3.OperationalError as error:
        if 'no such table' not in str(error):
            raise
        return {}
    return {'active':tuple(active) if active else None, 'pairs':[dict(row) for row in rows], 'counts':counts,
            'last':dict(last) if last else None,'recent':[dict(row) for row in recent], 'tested':tested,
            'activity':json.loads(activity['details'])['message'] if activity else None}


def handler(field):
    route = json.loads((field/'304-route.json').read_text(encoding='utf-8-sig'))
    map_id = route['map_id']
    token = secrets.token_urlsafe(24)
    operator = Lock()
    resume_process = None

    def metadata():
        marker = field/'current-capture.json'
        return json.loads(marker.read_text(encoding='utf-8-sig')) if marker.exists() else {}

    def connection():
        info = metadata()
        journal = Path(info['journal']).resolve() if info else None
        if journal and journal.is_relative_to(field) and journal.is_file():
            return info, sqlite3.connect(journal.as_uri()+'?mode=ro', uri=True, timeout=1.)
        return info,None

    class Dashboard(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def respond(self, content, mime, status=200):
            self.send_response(status)
            self.send_header('Content-Type',mime)
            self.send_header('Content-Length',str(len(content)))
            self.send_header('Cache-Control','no-store')
            self.end_headers()
            self.wfile.write(content)

        def do_GET(self):
            if self.headers.get('Host') not in (f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}'):
                self.respond(b'Use the local dashboard URL.', 'text/plain',421)
                return
            try:
                self.read_view()
            except (OSError,ValueError,KeyError,sqlite3.Error) as error:
                self.respond(str(error).encode('utf-8'),'text/plain; charset=utf-8',503)

        def do_POST(self):
            nonlocal resume_process
            port = self.server.server_port
            host = self.headers.get('Host')
            if (host not in (f'127.0.0.1:{port}',f'localhost:{port}')
                    or self.headers.get('Origin') != f'http://{host}'
                    or not secrets.compare_digest(self.headers.get('X-Dashboard-Token',''),token)):
                self.respond(b'Forbidden','text/plain',403)
                return
            try:
                with operator:
                    info = metadata()
                    if not info:
                        raise ValueError('请在 304 桌面开启首轮采集')
                    run = Path(info['run_dir']).resolve()
                    if not run.is_relative_to(field) or not run.is_dir():
                        raise ValueError('当前采集目录无效')
                    if self.path == '/api/pause' and info['status']=='running' and info.get('pause_supported'):
                        (run/'stop.pause').touch()
                    elif self.path == '/api/resume' and info['status']=='paused':
                        if resume_process and resume_process.poll() is None:
                            raise ValueError('已经在检查续采条件')
                        launcher = Path(__file__).with_name('run_fas_field.py')
                        with (run/'launcher.log').open('w',encoding='utf-8') as log:
                            resume_process = subprocess.Popen([sys.executable,'-X','utf8',str(launcher),'--operator-resume'],
                                stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                                creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
                    else:
                        raise ValueError('任务状态已变化，请刷新后重试')
                self.respond(b'{"ok":true}','application/json',202)
            except (OSError,ValueError,KeyError) as error:
                self.respond(str(error).encode('utf-8'),'text/plain; charset=utf-8',409)

        def read_view(self):
            if self.path == '/':
                self.respond(WEB.read_bytes(),'text/html; charset=utf-8')
                return
            if self.path in ('/map.png','/api/map'):
                meta = json.loads(car_get(VEHICLES['A'],'/api/state'))['map']
                if meta['map_id'] != map_id:
                    raise ValueError('地图和采集点版本不一致')
                if self.path == '/map.png':
                    self.respond(car_get(VEHICLES['A'],'/map.png?map_id='+map_id),'image/png')
                else:
                    self.respond(json.dumps(meta).encode(),'application/json')
                return
            if self.path not in ('/api/progress','/trials.csv'):
                self.respond(b'Not found','text/plain',404)
                return
            info, db = connection()
            try:
                if self.path == '/trials.csv':
                    self.export(db)
                    return
                data = journal_state(db) if db else {}
            finally:
                if db:
                    db.close()
            cars = {}
            for car,port in VEHICLES.items():
                try:
                    s = json.loads(car_get(port,'/api/fas/state'))
                    h = s['health']
                    cars[car] = {'phase':s['phase'], 'pose':h['pose'] if s['map']['map_id']==map_id and h['localization_confirmed'] else None,
                                 'voltage':h['voltage'],'charging':h['charging_stable'],'message':s['message']}
                except (OSError,ValueError,KeyError):
                    cars[car] = {'phase':'offline'}
            data.update(cars=cars,run=Path(info['run_dir']).name if info else None,status=info.get('status','waiting'),
                        limit=info.get('state_count',4096),points=route['points'][:info.get('point_count',30)],
                        pause_supported=bool(info.get('pause_supported')),control_token=token)
            if info.get('status')=='paused':
                run=Path(info['run_dir']).resolve()
                log=run/'launcher.log'
                if run.is_relative_to(field) and log.is_file():
                    failed=[line for line in log.read_text(encoding='utf-8',errors='replace').splitlines()
                            if line.startswith('采集未启动或已暂停')]
                    if failed:
                        data['activity']=failed[-1]
            self.respond(json.dumps(data,ensure_ascii=False).encode('utf-8'),'application/json; charset=utf-8')

        def export(self, db):
            headers = ('id','state_id','a','b','status','delta','created_utc','completed_utc','request','receipt')
            self.send_response(200)
            self.send_header('Content-Type','text/csv; charset=utf-8')
            self.send_header('Content-Disposition','attachment; filename="fas-trials.csv"')
            self.end_headers()
            row = io.StringIO()
            writer = csv.writer(row)
            self.wfile.write(b'\xef\xbb\xbf')
            writer.writerow(headers)
            self.wfile.write(row.getvalue().encode('utf-8'))
            if db:
                for record in db.execute(f'SELECT {",".join(headers)} FROM trials ORDER BY rowid'):
                    row.seek(0)
                    row.truncate()
                    writer.writerow(record)
                    self.wfile.write(row.getvalue().encode('utf-8'))

    return Dashboard


if __name__ == '__main__':
    field = Path(sys.argv[1]).resolve()
    with ThreadingHTTPServer(('127.0.0.1',8895),handler(field)) as server:
        server.serve_forever()
