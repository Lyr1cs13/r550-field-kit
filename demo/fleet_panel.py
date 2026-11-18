#!/usr/bin/env python3
"""A localhost dashboard for two individually routed R550 panels."""
import argparse
from contextlib import ExitStack
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import threading
import time
from urllib.parse import urlsplit

from desktop_panel import DesktopPanel, JSON, NETWORK_ERRORS
from fleet_zone import separated
from mission import QUIET
from server import handler


MOVEMENT = {'go', 'start', 'resume', 'return'}
LEASE_GRACE = 1.5


def zone(state):
    return state['health'].get('fleet') or {}


def armed(state):
    settings=zone(state)
    return bool(settings and settings.get('active',True))


def parked(state):
    return (state['phase'] in QUIET and state['health'].get('stationary') is True
            and not state.get('recharge_resume_pending'))


def pair_status(states):
    if any(s.get('vehicle_connected') is not True for s in states):
        return '等待两台车端面板上线'
    if states[0]['map']['map_id'] != states[1]['map']['map_id']:
        return '两台车的地图版本不一致'
    health = [s['health'] for s in states]
    zones = [zone(s) for s in states]
    if [z.get('robot_id') for z in zones] != ['a', 'b']:
        return '等待两台车启用各自的双车运行区'
    if not separated(zones[0].get('bounds'), zones[1].get('bounds')):
        return '请确认两车运行区及两区之间的间隔'
    if any(not h.get('ready') or not h.get('localization_confirmed') for h in health):
        return '等待在同一张地图上确认两车的位置与朝向'
    return next((z['motion_reason'] for z in zones if z['motion_reason']), '')


class Fleet:
    def __init__(self):
        self.panels = []
        self.lock = threading.Lock()
        self.paired = False
        self.last_lease = float('-inf')
        self.token = secrets.token_urlsafe(24)

    def states(self):
        return [DesktopPanel.view_state(panel) for panel in self.panels]

    def paired_reason(self, states):
        reason=pair_status(states)
        if reason:
            return reason
        if any(not armed(s) or not zone(s).get('leased') for s in states):
            return '正在确认两车运行许可'
        return ''

    def pair_start_reason(self, states):
        return pair_status(states) or ('请先让两台车结束任务并停稳' if not all(map(parked,states)) else '')

    def drive_reason(self, index, states):
        if self.paired:
            return self.paired_reason(states)
        if armed(states[index]):
            return '等待车端退出联动保护'
        peer=states[1-index]
        other=self.panels[1-index]
        if peer.get('vehicle_connected'):
            if not parked(peer):
                return '另一台车正在执行任务，停稳后可切换控制'
        elif other.motion_unknown or (other.last_state and not parked(other.last_state)):
            if (not other.last_state or not armed(other.last_state)
                    or time.monotonic()-self.last_lease < LEASE_GRACE):
                return '另一台车在任务中断线，请确认它已停稳'
        return ''

    def status(self):
        states = self.states()
        return {'paired':self.paired, 'pair_reason':self.pair_start_reason(states),
                'reason':self.paired_reason(states) if self.paired else '',
                'can_leave':all(parked(s) for s in states if s.get('vehicle_connected')),
                'control_token':self.token, 'vehicles':[
                    {'connected':s.get('vehicle_connected') is True, 'phase':s['phase'],
                     'readiness':s['health']['readiness'],'charging':bool(s['health'].get('charging_stable')),
                     'localized':s['health'].get('localization_confirmed',False)} for s in states]}

    def set_paired(self, requested):
        with self.lock:
            states=self.states()
            if requested==self.paired:
                return
            if requested:
                reason=self.pair_start_reason(states)
                if reason:
                    raise ValueError(reason)
                for panel,state in zip(self.panels,states):
                    if not armed(state):
                        panel.set_mode(True,state)
            elif not all(parked(s) for s in states if s.get('vehicle_connected')):
                raise ValueError('请先让在线车辆结束任务并停稳')
            self.paired=requested

    def maintain(self,stopped):
        while not stopped.is_set():
            states=self.states()
            online=[s for s in states if s.get('vehicle_connected')]
            if self.paired and not pair_status(states) and all(map(armed,states)):
                for panel,state in zip(self.panels,states):
                    if not self.paired:
                        break
                    try:
                        self.last_lease = time.monotonic()
                        _,code,_=panel.request('POST','/api/command',body=b'{"action":"fleet_lease"}',
                                               token=state['control_token'],timeout=.4)
                        if code!=200:break
                    except NETWORK_ERRORS:
                        break
            elif self.lock.acquire(blocking=False):
                try:
                    if self.paired and len(online)==1 and parked(online[0]) and time.monotonic()-self.last_lease>=LEASE_GRACE:
                        self.paired=False
                    if not self.paired:
                        for panel,state in zip(self.panels,self.states()):
                            if state.get('vehicle_connected') and armed(state) and parked(state):
                                try:
                                    panel.set_mode(False,state)
                                except ValueError:
                                    continue
                finally:
                    self.lock.release()
            stopped.wait(.25)


class FleetDesktopPanel(DesktopPanel):
    def __init__(self, fleet, url, data):
        self.fleet = fleet
        self.last_state = None
        self.motion_unknown = False
        self.refresh_lock = threading.Lock()
        super().__init__(url, data)

    def refresh(self):
        with self.refresh_lock:
            if not super().refresh():
                return False
            self.last_state=super().view_state()
            self.motion_unknown=False
            return True

    def set_mode(self, active, state):
        body=json.dumps({'action':'fleet_mode','active':active}).encode()
        try:
            data,code,_=self.request('POST','/api/command',body=body,token=state['control_token'])
        except NETWORK_ERRORS as error:
            raise ValueError('无法连接车端切换运行模式') from error
        if code!=200:
            raise ValueError(json.loads(data).get('error','车端拒绝切换运行模式'))
        if not self.refresh() or armed(self.last_state)!=active:
            raise ValueError('车端尚未确认运行模式')

    def view_state(self):
        index=self.fleet.panels.index(self)
        states=self.fleet.states()
        state,peer=states[index],states[1-index]
        peer_pose=None
        if (peer.get('vehicle_connected') and peer['map']['map_id']==state['map']['map_id']
                and peer['health'].get('localization_confirmed') and peer['health'].get('ready')):
            peer_pose=peer['health'].get('pose')
        docks=[{'number':i+1,'pose':s['dock']} for i,s in enumerate(states)
               if s.get('dock') and s['health'].get('dock_calibrated',True)
               and s['map']['map_id']==state['map']['map_id']]
        zones=[{'number':i+1,'bounds':zone(s).get('bounds')} for i,s in enumerate(states)
               if self.fleet.paired and zone(s).get('bounds') and s['map']['map_id']==state['map']['map_id']]
        return {**state,'fleet_paired':self.fleet.paired or armed(state),
                'fleet_control':self.fleet.drive_reason(index,states) if state.get('vehicle_connected') else '',
                'fleet_number':index+1,'fleet_peer':peer_pose,'fleet_docks':docks,'fleet_zones':zones}

    def command(self, data, token):
        try:
            command = json.loads(data)
        except (ValueError, TypeError):
            return {'error':'命令格式无效'},400,JSON
        if not isinstance(command,dict) or not isinstance(command.get('action'),str):
            return {'error':'命令格式无效'},400,JSON
        action=command['action']
        if action in ('fleet_lease','fleet_mode'):
            return {'error':'运行模式和许可由管理面板维护'},403,JSON
        if action not in MOVEMENT:
            return super().command(data,token)
        with self.fleet.lock:
            index=self.fleet.panels.index(self)
            states=self.fleet.states()
            if not states[index].get('vehicle_connected'):
                return {'error':'实车未连接'},503,JSON
            reason = self.fleet.drive_reason(index,states)
            if reason:
                return {'error':reason},409,JSON
            if self.fleet.paired and (action=='return' or (action=='start' and command.get('auto_return',True))):
                return {'error':'双车运行区请先关闭回充'},409,JSON
            result=super().command(data,token)
            if result[1] in (200,503) and not self.refresh():
                self.motion_unknown=True
            return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    settings = json.loads(args.config.read_text(encoding='utf-8'))
    robots = settings['vehicles']
    if len(robots)!=2:
        raise ValueError('需要两个独立车端地址')
    ports = [settings['port'], *(r['port'] for r in robots)]
    if len(set(ports))!=3 or not all(type(port) is int and 1024 <= port < 65536 for port in ports):
        raise ValueError('双车面板端口无效')
    if robots[0]['address']==robots[1]['address']:
        raise ValueError('两路必须使用不同的车端入口')

    fleet = Fleet()
    fleet.panels = [FleetDesktopPanel(fleet, robot['address'], Path(robot['data'])) for robot in robots]
    site_urls=[]
    for robot in robots:
        host=urlsplit(robot['address']).hostname
        url=robot.get('site_address') or (f'http://{host}:8896/' if host not in ('127.0.0.1','localhost','::1') else '')
        if url:
            parsed=urlsplit(url)
            if parsed.scheme!='http' or not parsed.hostname or parsed.username or parsed.query or parsed.fragment or parsed.path not in ('','/'):
                raise ValueError('新场地面板地址无效')
        site_urls.append(url)
    source = (Path(__file__).parent/'web/fleet.html').read_text(encoding='utf-8')
    page = source.replace('PORT_A',str(ports[1])).replace('PORT_B',str(ports[2])).encode('utf-8')

    class Dashboard(BaseHTTPRequestHandler):
        def local(self):
            if self.headers.get('Host') not in (f'127.0.0.1:{ports[0]}', f'localhost:{ports[0]}'):
                self.send_error(421)
                return False
            return True

        def reply(self,data,kind=JSON,status=200):
            if kind==JSON:
                data=json.dumps(data,ensure_ascii=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type',kind)
            self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if not self.local():
                return
            if self.path == '/api/fleet':
                self.reply({**fleet.status(),'site_available':[bool(u) for u in site_urls]})
            elif self.path == '/':
                self.reply(page,'text/html; charset=utf-8')
            elif self.path in ('/new-site/1','/new-site/2'):
                i=int(self.path[-1])-1
                if not site_urls[i]:
                    self.reply({'error':'此车还未配置新场地面板地址'},status=404)
                elif fleet.paired:
                    self.reply({'error':'先切回单车模式'},status=409)
                else:
                    self.send_response(303)
                    self.send_header('Location',site_urls[i])
                    self.send_header('Content-Length','0')
                    self.end_headers()
            else:
                self.send_error(404)

        def do_POST(self):
            if not self.local():
                return
            if self.path!='/api/fleet/mode':
                self.send_error(404)
                return
            if not secrets.compare_digest(self.headers.get('X-Control-Token','').encode(),fleet.token.encode()):
                self.reply({'error':'管理会话已变更，请刷新页面'},status=403)
                return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 1<=length<=256:
                    raise ValueError('请求大小无效')
                value=json.loads(self.rfile.read(length))
                if not isinstance(value,dict) or type(value.get('paired')) is not bool:
                    raise ValueError('运行模式无效')
                fleet.set_paired(value['paired'])
            except (ValueError, TypeError, KeyError) as error:
                self.reply({'error':str(error)},status=409)
                return
            self.reply({**fleet.status(),'site_available':[bool(u) for u in site_urls]})

        def log_message(self, *_):
            pass

    stopped = threading.Event()
    with ExitStack() as stack:
        browser = stack.enter_context(ThreadingHTTPServer(('127.0.0.1',ports[0]),Dashboard))
        for port,panel in zip(ports[1:],fleet.panels):
            web = stack.enter_context(ThreadingHTTPServer(('127.0.0.1',port),handler(None,desktop=panel)))
            threading.Thread(target=web.serve_forever,daemon=True).start()
            stack.callback(web.shutdown)
            threading.Thread(target=panel.monitor,args=(stopped,),daemon=True).start()
        threading.Thread(target=fleet.maintain,args=(stopped,),daemon=True).start()
        try:
            browser.serve_forever()
        finally:
            stopped.set()


if __name__ == '__main__':
    main()
