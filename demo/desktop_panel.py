#!/usr/bin/env python3
"""Windows panel for the configured RDK, with a saved map available offline."""
import argparse
import base64
from dataclasses import dataclass
from http.client import HTTPConnection, HTTPException
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import struct
import sys
import threading
from urllib.parse import urlsplit

from map_grid import MapGrid
from server import handler


JSON = 'application/json; charset=utf-8'
LIVE_PAGES = {
    '/api/collection/export', '/api/collection/bundle', '/api/collection/servo/export',
    '/api/collection/c5/export', '/api/broadcast/data', '/api/broadcast/export',
    '/api/survey/route', '/api/export', '/rssi',
}
NETWORK_ERRORS = (OSError, HTTPException)


@dataclass(frozen=True)
class SavedMap:
    info: dict
    image: bytes
    dock: list | None

    def __post_init__(self):
        if (not isinstance(self.info, dict) or not self.info.get('map_id')
                or not self.image.startswith(b'\x89PNG\r\n\x1a\n') or len(self.image) < 24):
            raise ValueError('无效的场地地图')
        # The panel image uses the same display rotation as MapGrid.png().
        if struct.unpack_from('>II', self.image, 16) != (self.info['height'], self.info['width']):
            raise ValueError('地图和图像尺寸不一致')


class DesktopPanel:
    def __init__(self, robot_url, data):
        self.target = urlsplit(robot_url)
        if (self.target.scheme != 'http' or not self.target.hostname
                or self.target.username is not None or self.target.path not in ('', '/')
                or self.target.query or self.target.fragment or self.target.port == 0):
            raise ValueError('车端面板地址应为 http://主机:端口/')
        self.data = Path(data)
        self.data.mkdir(parents=True, exist_ok=True)
        self.cache = self.data / 'map_snapshot.json'
        self.lock = threading.Lock()
        self.live_state = None
        self.saved_map = self.load_map()
        if self.saved_map is None and not self.refresh():
            raise ValueError('找不到已保存的实景地图。请提供现场地图或先连上车端巡航面板。')

    def load_map(self):
        try:
            record = json.loads(self.cache.read_text(encoding='utf-8'))
            image = base64.b64decode(record['png'], validate=True)
            return SavedMap(record['map'], image, record['dock'])
        except (OSError, ValueError, KeyError, TypeError):
            scene = self.data / 'scene'
            if not (scene / 'map.yaml').is_file():
                return None
            grid = MapGrid(scene)
            dock_file = scene / 'dock_start.json'
            dock = None
            if dock_file.is_file():
                settings = json.loads(dock_file.read_text(encoding='utf-8'))
                if settings.get('calibrated', True):
                    dock = [settings['robot_pose'][key] for key in ('x', 'y', 'yaw')]
            return SavedMap(grid.info(), grid.png(), dock)

    def save_map(self, saved):
        record = {'map': saved.info, 'png': base64.b64encode(saved.image).decode('ascii'),
                  'dock': saved.dock}
        pending = self.cache.with_suffix('.tmp')
        pending.write_text(json.dumps(record, ensure_ascii=False, allow_nan=False), encoding='utf-8')
        pending.replace(self.cache)

    def request(self, method, path, *, body=None, token=None, timeout=1):
        connection = HTTPConnection(self.target.hostname, self.target.port, timeout=timeout)
        headers = {}
        if token is not None:
            headers = {'Content-Type': 'application/json', 'X-Control-Token': token}
        try:
            connection.request(method, path, body, headers)
            with connection.getresponse() as response:
                return response.read(), response.status, response.getheader('Content-Type', JSON)
        finally:
            connection.close()

    def disconnect(self):
        with self.lock:
            self.live_state = None

    def refresh(self):
        try:
            data, status, _ = self.request('GET', '/api/state')
            if status != 200:
                raise ValueError('车端状态不可用')
            state = json.loads(data)
            if state['health']['mode'] != 'robot' or not isinstance(state['control_token'], str) or not state['control_token']:
                raise ValueError('车端控制会话不可用')
            with self.lock:
                previous = self.saved_map
            info = state['map']
            dock = state.get('dock') if state['health'].get('dock_calibrated',True) else None
            if previous is None or info != previous.info:
                image, code, kind = self.request('GET', '/map.png')
                if code != 200 or kind.split(';', 1)[0] != 'image/png':
                    raise ValueError('车端地图图像不可用')
            else:
                image = previous.image
            saved = SavedMap(info, image, dock)
        except (*NETWORK_ERRORS, ValueError, KeyError, TypeError):
            self.disconnect()
            return False

        state['vehicle_connected'] = True
        with self.lock:
            self.saved_map = saved
            self.live_state = state
        if previous != saved or not self.cache.exists():
            try:
                self.save_map(saved)
            except OSError as exc:
                print(f'地图缓存保存失败：{exc}', file=sys.stderr)
        return True

    def monitor(self, stopped):
        while not stopped.is_set():
            online = self.refresh()
            stopped.wait(0.35 if online else 1)

    def view_state(self):
        site_url = (f'http://{self.target.hostname}:8896/' if self.target.hostname not in
                    ('127.0.0.1','localhost','::1') else None)
        with self.lock:
            if self.live_state is not None:
                return {**self.live_state,'site_url':site_url}
            saved = self.saved_map
        return {
            'vehicle_connected': False, 'phase': 'disconnected','site_url':site_url,
            'message': '实车未连接，可查看已保存的场地地图。',
            'health': {'mode': 'robot', 'ready': False, 'pose': None,
                       'readiness': '导航、定位和采集离线'},
            'map': saved.info, 'dock': saved.dock, 'points': [], 'run': [],
            'index': 0, 'remaining': None, 'events': [],
            'collection': {'mode': 'timed', 'stage': 'disabled'}, 'c5': {},
        }

    def map_image(self, map_id=None):
        with self.lock:
            if map_id is None or self.saved_map.info['map_id'] == map_id:
                return self.saved_map.image
        return None

    def live_get(self, path):
        if path not in LIVE_PAGES:
            return {'error': '不存在的页面'}, 404, JSON
        with self.lock:
            connected = self.live_state is not None
        if not connected:
            return {'error': '实车未连接'}, 503, JSON
        try:
            return self.request('GET', path, timeout=15)
        except NETWORK_ERRORS:
            self.disconnect()
            return {'error': '实车连接中断'}, 503, JSON

    def command(self, data, token):
        with self.lock:
            state = self.live_state
        if state is None:
            return {'error': '实车未连接'}, 503, JSON
        if not secrets.compare_digest(token.encode('utf-8'), state['control_token'].encode('utf-8')):
            return {'error': '实车控制会话已变更，请等待页面刷新'}, 403, JSON
        try:
            return self.request('POST', '/api/command', body=data, token=token, timeout=8)
        except NETWORK_ERRORS:
            self.disconnect()
            return {'error': '实车连接中断，操作结果未知，请核对车端状态'}, 503, JSON


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-panel', required=True)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8891)
    args = parser.parse_args()
    panel = DesktopPanel(args.robot_panel, args.data)
    stopped = threading.Event()
    address = ('127.0.0.1', args.port)
    with ThreadingHTTPServer(address, handler(None, desktop=panel)) as web:
        threading.Thread(target=panel.monitor, args=(stopped,), daemon=True).start()
        print(f'R550 desktop panel: http://{address[0]}:{web.server_port}/', flush=True)
        try:
            web.serve_forever()
        finally:
            stopped.set()


if __name__ == '__main__':
    main()
