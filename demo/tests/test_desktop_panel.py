"""Exercise the desktop panel with a loopback stand-in and the generated map."""
from contextlib import contextmanager
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import shutil
import sys
from tempfile import TemporaryDirectory
import threading
from types import SimpleNamespace
import unittest

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1]))

from desktop_panel import DesktopPanel
from map_grid import MapGrid
from server import handler


@contextmanager
def serving(web):
    worker = threading.Thread(target=web.serve_forever, daemon=True)
    worker.start()
    try:
        yield
    finally:
        web.shutdown()
        worker.join()


def request(web, path, *, body=None, headers=None):
    connection = HTTPConnection('127.0.0.1', web.server_port, timeout=3)
    try:
        connection.request('POST' if body is not None else 'GET', path, body, headers or {})
        with connection.getresponse() as response:
            return response.status, response.read()
    finally:
        connection.close()


class DesktopPanelTests(unittest.TestCase):
    def test_real_map_stays_available_and_only_live_session_can_send_commands(self):
        fixture = HERE.parents[2] / 'scenes' / 'demo-room'
        grid = MapGrid(fixture)
        payload = b'{"action":"stop"}'
        calls = []
        live_map = dict(grid.info(), map_id='fixture-live-map', version='test connected scene')
        vehicle = SimpleNamespace(
            token='stand-in-token',
            image=grid.png(),
            view_state=lambda: {'health': {'mode': 'robot'}, 'map': live_map, 'dock': [0, 0, 0]},
            command=lambda action: calls.append(action) or {'phase': 'idle'},
        )
        with TemporaryDirectory() as temporary:
            data = Path(temporary)
            shutil.copytree(fixture, data / 'scene')
            with ThreadingHTTPServer(('127.0.0.1', 0), handler(vehicle)) as peer:
                robot_url = f'http://127.0.0.1:{peer.server_port}/'
                panel = DesktopPanel(robot_url, data)
                with ThreadingHTTPServer(('127.0.0.1', 0), handler(None, desktop=panel)) as local:
                    with serving(local):
                        code, state_body = request(local, '/api/state')
                        self.assertEqual(code, 200)
                        state = json.loads(state_body)
                        self.assertFalse(state['vehicle_connected'])
                        self.assertIsNone(state['health']['pose'])
                        self.assertEqual(state['map']['map_id'], grid.map_id)
                        self.assertNotIn('control_token', state)
                        self.assertEqual(request(local, '/map.png?map_id=' + grid.map_id), (200, grid.png()))
                        self.assertEqual(request(local, '/api/command', body=payload)[0], 503)
                        self.assertEqual(request(local, '/api/state', headers={'Host': 'invalid.local'})[0], 421)

                        with serving(peer):
                            self.assertTrue(panel.refresh())
                            state = json.loads(request(local, '/api/state')[1])
                            self.assertTrue(state['vehicle_connected'])
                            self.assertEqual(state['map'], live_map)
                            self.assertEqual(request(local, '/map.png?map_id=' + grid.map_id)[0], 404)
                            self.assertEqual(request(local, '/map.png?map_id=fixture-live-map'), (200, grid.png()))
                            self.assertEqual(request(local, '/api/command', body=payload)[0], 403)
                            self.assertEqual(request(local, '/api/command', body=payload,
                                                     headers={'X-Control-Token': state['control_token']})[0], 200)
                            self.assertEqual(calls, [{'action': 'stop'}])

                        peer.server_close()
                        self.assertFalse(panel.refresh())
                        state = json.loads(request(local, '/api/state')[1])
                        self.assertFalse(state['vehicle_connected'])
                        self.assertEqual(state['map'], live_map)
                        self.assertNotIn('control_token', state)
                        self.assertNotIn('stand-in-token', panel.cache.read_text(encoding='utf-8'))
                        self.assertEqual(request(local, '/api/command', body=payload,
                                                 headers={'X-Control-Token': 'stand-in-token'})[0], 503)
                        self.assertEqual(calls, [{'action': 'stop'}])
                        self.assertEqual(DesktopPanel(robot_url, data).view_state()['map'], live_map)
