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
from unittest.mock import patch

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1]))

from desktop_panel import CONTROL_MAX_AGE, OFFLINE_AFTER, DesktopPanel
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
    def test_transient_loss_keeps_selection_data_but_only_allows_stop(self):
        fixture = HERE.parents[2] / 'scenes' / 'demo-room'
        with TemporaryDirectory() as temporary:
            data = Path(temporary)
            shutil.copytree(fixture, data / 'scene')
            panel = DesktopPanel('http://127.0.0.1:1', data)
            state = {'health': {'mode': 'robot', 'pose': [1, 2, 0]}, 'phase': 'idle',
                     'control_token': 'session-a', 'map':panel.saved_map.info, 'points':[{'id':'one'}]}
            reply = (json.dumps(state).encode(), 200, 'application/json')
            with patch('desktop_panel.time.monotonic', return_value=10.) as clock, patch.object(panel,'request') as send:
                send.return_value = reply
                self.assertTrue(panel.refresh())
                self.assertEqual(panel.view_state()['connection_status'], 'online')
                send.side_effect = TimeoutError()
                clock.return_value = 12.
                self.assertFalse(panel.refresh())
                pending = panel.view_state()
                self.assertEqual(pending['connection_status'], 'reconnecting')
                self.assertFalse(pending['vehicle_connected'])
                self.assertEqual(pending['points'], state['points'])
                self.assertIsNone(pending['health']['pose'])
                self.assertNotIn('control_token', pending)
                send.reset_mock()
                self.assertEqual(panel.command(b'{"action":"start"}','session-a')[1],503)
                self.assertEqual(panel.command(b'{"action":"stop"}','wrong')[1],403)
                send.assert_not_called()
                send.side_effect = None
                self.assertEqual(panel.command(b'{"action":"stop"}',pending['stop_token'])[1],200)
                send.assert_called_once()
                clock.return_value = 10.+OFFLINE_AFTER
                offline = panel.view_state()
                self.assertEqual(offline['connection_status'], 'offline')
                self.assertNotIn('stop_token', offline)
                self.assertEqual(panel.command(b'{"action":"stop"}','session-a')[1],503)
                send.side_effect = [reply]
                self.assertTrue(panel.refresh())
                self.assertEqual(panel.view_state()['connection_status'], 'online')
                clock.return_value += CONTROL_MAX_AGE+.1
                self.assertFalse(panel.view_state()['vehicle_connected'])
                self.assertEqual(panel.command(b'{"action":"start"}','session-a')[1],503)

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
