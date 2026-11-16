import csv
from http.server import ThreadingHTTPServer
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import zipfile

from test_demo import Clock, ControlledBackend, ROOT
from test_servo_car import ServoPeer

sys.path.insert(0, str(ROOT.parent / 'tools'))
from c5_collector import handler, verify_capture
from c5_station import CollectorLink
from map_grid import MapGrid
from mission import Mission
from servo_car import MODE


RUN = ['02:00:00:00:00:01', 123, 7]
TAG = '02:00:00:00:00:02'
VISIT = '3' * 32


class CaptureVerificationTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.folder = self.directory / 'test-capture'
        self.folder.mkdir()

    def capture(self, plan, *, run_number=7, valid=True, completed=16):
        done = dict(zip(('root_mac', 'root_boot', 'run'), RUN))
        done.update(reason='duration' if plan == 'fixed' else 'group_limit', completed_groups=completed)
        result = {
            'capture_ok': True, 'device_done': True, 'done_event': done, 'selected_run': RUN,
            'quality': {'scan': {'runs': [{'joint_state_confirmed_groups': completed}]}},
        }
        (self.folder / 'result.json').write_text(json.dumps(result), encoding='utf-8')
        name = 'csi_packets_confirmed.csv' if plan == 'variable' else 'csi_packets.csv'
        with (self.folder / name).open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(('label', 'root_mac', 'root_boot', 'run', 'tx_role', 'tx_mac', 'rx_mac', 'csi_valid', 'csi_base64'))
            writer.writerow((VISIT, RUN[0], RUN[1], run_number, 'tag', TAG, RUN[0], valid, 'AQI='))

    def test_fixed_tag_data_and_matching_run_required(self):
        self.capture('fixed')
        self.assertEqual(verify_capture(self.directory, 'fixed', 16, VISIT, TAG)['valid_by_anchor'], {RUN[0]: 1})
        self.capture('fixed', run_number=8)
        with self.assertRaisesRegex(ValueError, '会话或停靠编号'):
            verify_capture(self.directory, 'fixed', 16, VISIT, TAG)
        self.capture('fixed', valid=False)
        with self.assertRaisesRegex(ValueError, '有效 CSI'):
            verify_capture(self.directory, 'fixed', 16, VISIT)

    def test_variable_requires_requested_groups(self):
        self.capture('variable', completed=15)
        with self.assertRaisesRegex(ValueError, '15/16'):
            verify_capture(self.directory, 'variable', 16, VISIT)
        self.capture('variable')
        self.assertEqual(verify_capture(self.directory, 'variable', 16, VISIT)['groups'], 16)


class FakeCollector:
    """Collector HTTP peer with independently controlled completion."""
    def __init__(self):
        self.visits = []
        self.state = 'sampling'

    def status(self):
        return {'ready': True, 'plan': 'fixed'}

    def start(self, visit):
        self.state = 'sampling'
        self.visits.append(visit)
        return self.receipt(visit['visit_id'])

    def receipt(self, identity, cancel=False):
        if cancel:
            self.state = 'canceled'
        return {'visit_id': identity, 'state': self.state,
                'result': {'plan': 'fixed', 'run': RUN, 'valid_csi': 42}}


class C5StationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.grid = MapGrid(ROOT.parent / 'scenes' / 'demo-room')

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.peer = FakeCollector()
        token = 'a' * 32
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler(self.peer, token))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        link = CollectorLink(f'http://127.0.0.1:{server.server_port}', token)
        config = patch.object(CollectorLink, 'configured', return_value=link)
        config.start()
        self.addCleanup(config.stop)
        self.backend = ControlledBackend()
        self.mission = Mission(self.grid, self.backend, directory.name, Clock())
        self.addCleanup(self.mission.collection.close)
        self.addCleanup(self.mission.c5.close)
        self.mission.save_points([
            {'id': 'a', 'name': 'A', 'x': .4, 'y': -2., 'yaw': 0, 'wait': 0},
            {'id': 'b', 'name': 'B', 'x': .4, 'y': -3., 'yaw': 0, 'wait': 0},
        ])

    def wait(self, check):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            if check():
                return
            time.sleep(.02)
        self.fail(f'C5 state: {self.mission.c5.snapshot()}')

    def arrive(self):
        self.backend.pose = [.42, -2.03, .04]
        self.backend.ops[self.mission.operation]['status'] = 'succeeded'
        self.mission.tick()

    def wait_for_saved(self):
        self.peer.state = 'saved'
        self.wait(lambda: self.mission.c5.snapshot()['stage'] == 'done')

    def test_stopped_pose_is_saved_and_pause_holds_after_capture(self):
        mission = self.mission
        mission.configure_collection('timed', c5=True)
        mission.start()
        self.backend.stationary = False
        self.arrive()
        self.assertEqual(mission.phase, 'settling')
        self.assertFalse(self.peer.visits)
        self.backend.stationary = True
        mission.tick()
        self.wait(lambda: len(self.peer.visits) == 1)
        self.assertEqual(self.peer.visits[0]['actual_pose'], self.backend.pose)
        mission.tick()
        self.assertEqual(len(self.backend.calls), 1)
        mission.pause()
        self.wait_for_saved()
        mission.tick()
        self.assertEqual(len(self.backend.calls), 1)
        mission.resume()
        mission.tick()
        self.assertEqual(self.backend.calls[-1][0]['id'], 'b')
        visit_id = self.peer.visits[0]['visit_id']
        with zipfile.ZipFile(io.BytesIO(mission.c5.export(mission.collection.directory / 'maps'))) as archive:
            record = json.loads(archive.read(f'visits/{visit_id}.json'))
            self.assertIn('released_utc', record)
            self.assertIn(f'maps/{self.grid.map_id}/map.yaml', archive.namelist())

    def test_ws63_permit_waits_for_c5_and_exports_shared_visit(self):
        mission = self.mission
        servo = ServoPeer()
        mission.collection.factory = lambda *args, **kwargs: servo
        mission.configure_collection(MODE, '/dev/ttyUSB_TEST', c5=True)
        self.wait(lambda: mission.collection.snapshot()['stage'] == 'ready')
        mission.start(identity='a')
        self.arrive()
        self.wait(lambda: self.peer.visits and servo.starts)
        servo.finish()
        self.wait(lambda: mission.collection.snapshot()['stage'] == 'done')
        mission.tick()
        self.assertEqual(mission.phase, 'collecting')
        self.assertTrue(mission.collection.snapshot()['motion_blocked'])
        self.wait_for_saved()
        mission.tick()
        self.assertEqual(mission.phase, 'completed')
        rows = mission.collection.servo_store.export(False).splitlines()
        self.assertEqual({json.loads(row)['visit_id'] for row in rows}, {self.peer.visits[0]['visit_id']})

    def test_failed_capture_cannot_advance_or_auto_return(self):
        mission = self.mission
        mission.configure_collection('timed', c5=True)
        mission.start(auto_return=True)
        self.arrive()
        self.wait(lambda: self.peer.visits)
        self.peer.state = 'failed'
        self.wait(lambda: mission.c5.snapshot()['stage'] == 'error')
        mission.tick()
        self.assertEqual(mission.phase, 'failed')
        self.assertEqual(len(self.backend.calls), 1)

    def test_stop_cancels_current_capture_and_keeps_robot_stationary(self):
        mission = self.mission
        mission.configure_collection('timed', c5=True)
        mission.start(auto_return=True)
        self.arrive()
        self.wait(lambda: self.peer.visits)
        with self.assertRaises(ValueError):
            mission.return_to_dock()
        mission.stop()
        self.wait(lambda: mission.c5.snapshot()['stage'] == 'canceled')
        mission.tick()
        self.assertEqual(mission.phase, 'idle')
        self.assertEqual(len(self.backend.calls), 1)
