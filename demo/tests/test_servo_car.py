"""Exercise the rotation protocol, its durable release gate, and the mission."""
import csv
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

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from collection import Collection
from map_grid import MapGrid
from mission import Mission
from servo_car import MODE
from test_demo import Clock, ControlledBackend, ROOT


class ServoPeer:
    def __init__(self):
        self.buffer = bytearray()
        self.lock = threading.RLock()
        self.round_id, self.position_id, self.state = 41, 0, 0
        self.link_ready, self.mask = 1, 0
        self.hello = '+CARHELLO:SERVO_CAR_2'
        self.station = None
        self.starts, self.commands = [], []
        self.drop_ready_ok = 0
        self.drop_move = 0
        self.before_ack = None
        self.disconnected = False

    def emit(self, *lines):
        with self.lock:
            self.buffer.extend(('\r\n'.join(lines) + '\r\n').encode('ascii'))

    def write(self, data):
        with self.lock:
            command = data.decode('ascii').removesuffix('\r\n')
            assert data.endswith(b'\r\n')
            self.commands.append(command)
            if command == 'AT+CARHELLO':
                self.emit(command, '[boot] uart ready', self.hello, 'OK')
            elif command == 'AT+CARSTATUS':
                self.emit(f'+CARSTATUS:{self.round_id},{self.position_id},{self.state},{self.link_ready},{self.mask}', 'OK')
            elif command.startswith('AT+CARREADY='):
                station = tuple(map(int, command.partition('=')[2].split(',')))
                if self.station != station:
                    assert station[0] > self.round_id
                    self.station = station
                    self.round_id, self.position_id = station[:2]
                    self.state, self.mask = 1, 0
                    self.starts.append(station)
                if self.drop_ready_ok:
                    self.drop_ready_ok -= 1
                else:
                    self.emit('OK')
            elif command == f'AT+CARACK={self.round_id}' and self.state in (2, 3):
                if self.before_ack:
                    self.before_ack(self.round_id)
                self.state = 3
                if self.drop_move:
                    self.drop_move -= 1
                    self.emit('OK')
                else:
                    self.emit(f'+CARMOVE:{self.round_id},{self.position_id}', 'OK')
            else:
                self.emit('ERROR')
            return len(data)

    def read(self, count):
        if self.disconnected:
            raise OSError('test cable unplugged')
        with self.lock:
            data = bytes(self.buffer[:min(count, 7)])
            del self.buffer[:len(data)]
        if not data:
            time.sleep(.002)
        return data

    def flush(self):
        pass

    def close(self):
        pass

    def result(self, node, station=None, rssi=None):
        station = self.station if station is None else station
        rssi = ('NA' if node else '-81.5') if rssi is None else rssi
        return '+CARRESULT:' + ','.join(map(str, (
            *station, node, 180 * node, 0, 4294967300, 4294968300, 1000,
            5000000000 if node == 0 else 0, 16 if node == 0 else 0, rssi,
            9 if node == 0 else 0, 64000000000 if node == 0 else 0, 0,
        )))

    def finish(self, *extra):
        with self.lock:
            self.state, self.mask = 2, 3
            self.emit(self.result(0), self.result(1),
                      f'+CARROUNDEND:{self.round_id},{self.position_id},2', *extra)


class ServoCarTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.grid = MapGrid(ROOT.parent / 'scenes' / 'demo-room')

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.peer = ServoPeer()
        self.backend, self.clock = ControlledBackend(), Clock()
        self.mission = Mission(self.grid, self.backend, self.temp.name, self.clock)
        self.collection = self.mission.collection
        self.collection.factory = lambda *args, **kwargs: self.peer
        self.addCleanup(self.collection.close)
        self.mission.save_points([
            {'id': point, 'name': point, 'x': .4, 'y': y, 'yaw': 0, 'wait': 0, 'enabled': True}
            for point, y in [('a', -2.), ('b', -3.)]
        ])
        self.collection.configure(MODE, '/dev/ttyUSB_TEST')
        self.wait(lambda: self.collection.snapshot()['stage'] == 'ready')

    def wait(self, condition, seconds=5):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if condition():
                return
            time.sleep(.01)
        self.fail(f'等待测试状态超时：{self.collection.snapshot()}')

    def arrive(self, pose):
        self.backend.pose = pose
        self.backend.ops[self.mission.operation]['status'] = 'succeeded'
        self.mission.tick()

    def start_one(self):
        self.mission.start(identity='a')
        self.arrive([.42, -2.03, .04])
        self.wait(lambda: len(self.peer.starts) == 1)

    def records(self):
        return [json.loads(line) for line in self.collection.servo_store.export(False).splitlines()]

    def test_two_stations_only_depart_after_durable_ack_and_correlated_move(self):
        self.mission.start()
        self.backend.stationary = False
        self.arrive([.42, -2.03, .04])
        self.assertEqual(self.peer.starts, [])
        self.backend.stationary = True
        self.mission.tick()
        self.wait(lambda: len(self.peer.starts) == 1)
        self.assertEqual(self.peer.station, (42, 1, 420, -2030))
        old = (41, 1, 420, -2030)
        self.peer.emit(self.peer.result(0, old), self.peer.result(0), self.peer.result(0),
                       '+CARROUNDEND:42,1,2', '+CARMOVE:42,1', 'OK')
        self.wait(lambda: len(self.records()) == 1)
        self.clock.advance(1000)
        self.mission.tick()
        self.assertEqual(len(self.backend.calls), 1)
        self.assertFalse(any(command.startswith('AT+CARACK') for command in self.peer.commands))
        with self.assertRaisesRegex(ValueError, '尚未放行'):
            self.mission.return_to_dock()

        def check_disk_before_ack(round_id):
            with self.collection.servo_store.database() as db:
                state = db.execute('SELECT state,ended FROM rounds WHERE round_id=?', (round_id,)).fetchone()
                self.assertEqual(tuple(state), ('await_move', 1))
                self.assertEqual(db.execute('SELECT COUNT(*) FROM nodes WHERE round_id=?', (round_id,)).fetchone()[0], 2)

        self.peer.before_ack = check_disk_before_ack
        self.peer.drop_move = 10
        self.peer.finish('+CARMOVE:42,1')
        self.wait(lambda: 'AT+CARACK=42' in self.peer.commands)
        self.mission.tick()
        self.assertEqual(len(self.backend.calls), 1)
        self.mission.pause()
        self.peer.emit('+CARMOVE:41,1', '+CARMOVE:42,1', '+CARMOVE:42,1')
        self.wait(lambda: self.collection.snapshot()['stage'] == 'done')
        self.mission.tick()
        self.assertEqual(len(self.backend.calls), 1)
        self.mission.resume()
        self.mission.tick()
        self.mission.tick()
        self.assertEqual(len(self.backend.calls), 2)
        self.assertEqual(self.backend.calls[-1][0]['id'], 'b')

        self.arrive([.39, -3.01, -.03])
        self.wait(lambda: len(self.peer.starts) == 2)
        self.assertEqual(self.peer.station, (43, 2, 390, -3010))
        self.peer.drop_move = 0
        self.peer.finish()
        self.wait(lambda: self.collection.snapshot()['stage'] == 'done')
        self.mission.tick()
        self.assertEqual(self.mission.phase, 'completed')
        self.assertEqual(len(self.backend.calls), 2)
        with zipfile.ZipFile(io.BytesIO(self.collection.servo_store.export(True))) as archive:
            rows = list(csv.DictReader(io.StringIO(archive.read('servo_results.csv').decode('utf-8-sig'))))
            self.assertEqual(len(rows), 4)
            self.assertEqual(rows[0]['bps'], '64000000000')
            self.assertEqual(rows[1]['rssi_dbm'], 'NA')
            self.assertEqual(rows[1]['packets'], '0')
            self.assertTrue(all(row['round_status'] == 'complete' for row in rows))
            rounds = json.loads(archive.read('rounds.json'))
            self.assertEqual([row['permit_consumed'] for row in rounds], [1, 1])
            self.assertIn(f'maps/{self.grid.map_id}/map.yaml', archive.namelist())

    def test_lost_ready_and_move_repeat_identical_commands(self):
        self.peer.drop_ready_ok = self.peer.drop_move = 1
        self.start_one()
        ready = 'AT+CARREADY=42,1,420,-2030'
        self.wait(lambda: self.peer.commands.count(ready) == 2)
        self.assertEqual(len(self.peer.starts), 1)
        self.peer.finish()
        self.wait(lambda: self.collection.snapshot()['stage'] == 'done')
        self.assertEqual(self.peer.commands.count('AT+CARACK=42'), 2)

    def test_one_retry_after_peer_failure_keeps_position_and_partial_data(self):
        self.start_one()
        self.peer.emit(self.peer.result(0))
        self.wait(lambda: len(self.records()) == 1)
        self.peer.state, self.peer.link_ready = 4, 0
        self.peer.emit('+CARERROR:42,1,SLE_LOST')
        self.wait(lambda: self.records()[0]['round_status'] == 'failed')
        self.assertEqual(len(self.peer.starts), 1)
        self.peer.link_ready = 1
        self.wait(lambda: len(self.peer.starts) == 2)
        self.assertEqual(self.peer.starts, [(42, 1, 420, -2030), (43, 1, 420, -2030)])
        self.peer.state = 4
        self.peer.emit('+CARERROR:43,1,SLE_LOST')
        self.wait(lambda: self.collection.snapshot()['stage'] == 'error')
        self.assertEqual(len(self.peer.starts), 2)
        self.assertTrue(self.collection.snapshot()['motion_blocked'])

    def test_conflicting_result_preserves_first_node_and_never_acks(self):
        self.start_one()
        self.peer.emit(self.peer.result(0), self.peer.result(0, rssi='-88'))
        self.wait(lambda: self.collection.snapshot()['stage'] == 'error')
        self.assertEqual(self.records()[0]['rssi_dbm'], -81.5)
        self.assertEqual(len(self.records()), 1)
        self.assertNotIn('AT+CARACK=42', self.peer.commands)

    def test_failed_storage_prevents_ack(self):
        self.start_one()
        with patch.object(self.collection.servo_store, 'save_node', side_effect=OSError('test disk fault')):
            self.peer.finish()
            self.wait(lambda: self.collection.snapshot()['stage'] == 'error')
        self.assertNotIn('AT+CARACK=42', self.peer.commands)

    def test_unplug_then_restart_requires_peer_idle_and_keeps_round_high_water(self):
        self.start_one()
        self.peer.emit(self.peer.result(0))
        self.wait(lambda: len(self.records()) == 1)
        self.peer.disconnected = True
        self.wait(lambda: self.collection.snapshot()['stage'] == 'error')
        self.collection.close()
        restarted = Collection(self.temp.name, self.grid, serial_factory=lambda *a, **k: self.peer)
        self.addCleanup(restarted.close)
        with self.assertRaisesRegex(ValueError, '尚未放行'):
            restarted.prepare(self.mission.points)
        self.peer.disconnected = False
        restarted.configure(MODE, '/dev/ttyUSB_TEST')
        self.wait(lambda: restarted.snapshot()['stage'] == 'blocked')
        self.peer.round_id, self.peer.position_id, self.peer.state, self.peer.mask = 0, 0, 0, 0
        self.peer.station = None
        restarted.configure(MODE, '/dev/ttyUSB_TEST')
        self.wait(lambda: restarted.snapshot()['stage'] == 'ready')
        target = self.mission.points[0]
        restarted.prepare([target])
        restarted.begin(target, self.backend.snapshot())
        self.wait(lambda: len(self.peer.starts) == 2)
        self.assertEqual(self.peer.station, (43, 1, 420, -2030))

    def test_old_firmware_cannot_start_rotation(self):
        self.peer.hello = '+CARHELLO:SERVO_CAR_1'
        self.collection.configure(MODE, '/dev/ttyUSB_TEST')
        self.wait(lambda: self.collection.snapshot()['stage'] == 'error')
        with self.assertRaises(ValueError):
            self.mission.start()
        self.assertEqual(self.backend.calls, [])
        self.assertEqual(self.peer.starts, [])


if __name__ == '__main__':
    unittest.main()
