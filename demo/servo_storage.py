"""Durable SERVO_CAR_2 rounds and portable measurement export."""
from contextlib import closing, contextmanager
import csv
from datetime import datetime, timezone
import io
import json
from pathlib import Path
import sqlite3
import zipfile

from rdk_link import ProtocolError


RESULT_FIELDS = (
    'round_id', 'position_id', 'x_mm', 'y_mm', 'node', 'horizontal_deg',
    'vertical_deg', 'start_us', 'end_us', 'duration_us', 'bytes', 'packets',
    'rssi_dbm', 'rssi_count', 'bps', 'payload_bps',
)
MAX_ROUND = 2**31 - 1


def utc_now():
    return datetime.now(timezone.utc).isoformat()


class ServoStore:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.path = self.directory / 'servo_car_v2.sqlite3'
        self.blocked = False
        if self.path.exists():
            with self.database() as db:
                self.blocked = db.execute('SELECT 1 FROM rounds WHERE locked=1').fetchone() is not None

    @contextmanager
    def database(self):
        with closing(sqlite3.connect(self.path, timeout=2)) as db:
            db.row_factory = sqlite3.Row
            db.execute('PRAGMA synchronous=FULL')
            db.execute('PRAGMA foreign_keys=ON')
            with db:
                yield db

    def setup(self):
        with self.database() as db:
            db.executescript("""
                CREATE TABLE IF NOT EXISTS rounds (
                    round_id INTEGER PRIMARY KEY,
                    station TEXT NOT NULL,
                    started_utc TEXT NOT NULL,
                    state TEXT NOT NULL,
                    detail TEXT,
                    ended INTEGER NOT NULL DEFAULT 0,
                    locked INTEGER NOT NULL DEFAULT 1,
                    permit_consumed INTEGER NOT NULL DEFAULT 0
                );
                CREATE UNIQUE INDEX IF NOT EXISTS one_locked_round ON rounds(locked) WHERE locked=1;
                CREATE TABLE IF NOT EXISTS nodes (
                    round_id INTEGER NOT NULL REFERENCES rounds(round_id),
                    node INTEGER NOT NULL CHECK (node IN (0, 1)),
                    result TEXT NOT NULL,
                    car_receive_time TEXT NOT NULL,
                    PRIMARY KEY (round_id, node)
                );
            """)

    def reserve(self, peer_round, station, replaces=None):
        with self.database() as db:
            db.execute('BEGIN IMMEDIATE')
            if replaces is not None:
                updated = db.execute(
                    "UPDATE rounds SET locked=0 WHERE round_id=? AND state='failed' AND locked=1",
                    (replaces,),
                )
                if updated.rowcount != 1:
                    raise ProtocolError('前一轮未确认失败，不能分配重测轮次')
            if db.execute('SELECT 1 FROM rounds WHERE locked=1').fetchone():
                raise ProtocolError('有未放行的轮次，请先恢复联调状态')
            last_local = db.execute('SELECT COALESCE(MAX(round_id), 0) FROM rounds').fetchone()[0]
            round_id = max(peer_round, last_local) + 1
            if round_id > MAX_ROUND:
                raise ProtocolError('轮次编号已达上限，请与固件方核对编号范围')
            db.execute(
                'INSERT INTO rounds(round_id,station,started_utc,state) VALUES(?,?,?,?)',
                (round_id, json.dumps(station, ensure_ascii=False, allow_nan=False), utc_now(), 'sampling'),
            )
        self.blocked = True
        return round_id

    def save_node(self, result):
        with self.database() as db:
            key = (result['round_id'], result['node'])
            previous = db.execute('SELECT result FROM nodes WHERE round_id=? AND node=?', key).fetchone()
            if previous is not None:
                if json.loads(previous['result']) != result:
                    raise ProtocolError(f"同一轮的节点 {result['node']} 重发内容冲突")
                return
            db.execute(
                'INSERT INTO nodes VALUES(?,?,?,?)',
                (*key, json.dumps(result, allow_nan=False), utc_now()),
            )

    def note_end(self, round_id):
        with self.database() as db:
            db.execute('UPDATE rounds SET ended=1 WHERE round_id=?', (round_id,))

    def arm_ack(self, round_id):
        with self.database() as db:
            # Each node commit and this state commit finish before CARACK leaves the host.
            updated = db.execute(
                """UPDATE rounds SET state='await_move' WHERE round_id=? AND state='sampling'
                   AND locked=1 AND ended=1 AND
                   (SELECT COUNT(*) FROM nodes WHERE round_id=?)=2""",
                (round_id, round_id),
            )
            if updated.rowcount != 1:
                raise ProtocolError('结果尚未完整写盘，不能发送 CARACK')

    def complete(self, round_id):
        with self.database() as db:
            updated = db.execute(
                "UPDATE rounds SET state='complete' WHERE round_id=? AND state='await_move' AND locked=1",
                (round_id,),
            )
            if updated.rowcount != 1:
                raise ProtocolError('移动许可与本地已确认结果不匹配')

    def fail(self, round_id, detail, state='failed'):
        with self.database() as db:
            db.execute(
                "UPDATE rounds SET state=?, detail=? WHERE round_id=? AND locked=1 AND state!='complete'",
                (state, str(detail)[:400], round_id),
            )

    def release(self, round_id, *, consume):
        with self.database() as db:
            updated = db.execute(
                "UPDATE rounds SET locked=0, permit_consumed=? WHERE round_id=? AND locked=1 AND state='complete'",
                (int(consume), round_id),
            )
        if updated.rowcount:
            self.blocked = False
        return updated.rowcount == 1

    def reconcile_on_connect(self, peer_idle):
        """Explicit reconnect after a stop/restart never resumes robot motion."""
        if not self.blocked:
            return
        with self.database() as db:
            db.execute("UPDATE rounds SET locked=0 WHERE locked=1 AND state='complete'")
            if peer_idle:
                db.execute(
                    """UPDATE rounds SET locked=0, state='interrupted',
                       detail=COALESCE(detail, '重连时确认 Client 已回待命') WHERE locked=1"""
                )
            blocked = db.execute('SELECT 1 FROM rounds WHERE locked=1').fetchone() is not None
        self.blocked = blocked

    def export(self, packed):
        rounds, nodes = {}, []
        if self.path.exists():
            with self.database() as db:
                db.execute('BEGIN')
                for row in db.execute('SELECT * FROM rounds ORDER BY round_id'):
                    record = dict(row)
                    record.update(json.loads(record.pop('station')))
                    rounds[record['round_id']] = record
                for row in db.execute('SELECT * FROM nodes ORDER BY round_id,node'):
                    station = rounds[row['round_id']]
                    nodes.append({
                        'session_id': station['session_id'], 'map_id': station['map_id'],
                        'point_id': station['point_id'], 'frame_id': 'map',
                        'position_source': 'ros_localization', 'yaw_rad': station['actual_pose'][2],
                        **json.loads(row['result']), 'car_receive_time': row['car_receive_time'],
                        'round_status': station['state'],
                    })
        if not packed:
            return ''.join(json.dumps(n, ensure_ascii=False) + '\n' for n in nodes).encode('utf-8')
        output = io.StringIO()
        fields = ('session_id', 'map_id', 'point_id', 'frame_id', 'position_source', 'yaw_rad',
                  *RESULT_FIELDS, 'car_receive_time', 'round_status')
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        for node in nodes:
            writer.writerow(node)
        buffer = io.BytesIO()
        with zipfile.ZipFile(buffer, 'w', zipfile.ZIP_DEFLATED) as archive:
            archive.writestr('servo_results.csv', output.getvalue().encode('utf-8-sig'))
            archive.writestr('rounds.json', json.dumps(list(rounds.values()), ensure_ascii=False, indent=2))
            for map_id in {record['map_id'] for record in rounds.values()}:
                for path in (self.directory / 'maps' / map_id).rglob('*'):
                    if path.is_file():
                        archive.write(path, path.relative_to(self.directory).as_posix())
        return buffer.getvalue()
