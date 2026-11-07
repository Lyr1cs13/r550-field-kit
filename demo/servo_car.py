"""SERVO_CAR_2 serial protocol. The worker owns the UART and never moves the car."""
from dataclasses import dataclass
import math
import re
import time

from rdk_link import LineTransport, ProtocolError
from servo_storage import MAX_ROUND, RESULT_FIELDS


MODE = 'servo_car_v2'
ROUND_TIMEOUT = 105.0
RETRY_LINK_TIMEOUT = 12.0
POLL_INTERVAL = 0.8
ACK_INTERVAL = 1.3
ACK_ATTEMPTS = 3


class PeerRejected(ProtocolError):
    pass


class PeerRoundFailed(ProtocolError):
    pass


def integers(values, count):
    if len(values) != count or any(re.fullmatch(r'[+-]?\d{1,20}', value) is None for value in values):
        raise ProtocolError('SERVO_CAR_2 整数字段格式错误')
    return tuple(map(int, values))


@dataclass(frozen=True)
class CarStatus:
    round_id: int
    position_id: int
    state: int
    link_ready: int
    result_mask: int

    @classmethod
    def parse(cls, line):
        status = cls(*integers(line.partition(':')[2].split(','), 5))
        if not (0 <= status.round_id <= MAX_ROUND and 0 <= status.position_id <= MAX_ROUND
                and 0 <= status.state <= 4 and status.link_ready in (0, 1) and 0 <= status.result_mask <= 3):
            raise ProtocolError('CARSTATUS 字段超出范围')
        return status

    @property
    def idle(self):
        return self.state == 0 and self.result_mask == 0

    @property
    def can_start(self):
        return bool(self.link_ready and (self.idle or self.state in (3, 4)))


class CarLink(LineTransport):
    def request(self, command, prefix=None, receive=None, check=None, timeout=2.0):
        self.send(command)
        deadline = self.clock() + timeout
        reply = None
        while self.clock() < deadline:
            if check:
                check()
            try:
                line = self.line(min(deadline, self.clock() + .2))
            except TimeoutError:
                continue
            if prefix is not None and line.startswith(prefix):
                reply = line
            elif line == 'OK':
                if prefix is None or reply is not None:
                    return reply
            elif line == 'ERROR' or line.startswith('+CME ERROR'):
                raise PeerRejected(f'Client 拒绝 {command}')
            elif receive and line.startswith('+CAR'):
                receive(line)
        raise TimeoutError(f'{command} 回复超时')

    def hello(self, check=None):
        reply = self.request('AT+CARHELLO', '+CARHELLO:', check=check)
        fields = [value.strip().upper() for value in reply.partition(':')[2].split(',')]
        if 'SERVO_CAR_2' not in fields and fields[0] not in ('2', 'V2'):
            raise ProtocolError(f'未确认 SERVO_CAR_2 协议 v2，收到 {reply[:120]}')
        return reply

    def status(self, receive=None, check=None):
        return CarStatus.parse(self.request('AT+CARSTATUS', '+CARSTATUS:', receive, check))


def finite_number(value):
    try:
        number = float(value)
    except ValueError as exc:
        raise ProtocolError('CARRESULT 数值格式错误') from exc
    if not math.isfinite(number):
        raise ProtocolError('CARRESULT 数值必须有限')
    return number


class RoundCapture:
    def __init__(self, link, store, round_id, station, check, progress):
        self.link, self.store = link, store
        self.identity = (round_id, station['position_id'])
        self.coords = (station['x_mm'], station['y_mm'])
        self.check, self.progress = check, progress
        self.nodes = set()
        self.ended = False
        self.acks = 0
        self.move = False
        self.error = None
        self.started = self.link.clock()

    def receive(self, line):
        tag, _, payload = line.partition(':')
        fields = [field.strip() for field in payload.split(',')]
        if tag in ('+CARBUSY', '+CARERROR'):
            # Error text varies by firmware. Only discard a clearly identified
            # older round. An unqualified error holds the current station.
            if len(fields) >= 2 and all(field.isdecimal() for field in fields[:2]):
                if int(fields[0]) < self.identity[0]:
                    return
            if tag == '+CARERROR':
                self.error = line[:240]
            else:
                self.progress(peer_progress=payload[:120])
            return
        if tag not in ('+CARRESULT', '+CARROUNDEND', '+CARMOVE'):
            return
        round_id, position_id = integers(fields[:2], 2)
        if round_id < self.identity[0]:
            return
        if (round_id, position_id) != self.identity:
            raise ProtocolError(f'{tag} 的轮次或点位与当前测量不匹配')
        self.check()
        if tag == '+CARRESULT':
            self.result(fields)
        elif tag == '+CARROUNDEND':
            if integers(fields, 3)[2] != 2:
                raise ProtocolError('CARROUNDEND 应确认两个角度节点')
            if not self.ended:
                self.store.note_end(round_id)
                self.ended = True
        elif integers(fields, 2) == self.identity and self.acks:
            self.move = True

    def result(self, fields):
        if len(fields) != len(RESULT_FIELDS):
            raise ProtocolError('CARRESULT 字段数不匹配')
        round_id, position_id, x_mm, y_mm, node = integers(fields[:5], 5)
        horizontal = finite_number(fields[5])
        vertical = finite_number(fields[6])
        counters = integers(fields[7:12] + fields[13:], 8)
        start, end, duration, byte_count, packets, rssi_count, bps, payload_bps = counters
        if (x_mm, y_mm) != self.coords:
            raise ProtocolError('CARRESULT 坐标与本轮停车快照不一致')
        if node not in (0, 1) or horizontal != (0, 180)[node]:
            raise ProtocolError('CARRESULT 节点与水平角度不匹配')
        if end < start or any(not 0 <= value < 2**64 for value in counters):
            raise ProtocolError('CARRESULT 时间或计数字段超出范围')
        rssi = 'NA' if fields[12] == 'NA' else finite_number(fields[12])
        values = (round_id, position_id, x_mm, y_mm, node, int(horizontal), vertical,
                  start, end, duration, byte_count, packets, rssi, rssi_count, bps, payload_bps)
        self.store.save_node(dict(zip(RESULT_FIELDS, values)))
        self.nodes.add(node)
        self.progress(node_count=len(self.nodes))

    def check_peer(self):
        status = self.link.status(self.receive, self.check)
        if (status.round_id, status.position_id) != self.identity:
            raise ProtocolError('Client 当前轮次发生变化，请核对是否重启')
        if status.state == 4:
            raise PeerRoundFailed(self.error or 'Client 已确认本轮失败')
        if not status.link_ready:
            raise ProtocolError('Client 与旋转基站的 SLE 连接中断')
        if status.state == 0 or (status.state == 3 and not self.acks):
            raise ProtocolError('Client 状态与本轮确认进度不符')
        if self.error:
            raise ProtocolError(self.error)

    def start(self):
        command = 'AT+CARREADY=' + ','.join(map(str, (*self.identity, *self.coords)))
        for attempt in range(2):
            self.check()
            try:
                self.link.request(command, receive=self.receive, check=self.check)
                return
            except PeerRejected:
                self.check_peer()
                return
            except TimeoutError:
                if attempt:
                    self.check_peer()

    def measure(self, timeout):
        self.start()
        deadline = self.started + timeout
        poll_due = self.link.clock() + POLL_INTERVAL
        ack_due = float('inf')
        while self.link.clock() < deadline:
            self.check()
            now = self.link.clock()
            if self.error:
                self.check_peer()
            if self.move:
                self.store.complete(self.identity[0])
                return
            if self.ended and self.nodes == {0, 1} and (self.acks == 0 or now >= ack_due):
                if self.acks == ACK_ATTEMPTS:
                    raise ProtocolError('未收到匹配的 CARMOVE，保持停车')
                if self.acks == 0:
                    # Drain events queued before the first ACK. A premature
                    # MOVE received in the same UART chunk cannot become valid.
                    self.check_peer()
                    self.store.arm_ack(self.identity[0])
                self.check()
                self.link.send(f'AT+CARACK={self.identity[0]}')
                self.acks += 1
                self.progress(waiting_move=True)
                ack_due = self.link.clock() + ACK_INTERVAL
                poll_due = max(poll_due, self.link.clock() + POLL_INTERVAL)
            if now >= poll_due:
                self.check_peer()
                poll_due = self.link.clock() + POLL_INTERVAL
                self.progress(elapsed=round(self.link.clock() - self.started, 1))
                continue
            try:
                line = self.link.line(min(deadline, poll_due, ack_due, now + .2))
            except TimeoutError:
                continue
            if line == 'ERROR' or line.startswith('+CME ERROR'):
                self.check_peer()
            elif line.startswith('+CAR'):
                self.receive(line)
        self.check_peer()
        raise ProtocolError('旋转定点测量超时，保持停车')


def wait_for_retry(link, identity, check):
    deadline = link.clock() + RETRY_LINK_TIMEOUT
    while link.clock() < deadline:
        check()
        status = link.status(check=check)
        if (status.round_id, status.position_id, status.state) != (*identity, 4):
            raise ProtocolError('重测前 Client 状态已改变，请重新连接并核对位置')
        if status.link_ready:
            return status
        time.sleep(.3)
    raise ProtocolError('等待 SLE 恢复超时，保持停车')


def measure_station(link, store, station, check, progress, timeout=ROUND_TIMEOUT):
    """Use one new round for a peer-confirmed failure, at the same frozen position."""
    check()
    status = link.status(check=check)
    if not status.can_start:
        raise ProtocolError('Client 未就绪或仍在处理上一轮')
    replaces = None
    for attempt in range(2):
        check()
        round_id = store.reserve(status.round_id, station, replaces)
        capture = RoundCapture(link, store, round_id, station, check, progress)
        progress(stage='sampling', round_id=round_id, position_id=station['position_id'],
                 node_count=0, peer_progress=None, elapsed=0, attempt=attempt + 1, waiting_move=False)
        try:
            capture.measure(timeout)
            return round_id
        except PeerRoundFailed as exc:
            store.fail(round_id, exc)
            if attempt:
                raise
            progress(stage='sampling', peer_progress='等待链路恢复后在原位置重测')
            status = wait_for_retry(link, capture.identity, check)
            replaces = round_id
        except InterruptedError as exc:
            store.fail(round_id, exc, state='interrupted')
            raise
        except Exception as exc:
            store.fail(round_id, exc)
            raise
