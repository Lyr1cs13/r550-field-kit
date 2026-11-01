"""Single-owner UART transport. No ROS import and no robot motion commands."""
from dataclasses import dataclass
import math
import secrets
import time


class ProtocolError(RuntimeError): pass
class CollectionFailed(ProtocolError): pass
class DeviceReset(ProtocolError): pass


@dataclass(frozen=True)
class Point:
    point: int
    map_tag: int
    x_mm: int
    y_mm: int
    yaw_mdeg: int

    @classmethod
    def from_pose(cls, point, map_tag, x, y, yaw):
        if not all(math.isfinite(v) for v in (x,y,yaw)):
            raise ValueError('Nonfinite pose')
        yaw=math.atan2(math.sin(yaw),math.cos(yaw))
        return cls(point,map_tag,round(x*1000),round(y*1000),round(math.degrees(yaw)*1000))

    def values(self):
        values=(self.point,self.map_tag,self.x_mm,self.y_mm,self.yaw_mdeg)
        if any(type(v) is not int for v in values):raise ValueError('Integer wire values required')
        if not (0<=self.point<2**31 and 0<=self.map_tag<2**31 and
                abs(self.x_mm)<=10000000 and abs(self.y_mm)<=10000000 and abs(self.yaw_mdeg)<=180000):
            raise ValueError('Pose outside protocol range')
        return values


class Link:
    def __init__(self, stream, *, allow_mock=False, clock=time.monotonic):
        self.stream=stream;self.allow_mock=allow_mock;self.clock=clock
        self.buffer=bytearray();self.events=[];self.run=None;self.visit=0;self.mode=None
        self.pending=False

    def line(self, deadline):
        while self.clock()<deadline:
            if b'\n' in self.buffer:
                line,_,remaining=self.buffer.partition(b'\n');self.buffer=bytearray(remaining)
                decoded=bytes(line).strip().decode('ascii',errors='replace')
                if getattr(self,'on_line',None): self.on_line(decoded)
                return decoded
            self.buffer.extend(self.stream.read(128))
            if len(self.buffer)>8192:raise ProtocolError('Overlong UART line')
        raise TimeoutError('UART response timeout')

    def command(self, cmd, timeout=2.):
        if '\r' in cmd or '\n' in cmd:raise ValueError('Embedded command delimiter')
        data=(cmd+'\r\n').encode('ascii')
        if self.stream.write(data)!=len(data):raise ProtocolError('Incomplete serial write')
        self.stream.flush();out=[];deadline=self.clock()+timeout
        while True:
            line=self.line(deadline)
            if line.startswith('+RDKREADY:'):raise DeviceReset('MCU restarted; establish a new session')
            if line.startswith('+RDK'):
                out.append(line);self.events.append(line)
                self.events=self.events[-128:]
            elif line=='OK':return out
            elif line=='ERROR' or line.startswith('+CME ERROR'):
                raise ProtocolError('MCU rejected '+cmd)
            # AT echo, boot text and unrelated diagnostics are not completion.

    def connect(self, run=None):
        response=self.command('AT+RDKHELLO')
        hello=next((v for v in response if v.startswith('+RDKHELLO:')),None)
        if hello not in ('+RDKHELLO:1,COMMTEST','+RDKHELLO:1,COLLECTOR'):
            raise ProtocolError('Unsupported communication firmware/version')
        self.mode=hello.split(',')[1]
        if self.mode=='COMMTEST' and not self.allow_mock:
            raise ProtocolError('COMMTEST firmware is not allowed for real collection')
        run=run if run is not None else secrets.randbelow(2**31-1)+1
        if type(run) is not int or not 0<run<2**31:raise ValueError('Invalid session')
        response=self.command(f'AT+RDKSESSION={run}')
        if f'+RDKSESSION:{run}' not in response:raise ProtocolError('Session ACK missing')
        self.run=run;self.visit=0;self.pending=False;self.events.clear()
        return self.mode

    def set_test_delay(self, milliseconds):
        if self.mode!='COMMTEST':raise ProtocolError('Synthetic delay only supported in COMMTEST')
        if type(milliseconds) is not int or not 0<=milliseconds<=30000:raise ValueError('Invalid delay')
        self.command(f'AT+RDKTESTDELAY={milliseconds}')

    def submit(self, point):
        if self.run is None:raise ProtocolError('Connect first')
        if self.pending:raise ProtocolError('Finish or cancel the current visit first')
        values=point.values();self.visit+=1
        fields=(self.run,self.visit,*values);self.last_fields=fields
        self.last_command='AT+RDKPOINT='+','.join(map(str,fields))
        self.events.clear()
        self.pending=True
        # Repeating the identical token is safe. Never create a new visit to retry.
        for attempt in range(2):
            try:
                result=self.command(self.last_command)
                if '+RDKACK:'+','.join(map(str,fields)) not in result:
                    raise ProtocolError('Point ACK missing or coordinate echo mismatch')
                return self.visit
            except TimeoutError:
                if attempt:raise

    def matching_done(self):
        for line in self.events:
            if line.startswith('+RDKDONE:'):
                try:run,visit,status=map(int,line.partition(':')[2].split(','))
                except ValueError as exc:raise ProtocolError('Malformed DONE') from exc
                if (run,visit)!=(self.run,self.visit):continue
                self.pending=False
                if status!=0:raise CollectionFailed(f'Collection failed: status={status}')
                return True
        return False

    def state(self):
        response=self.command('AT+RDKSTATE')
        try:
            state=next(v for v in response if v.startswith('+RDKSTATE:'))
            fields=tuple(map(int,state.partition(':')[2].split(',')))
        except (StopIteration,ValueError) as exc:raise ProtocolError('Invalid state reply') from exc
        if len(fields)!=5:raise ProtocolError('Invalid state fields')
        return fields

    def cancel(self):
        response=self.command(f'AT+RDKCANCEL={self.run},{self.visit}')
        if f'+RDKCANCELED:{self.run},{self.visit}' not in response:
            raise ProtocolError('Cancel ACK missing')
        self.pending=False

    def wait_done(self, timeout=30.):
        deadline=self.clock()+timeout
        while self.clock()<deadline:
            if self.matching_done():return
            run,visit,state,status,starts=self.state()
            if (run,visit)!=(self.run,self.visit):raise DeviceReset('Active batch disappeared/reset')
            if state==2:
                self.pending=False
                if status:raise CollectionFailed(f'Collection failed: status={status}')
                return
            if state==3:
                self.pending=False
                raise CollectionFailed('Batch canceled')
            if state!=1:raise ProtocolError('Unexpected batch state')
            time.sleep(.1)
        # A timeout is never silently converted into successful collection.
        try:self.cancel()
        except (ProtocolError,TimeoutError):pass
        raise TimeoutError('Acquisition did not finish before timeout')

    def collect(self, point, timeout=30.):
        self.submit(point);self.wait_done(timeout)
