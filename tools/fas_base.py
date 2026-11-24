"""Serial bridge for the WS63 one-shot receiver and ESP32-C5 fixed-state receiver."""
import argparse
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
from hmac import compare_digest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import re
import threading
import time
import uuid
from urllib.parse import parse_qs, urlsplit


PROTOCOL = 'FAS_PAIR_1'
WS_IDS = {'A': '11:22:33:44:55:61', 'B': '11:22:33:44:55:62'}
RECEIVERS = ['sle/main', 'c5/main']
SERIAL_CLOSE_FAULT = '接收板关窗未确认'
ID = re.compile(r'[a-zA-Z0-9_-]{1,64}')
MAC = re.compile(r'(?:[0-9a-f]{2}:){5}[0-9a-f]{2}')


class Canceled(Exception):
    pass


def retry_file_access(operation):
    """Allow short-lived Windows file sharing conflicts during atomic replacement."""
    for delay in (.02,.04,.08,.16,.32):
        try:
            return operation()
        except PermissionError:
            time.sleep(delay)
    return operation()


class Port:
    def __init__(self, name, baudrate):
        import serial
        self.serial = serial.Serial(port=None, baudrate=baudrate, timeout=.08, write_timeout=.5, exclusive=True)
        self.serial.port = name
        self.serial.dtr = self.serial.rts = False
        self.serial.open()
        self.pending = bytearray()
        self.log = None

    def reconnect(self):
        self.serial.close()
        self.pending.clear()
        self.serial.open()

    def send(self, command):
        if self.log:
            self.log.write('> ' + command + '\n')
        self.serial.write((command + '\n').encode('ascii'))

    def lines(self, deadline, cancel=None):
        while time.monotonic() < deadline:
            if cancel and cancel.is_set():
                raise Canceled()
            end = self.pending.find(b'\n')
            if end < 0:
                self.pending.extend(self.serial.read(min(65536, self.serial.in_waiting or 1)))
                if len(self.pending) > 2**20:
                    raise ValueError('串口超过 1 MiB 无换行')
                continue
            line = self.pending[:end].decode('ascii', errors='replace').strip()
            del self.pending[:end+1]
            if self.log:
                self.log.write(line + '\n')
            yield line

    def expect(self, predicate, timeout=2., cancel=None, *, check_error=True):
        for line in self.lines(time.monotonic()+timeout, cancel):
            if predicate(line):
                return line
            if check_error and line.startswith(('WS_ERROR,', 'E,')):
                raise ValueError(line)
        raise TimeoutError(f'{self.serial.port} 等待接收板回执超时')

    def request(self, command, predicate, timeout=2., cancel=None, *, check_error=True):
        if cancel and cancel.is_set():
            raise Canceled()
        self.send(command)
        return self.expect(predicate, timeout, cancel, check_error=check_error)


def fields(line):
    return dict(part.split('=',1) for part in line.split(',')[1:] if '=' in part)


def ws_status(line, source):
    return line.startswith('WS_STATUS,') and fields(line).get('source') == source


def stop_ws(port):
    status=fields(port.request('WSSTOP', lambda line: ws_status(line,'stopped'), check_error=False))
    if status.get('running')!='0':
        raise ValueError('WS63 未确认停止')
    return status


def stop_c5(port, session):
    port.request('SESSIONSTOP,' + session,
                 lambda line: line.startswith(f'Y,stopped,{session},'),
                 check_error=False)


def sample_ws(port, request, cancel):
    index=request['state_id']
    duration=math.ceil(request['window_s']*1000)
    status=fields(port.request('WSGET',lambda line:ws_status(line,'query'),cancel=cancel))
    if status.get('oneshot_api')!='1' or status.get('running')!='0':
        raise ValueError('WS63 需要空闲的单次采样固件')
    for attempt in range(8):
        try:
            port.request('WSROUND,6,1,2',lambda line:line.startswith('WS_ROUND_CONFIG,'),cancel=cancel)
            accepted=fields(port.request(f'WSONCE,{index},{duration+5},5',
                                         lambda line:ws_status(line,'one_shot'),cancel=cancel))
            break
        except ValueError as error:
            if not any(reason in str(error) for reason in ('busy_or_draining','stop_before_config')) or attempt==7:
                raise
            if cancel.wait(.15):
                raise Canceled() from error
    if int(accepted['node'])!=index:
        raise ValueError('WS63 采样确认状态不一致')
    latest_attempt=int(accepted['attempt'])
    verified_attempt=None
    result=None
    for line in port.lines(time.monotonic()+request['window_s']+9,cancel):
        if line.startswith('WS_ERROR,'):
            raise ValueError(line)
        if line.startswith('#STATE,'):
            state=fields(line)
            if (int(state['node'])==index and int(state['attempt'])>latest_attempt
                    and [int(state['n1']),int(state['n2'])]==request['radios']['sle']):
                verified_attempt=int(state['attempt'])
                result=None
        elif line.startswith('RF_STATE,'):
            row=fields(line)
            if verified_attempt is not None and int(row['state'])==index and row['valid']=='1':
                counts={car:int(row[f'count_{car.lower()}']) for car in ('A','B')}
                if min(counts.values())>=request['min_samples']:
                    result={'epoch':verified_attempt,'samples':counts,
                            'rssi_dbm':{car:int(row[f'sum_{car.lower()}'])/count for car,count in counts.items()}}
        elif ws_status(line,'one_shot_done'):
            end=fields(line)
            if result and int(end['node'])==index and int(end['attempt'])==verified_attempt and end['running']=='0':
                return result
            raise ValueError('WS63 当前状态的 A/B 有效样本不足')
    raise TimeoutError('WS63 未返回单窗结束确认')


def sample_c5(port, request, cancel, session):
    state=request['radios']['c5']
    main,sub=state&0xfff,state>>12
    config=request['tx']['c5']
    window_s=request.get('c5_window_s',request['window_s'])
    lease=min(60000,math.ceil(window_s*1000)+7000)
    port.request(f'SESSIONARM,{session},{lease}', lambda line:line.startswith(f'Y,armed,{session},'),cancel=cancel)
    selection=f"CLIENTCFG,2,{config['A']},{config['B']}"
    selected=lambda line:line.startswith('W,whitelist,') and 'selection_applied' in line
    try:
        port.request(selection,selected,cancel=cancel)
    except ValueError as exc:
        if 'client_not_discovered_or_incompatible' not in str(exc):
            raise
        port.request('CLIENTDISCOVER,5000',lambda line:'discovery_complete' in line,timeout=8.,cancel=cancel)
        port.request(selection,selected,cancel=cancel)
    port.request('CFG,10,10,16,1000',lambda line:line.startswith('S,') and line.endswith(',usb'),cancel=cancel)

    plan_confirmed=False

    def applied(line):
        nonlocal plan_confirmed
        parts=line.split(',')
        if parts[0]=='H' and parts[-1]=='gpio24':
            plan_confirmed=parts==['H','sequence','sequence','1','1',str(main),str(sub),'gpio24']
        # UART1 acknowledges the 12-bit auxiliary setting. The CSI row below
        # independently verifies the combined 24-bit setting.
        return (plan_confirmed and len(parts)==9 and parts[0]=='A' and parts[2]==str(sub) and parts[4]==str(sub)
                and parts[1]==parts[3] and parts[5]=='acked' and parts[8]=='gpio24')

    port.request(f'GPIO24CFG,1000,sequence,sequence,1,1,{main},{sub}',applied,cancel=cancel)
    port.request(f'SESSIONSTART,{session}',lambda line:line.startswith(f'Y,streaming,{session},'),cancel=cancel)
    samples=defaultdict(dict)
    epoch_start=None
    epochs=math.ceil(window_s*100)
    for line in port.lines(time.monotonic()+window_s+5,cancel):
        if line.startswith('Y,armed,'):
            raise ValueError('C5 会话在采样完成前结束')
        if not line.startswith('C,'):
            continue
        parts=line.split(',',5)
        if len(parts)!=6:
            continue
        _,index,address,rssi,reported_state,payload=parts
        if not index.isdecimal() or not reported_state.isdecimal() or int(reported_state)!=state or not payload:
            continue
        car=next((car for car in ('A','B') if address.lower()==config[car]),None)
        if car is None:
            continue
        index=int(index)
        level=int(rssi)
        if not -127<=level<=0:
            continue
        if epoch_start is None:
            epoch_start=index
        if index<epoch_start:
            continue
        if index>=epoch_start+epochs:
            counts={car:len(samples[car]) for car in ('A','B')}
            if min(counts.values())>=request['min_samples']:
                return {'epoch':f'{session}:{epoch_start}','samples':counts,
                        'rssi_dbm':{car:sum(samples[car].values())/count for car,count in counts.items()}}
            raise ValueError('C5 当前状态的 A/B 有效样本不足')
        samples[car].setdefault(index,level)
    raise TimeoutError('C5 未收到足够的新采样周期')


class Station:
    def __init__(self, ws, c5, directory, links):
        self.ws,self.c5,self.directory,self.links=ws,c5,directory,links
        self.lock=threading.RLock()
        self.cancel=threading.Event()
        self.closing=threading.Event()
        self.done=threading.Event()
        self.done.set()
        self.active=None
        self.fault=None
        self.directory.mkdir(parents=True,exist_ok=True)

    def prepare_ports(self):
        if stop_ws(self.ws).get('oneshot_api')!='1':
            raise ValueError('请烧录 WS63-BASE-ONCE.fwpkg 后重启')
        status=self.c5.request('SESSIONSTATUS',lambda line:line.startswith('Y,') and line.endswith(',query'))
        session=status.split(',')[2]
        if session.startswith('fas_'):
            stop_c5(self.c5,session)
        elif status.startswith(('Y,streaming,','Y,armed,')):
            raise ValueError('先关闭 C5 正在运行的采集会话')

    def prepare(self):
        self.prepare_ports()
        for path in self.directory.glob('*/receipt.json'):
            record=self.receipt(path.parent.name)
            if record and record['status']=='running':
                self.save(path,dict(record,status='incomplete',error='基站重启，旧窗口已关闭'))

    def recover_ports(self):
        delay=1.
        while not self.closing.wait(delay):
            with self.lock:
                fault=self.fault
                if self.active or not fault or not fault.startswith(SERIAL_CLOSE_FAULT):
                    return
            try:
                self.ws.reconnect()
                self.c5.reconnect()
                self.prepare_ports()
            except (OSError,ValueError,IndexError) as error:
                print(f'基站串口重连未通过：{error}',flush=True)
                delay=min(30.,max(3.,2.5*delay))
                continue
            with self.lock:
                if self.fault == fault:
                    self.fault=None
            print('基站接收链已重连并确认关闭旧采样窗口。',flush=True)
            return

    def save(self, path, data):
        tmp=path.with_suffix('.tmp')
        with tmp.open('w',encoding='utf-8') as f:
            json.dump(data,f,ensure_ascii=False,allow_nan=False)
            f.flush()
            os.fsync(f.fileno())
        retry_file_access(lambda:tmp.replace(path))

    def receipt(self, trial):
        if not ID.fullmatch(trial):
            raise ValueError('采样编号无效')
        path=self.directory/trial/'receipt.json'
        with self.lock:
            try:
                return json.loads(retry_file_access(lambda:path.read_text(encoding='utf-8')))
            except FileNotFoundError:
                return None

    def wait_receipt(self, trial, timeout):
        with self.lock:
            pending = bool(self.active and self.active[1] == trial)
        if pending and timeout:
            self.done.wait(timeout)
        return self.receipt(trial)

    def validate(self, request):
        if request['protocol']!=PROTOCOL or request['tx']!=self.links:
            raise ValueError('协议或 A/B 发端身份不匹配')
        if any(not isinstance(request[key],str) or not ID.fullmatch(request[key]) for key in ('trial_id','run_id')):
            raise ValueError('实验编号无效')
        state=request['state_id']
        radios=request['radios']
        if (type(state) is not int or not 0<=state<4096 or radios['sle']!=list(divmod(state,64))
                or type(radios['c5']) is not int or radios['c5']!=c5_word(state)):
            raise ValueError('实验状态与硬件状态映射不匹配')
        windows=(request['window_s'],request.get('c5_window_s',request['window_s']))
        if (any(type(window) not in (int,float) or not .2<=window<=5 for window in windows)
                or type(request['min_samples']) is not int or not 1<=request['min_samples']<=500):
            raise ValueError('基站窗口支持 0.2–5 秒和 1–500 样本')
        if type(request.get('parallel_radios',False)) is not bool:
            raise ValueError('并行窗口选项须为布尔值')
        poses=request['cars']
        if set(poses)!={'A','B'} or poses['A']['map_id']!=poses['B']['map_id']:
            raise ValueError('A/B 位姿须来自同一张地图')
        for pose in poses.values():
            if (pose['frame']!='map' or not pose['point_id'] or len(pose['pose'])!=3
                    or not all(type(v) in (int,float) and math.isfinite(v) for v in pose['pose'])):
                raise ValueError('车辆位姿无效')

    def start(self, request):
        self.validate(request)
        trial=request['trial_id']
        with self.lock:
            record=self.receipt(trial)
            if record is not None:
                if any(record.get(key)!=value for key,value in request.items()):
                    raise ValueError('采样编号已被另一个请求使用')
                return record
            if self.fault or self.active:
                raise ValueError(self.fault or '基站已在采样')
            path=self.directory/trial
            path.mkdir(exist_ok=True)
            self.save(path/'receipt.json',dict(request,status='running'))
            self.active=(request['run_id'],trial)
            self.cancel.clear()
            self.done.clear()
            threading.Thread(target=self.capture,args=(request,path),daemon=True).start()
            return dict(request,status='running')

    def capture(self, request, path):
        data={}
        error=None
        status='incomplete'
        session='fas_'+uuid.uuid4().hex[:20]

        def record(port, filename, sample, *args):
            with (path/filename).open('w',encoding='ascii',errors='replace') as log:
                port.log=log
                try:
                    return sample(port,request,self.cancel,*args)
                finally:
                    port.log=None
                    log.flush()
                    os.fsync(log.fileno())

        try:
            if request.get('parallel_radios'):
                with ThreadPoolExecutor(max_workers=2) as pool:
                    ws=pool.submit(record,self.ws,'ws63.log',sample_ws)
                    c5=pool.submit(record,self.c5,'c5.log',sample_c5,session)
                    data['sle/main']=ws.result()
                    data['c5/main']=c5.result()
            else:
                data['sle/main']=record(self.ws,'ws63.log',sample_ws)
                data['c5/main']=record(self.c5,'c5.log',sample_c5,session)
            status='complete'
        except Canceled:
            status='canceled'
        except (OSError,ValueError,KeyError,TypeError,IndexError) as exc:
            error=str(exc)
        finally:
            try:
                stop_ws(self.ws)
                # SESSIONARM may have been aborted before acknowledgment.
                current=self.c5.request('SESSIONSTATUS',lambda line:line.startswith('Y,') and line.endswith(',query'),
                                        check_error=False)
                if current.split(',')[2]==session:
                    stop_c5(self.c5,session)
                elif current.startswith('Y,streaming,'):
                    raise ValueError('C5 仍有未关闭的会话')
            except (OSError,ValueError,IndexError) as exc:
                self.fault=SERIAL_CLOSE_FAULT+'，正在复核基站串口：'+str(exc)
                status,error='incomplete',self.fault
            with self.lock:
                if self.cancel.is_set() and not self.fault:
                    status='canceled'
                try:
                    self.save(path/'receipt.json',dict(request,status=status,error=error,data=data,raw_ref=str(path)))
                except OSError as exc:
                    self.fault='采集结果持久化失败：'+str(exc)
                finally:
                    self.active=None
                    self.done.set()
            if self.fault and self.fault.startswith(SERIAL_CLOSE_FAULT):
                threading.Thread(target=self.recover_ports,daemon=True).start()

    def abort(self, request):
        with self.lock:
            trial=request['trial_id']
            record=self.receipt(trial)
            if record and record['run_id']!=request['run_id']:
                raise ValueError('采样不属于此实验')
            if self.active==(request['run_id'],trial):
                self.cancel.set()
                wait=True
            else:
                wait=False
        if wait and not self.done.wait(1.25):
            return {'status':'canceling'}
        with self.lock:
            if self.fault or self.active:
                return {'status':'blocked','error':self.fault or '另一采样窗口仍在运行'}
            return {'status':'idle'}


def c5_word(index):
    sub,main=divmod(index,64)
    return main*65 | ((sub*65)<<12)


def make_handler(station,key):
    class Handler(BaseHTTPRequestHandler):
        def reply(self,status,obj):
            body=json.dumps(obj,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type','application/json; charset=utf-8')
            self.send_header('Content-Length',str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if not compare_digest(self.headers.get('X-FAS-Key','').encode(),key):
                return self.reply(403,{'error':'基站授权失败'})
            if self.path=='/fas/health':
                return self.reply(200,{'protocol':PROTOCOL,'ready':station.fault is None,
                                       'tx_configured':station.links,'receivers':RECEIVERS,'error':station.fault,
                                       'separate_c5_window':True,'parallel_radios':True,'receipt_wait':True})
            try:
                path=urlsplit(self.path)
                if path.path.startswith('/fas/capture/'):
                    timeout=float(parse_qs(path.query).get('wait',['0'])[0])
                    if not 0. <= timeout <= .5:
                        raise ValueError('采样回执等待范围为 0–0.5 秒')
                    record=station.wait_receipt(path.path.split('/')[-1],timeout)
                    if record is not None:
                        return self.reply(200,record)
            except (OSError,ValueError) as exc:
                return self.reply(400,{'error':str(exc)})
            return self.reply(404,{'error':'采样编号不存在'})

        def do_POST(self):
            if not compare_digest(self.headers.get('X-FAS-Key','').encode(),key):
                return self.reply(403,{'error':'基站授权失败'})
            if self.path not in ('/fas/capture','/fas/cancel'):
                return self.reply(404,{'error':'基站接口不存在'})
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 1<=size<=32768:
                    raise ValueError('请求长度无效')
                request=json.loads(self.rfile.read(size))
                result=station.start(request) if self.path=='/fas/capture' else station.abort(request)
            except (ValueError,TypeError,KeyError,OSError) as exc:
                return self.reply(400,{'error':str(exc)})
            return self.reply(200,result)

    return Handler


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ws63',required=True)
    parser.add_argument('--c5',required=True)
    parser.add_argument('--mac-a',type=str.lower,required=True)
    parser.add_argument('--mac-b',type=str.lower,required=True)
    parser.add_argument('--key-file',type=Path,required=True)
    parser.add_argument('--data',type=Path,required=True)
    parser.add_argument('--bind',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=8795)
    args=parser.parse_args()
    if (args.ws63.casefold()==args.c5.casefold() or args.mac_a==args.mac_b
            or not all(MAC.fullmatch(x) for x in (args.mac_a,args.mac_b))):
        parser.error('两块基站接收板应使用不同串口，两车 C5 MAC 应不同')
    key=args.key_file.read_bytes().strip()
    if not key or not key.isascii() or any(chr(c).isspace() for c in key):
        parser.error('基站密钥应为非空且无空格的 ASCII 内容')
    ws=Port(args.ws63,115200)
    try:
        c5=Port(args.c5,3000000)
        try:
            station=Station(ws,c5,args.data.resolve(),{'sle':WS_IDS,'c5':{'A':args.mac_a,'B':args.mac_b}})
            station.prepare()
            with ThreadingHTTPServer((args.bind,args.port),make_handler(station,key)) as server:
                print(f'{PROTOCOL} http://{args.bind}:{args.port} · Ctrl+C 关窗退出',flush=True)
                try:
                    server.serve_forever()
                except KeyboardInterrupt:
                    station.closing.set()
                    station.cancel.set()
                    station.done.wait(10)
        finally:
            c5.serial.close()
    finally:
        ws.serial.close()


if __name__=='__main__':
    main()
