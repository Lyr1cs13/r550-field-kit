#!/usr/bin/env python3
"""Link stopped R550 waypoints to the original plan_fixed/plan_variable collector."""
import argparse
import csv
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib
import json
import os
from pathlib import Path
import secrets
import subprocess
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'demo'))
from c5_station import MAC, PROTOCOL, TOKEN, VISIT_ID, read_json, read_token, utc, validate_visit, write_json


LEASE_SECONDS = 20
DISCOVERY_SECONDS = 45
DRAIN_SECONDS = 15
WORKER = Path(__file__).with_name('c5_capture_worker.py').resolve()


def serial_device(port):
    from serial.tools import list_ports

    match = next((p for p in list_ports.comports(include_links=True)
                  if p.device.casefold() == port.casefold()), None)
    if match is None:
        raise ValueError('没有找到该串口，请核对主节点的设备号')
    description = ' '.join(getattr(match, key) or '' for key in ('description', 'manufacturer', 'hwid')).lower()
    if ('intel' in description and ('sol' in description or 'active management' in description)
            or 'pci\\ven_8086' in description):
        raise ValueError('所选串口是电脑的管理端口，请选择 C5 主节点的 USB 转串口')
    return match.device


def probe(script, plan, port):
    """Read STATUS through the vendor decoder, without selecting a root or capturing."""
    import serial
    match = serial_device(port)
    sys.path.insert(0, str(script.parent))
    try:
        protocol = importlib.import_module(plan + '_protocol')
    finally:
        sys.path.pop(0)
    decoder = protocol.Decoder()
    options = {'exclusive': True} if os.name == 'posix' else {}
    with serial.Serial(port=None, baudrate=protocol.BAUD, timeout=.15, write_timeout=1, **options) as stream:
        stream.dtr = False
        stream.rts = False
        stream.port = match
        stream.open()
        deadline = time.monotonic() + 3
        next_get = 0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_get:
                stream.write(b'GET\n')
                next_get = time.monotonic() + .6
            for event in decoder.feed(stream.read(4096)):
                if event.get('type') != 'STATUS' or event.get('fw') != protocol.FW:
                    continue
                if event.get('role') != 'anchor':
                    raise ValueError('该串口接的是车载标签，请接固定主节点')
                if event.get('active') or event.get('draining'):
                    raise ValueError('固定主节点仍在采集，请先结束上一轮')
                return match
    raise ValueError('未收到匹配的固定板 STATUS，请核对固件、UART 口与 3 Mbaud')


def verify_capture(directory, plan, groups, visit_id, expected_tag=None):
    """Accept a saved run with actual tag CSI. Variable runs must finish the requested groups."""
    folders = [path for path in directory.iterdir() if path.is_dir()]
    if len(folders) != 1:
        raise ValueError('没有找到唯一的 C5 采集目录')
    folder = folders[0]
    record = read_json(folder / 'result.json')
    done = record.get('done_event') or {}
    run = record.get('selected_run')
    if (record.get('capture_ok') is not True or record.get('device_done') is not True
            or not isinstance(run, list) or len(run) != 3
            or run != [done.get(key) for key in ('root_mac', 'root_boot', 'run')]):
        raise ValueError(record.get('error') or 'C5 缺少对应会话的 DONE 或保存结果')
    reason = done.get('reason')
    if plan == 'variable':
        runs = (record.get('quality') or {}).get('scan', {}).get('runs', [])
        if (reason != 'group_limit' or done.get('completed_groups') != groups or len(runs) != 1
                or runs[0].get('joint_state_confirmed_groups') != groups):
            raise ValueError(f'C5 只完成 {done.get("completed_groups", 0)}/{groups} 组，已保留部分数据')
    elif reason != 'duration':
        raise ValueError(f'C5 固定状态采集提前停止（{reason}）')

    counts = {}
    valid_file = 'csi_packets_confirmed.csv' if plan == 'variable' else 'csi_packets.csv'
    with (folder / valid_file).open(encoding='utf-8-sig', newline='') as stream:
        for row in csv.DictReader(stream):
            if row.get('tx_role') != 'tag':
                continue
            if (row.get('label') != visit_id
                    or [row.get(k) for k in ('root_mac', 'root_boot', 'run')] != list(map(str, run))):
                raise ValueError('C5 数据的采集会话或停靠编号不匹配')
            tag, anchor = ((row.get(key) or '').lower() for key in ('tx_mac', 'rx_mac'))
            if not MAC.fullmatch(tag) or not MAC.fullmatch(anchor):
                raise ValueError('C5 数据缺少有效的标签或锚点 MAC')
            by_anchor = counts.setdefault(tag, {})
            if (row.get('csi_valid') or '').lower() in ('true', '1') and row.get('csi_base64'):
                by_anchor[anchor] = by_anchor.get(anchor, 0) + 1
    if expected_tag is None and len(counts) > 1:
        raise ValueError('本轮有多个移动标签，请用 --tag-mac 指定小车标签')
    tag = expected_tag or next(iter(counts), None)
    by_anchor = counts.get(tag, {})
    if not by_anchor:
        raise ValueError('本轮没有收到车载标签的有效 CSI')
    return {
        'plan': plan, 'run': run, 'reason': reason, 'tag_mac': tag,
        'valid_csi': sum(by_anchor.values()), 'valid_by_anchor': by_anchor,
        'crc_errors': record.get('crc_errors', 0), 'groups': done.get('completed_groups'),
        'capture_folder': directory.name + '/' + folder.name,
    }


class Collector:
    def __init__(self, script, port, plan, seconds, groups, output, tag=None):
        self.script, self.port, self.plan = script, port, plan
        self.seconds, self.groups, self.tag = seconds, groups, tag
        self.output = output
        self.output.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.job = None
        self.worker = None

    def status(self):
        with self.lock:
            error = None
            if self.job is None:
                try:
                    serial_device(self.port)
                except (OSError, ValueError) as exc:
                    error = str(exc)
            return {'ready': self.job is None and error is None, 'plan': self.plan, 'device': self.port, 'error': error,
                    'visit_id': self.job['visit_id'] if self.job else None}

    def start(self, visit):
        visit = validate_visit(visit)
        identity = visit['visit_id']
        folder = self.output / identity
        with self.lock:
            if folder.exists():
                if read_json(folder / 'visit.json') != visit:
                    raise ValueError('同一停靠编号的坐标或实验信息已改变')
                return self.receipt(identity)
            if self.job is not None:
                raise ValueError('采集端仍在处理上一个停靠点')
            serial_device(self.port)
            folder.mkdir()
            write_json(folder / 'visit.json', visit)
            write_json(folder / 'receipt.json', {'visit_id': identity, 'state': 'sampling'})
            self.job = {'visit_id': identity, 'lease': time.monotonic() + LEASE_SECONDS, 'cancel': False}
            self.worker = threading.Thread(target=self._run, args=(visit, folder), name='c5-capture')
            self.worker.start()
            return {'visit_id': identity, 'state': 'sampling'}

    def receipt(self, identity, cancel=False):
        with self.lock:
            folder = self.output / identity
            if not folder.is_dir():
                raise FileNotFoundError('没有找到该停靠轮次')
            if self.job and self.job['visit_id'] == identity:
                self.job['lease'] = time.monotonic() + LEASE_SECONDS
                self.job['cancel'] |= cancel
                return {'visit_id': identity, 'state': 'sampling'}
            path = folder / 'receipt.json'
            receipt = read_json(path) if path.exists() else {'visit_id': identity, 'state': 'sampling'}
            if receipt['state'] == 'sampling':
                return {'visit_id': identity, 'state': 'interrupted', 'error': '采集服务重启，保留上一轮数据'}
            return receipt

    def _run(self, visit, folder):
        name = visit['visit_id']
        receipt = {'visit_id': name, 'state': 'failed'}
        stop_file = folder / 'stop.requested'
        output = folder / 'capture'
        args = [
            sys.executable, '-u', str(WORKER), str(self.script), str(stop_file),
            '--port', self.port, '--seconds', str(self.seconds), '--delay', '0',
            '--discovery-timeout', str(DISCOVERY_SECONDS), '--output', str(output),
            '--label=' + name, '--point=' + visit['point_id'],
        ]
        if self.plan == 'variable':
            args += ['--groups', str(self.groups)]
        deadline = time.monotonic() + DISCOVERY_SECONDS + self.seconds + DRAIN_SECONDS + 25
        reason = None
        process = None
        try:
            with (folder / 'capture.log').open('wb') as log:
                process = subprocess.Popen(args, cwd=self.script.parent.parent, stdout=log,
                                           stderr=subprocess.STDOUT,
                                           creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
                while process.poll() is None:
                    now = time.monotonic()
                    with self.lock:
                        if reason is None:
                            if self.job['cancel']:
                                reason = '小车停止本轮采集'
                            elif now > self.job['lease']:
                                reason = '与 RDK 的联动中断'
                            elif now > deadline:
                                reason = 'C5 采集超过本轮等待时间'
                            if reason:
                                stop_file.touch()
                                stop_deadline = now + DRAIN_SECONDS
                    if reason and now >= stop_deadline:
                        process.kill()
                        break
                    time.sleep(.15)
                process.wait(timeout=5)
            if reason:
                receipt.update(state='canceled', error=reason)
            else:
                result = verify_capture(output, self.plan, self.groups, name, self.tag)
                if process.returncode != 0:
                    raise ValueError(f'原采集程序退出码 {process.returncode}')
                result['capture_folder'] = name + '/' + result['capture_folder']
                receipt.update(state='saved', result=result)
        except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as exc:
            receipt['error'] = str(exc)
        finally:
            if process is not None and process.poll() is None:
                process.kill()
                process.wait()
            with self.lock:
                try:
                    if self.job['cancel'] and receipt['state'] == 'saved':
                        receipt.update(state='canceled', error='小车停止本轮采集')
                    receipt['finished_utc'] = utc()
                    write_json(folder / 'receipt.json', receipt)
                except OSError as exc:
                    print(f'C5 保存回执失败：{exc}', file=sys.stderr, flush=True)
                finally:
                    self.job = None

    def close(self):
        with self.lock:
            if self.job:
                self.job['cancel'] = True
            worker = self.worker
        if worker:
            worker.join(timeout=DRAIN_SECONDS + 6)


def handler(collector, token):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, data, status=200):
            body = json.dumps({'protocol': PROTOCOL, **data}, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def dispatch(self, method):
            if not secrets.compare_digest(self.headers.get('Authorization', '').encode(), ('Bearer ' + token).encode()):
                return self.reply({'error': '密钥不匹配'}, 401)
            try:
                if method == 'GET' and self.path == '/v1/status':
                    return self.reply(collector.status())
                if method == 'POST' and self.path == '/v1/visits':
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 < length <= 8192:
                        raise ValueError('停靠记录长度无效')
                    return self.reply(collector.start(json.loads(self.rfile.read(length))))
                parts = self.path.split('/')
                if (len(parts) in (4, 5) and parts[:3] == ['', 'v1', 'visits']
                        and VISIT_ID.fullmatch(parts[3])):
                    if method == 'GET' and len(parts) == 4:
                        return self.reply(collector.receipt(parts[3]))
                    if method == 'POST' and parts[4:] == ['cancel']:
                        return self.reply(collector.receipt(parts[3], cancel=True))
                return self.reply({'error': '没有该接口'}, 404)
            except FileNotFoundError as exc:
                return self.reply({'error': str(exc)}, 404)
            except (ValueError, TypeError, KeyError) as exc:
                return self.reply({'error': str(exc)}, 409)
            except OSError:
                return self.reply({'error': '采集端无法写入数据目录'}, 500)

        def do_GET(self):
            self.dispatch('GET')

        def do_POST(self):
            self.dispatch('POST')

        def log_request(self, code='-', size='-'):
            if str(code) != '200':
                super().log_request(code, size)
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--script', type=Path, required=True, help='original fixed_node/tools/capture.py')
    parser.add_argument('--serial', required=True, help='fixed root port, for example COM11 or /dev/ttyUSB2')
    parser.add_argument('--plan', choices=['fixed', 'variable'], default='fixed')
    parser.add_argument('--seconds', type=int)
    parser.add_argument('--groups', type=int, default=16, help='variable plan only')
    parser.add_argument('--tag-mac', type=str.lower)
    parser.add_argument('--listen', default='127.0.0.1')
    parser.add_argument('--http-port', type=int, default=8791)
    parser.add_argument('--token-file', type=Path, default=TOKEN)
    parser.add_argument('--output', type=Path, default=TOKEN.parent / 'c5_captures')
    parser.add_argument('--probe', action='store_true', help='verify USB and firmware then exit')
    args = parser.parse_args()
    seconds = args.seconds if args.seconds is not None else (30 if args.plan == 'fixed' else 120)
    script = args.script.resolve()
    if script.name != 'capture.py' or not script.is_file() or not script.with_name(args.plan + '_protocol.py').is_file():
        parser.error('请提供所选方案 fixed_node/tools/capture.py')
    if not 1 <= seconds <= 86400 or not 1 <= args.groups <= 4096 or (args.tag_mac and not MAC.fullmatch(args.tag_mac)):
        parser.error('秒数、组数或标签 MAC 无效')
    port = os.path.realpath(args.serial) if os.name == 'posix' else args.serial
    try:
        port = probe(script, args.plan, port)
        if args.probe:
            print(f'{port} 已返回 {args.plan} 固定节点 STATUS，3 Mbaud 串口可用')
            return
        token_file = args.token_file.resolve()
        token_file.parent.mkdir(parents=True, exist_ok=True)
        if not token_file.exists():
            fd = os.open(token_file, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(fd, 'w', encoding='ascii') as stream:
                stream.write(secrets.token_urlsafe(32) + '\n')
        token = read_token(token_file)
        collector = Collector(script, port, args.plan, seconds, args.groups, args.output.resolve(), args.tag_mac)
        with ThreadingHTTPServer((args.listen, args.http_port), handler(collector, token)) as server:
            print(f'C5 采集端已启动 http://{args.listen}:{args.http_port}', flush=True)
            print(f'密钥文件 {token_file}\n数据目录 {collector.output}', flush=True)
            try:
                server.serve_forever()
            except KeyboardInterrupt:
                pass
            finally:
                collector.close()
    except (OSError, ValueError, ImportError) as exc:
        parser.exit(1, f'C5 采集端启动失败：{exc}\n')


if __name__ == '__main__':
    main()
