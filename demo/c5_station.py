"""ESP32-C5 requests from a stopped waypoint and its durable pose records."""
from datetime import datetime, timezone
from http.client import HTTPException
import io
import json
import math
import os
from pathlib import Path
import re
import tempfile
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import zipfile


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'state' / 'c5_client.json'
TOKEN = ROOT / 'state' / 'c5.token'
PROTOCOL = 'r550-csi-v1'
VISIT_ID = re.compile(r'[0-9a-f]{32}')
MAC = re.compile(r'(?:[0-9a-f]{2}:){5}[0-9a-f]{2}')


def utc():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, data):
    """Replace a complete record only after writing and syncing its contents."""
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, encoding='utf-8', delete=False) as temporary:
        name = Path(temporary.name)
        try:
            json.dump(data, temporary, ensure_ascii=False, indent=2, allow_nan=False)
            temporary.write('\n')
            temporary.flush()
            os.fsync(temporary.fileno())
        except BaseException:
            temporary.close()
            name.unlink(missing_ok=True)
            raise
    try:
        os.replace(name, path)
    finally:
        name.unlink(missing_ok=True)


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def read_token(path):
    token = path.read_text(encoding='ascii').strip()
    if not re.fullmatch(r'[A-Za-z0-9_-]{24,128}', token):
        raise ValueError('C5 采集密钥格式不正确，请使用采集端生成的 token 文件')
    return token


def validate_visit(data):
    keys = ('visit_id', 'session_id', 'map_id', 'point_id', 'point_name', 'arrived_utc', 'actual_pose')
    if not isinstance(data, dict) or any(key not in data for key in keys):
        raise ValueError('C5 停靠记录字段缺失')
    visit = {key: data[key] for key in keys}
    for key in ('visit_id', 'session_id'):
        if not isinstance(visit[key], str) or not VISIT_ID.fullmatch(visit[key]):
            raise ValueError('C5 停靠编号或会话编号无效')
    if not isinstance(visit['map_id'], str) or not re.fullmatch(r'[0-9a-f]{64}', visit['map_id']):
        raise ValueError('C5 停靠记录缺少有效的地图版本')
    for key in ('point_id', 'point_name'):
        if not isinstance(visit[key], str) or not 0 < len(visit[key]) <= 64 or any(ord(c) < 32 for c in visit[key]):
            raise ValueError('C5 点位名称或编号无效')
    timestamp = datetime.fromisoformat(visit['arrived_utc'])
    if timestamp.tzinfo is None:
        raise ValueError('C5 到站时间应包含时区')
    pose = visit['actual_pose']
    if (not isinstance(pose, (list, tuple)) or len(pose) != 3
            or not all(type(v) in (int, float) and math.isfinite(v) for v in pose)):
        raise ValueError('C5 采集需要有效的实际地图位姿')
    visit.update(actual_pose=list(pose), frame_id='map', base_frame='base_footprint',
                 position_source='ros_localization')
    return visit


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


class CollectorLink:
    def __init__(self, url, token):
        parts = urllib.parse.urlsplit(url)
        if (parts.scheme not in ('http', 'https') or not parts.hostname or parts.username is not None
                or parts.path not in ('', '/') or parts.query or parts.fragment):
            raise ValueError('填写 C5 采集端地址，例如 http://192.168.0.210:8791')
        self.url, self.token = url.rstrip('/'), token
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())

    @classmethod
    def configured(cls):
        if not CONFIG.is_file() or not TOKEN.is_file():
            raise ValueError('请先复制采集端的 state/c5.token，并运行 ./kit c5-config 采集端地址')
        return cls(read_json(CONFIG)['url'], read_token(TOKEN))

    def request(self, path, data=None, timeout=2):
        request = urllib.request.Request(self.url + '/v1/' + path,
                                        data=None if data is None else json.dumps(data, allow_nan=False).encode(),
                                        headers={'Authorization': 'Bearer ' + self.token, 'Content-Type': 'application/json'})
        with self.opener.open(request, timeout=timeout) as response:
            result = json.loads(response.read(65537))
        if not isinstance(result, dict) or result.get('protocol') != PROTOCOL:
            raise ValueError('C5 采集端协议不匹配')
        return result

    def ready(self):
        result = self.request('status', timeout=4)
        if not result.get('ready'):
            raise ValueError(result.get('error') or 'C5 采集端正在处理上一轮')
        if result.get('plan') not in ('fixed', 'variable'):
            raise ValueError('C5 采集端未报告有效的采集方案')
        return result['plan']


class C5Station:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.link = None
        self.plan = None
        self.lock = threading.RLock()
        self.worker = None
        self.cancel_requested = threading.Event()
        self.status = {'stage': 'disabled'}

    @property
    def enabled(self):
        return self.link is not None

    @property
    def active(self):
        return self.status['stage'] in ('starting', 'sampling', 'canceling')

    def selection(self, enabled):
        if self.active:
            raise ValueError('请等当前 C5 采集停止')
        if not enabled:
            return None, None
        try:
            link = CollectorLink.configured()
        except (OSError, KeyError) as exc:
            raise ValueError(f'无法读取 C5 采集端配置：{exc}') from exc
        return link, self.check(link)

    def configure(self, link, plan):
        with self.lock:
            self.link, self.plan = link, plan
            self.status = {'stage': 'ready' if link else 'disabled'}

    def check(self, link=None):
        try:
            return (link or self.link).ready()
        except urllib.error.HTTPError as exc:
            raise ValueError(f'C5 采集端 HTTP {exc.code}，请核对地址和密钥') from exc
        except (OSError, HTTPException, KeyError) as exc:
            raise ValueError(f'C5 采集端未就绪：{exc}') from exc

    def snapshot(self):
        with self.lock:
            return {**self.status, 'enabled': self.enabled, 'active': self.active, 'plan': self.plan}

    def begin(self, visit):
        visit = validate_visit(visit)
        with self.lock:
            if not self.enabled or self.active:
                raise ValueError('C5 采集正在运行或尚未配置')
            self.directory.mkdir(parents=True, exist_ok=True)
            self.record = {**visit, 'state': 'starting', 'plan': self.plan}
            self.path = self.directory / (visit['visit_id'] + '.json')
            write_json(self.path, self.record)
            self.status = {'stage': 'starting', 'visit_id': visit['visit_id'], 'point_name': visit['point_name']}
            self.cancel_requested.clear()
            self.worker = threading.Thread(target=self._capture, args=(visit,), name='c5-station', daemon=True)
            self.worker.start()

    def cancel(self):
        with self.lock:
            if self.active:
                self.cancel_requested.set()
                self.status['stage'] = 'canceling'

    def close(self):
        self.cancel()
        if self.worker:
            self.worker.join(timeout=3)

    def _finish(self, state, **data):
        with self.lock:
            self.record.update(state=state, finished_utc=utc(), **data)
            write_json(self.path, self.record)
            self.status.update(stage=state, **data)

    def _capture(self, visit):
        link = self.link
        identity = visit['visit_id']
        endpoint = 'visits/' + identity
        started = time.monotonic()
        try:
            # Retries use the same ID if the collector accepted START but the reply was lost.
            for attempt in range(3):
                if self.cancel_requested.is_set():
                    break
                try:
                    link.request('visits', visit)
                    break
                except urllib.error.HTTPError:
                    raise
                except (OSError, HTTPException):
                    if attempt == 2:
                        raise
            failures = 0
            cancel_sent = False
            while True:
                if self.cancel_requested.is_set() and not cancel_sent:
                    try:
                        link.request(endpoint + '/cancel', {})
                    except urllib.error.HTTPError as exc:
                        if exc.code != 404:
                            raise
                        self._finish('canceled')
                        return
                    cancel_sent = True
                try:
                    receipt = link.request(endpoint)
                except urllib.error.HTTPError:
                    raise
                except (OSError, HTTPException):
                    failures += 1
                    if failures == 3:
                        raise
                else:
                    failures = 0
                    if receipt.get('visit_id') != identity:
                        raise ValueError('C5 返回了其他停靠点的结果')
                    state = receipt.get('state')
                    if state == 'saved':
                        result = receipt.get('result', {})
                        if (result.get('plan') != self.plan or not result.get('run')
                                or not isinstance(result.get('valid_csi'), int) or result['valid_csi'] < 1):
                            raise ValueError('C5 本轮缺少已保存的有效标签数据')
                        if self.cancel_requested.is_set():
                            self._finish('canceled', result=result)
                        else:
                            self._finish('done', result=result)
                        return
                    if state in ('failed', 'canceled', 'interrupted'):
                        self._finish('canceled' if self.cancel_requested.is_set() else 'error',
                                     error=receipt.get('error') or 'C5 采集被中断')
                        return
                    if state != 'sampling':
                        raise ValueError('C5 采集状态无法识别')
                    with self.lock:
                        self.status.update(stage='canceling' if cancel_sent else 'sampling',
                                           elapsed=round(time.monotonic() - started))
                time.sleep(.65)
        except (OSError, HTTPException, ValueError, KeyError, TypeError) as exc:
            try:
                link.request(endpoint + '/cancel', {}, timeout=1)
            except (OSError, HTTPException, ValueError):
                pass
            error = 'C5 采集未确认保存：' + str(exc)
            try:
                self._finish('error', error=error)
            except OSError:
                with self.lock:
                    self.status.update(stage='error', error=error)

    def release(self):
        with self.lock:
            if self.status['stage'] != 'done':
                raise ValueError('当前 C5 停靠尚未保存完成')
            self.record['released_utc'] = utc()
            write_json(self.path, self.record)

    def export(self, maps):
        output = io.BytesIO()
        with self.lock, zipfile.ZipFile(output, 'w', zipfile.ZIP_DEFLATED) as archive:
            map_ids = set()
            for path in sorted(self.directory.glob('*.json')):
                if VISIT_ID.fullmatch(path.stem):
                    record = read_json(path)
                    map_ids.add(record['map_id'])
                    archive.write(path, 'visits/' + path.name)
            for map_id in sorted(map_ids):
                for path in (maps / map_id).rglob('*'):
                    if path.is_file():
                        archive.write(path, 'maps/' + path.relative_to(maps).as_posix())
        return output.getvalue()
