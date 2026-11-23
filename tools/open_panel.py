#!/usr/bin/env python3
"""Open the actual R550 panel on the Windows desktop."""
from http.client import HTTPConnection, HTTPException
import json
import msvcrt
from pathlib import Path
import subprocess
import sys
import time
import webbrowser


ROOT = Path(__file__).resolve().parents[1]
VISION = (ROOT/'demo/vision_node.py').is_file()
DATA = ROOT / 'demo' / 'data' / ('desktop-vision' if VISION else 'desktop-panel')
HOST = '127.0.0.1'
PORT = 8992 if VISION else 8891
URL = f'http://{HOST}:{PORT}/'
ROBOT_PANEL = 'http://192.168.0.100:8990/' if VISION else 'http://192.168.0.100:8890/'


def panel_ready():
    connection = HTTPConnection(HOST, PORT, timeout=0.5)
    try:
        connection.request('GET', '/api/state')
        with connection.getresponse() as response:
            if response.status != 200:
                return False
            state = json.loads(response.read(1024 * 1024))
        health = state.get('health') if isinstance(state, dict) else None
        return (isinstance(health, dict) and health.get('mode') == 'robot'
                and isinstance(state.get('vehicle_connected'), bool))
    except (OSError, HTTPException, ValueError):
        return False
    finally:
        connection.close()


def start_panel():
    error_log = DATA / 'server_error.log'
    args = [sys.executable, '-B', '-X', 'utf8', str(ROOT / 'demo/desktop_panel.py'),
            '--port', str(PORT),
            '--data', str(DATA), '--robot-panel', ROBOT_PANEL]
    with (DATA / 'server.log').open('wb') as output, error_log.open('wb') as errors:
        process = subprocess.Popen(args, cwd=ROOT, stdin=subprocess.DEVNULL,
                                   stdout=output, stderr=errors,
                                   creationflags=subprocess.CREATE_NO_WINDOW)
    deadline = time.monotonic() + 7
    while time.monotonic() < deadline:
        if panel_ready():
            return
        if process.poll() is not None:
            break
        time.sleep(0.15)
    if process.poll() is None:
        process.terminate()
    raise RuntimeError(f'本机巡航面板启动失败。日志在 {error_log}')


def main():
    DATA.mkdir(parents=True, exist_ok=True)
    with (DATA / 'launcher.lock').open('a+b') as lock:
        msvcrt.locking(lock.fileno(), msvcrt.LK_LOCK, 1)
        if not panel_ready():
            start_panel()
    if not webbrowser.open(URL):
        raise RuntimeError(f'请在浏览器打开 {URL}')


if __name__ == '__main__':
    try:
        main()
    except (OSError, RuntimeError) as exc:
        raise SystemExit(str(exc)) from exc
