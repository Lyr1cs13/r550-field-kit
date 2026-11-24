"""Interactive 304 acquisition launcher. Run from the 304 desktop console."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from urllib.request import Request, urlopen


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def write(path, data):
    pending = path.with_suffix('.tmp')
    pending.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    pending.replace(path)


def get(url, headers=None):
    with urlopen(Request(url, headers=headers or {}), timeout=4) as response:
        return json.load(response)


def main(choice=None, *, early_launch_voltage=None, expected_run=None):
    kit = Path(__file__).resolve().parents[1]
    bundle = kit.parent
    field = bundle.parent / 'field-data'
    marker = field / 'current-capture.json'
    config_path = bundle / 'host-config/fas.json'
    config = read(config_path)
    route = field / '304-route.json'
    route_data = read(route)
    route_map = route_data['map_id']
    full_count = len(route_data['points'])
    print(f'正式采集 · 新实验使用全部 {full_count} 个点位 · 每对状态 0–4095')
    print('两车将离桩。请确认现场通道可通行，且可以通过巡航面板停车。')
    if choice is None:
        choice = input('输入 START 开始新实验，输入 RESUME 续采，直接回车退出。 ').strip().upper()
    if choice not in ('START', 'RESUME'):
        return
    if marker.exists() and read(marker).get('status') == 'running':
        raise ValueError('还有一轮采集在运行。请先查看采集看板。')
    if choice == 'RESUME':
        info = read(marker)
        run = Path(info['run_dir'])
        if expected_run is not None and (run.name!=expected_run or info['status']!='paused'):
            raise ValueError('待续采实验与操作员确认的实验不一致')
        if not run.resolve().is_relative_to(field.resolve()):
            raise ValueError('实验目录不在采集目录内')
        config_path = Path(info['config'])
        config = read(config_path)
        point_count = info['point_count']
    else:
        run = field / time.strftime('capture-%Y%m%d-%H%M%S')
        point_count = full_count
    key = (config_path.parent / config['base']['key_file']).read_text(encoding='ascii').strip()
    base = get(config['base']['address']+'/fas/health', {'X-FAS-Key':key})
    if not base['ready']:
        raise ValueError('基站尚未就绪')
    for car, url in config['cars'].items():
        state = get(url+'/api/fas/state')
        h, f = state['health'], state['fas']
        if (state['map']['map_id'] != route_map or f['car'] != car or not f['usb'] or f['owner']
                or not all(h[k] for k in ('ready','stationary','localization_confirmed'))):
            raise ValueError(f'{car} 车需要停稳、定位和 USB 就绪，且控制权空闲')
    if choice == 'START':
        run.mkdir()
        config_path = config_path.with_name('fas-'+run.name+'.json')
        config['resume_voltage_by_car'] = {'A':11.4, 'B':11.4}
        write(config_path, config)
    journal = run / 'collection.sqlite'
    pause = run / 'stop.pause'
    pause.unlink(missing_ok=True)
    args = [sys.executable, '-u', str(kit/'tools/fas_experiment.py'), '--route', str(route),
            '--station-count',str(point_count), '--journal',str(journal), '--live-config',str(config_path),
            '--energy-history',str(field/'energy-history.sqlite'),
            '--pair-first','--start','0','--count','4096','--max-pairs','0','--min-samples','30','--window-s','2']
    release_args=[*args,'--release']
    if early_launch_voltage is not None:
        args.extend(('--early-launch-voltage',str(early_launch_voltage)))
    info = {'run_dir':str(run), 'journal':str(journal), 'config':str(config_path),'pause_supported':True,
            'point_count':point_count, 'state_count':4096, 'status':'running'}
    write(marker, info)
    print(f'采集数据库 {journal}')
    code = 1
    try:
        with (run/'console.log').open('a', encoding='utf-8', buffering=1) as log:
            child = subprocess.Popen([*args,'--pause-file',str(pause)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     encoding='utf-8', errors='replace', cwd=run,
                                     env={**os.environ, 'PYTHONIOENCODING':'utf-8'})
            try:
                for line in child.stdout:
                    print(line, end='', flush=True)
                    log.write(line)
            except KeyboardInterrupt:
                pause.touch()
                print('\n正在关闭采样窗口并释放控制。')
            code = child.wait(timeout=45)
    finally:
        info.update(status='completed' if code == 0 else 'paused', exit_code=code)
        if code and journal.exists():
            subprocess.run(release_args, cwd=run, check=False)
        write(marker, info)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--operator-resume',action='store_true')
    parser.add_argument('--expected-run')
    parser.add_argument('--early-launch-voltage',type=float)
    args=parser.parse_args()
    if args.early_launch_voltage is not None and (not args.operator_resume or not args.expected_run):
        parser.error('提前离桩须明确续采哪一个原实验')
    try:
        main('RESUME' if args.operator_resume else None,early_launch_voltage=args.early_launch_voltage,
             expected_run=args.expected_run)
    except (ValueError, OSError, KeyError) as exc:
        print(f'采集未启动或已暂停。{exc}', file=sys.stderr)
