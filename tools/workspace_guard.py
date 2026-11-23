"""Coordinate hardware use by app workspaces on the same Linux host."""
from contextlib import contextmanager
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from urllib.request import ProxyHandler, build_opener


@contextmanager
def hardware_command(root, *, stop=False):
    """Protect browser commands as well as shell starts during a handover."""
    if not (root/'state/workspace.env').is_file():
        yield
        return
    import fcntl
    directory=Path.home()/'.local/state/r550-workspaces'
    if os.environ.get('R550_KIT_ROOT')!=str(root) or not os.environ.get('R550_WORKSPACE_DOMAIN') or (
            os.environ.get('ROS_DOMAIN_ID')!=os.environ['R550_WORKSPACE_DOMAIN']):
        raise ValueError('请从本工程的 ./kit 入口启动实车环境')
    with (directory/'control.lock').open('a') as lock:
        if not stop:
            try:
                fcntl.flock(lock,fcntl.LOCK_SH|fcntl.LOCK_NB)
            except BlockingIOError:
                raise ValueError('实车正在交接') from None
        owner=directory/'owner'
        if not owner.exists() or owner.read_text().strip()!=str(root):
            raise ValueError('当前工程没有实车权')
        yield


def root_processes(root):
    site = str(root / 'rrt_mapping/site_service.py')
    panel = str(root / 'demo/server.py')
    launches = {str(root / 'demo/demo.launch.py'), str(root / 'rrt_mapping/mapping.launch.py')}
    worker = str(root / 'demo/vision_mode.py')
    found = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            args = (entry / 'cmdline').read_bytes().decode(errors='replace').split('\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if site in args:
            kind = 'site'
        elif worker in args and '--worker' in args:
            kind = 'switch'
        elif (panel in args and 'ros' in args) or launches.intersection(args):
            kind = 'hardware'
        else:
            continue
        found[int(entry.name)] = kind
    return found


def panel_state(port, path):
    opener = build_opener(ProxyHandler({}))
    try:
        with opener.open(f'http://127.0.0.1:{int(port)}{path}', timeout=3) as response:
            return json.load(response)
    except (OSError, ValueError) as exc:
        raise ValueError('面板状态暂不可读，保留当前工程的实车权') from exc


def check_quiet(state):
    h = state.get('health', {})
    if (state.get('phase') not in {'idle','completed','stopped','docked','failed'}
            or h.get('stationary') is not True or state.get('recharge_resume_pending')
            or (state.get('fas') or {}).get('lease_live')
            or (h.get('sensor_switch') or {}).get('busy')):
        raise ValueError('请先停止当前任务、续跑或模式切换，等车停稳再交接')


def run_kit(root, args):
    env = dict(os.environ, R550_GUARD_PARENT=str(os.getpid()))
    return subprocess.run(['bash', str(root / 'kit'), *args], cwd=root, env=env).returncode


def devices_free():
    devices=[path for path in ('/dev/wheeltec_controller','/dev/wheeltec_lidar') if Path(path).exists()]
    return not devices or subprocess.run(['fuser','-s',*devices]).returncode==1


def finish_empty_launchers(root, sig):
    """Some Humble launch parents stay alive after every child and both serial ports exit."""
    if not devices_free():
        return
    launch_files={str(root/'demo/demo.launch.py'),str(root/'rrt_mapping/mapping.launch.py')}
    parents=set()
    # The RDK kernel provides /proc/PID/stat but not /proc/PID/task/TID/children.
    for stat_file in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields=stat_file.read_text().rsplit(')',1)[1].split()
            parents.add(int(fields[1]))
        except (FileNotFoundError,ProcessLookupError):
            continue
    for pid,kind in root_processes(root).items():
        if kind!='hardware' or pid in parents:
            continue
        proc=Path('/proc')/str(pid)
        try:
            args=(proc/'cmdline').read_bytes().decode().split('\0')
            if 'launch' not in args or not launch_files.intersection(args):
                continue
            os.kill(pid,sig)
        except (FileNotFoundError, ProcessLookupError):
            continue


def release(root):
    found = root_processes(root)
    if 'switch' in found.values() or (root / 'state/site-mapping/owns-robot').exists():
        raise ValueError('场地或传感器切换任务仍在运行，请先结束任务')
    if 'site' in found.values():
        site = panel_state(os.environ['R550_SITE_PORT'], '/api/site/state')
        if site.get('job') or site.get('live'):
            raise ValueError('建图面板正在使用实车，请先结束建图')
    if 'hardware' in found.values():
        try:
            current=panel_state(os.environ['R550_PANEL_PORT'], '/api/state')
        except ValueError:
            finish_empty_launchers(root,signal.SIGTERM)
            time.sleep(.3)
            finish_empty_launchers(root,signal.SIGKILL)
            time.sleep(.1)
            if 'hardware' in root_processes(root).values():
                raise
        else:
            check_quiet(current)
            if run_kit(root, ['stop']):
                raise ValueError('原工程停车未完成，保留实车权')
    # The mapping web server can start hardware. Close its idle listener too.
    for pid, kind in root_processes(root).items():
        if kind in ('site','hardware'):
            try:
                os.kill(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
    started=time.monotonic()
    while time.monotonic()-started<35:
        remaining=root_processes(root)
        if not remaining and devices_free():
            return
        elapsed=time.monotonic()-started
        if 'hardware' in remaining.values() and elapsed>=6:
            finish_empty_launchers(root,signal.SIGKILL if elapsed>=12 else signal.SIGTERM)
        time.sleep(.25)
    raise ValueError('仍有进程或串口在使用，原工程继续持有实车权')


def dispatch(root, directory, action, args):
    import fcntl
    directory.mkdir(parents=True, exist_ok=True)
    owner_path = directory / 'owner'
    with (directory / 'control.lock').open('a') as lock:
        mode = fcntl.LOCK_SH if action in ('run','status') else fcntl.LOCK_EX
        try:
            fcntl.flock(lock, mode | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError('另一项工程操作正在执行，请稍后再试') from None
        owner = owner_path.read_text().strip() if owner_path.exists() else ''
        if action == 'status':
            print(f'当前实车工程 {owner or "空闲"}')
        elif action == 'select':
            if owner and owner != str(root):
                raise ValueError(f'当前由 {owner} 持有实车权，请先在该工程执行 ./kit profile release')
            with (root/'state/deploy.lock').open('a') as deployed:
                try:
                    fcntl.flock(deployed,fcntl.LOCK_SH|fcntl.LOCK_NB)
                except BlockingIOError:
                    raise ValueError('本工程正在更新，结束后可接管实车') from None
            temp = owner_path.with_suffix('.tmp')
            temp.write_text(str(root) + '\n')
            os.replace(temp, owner_path)
            print(f'实车工程 {root}')
        elif action in ('release','run'):
            if owner != str(root):
                raise ValueError(f'实车权属于 {owner or "空闲"}，当前工程 {root}。预览可用 ./kit preview')
            if action == 'run':
                return run_kit(root, args)
            release(root)
            owner_path.unlink()
            print('实车已释放')
        else:
            raise ValueError('用法：./kit profile status|select|release')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(dispatch(Path(__file__).resolve().parents[1],
                          Path.home()/'.local/state/r550-workspaces',sys.argv[1],sys.argv[2:]))
    except (ValueError, OSError) as error:
        sys.exit(str(error))
