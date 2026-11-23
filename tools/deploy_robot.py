#!/usr/bin/env python3
"""Prepare or deploy source from the VM; keep vehicle scene/data and build on ARM."""
import argparse
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import time
import types
import zipfile

ROOT=Path(__file__).resolve().parents[1]
SCENE_FILES={'map.yaml','WHEELTEC.yaml','WHEELTEC.pgm','threshold_keepout.yaml','threshold_keepout.pgm',
             'boundary.json','dock_start.json','map_metadata.json','survey_settings.json','survey_route.json',
             'corridor_route.json','manifest.json'}
PRESERVE={'config/vehicle.yaml','config/endurance.yaml','demo/vision_config.json'}


def workspace_target(manifest, target=None, home=None):
    home=home or Path.home()
    variant='vision' if 'demo/vision_node.py' in manifest else 'patrol'
    profile=home/('r550-'+variant)
    if target is None:
        target=profile if (profile/'state/workspace.env').is_file() else home/'lab_cruise_ws'
    target=target.expanduser().resolve()
    settings=target/'state/workspace.env'
    if settings.is_file():
        name=re.search(r'^export R550_WORKSPACE_NAME=(\w+)$',settings.read_text(),re.M)
        if not name or name[1]!=variant:
            raise ValueError('部署包与目标工程类型不符：'+str(target))
        if not {'tools/workspace_guard.py','tools/ros_environment.sh'}.issubset(manifest):
            raise ValueError('部署包缺少双工程隔离入口，请从对应工程打包')
    return target


def source_files(root):
    for name in ('kit','README.md','AGENTS.md','LICENSE','NOTICE.md'):
        yield root/name
    for directory in ('demo','tools','config','docs','vendor','rrt_mapping','scenes/demo-room'):
        for path in (root/directory).rglob('*'):
            if not path.is_file():continue
            relative=path.relative_to(root)
            if any(p in ('data','__pycache__','build','install','log') for p in relative.parts):continue
            if path.suffix in ('.pyc','.bag','.db3','.mcap','.fwpkg') or path.name=='local.env':continue
            if relative.parts[:2]==('demo','assets') and path.name in SCENE_FILES:continue
            yield path


def prepare(root, output):
    files=sorted(set(source_files(root)))
    manifest={p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    output.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(output,'w',zipfile.ZIP_DEFLATED) as archive:
        for path in files:archive.write(path,path.relative_to(root).as_posix())
        archive.writestr('DEPLOY_MANIFEST.json',json.dumps(manifest,indent=2))
    return manifest


def apply_archive(archive_path, target):
    """No ROS/motion commands; verify payload, back up replacements, retain scene."""
    target=target.resolve()
    with zipfile.ZipFile(archive_path) as archive:
        manifest=json.loads(archive.read('DEPLOY_MANIFEST.json'))
        workspace_target(manifest,target)
        for name,digest in manifest.items():
            destination=(target/name).resolve()
            if not destination.is_relative_to(target) or name.startswith('/'):
                raise ValueError('部署包路径越界')
            if hashlib.sha256(archive.read(name)).hexdigest()!=digest:raise ValueError('部署包校验失败：'+name)
            if (name.startswith(('state/','demo/data/')) or
                    name.startswith('demo/assets/') and Path(name).name in SCENE_FILES):
                raise ValueError('部署包不能覆盖场景或运行数据')
        module=types.ModuleType('deployment_map_validation')
        exec(compile(archive.read('demo/map_grid.py'),'map_grid.py','exec'),module.__dict__)
        original_grid=module.MapGrid(target/'demo/assets')
        original_scene={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (target/'demo/assets').iterdir() if p.is_file() and p.name in SCENE_FILES}
        backup=target/'state/backups'/('source_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid()))
        backup.mkdir(parents=True,exist_ok=False)
        applied={};kept=[]
        for name,digest in manifest.items():
            destination=target/name
            if name in PRESERVE and destination.exists():kept.append(name);continue
            if destination.exists():
                saved=backup/name;saved.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(destination,saved)
            destination.parent.mkdir(parents=True,exist_ok=True)
            temporary=destination.with_name(destination.name+'.deploy-tmp')
            data=archive.read(name)
            if name=='rrt_mapping/navigation.yaml' and (target/'state/workspace.env').is_file():
                data=data.replace(b'/home/wheeltec/lab_cruise_ws/rrt_mapping/navigation_recovery.xml',
                                  os.fsencode(target/'rrt_mapping/navigation_recovery.xml'))
                digest=hashlib.sha256(data).hexdigest()
            temporary.write_bytes(data);os.replace(temporary,destination)
            if destination.suffix=='.sh' or name=='kit':destination.chmod(0o755)
            applied[name]=digest
        report={'backup':str(backup),'applied':applied,'preserved_configuration':kept}
        if module.MapGrid(target/'demo/assets').map_id!=original_grid.map_id:raise RuntimeError('地图校验发生变化')
        if any(hashlib.sha256((target/'demo/assets'/name).read_bytes()).hexdigest()!=digest for name,digest in original_scene.items()):raise RuntimeError('场景资产发生变化')
        report['preserved_map_id']=original_grid.map_id
        (target/'state/deployed_source.json').write_text(json.dumps(report,indent=2))
    # Adopt the existing vehicle assets; never replace them with the VM map.
    active=target/'state/active_scene.json'
    if not active.exists():
        sys.path.insert(0,str(target/'demo'))
        from map_grid import MapGrid
        grid=MapGrid(target/'demo/assets')
        scene=target/'scenes/vehicle-existing';scene.mkdir(parents=True,exist_ok=True)
        for name in SCENE_FILES:
            source=target/'demo/assets'/name
            if source.exists():shutil.copy2(source,scene/name)
        active.write_text(json.dumps({'scene':'vehicle-existing','map_id':grid.map_id},indent=2))
    print('源码已校验并部署；原地图、桩位和数据保留。备份：'+str(backup))


@contextmanager
def deployment_window(target):
    if (target/'state/workspace.env').is_file():
        import fcntl
        owners=Path.home()/'.local/state/r550-workspaces'
        owners.mkdir(parents=True,exist_ok=True)
        with (owners/'control.lock').open('a') as control,(target/'state/deploy.lock').open('a') as updating:
            fcntl.flock(updating,fcntl.LOCK_EX|fcntl.LOCK_NB)
            fcntl.flock(control,fcntl.LOCK_SH|fcntl.LOCK_NB)
            owner=owners/'owner'
            if owner.exists() and owner.read_text().strip()==str(target):
                raise SystemExit('更新此工程前请先执行 ./kit profile release')
            fcntl.flock(control,fcntl.LOCK_UN)
            yield
        return
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name)==os.getpid():continue
        try:command=(proc/'cmdline').read_bytes()
        except OSError:continue
        if any(token in command for token in (b'wheeltec_robot_node',b'controller_server',b'slam_toolbox_node',b'demo/server.py',b'motion_gate.py')):
            raise SystemExit('车端底盘/导航仍在运行。先停车并关闭旧程序，再重新部署。未覆盖源码。')
    yield


def remote_apply(archive,target,build=False):
    with zipfile.ZipFile(archive) as src:
        manifest=json.loads(src.read('DEPLOY_MANIFEST.json'))
        target=workspace_target(manifest,target)
        if (target/'state/workspace.env').is_file() and b'tools/workspace_guard.py' not in src.read('kit'):
            raise ValueError('部署包的启动入口没有实车互斥')
    if not (target/'demo/assets/map.yaml').is_file():raise SystemExit('车端没有既有场景，停止部署；先导入实际地图。')
    with deployment_window(target):
        apply_archive(archive,target)
        if build:
            subprocess.run(['bash','-c',BUILD],cwd=target,check=True)
    print('车端工程：'+str(target),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--host',default='r550')
    parser.add_argument('--remote-apply',type=Path)
    parser.add_argument('--target',help='车端工程目录；默认按源码类型选择已隔离工程')
    parser.add_argument('--build',action='store_true',help=argparse.SUPPRESS)
    args=parser.parse_args()
    if args.remote_apply:
        remote_apply(args.remote_apply,Path(args.target) if args.target else None,args.build);return
    archive=ROOT/'state/deployment/r550-source.zip';manifest=prepare(ROOT,archive)
    print(f'已准备{len(manifest)}个源码文件：{archive}',flush=True)
    if args.prepare_only:return
    if not re.fullmatch(r'[A-Za-z0-9_.@-]+',args.host) or args.host.startswith('-'):raise SystemExit('无效SSH主机')
    remote=subprocess.check_output(['ssh',args.host,'mktemp -d /tmp/r550_deploy_XXXXXXXX'],text=True).strip()
    if not re.fullmatch(r'/tmp/r550_deploy_[A-Za-z0-9]+',remote):raise ValueError('无法准备车端临时目录')
    subprocess.run(['scp',str(archive),str(Path(__file__).resolve()),args.host+':'+remote+'/'],check=True)
    command='python3 '+shlex.quote(remote+'/deploy_robot.py')+' --remote-apply '+shlex.quote(remote+'/r550-source.zip')
    if args.target:command+=' --target '+shlex.quote(args.target)
    subprocess.run(['ssh','-t',args.host,command+' --build'],check=True)


BUILD="""set -eo pipefail
source /opt/ros/humble/setup.bash
missing=0
for package in nav2_collision_monitor nav2_mppi_controller nav2_bringup rmw_cyclonedds_cpp; do
  ros2 pkg prefix "$package" >/dev/null 2>&1 || missing=1
done
python3 -c 'import yaml,serial,numpy,scipy,PIL' || missing=1
command -v screen >/dev/null || missing=1
command -v colcon >/dev/null || missing=1
if [[ -f state/workspace.env ]]; then
  [[ "$missing" = 0 ]] || { echo '请协调安装缺少的共享系统依赖'; exit 1; }
  nice -n 15 bash tools/install.sh robot --skip-apt
elif [[ "$missing" = 0 ]]; then
  bash kit setup robot --skip-apt
else
  bash kit setup robot
fi
./kit doctor --navigation"""


if __name__=='__main__':main()
