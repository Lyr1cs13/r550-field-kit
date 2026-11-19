#!/usr/bin/env python3
"""Prepare or deploy source from the VM; keep vehicle scene/data and build on ARM."""
import argparse
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
             'boundary.json','dock_start.json','map_metadata.json','survey_settings.json','survey_route.json','manifest.json'}
PRESERVE={'config/vehicle.yaml','config/endurance.yaml'}


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
        for name,digest in manifest.items():
            destination=(target/name).resolve()
            if not destination.is_relative_to(target) or name.startswith('/'):
                raise ValueError('部署包路径越界')
            if hashlib.sha256(archive.read(name)).hexdigest()!=digest:raise ValueError('部署包校验失败：'+name)
            if name.startswith('demo/assets/') and Path(name).name in SCENE_FILES:raise ValueError('部署包不能覆盖场景')
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
            temporary.write_bytes(archive.read(name));os.replace(temporary,destination)
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


def remote_apply(archive, target):
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name)==os.getpid():continue
        try:command=(proc/'cmdline').read_bytes()
        except OSError:continue
        if any(token in command for token in (b'wheeltec_robot_node',b'controller_server',b'slam_toolbox_node',b'demo/server.py',b'motion_gate.py')):
            raise SystemExit('车端底盘/导航仍在运行。先停车并关闭旧程序，再重新部署。未覆盖源码。')
    # Require an existing, usable field scene before writing source.
    if not (target/'demo/assets/map.yaml').is_file():raise SystemExit('车端没有既有场景，停止部署；先导入实际地图。')
    apply_archive(archive,target)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--host',default='r550')
    parser.add_argument('--remote-apply',type=Path)
    parser.add_argument('--target',type=Path,default=Path.home()/'lab_cruise_ws')
    args=parser.parse_args()
    if args.remote_apply:
        remote_apply(args.remote_apply,args.target);return
    archive=ROOT/'state/deployment/r550-source.zip';manifest=prepare(ROOT,archive)
    print(f'已准备{len(manifest)}个源码文件：{archive}',flush=True)
    if args.prepare_only:return
    if not re.fullmatch(r'[A-Za-z0-9_.@-]+',args.host) or args.host.startswith('-'):raise SystemExit('无效SSH主机')
    stamp=time.strftime('%Y%m%d_%H%M%S');remote='/tmp/r550_deploy_'+stamp
    subprocess.run(['ssh',args.host,'mkdir -p '+shlex.quote(remote)],check=True)
    subprocess.run(['scp',str(archive),str(Path(__file__).resolve()),args.host+':'+remote+'/'],check=True)
    command='python3 '+shlex.quote(remote+'/deploy_robot.py')+' --remote-apply '+shlex.quote(remote+'/r550-source.zip')
    subprocess.run(['ssh','-t',args.host,command],check=True)
    # Compiles on the vehicle architecture. No VM binary is shipped.
    build="""set -eo pipefail
cd ~/lab_cruise_ws && source /opt/ros/humble/setup.bash &&
missing=0
for package in nav2_collision_monitor nav2_mppi_controller nav2_bringup rmw_cyclonedds_cpp; do
  ros2 pkg prefix "$package" >/dev/null 2>&1 || missing=1
done
python3 -c 'import yaml,serial,numpy,scipy,PIL' || missing=1
command -v screen >/dev/null || missing=1
command -v colcon >/dev/null || missing=1
if [ "$missing" = 0 ]; then bash kit setup robot --skip-apt; else bash kit setup robot; fi && ./kit doctor --navigation"""
    subprocess.run(['ssh','-t',args.host,'bash -lc '+shlex.quote(build)],check=True)
    print('车端部署与检查完成。尚未启动导航或发车；下一步在车端执行：cd ~/lab_cruise_ws && ./kit nav')


if __name__=='__main__':main()
