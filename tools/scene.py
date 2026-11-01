#!/usr/bin/env python3
"""Create, activate and configure portable scene bundles."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import os
import sys
import time
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'demo'))
from map_grid import MapGrid, pgm


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def scene_dir(name):
    if not name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in name):
        raise ValueError('场景名使用字母、数字、横线或下划线')
    return ROOT/'scenes'/name


def ensure_stopped():
    if os.name!='nt' and Path('/proc').exists():
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit():continue
            try:cmd=(proc/'cmdline').read_bytes()
            except OSError:continue
            if any(x in cmd for x in (b'wheeltec_robot_node',b'controller_server',b'sync_slam_toolbox_node',b'async_slam_toolbox_node')):raise ValueError('先运行 ./kit stop 关闭底盘/建图/导航，再切换场景')
    try:
        with urllib.request.urlopen('http://127.0.0.1:8890/api/state', timeout=1) as r:
            s = json.load(r)
    except urllib.error.URLError:
        return
    if s['phase'] not in ('idle', 'docked', 'completed', 'failed') or not s['health']['stationary']:
        raise ValueError('先停止任务并停稳，再切换场景')
    raise ValueError('先运行 ./kit stop 关闭面板与导航，再切换场景')


def make_mask(directory, rectangles):
    for rect in rectangles:
        if len(rect)!=4 or not all(math.isfinite(v) for v in rect) or rect[0]>=rect[1] or rect[2]>=rect[3]:raise ValueError('禁区矩形使用 xmin xmax ymin ymax')
    import yaml
    info = yaml.safe_load((directory/'map.yaml').read_text(encoding='utf-8'))
    w, h, _ = pgm(directory/info['image']);res = float(info['resolution']);ox, oy, angle = info['origin']
    if abs(angle) > 1e-8:raise ValueError('请导入 yaw=0 的地图')
    pixels = bytearray([254])*(w*h)
    for row in range(h):
        y = oy+(h-row-.5)*res
        for col in range(w):
            x = ox+(col+.5)*res
            if any(a <= x <= b and c <= y <= d for a,b,c,d in rectangles):pixels[row*w+col] = 0
    (directory/'threshold_keepout.pgm').write_bytes(f'P5\n{w} {h}\n255\n'.encode()+pixels)
    info['image'] = 'threshold_keepout.pgm'
    (directory/'threshold_keepout.yaml').write_text(yaml.safe_dump(info), encoding='utf-8')


def create(name, source):
    import yaml
    target = scene_dir(name)
    target.mkdir(exist_ok=False)
    source = source.expanduser().resolve();info = yaml.safe_load(source.read_text(encoding='utf-8'))
    image = (source.parent/info['image']).resolve()
    if image.suffix.lower() == '.pgm':shutil.copy2(image, target/'WHEELTEC.pgm')
    else:
        from PIL import Image
        Image.open(image).convert('L').save(target/'WHEELTEC.pgm')
    info['image'] = 'WHEELTEC.pgm';info.setdefault('negate',0)
    text = yaml.safe_dump(info)
    (target/'map.yaml').write_text(text, encoding='utf-8');(target/'WHEELTEC.yaml').write_text(text, encoding='utf-8')
    make_mask(target, [])
    w,h,_ = pgm(target/'WHEELTEC.pgm');ox,oy,angle = info['origin'];res=info['resolution']
    if abs(angle)>1e-8:raise ValueError('请导入 yaw=0 的地图')
    bounds = [ox,ox+w*res,oy,oy+h*res]
    write_json(target/'boundary.json', {'rectangle': bounds})
    write_json(target/'map_metadata.json', {'version': name, 'scene': name})
    write_json(target/'survey_settings.json', {'bounds': bounds, 'display_bounds': bounds, 'bin_origin':[0,0]})
    write_json(target/'dock_start.json', {'calibrated':False,'robot_pose':{'x':0.,'y':0.,'yaw':0.},
               'charger_approach':{'p_x':.6,'p_y':0.,'orien_z':0.,'orien_w':1.},'departure_distance':.3})
    print('已建立场景：',target)
    return target


def activate(name):
    ensure_stopped();directory=scene_dir(name);grid=MapGrid(directory)
    assets=ROOT/'demo/assets';backup=ROOT/'state/backups'/time.strftime('scene_%Y%m%d_%H%M%S')
    backup.mkdir(parents=True, exist_ok=False)
    names=['map.yaml','WHEELTEC.yaml','WHEELTEC.pgm','threshold_keepout.pgm','threshold_keepout.yaml',
           'boundary.json','dock_start.json','map_metadata.json','survey_settings.json','survey_route.json']
    for name in names:
        old=assets/name
        if old.exists():shutil.copy2(old,backup/name)
        if (directory/name).exists():shutil.copy2(directory/name,old)
        elif name=='survey_route.json':old.unlink(missing_ok=True)
    write_json(ROOT/'state/active_scene.json',{'scene':directory.name,'map_id':grid.map_id})
    print('已启用：',directory.name,'map_id:',grid.map_id)


def dock(name,x,y,degrees,approach_distance=.6,approach_angle=-15.):
    directory=scene_dir(name);grid=MapGrid(directory);grid.validate(x,y,0.)
    angle=math.radians(degrees);approach=angle+math.radians(approach_angle)
    write_json(directory/'dock_start.json', {'calibrated':True,'robot_pose':{'x':x,'y':y,'yaw':angle},
        'departure_distance':.3,'charger_approach':{'p_x':x+approach_distance*math.cos(angle),
            'p_y':y+approach_distance*math.sin(angle),'orien_z':math.sin(approach/2),'orien_w':math.cos(approach/2)}})
    print('已保存桩位。运行 ./kit scene use',name,'应用。')


def route(name, anchors_file, speed=.08):
    directory=scene_dir(name);grid=MapGrid(directory);anchors=json.loads(anchors_file.read_text(encoding='utf-8'))
    # Separate legs explicitly at the turnaround. Coordinates follow the driven path.
    if not isinstance(anchors,list) or not anchors:raise ValueError('路线文件是路段数组，详见 config/survey-route.example.json')
    points=[];preview=[]
    for i,leg in enumerate(anchors):
        if len(leg)<2:raise ValueError('每个路段至少两个坐标')
        rounded=[leg[0]]
        for a,b,c in zip(leg,leg[1:],leg[2:]):
            if math.dist(a,b)<1e-6 or math.dist(b,c)<1e-6:raise ValueError('路线包含重复坐标')
            trim=min(.35,math.dist(a,b)*.30,math.dist(b,c)*.30)
            p=[b[j]+(a[j]-b[j])*trim/math.dist(a,b) for j in range(2)]
            q=[b[j]+(c[j]-b[j])*trim/math.dist(b,c) for j in range(2)]
            curve=[[(1-t)**2*p[j]+2*(1-t)*t*b[j]+t*t*q[j] for j in range(2)] for t in [k/20 for k in range(21)]]
            try:
                for point in curve:grid.validate(*point,.23)
            except ValueError:rounded.append(b)
            else:rounded.extend(curve)
        rounded.append(leg[-1]);leg=rounded
        dense=[leg[0]]
        for a,b in zip(leg,leg[1:]):
            count=max(1,math.ceil(math.dist(a,b)/.025))
            for k in range(1,count+1):dense.append([a[j]+(b[j]-a[j])*k/count for j in range(2)])
        for p in dense:grid.validate(*p,.23)
        if preview and math.dist(preview[-1],dense[0])>.05:raise ValueError('相邻路段首尾要衔接')
        x,y=dense[-1];points.append({'id':f'survey-leg-{i+1}','name':f'采集路段 {i+1}',
            'x':x,'y':y,'yaw':0.,'wait':0.,'enabled':True,'survey_pass_radius':.16,'survey_path':dense})
        preview.extend(dense)
    write_json(directory/'survey_route.json',{'schema':1,'map_id':grid.map_id,'points':points,'preview_path':preview,'planned_grid_path_length_m':sum(math.dist(a,b) for a,b in zip(preview,preview[1:]))})
    print('已保存',len(points),'段连续路线；场景重新启用后在面板加载。')


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='action',required=True)
    c=sub.add_parser('new');c.add_argument('name');c.add_argument('--map',required=True,type=Path)
    c=sub.add_parser('use');c.add_argument('name')
    c=sub.add_parser('dock');c.add_argument('name');c.add_argument('--pose',nargs=3,required=True,type=float,metavar=('X','Y','DEG'));c.add_argument('--distance',type=float,default=.6);c.add_argument('--angle',type=float,default=-15.)
    c=sub.add_parser('keepout');c.add_argument('name');c.add_argument('--rect',nargs=4,type=float,action='append',default=[])
    c=sub.add_parser('survey');c.add_argument('name');c.add_argument('--bounds',nargs=4,type=float);c.add_argument('--route',type=Path)
    sub.add_parser('list')
    a=p.parse_args()
    if a.action=='new':create(a.name,a.map)
    elif a.action=='use':activate(a.name)
    elif a.action=='dock':dock(a.name,*a.pose,a.distance,a.angle)
    elif a.action=='keepout':make_mask(scene_dir(a.name),a.rect);print('禁区已保存；重新启用场景应用。')
    elif a.action=='survey':
        if a.bounds:
            q=scene_dir(a.name)/'survey_settings.json';value=json.loads(q.read_text());value['bounds']=a.bounds;write_json(q,value)
        if a.route:route(a.name,a.route)
    else:
        for d in (ROOT/'scenes').iterdir():
            if d.is_dir():print(d.name)


if __name__=='__main__':main()
