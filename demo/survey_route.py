"""Load a checked route belonging to the active scene."""
import json, math
from scene_settings import settings

def north_route(grid):
    p=grid.directory/'survey_route.json'
    if not p.exists():raise ValueError('先使用 ./kit scene survey 配置连续采集路线')
    value=json.loads(p.read_text(encoding='utf-8'))
    if value['map_id']!=grid.map_id:raise ValueError('采集路线与当前地图不一致，请重新配置')
    return value

def north_grid_route(grid,spacing=.5,clearance=.28):
    a,b,c,d=settings(grid)['bounds'];points=[];i=0;x=a+spacing/2
    while x<b:
        y=c+spacing/2
        while y<d:
            try:grid.validate(x,y,clearance)
            except ValueError:y+=spacing;continue
            i+=1;points.append(dict(id=f'sample-{i}',name=f'采样 {i}',x=x,y=y,yaw=0.,wait=7.,enabled=True));y+=spacing
        x+=spacing
    points=points[::max(1,math.ceil(len(points)/100))]
    return dict(map_id=grid.map_id,points=points,spacing_m=spacing,clearance_m=clearance,planned_grid_path_length_m=0.)
