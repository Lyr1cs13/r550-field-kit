#!/usr/bin/env python3
"""Export equal-spacing points along supplied scene coordinates; never start motion."""
import argparse
import json
import math
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'demo'))
from directed_route import uniform_route
from map_grid import MapGrid


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True)
    p.add_argument('--vertices',type=Path,required=True,help='JSON array [[x,y],...]; repeat first vertex to close')
    p.add_argument('--count',type=int,default=50)
    p.add_argument('--heading-deg',type=float,required=True)
    p.add_argument('--wait',type=float,default=3.)
    p.add_argument('--clearance',type=float,default=.20,help='minimum static map clearance in metres')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();grid=MapGrid(a.scene)
    vertices=json.loads(a.vertices.read_text(encoding='utf-8'))
    points=uniform_route(vertices,a.count,math.radians(a.heading_deg),a.wait)
    if not math.isfinite(a.clearance) or not .18 <= a.clearance <= .60:
        p.error('clearance must be .18.. .60 m')
    # Validate the entire explicitly drawn corridor, not just its sample points.
    for start,end in zip(vertices,vertices[1:]):
        n=max(1,math.ceil(math.dist(start,end)/min(.04,grid.resolution)))
        for i in range(n+1):
            grid.validate(start[0]+(end[0]-start[0])*i/n,start[1]+(end[1]-start[1])*i/n,a.clearance)
    for point in points:
        grid.validate(point['x'],point['y'],a.clearance)
    a.output.write_text(json.dumps(dict(schema=1,map_id=grid.map_id,points=points,vertices=vertices,
        clearance_m=a.clearance),ensure_ascii=False,indent=2),encoding='utf-8')
    print(f'已生成{len(points)}个统一朝向点位；导入面板后核对地图，不会自动发车。')


if __name__=='__main__':main()
