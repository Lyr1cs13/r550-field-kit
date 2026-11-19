#!/usr/bin/env python3
"""Static chord coverage and stopped-pose sensitivity for one saved station route."""
import argparse
from collections import Counter
from functools import lru_cache
import json
import math
from pathlib import Path
import random
import sys

from fas_spin_benchmark import R550_TURN, extract_turns, quantile, simulate
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'demo'))
from arrival_policy import FAS_STATION_POSITION_TOLERANCE, FAS_STATION_YAW_TOLERANCE, FAS_TURN_MARGIN_ANGLE, refinement_stage
from directed_route import angle_error, fas_station_leg
from map_grid import MapGrid


def analyze(scene, route_path, traces, seed):
    grid=MapGrid(scene)
    route=json.loads(route_path.read_text(encoding='utf-8'))
    if route['map_id']!=grid.map_id:
        raise ValueError('Route and occupancy grid must share map_id')
    points=route['points']
    held=[]
    for path in traces:
        turns=list(extract_turns(path))
        split=max(1,len(turns)*2//3) if len(turns)>1 else 0
        held.extend(turns[split:])
    if not held:
        raise ValueError('No held-out turns in the input traces')

    @lru_cache(maxsize=None)
    def clear(ix,iy,clearance):
        try:
            grid.validate(*grid.world(ix,iy),clearance=clearance)
        except ValueError:
            return False
        return True

    all_clear=0
    adjacent=[]
    for i,origin in enumerate(points):
        pose=(origin['x'],origin['y'],origin['yaw'])
        for j,goal in enumerate(points):
            if i==j:
                continue
            leg=fas_station_leg(pose,goal)
            if leg and all(clear(*grid.cell_xy(p['x'],p['y']),leg['clearance']) for p in leg['poses']):
                all_clear+=1
                if abs(j-i)==1:
                    adjacent.append((leg,goal))

    turns_deg=[abs(math.degrees(angle_error(leg['poses'][-1]['yaw'],goal['yaw']))) for leg,goal in adjacent]
    # Paired stress cases have identical motor profile, approach and disturbances
    # for each offset. Offset is assumed, not derived from wheel odometry.
    results=[]
    for offset,use_margin in ((r,m) for r in (0.,.015,.030,.060) for m in (False,True)):
        source=random.Random(seed)
        counts=Counter()
        shifts=[]
        for leg,goal in adjacent:
            for profile in held:
                for _ in range(4):
                    approach=leg['poses'][-1]['yaw']
                    heading=approach+math.radians(source.uniform(-4.,4.))
                    limits=(.012,.023) if use_margin and abs(angle_error(approach,goal['yaw']))>=FAS_TURN_MARGIN_ANGLE else (.020,.035)
                    along=source.uniform(*limits)*(-1 if leg['reverse'] else 1)
                    cross=source.uniform(-.006,.006)
                    x=-along*math.cos(approach)-cross*math.sin(approach)
                    y=-along*math.sin(approach)+cross*math.cos(approach)
                    axis=source.uniform(-math.pi,math.pi)+heading
                    wanted=angle_error(goal['yaw'],heading)
                    noise_seed=source.randrange(1<<30)
                    if abs(wanted)<=FAS_STATION_YAW_TOLERANCE:
                        counts['no_turn_needed']+=1
                        continue
                    counts['turns']+=1
                    error,_,moved,rest=simulate(profile,math.degrees(wanted),'profiled',R550_TURN,random.Random(noise_seed))
                    actual=wanted+math.copysign(1.,wanted)*math.radians(error)
                    dx=offset*(math.cos(axis)-math.cos(axis+actual))
                    dy=offset*(math.sin(axis)-math.sin(axis+actual))
                    shifts.append(math.hypot(dx,dy))
                    verdict=refinement_stage(math.hypot(x+dx,y+dy),math.radians(error),
                        position_tolerance=FAS_STATION_POSITION_TOLERANCE,yaw_tolerance=FAS_STATION_YAW_TOLERANCE)
                    counts[verdict if moved and rest else 'incomplete']+=1
        results.append(dict(offset_m=offset,with_approach_margin=use_margin,counts=dict(counts),
                            p95_centre_shift_m=round(quantile(shifts,.95),4) if shifts else None))
    return dict(map_id=grid.map_id,points=len(points),ordered_pairs=len(points)*(len(points)-1),
                clear_direct=all_clear,adjacent_clear=len(adjacent),adjacent_forward=sum(not leg['reverse'] for leg,_ in adjacent),
                adjacent_reverse=sum(leg['reverse'] for leg,_ in adjacent),seed=seed,
                station_headings_deg=sorted({round(math.degrees(p['yaw']),4) for p in points}),
                adjacent_nominal_turn_deg=dict(up_to_3=sum(a<=3.+1e-8 for a in turns_deg),
                    from_3_to_45=sum(3.+1e-8<a<45. for a in turns_deg),at_least_45=sum(a>=45. for a in turns_deg)),
                approach_assumptions=dict(along_m=[.020,.035],large_turn_margin_along_m=[.012,.023],
                                          cross_m=[-.006,.006],heading_deg=[-4,4]),
                sensitivity=results)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',type=Path,required=True)
    parser.add_argument('--route',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seed',type=int,default=550)
    parser.add_argument('traces',nargs='+',type=Path)
    args=parser.parse_args()
    report=analyze(args.scene,args.route,args.traces,args.seed)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))
