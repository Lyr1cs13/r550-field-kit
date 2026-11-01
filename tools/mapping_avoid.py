#!/usr/bin/env python3
"""Separate world-coordinate navigation keepouts for a live growing SLAM map."""
import argparse
from pathlib import Path
import math
import json
import rclpy
from nav2_msgs.srv import LoadMap

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--rect',nargs=4,type=float,action='append',default=[]);a=p.parse_args();rects=a.rect
    if any(not all(math.isfinite(v) for v in r) or r[0]>=r[1] or r[2]>=r[3] for r in rects):raise ValueError('矩形使用 xmin xmax ymin ymax')
    state=ROOT/'state/mapping';state.mkdir(parents=True,exist_ok=True)
    res=.05
    xmin=min([r[0] for r in rects],default=-1.)-1.;xmax=max([r[1] for r in rects],default=1.)+1.
    ymin=min([r[2] for r in rects],default=-1.)-1.;ymax=max([r[3] for r in rects],default=1.)+1.
    w,h=math.ceil((xmax-xmin)/res),math.ceil((ymax-ymin)/res)
    if w*h>16000000:raise ValueError('范围过大，请检查坐标单位为米')
    pixels=bytearray([254])*(w*h)
    for row in range(h):
        y=ymin+(h-row-.5)*res
        for col in range(w):
            x=xmin+(col+.5)*res
            if any(a<=x<=b and c<=y<=d for a,b,c,d in rects):pixels[row*w+col]=0
    (state/'threshold_keepout.pgm').write_bytes(f'P5\n{w} {h}\n255\n'.encode()+pixels)
    path=state/'threshold_keepout.yaml';path.write_text(f'image: threshold_keepout.pgm\nresolution: {res}\norigin: [{xmin}, {ymin}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    (state/'threshold_avoid.json').write_text(json.dumps({'additional_keepout_rectangles':rects}))
    rclpy.init();n=rclpy.create_node('kit_mapping_keepout_reload');c=n.create_client(LoadMap,'/threshold_mask_server/load_map')
    try:
        if c.wait_for_service(timeout_sec=2):
            f=c.call_async(LoadMap.Request(map_url=str(path)));rclpy.spin_until_future_complete(n,f,timeout_sec=5)
            if not f.done() or f.result() is None or f.result().result!=0:raise RuntimeError('禁区已写入，但在线加载未成功，请暂停后重启自动探索')
        print('禁区已保存，SLAM 地图未修改。')
    finally:n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
