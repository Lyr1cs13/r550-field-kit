"""Read-only scan alignment and timestamp audit; never publishes robot commands.

A good score is only local map consistency, not proof of physical accuracy in
repeating aisles. Unknown map endpoints are reported separately.
"""
import json
import math
from collections import deque
from pathlib import Path
import time

import numpy as np
from scipy.ndimage import distance_transform_edt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

rclpy.init();n=Node('read_only_scan_alignment_audit');buf=Buffer();listener=TransformListener(buf,n)
state={'map':None};scans=deque(maxlen=40);out=Path(__file__).resolve().parent.parent/'state/diagnostics'
out.mkdir(parents=True,exist_ok=True)

def mapped(m):
    g=np.asarray(m.data).reshape(m.info.height,m.info.width)
    state['map']=(m,g,distance_transform_edt(g<65)*m.info.resolution)
def scanned(m):scans.append((m,time.time()))
q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(OccupancyGrid,'/map',mapped,q)
n.create_subscription(LaserScan,'/scan',scanned,qos_profile_sensor_data)
last=0;start=time.monotonic()
while time.monotonic()-start<7200:
    rclpy.spin_once(n,timeout_sec=.03)
    if time.monotonic()-last<1 or state['map'] is None or not scans:continue
    ready=[v for v in scans if time.time()-v[1]>=.15]
    if not ready:continue
    last=time.monotonic();s,arrival=ready[-1];m,g,dist=state['map']
    stamp=s.header.stamp.sec+s.header.stamp.nanosec*1e-9
    row={'time':time.time(),'scan_delay_s':arrival-stamp,'scan_age_s':time.time()-stamp}
    try:
        t=buf.lookup_transform('map',s.header.frame_id,Time.from_msg(s.header.stamp))
        q=t.transform.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        ranges=np.asarray(s.ranges)[::3];angles=s.angle_min+np.arange(0,len(s.ranges),3)*s.angle_increment+yaw
        use=np.isfinite(ranges)&(ranges>=max(.3,s.range_min))&(ranges<=min(6.,s.range_max))
        x=t.transform.translation.x+ranges[use]*np.cos(angles[use])
        y=t.transform.translation.y+ranges[use]*np.sin(angles[use])
        i=np.floor((x-m.info.origin.position.x)/m.info.resolution).astype(int)
        j=np.floor((y-m.info.origin.position.y)/m.info.resolution).astype(int)
        valid=(i>=0)&(i<m.info.width)&(j>=0)&(j<m.info.height);i=i[valid];j=j[valid]
        known=g[j,i]>=0;d=dist[j[known],i[known]]
        row.update(rays=int(use.sum()),known_endpoints=int(known.sum()),
                   known_fraction=float(known.mean()) if len(known) else None,
                   inliers_15cm=float((d<=.15).mean()) if len(d) else None,
                   median_distance_m=float(np.median(d)) if len(d) else None,
                   p90_distance_m=float(np.quantile(d,.9)) if len(d) else None)
    except Exception as e:row['tf_error']=str(e)[:220]
    tmp=out/'alignment_latest.tmp';tmp.write_text(json.dumps(row));tmp.replace(out/'alignment_latest.json')
    with (out/'alignment.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
    print(json.dumps(row),flush=True)
n.destroy_node();rclpy.shutdown()
