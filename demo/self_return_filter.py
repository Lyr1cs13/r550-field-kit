#!/usr/bin/env python3
"""Preserve raw /scan_raw and mask only the confirmed onboard antenna volume."""
import math,time,json
from pathlib import Path
import yaml
import self_return_geometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer,TransformListener
from self_return_geometry import antenna_return
class Filter(Node):
    def __init__(self):
        config=yaml.safe_load((Path(__file__).resolve().parent.parent/'config/vehicle.yaml').read_text())['antenna_filter']
        self_return_geometry.CENTER=tuple(config['center']);self_return_geometry.RADIUS=float(config['radius'])
        super().__init__('lab_antenna_self_filter');self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.out=self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data);self.last_log=0.
        self.create_subscription(LaserScan,'/scan_raw',self.scan,qos_profile_sensor_data)
    def scan(self,m):
        try:t=self.tf.lookup_transform('base_footprint',m.header.frame_id,Time()).transform
        except Exception:return  # Missing TF produces stale-scan stop, never a guessed mask.
        q=t.rotation;a=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));count=0
        ranges=list(m.ranges)
        for i,r in enumerate(ranges):
            if m.range_min<=r<=m.range_max and antenna_return(r,m.angle_min+i*m.angle_increment,t.translation.x,t.translation.y,a):
                ranges[i]=float('nan');count+=1
        m.ranges=ranges;self.out.publish(m)
        if time.monotonic()-self.last_log>10:
            self.get_logger().info(f'Onboard antenna returns masked: {count}; raw scan retained');self.last_log=time.monotonic()
def main():
    rclpy.init();n=Filter()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
