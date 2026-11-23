#!/usr/bin/env python3
"""Preserve /scan_raw and mask only measured onboard reflections."""
import math,time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer,TransformListener
from self_return_geometry import antenna_echo_indices, rear_center_glint_indices
class Filter(Node):
    def __init__(self):
        config=yaml.safe_load((Path(__file__).resolve().parent.parent/'config/vehicle.yaml').read_text())['antenna_filter']
        self.center=tuple(config['center']);self.radius=float(config['radius'])
        self.secondary_echo=config.get('retreat_echo')
        self.rear_center_glint=config.get('rear_center_glint') is True
        super().__init__('lab_antenna_self_filter');self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.out=self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data);self.last_log=0.
        current=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan,'/scan_raw',self.scan,current)
    def scan(self,m):
        try:t=self.tf.lookup_transform('base_footprint',m.header.frame_id,Time()).transform
        except Exception:return  # Missing TF produces stale-scan stop, never a guessed mask.
        q=t.rotation;a=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        ranges=list(m.ranges)
        geometry=(a+m.angle_min,m.angle_increment,(t.translation.x,t.translation.y),(m.range_min,m.range_max))
        masked=antenna_echo_indices(ranges,*geometry,self.center,self.radius,self.secondary_echo)
        if self.rear_center_glint:
            masked.update(rear_center_glint_indices(ranges,*geometry))
        for index in masked:
            ranges[index]=float('nan')
        m.ranges=ranges;self.out.publish(m)
        if time.monotonic()-self.last_log>10:
            self.get_logger().info(f'Onboard returns masked: {len(masked)}; raw scan retained');self.last_log=time.monotonic()
def main():
    rclpy.init();n=Filter()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
