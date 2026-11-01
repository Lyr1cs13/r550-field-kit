#!/usr/bin/env python3
"""Save/reload support around native SLAM services; never edits sensor data."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from slam_toolbox.srv import Pause, ClearQueue, SerializePoseGraph
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
STATE = ROOT / 'state/mapping'


def call(node, kind, name, request, timeout=10):
    client = node.create_client(kind, name)
    if not client.wait_for_service(timeout_sec=3):
        raise RuntimeError(f'{name}: service unavailable')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise RuntimeError(f'{name}: timeout/no response')
    return future.result()


def pause_mapping(node):
    req = GetParameters.Request(names=['paused_new_measurements'])
    value = call(node, GetParameters, '/slam_toolbox/get_parameters', req)
    if not value.values[0].bool_value:
        call(node, Pause, '/slam_toolbox/pause_new_measurements', Pause.Request())
    call(node, ClearQueue, '/slam_toolbox/clear_queue', ClearQueue.Request())


def save(node, output):
    # A stationary checkpoint and unchanged docking anchor are separate facts.
    # Never assume that any stationary robot is physically at the charger.
    pause = node.create_client(Trigger, '/rrt_pause')
    if pause.wait_for_service(timeout_sec=1):
        f = pause.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, f, timeout_sec=4)
        if not f.done() or f.result() is None or not f.result().success:
            raise RuntimeError('Could not pause task scheduler')
    speeds, charging, voltage, poses = [], [], [], []
    tf = Buffer()
    listener = TransformListener(tf, node)
    node.create_subscription(Odometry, '/odom', lambda m: speeds.append(
        [m.twist.twist.linear.x, m.twist.twist.angular.z]), qos_profile_sensor_data)
    node.create_subscription(Bool, '/robot_charging_flag',
                             lambda m: charging.append(m.data), qos_profile_sensor_data)
    node.create_subscription(Float32, '/PowerVoltage',
                             lambda m: voltage.append(m.data), qos_profile_sensor_data)
    node.create_subscription(PoseWithCovarianceStamped, '/pose',
                             lambda m: poses.append(m), 10)
    until = time.monotonic() + 2
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.1)
    if not speeds or any(abs(v) > .02 for pair in speeds[-10:] for v in pair):
        raise RuntimeError('No stationary wheel feedback; stop before saving')
    pause_mapping(node)
    # Check data exists: DeserializePoseGraph has an empty response in this build.
    grid = call(node, GetMap, '/slam_toolbox/dynamic_map', GetMap.Request()).map
    if not grid.info.width or not grid.info.height:
        raise RuntimeError('Native SLAM has not produced a map')
    output.mkdir(parents=True, exist_ok=False)
    req = SerializePoseGraph.Request(filename=str(output / 'slam_graph'))
    result = call(node, SerializePoseGraph, '/slam_toolbox/serialize_map', req, 25)
    if result.result != 0:
        raise RuntimeError(f'Native graph save failed: {result.result}')
    command = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
               '-f', str(output / 'WHEELTEC'), '--ros-args',
               '-p', 'save_map_timeout:=10.0',
               '-p', 'free_thresh_default:=0.196',
               '-p', 'occupied_thresh_default:=0.65',
               '-p', 'map_subscribe_transient_local:=true']
    saved = subprocess.run(command, capture_output=True, text=True, timeout=25)
    (output / 'map_save.log').write_text(saved.stdout + saved.stderr)
    if saved.returncode:
        raise RuntimeError('Native map saver failed; see map_save.log')
    from ament_index_python.packages import get_package_share_directory
    vendor = Path(get_package_share_directory('wheeltec_slam_toolbox'))
    shutil.copy2(vendor / 'config/mapper_params_online_sync.yaml',
                 output / 'native_slam_params.yaml')
    for name in ('dock_start.json', 'boundary.json', 'threshold_avoid.json', 'mapping_region.json',
                 'dock_restore_attempt.json', 'native_restore_attempt.json'):
        if (STATE / name).exists():
            shutil.copy2(STATE / name, output / name)
    shutil.copy2(HERE / 'navigation.yaml', output / 'navigation.yaml')
    dock = json.loads((output / 'dock_start.json').read_text())
    anchor = dock['robot_pose']
    current = None
    try:
        transform = tf.lookup_transform('map', 'base_footprint', Time()).transform
        q = transform.rotation
        current = [transform.translation.x, transform.translation.y,
                   math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
    except Exception:
        pass
    # Keepout is a SEPARATE navigation mask; the SLAM occupancy grid is untouched.
    avoid = json.loads((output / 'threshold_avoid.json').read_text()) if (output / 'threshold_avoid.json').exists() else {}
    w, h, res = grid.info.width, grid.info.height, grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    mask = bytearray([254]) * (w*h)
    rectangles = ([avoid['keepout_rectangle']] if 'keepout_rectangle' in avoid else []) + avoid.get('additional_keepout_rectangles', [])
    blocked = 0
    for row in range(h):
        y = oy + (h-row-.5)*res
        for col in range(w):
            x = ox + (col+.5)*res
            if any(a <= x <= b and c <= y <= d for a,b,c,d in rectangles):
                mask[row*w+col] = 0
                blocked += 1
    (output / 'threshold_keepout.pgm').write_bytes(
        f'P5\n{w} {h}\n255\n'.encode() + mask)
    (output / 'threshold_keepout.yaml').write_text(
        f'image: threshold_keepout.pgm\nmode: trinary\nresolution: {res}\n'
        f'origin: [{ox}, {oy}, 0.0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    names = ['WHEELTEC.pgm', 'WHEELTEC.yaml', 'slam_graph.posegraph',
             'slam_graph.data', 'native_slam_params.yaml', 'dock_start.json',
             'threshold_keepout.pgm', 'threshold_keepout.yaml', 'navigation.yaml']
    hashes = {}
    for name in names:
        p = output / name
        if not p.is_file() or p.stat().st_size == 0:
            raise RuntimeError(f'Missing saved file: {p}')
        hashes[name] = hashlib.sha256(p.read_bytes()).hexdigest()
    manifest = {
        'created': time.strftime('%Y-%m-%d %H:%M:%S %z'),
        'directory': str(output),
        'graph_prefix': 'slam_graph',
        'startup_pose': [anchor['x'], anchor['y'], anchor['yaw']],
        'shutdown_pose': current,
        'charging_feedback': charging[-1] if charging else None,
        'voltage': voltage[-1] if voltage else None,
        'width': w, 'height': h, 'resolution': res,
        'keepout_cells': blocked, 'sha256': hashes,
        'continuation': 'native synchronous SLAM Toolbox, START_AT_GIVEN_POSE',
        'requires': 'same physical dock and heading; verify scan/map alignment before motion',
        'mapping_paused': True,
    }
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    (ROOT/'maps').mkdir(exist_ok=True)
    (ROOT / 'maps/latest_checkpoint.json').write_text(
        json.dumps({'directory': str(output)}, indent=2))
    print(json.dumps(manifest, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=['pause', 'save'])
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('mapping_checkpoint')
    try:
        if args.command == 'pause':
            pause_mapping(node)
            print('Native mapping paused; no movement requested.')
        else:
            save(node, args.output or ROOT / 'maps' / time.strftime('checkpoint_%Y%m%d_%H%M%S'))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
