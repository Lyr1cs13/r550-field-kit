#!/usr/bin/env python3
"""Read-only installation and ROS package checks."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--offline',action='store_true');a=p.parse_args();errors=[]
    for package in ('yaml','serial','numpy','PIL'):
        ok=importlib.util.find_spec(package) is not None;print(('OK  ' if ok else '缺少 ')+package)
        if not ok:errors.append(package)
    if not a.offline:
        for package in ('turn_on_wheeltec_robot','wheeltec_nav2','wheeltec_slam_toolbox','wheeltec_robot_rrt','wheeltec_rrt_msg','slam_toolbox','nav2_controller','nav2_rotation_shim_controller'):
            try:ok=subprocess.run(['ros2','pkg','prefix',package],capture_output=True,timeout=6).returncode==0
            except FileNotFoundError:ok=False
            print(('OK  ' if ok else '缺少 ')+package)
            if not ok:errors.append(package)
    active=ROOT/'state/active_scene.json'
    print('场景：'+(json.loads(active.read_text())['scene'] if active.exists() else '未激活；首次建图后导入'))
    if errors:raise SystemExit('需要补齐：'+', '.join(errors))


if __name__=='__main__':main()
