#!/usr/bin/env python3
"""Idempotent SLAM write pause/resume and automatic exploration commands."""
import argparse
import rclpy
from rcl_interfaces.srv import GetParameters
from slam_toolbox.srv import Pause,ClearQueue
from std_srvs.srv import Trigger


def call(n,kind,name,request):
    c=n.create_client(kind,name)
    if not c.wait_for_service(timeout_sec=2):raise RuntimeError(name+' 不可用')
    f=c.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=5)
    if not f.done() or f.result() is None:raise RuntimeError(name+' 超时')
    return f.result()


def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['pause','resume','return','write']);a=p.parse_args()
    rclpy.init();n=rclpy.create_node('kit_exploration_control')
    try:
        if a.action=='pause':call(n,Trigger,'/rrt_pause',Trigger.Request())
        current=call(n,GetParameters,'/slam_toolbox/get_parameters',GetParameters.Request(names=['paused_new_measurements'])).values[0].bool_value
        desired=a.action=='pause'
        if current!=desired:call(n,Pause,'/slam_toolbox/pause_new_measurements',Pause.Request())
        if desired:call(n,ClearQueue,'/slam_toolbox/clear_queue',ClearQueue.Request())
        elif a.action!='write':call(n,Trigger,'/rrt_return_to_dock' if a.action=='return' else '/rrt_resume',Trigger.Request())
        print({'pause':'建图已暂停','resume':'探索已继续','return':'已请求返回桩前','write':'地图写入已恢复'}[a.action])
    finally:n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
