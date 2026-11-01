#!/usr/bin/env python3
"""Cancel native navigation, disable IR drive, stop wheels and pause SLAM writes."""
import time
import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from rcl_interfaces.srv import GetParameters
from slam_toolbox.srv import Pause,ClearQueue
from std_srvs.srv import Trigger


def main():
    rclpy.init();n=rclpy.create_node('kit_hold');zero=n.create_publisher(Twist,'/cmd_vel',10);ir=n.create_publisher(Int8,'/robot_recharge_flag',10)
    def stop_output():ir.publish(Int8(data=0));zero.publish(Twist())
    n.create_timer(.05,stop_output);stop_output()
    def call(kind,name,request):
        c=n.create_client(kind,name)
        if not c.wait_for_service(timeout_sec=.3):return None
        f=c.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=2)
        return f.result() if f.done() else None
    try:
        call(Trigger,'/rrt_pause',Trigger.Request())
        for action in ('navigate_to_pose','follow_path','spin'):call(CancelGoal,f'/{action}/_action/cancel_goal',CancelGoal.Request())
        result=call(GetParameters,'/slam_toolbox/get_parameters',GetParameters.Request(names=['paused_new_measurements']))
        if result and result.values and not result.values[0].bool_value:call(Pause,'/slam_toolbox/pause_new_measurements',Pause.Request())
        if result:call(ClearQueue,'/slam_toolbox/clear_queue',ClearQueue.Request())
        end=time.monotonic()+1.
        while time.monotonic()<end:ir.publish(Int8(data=0));zero.publish(Twist());rclpy.spin_once(n,timeout_sec=.05)
        print('已取消运动并暂停地图写入。')
    finally:n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
