#!/usr/bin/env python3
"""Isolated native Nav2 checks using a synthetic 2D differential-drive plant.

No serial device, robot host, or vendor hardware launch is used. This exercises
real controller/collision/smoother nodes, NOT real traction or AMCL accuracy.
Run on ROS2 Humble with the repository overlay sourced.
"""
import json
import argparse
from collections import deque
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import signal
import threading
import time
from types import MethodType, SimpleNamespace
import yaml

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repeat',type=int,default=0,help='additional alternating forward/reverse arrivals')
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--safety-only',action='store_true')
    mode.add_argument('--corridor-only',action='store_true',help='check the guided corridor and dock handoff')
    args=parser.parse_args()
    if not 0<=args.repeat<=100:raise SystemExit('--repeat must be 0..100')
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID')!='187':
        raise SystemExit('This check requires ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=187')
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import Twist, TwistStamped, TransformStamped, PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
    from nav2_msgs.action import DriveOnHeading, FollowPath, ComputePathToPose, Spin
    from nav2_msgs.srv import IsPathValid, ManageLifecycleNodes
    from rcl_interfaces.srv import SetParametersAtomically
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Bool, Int8, UInt8
    from tf2_ros import TransformBroadcaster
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    sys.path.insert(0,str(ROOT/'demo'))
    from directed_route import fas_axis_leg, straight_approach_speed
    from arrival_policy import FAS_STATION_POSITION_TOLERANCE, FAS_STATION_YAW_TOLERANCE, SMALL_SPIN_ANGLE
    from ros_backend import RosBackend
    from turn_approach import TurnApproach

    out=ROOT/'state/robustness/native';out.mkdir(parents=True,exist_ok=True)
    (out/'result.json').unlink(missing_ok=True)
    config=yaml.safe_load((ROOT/'demo/navigation.yaml').read_text())
    controller=config['controller_server']['ros__parameters']
    arrival=config['demo_arrival']['ros__parameters']
    position_tolerance=arrival['final_position_tolerance']
    yaw_tolerance=arrival.get('final_yaw_tolerance',controller['goal_checker']['yaw_goal_tolerance'])
    if args.corridor_only:
        controller['controller_plugins']=['FollowPath','PointPosition','FixedHeading','CorridorTrack','CorridorBend']
    else:
        controller['controller_plugins']=(['FixedHeading'] if args.safety_only else ['FixedHeading','PointPose'])
        controller['goal_checker_plugins']=(['point_position_checker'] if args.safety_only else
                                             ['point_position_checker','point_pose_checker'])
        controller['goal_checker_plugins'].extend(['directed_position_checker','fas_turn_position_checker'])
        controller['goal_checker_plugin']='point_position_checker'
        controller['point_position_checker']['xy_goal_tolerance']=controller['directed_position_checker']['xy_goal_tolerance']
    local=config['local_costmap']['local_costmap']['ros__parameters']
    local.pop('filters',None)
    local['voxel_layer']['observation_sources']='scan'
    local['voxel_layer']['scan']['topic']='/scan'
    global_map=config['global_costmap']['global_costmap']['ros__parameters']
    global_map.pop('filters',None)
    global_map['obstacle_layer']['observation_sources']='scan'
    params=out/'parameters.yaml';params.write_text(yaml.safe_dump(config,sort_keys=False))
    children=[];logs=[]
    def spawn(name,cmd):
        log=(out/(name+'.log')).open('w');logs.append(log)
        p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(p)
        return p
    spawn('controller',['ros2','run','nav2_controller','controller_server','--ros-args','--params-file',str(params),'-r','cmd_vel:=/nav_cmd_vel'])
    spawn('smoother',['ros2','run','nav2_velocity_smoother','velocity_smoother','--ros-args','--params-file',str(params),'-r','cmd_vel:=/nav_cmd_vel','-r','cmd_vel_smoothed:=/collision_cmd_vel'])
    spawn('collision',['ros2','run','nav2_collision_monitor','collision_monitor','--ros-args','--params-file',str(params)])
    spawn('gate',[sys.executable,str(ROOT/'demo/motion_gate.py')])
    spawn('planner',['ros2','run','nav2_planner','planner_server','--ros-args','--params-file',str(params)])
    spawn('behavior',['ros2','run','nav2_behaviors','behavior_server','--ros-args','--params-file',str(params),'-r','cmd_vel:=/nav_cmd_vel'])
    managed=['planner_server','controller_server','velocity_smoother','collision_monitor','behavior_server']
    if args.corridor_only:
        spawn('bt_navigator',['ros2','run','nav2_bt_navigator','bt_navigator','--ros-args','--params-file',str(params)])
        managed.append('bt_navigator')
    spawn('lifecycle',['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args','-p','autostart:=true',
                       '-p','node_names:=['+', '.join(managed)+']'])
    rclpy.init();node=rclpy.create_node('synthetic_navigation_plant')
    tf=TransformBroadcaster(node)
    odom=node.create_publisher(Odometry,'/odom',10)
    combined=node.create_publisher(Odometry,'/odom_combined',10)
    scan=node.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
    imu=node.create_publisher(Imu,'/imu/data_raw',qos_profile_sensor_data)
    permit=node.create_publisher(Bool,'/navigation_motion_permit',1)
    raw=node.create_publisher(Twist,'/collision_cmd_vel',1)
    oem=node.create_publisher(Bool,'/navigation_oem_active',1)
    ir_commands=[]
    node.create_subscription(Int8,'/robot_recharge_flag',lambda message:ir_commands.append(message.data),10)
    map_publisher=node.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    world=OccupancyGrid();world.header.frame_id='map';world.header.stamp=node.get_clock().now().to_msg()
    world.info.resolution=.05;world.info.width=100;world.info.height=80
    world.info.origin.position.x=-2.;world.info.origin.position.y=-2.;world.info.origin.orientation.w=1.
    world.data=[100 if x in (0,99) or y in (0,79) else 0 for y in range(80) for x in range(100)]
    fixture=None;fixture_grid=None;corridor_bridge=None
    if args.corridor_only:
        import numpy as np
        from map_grid import MapGrid
        fixture=tempfile.TemporaryDirectory()
        scene=Path(fixture.name)/'demo-room'
        shutil.copytree(ROOT/'scenes/demo-room',scene)
        fixture_grid=MapGrid(scene)
        lane_x=.5
        guide={'schema':1,'map_id':fixture_grid.map_id,'lanes':[[lane_x,-6.,-3.],[lane_x+1.,-6.,-3.]],
               'links':[-6.,-3.],'clearance_m':.27}
        (scene/'corridor_route.json').write_text(json.dumps(guide))
        world.info.resolution=fixture_grid.resolution
        world.info.width=fixture_grid.width;world.info.height=fixture_grid.height
        world.info.origin.position.x,world.info.origin.position.y=fixture_grid.origin[:2]
        world.data=fixture_grid.cells
        cells=np.asarray(fixture_grid.cells).reshape(fixture_grid.height,fixture_grid.width)
        rays=np.arange(.05,10.01,.045)[None,:]
        ray_angles=np.linspace(-math.pi,math.pi,360,endpoint=False)[:,None]
        amcl=node.create_publisher(PoseWithCovarianceStamped,'/amcl_pose',
                                   QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        ir=node.create_publisher(UInt8,'/robot_red_flag',qos_profile_sensor_data)
    map_publisher.publish(world)
    state=dict(x=0.,y=.012,a=.01,v=0.,w=0.,command=(0.,0.),command_time=0.,obstacle=None,
               sensors=True,allow=True,manual=False,freeze=False,scan_at=0.)
    samples=[];results=[];started=time.monotonic()
    bridge=None
    def bridge_sensor(method, message):
        if bridge is not None:
            getattr(bridge,method)(message)
    node.create_subscription(Imu,'/imu/data_raw',lambda m:bridge_sensor('gyro_received',m),qos_profile_sensor_data)
    def received(m):
        state['command']=(m.linear.x,m.angular.z);state['command_time']=time.monotonic()
    node.create_subscription(Twist,'/cmd_vel',received,1)
    previous=time.monotonic()
    def step():
        nonlocal previous
        rclpy.spin_once(node,timeout_sec=.003)
        now=time.monotonic();dt=min(.04,now-previous);previous=now
        v,w=state['command'] if now-state['command_time']<.35 else (0.,0.)
        # Bounded plant acceleration; independent truth never snaps to the goal.
        state['v']+=max(-.8*dt,min(.4*dt,v-state['v']))
        state['w']+=max(-.8*dt,min(.8*dt,w-state['w']))
        if not state['freeze']:
            state['x']+=state['v']*math.cos(state['a'])*dt
            state['y']+=state['v']*math.sin(state['a'])*dt
            # Offset instantaneous centre, expressed in metres in the body frame.
            # Exact centre displacement for this angular increment, independent of odometry.
            offset=state.get('turn_offset',0.)
            state['x']+=offset*(math.cos(state['a'])-math.cos(state['a']+state['w']*dt))
            state['y']+=offset*(math.sin(state['a'])-math.sin(state['a']+state['w']*dt))
            state['a']+=state['w']*dt
        stamp=node.get_clock().now().to_msg()
        transforms=[]
        for parent,child,x,y,a in [('map','odom_combined',0.,0.,0.),('odom_combined','base_footprint',state['x'],state['y'],state['a']),('base_footprint','laser',0.,0.,0.)]:
            t=TransformStamped();t.header.stamp=stamp;t.header.frame_id=parent;t.child_frame_id=child
            t.transform.translation.x=float(x);t.transform.translation.y=float(y)
            t.transform.rotation.z=math.sin(a/2);t.transform.rotation.w=math.cos(a/2);transforms.append(t)
        tf.sendTransform(transforms)
        m=Odometry();m.header.stamp=stamp;m.header.frame_id='odom_combined';m.child_frame_id='base_footprint'
        m.pose.pose.position.x=state['x'];m.pose.pose.position.y=state['y'];m.pose.pose.orientation=transforms[1].transform.rotation
        m.twist.twist.linear.x=state['v'];m.twist.twist.angular.z=state['w'];odom.publish(m);combined.publish(m)
        if bridge is not None:
            bridge.wheel_received(m)
        if corridor_bridge is not None:
            sample=PoseWithCovarianceStamped()
            sample.header.stamp=stamp;sample.header.frame_id='map'
            sample.pose.pose=m.pose.pose
            for index in (0,7,35):sample.pose.covariance[index]=.001
            amcl.publish(sample)
            ir.publish(UInt8(data=state.get('ir',0)))
        if state['allow'] is not None:permit.publish(Bool(data=state['allow']))
        if state['sensors'] and now-state['scan_at']>.05:
            state['scan_at']=now
            if bridge is not None or corridor_bridge is not None:
                rate=Imu();rate.header.stamp=stamp;rate.angular_velocity.z=state['w']+.004
                imu.publish(rate)
            s=LaserScan();s.header.stamp=stamp;s.header.frame_id='laser'
            s.angle_min=-math.pi;s.angle_increment=math.pi/180;s.angle_max=math.pi-s.angle_increment
            s.range_min=.02;s.range_max=10.;s.scan_time=.05
            ranges=[]
            if fixture_grid is not None:
                radians=ray_angles+state['a']
                ix=np.floor((state['x']+np.cos(radians)*rays-fixture_grid.origin[0])/fixture_grid.resolution).astype(int)
                iy=np.floor((state['y']+np.sin(radians)*rays-fixture_grid.origin[1])/fixture_grid.resolution).astype(int)
                inside=(0<=ix)&(ix<fixture_grid.width)&(0<=iy)&(iy<fixture_grid.height)
                hit=~inside|(cells[iy.clip(0,fixture_grid.height-1),ix.clip(0,fixture_grid.width-1)]!=0)
                ray_ranges=np.where(hit.any(axis=1),rays[0,hit.argmax(axis=1)],s.range_max)
            for i in range(360):
                a=state['a']+s.angle_min+i*s.angle_increment;cx,cy=math.cos(a),math.sin(a)
                if fixture_grid is None:
                    distances=[(wall-state['x'])/cx for wall in (-2.,3.) if abs(cx)>1e-9 and (wall-state['x'])/cx>0]
                    distances += [(wall-state['y'])/cy for wall in (-2.,2.) if abs(cy)>1e-9 and (wall-state['y'])/cy>0]
                    d=min(distances)
                else:
                    d=ray_ranges[i]
                if state['obstacle'] is not None:
                    ox,oy=state['obstacle'];rx,ry=ox-state['x'],oy-state['y']
                    projection=rx*cx+ry*cy;cross=rx*cy-ry*cx
                    if projection>0 and abs(cross)<.07:
                        d=min(d,projection-math.sqrt(.07**2-cross**2))
                ranges.append(float(max(.021,d)))
            s.ranges=[math.nan]*360 if state['sensors']=='invalid' else ranges
            if state['sensors']=='stale':s.header.stamp.sec-=10
            scan.publish(s)
        if bridge is not None:
            bridge.pose=[state['x'],state['y'],state['a']]
            bridge.pose_stamp=time.time();bridge.pose_observed_at=now;bridge.last_scan=state['scan_at']
            if now-bridge.last_lease>=.05:
                bridge.last_lease=now
                bridge.publish_motion_permit()
            op=bridge.ops[bridge.active]
            if op['status'] in ('position_settling','heading_settling'):
                bridge.finish_waypoint_position(op,now)
            elif op['status'] in ('line_alignment','line_settling'):
                bridge.update_line_alignment(op,now)
            elif op['status']=='recovery_wait' and now>=op['retry_at']:
                bridge.send_arrival_retry(op)
            elif op['status']=='navigating' and op.get('handle'):
                distance=math.hypot(state['x']-op['goal']['x'],state['y']-op['goal']['y'])
                if not bridge.stop_at_waypoint(op,distance,now):
                    bridge.request_speed(straight_approach_speed(distance,reverse=op['reverse']),'FixedHeading')
        samples.append([now-started,state['x'],state['y'],state['a'],v,w,state['v'],state['w']])
        time.sleep(.012)
    def run_for(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:step()
    def wait(future,timeout=15.):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:step()
        if not future.done():raise TimeoutError('native action timeout')
        return future.result()
    client=ActionClient(node,FollowPath,'/follow_path')
    drive=ActionClient(node,DriveOnHeading,'/drive_on_heading')
    spin=ActionClient(node,Spin,'/spin')
    leg_validator=node.create_client(IsPathValid,'/is_path_valid')
    spin_limit=node.create_publisher(TwistStamped,'/navigation_spin_limit',1)
    nav_zero=node.create_publisher(Twist,'/nav_cmd_vel',1)
    def station_bridge(goal, blocked_until, *, fas_station=True, turn_approach=None):
        sink=SimpleNamespace(publish=lambda msg:None)
        def check_wall(x,y,clearance):
            if min(x+2,3-x,y+2,2-y)<=clearance:raise ValueError('Synthetic wall clearance')
        b=SimpleNamespace(ops={},active=None,lock=threading.RLock(),spin_action=spin,
            get_clock=node.get_clock,spin_limit_pub=spin_limit,motion_permit=permit,retreat_permit=sink,oem_lease=sink,
            dock_calibrated=False,docking_since=None,fleet_zone=None,localization_guard=SimpleNamespace(reason=''),confirmed=True,
            gyro=None,rest_gyro=deque(maxlen=24),stationary_since=time.monotonic()-.8,last_wheel=0.,wheel_distance=0.,
            encoders=None,wheel_yaw_rate=0.,joint_terminal=True,arrival=arrival,goal_xy_tolerance=.10,
            goal_yaw_tolerance=.25,position_xy_tolerance=.035,turn_approach_radius=.022,
            turn_approach=turn_approach if turn_approach is not None else TurnApproach(),track=deque(),
            sequential_waypoints=True,precision_arrival=True,pose=[state[k] for k in ('x','y','a')],
            grid=SimpleNamespace(validate=check_wall),dock_localization=SimpleNamespace(arm=lambda:None),
            leg_validator=leg_validator,controller_parameters=parameters,
            directed_lookahead=controller['FixedHeading']['lookahead_dist'],speed_limits={},speed_update=None,follow=client,get_logger=node.get_logger,plan_coordinates=RosBackend.plan_coordinates,
            raw_cmd=nav_zero,cmd=raw,last_lease=0.,request_stationary_amcl=lambda now:None)
        b.snapshot=lambda:dict(charge_contact_live=False,ready=time.monotonic()>=blocked_until,stationary=b.stationary_since is not None,
                               scan_map_agreement=.99,scan_structure_verified=True)
        for name in ('gyro_received','wheel_received','begin_waypoint_heading','start_fas_spin','spin_accepted',
                     'spin_done','finish_native_cancellation','settle_waypoint_heading','should_brake_heading',
                     'navigate','send_fas_line','fas_line','stamped','navigation_path_allowed','send_navigation','directed_validated','directed_speed_ready',
                     'update_line_alignment','directed_lookahead_ready','send_directed_path',
                     'follow_directed','nav_accepted','nav_done','retry_arrival','send_arrival_retry','request_speed','stop_at_waypoint',
                     'publish_motion_permit','publish_spin_limit','zero_motion','finish_waypoint_position','complete_waypoint'):
            setattr(b,name,MethodType(getattr(RosBackend,name),b))
        b.navigate({**goal,'fas_station':fas_station},'waypoint')
        return b
    def path_goal(x,heading=0.,terminal=False):
        goal=FollowPath.Goal();goal.controller_id='PointPose' if terminal else 'FixedHeading'
        goal.goal_checker_id='point_pose_checker' if terminal else 'point_position_checker'
        p=NavPath();p.header.frame_id='map';p.header.stamp=node.get_clock().now().to_msg()
        count=1 if terminal else max(2,math.ceil(abs(x-state['x'])/.04))
        for i in range(count+1):
            f=i/count;q=PoseStamped();q.header=p.header
            q.pose.position.x=state['x']+(x-state['x'])*f;q.pose.position.y=state['y']*(1-f)
            q.pose.orientation.z=math.sin(heading/2);q.pose.orientation.w=math.cos(heading/2);p.poses.append(q)
        goal.path=p
        return goal
    corridor_executor=None;corridor_thread=None
    def check_corridors():
        nonlocal corridor_bridge,corridor_executor,corridor_thread
        from rclpy.executors import SingleThreadedExecutor
        from corridor_ros import CorridorBackend
        from directed_route import angle_error
        from fas_car import CorridorLease
        from mission import Mission

        heading=math.radians(-15)
        dock=dict(calibrated=True,map_id=fixture_grid.map_id,departure_distance=.25,
                  robot_pose=dict(x=lane_x-.45,y=-4.5,yaw=0.),
                  charger_approach=dict(p_x=lane_x,p_y=-4.5,orien_z=math.sin(heading/2),orien_w=math.cos(heading/2)))
        state.update(x=lane_x-.45,y=-4.5,a=0.,v=0.,w=0.,allow=None)
        corridor_bridge=CorridorBackend(fixture_grid,dock)
        corridor_bridge.fas_lease=CorridorLease()
        # The plant publishes the map, TF and pose samples. Controller, planner,
        # smoother and collision checks use their real lifecycle and services.
        for alias in ('amcl','map_server'):
            corridor_bridge.lifecycle_clients.pop(alias)
        corridor_executor=SingleThreadedExecutor();corridor_executor.add_node(corridor_bridge)
        corridor_thread=threading.Thread(target=corridor_executor.spin,daemon=True)
        corridor_thread.start()
        run_for(3.)
        corridor_bridge.confirm_localization()

        def move(name, goal, kind='waypoint', *, reverse=None, obstruct=False):
            begin=time.monotonic();first=len(samples);block=None;released=False
            if kind=='waypoint':
                goal={**goal,'fas_station':True}
            identity=corridor_bridge.navigate(goal,kind)
            while time.monotonic()-begin<75.:
                status=corridor_bridge.operation(identity)
                if status['status'] in ('succeeded','failed','canceled'):
                    break
                if obstruct and block is None and state['v']>.038:
                    state['obstacle']=(state['x']+.24*math.cos(state['a']),state['y']+.24*math.sin(state['a']))
                    block=time.monotonic()
                if block is not None and not released and time.monotonic()-block>2.2:
                    held=[row for row in samples[first:] if block+1.1<=started+row[0]<=block+2.]
                    if not held or any(abs(row[4])>.005 or abs(row[6])>.012 for row in held):
                        raise AssertionError('guided leg moved through a simulated obstruction')
                    state['obstacle']=None;released=True
                step()
            run_for(.55)
            error=math.dist((state['x'],state['y']),(goal['x'],goal['y']))
            angle=math.degrees(abs(angle_error(state['a'],goal['yaw'])))
            forward=any(s[4]>.018 for s in samples[first:])
            backward=any(s[4]<-.018 for s in samples[first:])
            arrival_status=corridor_bridge.snapshot()['arrival']
            record=dict(case=name,status=status['status'],detail=status.get('detail') or status.get('error'),
                        position_error_m=error,yaw_error_deg=angle,forward=forward,reverse=backward,
                        controller=arrival_status['controller'],
                        heading_attempts=arrival_status['heading_attempts'],
                        obstruction_cleared=released,seconds=time.monotonic()-begin)
            results.append(record);print(json.dumps(record,ensure_ascii=False),flush=True)
            limit={'departure':.055,'return':.081}.get(kind,FAS_STATION_POSITION_TOLERANCE+.001)
            if status['status']!='succeeded' or error>limit:
                raise AssertionError('position not reached')
            if kind!='departure' and angle>(10.1 if kind=='return' else 4.1):
                raise AssertionError('heading not reached')
            if reverse is not None and (backward!=reverse or forward==reverse):
                raise AssertionError('guided segment used the wrong direction')
            if obstruct and not released:
                raise AssertionError('guided recovery was not exercised')
            return record

        move('corridor_departure',dict(x=lane_x-.20,y=-4.5,yaw=0.),'departure',reverse=False)
        move('corridor_entry',dict(x=lane_x,y=-4.02,yaw=math.pi/2))
        move('corridor_forward_obstacle',dict(x=lane_x,y=-3.48,yaw=math.pi/2),reverse=False,obstruct=True)
        move('corridor_reverse',dict(x=lane_x,y=-3.80,yaw=math.pi/2),reverse=True)
        move('corridor_cross_aisles',dict(x=lane_x+1.,y=-3.45,yaw=math.pi/2))
        with tempfile.TemporaryDirectory() as directory:
            mission=Mission(fixture_grid,corridor_bridge,directory)
            if mission._can_handoff_dock(corridor_bridge.snapshot()):
                raise AssertionError('dock handoff accepted missing IR')
            from fas_experiment import sample_box
            lease=corridor_bridge.fas_lease
            lease.owner='fixture'
            approach=corridor_bridge.dock_approach
            planned=corridor_bridge.preview_route(approach,'return',corridor_bridge.snapshot())
            if not any(abs(point[1]+3.) < .025 for point in planned):
                raise AssertionError('return did not preview the aisle connector')
            epoch='1'*32
            lease.reserve(epoch,'return',sample_box(planned,pad=.6))
            mission.return_to_dock()
            navigation=mission.operation
            deadline=time.monotonic()+135.
            while mission.phase not in ('docking','failed') and time.monotonic()<deadline:
                lease.renew(epoch)
                step()
                if math.dist((state['x'],state['y']),(approach['x'],approach['y']))<=.14:
                    state['ir']=2
                mission.tick()
            run_for(.45)
            controller_id=corridor_bridge.ops[navigation].get('controller_id')
            if controller_id!='CorridorTrack':
                raise AssertionError('return did not use the aisle controller')
            if mission.phase!='docking' or 1 not in ir_commands:
                raise AssertionError('native IR handoff was not requested: '+mission.message)
            mission.stop()
            run_for(.6)
            if ir_commands[-1]!=0:
                raise AssertionError('dock stop did not disable OEM drive')
        results.append(dict(case='aisle_return_and_native_ir_handoff',passed=True,controller=controller_id,
                            map_id=fixture_grid.map_id))
    try:
        run_for(6.)
        if not client.server_is_ready():raise RuntimeError('controller server unavailable; inspect lifecycle log')
        owners=node.get_publishers_info_by_topic('/cmd_vel')
        if len(owners)!=1 or owners[0].node_name!='navigation_motion_gate':
            raise AssertionError('final velocity must have exactly one publisher')
        if args.corridor_only:
            check_corridors()
            (out/'result.json').write_text(json.dumps({'simulation_only':True,'results':results},indent=2))
            return
        if not drive.server_is_ready():raise RuntimeError('DriveOnHeading unavailable; inspect behavior log')
        first=len(samples)
        origin=(state['x'],state['y'])
        departure=DriveOnHeading.Goal()
        departure.target.x=.30;departure.speed=.08;departure.time_allowance.sec=10
        handle=wait(drive.send_goal_async(departure))
        if not handle.accepted:raise RuntimeError('DriveOnHeading rejected')
        status=wait(handle.get_result_async(),13.).status
        run_for(.6)
        distance=math.dist(origin,(state['x'],state['y']))
        reverse_frames=sum(row[4]<-1e-6 for row in samples[first:])
        max_angular=max(abs(row[5]) for row in samples[first:])
        passed=status==4 and abs(distance-.30)<.035 and not reverse_frames and max_angular<1e-6
        record=dict(case='straight_departure',passed=passed,distance=distance,
                    reverse_frames=reverse_frames,max_angular=max_angular,status=status)
        results.append(record);print(json.dumps(record),flush=True)
        if not passed:raise AssertionError('Straight departure must move forward without rotation')
        start=state['x'],state['y'],state['a']
        target={'x':start[0]-.32,'y':start[1],'yaw':0.}
        leg=fas_axis_leg(start,target)
        assert leg and leg['reverse']
        command_speed=straight_approach_speed(.32,reverse=True)
        parameters=node.create_client(SetParametersAtomically,'/controller_server/set_parameters_atomically')
        speed_request=SetParametersAtomically.Request()
        speed_request.parameters=[Parameter('FixedHeading.desired_linear_vel',value=command_speed).to_parameter_msg()]
        if not wait(parameters.call_async(speed_request),3.).result.successful:
            raise RuntimeError('Reverse speed limit was not accepted')
        reverse_goal=FollowPath.Goal()
        reverse_goal.controller_id='FixedHeading';reverse_goal.goal_checker_id='point_position_checker'
        reverse_goal.path.header.frame_id='map';reverse_goal.path.header.stamp=node.get_clock().now().to_msg()
        for point in leg['poses']:
            q=PoseStamped();q.header=reverse_goal.path.header
            q.pose.position.x=point['x'];q.pose.position.y=point['y']
            q.pose.orientation.z=math.sin(point['yaw']/2);q.pose.orientation.w=math.cos(point['yaw']/2)
            reverse_goal.path.poses.append(q)
        first=len(samples);started_reversing=time.monotonic()
        handle=wait(client.send_goal_async(reverse_goal))
        if not handle.accepted:raise RuntimeError('Reverse FollowPath rejected')
        status=wait(handle.get_result_async(),14.).status
        elapsed=time.monotonic()-started_reversing
        state['allow']=False;run_for(.7);state['allow']=True
        commands=[row[4] for row in samples[first:]]
        error=math.dist((state['x'],state['y']),(target['x'],target['y']))
        yaw_drift=math.degrees(max(abs(row[3]-start[2]) for row in samples[first:]))
        passed=(status==4 and error<=.04 and yaw_drift<=3. and
                min(commands)<-.03 and min(commands)>=-command_speed-1e-4 and max(commands)<1e-6)
        record=dict(case='fas_reverse_station',passed=passed,seconds=elapsed,position_error_m=error,
                    maximum_yaw_change_deg=yaw_drift,min_forward_velocity=min(commands),status=status)
        results.append(record);print(json.dumps(record),flush=True)
        if not passed:raise AssertionError('Reverse tracking must reach the station while holding heading')
        if not spin.server_is_ready():raise RuntimeError('Spin unavailable; inspect behavior log')
        initial_pose=(state['x'],state['y'],state['a'])
        turn=Spin.Goal();turn.target_yaw=.42;turn.time_allowance.sec=8
        first=len(samples)
        handle=wait(spin.send_goal_async(turn))
        if not handle.accepted:raise RuntimeError('Spin rejected')
        status=wait(handle.get_result_async(),10.).status
        run_for(.6)
        turn_error=abs(math.remainder(state['a']-initial_pose[2]-turn.target_yaw,2*math.pi))
        max_linear=max(abs(row[4]) for row in samples[first:])
        min_angular=min(row[5] for row in samples[first:])
        passed=status==4 and turn_error<math.radians(3.) and max_linear<1e-6 and min_angular>=-1e-6
        record=dict(case='fas_native_spin',passed=passed,max_linear_velocity=max_linear,
                    heading_error_deg=math.degrees(turn_error),opposite_angular_command=min_angular,status=status)
        results.append(record);print(json.dumps(record),flush=True)
        if not passed:raise AssertionError('Spin must rotate in place without reversing direction')
        state['allow']=None
        for angle,hold in ((-3.5,0.),(-7.,0.),(30.,2.2),(90.,0.)):
            goal={'x':state['x']+.018,'y':state['y'],'yaw':state['a']+math.radians(angle)}
            if angle==90.:
                h=state['a']
                goal.update(x=state['x']+.42*math.cos(h),y=state['y']+.42*math.sin(h))
            state['allow']=None;state['turn_offset']=.02
            start_time=time.monotonic()
            bridge=station_bridge(goal,start_time+hold)
            identity=bridge.active
            op=bridge.ops[identity]
            if abs(math.radians(angle))<=op['yaw_tolerance']:
                first=len(samples)
                while op['status']!='succeeded' and time.monotonic()-start_time<8.:
                    step()
                passed=(op['status']=='succeeded' and not op['heading_attempts']
                        and op['final_position_error']<=FAS_STATION_POSITION_TOLERANCE
                        and op['final_yaw_error_deg']<=math.degrees(FAS_STATION_YAW_TOLERANCE)
                        and all(abs(r[4])+abs(r[5])<1e-6 for r in samples[first:]))
                record=dict(case='station_already_within_tolerance',passed=passed,requested_deg=angle)
                results.append(record);print(json.dumps(record),flush=True)
                if not passed:raise AssertionError('An in-tolerance stationary waypoint should complete without motion')
                bridge=None
                continue
            if angle==90.:
                while state['v']<.035 and time.monotonic()-start_time<6.:
                    step()
                if state['v']<.035:raise AssertionError('Station approach did not start: '+str(op.get('error')))
                state['obstacle']=(state['x']+.22*math.cos(h),state['y']+.22*math.sin(h))
                run_for(1.)
                blocked=len(samples);run_for(.5)
                if any(abs(r[4])>.005 for r in samples[blocked:]):
                    raise AssertionError('Station approach did not stop for a new obstacle')
                state['obstacle']=None
            while op['heading_attempts']==0 and time.monotonic()-start_time<22.:
                step()
            if not op['heading_attempts']:raise AssertionError('Station did not transition to Spin: '+str(op.get('error')))
            first=len(samples)
            origin=(state['x'],state['y'])
            approach_error=math.dist(origin,(goal['x'],goal['y'])) if angle==90. else None
            spin_start=time.monotonic()
            while op['status']!='succeeded' and time.monotonic()-spin_start<22.:
                step()
            commands=samples[first:]
            sign=math.copysign(1.,angle)
            during_hold=[r for r in commands if r[0]+started<start_time+hold-.1]
            small_turn=abs(math.radians(angle))<=SMALL_SPIN_ANGLE
            attempts_ok=(1<=op['heading_attempts']<=2 and time.monotonic()-spin_start<=6.
                         if small_turn else op['heading_attempts']==1)
            passed=(bridge.active==identity and op['status']=='succeeded' and attempts_ok
                    and (approach_error is None or approach_error<=.03)
                    and op['final_position_error']<=FAS_STATION_POSITION_TOLERANCE
                    and op['final_yaw_error_deg']<=math.degrees(FAS_STATION_YAW_TOLERANCE)
                    and max(abs(r[4]) for r in commands)<1e-6 and min(sign*r[5] for r in commands)>=-1e-6
                    and any(sign*r[5]>.02 for r in commands)
                    and all(abs(r[5])<1e-6 for r in during_hold))
            record=dict(case='station_profile_'+str(angle),passed=passed,attempts=op['heading_attempts'],
                        yaw_error_deg=op.get('final_yaw_error_deg'),position_error_m=op.get('final_position_error'),
                        centre_shift_m=math.dist(origin,(state['x'],state['y'])),held_s=hold,
                        approach_error_m=approach_error,obstacle_wait_resume=approach_error is not None,
                        max_linear_command=max(abs(r[4]) for r in commands),
                        turn_and_settle_s=time.monotonic()-spin_start,seconds=time.monotonic()-start_time)
            results.append(record);print(json.dumps(record),flush=True)
            bridge.publish_spin_limit(0.);bridge=None
            if not passed:raise AssertionError('Station turn exceeded its motion, precision or retry limits')
        # The panel uses map +Y for left. Start once at the first station and
        # carry the measured pose through the side leg and both horizontal legs.
        state.update(x=.1,y=-.4,a=math.pi/2,v=0.,w=0.,allow=None,turn_offset=.02)
        run_for(.8)
        for name,x,y,expected in (('left_heading_side',.42,-.4,'forward'),
                                  ('left_heading_forward',.42,.05,'forward'),
                                  ('left_heading_reverse',.42,-.4,'reverse')):
            goal=dict(x=x,y=y,yaw=math.pi/2)
            start_time=time.monotonic()
            bridge=station_bridge(goal,start_time)
            identity=bridge.active
            op=bridge.ops[identity]
            initial=(state['x'],state['y'])
            dx,dy=x-initial[0],y-initial[1]
            length=math.hypot(dx,dy)
            track=[]
            turn_speed=[]
            while op['status'] not in ('succeeded','failed','canceled') and time.monotonic()-start_time<45.:
                turning=op['status']=='spinning'
                step()
                row=samples[-1]
                if abs(row[4])>.005:
                    track.append((abs((row[1]-initial[0])*dy-(row[2]-initial[1])*dx)/length,row[4]))
                if turning:
                    turn_speed.append(abs(row[4]))
            max_cross=max((t[0] for t in track),default=0.)
            pure_spin=max(turn_speed,default=0.)
            sign=-1 if expected=='reverse' else 1
            passed=(bridge.active==identity and op['status']=='succeeded' and op['travel_direction']==expected
                    and track and all(sign*v>0. for _,v in track) and op['arrival_retries']==0
                    and bool(op.get('line_turn_used'))==(name=='left_heading_side')
                    and op['heading_attempts']<=1 and pure_spin<1e-6 and max_cross<=.055
                    and op['final_position_error']<=FAS_STATION_POSITION_TOLERANCE
                    and op['final_yaw_error_deg']<=math.degrees(FAS_STATION_YAW_TOLERANCE))
            record=dict(case=name,passed=bool(passed),status=op['status'],direction=op.get('travel_direction'),
                        pre_turn=bool(op.get('line_turn_used')),final_turns=op['heading_attempts'],
                        position_replans=op['arrival_retries'],max_cross_track_m=max_cross,
                        max_spin_linear_m_s=pure_spin,position_error_m=op.get('final_position_error'),
                        yaw_error_deg=op.get('final_yaw_error_deg'),seconds=time.monotonic()-start_time)
            results.append(record);print(json.dumps(record),flush=True)
            bridge.publish_spin_limit(0.);bridge=None
            if not passed:raise AssertionError('Fixed left-heading path failed: '+str(record))
        # Same ordinary waypoint, plant and initial state. Only the second approach
        # has independent synthetic calibration stations. Validate against plant pose.
        for calibrated in (False,True):
            radius=.028
            state.update(x=-.1,y=.15,a=0.,v=0.,w=0.,turn_offset=radius,allow=None)
            run_for(.8)
            model=TurnApproach()
            if calibrated:
                for n,angle in enumerate(map(math.radians,(70,80,100,110))):
                    before=(float(n),0.,0.)
                    after=(n+radius*(1-math.cos(angle)),-radius*math.sin(angle),angle)
                    model.observe(n,before,after,angle,math.radians(4),position_tolerance)
            goal=dict(x=.51,y=.15,yaw=math.pi/2)
            start_time=time.monotonic()
            bridge=station_bridge(goal,start_time,fas_station=False,turn_approach=model)
            op=bridge.ops[bridge.active]
            while op['status'] not in ('succeeded','failed','canceled') and time.monotonic()-start_time<38.:
                step()
            error=math.dist((state['x'],state['y']),(goal['x'],goal['y']))
            angle_error=abs(math.remainder(state['a']-goal['yaw'],math.tau))
            aim=op.get('turn_aim')
            passed=(op['status']=='succeeded' and op['heading_attempts']==1 and op['arrival_retries']==0
                    and error<=position_tolerance and angle_error<=yaw_tolerance and aim
                    and (math.dist(aim,(goal['x'],goal['y']))>.005)==calibrated)
            record=dict(case='ordinary_turn_'+('calibrated' if calibrated else 'cold'),passed=bool(passed),
                        position_error_m=error,yaw_error_deg=math.degrees(angle_error),attempts=op['heading_attempts'],
                        position_replans=op['arrival_retries'],aim=aim,seconds=time.monotonic()-start_time)
            results.append(record);print(json.dumps(record),flush=True)
            bridge.publish_spin_limit(0.);bridge=None
            if not passed:raise AssertionError('Ordinary station missed its stopped pose: '+str(record))
        state['turn_offset']=0.
        state['allow']=True
        state['x'],state['y'],state['a']=initial_pose  # Reset only between independent fixture cases.
        run_for(.3)
        speed_request.parameters=[Parameter('FixedHeading.desired_linear_vel',value=.12).to_parameter_msg()]
        wait(parameters.call_async(speed_request),3.)
        scenarios=[('forward',.65,False),('reverse',.05,False),('pedestrian_wait_resume',.65,True)]
        scenarios += [(f'repeat_{i+1}',.05 if i%2==0 else .65,i%4==3) for i in range(args.repeat)]
        if args.safety_only:scenarios=[]
        for name,target,obstruct in scenarios:
            expected_reverse=target<state['x']-.05
            first=len(samples);wall=time.monotonic();attempts=0;block_start=None;block_end=None
            while True:
                handle=wait(client.send_goal_async(path_goal(target)))
                if not handle.accepted:raise RuntimeError('FollowPath rejected')
                result=handle.get_result_async();attempts+=1
                while not result.done():
                    elapsed=time.monotonic()-wall
                    if obstruct and 1.5<elapsed<4.5:
                        if state['obstacle'] is None:
                            state['obstacle']=(state['x']+(-.27 if expected_reverse else .27),0.);block_start=len(samples)
                    elif state['obstacle'] is not None:
                        state['obstacle']=None;block_end=len(samples)
                    if elapsed>60:raise TimeoutError(name)
                    step()
                if result.result().status==4:break
                # Retain the same target after a native obstacle/progress abort.
                state['allow']=False
                delay=min(10.,1.5**attempts)
                end=time.monotonic()+delay
                while time.monotonic()<end:
                    if obstruct and time.monotonic()-wall>=4.5 and state['obstacle'] is not None:
                        state['obstacle']=None;block_end=len(samples)
                    step()
                state['allow']=True
                if attempts>=8:raise RuntimeError(name+' repeatedly failed')
            state['allow']=False;run_for(.7);state['allow']=True
            error=math.hypot(state['x']-target,state['y']);angle=abs(math.atan2(math.sin(state['a']),math.cos(state['a'])))
            refined=error>position_tolerance or angle>yaw_tolerance
            terminal_status=4
            if refined:
                h=wait(client.send_goal_async(path_goal(target,terminal=True)))
                terminal_status=wait(h.get_result_async(),40.).status
                state['allow']=False;run_for(.7);state['allow']=True
                error=math.hypot(state['x']-target,state['y'])
                angle=abs(math.atan2(math.sin(state['a']),math.cos(state['a'])))
            rows=samples[first:]
            record=dict(case=name,position_error_m=error,yaw_error_deg=math.degrees(angle),attempts=attempts,
                        terminal_status=terminal_status,refined=refined,seconds=time.monotonic()-wall,
                        max_abs_heading_deg=max(abs(math.degrees(r[3])) for r in rows),
                        reverse_commands=sum(r[4]<-.005 for r in rows))
            if obstruct:
                record['stationary_during_block']=any(abs(r[4])<1e-8 and abs(r[6])<.01 for r in samples[(block_start or first)+20:block_end])
            results.append(record);print(json.dumps(record),flush=True)
            if error>position_tolerance or angle>yaw_tolerance:raise AssertionError('terminal precision not achieved in synthetic plant')
            if expected_reverse and record['reverse_commands']==0:raise AssertionError('reverse leg turned around')
            if obstruct and not record['stationary_during_block']:raise AssertionError('no observed obstacle stop')
        for name,heading in ([] if args.safety_only else [('turn_90',math.pi/2),('restore_shared_heading',0.)]):
            target=state['x'];wall=time.monotonic()
            h=wait(client.send_goal_async(path_goal(target,heading=heading,terminal=True)))
            result=wait(h.get_result_async(),55.)
            state['allow']=False;run_for(.7);state['allow']=True
            error=math.hypot(state['x']-target,state['y'])
            angle=abs(math.atan2(math.sin(state['a']-heading),math.cos(state['a']-heading)))
            record=dict(case=name,position_error_m=error,yaw_error_deg=math.degrees(angle),
                        terminal_status=result.status,seconds=time.monotonic()-wall)
            results.append(record);print(json.dumps(record),flush=True)
            if result.status!=4 or error>position_tolerance or angle>yaw_tolerance:raise AssertionError(name)
        # Frozen plant isolates the real collision monitor and permission watchdog.
        state['freeze']=True;state['allow']=True
        state['obstacle']=(state['x']+.5,0.);run_for(2.5)
        planner=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
        candidate=path_goal(state['x']+1.).path
        query=ComputePathToPose.Goal();query.goal=candidate.poses[-1];query.planner_id='GridBased';query.use_start=False
        planned=wait(wait(planner.send_goal_async(query)).get_result_async())
        if planned.status!=4:raise AssertionError('native detour planning failed')
        bend=max(abs(p.pose.position.y) for p in planned.result.path.poses)
        validator=node.create_client(IsPathValid,'/is_path_valid')
        request=IsPathValid.Request();request.path=candidate
        straight_valid=wait(validator.call_async(request)).is_valid
        record=dict(case='native_obstacle_detour',max_lateral_offset_m=bend,blocked_straight_valid=straight_valid)
        results.append(record);print(json.dumps(record),flush=True)
        if bend<.08 or straight_valid:raise AssertionError('obstacle was not reflected in native planning')
        state['obstacle']=None;run_for(.5)
        for name,obstacle,sensors,allow in [('clear',None,True,True),('front',(state['x']+.15,state['y']),True,True),
                ('rear',(state['x']-.15,state['y']),True,True),('scan_missing',None,False,True),
                ('scan_invalid',None,'invalid',True),('scan_stale',None,'stale',True),
                ('permit_missing',None,True,None),('permit_revoked',None,True,False)]:
            state.update(obstacle=obstacle,sensors=sensors,allow=allow)
            end=time.monotonic()+1.2;last=[]
            while time.monotonic()<end:
                m=Twist();m.linear.x=-.1 if name=='rear' else .1;raw.publish(m);step()
                if end-time.monotonic()<.25:last.append(state['command'][0])
            passed=(any(abs(v)>.05 for v in last) if name=='clear' else all(abs(v)<1e-8 for v in last))
            record=dict(case=name,passed=passed);results.append(record);print(json.dumps(record),flush=True)
            if not passed:raise AssertionError(name)
        oem.publish(Bool(data=True));run_for(.1)
        oem.publish(Bool(data=True));run_for(1.)
        passed=bool(ir_commands) and all(value==0 for value in ir_commands)
        results.append(dict(case='oem_heartbeat_lost_disables_only',passed=passed))
        if not passed:raise AssertionError('OEM heartbeat loss was not revoked')
        (out/'result.json').write_text(json.dumps({'simulation_only':True,'results':results},indent=2))
    finally:
        bridge=None
        if corridor_bridge is not None:
            corridor_bridge.stop()
        if corridor_executor is not None:
            corridor_executor.shutdown()
            corridor_thread.join(timeout=3.)
            corridor_bridge.destroy_node()
        if fixture is not None:
            fixture.cleanup()
        state['allow']=False
        run_for(.2)
        lifecycle=node.create_client(ManageLifecycleNodes,'/lifecycle_manager/manage_nodes')
        if lifecycle.service_is_ready():
            request=ManageLifecycleNodes.Request();request.command=ManageLifecycleNodes.Request.SHUTDOWN
            try:wait(lifecycle.call_async(request),6.)
            except Exception as error:print('Lifecycle teardown: '+str(error),file=sys.stderr)
        (out/'partial_results.json').write_text(json.dumps(results,indent=2))
        import csv
        with (out/'trajectory.csv').open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(['time','x_truth','y_truth','yaw_truth','command_v','command_w','actual_v','actual_w']);writer.writerows(samples)
        node.destroy_node();rclpy.shutdown()
        for child in children:
            try:os.killpg(child.pid,signal.SIGINT)
            except ProcessLookupError:pass
        for child in children:
            try:child.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in logs:log.close()


if __name__=='__main__':main()
