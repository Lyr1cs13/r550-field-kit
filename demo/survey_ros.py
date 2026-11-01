"""Continuous survey via native collision-aware FollowPath, with native rejoin."""
import math,time,uuid
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath,ComputePathToPose
from nav2_msgs.srv import IsPathValid
from rcl_interfaces.srv import SetParametersAtomically,GetParameters
from rclpy.parameter import Parameter
from survey_path import checked_path,advance_index,ahead

class SurveyPathMixin:
    def navigate_survey_path(self,point):
        with self.lock:
            if self.active and self.ops[self.active]['status'] not in ('succeeded','failed','canceled'):raise ValueError('旧导航尚未停止')
            path=checked_path(self.grid,point['survey_path'],(point['x'],point['y']))
            if self.pose is None or math.dist(self.pose[:2],path[0])>.4:raise ValueError('当前位置不在连续路线起点附近')
            identity=uuid.uuid4().hex
            self.ops[identity]=dict(status='validating',goal=dict(point),route_goal=dict(point),kind='survey',route_path=path,route_index=0,
                route_retries=0,route_rejoining=False,sequential=False,controller_id='PointPosition',xy_tolerance=.16,yaw_tolerance=math.pi,
                started=time.monotonic(),created=time.monotonic(),timeout=30.,cancel=False,handle=None,heading_attempts=0,arrival_retries=0)
            self.active=identity;self.path=path
            if not hasattr(self,'survey_path_validator'):self.survey_path_validator=self.create_client(IsPathValid,'/is_path_valid')
            self.validate_survey_head(identity)
            return identity

    def survey_message_path(self,points):
        path=Path();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
        for i,p in enumerate(points):
            other=points[min(i+1,len(points)-1)] if i+1<len(points) else points[max(0,i-1)]
            angle=math.atan2(other[1]-p[1],other[0]-p[0]) if i+1<len(points) else math.atan2(p[1]-other[1],p[0]-other[0])
            path.poses.append(self.stamped(dict(x=p[0],y=p[1],yaw=angle)))
        return path

    def validate_survey_head(self,identity):
        op=self.ops[identity]
        if op['cancel']:op['status']='canceled';return
        if not self.survey_path_validator.service_is_ready():op.update(status='failed',error='实时路线检查服务尚未就绪');return
        points=op['route_path'];i=op['route_index'];end=ahead(points,i,1.2)
        req=IsPathValid.Request();req.path=self.survey_message_path([list(self.pose[:2])]+points[i:end+1])
        self.survey_path_validator.call_async(req).add_done_callback(lambda f:self.survey_validated(f,identity))

    def survey_validated(self,future,identity):
        with self.lock:
            op=self.ops[identity]
            if op['cancel']:op['status']='canceled';return
            try:
                if not future.result().is_valid:self.rejoin_survey(identity,'前方实时路径被占用');return
                if not hasattr(self,'survey_original_controller_parameters'):
                    if not hasattr(self,'survey_read_parameters'):self.survey_read_parameters=self.create_client(GetParameters,'/controller_server/get_parameters')
                    request=GetParameters.Request();request.names=['point_position_checker.xy_goal_tolerance','PointPosition.desired_linear_vel','PointPosition.lookahead_dist','PointPosition.min_lookahead_dist','PointPosition.max_lookahead_dist','PointPosition.max_robot_pose_search_dist']
                    self.survey_read_parameters.call_async(request).add_done_callback(lambda f:self.survey_saved_parameters(f,identity,request.names));return
                self.configure_survey_follow(identity)
            except Exception as exc:op.update(status='failed',error=str(exc))

    def survey_saved_parameters(self,future,identity,names):
        with self.lock:
            op=self.ops[identity]
            try:
                values=future.result().values
                if len(values)!=len(names) or any(v.type!=3 for v in values):raise ValueError('无法保存原控制参数，未启动')
                self.survey_original_controller_parameters={k:v.double_value for k,v in zip(names,values)}
                if op['cancel']:op['status']='canceled';return
                self.configure_survey_follow(identity)
            except Exception as exc:op.update(status='failed',error=str(exc))

    def configure_survey_follow(self,identity):
        with self.lock:
            op=self.ops[identity]
            try:
                values={'point_position_checker.xy_goal_tolerance':.16,'PointPosition.desired_linear_vel':.08,
                    'PointPosition.lookahead_dist':.65,'PointPosition.min_lookahead_dist':.55,
                    'PointPosition.max_lookahead_dist':.70,'PointPosition.max_robot_pose_search_dist':.50}
                req=SetParametersAtomically.Request();req.parameters=[Parameter(k,value=float(v)).to_parameter_msg() for k,v in values.items()]
                op['status']='configuring'
                self.controller_parameters.call_async(req).add_done_callback(lambda f:self.survey_configured(f,identity))
            except Exception as exc:op.update(status='failed',error=str(exc))

    def survey_configured(self,future,identity):
        with self.lock:
            op=self.ops[identity]
            if op['cancel']:op['status']='canceled';return
            try:
                result=future.result().result
                if not result.successful:raise ValueError(result.reason)
                points=op['route_path'];i=op['route_index']
                current=list(self.pose[:2]);remaining=[current]+points[i:]
                goal=FollowPath.Goal();goal.path=self.survey_message_path(remaining);goal.controller_id='PointPosition';goal.goal_checker_id='point_position_checker'
                op.update(status='navigating',route_rejoining=False,started=time.monotonic(),timeout=max(60,sum(math.dist(a,b) for a,b in zip(remaining,remaining[1:]))/.055+60))
                self.path=remaining;self.speed_limits['PointPosition']=.08
                self.follow.send_goal_async(goal).add_done_callback(lambda f:self.survey_accepted(f,identity))
            except Exception as exc:op.update(status='failed',error=str(exc))

    def survey_accepted(self,future,identity):
        with self.lock:
            op=self.ops[identity]
            try:
                h=future.result()
                if not h.accepted:raise ValueError('原生路线跟踪拒绝任务')
                op['handle']=h;h.get_result_async().add_done_callback(lambda f:self.survey_done(f,identity))
                if op['cancel']:h.cancel_goal_async()
            except Exception as exc:op.update(status='failed',error=str(exc))

    def survey_route_tick(self,op):
        if getattr(self,'survey_restore_pending',False) and self.stationary_since and time.monotonic()-self.stationary_since>.7:self.restore_survey_controller()
        if not op.get('route_path') or not self.pose or op.get('route_rejoining') or op['status']!='navigating':return
        path=op['route_path'];index=advance_index(path,op['route_index'],self.pose);op['route_index']=index
        # Bounded forward search cannot jump from the outward to the adjacent return leg.
        distance=math.dist(path[index],self.pose[:2]);now=time.monotonic()
        if distance>.28:
            op.setdefault('off_route_since',now)
            if now-op['off_route_since']>.6 and op.get('handle'):
                op.update(status='route_stopping',started=now,timeout=12.)
                op['handle'].cancel_goal_async();self.zero_motion();return
        else:op.pop('off_route_since',None)
        j=ahead(path,index,.65);k=ahead(path,index,.12)
        a=math.atan2(path[k][1]-path[index][1],path[k][0]-path[index][0])
        k2=ahead(path,j,.12);b=math.atan2(path[k2][1]-path[j][1],path[k2][0]-path[j][0])
        turning=abs(math.atan2(math.sin(b-a),math.cos(b-a)))>.30
        op['track_speed']=.045 if turning else .08
        if op.get('track_turning')==turning or op.get('track_update_pending') or self.speed_update is not None:return
        values={'PointPosition.desired_linear_vel':op['track_speed'],
                'PointPosition.lookahead_dist':.25 if turning else .65,
                'PointPosition.min_lookahead_dist':.20 if turning else .55,
                'PointPosition.max_lookahead_dist':.30 if turning else .70}
        request=SetParametersAtomically.Request();request.parameters=[Parameter(k,value=float(v)).to_parameter_msg() for k,v in values.items()]
        op['track_update_pending']=True
        def configured(f):
            with self.lock:
                op['track_update_pending']=False
                try:
                    if f.result().result.successful:
                        op['track_turning']=turning;self.speed_limits['PointPosition']=values['PointPosition.desired_linear_vel']
                    else:self.get_logger().error('采集弯道参数未应用：'+f.result().result.reason)
                except Exception as exc:self.get_logger().error(str(exc))
        self.controller_parameters.call_async(request).add_done_callback(configured)

    def survey_done(self,future,identity):
        with self.lock:
            op=self.ops[identity];op['handle']=None
            try:
                result=future.result()
                if op['cancel']:self.finish_native_cancellation(op);return
                if result.status==4:
                    p=self.pose;g=op['goal']
                    if not p or math.hypot(p[0]-g['x'],p[1]-g['y'])>.20:raise ValueError('路线完成反馈与当前位置不符')
                    op['status']='succeeded';self.path=[];return
                self.zero_motion();self.rejoin_survey(identity,'实时障碍中断跟踪，重新规划接回路线')
            except Exception as exc:op.update(status='failed',error=str(exc))

    def rejoin_survey(self,identity,reason):
        op=self.ops[identity]
        if op['cancel']:op['status']='canceled';return
        if op['route_retries']>=3:op.update(status='failed',error='连续路线三次避障接回未成功，保持停车');return
        op['route_retries']+=1;points=op['route_path'];index=advance_index(points,op['route_index'],self.pose)
        target=ahead(points,index,1.2);x,y=points[target]
        op.update(controller_id='FollowPath',route_rejoining=True,route_rejoin_index=target,goal=dict(x=x,y=y,yaw=0.),status='planning',started=time.monotonic(),timeout=30.)
        self.get_logger().warn(reason+'；使用原生避障规划接回，不直接直推')
        goal=ComputePathToPose.Goal();goal.goal=self.stamped(op['goal']);goal.planner_id='GridBased';goal.use_start=False
        self.plan.send_goal_async(goal).add_done_callback(lambda f:self.plan_accepted(f,identity))

    def finish_survey_rejoin(self,identity):
        op=self.ops[identity]
        op.update(controller_id='PointPosition',goal=dict(op['route_goal']),route_index=op['route_rejoin_index'],route_rejoining=False,status='validating',started=time.monotonic(),timeout=30.)
        self.validate_survey_head(identity)

    def restore_survey_controller(self):
        values=getattr(self,'survey_original_controller_parameters',None)
        if not values:return
        if not self.stationary_since or time.monotonic()-self.stationary_since<.7:
            self.survey_restore_pending=True;return
        self.survey_restore_pending=False
        request=SetParametersAtomically.Request();request.parameters=[Parameter(k,value=float(v)).to_parameter_msg() for k,v in values.items()]
        future=self.controller_parameters.call_async(request)
        def done(f):
            try:
                if not f.result().result.successful:self.get_logger().error('采集控制参数恢复失败：'+f.result().result.reason)
                else:self.speed_limits.pop('PointPosition',None)
            except Exception as exc:self.get_logger().error('采集控制参数恢复失败：'+str(exc))
        future.add_done_callback(done)
