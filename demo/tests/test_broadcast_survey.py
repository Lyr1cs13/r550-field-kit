"""Synthetic protocol replay, not RF/robot field evidence."""
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile
import queue
import time
import ast
import math
import threading
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from broadcast_survey import BroadcastSurvey,saved_data,export_saved
from survey_route import north_grid_route as north_route, north_route as loop_route
from map_grid import MapGrid
from mission import Mission
from test_demo import ControlledBackend
from collection import Collection


class Clock:
    def __init__(self):self.t=100.
    def __call__(self):return self.t


class BroadcastTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.grid=MapGrid(Path(__file__).resolve().parents[1]/'assets')
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.clock=Clock();self.b=BroadcastSurvey(Path(self.tmp.name)/'collection',self.grid,'fake',clock=self.clock,autostart=False)
        self.addCleanup(self.b.close)
        self.b.status.update(connected=True,stage='ready');self.b.last_packet=100.
        self.p={'id':'p','x':.35,'y':-1.4,'yaw':0,'name':'P','wait':0,'enabled':True}
        self.h={'ready':True,'stationary':True,'localization_confirmed':True,'pose':[.35,-1.4,0.]}
        self.b.prepare([self.p]);self.b.observe(self.h);self.b.begin(self.p,self.h);self.seq=0
    def sample(self,delta=.1,rssi=-62,health=None,device_us=None):
        self.clock.t+=delta;self.seq+=1;self.b.observe(health or self.h)
        self.b.handle_line(f'+BCRSSI:{device_us if device_us is not None else round(self.clock.t*1e6)},{self.seq},1,{rssi},0')
        self.b.check_window()
    def test_complete_requires_duration_and_samples(self):
        for _ in range(60):self.sample()
        self.assertEqual(self.b.snapshot()['stage'],'sampling')
        for _ in range(20):self.sample()
        self.assertEqual(self.b.snapshot()['stage'],'done')
        self.assertEqual(len(self.b.points),1);self.assertEqual(self.b.points[0]['median'],-62)
        with zipfile.ZipFile(io.BytesIO(self.b.export())) as z:
            self.assertIn('heatmap.html',z.namelist());self.assertIn('raw.csv',z.namelist());self.assertIn('map/map.yaml',z.namelist())
            self.assertIn('summary.csv',z.namelist())
    def test_transit_never_enters_heatmap(self):
        self.b.visit=None;self.b.status['stage']='ready'
        for _ in range(80):self.sample(health={**self.h,'stationary':False})
        self.assertEqual(self.b.points,[])
    def test_weak_packet_count_not_fake_done(self):
        for _ in range(20):self.sample(delta=1.)
        self.assertEqual(self.b.snapshot()['stage'],'error');self.assertEqual(self.b.points,[])
    def test_pose_jump_invalidates_visit(self):
        for _ in range(15):self.sample()
        self.sample(health={**self.h,'pose':[.5,-1.4,0.]})
        self.assertEqual(self.b.snapshot()['stage'],'error');self.assertEqual(self.b.points,[])
    def test_radio_reset_invalidates_visit(self):
        self.b.handle_line('+BCSTATUS:100000000,1,0,0')
        self.b.handle_line('+BCSTATUS:1000000,0,0,0')
        self.assertEqual(self.b.snapshot()['stage'],'error')
    def test_receive_sequence_replay_is_not_a_new_sample(self):
        self.sample();self.b.handle_line('+BCRSSI:100100000,1,1,-50,0')
        self.assertEqual(self.b.snapshot()['stage'],'error')
    def test_uart_backlog_excluded(self):
        self.sample();self.clock.t+=2;self.seq+=1;self.b.observe(self.h)
        self.b.handle_line(f'+BCRSSI:100200000,{self.seq},1,-40,0')
        self.assertEqual(len(self.b.visit['rssi']),0)
    def test_queue_overflow_rejects_window(self):
        self.b.drops=1;self.b.check_window();self.assertEqual(self.b.snapshot()['stage'],'error')
    def test_missing_stationary_and_outside_region_rejected(self):
        with self.assertRaises(ValueError):self.b.begin(self.p,{**self.h,'stationary':False})
        with self.assertRaises(ValueError):self.b.prepare([{**self.p,'x':-2}])
    def test_saved_data_survives_worker_close(self):
        for _ in range(80):self.sample()
        self.b.close();d=saved_data(self.tmp.name,self.grid)
        self.assertEqual(len(d['points']),1)
        with zipfile.ZipFile(io.BytesIO(export_saved(self.tmp.name,self.grid))) as z:
            self.assertIn('heatmap.html',z.namelist());self.assertIn('summary.csv',z.namelist())
    def test_no_beacon_before_departure_rejected(self):
        self.clock.t+=4
        with self.assertRaises(ValueError):self.b.prepare([self.p])
    def test_north_route_known_map_safe_and_bounded(self):
        route=north_route(self.grid);self.assertGreater(len(route['points']),20);self.assertLessEqual(len(route['points']),100)
        self.assertEqual(len({p['id'] for p in route['points']}),len(route['points']))
        for p in route['points']:
            self.grid.validate(p['x'],p['y'],.28);self.assertGreaterEqual(p['x'],-.2);self.assertEqual(p['yaw'],0)

    def mission(self):
        backend=ControlledBackend();backend.pose=list(self.h['pose'])
        m=Mission(self.grid,backend,self.tmp.name,self.clock)
        self.addCleanup(m.collection.close)
        m.collection.broadcast=self.b;m.collection.config={'mode':'broadcast','device':'fake'}
        m.save_points([self.p,{**self.p,'id':'p2'}])
        return m,backend

    def test_mission_waits_for_real_window_then_advances_and_clears_speed_cap(self):
        m,backend=self.mission();m.start();self.assertEqual(backend.survey_speed_limit,.14)
        backend.ops[m.operation]['status']='succeeded';m.tick()
        self.assertEqual(m.phase,'collecting');self.assertEqual(len(backend.calls),1)
        for _ in range(60):self.sample()
        m.tick();self.assertEqual(len(backend.calls),1)
        for _ in range(20):self.sample()
        m.tick();self.assertEqual(len(backend.calls),2);self.assertEqual(m.phase,'navigating')
        m.stop();self.assertIsNone(backend.survey_speed_limit)

    def test_missing_beacon_blocks_mission_before_any_motion(self):
        m,backend=self.mission();self.clock.t+=4
        with self.assertRaises(ValueError):m.start()
        self.assertEqual(backend.calls,[])

    def test_moving_loop_collects_without_dwell_and_records_actual_positions(self):
        self.b.moving_demo=True;self.b.prepare([self.p])
        for i in range(90):self.sample(health={**self.h,'stationary':False,'survey_active':True,'pose':[.35,-1.4-i*.005,0.]})
        self.assertGreater(len(self.b.points),0)
        self.assertTrue(all(p['profile']=='moving_loop_demo' for p in self.b.points))
        self.assertLess(self.b.points[-1]['y'],-1.5)
        count=self.b.snapshot()['samples'];self.b.finish_drive()
        for _ in range(10):self.sample(health={**self.h,'survey_active':True})
        self.assertEqual(self.b.snapshot()['samples'],count)

    def test_moving_loop_excludes_non_survey_and_rapid_turns(self):
        self.b.moving_demo=True;self.b.prepare([self.p])
        for _ in range(30):self.sample(health={**self.h,'survey_active':False})
        self.assertEqual(self.b.points,[])
        for i in range(30):self.sample(health={**self.h,'survey_active':True,'pose':[.35,-1.4,i*.09]})
        self.assertLessEqual(self.b.status['samples'],1)  # The initial zero-angle sample is valid.

    def test_loop_guides_advance_without_waiting_or_final_yaw(self):
        self.b.moving_demo=True;m,backend=self.mission();m.start()
        self.assertEqual(backend.calls[-1][1],'survey');self.assertEqual(backend.survey_speed_limit,.12)
        backend.ops[m.operation]['status']='succeeded';m.tick()
        self.assertEqual(m.index,1);self.assertEqual(m.phase,'navigating')

    def test_fast_route_is_few_guides_and_respects_thresholds(self):
        r=loop_route(self.grid);self.assertEqual(len(r['points']),2)
        self.assertTrue(all(p['wait']==0 for p in r['points']))
        self.assertLess(r['planned_grid_path_length_m'],40)
        for x,y in r['preview_path']:self.grid.validate(x,y,.24)

    def test_real_backend_survey_arrival_never_enters_heading_correction(self):
        source=ast.parse((Path(__file__).resolve().parents[1]/'ros_backend.py').read_text(encoding='utf-8'))
        method=next(n for c in source.body if isinstance(c,ast.ClassDef) for n in c.body
                    if isinstance(n,ast.FunctionDef) and n.name=='finish_waypoint_position')
        namespace={'math':math};exec(compile(ast.Module(body=[method],type_ignores=[]),'actual_backend','exec'),namespace)
        stub=SimpleNamespace(pose=[.35,-1.4,2.],stationary_since=99.,zero_motion=lambda:None)
        op={'kind':'survey','goal':{'x':.4,'y':-1.4,'yaw':0}}
        namespace['finish_waypoint_position'](stub,op,100.)
        self.assertEqual(op['status'],'succeeded');self.assertEqual(op['quality'],'transit')

    def test_default_panel_loads_loop_preview_and_blocks_wrong_mode(self):
        from server import App
        from mock_backend import MockBackend
        dock=json.loads((Path(__file__).resolve().parents[1]/'assets/dock_start.json').read_text(encoding='utf-8'))
        backend=MockBackend(self.grid,dock);backend.confirm_localization()
        app=App(self.grid,backend,self.tmp.name);self.addCleanup(app.mission.collection.close)
        state=app.command({'action':'survey_load_north'})
        self.assertEqual(len(state['points']),2);self.assertGreater(len(state['survey_route_preview']),10)
        self.assertTrue(all(len(p['survey_path'])>100 for p in state['points']))
        with self.assertRaises(ValueError):app.mission.start()
        self.assertEqual(app.mission.phase,'idle')

    def test_resume_keeps_prior_measurements_and_original_segment(self):
        self.b.moving_demo=True;self.b.prepare([self.p])
        for i in range(30):self.sample(health={**self.h,'survey_active':True})
        self.assertTrue(self.b.points);previous=self.b.session;count=self.b.status['samples']
        self.b.finish_drive();self.b.inherit_session=previous;self.b.prepare([self.p])
        self.assertEqual(self.b.status['samples'],count);self.assertTrue(self.b.points)
        self.assertTrue((self.b.folder/'segments'/previous/'raw.csv').exists())

    def test_survey_guide_near_obstacle_can_pass_without_exact_endpoint(self):
        source=ast.parse((Path(__file__).resolve().parents[1]/'ros_backend.py').read_text(encoding='utf-8'))
        methods=[n for c in source.body if isinstance(c,ast.ClassDef) for n in c.body
                 if isinstance(n,ast.FunctionDef) and n.name in ('retry_arrival','finish_waypoint_position')]
        namespace={'math':math,'time':time};exec(compile(ast.Module(body=methods,type_ignores=[]),'actual_backend','exec'),namespace)
        stub=SimpleNamespace(pose=[3.735,-6.752,0.],stationary_since=99.,zero_motion=lambda:None)
        op={'kind':'survey','goal':{'x':4.025,'y':-6.675,'yaw':1.57}}
        namespace['retry_arrival'](stub,op,'blocked endpoint')
        self.assertEqual(op['status'],'position_settling')
        namespace['finish_waypoint_position'](stub,op,100.)
        self.assertEqual(op['status'],'succeeded')

    def test_resume_aggregation_does_not_block_uart_lock(self):
        self.b.moving_demo=True;self.b.prepare([self.p])
        for _ in range(20):self.sample(health={**self.h,'survey_active':True})
        self.b.finish_drive();self.b.inherit_session=self.b.session
        acquired=[];original=BroadcastSurvey._drive_sample
        def aggregate(instance,*args):
            if instance is not self.b and not acquired:
                def probe():
                    ok=self.b.lock.acquire(timeout=.2);acquired.append(ok)
                    if ok:self.b.lock.release()
                t=threading.Thread(target=probe);t.start();t.join(.4)
            return original(instance,*args)
        BroadcastSurvey._drive_sample=aggregate
        try:self.b.prepare([self.p])
        finally:BroadcastSurvey._drive_sample=original
        self.assertEqual(acquired,[True])

    def test_moving_receiver_reboot_starts_new_radio_epoch(self):
        self.b.moving_demo=True;self.b.prepare([self.p])
        self.b.handle_line('+BCSTATUS:100000000,1,0,0')
        self.b.handle_line('+BCSTATUS:1000000,0,0,0')
        self.assertEqual(self.b.epoch,1);self.assertNotEqual(self.b.status['stage'],'error')

    def test_moving_duplicate_is_discarded_without_breaking_receiver(self):
        self.b.moving_demo=True;self.b.prepare([self.p])
        self.b.handle_line('+BCRSSI:100000000,1,1,-70,0')
        self.b.handle_line('+BCRSSI:100000000,1,1,-70,0')
        self.assertTrue(self.b.status['connected']);self.assertIsNone(self.b.status['error'])


class StreamTests(unittest.TestCase):
    def test_serial_io_failure_reconnects_without_manual_intervention(self):
        devices=[]
        class Stream:
            def __init__(self,n):self.n=n;self.buffer=bytearray();self.closed=False
            def reset_input_buffer(self):pass
            def write(self,data):
                self.buffer.extend(f'+BCHELLO:1,RX,1,0,100\r\n+BCSTATUS:{(self.n+1)*1000000},1,0,0\r\n+BCRSSI:{(self.n+1)*1000000},{self.n+1},1,-64,0\r\n'.encode())
            def read(self,n):
                if self.buffer:data=bytes(self.buffer[:n]);del self.buffer[:n];return data
                if self.n==0:raise OSError('synthetic USB disconnect')
                time.sleep(.02);return b''
            def flush(self):pass
            def close(self):self.closed=True
        def factory(*args,**kwargs):
            device=Stream(len(devices));devices.append(device);return device
        with tempfile.TemporaryDirectory() as folder:
            grid=MapGrid(Path(__file__).resolve().parents[1]/'assets');b=BroadcastSurvey(folder,grid,'fake',factory=factory)
            try:
                deadline=time.monotonic()+5
                while time.monotonic()<deadline:
                    if len(devices)>=2 and b.snapshot()['connected'] and b.snapshot()['beacon_recent']:break
                    time.sleep(.02)
                self.assertGreaterEqual(len(devices),2);self.assertTrue(b.snapshot()['connected'])
                self.assertGreaterEqual(b.reconnects,1);self.assertIsNone(b.snapshot()['error'])
            finally:b.close()

    def test_configuration_queued_is_not_ready_to_depart(self):
        with tempfile.TemporaryDirectory() as folder:
            grid=MapGrid(Path(__file__).resolve().parents[1]/'assets');c=Collection(folder,grid)
            try:
                c.config={'mode':'broadcast','device':'auto'}
                with self.assertRaises(ValueError):c.prepare([])
            finally:c.close()

    def test_uart_worker_handshakes_and_reads_fragmented_reports(self):
        class Stream:
            def __init__(self):self.q=queue.Queue();self.writes=[];self.closed=False
            def reset_input_buffer(self):pass
            def write(self,data):
                self.writes.append(data)
                self.q.put(b'+BCHELLO:1,RX,1,0,100\r\nOK\r\n+BCSTA')
                self.q.put(b'TUS:1000000,1,0,0\r\n+BCRSSI:1000000,1,1,-63,0\r\n')
            def read(self,n):
                try:return self.q.get(timeout=.02)
                except queue.Empty:return b''
            def flush(self):pass
            def close(self):self.closed=True
        with tempfile.TemporaryDirectory() as folder:
            stream=Stream();grid=MapGrid(Path(__file__).resolve().parents[1]/'assets')
            b=BroadcastSurvey(folder,grid,'fake',factory=lambda *a,**k:stream)
            try:
                deadline=time.monotonic()+3
                while not b.snapshot()['beacon_recent'] and time.monotonic()<deadline:time.sleep(.01)
                status=b.snapshot();self.assertTrue(status['connected']);self.assertTrue(status['beacon_recent'])
                self.assertEqual(status['last_rssi'],-63);self.assertEqual(stream.writes,[b'AT+BCHELLO\r\n'])
            finally:b.close()
            self.assertTrue(stream.closed)


if __name__=='__main__':unittest.main()
