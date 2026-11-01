import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from map_grid import MapGrid
from mission import Mission
from mock_backend import MockBackend


class Clock:
    def __init__(self): self.now = 100.
    def __call__(self): return self.now
    def advance(self,seconds): self.now += seconds


class ControlledBackend:
    """Delayed cancellation and real arrival/charge notifications, not distance guesses."""
    def __init__(self):
        self.dock_pose = [0,0,0];self.dock_approach = {'x':.6,'y':0,'yaw':-.26}
        self.ops = {};self.calls = [];self.confirmed = True;self.ready = True
        self.charging = False;self.docking_error = None;self.docking_started = False;self.voltage = 11.
        self.stationary = True
        self.pose = None;self.ir = 0
    def snapshot(self):
        return {'ready':self.ready,'readiness':'sensor unavailable' if not self.ready else 'ready',
                'localization_confirmed':self.confirmed,'charging':self.charging,'voltage':self.voltage,'stationary':self.stationary,
                'pose':self.pose,'ir':self.ir}
    def navigate(self,point,kind):
        identity = str(len(self.calls));self.calls.append((dict(point),kind))
        self.ops[identity] = {'status':'navigating'};return identity
    def operation(self,identity): return self.ops[identity]
    def cancel(self,identity): self.ops[identity]['status'] = 'canceling'
    def stop(self): pass
    def tick(self): pass
    def begin_docking(self): self.docking_started = True


class DemoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.grid = MapGrid(ROOT/'assets')
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory();self.addCleanup(self.directory.cleanup)
        self.clock = Clock();self.backend = ControlledBackend()
        self.mission = Mission(self.grid,self.backend,self.directory.name,self.clock)
        self.points = [{'id':'a','name':'A','x':.4,'y':-2.,'yaw':-math.pi/2,'wait':10,'enabled':True},
                       {'id':'b','name':'B','x':.4,'y':-3.,'yaw':-math.pi/2,'wait':3,'enabled':True}]
        self.mission.save_points(self.points)

    def arrive(self):
        self.backend.ops[self.mission.operation]['status'] = 'succeeded';self.mission.tick()

    def test_arrival_then_dwell_and_order(self):
        m = self.mission;m.start();self.clock.advance(90);m.tick()
        self.assertEqual(m.phase,'navigating');self.assertEqual(len(self.backend.calls),1)
        self.arrive();self.assertEqual(m.phase,'dwelling')
        self.clock.advance(9.9);m.tick();self.assertEqual(len(self.backend.calls),1)
        self.clock.advance(.1);m.tick();self.assertEqual(self.backend.calls[-1][0]['id'],'b')
        self.arrive();self.clock.advance(3);m.tick();self.assertEqual(m.phase,'completed')

    def test_pause_keeps_remaining_dwell(self):
        m = self.mission;m.start();self.arrive();self.clock.advance(3);m.pause()
        self.assertAlmostEqual(m.remaining,7);self.clock.advance(100);m.tick()
        self.assertEqual(len(self.backend.calls),1);m.resume();self.clock.advance(6.9);m.tick()
        self.assertEqual(m.phase,'dwelling');self.clock.advance(.1);m.tick()
        self.assertEqual(m.phase,'navigating')

    def test_dwell_waits_for_stationary_feedback(self):
        m=self.mission;m.start();self.backend.stationary=False;self.arrive()
        self.assertEqual(m.phase,'settling');self.clock.advance(4);m.tick()
        self.assertEqual(m.phase,'settling');self.backend.stationary=True;m.tick()
        self.assertEqual(m.phase,'dwelling');self.assertAlmostEqual(m.until-self.clock(),10)

    def test_return_waits_for_cancel_ack(self):
        m = self.mission;m.start();old=m.operation;m.return_to_dock();m.tick()
        self.assertEqual(m.phase,'canceling');self.assertEqual(len(self.backend.calls),1)
        self.backend.ops[old]['status']='canceled';m.tick()
        self.assertEqual(self.backend.calls[-1][1],'return');self.assertEqual(len(self.backend.calls),2)
        self.arrive();self.assertTrue(self.backend.docking_started);self.assertEqual(m.phase,'docking')
        m.tick();self.assertEqual(m.phase,'docking')
        self.backend.charging=True;m.tick();self.assertEqual(m.phase,'docked')

    def test_pause_navigation_replans_same_point(self):
        m=self.mission;m.start();old=m.operation;m.pause();m.tick()
        self.backend.ops[old]['status']='canceled';m.tick();self.assertEqual(m.phase,'paused_navigation')
        m.resume();self.assertEqual(self.backend.calls[-1][0]['id'],'a');self.assertEqual(m.index,0)

    def test_stop_during_dwell_does_not_start_next(self):
        m=self.mission;m.start();self.arrive();m.stop();self.clock.advance(1000);m.tick()
        self.assertEqual(m.phase,'idle');self.assertEqual(len(self.backend.calls),1)

    def test_failure_does_not_skip_to_next(self):
        m=self.mission;m.start();self.backend.ops[m.operation].update(status='failed',error='blocked')
        m.tick();self.assertEqual(m.phase,'failed');self.assertEqual(len(self.backend.calls),1)

    def test_readiness_confirmation_and_health_loss(self):
        m=self.mission;self.backend.confirmed=False
        with self.assertRaises(ValueError):m.start()
        self.backend.confirmed=True;m.start();self.backend.ready=False;m.tick()
        self.assertEqual(m.phase,'canceling');self.assertEqual(self.backend.ops[m.operation]['status'],'canceling')

    def test_failed_contact_retries_the_return_flow_once(self):
        m=self.mission;self.backend.pose=[.6,0.,0.];self.backend.ir=3
        m.phase='docking';m._fail('对接超时，未收到充电反馈，已停止')
        m.tick();self.clock.advance(3);m.tick()
        self.assertEqual(m.dock_retry_count,1);self.assertEqual(m.phase,'docking')
        m._fail('对接超时，未收到充电反馈，已停止');self.clock.advance(10);m.tick()
        self.assertEqual(m.phase,'failed');self.assertFalse(m.dock_retry_pending)

    def test_late_stable_charge_does_not_push_again(self):
        m=self.mission;m.phase='docking';m._fail('桩旁轮程已达到保护上限但未检测到充电，已停止对接')
        self.backend.charging=True;m.tick()
        self.assertEqual(m.phase,'docked');self.assertFalse(self.backend.docking_started)

    def test_charge_confirmation_after_retry_budget_does_not_remain_failed(self):
        m=self.mission;m.phase='docking';m.dock_retry_count=1
        m._fail('桩旁轮程已达到保护上限但未检测到充电，已停止对接')
        self.backend.charging=True;m.tick()
        self.assertEqual(m.phase,'docked');self.assertFalse(self.backend.docking_started)

    def test_stop_cancels_scheduled_dock_retry(self):
        m=self.mission;m.phase='docking';m._fail('对接超时，未收到充电反馈，已停止')
        m.stop();self.clock.advance(10);m.tick()
        self.assertEqual(m.phase,'idle');self.assertFalse(self.backend.docking_started)

    def test_unknown_keepout_and_map_version(self):
        self.assertGreater(self.grid.cells.count(-1),0)
        unknown = self.grid.cells.index(-1)
        ux,uy = self.grid.world(unknown%self.grid.width,unknown//self.grid.width)
        with self.assertRaises(ValueError):self.grid.validate(ux,uy)
        with self.assertRaises(ValueError):self.grid.validate(-10,-10)
        with self.assertRaises(ValueError):self.grid.validate(2.4,.25)
        p=Path(self.directory.name)/'waypoints.json';d=json.loads(p.read_text(encoding='utf-8'));d['map_id']='wrong';p.write_text(json.dumps(d),encoding='utf-8')
        other=Mission(self.grid,self.backend,self.directory.name,self.clock);self.assertEqual(other.points,[])

    def test_invalid_point_does_not_replace_good_data(self):
        original=json.loads((Path(self.directory.name)/'waypoints.json').read_text(encoding='utf-8'))
        bad=[dict(self.points[0],wait=float('nan'))]
        with self.assertRaises(ValueError):self.mission.save_points(bad)
        self.assertEqual(json.loads((Path(self.directory.name)/'waypoints.json').read_text(encoding='utf-8')),original)

    def test_offline_path_bypasses_threshold(self):
        path=self.grid.offline_path([0,0],[3.3,-.2])
        self.assertGreater(len(path),40)
        self.assertFalse(any(1.8<=x<=2.2 and -.4<=y<=.3 for x,y in path))
        self.assertLess(min(y for x,y in path),-.5)

    def test_mock_full_mission_with_charging(self):
        dock=json.loads((ROOT/'assets/dock_start.json').read_text(encoding='utf-8'))
        backend=MockBackend(self.grid,dock,self.clock,speed=4);backend.confirm_localization()
        m=Mission(self.grid,backend,self.directory.name,self.clock);m.start(auto_return=True)
        for _ in range(400):
            self.clock.advance(.25);m.tick()
            if m.phase in ('docked','failed'):break
        self.assertEqual(m.phase,'docked');self.assertTrue(backend.charging)
        self.assertEqual(sum('停留结束' in e['text'] for e in m.events),2)

    def test_departure_is_forward_before_first_waypoint(self):
        m=self.mission;self.backend.charging=True;m.start()
        self.assertEqual(m.phase,'departing');self.assertEqual(self.backend.calls[0][1],'departure')
        self.assertEqual(self.backend.calls[0][0]['yaw'],0)
        self.assertAlmostEqual(self.backend.calls[0][0]['x'],.30)
        self.assertAlmostEqual(self.backend.dock_approach['x'],.60)
        self.backend.charging=False;self.arrive()
        self.assertEqual(self.backend.calls[1][0]['id'],'a');self.assertEqual(m.index,0)

    def test_departure_without_disconnect_does_not_continue(self):
        m=self.mission;self.backend.charging=True;m.start();self.arrive()
        self.assertEqual(m.phase,'failed');self.assertEqual(len(self.backend.calls),1)

    def test_operator_confirmed_dock_departure_without_charge_flag(self):
        original=self.backend.snapshot
        self.backend.snapshot=lambda:{**original(),'dock_position_confirmed':True}
        self.assertFalse(self.backend.charging)
        self.mission.start()
        self.assertEqual(self.mission.phase,'departing')
        self.assertEqual(self.backend.calls[0][1],'departure')

    def test_dock_capture_waits_for_cancel_ack(self):
        m=self.mission;m.return_to_dock();old=m.operation
        self.backend.pose=[.59,0,.25];self.backend.ir=2;m.tick()
        self.assertEqual(m.phase,'canceling');self.assertFalse(self.backend.docking_started)
        m.tick();self.assertFalse(self.backend.docking_started)
        self.backend.ops[old]['status']='canceled';m.tick()
        self.assertEqual(m.phase,'docking');self.assertTrue(self.backend.docking_started)

    def test_dock_capture_rechecks_signal_after_cancel(self):
        m=self.mission;m.return_to_dock();old=m.operation
        self.backend.pose=[.59,0,.25];self.backend.ir=2;m.tick()
        self.backend.ir=0;self.backend.ops[old]['status']='canceled';m.tick()
        self.assertEqual(m.phase,'failed');self.assertFalse(self.backend.docking_started)

    def test_dock_capture_requires_stopped_nearby_aligned_and_ir(self):
        good={'ready':True,'localization_confirmed':True,'stationary':True,'ir':2,'pose':[.59,0,.25]}
        self.assertTrue(self.mission._can_handoff_dock(good))
        for patch in ({'pose':[1.2,0,0]},{'pose':[.6,0,math.pi/2]},
                      {'stationary':False},{'ir':0},{'ready':False},{'localization_confirmed':False}):
            self.assertFalse(self.mission._can_handoff_dock({**good,**patch}),patch)

    def test_dock_capture_does_not_relax_waypoint_success(self):
        self.mission.start();self.backend.pose=[.59,0,.25];self.backend.ir=2
        self.backend.ops[self.mission.operation].update(status='failed',error='heading')
        self.mission.tick();self.assertEqual(self.mission.phase,'failed')
        self.assertFalse(self.backend.docking_started)

    def test_near_home_return_uses_ir_and_allows_factory_alignment_time(self):
        self.backend.pose=[.02,.03,.1];self.backend.ir=2
        m=self.mission;m.return_to_dock()
        self.assertEqual(m.phase,'docking');self.assertEqual(self.backend.calls,[])
        self.clock.advance(31);m.tick()
        self.assertEqual(m.phase,'docking')
        self.backend.charging=True;m.tick();self.assertEqual(m.phase,'docked')

    def test_dock_capture_rejects_pose_behind_station(self):
        h={'ready':True,'localization_confirmed':True,'stationary':True,'ir':2,'pose':[-.1,0,0]}
        self.assertFalse(self.mission._can_handoff_dock(h))

    def test_dock_alignment_still_has_a_total_timeout(self):
        self.backend.pose=[.02,.03,.1];self.backend.ir=2
        m=self.mission;m.return_to_dock();self.clock.advance(91);m.tick()
        self.assertEqual(m.phase,'failed');self.assertIn('对接超时',m.message)

    def test_charge_bounce_does_not_finish_and_lost_contact_is_reported(self):
        self.backend.pose=[.02,.03,.1];self.backend.ir=2
        original=self.backend.snapshot;stable=[False]
        self.backend.snapshot=lambda:dict(original(),charging_stable=stable[0])
        self.mission.return_to_dock();self.backend.charging=True;self.mission.tick()
        self.assertEqual(self.mission.phase,'docking')
        stable[0]=True;self.mission.tick();self.assertEqual(self.mission.phase,'docked')
        stable[0]=False;self.backend.charging=False;self.mission.tick();self.clock.advance(2.1);self.mission.tick()
        self.assertEqual(self.mission.phase,'idle');self.assertIn('接触已中断',self.mission.message)


if __name__ == '__main__': unittest.main()
