"""No-motion tests of real mission handoff and actual serial state machine."""
from pathlib import Path
import sys
import tempfile
import io
import json
import zipfile
import time
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from collection_export import merged_csv
from collection import Collection
from map_grid import MapGrid
from mission import Mission
from test_demo import ControlledBackend, Clock, ROOT


class SerialDevice:
    def __init__(self,*args,**kwargs):
        self.buffer=bytearray();self.run=0;self.visit=0;self.state=0
        self.result=0;self.points=[];self.late=False
    def write(self,data):
        cmd=data.decode().strip();lines=[]
        if cmd=='AT+RDKHELLO': lines=['+RDKHELLO:1,COLLECTOR','+RDKCAP:2,SLE_SCAN_GATED']
        elif cmd.startswith('AT+RDKSESSION='):
            self.run=int(cmd.split('=')[1]);lines=[f'+RDKSESSION:{self.run}']
        elif cmd.startswith('AT+RDKPOINT='):
            fields=cmd.split('=')[1];v=list(map(int,fields.split(',')))
            self.run,self.visit=v[:2];self.points.append(v);self.state=1
            lines=['+RDKACK:'+fields,f'+RDKBIND:{self.run},{self.visit},4294967295']
            if self.late: lines += [f'+RDKDONE:{self.run},{self.visit-1},0']
        elif cmd=='AT+RDKSTATE':
            lines=[f'+RDKSTATE:{self.run},{self.visit},{self.state},{self.result},{len(self.points)}']
        elif cmd.startswith('AT+RDKCANCEL='):
            if self.state==2:
                self.buffer.extend(b'ERROR\r\n');return len(data)
            self.state=3;lines=[f'+RDKCANCELED:{self.run},{self.visit}']
        self.buffer.extend(('\r\n'.join(lines+['OK'])+'\r\n').encode());return len(data)
    def read(self,n):
        b=self.buffer[:n];del self.buffer[:n];return b
    def flush(self): pass
    def close(self): pass


def wait_for(check):
    deadline=time.monotonic()+3
    while time.monotonic()<deadline:
        if check(): return
        time.sleep(.01)
    raise AssertionError('worker did not reach expected state')


class CollectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.grid=MapGrid(ROOT/'assets')
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.device=SerialDevice()
        self.c=Collection(self.tmp.name,self.grid,serial_factory=lambda *a,**k:self.device)
        self.addCleanup(self.c.close)
        self.c.configure('ws63','/dev/ttyUSB_TEST')
        wait_for(lambda:self.c.snapshot()['stage']=='ready')
        self.p={'id':'point-a','name':'A','x':.4,'y':-2.,'yaw':0,'wait':0,'enabled':True}
        self.health={'pose':[.41,-2.01,.01],'ready':True,'localization_confirmed':True,'stationary':True}
        self.c.prepare([self.p])
    def start_sampling(self):
        self.c.begin(self.p,self.health);wait_for(lambda:self.c.snapshot()['stage']=='sampling')
    def test_real_done_not_dwell_timer_and_metadata(self):
        self.start_sampling();self.assertEqual(self.device.points[0][-3:],[410,-2010,573])
        self.device.state=2;wait_for(lambda:self.c.snapshot()['stage']=='done')
        journal=(Path(self.tmp.name)/'collection/journal.jsonl').read_text()
        self.assertIn('4294967295',journal);self.assertIn(self.grid.map_id,journal)
        self.assertTrue((Path(self.tmp.name)/'collection/maps'/self.grid.map_id/'map.yaml').exists())
    def test_old_done_does_not_release_next_visit(self):
        self.start_sampling();self.device.state=2;wait_for(lambda:self.c.snapshot()['stage']=='done')
        self.device.late=True;self.c.begin(self.p,self.health)
        wait_for(lambda:len(self.device.points)==2)
        time.sleep(.6);self.assertEqual(self.c.snapshot()['stage'],'sampling')
    def test_invalid_quality_holds_and_cancel_is_acknowledged(self):
        self.start_sampling();self.device.result=1;self.device.state=2
        wait_for(lambda:self.c.snapshot()['stage']=='error')
        self.c.cancel();wait_for(lambda:self.c.snapshot()['stage']=='canceled')
    def test_no_actual_pose_or_moving_must_not_submit(self):
        for key,value in [('pose',None),('stationary',False),('localization_confirmed',False)]:
            with self.assertRaises(ValueError):self.c.begin(self.p,{**self.health,key:value})
        self.assertEqual(self.device.points,[])
    def test_device_reset_does_not_release_point(self):
        self.start_sampling();self.device.run+=1
        wait_for(lambda:self.c.snapshot()['stage']=='error')
        self.assertIn('会话消失',self.c.snapshot()['error'])
    def test_export_joins_scan_to_actual_pose_and_saved_map(self):
        self.start_sampling()
        self.c._line_log('[scan_csv] 4294967295,0,1,2,3,4,5,5000,1000,10,1600,20,-60,-70,-50,OK')
        data=self.c.export()
        csv=merged_csv(data).decode('utf-8-sig')
        self.assertIn('point-a',csv);self.assertIn('0.41,-2.01,0.01',csv)
        with zipfile.ZipFile(io.BytesIO(self.c.export(packed=True))) as z:
            self.assertIn('rssi_with_positions.csv',z.namelist())
            self.assertIn('maps/'+self.grid.map_id+'/map.yaml',z.namelist())
    def test_unbound_measurement_is_not_exported_as_a_valid_station(self):
        self.c._log('device',session=self.c.session,line='[scan_csv] 99,0,1,2,3,4,5,5000,1000,10,1600,20,-60,-70,-50,OK')
        with self.assertRaises(ValueError):merged_csv(self.c.export())

    def test_old_mock_firmware_rejected(self):
        old=self.device.write
        def mock(data):
            if data==b'AT+RDKHELLO\r\n':
                self.device.buffer.extend(b'+RDKHELLO:1,COMMTEST\r\nOK\r\n');return len(data)
            return old(data)
        self.device.write=mock;self.c.configure('ws63','/dev/ttyUSB_TEST')
        wait_for(lambda:self.c.snapshot()['stage']=='error')
        self.assertFalse(self.c.snapshot()['connected'])


class MissionCollectionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.backend=ControlledBackend();self.clock=Clock()
        self.m=Mission(MapGrid(ROOT/'assets'),self.backend,self.tmp.name,self.clock)
        self.addCleanup(self.m.collection.close)
        self.status={'mode':'ws63','stage':'ready','connected':True};self.begins=[]
        self.m.collection.snapshot=lambda:dict(self.status)
        self.m.collection.prepare=lambda points:None
        self.m.collection.begin=lambda p,h:(self.begins.append((p,h)),self.status.update(stage='sampling'))
        self.m.collection.cancel=lambda:self.status.update(stage='canceling')
        self.m.save_points([{'id':v,'name':v,'x':.4,'y':y,'yaw':0,'wait':0,'enabled':True} for v,y in [('a',-2.),('b',-3.)]])
    def arrive(self):
        self.backend.ops[self.m.operation]['status']='succeeded';self.m.tick()
    def test_nav_success_without_stop_cannot_start_collection(self):
        self.m.start();self.backend.stationary=False;self.arrive()
        self.assertEqual(self.begins,[]);self.backend.stationary=True;self.m.tick()
        self.assertEqual(len(self.begins),1)
        self.clock.advance(10000);self.m.tick();self.assertEqual(len(self.backend.calls),1)
        self.status['stage']='done';self.m.tick();self.assertEqual(len(self.backend.calls),2)
    def test_pause_keeps_done_without_departing(self):
        self.m.start();self.arrive();self.m.pause();self.status['stage']='done';self.m.tick()
        self.assertEqual(len(self.backend.calls),1);self.m.resume();self.m.tick()
        self.assertEqual(len(self.backend.calls),2)
    def test_return_waits_for_serial_cancel(self):
        self.m.start();self.arrive();self.m.return_to_dock();self.m.tick()
        self.assertEqual(self.m.phase,'canceling_collection');self.assertEqual(len(self.backend.calls),1)
        self.status['stage']='canceled';self.m.tick();self.assertEqual(self.backend.calls[-1][1],'return')
    def test_motion_during_collection_invalidates_without_next_point(self):
        self.m.start();self.arrive();self.backend.stationary=False;self.m.tick()
        self.assertEqual(self.m.phase,'failed');self.assertEqual(len(self.backend.calls),1)


if __name__=='__main__': unittest.main()
