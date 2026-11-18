"""New-site acceptance with isolated storage and a simulated mapping backend."""
from contextlib import contextmanager
import hashlib
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import shutil
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'rrt_mapping'))
from site_core import GridFrame, ManualDrive, dock_record, inside, rectangle
from site_service import SiteService, atomic_json, handler, scene, wait_for


def checkpoint(root,folder):
    folder.mkdir(parents=True)
    source=root/'scenes/demo-room'
    for name in ('WHEELTEC.yaml','WHEELTEC.pgm','dock_start.json','threshold_keepout.yaml','threshold_keepout.pgm'):
        shutil.copyfile(source/name,folder/name)
    for name in ('slam_graph.posegraph','slam_graph.data','native_slam_params.yaml'):
        (folder/name).write_text('synthetic')
    hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir() if p.is_file()}
    atomic_json(folder/'manifest.json',{'graph_prefix':'slam_graph','sha256':hashes})


class FakeNode:
    def __init__(self):
        self.verified=True
        self.fault=''
        self.manual_mode=False
        self.drive_session='test-session'
        self.stationary=lambda t:True
        self.manual=ManualDrive()
        self.mode='pause'
        self.busy=False
        self.frame=GridFrame([0]*48,8,6,.05,(-.06,-.06))
        self.rectangle=None
        self.actions=[]

    def begin(self,mode):
        self.actions.append(mode)
        self.mode='pause' if mode=='manual' else mode
        self.manual_mode=mode=='manual'

    def halt(self):
        self.actions.append('stop')
        self.mode='pause'
        self.manual_mode=False
        self.manual.clear()

    def view(self):
        return {'phase':self.mode,'map':self.frame.view()}


class SiteMappingTests(unittest.TestCase):
    def setUp(self):
        temporary=tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root=Path(temporary.name)
        (self.root/'scenes').mkdir()
        (self.root/'demo/assets').mkdir(parents=True)
        self.node=FakeNode()
        self.writes=[]
        self.site=SiteService(self.root)
        self.site.ros=SimpleNamespace(node=self.node,write=self.writes.append,lock=threading.RLock(),
                                      done=threading.Event(),close=lambda:None)
        self.site.draft={'id':'site-test','name':'测试场地','checkpoint':None,'bounds':None}

    def action(self,action,**fields):
        self.site.dispatch({'action':action,**fields})
        wait_for(lambda:not self.site.job,3.,'job did not finish')

    def test_manual_drive_has_one_owner_and_rejects_packets_ordered_before_release(self):
        drive=ManualDrive()
        drive.update(1.,'owner-a','forward',1)
        self.assertEqual(drive.output(1.1),(.10,0.))
        with self.assertRaises(ValueError):
            drive.update(1.1,'owner-b','left',1)
        drive.update(1.1,'owner-a','stop',3)
        drive.update(1.2,'owner-a','forward',2)
        self.assertEqual(drive.output(1.2),(0.,0.))
        drive.update(1.2,'owner-a','forward',4)
        self.assertEqual(drive.output(1.51),(0.,0.))

    def test_manual_explore_and_return_use_current_unfinished_map(self):
        self.action('manual')
        self.site.dispatch(dict(action='drive',owner='abcdef0123',direction='forward',
                                session='test-session',sequence=1))
        self.assertEqual(self.node.manual.velocity,(.10,0.))
        self.action('stop')
        self.action('bounds',bounds=[-1.,4.,-2.,4.])
        self.action('explore')
        self.action('return')
        self.assertEqual(self.node.actions,['manual','stop','stop','explore','stop','return'])
        self.assertEqual(self.writes,[True,False,True,False])
        self.assertIsNone(self.site.draft['checkpoint'])

    def test_stop_during_a_mode_switch_revokes_pending_motion(self):
        with self.site.lock:
            self.site.epoch=2
        self.site.driving('return',1)
        self.assertNotIn('return',self.node.actions)

    def test_dock_calibration_requires_confirmation_and_reuses_saved_graph(self):
        with self.assertRaisesRegex(ValueError,'确认'):
            self.site.dispatch({'action':'restore','seated':False})
        self.site.draft={**self.site.draft,'checkpoint':'snapshot'}
        checkpoint(ROOT,self.site.checkpoint())
        with (patch.object(self.site,'take_over'),
              patch.object(self.site,'launch_mapping') as launch,
              patch.object(self.site,'started') as start):
            self.action('restore',seated=True)
        launch.assert_called_once_with(self.site.checkpoint())
        start.assert_called_once_with(True)
        self.assertIn('校准',self.site.message)

    def test_export_import_keeps_the_dock_and_does_not_change_current_map(self):
        self.site.draft={**self.site.draft,'checkpoint':'snapshot'}
        checkpoint(ROOT,self.site.checkpoint())
        with patch.object(scene,'ROOT',self.root):
            self.action('export')
        destination=self.root/'scenes/site-test'
        self.assertTrue((destination/'site.json').is_file(),self.site.message)
        self.assertEqual((destination/'dock_start.json').read_bytes(),
                         (self.site.checkpoint()/'dock_start.json').read_bytes())
        self.assertFalse((self.root/'state/active_scene.json').exists())

    def test_activate_uses_the_saved_map_and_releases_mapping_ownership(self):
        self.site.ros=None
        self.site.inhibit.touch()
        self.site.draft={**self.site.draft,'checkpoint':'snapshot'}
        checkpoint(ROOT,self.site.checkpoint())
        with (patch.object(scene,'ROOT',self.root),
              patch.object(scene,'ensure_stopped'),
              patch.object(self.site,'nav_status',return_value=None),
              patch('site_service.subprocess.run') as launch):
            self.site.export_scene()
            self.site.activate('site-test')
        active=json.loads((self.root/'state/active_scene.json').read_text())
        self.assertEqual(active['scene'],'site-test')
        self.assertFalse(self.site.inhibit.exists())
        self.assertEqual(launch.call_args.args[0][-1],'nav')

    def test_unknown_frontier_goals_remain_in_free_space_and_inside_the_boundary(self):
        data=np.full((100,100),-1)
        data[20:80,15:72]=0
        data[20:80,14]=100
        grid=GridFrame(data,100,100,.05,(0.,0.))
        bounds=rectangle([.8,3.5,1.0,4.0])
        goals=grid.frontier_candidates(bounds,(1.4,2.1))
        self.assertTrue(goals)
        for x,y in goals:
            self.assertTrue(inside(bounds,(x,y),.20))
            self.assertEqual(data[int(y/.05),int(x/.05)],0)
        self.assertEqual(dock_record((3.,4.,np.pi/2))['charger_approach']['p_y'],4.6)

    def test_api_map_and_control_session_are_separate(self):
        with ThreadingHTTPServer(('127.0.0.1',0),handler(self.site)) as web:
            worker=threading.Thread(target=web.serve_forever,daemon=True)
            worker.start()
            address=f'http://127.0.0.1:{web.server_port}'
            try:
                with urlopen(address+'/api/site/state') as response:
                    state=json.load(response)
                with urlopen(address+'/api/site/map?id='+state['map']['map_id']) as response:
                    self.assertEqual(response.headers['Content-Type'],'image/png')
                request=Request(address+'/api/site/command',data=b'{"action":"return"}',
                                headers={'Content-Type':'application/json'})
                with self.assertRaises(HTTPError) as error:
                    urlopen(request)
                self.assertEqual(error.exception.code,403)
                self.assertEqual(self.node.actions,[])
            finally:
                web.shutdown()


if __name__=='__main__':
    unittest.main()
