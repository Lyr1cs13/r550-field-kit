import importlib.util
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch
import yaml

HERE=Path(__file__).resolve().parents[1];ROOT=HERE.parent
sys.path.insert(0,str(HERE))
from map_grid import MapGrid
from scene_settings import settings,contains
from broadcast_survey import BroadcastSurvey
from mission import Mission
from test_demo import ControlledBackend

spec=importlib.util.spec_from_file_location('portable_scene_cli',ROOT/'tools/scene.py')
scene=importlib.util.module_from_spec(spec);spec.loader.exec_module(scene)


class PortableSceneTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);(self.root/'scenes').mkdir();(self.root/'demo/assets').mkdir(parents=True)
        self.previous=scene.ROOT;scene.ROOT=self.root;self.addCleanup(setattr,scene,'ROOT',self.previous)
        source=self.root/'source';source.mkdir()
        shutil.copy2(HERE/'assets/WHEELTEC.pgm',source/'map.pgm')
        value=yaml.safe_load((HERE/'assets/map.yaml').read_text());value['image']='map.pgm';value['origin']=[100.,200.,0.]
        self.map=source/'map.yaml';self.map.write_text(yaml.safe_dump(value))

    def test_import_activate_uses_new_geometry_and_unset_dock(self):
        directory=scene.create('other-site',self.map);g=MapGrid(directory)
        self.assertGreater(g.origin[0],90)
        self.assertFalse(json.loads((directory/'dock_start.json').read_text())['calibrated'])
        with patch.object(scene,'ensure_stopped'):
            scene.activate('other-site')
        active=json.loads((self.root/'state/active_scene.json').read_text())
        self.assertEqual(active['map_id'],MapGrid(self.root/'demo/assets').map_id)
        self.assertTrue(contains(settings(g)['bounds'],[102,202]))
        self.assertFalse(contains(settings(g)['bounds'],[.3,-2]))

    def test_route_and_rssi_capture_do_not_depend_on_laboratory_coordinates(self):
        directory=scene.create('other-site',self.map)
        file=self.root/'route.json';file.write_text(json.dumps([[[102,202],[104,202]]]))
        scene.route('other-site',file);g=MapGrid(directory)
        self.assertEqual(json.loads((directory/'survey_route.json').read_text())['map_id'],g.map_id)
        b=BroadcastSurvey(self.root/'data',g,'fake',clock=lambda:100.,autostart=False)
        self.addCleanup(b.close);b.moving_demo=True;b.status['connected']=True;b.last_packet=100.
        b.prepare([dict(x=102,y=202,id='a')])
        self.assertEqual(b.bounds,[100.,107.,200.,210.])
        b._drive_sample([102,202,0],-80,100.)
        self.assertEqual(b.points[0]['x'],102)

    def test_mask_changes_map_identity_without_editing_occupancy(self):
        directory=scene.create('other-site',self.map);g=MapGrid(directory);raw=(directory/'WHEELTEC.pgm').read_bytes()
        scene.make_mask(directory,[[102,102.5,202,202.5]])
        new=MapGrid(directory)
        self.assertNotEqual(g.map_id,new.map_id)
        self.assertEqual(raw,(directory/'WHEELTEC.pgm').read_bytes())
        with self.assertRaises(ValueError):new.validate(102.25,202.25)

    def test_no_dock_return_until_scene_calibration(self):
        g=MapGrid(HERE/'assets');backend=ControlledBackend();backend.dock_calibrated=False
        m=Mission(g,backend,self.root/'mission');self.addCleanup(m.collection.close)
        with self.assertRaisesRegex(ValueError,'标定'):m.return_to_dock()
        self.assertEqual(backend.calls,[])

    def test_scene_name_cannot_escape_directory(self):
        with self.assertRaises(ValueError):scene.scene_dir('../other')

    def test_network_profile_has_peer_and_domain_without_secrets(self):
        spec=importlib.util.spec_from_file_location('portable_network',ROOT/'tools/network.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        module.configure('wlan0','192.168.0.250',42,self.root/'network')
        text=(self.root/'network/network.sh').read_text()
        self.assertIn('ROS_DOMAIN_ID=42',text)
        self.assertIn('192.168.0.250',(self.root/'network/network.xml').read_text())


if __name__=='__main__':unittest.main()
