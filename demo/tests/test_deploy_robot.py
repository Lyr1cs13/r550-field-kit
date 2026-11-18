"""Deployment applies to a temporary directory only; never contacts a vehicle."""
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
import zipfile

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from deploy_robot import prepare, apply_archive, SCENE_FILES


class DeploymentTests(unittest.TestCase):
    def test_keep_scene_data_and_local_configuration_and_back_up_source(self):
        with tempfile.TemporaryDirectory() as directory:
            base=Path(directory);target=base/'robot';archive=base/'source.zip'
            assets=target/'demo/assets';shutil.copytree(ROOT/'scenes/demo-room',assets)
            (target/'config').mkdir();(target/'config/vehicle.yaml').write_text('local vehicle settings')
            (target/'config/endurance.yaml').write_text('local battery settings')
            (target/'demo/data/ros').mkdir(parents=True)
            saved=target/'demo/data/ros/waypoints.json';saved.write_text('keep existing waypoints')
            (target/'demo/server.py').write_text('old source')
            before={p.name:p.read_bytes() for p in assets.iterdir() if p.name in SCENE_FILES}
            manifest=prepare(ROOT,archive)
            self.assertFalse(any(n.startswith('demo/data/') for n in manifest))
            self.assertNotIn('demo/assets/map.yaml',manifest)
            apply_archive(archive,target)
            self.assertEqual(saved.read_text(),'keep existing waypoints')
            self.assertEqual((target/'config/vehicle.yaml').read_text(),'local vehicle settings')
            self.assertEqual((target/'config/endurance.yaml').read_text(),'local battery settings')
            self.assertEqual(before,{name:(assets/name).read_bytes() for name in before})
            report=json.loads((target/'state/deployed_source.json').read_text())
            self.assertEqual((Path(report['backup'])/'demo/server.py').read_text(),'old source')
            active=json.loads((target/'state/active_scene.json').read_text())
            self.assertEqual(active['map_id'],report['preserved_map_id'])
            self.assertTrue((target/'vendor/nav2_controller/package.xml').exists())

    def test_bad_digest_rejected_before_replacing_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);archive=root/'bad.zip';target=root/'robot'
            with zipfile.ZipFile(archive,'w') as z:
                z.writestr('README.md','corrupted')
                z.writestr('DEPLOY_MANIFEST.json',json.dumps({'README.md':hashlib.sha256(b'expected').hexdigest()}))
            with self.assertRaises(ValueError):apply_archive(archive,target)
            self.assertFalse(target.exists())

    def test_path_traversal_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);archive=root/'bad.zip'
            with zipfile.ZipFile(archive,'w') as z:
                z.writestr('../outside','x')
                z.writestr('DEPLOY_MANIFEST.json',json.dumps({'../outside':hashlib.sha256(b'x').hexdigest()}))
            with self.assertRaises(ValueError):apply_archive(archive,root/'robot')
            self.assertFalse((root/'outside').exists())
