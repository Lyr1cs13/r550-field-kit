import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
import workspace_guard as guard


class WorkspaceTests(unittest.TestCase):
    def test_release_requires_idle_stationary_and_no_pending_resumption(self):
        state = {'phase': 'docked', 'health': {'stationary': True}}
        guard.check_quiet(state)
        for changed in ({'phase':'returning'}, {'health':{'stationary':False}},
                        {'fas':{'lease_live':True}}, {'recharge_resume_pending':True}):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                guard.check_quiet(dict(state, **changed))

    @unittest.skipUnless(sys.platform.startswith('linux'), 'host uses Linux file locks')
    def test_handover_and_commands_from_both_entry_points(self):
        import fcntl
        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp)
            patrol, vision = home/'patrol', home/'vision'
            directory = home/'.local/state/r550-workspaces'
            for root in (patrol, vision):
                (root/'state').mkdir(parents=True)
                (root/'state/workspace.env').touch()
            guard.dispatch(patrol,directory,'select',[])
            with self.assertRaises(ValueError):
                guard.dispatch(vision,directory,'select',[])
            with patch.object(guard,'run_kit') as start:
                with self.assertRaises(ValueError):
                    guard.dispatch(vision,directory,'run',['stop'])
                start.assert_not_called()
            env = dict(HOME=str(home), R550_KIT_ROOT=str(patrol),
                       ROS_DOMAIN_ID='41', R550_WORKSPACE_DOMAIN='41')
            with patch.dict(os.environ,env):
                with guard.hardware_command(patrol):
                    with self.assertRaises(ValueError):
                        guard.dispatch(patrol,directory,'release',[])
                with patch.dict(os.environ,dict(R550_KIT_ROOT=str(vision),ROS_DOMAIN_ID='42',R550_WORKSPACE_DOMAIN='42')):
                    with self.assertRaises(ValueError),guard.hardware_command(vision,stop=True):
                        self.fail('inactive workspace accepted a hardware stop')
                with patch.dict(os.environ,dict(ROS_DOMAIN_ID='0')):
                    with self.assertRaises(ValueError),guard.hardware_command(patrol):
                        self.fail('wrong ROS domain accepted')
            with patch.object(guard,'release',side_effect=ValueError('busy')),self.assertRaises(ValueError):
                guard.dispatch(patrol,directory,'release',[])
            self.assertEqual((directory/'owner').read_text().strip(),str(patrol))
            with patch.object(guard,'release'):
                guard.dispatch(patrol,directory,'release',[])
            with (vision/'state/deploy.lock').open('a') as updating:
                fcntl.flock(updating,fcntl.LOCK_EX)
                with self.assertRaises(ValueError):
                    guard.dispatch(vision,directory,'select',[])
                guard.dispatch(patrol,directory,'select',[])
                with patch.object(guard,'release'):
                    guard.dispatch(patrol,directory,'release',[])
            guard.dispatch(vision,directory,'select',[])
            self.assertEqual((directory/'owner').read_text().strip(),str(vision))
