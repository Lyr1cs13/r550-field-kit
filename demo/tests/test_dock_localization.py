import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dock_localization import DockLocalization, localization_inputs_ready


class DockLocalizationTests(unittest.TestCase):
    def test_cold_start_initialization_does_not_wait_for_costmap_tf(self):
        states={'amcl':(3,99.),'map_server':(3,99.),'controller_server':(1,99.)}
        self.assertTrue(localization_inputs_ready(100.,map_matches=True,mapping=False,
            last_scan=99.8,lifecycle=states))
        self.assertFalse(localization_inputs_ready(100.,map_matches=True,mapping=False,
            last_scan=90.,lifecycle=states))
        states['amcl']=(2,99.)
        self.assertFalse(localization_inputs_ready(100.,map_matches=True,mapping=False,
            last_scan=99.8,lifecycle=states))

    def setUp(self):
        self.p = DockLocalization([0., 0., 0.])
        self.h = dict(charging=True, stationary=True, sensors_ready=True, busy=False,
                      pose=[0., 0., 0.], amcl_updated=True, pose_request_pending=False)

    def step(self, t, **overrides):
        return self.p.step(t, **(self.h | overrides))

    def test_requires_stable_charge_and_stopped_wheels(self):
        self.assertIsNone(self.step(0))
        self.assertIsNone(self.step(2))
        self.assertIsNone(self.step(3, stationary=False))
        self.assertIsNone(self.step(4))
        self.assertEqual(self.step(7), 'initialize')
        self.assertIsNone(self.step(7.2, pose_request_pending=True))
        self.assertIsNone(self.step(7.5))
        self.assertEqual(self.step(8.6), 'confirm')
        self.assertEqual(self.p.count, 1)
        for t in (9, 15, 100):
            self.assertIsNone(self.step(t))

    def test_no_initialization_during_motion_or_missing_sensors(self):
        for t in (0, 5, 10):
            self.assertIsNone(self.step(t, busy=True))
            self.assertIsNone(self.step(t, sensors_ready=False))
        self.assertEqual(self.p.count, 0)

    def test_side_contact_does_not_force_pose_to_dock_reference(self):
        for t in (0,4,10):
            self.assertIsNone(self.step(t,pose=[0.,.03,.14]))
        self.assertEqual(self.p.status,'alignment_required');self.assertEqual(self.p.count,0)
        self.assertIsNone(self.step(12,pose=[.01,.01,.02]))
        self.assertEqual(self.step(15,pose=[.01,.01,.02]),'initialize')

    def test_failed_amcl_confirmation_does_not_fake_success(self):
        self.step(0)
        self.assertEqual(self.step(3), 'initialize')
        for t in (4, 8, 19):
            self.assertIsNone(self.step(t, pose=[1., 0., 0.]))
        self.assertEqual(self.p.status, 'attention')
        self.assertEqual(self.p.count, 0)
        self.assertEqual(self.p.reference, (0., 0., 0.))

    def test_fresh_amcl_update_is_required(self):
        self.step(0); self.step(3)
        for t in (4, 6, 8):
            self.assertIsNone(self.step(t, amcl_updated=False))
        self.assertEqual(self.p.count, 0)

    def test_manual_pose_is_never_overwritten(self):
        self.p.suppress()
        for t in (0, 5, 10):
            self.assertIsNone(self.step(t))
        self.assertEqual(self.p.status, 'manual')

    def test_next_visit_rearms_after_charge_disappears(self):
        self.step(0); self.step(3); self.step(4); self.step(5)
        self.assertEqual(self.p.count, 1)
        self.step(6, charging=False)
        self.step(11, charging=False)
        self.step(12)
        self.assertEqual(self.step(15), 'initialize')
        self.step(16)
        self.assertEqual(self.step(17), 'confirm')
        self.assertEqual(self.p.count, 2)

    def test_manual_dock_initialization_is_not_sent_twice(self):
        self.p.manual_dock(0)
        self.assertIsNone(self.step(0))
        self.assertIsNone(self.step(3))
        self.assertEqual(self.step(4.1), 'confirm')


if __name__ == '__main__':
    unittest.main()
