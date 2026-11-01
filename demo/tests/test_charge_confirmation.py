import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from charge_confirmation import ChargeConfirmation, NativeDockingWatchdog


class ChargeTests(unittest.TestCase):









    def test_current_confirms_charge_even_when_firmware_flag_is_false(self):
        p=ChargeConfirmation()
        self.assertFalse(p.update(0,charging=False,stationary=True,current=.18,voltage=11.8))
        self.assertTrue(p.update(2,charging=False,stationary=True,current=.18,voltage=11.8))
        self.assertFalse(p.update(3,charging=False,stationary=False,current=.18,voltage=11.8))



    def test_contact_bounce_does_not_finish_docking(self):
        p=ChargeConfirmation()
        def step(t,flag=True,current=1.6,stationary=True):
            return p.update(t,charging=flag,stationary=stationary,current=current,voltage=11.4)
        self.assertFalse(step(0));self.assertFalse(step(.1,False,0))
        self.assertFalse(step(1));self.assertFalse(step(2.9))
        self.assertTrue(step(3));self.assertFalse(step(3.1,False,0))

    def test_flag_without_current_is_not_charging(self):
        p=ChargeConfirmation()
        for t in (0,3,10):
            self.assertFalse(p.update(t,charging=True,stationary=True,current=0.,voltage=11.))

    def test_full_battery_still_requires_flag_and_stopped_wheels(self):
        p=ChargeConfirmation()
        for t in (0,3):self.assertFalse(p.update(t,charging=False,stationary=True,current=0.,voltage=12.6))
        self.assertFalse(p.update(4,charging=True,stationary=True,current=0.,voltage=12.6))
        self.assertTrue(p.update(6,charging=True,stationary=True,current=0.,voltage=12.6))
        self.assertFalse(p.update(7,charging=True,stationary=False,current=1.,voltage=12.6))


class NativeWatchdogTests(unittest.TestCase):
    def test_healthy_native_capture_is_uninterrupted(self):
        g=NativeDockingWatchdog();g.reset(10.,2.)
        for t,d in ((10.,2.),(12.,2.02),(14.,2.06)):
            self.assertIsNone(g.update(t,travel=d,near=True,fresh=True))

    def test_encoder_drift_cannot_reset_near_dock_budget(self):
        g=NativeDockingWatchdog();g.reset(0.,0.)
        self.assertIsNone(g.update(0.,travel=0.,near=True,fresh=True))
        self.assertIsNone(g.update(3.,travel=.201,near=False,fresh=True))
        self.assertIsNotNone(g.update(4.,travel=.281,near=False,fresh=True))

    def test_stalled_wheels_are_bounded_by_time(self):
        g=NativeDockingWatchdog();g.reset(0.,0.)
        g.update(0.,travel=0.,near=True,fresh=True)
        self.assertIsNone(g.update(12.,travel=.20,near=True,fresh=True))
        self.assertIsNotNone(g.update(20.,travel=.20,near=True,fresh=True))

    def test_bad_pose_cannot_remove_total_budget(self):
        for t,d in ((45.,0.),(2.,1.5)):
            g=NativeDockingWatchdog();g.reset(0.,0.)
            self.assertIsNotNone(g.update(t,travel=d,near=False,fresh=True))

    def test_stale_feedback_disables_native_motion(self):
        g=NativeDockingWatchdog()
        self.assertIsNotNone(g.update(1.,travel=0.,near=False,fresh=False))
