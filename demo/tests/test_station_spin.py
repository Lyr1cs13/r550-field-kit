import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from arrival_policy import StationCheck
from motion_health import VelocityLease
from spin_control import ProfiledSpin
from turn_approach import TurnApproach, approach_radius


class StationSpinTests(unittest.TestCase):
    def test_directional_aim_needs_independent_stations_and_respects_reverse_and_xy(self):
        model=TurnApproach()
        def sample(site,degrees,radius=.04):
            angle=math.radians(degrees)
            # Known off-centre pivot on a chassis initially facing map +Y.
            before=(site,0.,math.pi/2)
            after=(site+radius*math.sin(angle),radius*(1-math.cos(angle)),math.pi/2+angle)
            model.observe(site,before,after,angle,math.radians(3),.055)
        goal=dict(x=0.,y=0.,yaw=math.pi)
        for _ in range(8):
            sample(0,90)
        self.assertIsNone(model.aim((0.,-1.),goal,False,.055,.022))
        for n,angle in enumerate((75,100,105),1):
            sample(n,angle)
        a=model.aim((0.,-1.),goal,False,.055,.022)
        b=model.aim((0.,1.),goal,True,.055,.022)
        self.assertLess(a['x'],0.)
        self.assertLessEqual(a['y'],0.)
        self.assertLess(b['x'],0.)
        self.assertGreaterEqual(b['y'],0.)
        for point in (a,b):
            self.assertLessEqual(math.hypot(point['x'],point['y'])+.022,.055+1e-12)
            self.assertEqual(point['yaw'],math.pi)
        self.assertIsNone(model.prediction(-math.pi/2,.055))
        self.assertIsNone(model.prediction(math.radians(12),.055))
        self.assertIsNone(TurnApproach().aim((0.,-1.),goal,False,.055,.022))
        sample(4,90,-.04)
        self.assertIsNone(model.prediction(math.pi/2,.055))

    def test_radius_changes_continuously_and_inconsistent_turns_get_no_offset(self):
        outer,inner=.035,.022
        get=lambda angle:approach_radius(math.radians(angle),math.radians(3),outer,inner)
        self.assertIsNone(get(2.))
        self.assertGreater(get(12.),get(30.))
        self.assertGreater(get(30.),get(45.))
        self.assertAlmostEqual(get(45.),get(-90.))
        self.assertAlmostEqual(get(45.),inner)
        model=TurnApproach()
        for site,drift in enumerate((.04,-.04,.045,-.045)):
            model.observe(site,(0.,0.,0.),(drift,0.,math.pi/2),math.pi/2,math.radians(3),.055)
        self.assertIsNone(model.prediction(math.pi/2,.055))

    def test_both_directions_bias_and_latched_brake(self):
        for sign in (-1,1):
            turn=ProfiledSpin(sign*math.radians(6),12.,.013)
            for frame in range(1,30):
                speed=min(.24,max(0.,(frame-9)*.023))
                if turn.observe(12.+frame*.05,.013+sign*speed):
                    break
                self.assertGreater(turn.signed_limit*sign,0.)
            self.assertTrue(turn.stopped)
            self.assertLess(turn.progress,turn.target)
            self.assertTrue(turn.observe(14.,.013-sign*.02))
            self.assertEqual(turn.signed_limit,0.)

    def test_missing_imu_while_turning_and_wrong_direction(self):
        turn=ProfiledSpin(1.,0.,0.)
        turn.observe(.05,.08)
        self.assertTrue(turn.observe(.28,.08))
        turn=ProfiledSpin(-1.,0.,0.)
        for frame in range(1,11):
            turn.observe(.05*frame,.10)
        self.assertTrue(turn.stopped)

    def test_spin_permission_can_only_clip_recent_collision_checked_pure_yaw(self):
        lease=VelocityLease()
        lease.receive_command(1.,(0.,0.,-.24))
        lease.receive_spin_limit(1.02,5.,0.,-.18)
        self.assertFalse(any(lease.output(1.03)))
        for twist,expected in [((0.,0.,-.24),-.18),((0.,0.,-.04),-.04),((0.,0.,0.),0.),
                               ((.001,0.,-.20),0.),((0.,0.,.20),0.)]:
            lease.receive_command(1.05,twist)
            self.assertEqual(lease.output(1.06),(0.,0.,expected))
        lease.receive_command(1.14,(0.,0.,-.22))
        self.assertFalse(any(lease.output(1.19)))
        lease.receive_spin_limit(1.22,5.2,.2,-.18)
        self.assertFalse(any(lease.output(1.23)))
        lease.receive_spin_limit(1.25,5.25,0.,-.18)
        lease.receive_command(1.26,(0.,0.,-.22))
        lease.receive_permit(1.28,False,5.28)
        lease.receive_spin_limit(1.29,5.26,.03,-.18)
        self.assertFalse(any(lease.output(1.29)))
        lease.receive_permit(1.4,True)
        lease.receive_command(1.41,(.08,0.,0.))
        self.assertEqual(lease.output(1.42),(.08,0.,0.))

    def test_flicker_and_repeated_transforms_cannot_reopen_xy_or_end_station(self):
        c=StationCheck()
        def stage(t,d,a=2.,scan=None):
            return c.observe(t,t if scan is None else scan,d,math.radians(a),.055,math.radians(3))
        self.assertEqual(stage(1.,.054),'wait')
        self.assertEqual(stage(1.1,.058),'wait')
        self.assertEqual(stage(1.4,.058,scan=1.1),'wait')
        self.assertEqual(stage(1.4,.052),'wait')
        self.assertEqual(stage(1.5,.051),'wait')
        self.assertEqual(stage(1.6,.051),'complete')
        c=StationCheck()
        for i in range(8):
            self.assertEqual(stage(2.,.06),'wait')
        self.assertEqual(stage(2.14,.06),'wait')
        self.assertEqual(stage(2.28,.06),'position')
        c=StationCheck()
        for i in range(4):
            decision=stage(3.+i*.1,.052,-4.)
        self.assertEqual(decision,'heading')
