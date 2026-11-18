"""Laser/encoder agreement in a room and in a corridor with no axial landmarks."""
from importlib.util import find_spec
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def scan(x, *, corridor=False):
    width=30. if corridor else 3.
    increment=2*math.pi/720
    ranges=[]
    for index in range(720):
        angle=-math.pi+index*increment
        dx,dy=math.cos(angle),math.sin(angle)
        a=(math.copysign(width,dx)-x)/dx if abs(dx)>1e-8 else math.inf
        b=1.3/abs(dy) if abs(dy)>1e-8 else math.inf
        distance=min(a,b)
        ranges.append(distance if distance<5. else math.inf)
    return SimpleNamespace(ranges=ranges,angle_min=-math.pi,angle_increment=increment,range_min=.15,range_max=12.)


@unittest.skipUnless(find_spec('numpy') and find_spec('scipy'),'Requires laser matching dependencies')
class SlipTests(unittest.TestCase):
    def test_nonmoving_laser_with_encoder_progress_trips_after_consistency_window(self):
        from slip_monitor import SlipMonitor
        monitor=SlipMonitor()
        still=scan(0.)
        self.assertIsNone(monitor.observe(1.,(0.,0.,0.),(1.,2.,0.),still))
        self.assertIsNone(monitor.observe(1.7,(.09,0.,0.),(1.09,2.,0.),still))
        self.assertIsNone(monitor.observe(1.85,(.11,0.,0.),(1.11,2.,0.),still))
        self.assertEqual(monitor.observe(1.98,(.13,0.,0.),(1.13,2.,0.),still),(1.,2.,0.))

    def test_laser_can_flag_wheels_turning_in_place_without_rotation(self):
        from slip_monitor import SlipMonitor
        monitor=SlipMonitor();still=scan(0.)
        monitor.observe(1.,(0.,0.,0.),(1.,2.,0.),still)
        self.assertIsNone(monitor.observe(1.7,(0.,0.,math.radians(8)),(1.,2.,.14),still))
        self.assertEqual(monitor.observe(1.98,(0.,0.,math.radians(9)),(1.,2.,.16),still),(1.,2.,0.))

    def test_agreement_and_an_unobservable_corridor_do_not_flag_slip(self):
        from slip_monitor import SlipMonitor, relative_motion, scan_features
        _,locations,normals=scan_features(scan(0.))
        motion=relative_motion((locations,normals),scan_features(scan(.12))[0])
        self.assertIsNotNone(motion)
        self.assertAlmostEqual(motion[0],.12,places=2)
        for corridor in (False,True):
            monitor=SlipMonitor()
            for frame in range(15):
                x=frame*.02
                self.assertIsNone(monitor.observe(frame*.20,(x,0.,0.),(x,0.,0.),scan(x,corridor=corridor)))
