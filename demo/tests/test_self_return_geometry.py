import sys,unittest,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from self_return_geometry import antenna_return,inside_antenna
class SelfReturnTests(unittest.TestCase):
    def test_measured_antenna_envelope(self):
        for x,y in ((-.1429,-.0823),(-.1033,-.0649),(-.120,-.073)):
            self.assertTrue(inside_antenna(x,y))
    def test_real_obstacles_beyond_or_beside_mask_remain(self):
        self.assertFalse(inside_antenna(-.18,-.073))
        self.assertFalse(inside_antenna(-.12,-.12))
        self.assertFalse(inside_antenna(.1,0.))
    def test_same_ray_farther_return_is_not_a_blind_sector(self):
        angle=math.atan2(-.073,-.120-.03163);r=math.hypot(-.120-.03163,-.073)
        self.assertTrue(antenna_return(r,angle,.03163,0.,0.))
        self.assertFalse(antenna_return(1.,angle,.03163,0.,0.))
    def test_transform_and_invalid_returns(self):
        self.assertTrue(antenna_return(math.hypot(.12,.073),math.atan2(.073,.12),0.,0.,math.pi))
        for r in (float('nan'),float('inf')):self.assertFalse(antenna_return(r,0.,0.,0.,0.))
