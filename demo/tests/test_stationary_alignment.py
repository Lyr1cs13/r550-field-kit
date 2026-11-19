import math
from pathlib import Path
import sys
from unittest import TestCase, skipIf

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from map_grid import MapGrid

try:
    from stationary_alignment import StationaryAligner
except ImportError:
    StationaryAligner=None


@skipIf(StationaryAligner is None,'NumPy and SciPy are required for stationary alignment')
class StationaryAlignmentTests(TestCase):
    def test_fixed_edges_keep_pose_when_a_sector_of_returns_changes(self):
        grid=MapGrid(Path(__file__).resolve().parents[2]/'scenes/demo-room')
        x,y,yaw=1.2,-1.25,.4
        angle_step=math.tau/720
        ranges=[]
        for index in range(720):
            angle=-math.pi+angle_step*index+yaw
            c,s=math.cos(angle),math.sin(angle)
            distance=None
            for sample in range(8,390):
                radius=.018*sample
                cell=grid.value(*grid.cell_xy(x+radius*c,y+radius*s))
                if cell==100:
                    distance=radius
                if cell!=0:
                    break
            ranges.append(distance)
        for start,end,distance in ((25,160,.47),(253,329,.31),(381,453,.48),(550,620,.24)):
            ranges[start:end]=[distance]*(end-start)
        frame={'ranges':ranges,'angle_min':-math.pi,'angle_increment':angle_step,'range_min':.05,'range_max':8.}
        solver=StationaryAligner(grid)
        result=solver.fit((x+.16,y-.13,yaw+.13),[frame]*6,(0.,0.,0.),use_fixed_edges=True)
        self.assertLess(result['wall_support'],.52)
        self.assertTrue(result['fixed_edges'])
        self.assertLess(math.dist(result['pose'][:2],(x,y)),.07)
        self.assertLess(abs(result['pose'][2]-yaw),math.radians(2))
        points=[(x+distance*math.cos(-math.pi+angle_step*i+yaw),
                 y+distance*math.sin(-math.pi+angle_step*i+yaw))
                for i,distance in enumerate(ranges) if distance is not None]
        self.assertTrue(solver.verifies_tracking(points,(x,y)))
        self.assertFalse(solver.verifies_tracking([(px+.12,py) for px,py in points],(x+.12,y)))
        self.assertFalse(solver.verifies_tracking(points[:90],(x,y)))
        with self.assertRaises(ValueError):
            solver.fit((x,y,yaw),[{**frame,'ranges':[None]*720}]*6,(0.,0.,0.))
