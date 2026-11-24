import math
from pathlib import Path
from random import Random
import sys
from unittest import TestCase, skipIf

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from map_grid import MapGrid

try:
    from scipy.ndimage import distance_transform_edt
    from stationary_alignment import ScanWallAgreement, StationaryAligner
except ImportError:
    StationaryAligner=None


@skipIf(StationaryAligner is None,'NumPy and SciPy are required for stationary alignment')
class StationaryAlignmentTests(TestCase):
    def test_fast_scan_wall_score_matches_original_per_point_map_lookup(self):
        grid=MapGrid(Path(__file__).resolve().parents[2]/'scenes/demo-room')
        distances=distance_transform_edt([[grid.value(ix,iy)<65 for ix in range(grid.width)]
                                          for iy in range(grid.height)])*grid.resolution
        fast=ScanWallAgreement(grid)
        rng=Random(37)
        points=[(rng.uniform(grid.origin[0]-.3,grid.max_x+.3),rng.uniform(grid.origin[1]-.3,grid.max_y+.3))
                for _ in range(4000)]
        points.extend((grid.origin[0]+ix*grid.resolution,grid.origin[1]+iy*grid.resolution)
                      for ix,iy in ((0,0),(1,2),(2,1),(grid.width,grid.height)))
        known=[distances[iy,ix]<=.15 for ix,iy in map(lambda point:grid.cell_xy(*point),points)
               if grid.value(ix,iy)>=0]
        self.assertEqual(fast.score(points),(len(known),sum(known)/len(known)))
        self.assertEqual(fast.score([(grid.max_x+1.,grid.max_y+1.)]),(0,None))
        self.assertEqual(fast.score([]),(0,None))

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
