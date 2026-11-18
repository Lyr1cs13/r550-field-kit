"""Check that encoder travel corresponds to motion seen by the laser."""
import math

import numpy as np
from scipy.spatial import cKDTree

from directed_route import angle_error


def scan_features(scan,range_ratio=1.):
    step=max(1,len(scan.ranges)//260)
    distances=np.asarray(scan.ranges[::step],dtype=float)
    angles=scan.angle_min+np.arange(len(distances))*step*scan.angle_increment
    valid=(np.isfinite(distances)&(distances>=max(.25,scan.range_min))
           &(distances<=min(5.,scan.range_max*range_ratio)))
    points=np.column_stack((np.cos(angles),np.sin(angles)))*np.where(valid,distances,0.)[:,None]
    tangents=points[2:]-points[:-2]
    lengths=np.linalg.norm(tangents,axis=1)
    adjacent=(valid[:-2]&valid[1:-1]&valid[2:]&(lengths>.012)&(lengths<.28)
              &(np.linalg.norm(points[1:-1]-points[:-2],axis=1)<.17)
              &(np.linalg.norm(points[2:]-points[1:-1],axis=1)<.17))
    tangents=tangents[adjacent]/lengths[adjacent,None]
    normals=np.column_stack((-tangents[:,1],tangents[:,0]))
    return points[valid],points[1:-1][adjacent],normals


def relative_motion(reference, current_points):
    locations,normals=reference
    if min(len(locations),len(current_points))<65:
        return None
    tree=cKDTree(locations)
    estimate=np.zeros(3)
    for _ in range(5):
        c,s=math.cos(estimate[2]),math.sin(estimate[2])
        rotated=current_points@np.array(((c,s),(-s,c)))
        moved=rotated+estimate[:2]
        distances,index=tree.query(moved)
        usable=distances<.19
        if np.count_nonzero(usable)<max(65,.60*len(current_points)):
            return None
        points=rotated[usable];near=locations[index[usable]];directions=normals[index[usable]]
        error=np.sum((moved[usable]-near)*directions,axis=1)
        inliers=np.abs(error)<=max(.018,float(np.quantile(np.abs(error),.82)))
        points,directions,error=points[inliers],directions[inliers],error[inliers]
        if len(error)<60 or np.linalg.eigvalsh(directions.T@directions/len(directions))[0]<.085:
            return None
        rotation=np.sum(points[:,[1,0]]*np.array((-1.,1.))*directions,axis=1)
        delta,_,rank,_=np.linalg.lstsq(np.column_stack((directions,rotation)),-error,rcond=None)
        if rank!=3:
            return None
        estimate+=delta
        if np.linalg.norm(estimate[:2])>.30 or abs(estimate[2])>math.radians(13):
            return None
        if np.linalg.norm(delta)<.002:
            return tuple(estimate) if np.median(np.abs(error))<.019 else None
    return None


class SlipMonitor:
    def __init__(self,range_ratio=1.):
        self.range_ratio=range_ratio
        self.reset()

    def reset(self):
        self.reference=None
        self.last_stamp=-math.inf
        self.stationary_since=None

    def observe(self, stamp, encoders, pose, scan):
        if stamp<=self.last_stamp:
            return None
        self.last_stamp=stamp
        if self.reference is not None:
            then,wheel,anchor,features=self.reference
            travel=math.dist(wheel[:2],encoders[:2])
            rotation=abs(angle_error(wheel[2],encoders[2]))
            if rotation<math.radians(13) and travel<.27 and stamp-then<4.:
                if (travel<.075 and rotation<math.radians(7)) or stamp-then<.45:
                    return None
                points,locations,normals=scan_features(scan,self.range_ratio)
                motion=relative_motion(features,points)
                if (motion is not None and math.hypot(*motion[:2])<.022
                        and abs(motion[2])<math.radians(3)):
                    if self.stationary_since is None:
                        self.stationary_since=stamp
                    if stamp-self.stationary_since>=.22:
                        return anchor
                    return None
                self.reference=stamp,encoders,tuple(pose),(locations,normals)
                self.stationary_since=None
                return None
        _,locations,normals=scan_features(scan,self.range_ratio)
        self.reference=stamp,encoders,tuple(pose),(locations,normals)
        self.stationary_since=None
        return None
