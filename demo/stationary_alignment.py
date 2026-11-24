"""Stationary alignment against the existing occupancy grid."""
from itertools import product
import math
import warnings

import numpy as np
from scipy.ndimage import convolve, distance_transform_edt, map_coordinates
from scipy.optimize import minimize


class ScanWallAgreement:
    """Precompute each map cell's scan-score contribution."""
    def __init__(self,grid):
        self.origin=grid.origin[:2]
        self.resolution=grid.resolution
        self.width,self.height=grid.width,grid.height
        cells=np.asarray(grid.cells).reshape(self.height,self.width)
        distance=distance_transform_edt(cells<65)*self.resolution
        self.support=np.where(cells<0,-1,distance<=.15).astype(np.int8)

    def score(self,points):
        if len(points)==0:
            return 0,None
        x,y=np.floor((np.asarray(points)-self.origin)/self.resolution).T
        inside=(x>=0)&(x<self.width)&(y>=0)&(y<self.height)
        matches=self.support[y[inside].astype(np.intp),x[inside].astype(np.intp)]
        known=int(np.count_nonzero(matches>=0))
        return known,float(np.count_nonzero(matches==1)/known) if known else None


def _rotation(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,s],[-s,c]])


def _prepare(scans, sensor, *, check_beams=False):
    ref=scans[-1]
    compatible=[s for s in scans if len(s['ranges'])==len(ref['ranges'])
                and abs(s['angle_min']-ref['angle_min'])<1e-5
                and abs(s['angle_increment']-ref['angle_increment'])<1e-6]
    if len(compatible)<2:
        raise ValueError('需要同格式的连续静止雷达帧')
    distances=np.asarray([s['ranges'] for s in compatible],dtype=float)
    distances[(distances<max(.18,ref['range_min'])) | (distances>min(8.,ref['range_max']))]=np.nan
    with warnings.catch_warnings():
        warnings.simplefilter('ignore',RuntimeWarning)
        ranges=np.nanmedian(distances,axis=0)
    indices=np.flatnonzero(np.isfinite(ranges)&(np.isfinite(distances).sum(axis=0)*2>=len(distances)))
    indices=indices[::max(1,math.ceil(len(indices)/220))]
    tx,ty,theta=sensor
    bearing=ref['angle_min']+indices*ref['angle_increment']+theta
    rays=ranges[indices,None]*np.column_stack((np.cos(bearing),np.sin(bearing)))
    start=np.array([tx,ty])
    ends=rays+start
    if len(ends)<65:
        raise ValueError('静止扫描回波不足，等雷达稳定后重试')
    probes=_beam_probes(rays)+start if check_beams else np.concatenate((start+.45*rays,start+.75*rays))
    return ends,probes


def _beam_probes(rays):
    ranges=np.linalg.norm(rays,axis=1)
    # Keep inspection points away from the reflecting surface itself.
    limit=np.maximum(.2,1.-.10/ranges)
    return np.concatenate([np.minimum(fraction,limit)[:,None]*rays for fraction in (.18,.35,.55,.75,.90,.94)])


class StationaryAligner:
    def __init__(self, grid):
        self.grid=grid
        self.cells=np.asarray(grid.cells).reshape(grid.height,grid.width)
        occupied=self.cells==100
        if not occupied.any():
            raise ValueError('底图未记录可用于定位的障碍边缘')
        length=max(5,round(.55/grid.resolution))
        along_x=convolve(occupied.astype(float),np.ones((3,length)),mode='constant')
        along_y=convolve(occupied.astype(float),np.ones((length,3)),mode='constant')
        horizontal=occupied&(along_x>=.62*length)
        vertical=occupied&(along_y>=.62*length)
        walls=horizontal|vertical
        if not walls.any():
            raise ValueError('底图中没有足够长的结构边缘')
        self.occupied=occupied.astype(float)
        self.wall_distance=distance_transform_edt(~walls)*grid.resolution
        self.edge_distances=[distance_transform_edt(~edge)*grid.resolution if edge.any()
                             else np.ones(self.cells.shape) for edge in (horizontal,vertical)]
        all_distance=distance_transform_edt(~occupied)*grid.resolution
        # Unmatched returns can be a moved object or a different door angle.
        self.response=.78*np.exp(-.5*(self.wall_distance/.085)**2)+.22*np.exp(-.5*(all_distance/.085)**2)
        self.origin=np.asarray(grid.origin[:2])
        self.scale=grid.resolution

    def _project(self, points, pose):
        return ((points@_rotation(pose[2])+pose[:2]-self.origin)/self.scale-.5)[:,::-1].T

    def _score(self, pose, ends, probes):
        ix,iy=self.grid.cell_xy(*pose[:2])
        if self.grid.value(ix,iy)!=0 or self.grid.forbidden(ix,iy):
            return -1.
        hits=map_coordinates(self.response,self._project(ends,pose),order=1,mode='constant',cval=0.)
        blocked=map_coordinates(self.occupied,self._project(probes,pose),order=1,mode='constant',cval=0.)
        return float(hits.mean()-.24*blocked.mean())

    def verifies_tracking(self, points, sensor_xy):
        """Verify a pose from observable fixed edges, excluding unrelated returns."""
        if len(points)<75:
            return False
        ix,iy=self.grid.cell_xy(*sensor_xy)
        if self.grid.value(ix,iy)!=0 or self.grid.forbidden(ix,iy):
            return False
        rays=np.asarray(points,dtype=float)-np.asarray(sensor_xy)
        distance=np.linalg.norm(rays,axis=1)
        rays=rays[(distance>=.18)&(distance<=8.)]
        if len(rays)<75:
            return False
        pose=np.asarray([*sensor_xy,0.])
        coords=self._project(rays,pose)
        depths=[map_coordinates(field,coords,order=1,mode='constant',cval=1.) for field in self.edge_distances]
        edge_rays=[rays[(near<=.11)&(near+.02<other)] for near,other in (depths,depths[::-1])]
        if sum(map(len,edge_rays))<44 or any(len(group)<12 or np.ptp(group[:,axis])<.65
                                           for axis,group in enumerate(edge_rays)):
            return False
        matched=np.concatenate(edge_rays)
        # Use distinct wall support across the scan with clear space in front of it.
        sectors=np.floor((np.arctan2(matched[:,1],matched[:,0])+math.pi)/(math.pi/4)).astype(int)%8
        if np.count_nonzero(np.bincount(sectors)>=6)<3:
            return False
        probes=_beam_probes(matched)
        if map_coordinates(self.occupied,self._project(probes,pose),order=1,mode='constant',cval=1.).mean()>.12:
            return False

        def residual(candidate):
            errors=[np.minimum(.22,map_coordinates(field,self._project(group,candidate),order=1,
                                                   mode='constant',cval=1.)).mean()
                    for field,group in zip(self.edge_distances,edge_rays)]
            return sum(errors)/len(errors)

        error=residual(pose)
        if error>.058:
            return False
        def loss(candidate):
            blocked=map_coordinates(self.occupied,self._project(probes,candidate),order=1,mode='constant',cval=1.)
            return residual(candidate)+.20*blocked.mean()

        accepted=loss(pose)
        # Require both position axes and yaw to be distinguishable from nearby alternatives.
        angle=math.radians(5)
        return all(loss(pose+offset)>accepted+.012
                   for offset in product((-.12,0.,.12),(-.12,0.,.12),(-angle,0.,angle))
                   if any(offset))

    def _refine(self, seed, ends, probes, lower, upper, radius):
        bounds=list(zip(np.maximum(seed-radius,lower),np.minimum(seed+radius,upper)))
        result=minimize(lambda pose:-self._score(pose,ends,probes),seed,method='Powell',bounds=bounds,
                        options={'maxiter':20,'xtol':.0008,'ftol':.00005})
        return result.x if self._score(result.x,ends,probes)>=self._score(seed,ends,probes) else seed

    def fit(self, hint, scans, sensor, *, radius=.45, heading_degrees=18., use_fixed_edges=False):
        if len(scans)<4:
            raise ValueError('静止扫描帧不足')
        hint=np.asarray(hint,dtype=float)
        if hint.shape!=(3,) or not np.all(np.isfinite(hint)):
            raise ValueError('请提供大致位置与车头方向')
        ends,probes=_prepare(scans,sensor,check_beams=use_fixed_edges)
        extent=np.array([radius,radius,math.radians(heading_degrees)])
        lower,upper=hint-extent,hint+extent
        gx,gy=self.grid.cell_xy(*lower[:2])
        hx,hy=self.grid.cell_xy(*upper[:2])
        centers=np.array([self.grid.world(x,y) for x in range(gx,hx+1,2) for y in range(gy,hy+1,2)
                          if self.grid.value(x,y)==0 and not self.grid.forbidden(x,y)])
        if len(centers)==0:
            raise ValueError('粗定位附近没有可用的地图空地')
        centers=centers[np.all((centers>=lower[:2])&(centers<=upper[:2]),axis=1)]
        if not len(centers):
            raise ValueError('粗定位附近没有可用的地图空地')
        nearest=np.rint((centers-self.origin)/self.scale-.5).astype(int)
        candidates=[]
        headings=np.linspace(lower[2],upper[2],max(3,math.ceil(heading_degrees/1.5)+1))
        for heading in headings:
            offsets=np.rint(ends[::2]@_rotation(heading)/self.scale).astype(int)
            x=nearest[:,0,None]+offsets[:,0]
            y=nearest[:,1,None]+offsets[:,1]
            valid=(x>=0)&(y>=0)&(x<self.grid.width)&(y<self.grid.height)
            lookup=self.response[y.clip(0,self.grid.height-1),x.clip(0,self.grid.width-1)]
            best=np.argsort((lookup*valid).mean(axis=1))[-7:]
            candidates.extend(np.r_[centers[index],heading] for index in best)
        candidates.sort(key=lambda p:self._score(p,ends,probes),reverse=True)
        seeds=[]
        for candidate in candidates:
            if all(np.linalg.norm(candidate[:2]-seed[:2])>.10 or abs(candidate[2]-seed[2])>math.radians(5)
                   for seed in seeds):
                seeds.append(candidate)
                if len(seeds)==10:
                    break
        fine_radius=np.array([.14,.14,math.radians(7.)])
        fits=[self._refine(seed,ends,probes,lower,upper,fine_radius) for seed in seeds]
        best=max(fits,key=lambda pose:self._score(pose,ends,probes))
        score=self._score(best,ends,probes)
        rivals=[self._score(other,ends,probes) for other in fits
                if np.linalg.norm(other[:2]-best[:2])>.14 or abs(other[2]-best[2])>math.radians(7)]
        gap=score-max(rivals,default=0.)
        distance=map_coordinates(self.wall_distance,self._project(ends,best),order=1,mode='constant',cval=1.)
        hit=distance<=.13
        quadrants=[np.sum(hit&(ends[:,0]*sx>=0)&(ends[:,1]*sy>=0))
                   for sx,sy in ((1,1),(1,-1),(-1,1),(-1,-1))]
        fixed_edges=(use_fixed_edges and self.verifies_tracking(ends@_rotation(best[2])+best[:2],
                                                               np.asarray(sensor[:2])@_rotation(best[2])+best[:2]))
        if gap<.014 or (not fixed_edges and (float(hit.mean())<.52 or sum(n>=7 for n in quadrants)<2)):
            raise ValueError('现场与底图的固定边缘匹配有歧义，请靠近墙角或提供更准确的粗定位')
        split=[]
        for frames in (scans[::2],scans[1::2]):
            group,checks=_prepare(frames,sensor,check_beams=use_fixed_edges)
            split.append(self._refine(best,group,checks,lower,upper,
                                      np.array([.065,.065,math.radians(4.)])))
        spread=float(np.linalg.norm(split[0][:2]-split[1][:2]))
        yaw_spread=abs(math.degrees(split[0][2]-split[1][2]))
        if spread>.052 or yaw_spread>3.:
            raise ValueError('前后雷达帧定位不一致，等待现场静止后重试')
        best[2]=math.remainder(best[2],2*math.pi)
        return {'pose':best.tolist(),'wall_support':float(hit.mean()),'score_gap':gap,
                'split_position_m':spread,'split_heading_deg':yaw_spread,'fixed_edges':fixed_edges}
