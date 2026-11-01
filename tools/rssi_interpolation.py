"""Compare spatial RSSI estimates with blocked validation; no synthetic samples."""
import numpy as np


def ray_obstacles(grid,origins,targets):
    occupied=np.array(grid.cells).reshape(grid.height,grid.width)>0
    result=np.zeros((len(origins),len(targets)),dtype=np.float32)
    for i,start in enumerate(origins):
        distance=np.linalg.norm(targets-start,axis=1);steps=max(2,int(np.ceil(distance.max()/grid.resolution))+1)
        t=np.linspace(0,1,steps)
        q=start[None,None,:]+(targets-start)[:,None,:]*t[None,:,None]
        ix=np.floor((q[:,:,0]-grid.origin[0])/grid.resolution).astype(int)
        iy=np.floor((q[:,:,1]-grid.origin[1])/grid.resolution).astype(int)
        valid=(ix>=0)&(iy>=0)&(ix<grid.width)&(iy<grid.height)
        mask=occupied[np.clip(iy,0,grid.height-1),np.clip(ix,0,grid.width-1)]&valid
        result[i]=mask[:,0].astype(int)+np.sum(mask[:,1:]&~mask[:,:-1],axis=1)
    return result


def idw(distance,values,k=8):
    order=np.argsort(distance,axis=0)[:min(k,len(values))]
    d=np.take_along_axis(distance,order,axis=0)
    w=np.where(np.isfinite(d),1/np.maximum(d,.05)**2,0);den=w.sum(axis=0)
    result=np.full(distance.shape[1],np.nan);valid=den>0
    result[valid]=(w*values[order]).sum(axis=0)[valid]/den[valid]
    return result


def robust_trend(x,y):
    weights=np.ones(len(y));beta=np.zeros(x.shape[1])
    for _ in range(4):
        beta=np.linalg.solve(x.T@(weights[:,None]*x)+np.diag([1e-6,.1,.2]),x.T@(weights*y))
        beta[1]=np.clip(beta[1],-45,0);beta[2]=np.clip(beta[2],-12,0)
        beta[0]=np.average(y-x[:,1:]@beta[1:],weights=weights)
        residual=y-x@beta;scale=max(1.,1.4826*np.median(np.abs(residual-np.median(residual))))
        weights=np.minimum(1.,1.5*scale/np.maximum(np.abs(residual),1e-6))
    return beta


def estimate(grid,cells,sources,geodesic,world,index,anchor=None):
    xy=np.array([world(c) for c,_ in sources]);values=np.array([v for _,v in sources])
    query=np.array([world(c) for c in cells]);columns=[index[c] for c,_ in sources]
    gd=geodesic[:,columns];euclidean=np.linalg.norm(xy[:,None,:]-xy[None,:,:],axis=2)
    walls=ray_obstacles(grid,xy,xy)
    anchor_known=anchor is not None
    if anchor is None:anchor=(0.,0.)
    anchor_walls=ray_obstacles(grid,np.array([anchor]),xy)[0]
    features=np.column_stack([np.ones(len(xy)),np.log10(np.maximum(np.linalg.norm(xy-np.array(anchor),axis=1),.25)),anchor_walls])
    # Every 0.8m block stays in one fold; packets at one position cannot leak.
    folds=np.array([((c[0]//8)+3*(c[1]//8))%4 for c,_ in sources])
    configs=[('geodesic',0.),('distance',0.),('occlusion',.25),('occlusion',.5),('occlusion',1.),('trend_residual',.5)]
    if not anchor_known:configs=[p for p in configs if p[0]!='trend_residual']
    scores=[]
    for method,gamma in configs:
        residuals=[]
        for fold in range(4):
            train=np.where(folds!=fold)[0];test=np.where(folds==fold)[0]
            if len(train)<8 or not len(test):continue
            d=gd[np.ix_(train,test)] if method=='geodesic' else euclidean[np.ix_(train,test)]*(1+gamma*walls[np.ix_(train,test)])
            if method=='trend_residual':
                beta=robust_trend(features[train],values[train])
                pred=features[test]@beta+idw(d,values[train]-features[train]@beta)
            else:pred=idw(d,values[train])
            valid=np.isfinite(pred);residuals.extend((pred[valid]-values[test][valid]).tolist())
        mae=float(np.mean(np.abs(residuals))) if residuals else float('inf')
        scores.append(dict(method=method,occlusion_penalty=gamma,mae_db=mae,rmse_db=float(np.sqrt(np.mean(np.array(residuals)**2))) if residuals else float('inf'),validation_points=len(residuals)))
    selected=min(scores,key=lambda p:p['mae_db'])
    if scores[0]['mae_db']-selected['mae_db']<.15:selected=scores[0]
    method=selected['method'];gamma=selected['occlusion_penalty'];beta=None
    if method=='geodesic':prediction=idw(geodesic,values)
    else:
        d=np.linalg.norm(xy[:,None,:]-query[None,:,:],axis=2)
        if gamma:d*=1+gamma*ray_obstacles(grid,xy,query)
        if method=='trend_residual':
            beta=robust_trend(features,values)
            f=np.column_stack([np.ones(len(query)),np.log10(np.maximum(np.linalg.norm(query-np.array(anchor),axis=1),.25)),ray_obstacles(grid,np.array([anchor]),query)[0]])
            prediction=f@beta+idw(d,values-features@beta)
        else:prediction=idw(d,values)
    # Do not predict across a disconnected component without measurements.
    prediction[~np.isfinite(np.min(geodesic,axis=0))]=np.nan
    prediction=np.clip(prediction,np.quantile(values,.01)-3,np.quantile(values,.99)+3)
    report=dict(selected=selected,candidates=scores,validation='4-fold 0.8m spatial blocks; each support is a measured grid median',
                anchor_xy_approximate=list(anchor) if anchor_known else None,anchor_source='scene metadata' if anchor_known else 'not configured',
                fitted_trend_coefficients=beta.tolist() if beta is not None else None,
                obstacle_note='2D occupancy crossings; material and height unknown; not a calibrated RF propagation model')
    return prediction,report
