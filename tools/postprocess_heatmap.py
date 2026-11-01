"""Preserve measured data; estimate known-free north area using geodesic IDW.

The estimate is not new measurement, a radio propagation model or localization
accuracy evidence. Run on PC after exporting the acquisition archive.
"""
import argparse
import csv
import hashlib
import heapq
import io
import json
import math
from pathlib import Path
import sys
import zipfile
import numpy as np
from PIL import Image,ImageDraw,ImageFont


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('archive',type=Path);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();out=a.out;out.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(a.archive) as z:
        for name in z.namelist():
            rel=Path(name)
            if rel.is_absolute() or '..' in rel.parts:raise ValueError('Invalid archive path')
        z.extractall(out/'original_export')
    src=out/'original_export';meta=json.loads((src/'session.json').read_text(encoding='utf-8'))
    points=json.loads((src/'points.json').read_text(encoding='utf-8'))
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'demo'))
    from map_grid import MapGrid
    g=MapGrid(src/'map');assert g.map_id==meta['map']['map_id']
    rows=[]
    for f in sorted((src/'segments').glob('*/raw.csv'))+[src/'raw.csv']:
        if f.exists():
            with f.open(encoding='utf-8',newline='') as h:rows.extend(csv.DictReader(h))
    fields=list(dict.fromkeys(k for r in rows for k in r))
    with (out/'raw_all_segments.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
    xmin,xmax,ymin,ymax=meta.get('display_bounds',[g.origin[0],g.max_x,g.origin[1],g.max_y]);res=.1
    nx,ny=round((xmax-xmin)/res),round((ymax-ymin)/res)
    def world(c):return xmin+(c[0]+.5)*res,ymin+(c[1]+.5)*res
    free=set()
    for ix in range(nx):
        for iy in range(ny):
            x,y=world((ix,iy));cell=g.cell_xy(x,y)
            if g.value(*cell)==0 and not g.forbidden(*cell):free.add((ix,iy))
    cells=sorted(free);index={c:i for i,c in enumerate(cells)}
    sources=[]
    for item in points:
        if item['n']<3:continue
        c=(min(nx-1,max(0,math.floor((item['x']-xmin)/res))),min(ny-1,max(0,math.floor((item['y']-ymin)/res))))
        if c not in free:
            c=min(free,key=lambda q:math.dist(world(q),(item['x'],item['y'])))
            if math.dist(world(c),(item['x'],item['y']))>.16:continue
        sources.append((c,float(item['median'])))
    if not sources:raise ValueError('No valid measured supports')
    distances=np.full((len(sources),len(cells)),np.inf,dtype=np.float32)
    for i,(source,value) in enumerate(sources):
        queue=[(0.,source)];best={source:0.}
        while queue:
            d,c=heapq.heappop(queue)
            if d>best[c]:continue
            distances[i,index[c]]=d
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                n=c[0]+dx,c[1]+dy
                if n in free and d+res<best.get(n,float('inf')):
                    best[n]=d+res;heapq.heappush(queue,(d+res,n))
    nearest=np.min(distances,axis=0);k=min(8,len(sources))
    neighbors=np.argsort(distances,axis=0)[:k];d=np.take_along_axis(distances,neighbors,axis=0)
    weight=np.where(np.isfinite(d),1/np.maximum(d,.05)**2,0)
    vals=np.array([v for _,v in sources]);den=weight.sum(axis=0)
    estimate=np.full(len(cells),np.nan);valid=den>0
    estimate[valid]=(weight*vals[neighbors]).sum(axis=0)[valid]/den[valid]
    from rssi_interpolation import estimate as select_estimate
    estimate,model_report=select_estimate(g,cells,sources,distances,world,index,anchor=([meta['anchor_metadata'][k] for k in ('x_m','y_m')] if all(meta.get('anchor_metadata',{}).get(k) is not None for k in ('x_m','y_m')) else None))
    valid=np.isfinite(estimate)
    # Mild smoothing only along connected free cells, with measurements pinned.
    pinned={index[c]:v for c,v in sources}
    for _ in range(12):
        nxt=estimate.copy()
        for c,j in index.items():
            if j in pinned:nxt[j]=pinned[j];continue
            neighbors=[index[n] for n in ((c[0]-1,c[1]),(c[0]+1,c[1]),(c[0],c[1]-1),(c[0],c[1]+1)) if n in index and np.isfinite(estimate[index[n]])]
            if neighbors and np.isfinite(estimate[j]):nxt[j]=.7*estimate[j]+.3*float(np.mean(estimate[neighbors]))
        estimate=nxt
    with (out/'estimated_grid.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.writer(f);w.writerow(['x_m','y_m','estimated_rssi_dbm_NOT_MEASURED','nearest_measured_path_distance_m'])
        for c,value,distance in zip(cells,estimate,nearest):
            if np.isfinite(value):w.writerow([*world(c),round(float(value),3),round(float(distance),3)])
    scale=100;W,H=round((ymax-ymin)*scale),round((xmax-xmin)*scale)
    raw=Image.open(io.BytesIO(g.png())).convert('RGB')
    crop=((g.max_y-ymax)/g.resolution,(g.max_x-xmax)/g.resolution,(g.max_y-ymin)/g.resolution,(g.max_x-xmin)/g.resolution)
    base=raw.crop(tuple(round(v) for v in crop)).resize((W,H),Image.Resampling.NEAREST)
    def xy(x,y):return (ymax-y)*scale,(xmax-x)*scale
    palette=np.array([[147,220,187],[36,185,177],[48,143,194],[78,87,184],[77,43,126]],dtype=float)
    def color(value,lo=-95,hi=-76):
        t=max(0,min(1,(value-lo)/(hi-lo)))*4;i=min(3,int(t));f=t-i
        return tuple(np.round(palette[i]*(1-f)+palette[i+1]*f).astype(int))
    font_path=next((str(p) for p in [Path('C:/Windows/Fonts/msyh.ttc'),Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')] if p.exists()),'DejaVuSans.ttf')
    font=ImageFont.truetype(font_path,18);small=ImageFont.truetype(font_path,14)
    def save(im,name,title,subtitle,distance=False):
        canvas=Image.new('RGB',(W,H+120),'white');canvas.paste(im,(0,50));draw=ImageDraw.Draw(canvas)
        clean_title='区域RSSI' if name.startswith(('原始','路径')) else '区域RSSI 插值估计' if '插值' in name else '估计依据距离'
        draw.text((14,12),clean_title,font=font,fill='#152c3b')
        left,right=25,W-25;top=H+72
        for px in range(left,right):
            value=(px-left)/(right-left)*(4 if distance else 19)+(0 if distance else -95)
            draw.line((px,top,px,top+14),fill=color(value,0,4) if distance else color(value))
        draw.text((left,top+18),'0 m' if distance else '≤ −95 dBm',font=small,fill='#263f4e')
        text='4 m 以上' if distance else '≥ −76 dBm';draw.text((right-100,top+18),text,font=small,fill='#263f4e')
        canvas.save(out/name)
    measured=base.copy();draw=ImageDraw.Draw(measured)
    for item in points:
        x,y=xy(item['x'],item['y']);draw.ellipse((x-3,y-3,x+3,y+3),fill=color(item['median']))
    save(measured,'原始实测点图_不插值.png','北区原始实测点图｜不插值',f'{len(points)} 个约10cm汇总格；逐条数据保留在 CSV，未测区域不着色')
    path_image=base.copy();pd=ImageDraw.Draw(path_image)
    def free_segment(a,b):
        steps=max(2,math.ceil(math.dist((a['x'],a['y']),(b['x'],b['y']))/.025))
        for t in np.linspace(0,1,steps):
            c=g.cell_xy(a['x']+(b['x']-a['x'])*t,a['y']+(b['y']-a['y'])*t)
            if g.value(*c)!=0 or g.forbidden(*c):return False
        return True
    # Connect successive real accepted packets, not arbitrary nearby bins from
    # different passes. Parallel legs therefore cannot create cross-lane bridges.
    point_lookup={p['point_id']:p for p in points};previous=None
    for record in rows:
        if record.get('accepted')!='True':continue
        x,y=float(record['x_m']),float(record['y_m']);stamp=float(record['rx_host_monotonic'])
        origin=meta.get('bin_origin',[0,0])
        key=f"path-bin-{math.floor((x-origin[0])/.1)}-{math.floor((y-origin[1])/.1)}"
        value=point_lookup.get(key,{}).get('median',float(record['rssi_dbm']))
        current=dict(x=x,y=y,t=stamp,value=value,session=record.get('session'),epoch=record.get('radio_epoch','0'))
        ax,ay=xy(x,y)
        if previous:
            distance=math.dist((previous['x'],previous['y']),(x,y));dt=stamp-previous['t']
            if 0<=dt<=2.5 and distance<=.20 and previous['session']==current['session'] and previous['epoch']==current['epoch'] and free_segment(previous,current):
                bx,by=xy(previous['x'],previous['y']);steps=max(2,math.ceil(distance*scale));last=(bx,by)
                for t in np.linspace(0,1,steps):
                    nxt=(bx+(ax-bx)*t,by+(ay-by)*t);v=previous['value']+(value-previous['value'])*t
                    pd.line((*last,*nxt),fill=color(v),width=11);last=nxt
        pd.ellipse((ax-5.5,ay-5.5,ax+5.5,ay+5.5),fill=color(value));previous=current
    allowed=Image.fromarray((np.min(np.array(base),axis=2)>220).astype(np.uint8)*255)
    path_image=Image.composite(path_image,base,allowed)
    save(path_image,'路径RSSI原图.png','区域RSSI','相邻实测格的路径显示，不是全区域插值')
    area=base.copy();support=base.copy();ad=ImageDraw.Draw(area);sd=ImageDraw.Draw(support)
    for c,v,distance in zip(cells,estimate,nearest):
        if not np.isfinite(v):continue
        x,y=xy(*world(c));box=(round(x-5),round(y-5),round(x+5),round(y+5))
        ad.rectangle(box,fill=color(float(v)));sd.rectangle(box,fill=color(min(float(distance),4),0,4))
    allowed=Image.fromarray((np.min(np.array(base),axis=2)>220).astype(np.uint8)*255)
    area=Image.composite(area,base,allowed);support=Image.composite(support,base,allowed)
    save(area,'北区全局热力图_插值估计_非实测.png','北区全局 RSSI 热力图｜插值估计，非全区域实测','仅对地图已知可通行区估计；8近邻、路径距离 IDW；不作为定位精度证明')
    save(support,'估计依据距离图.png','估计依据距离｜到最近有效实测支撑点的连通距离','插值支撑点每格至少3条；越远通常越不可靠；不是误差置信区间',distance=True)
    summary={'map_id':g.map_id,'measured_bins':len(points),'accepted_records':sum(r.get('accepted')=='True' for r in rows),
             'raw_records':len(rows),'estimated_cells':int(valid.sum()),'interpolation_supports':len(sources),
             'interpolation':model_report['selected']['method']+'; 0.1m known-free grid; 12 graph smoothing steps, measurements pinned; supports n>=3',
             'maximum_support_distance_m':float(np.max(nearest[valid])),'whole_area_measured':False,
             'model_comparison':model_report,
             'software_interruptions':bool(list((out/'original_export').glob('segments/*'))),'archive_sha256':hashlib.sha256(a.archive.read_bytes()).hexdigest()}
    (out/'postprocess_report.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    (out/'查看热力图.html').write_text('''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>北区 RSSI 实测与区域估计对照</title><style>body{margin:24px;background:#122331;color:#edf5fa;font:16px/1.7 system-ui}main{max-width:1200px;margin:auto}img{width:100%;display:block;margin:20px 0}a{color:#7fe0e7}section{margin:32px 0}h1{font-size:25px}</style><main><h1>北区 RSSI 实测与区域估计对照</h1><p>原始实测图与数据完整保留。全局图是基于有限回环数据的插值估计，不能把颜色覆盖等同于已经测量，更不能据此认定定位精度。</p><section><h2>原始实测（不插值）</h2><img src="路径RSSI原图.png"></section><section><h2>区域估计（非实测）</h2><img src="北区全局热力图_插值估计_非实测.png"></section><section><h2>估计依据距离</h2><img src="估计依据距离图.png"></section><p><a href="original_export/heatmap.html">打开保留的连续实测路径图</a> · <a href="raw_all_segments.csv">合并原始 CSV</a> · <a href="estimated_grid.csv">估计网格 CSV</a></p></main>''',encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False))


if __name__=='__main__':main()
