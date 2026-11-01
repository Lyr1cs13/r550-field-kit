"""Standalone measured-point map, works live or exported without a server."""
import base64
import json


def html_report(data=None,png=None):
    embedded=json.dumps(data,ensure_ascii=False).replace('<','\\u003c') if data else 'null'
    source='data:image/png;base64,'+base64.b64encode(png).decode() if png else '/map.png'
    return TEMPLATE.replace('__DATA__',embedded).replace('__IMAGE__',source)


TEMPLATE='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>区域 RSSI 连续路径热力图</title>
<style>*{box-sizing:border-box}body{margin:0;padding:24px;background:#101d29;color:#e5edf5;font:15px/1.6 system-ui,"Microsoft YaHei"}main{max-width:1250px;margin:auto}h1{margin:0 0 8px;font-size:24px}.tools{display:flex;gap:16px;align-items:center;flex-wrap:wrap;margin:18px 0}button,a,select{padding:8px 12px;border-radius:6px;border:1px solid #456178;background:#203548;color:#eef5ff;text-decoration:none}canvas{width:100%;height:70vh;min-height:420px;background:#182b39;border:1px solid #30485c;border-radius:8px}.scale{display:inline-block;width:180px;height:12px;background:linear-gradient(90deg,#93dcbb,#24b9b1,#308fc2,#4e57b8,#4d2b7e)}#info{white-space:pre-wrap;background:#203548;padding:14px;border-radius:8px}p{color:#a8becf}</style>
<main><h1>区域RSSI</h1><p>固定锚点 1 · 连续接收，约10cm格汇总 · 沿实测路径显示色带 · 未测区域留空</p><div class="tools"><label>颜色指标 <select id="metric"><option value="median">RSSI 中位数</option><option value="std">RSSI 波动标准差</option><option value="n">有效样本数</option></select></label><span id="status">等待采集</span><button id="png">保存 PNG</button><a href="/api/broadcast/export" id="export">导出数据与离线热力图</a></div><div><span id="low">−100 dBm</span> <i class="scale"></i> <span id="high">−30 dBm</span></div><canvas id="map"></canvas><p>色带仅连接相邻实测格，不穿墙、不跨越大空缺；显示宽度不代表两侧均已测量。点击查看原始位置、样本数与波动。</p><div id="info">点击彩色实测点查看详情；没有彩色点时，请先连接 Client、加载区域点位并开始采集。</div></main>
<script>
let data=__DATA__,img=new Image(),c=document.getElementById('map'),ctx=c.getContext('2d'),marks=[];img.src='__IMAGE__';
function draw(){if(!data||!img.complete||!img.naturalWidth)return;const dpr=devicePixelRatio||1,w=c.clientWidth,h=c.clientHeight;c.width=w*dpr;c.height=h*dpr;ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
const m=data.map||{max_x:5.5,max_y:1.1,resolution:.05},bounds=data.region||[m.origin[0],m.max_x,m.origin[1],m.max_y],ox=(m.max_y-bounds[3])/m.resolution,oy=(m.max_x-bounds[1])/m.resolution,rw=(bounds[3]-bounds[2])/m.resolution,rh=(bounds[1]-bounds[0])/m.resolution,s=Math.min((w-30)/rw,(h-30)/rh),px=(w-rw*s)/2,py=(h-rh*s)/2;
ctx.imageSmoothingEnabled=false;ctx.drawImage(img,ox,oy,rw,rh,px,py,rw*s,rh*s);marks=[];const metric=document.getElementById('metric').value,lo=metric==='median'?-95:0,hi=metric==='median'?-76:metric==='std'?12:100;
document.getElementById('low').textContent=lo+(metric==='median'?' dBm':'');document.getElementById('high').textContent=hi+(metric==='median'?' dBm':'');
const color=p=>{const t=Math.max(0,Math.min(1,(p[metric]-lo)/(hi-lo)))*4,i=Math.min(3,Math.floor(t)),f=t-i,a=[[147,220,187],[36,185,177],[48,143,194],[78,87,184],[77,43,126]];return 'rgb('+a[i].map((v,j)=>Math.round(v*(1-f)+a[i+1][j]*f)).join(',')+')';};
if(!img.mask){const mask=document.createElement('canvas');mask.width=img.naturalWidth;mask.height=img.naturalHeight;const mc=mask.getContext('2d');mc.drawImage(img,0,0);img.mask=mc.getImageData(0,0,mask.width,mask.height).data;}
const freeLine=(a,b)=>{const n=Math.max(2,Math.ceil(Math.hypot(a.x-b.x,a.y-b.y)/.025));for(let j=0;j<=n;j++){const t=j/n,x=a.x+(b.x-a.x)*t,y=a.y+(b.y-a.y)*t,ix=Math.floor((m.max_y-y)/m.resolution),iy=Math.floor((m.max_x-x)/m.resolution),k=(iy*img.naturalWidth+ix)*4;if(ix<0||iy<0||ix>=img.naturalWidth||iy>=img.naturalHeight||img.mask[k]<220||img.mask[k+1]<220||img.mask[k+2]<220)return false;}return true;};
for(const p of data.points)marks.push({x:px+(bounds[3]-p.y)/m.resolution*s,y:py+(bounds[1]-p.x)/m.resolution*s,p});
ctx.lineWidth=Math.max(3,.11/m.resolution*s);ctx.lineCap='round';
for(let i=0;i<marks.length;i++)for(let j=i+1;j<marks.length;j++){const a=marks[i],b=marks[j],distance=Math.hypot(a.p.x-b.p.x,a.p.y-b.p.y);if(distance>.22||distance<.001||!freeLine(a.p,b.p))continue;const gradient=ctx.createLinearGradient(a.x,a.y,b.x,b.y);gradient.addColorStop(0,color(a.p));gradient.addColorStop(1,color(b.p));ctx.strokeStyle=gradient;ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();}
for(const a of marks){ctx.fillStyle=color(a.p);ctx.beginPath();ctx.arc(a.x,a.y,Math.max(2,.055/m.resolution*s),0,Math.PI*2);ctx.fill();}

document.getElementById('status').textContent=(data.points.length||0)+' 个沿途实测热力格'+(data.status?' · '+(data.status.error||data.status.stage):'');}
c.onclick=e=>{const r=c.getBoundingClientRect(),x=e.clientX-r.left,y=e.clientY-r.top;const hit=marks.filter(p=>Math.hypot(p.x-x,p.y-y)<14).sort((a,b)=>Math.hypot(a.x-x,a.y-y)-Math.hypot(b.x-x,b.y-y))[0];if(hit){const p=hit.p;document.getElementById('info').textContent=p.point_id+'\\nx '+p.x.toFixed(3)+' m，y '+p.y.toFixed(3)+' m，yaw '+(p.yaw*180/Math.PI).toFixed(1)+'°\\n中位数 '+p.median.toFixed(1)+' dBm；标准差 '+p.std.toFixed(2)+' dB；有效样本 '+p.n+'；耗时 '+p.duration.toFixed(1)+' 秒';}};
document.getElementById('metric').onchange=draw;document.getElementById('png').onclick=()=>{let a=document.createElement('a');a.href=c.toDataURL('image/png');a.download='区域RSSI实测.png';a.click();};img.onload=draw;window.onresize=draw;
if(data){document.getElementById('export').style.display='none';draw();}else{async function poll(){try{const r=await fetch('/api/broadcast/data');if(!r.ok)throw Error('未启用广播采集');data=await r.json();draw();}catch(e){document.getElementById('status').textContent=e.message;}setTimeout(poll,1000);}poll();}
</script></html>'''
