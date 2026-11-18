'use strict';
const $=id=>document.getElementById(id);
const canvas=$('map'),ctx=canvas.getContext('2d');
const owner=Array.from(crypto.getRandomValues(new Uint32Array(4)),v=>v.toString(16).padStart(8,'0')).join('');
const phaseText={
 empty:'等待标定新桩位',restore:'已载入检查点，等待在桩位校准',
 pause:'已停车',manual:'遥控建图',leaving:'正在低速离桩',explore:'正在探索',
 waiting_for_frontiers:'等待可达的新边界',planning_explore:'正在规划下一段',
 navigating_explore:'正在扫描新区域',return:'正在规划回桩',
 planning_return:'正在规划回桩',navigating_return:'正在回桩',capture:'桩前寻找红外',
 docking:'正在原厂红外对接',docked:'已停稳并稳定充电',
 sensors_lost:'传感器反馈中断，已停车',localization_fault:'定位待恢复',
 outside_region:'接近探索边界，已停车',departure_blocked:'离桩受阻，已停车',
 no_ir:'桩前未收到可用红外，已停车',dock_failed:'贴合未完成，已停车',
 no_frontiers:'暂无可达的新边界，可遥控补扫、保存或回桩',
 return_unreachable:'当前地图没有可靠的回桩通路，可手动放回后校准',
};
const jobText={new:'正在标定桩位并建立首个检查点',restore:'正在恢复地图和桩位校准',
 save:'正在保存地图与 SLAM 检查点',pause:'正在暂停地图写入',activate:'正在切换巡航场地',
 leave:'正在保留检查点并结束建图',export:'正在保存巡航场地',manual:'正在切换遥控',
 explore:'正在准备自动探索',return:'正在准备回桩'};
let state,error='',online=false,shown=null,wanted='',scale=1,panX=0,panY=0,fitted=false;
let choosing=false,drag=null,held=null,timer,loading=false,refreshPending=false,driveSequence=0;

function navLink(port){
 const dest=new URL(location.href);
 dest.port=port;
 dest.pathname='/';
 dest.search=dest.hash='';
 return dest.href;
}

async function send(action,data={},quiet=false){
 if(!state||!online)return false;
 try{
  const response=await fetch('/api/site/command',{method:'POST',keepalive:action==='stop'||data.direction==='stop',
   headers:{'Content-Type':'application/json','X-Control-Token':state.control_token},
   body:JSON.stringify({action,...data})});
  if(!response.ok)throw Error((await response.json()).error);
  if(!quiet)await poll();
  return true;
 }catch(err){
  if(!quiet){error=err.message;render();}
  return false;
 }
}

async function poll(){
 if(loading){refreshPending=true;return;}
 loading=true;
 try{
  const response=await fetch('/api/site/state',{cache:'no-store',signal:AbortSignal.timeout(2200)});
  if(!response.ok)throw Error('连接中断');
  state=await response.json();
  online=true;
  loadMap(state.map);
 }catch{
  online=false;
  release(false);
 }finally{
  loading=false;
  render();
  if(refreshPending){refreshPending=false;queueMicrotask(poll);}
 }
}

function loadMap(map){
 if(!map||map.map_id===wanted)return;
 wanted=map.map_id;
 const image=new Image();
 image.onload=()=>{
  if(map.map_id!==wanted)return;
  if(shown&&fitted){
   panX+=(shown.map.max_y-map.max_y)*scale;
   panY+=(shown.map.max_x-map.max_x)*scale;
  }
  shown={map,image};
  if(!fitted)fit();
  draw();
 };
 image.onerror=()=>{if(wanted===map.map_id)wanted='';};
 image.src='/api/site/map?id='+encodeURIComponent(map.map_id);
}

function fit(){
 if(!shown)return;
 const m=shown.map,w=canvas.clientWidth,h=canvas.clientHeight;
 const margin=.65;
 scale=Math.min(w/(m.height*m.resolution+2*margin),h/(m.width*m.resolution+2*margin),180);
 panX=(w-m.height*m.resolution*scale)/2;
 panY=(h-m.width*m.resolution*scale)/2;
 fitted=true;
 draw();
}

function pixel(x,y){
 return [panX+(shown.map.max_y-y)*scale,panY+(shown.map.max_x-x)*scale];
}
function world(px,py){
 return [shown.map.max_x-(py-panY)/scale,shown.map.max_y-(px-panX)/scale];
}
function body(pose,back,front,width,fill){
 const [x,y,theta]=pose,c=Math.cos(theta),s=Math.sin(theta),half=width/2;
 const at=([a,b])=>pixel(x+a*c-b*s,y+a*s+b*c);
 ctx.beginPath();
 [[back,-half],[front,-half],[front,half],[back,half]].forEach((p,i)=>i?ctx.lineTo(...at(p)):ctx.moveTo(...at(p)));
 ctx.closePath();
 ctx.fillStyle=fill;
 ctx.fill();
 ctx.lineWidth=1;
 ctx.strokeStyle='#213442';
 ctx.stroke();
 return at;
}
function box(bounds){
 const [a,b,c,d]=bounds,[x,y]=pixel(b,d),[right,bottom]=pixel(a,c);
 ctx.fillStyle='#f6b34f23';ctx.strokeStyle='#ec9b24';ctx.lineWidth=2;
 ctx.fillRect(x,y,right-x,bottom-y);
 ctx.setLineDash([5,4]);ctx.strokeRect(x,y,right-x,bottom-y);ctx.setLineDash([]);
}
function draw(){
 const width=canvas.clientWidth,height=canvas.clientHeight,dpr=window.devicePixelRatio||1;
 if(canvas.width!==Math.round(width*dpr)||canvas.height!==Math.round(height*dpr)){
  canvas.width=Math.round(width*dpr);canvas.height=Math.round(height*dpr);
 }
 ctx.setTransform(dpr,0,0,dpr,0,0);
 ctx.fillStyle='#142431';ctx.fillRect(0,0,width,height);
 if(!shown)return;
 const {map,image}=shown;
 ctx.imageSmoothingEnabled=false;
 ctx.drawImage(image,panX,panY,map.height*map.resolution*scale,map.width*map.resolution*scale);
 const bounds=drag?.bounds||state?.bounds||state?.draft?.bounds;
 if(bounds)box(bounds);
 const path=state?.path||[];
 if(path.length>1){
  ctx.strokeStyle='#4689e3';ctx.lineWidth=2;ctx.beginPath();
  path.forEach((p,i)=>i?ctx.lineTo(...pixel(...p)):ctx.moveTo(...pixel(...p)));ctx.stroke();
 }
 ctx.fillStyle='#ed575d';
 for(const p of state?.radar||[]){const [x,y]=pixel(...p);ctx.fillRect(x-1,y-1,2,2);}
 if(state?.dock){
  body(state.dock,-.2235,-.1435,.192,'#354558');
  body(state.dock,-.1435,-.1325,.134,'#e5edf3');
 }
 if(state?.pose){
  const at=body(state.pose,-.1325,.1325,.207,'#08c7cf');
  ctx.fillStyle='#14344a';ctx.beginPath();ctx.moveTo(...at([.10,0]));
  ctx.lineTo(...at([-.04,.044]));ctx.lineTo(...at([-.04,-.044]));ctx.fill();
 }
}

function render(){
 const usable=online&&!!state,job=state?.job,ready=usable&&!job&&state.ready;
 const seated=$('seated').checked;
 $('connection').textContent=online?(state?.live?'建图在线':'场地管理在线'):'离线';
 $('status').textContent=error||state?.fault||(job?jobText[job]:null)||
  (state?.phase&&state.phase!=='pause'&&!['restore','empty'].includes(state.phase)?phaseText[state.phase]:null)||
  state?.message||phaseText[state?.phase]||'正在连接';
 $('status').classList.toggle('error',!!(error||state?.fault));
 $('map-title').textContent=state?.draft?.name||'现场地图';
 $('empty').hidden=!!shown;
 $('new').disabled=!usable||!!job||state.live||!seated||!$('name').value.trim();
 $('restore').disabled=!usable||!!job||!state.draft||!seated||(state.live&&!state.stationary);
 $('manual').disabled=!ready;
 $('explore').disabled=!ready||!state.nav_ready||!state.bounds;
 $('return').disabled=!ready||!state.nav_ready;
 $('save').disabled=!ready;
 $('leave').disabled=!usable||!!job||!state.live;
 $('stop').disabled=!usable||(!state.live&&!job);
 $('drive').hidden=!(ready&&state.phase==='manual');
 if($('drive').hidden)release(false);
 $('region').disabled=!ready||!state.stationary||!shown;
 $('fit').disabled=!shown;
 $('export').disabled=!usable||!!job||!state.draft?.checkpoint||state.scenes.some(s=>s.id===state.draft.id);
 const options=state?.scenes||[];
 const select=$('scene'),signature=JSON.stringify(options);
 if(signature!==select.dataset.items){
  const prior=select.value;
  select.replaceChildren(...options.map(s=>new Option(s.name,s.id)));
  select.value=options.some(s=>s.id===prior)?prior:(options.find(s=>s.id===state?.draft?.id)?.id||options[0]?.id||'');
  select.dataset.items=signature;
 }
 $('activate').disabled=!usable||!!job||!seated||!select.value;
 if(state)$('cruise').href=navLink(state.nav_port);
 draw();
}

function command(action,data={}){
 error='';
 return send(action,data);
}
async function confirmDock(action,extra={}){
 if(await command(action,{...extra,seated:$('seated').checked})){
  $('seated').checked=false;
  render();
 }
}
$('new').onclick=()=>{shown=null;wanted='';fitted=false;confirmDock('new',{name:$('name').value});};
$('restore').onclick=()=>confirmDock('restore');
$('activate').onclick=()=>confirmDock('activate',{scene:$('scene').value});
for(const name of ['manual','explore','return','save','export','leave']){
 $(name).onclick=()=>{release(true);command(name);};
}
$('stop').onclick=()=>{release(true);command('stop');};
$('fit').onclick=()=>fit();
for(const id of ['name','seated','scene'])$(id).oninput=render;
$('region').onclick=()=>{choosing=!choosing;$('region').setAttribute('aria-pressed',String(choosing));canvas.classList.toggle('region',choosing);};

function drive(direction){
 return send('drive',{owner,direction,session:state?.drive_session,sequence:++driveSequence},true);
}
function startDrive(direction){
 if($('drive').hidden)return;
 if(direction==='stop'){release(true);return;}
 if(held===direction)return;
 release(true);
 held=direction;
 $(direction).classList.add('held');
 const beat=()=>drive(direction);
 beat();timer=setInterval(beat,95);
}
function release(transmit){
 if(!held)return;
 $(held).classList.remove('held');held=null;
 clearInterval(timer);
 if(transmit)drive('stop');
}
for(const button of document.querySelectorAll('[data-drive]')){
 button.onpointerdown=e=>{if(e.button!==0)return;e.preventDefault();button.setPointerCapture(e.pointerId);startDrive(button.dataset.drive);};
 button.onpointerup=button.onpointercancel=button.onlostpointercapture=()=>release(true);
}
const keys={ArrowUp:'forward',ArrowDown:'back',ArrowLeft:'left',ArrowRight:'right'};
window.addEventListener('keydown',e=>{
 if(e.key==='Escape'){$('stop').click();return;}
 if(keys[e.key]&&!$('drive').hidden&&!['INPUT','SELECT'].includes(e.target.tagName)){
  e.preventDefault();startDrive(keys[e.key]);
 }
});
window.addEventListener('keyup',e=>{if(held&&held===keys[e.key])release(true);});
window.addEventListener('blur',()=>release(true));
document.addEventListener('visibilitychange',()=>{if(document.hidden)release(true);});
window.addEventListener('pagehide',()=>release(true));

function local(e){const b=canvas.getBoundingClientRect();return [e.clientX-b.left,e.clientY-b.top];}
canvas.onpointerdown=e=>{
 if(e.button!==0||!shown)return;
 canvas.setPointerCapture(e.pointerId);
 const [x,y]=local(e);
 drag={x,y,panX,panY,region:choosing&&!$('region').disabled};
};
canvas.onpointermove=e=>{
 if(!drag)return;
 const [x,y]=local(e);
 if(drag.region){
  const [a,b]=world(drag.x,drag.y),[c,d]=world(x,y);
  drag.bounds=[Math.min(a,c),Math.max(a,c),Math.min(b,d),Math.max(b,d)];
 }else{
  panX=drag.panX+x-drag.x;panY=drag.panY+y-drag.y;
 }
 draw();
};
canvas.onpointerup=()=>{
 if(drag?.bounds){
  command('bounds',{bounds:drag.bounds});
  choosing=false;$('region').setAttribute('aria-pressed','false');canvas.classList.remove('region');
 }
 drag=null;
 draw();
};
canvas.onpointercancel=()=>{drag=null;draw();};
canvas.addEventListener('wheel',e=>{
 if(!shown)return;
 e.preventDefault();
 const [x,y]=local(e),next=Math.max(4,Math.min(450,scale*Math.exp(-e.deltaY*.001)));
 panX=x+(panX-x)*next/scale;panY=y+(panY-y)*next/scale;scale=next;draw();
},{passive:false});
new ResizeObserver(()=>{if(!fitted)fit();else draw();}).observe(canvas);
poll();setInterval(poll,350);
