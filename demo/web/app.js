'use strict';
const $ = id => document.getElementById(id);
const phaseNames = {disconnected:'实车离线',collecting:'等待采集完成',paused_collection:'采集等待已暂停',canceling_collection:'正在取消采集',idle:'等待开始',departing:'驶离充电桩',settling_departure:'离桩等待停稳',paused_departure:'离桩已暂停',paused_settling_departure:'停稳确认已暂停',settling:'等待停稳',settling_return:'桩前等待停稳',paused_settling:'停稳确认已暂停',paused_settling_return:'停稳确认已暂停',navigating:'正在导航',dwelling:'定点停留',returning:'正在回充',docking:'红外对接',docked:'桩上充电',completed:'巡航完成',failed:'任务停止',canceling:'正在停车',paused_navigation:'导航已暂停',paused_return:'回程已暂停',paused_dwell:'停留已暂停',paused_dock:'对接已暂停'};
const collectionNames={ws63:'WS63 旧版',broadcast:'WS63 沿途',broadcast_station:'WS63 定点',servo_car_v2:'WS63 旋转定点'};
phaseNames.reconnecting='正在重连';
let requestCount = 0;
let collectionDirty=false;
let autoReturnBeforePair;
let imageMapId = null;
let showSimulationPoints = true;
let state, selected = null, mode = 'pan', busy = false, pointsKey = '', zoom = 1, panX = 0, panY = 0, gesture, hover, fitted = false;
const canvas = $('map'), ctx = canvas.getContext('2d'), image = new Image();
image.onload = () => { if (state) fitted=fit(false); };
image.onerror = () => { imageMapId = null; fitted = false; };
const vehicleLinks = [...document.querySelectorAll('[data-vehicle-link]')].map(link => [link, link.getAttribute('href')]);

function syncMapImage() {
 const mapId = state.map.map_id || '';
 if (imageMapId === mapId) return;
 fitted = false;
 imageMapId = mapId;
 image.src = '/map.png?map_id='+encodeURIComponent(mapId);
}
const esc = text => String(text).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const returnPhases = new Set(['returning','settling_return','docking','paused_return',
 'paused_settling_return','paused_dock']);
const canCancelReturn = () => returnPhases.has(state?.phase) && !state?.fleet_simulation?.running;
const quiet = () => state && state.vehicle_connected !== false && ['idle','completed','failed','docked'].includes(state.phase);
function toast(text){ $('toast').textContent = text;$('toast').classList.add('show');clearTimeout(toast.timer);toast.timer = setTimeout(() => $('toast').classList.remove('show'), 4200); }
async function command(data) {
 if (!state || (state.vehicle_connected===false && !(data.action==='stop' && state.stop_token))) return false;
 if (busy && data.action !== 'stop') return false;
 const token = state.control_token || state.stop_token;
 requestCount++;
 busy = true;
 try {
  const res = await fetch('/api/command', {
   method:'POST', headers:{'Content-Type':'application/json','X-Control-Token':token},
   body:JSON.stringify(data)
  });
  const result = await res.json();
  if (!res.ok) throw new Error(result.error);
  if (state.control_token === token) {
   state = {...result, control_token:token, vehicle_connected:state.vehicle_connected,connection_status:state.connection_status,
    fleet_control:state.fleet_control,fleet_paired:state.fleet_paired,fleet_number:state.fleet_number,
    fleet_peer:state.fleet_peer,fleet_docks:state.fleet_docks,fleet_zones:state.fleet_zones,
    fleet_simulation:state.fleet_simulation,site_url:state.site_url};
   render();
  }
  return true;
 } catch (e) { toast(e.message);return false; }
 finally { busy = --requestCount > 0; }
}
async function poll(){try{const res=await fetch('/api/state');state=await res.json();render();if(!fitted)fitted=fit(false);}catch(e){$('message').textContent='页面连接中断';$('message').hidden=false;['start','return','confirm'].forEach(k=>$(k).disabled=true);}finally{setTimeout(poll,250);} }
function setMode(value){
 if(value==='add'||value==='move')showSimulationPoints=false;
 mode=value;canvas.className=value;
 ['pan-tool','add-tool','pose-tool'].forEach(k=>$(k).classList.toggle('active',k.startsWith(value)));
 $('map-tip').hidden=value==='pan';
 $('map-tip').textContent={add:'点击加点，拖动设朝向',move:'点击新位置，拖动改朝向',pose:'从实车位置拖出车头方向'}[value]||'';
}
function screen(x,y){const m=state.map;return [panX+(m.max_y-y)/m.resolution*zoom,panY+(m.max_x-x)/m.resolution*zoom];}
function world(x,y){const m=state.map;return [m.max_x-(y-panY)/zoom*m.resolution,m.max_y-(x-panX)/zoom*m.resolution];}
// Top view in metres. EC130 dimensions come from the vendor's dock STEP model.
const chassis={length:.265,width:.207};
const charger={caseDepth:.080,width:.192,contactDepth:.011,contactWidth:.134};
function bodyScreen([x,y,yaw],forward,lateral){
 const cos=Math.cos(yaw),sin=Math.sin(yaw);
 return screen(x+cos*forward-sin*lateral,y+sin*forward+cos*lateral);
}
function drawBody(pose,back,front,width,fill,stroke){
 const half=width/2;
 const corners=[[back,-half],[front,-half],[front,half],[back,half]];
 ctx.beginPath();
 corners.forEach(([forward,lateral],index)=>{
  const point=bodyScreen(pose,forward,lateral);
  if(index===0)ctx.moveTo(...point);
  else ctx.lineTo(...point);
 });
 ctx.closePath();
 ctx.fillStyle=fill;
 ctx.fill();
 if(stroke){
  ctx.lineWidth=1;
  ctx.strokeStyle=stroke;
  ctx.stroke();
 }
}
function mapLabel(label,x,y){
 ctx.font='11px Microsoft YaHei, sans-serif';
 ctx.lineWidth=3;
 ctx.strokeStyle='#142431';
 ctx.fillStyle='#fff';
 ctx.strokeText(label,x,y);
 ctx.fillText(label,x,y);
}
function drawCharger(pose,number){
 // The saved pose is the seated vehicle centre. The charger lies behind its rear bumper.
 const contact=-chassis.length/2;
 const casing=contact-charger.contactDepth;
 drawBody(pose,casing-charger.caseDepth,casing,charger.width,'#354558','#e5edf3');
 drawBody(pose,casing,contact,charger.contactWidth,'#e5edf3');
 const [x,y]=bodyScreen(pose,casing-charger.caseDepth/2,0);
 const radius=charger.width/2*zoom/state.map.resolution;
 mapLabel(number?number+'号充电桩':'充电桩',x+radius+5,y+4);
}
function drawChassis(pose,color,number){
 const half=chassis.length/2;
 drawBody(pose,-half,half,chassis.width,color,'#183649');
 // The triangle marks the nose and rotates with the body.
 ctx.beginPath();
 ctx.moveTo(...bodyScreen(pose,half*.75,0));
 ctx.lineTo(...bodyScreen(pose,-half*.25,.043));
 ctx.lineTo(...bodyScreen(pose,-half*.25,-.043));
 ctx.closePath();
 ctx.fillStyle='#173447';
 ctx.fill();
 if(number){
  const [x,y]=bodyScreen(pose,0,0);
  const yaw=pose[2];
  const top=(Math.abs(Math.cos(yaw))*chassis.length+Math.abs(Math.sin(yaw))*chassis.width)/2;
  mapLabel(number+'号车',x+4,y-top*zoom/state.map.resolution-5);
 }
}
function fit(full){
 const w=canvas.clientWidth,h=canvas.clientHeight;
 if(!state||!image.complete||!image.naturalWidth||w<=40||h<=55)return false;
 const m=state.map,north=state.points.length>0&&state.points.every(p=>p.id.startsWith('north-'));
 let b=full?[m.origin[0],m.max_x,m.origin[1],m.max_y]:north?[-.5,5.5,-8.2,1.0]:m.focus_bounds;
 const points=showSimulationPoints?state.fleet_simulation?.points:[];
 if(!full&&points?.length){
  const xs=points.map(p=>p.x),ys=points.map(p=>p.y);
  b=[Math.min(...xs)-.55,Math.max(...xs)+.55,Math.min(...ys)-.55,Math.max(...ys)+.55];
 }
 const a=[(m.max_y-b[3])/m.resolution,(m.max_x-b[1])/m.resolution],z=[(m.max_y-b[2])/m.resolution,(m.max_x-b[0])/m.resolution];
 zoom=Math.min((w-40)/(z[0]-a[0]),(h-55)/(z[1]-a[1]));
 panX=(w-(z[0]-a[0])*zoom)/2-a[0]*zoom;panY=(h-(z[1]-a[1])*zoom)/2-a[1]*zoom;
 draw();return true;
}
function arrow(x,y,yaw,color,size=23){const p=screen(x,y),dx=-Math.sin(yaw),dy=-Math.cos(yaw);ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(p[0]+dx*size,p[1]+dy*size);ctx.stroke();ctx.beginPath();ctx.moveTo(p[0]+dx*size,p[1]+dy*size);ctx.lineTo(p[0]+dx*(size-7)-dy*4,p[1]+dy*(size-7)+dx*4);ctx.lineTo(p[0]+dx*(size-7)+dy*4,p[1]+dy*(size-7)-dx*4);ctx.fill();}
const pairColors={A:'#087f89',B:'#8055bf'};
function drawPairPoints(simulation){
 const radius=Math.min(9,Math.max(5,.10*zoom/state.map.resolution));
 for(const point of simulation.points){
  const car=simulation.targets.A===point.id?'A':simulation.targets.B===point.id?'B':null;
  const color=pairColors[car]||'#3a6178',position=screen(point.x,point.y);
  arrow(point.x,point.y,point.yaw,color,radius+8);
  ctx.beginPath();ctx.arc(...position,radius+(car?2:0),0,Math.PI*2);
  ctx.fillStyle=car?color:'#f1f7f8';ctx.fill();ctx.strokeStyle=color;ctx.lineWidth=car?2:1;ctx.stroke();
  ctx.font=`600 ${radius+1}px system-ui`;ctx.textAlign='center';
  ctx.fillStyle=car?'#fff':color;ctx.fillText(point.number,position[0],position[1]+3);
  ctx.textAlign='left';
 }
}
function draw(){if(!state)return;const ratio=window.devicePixelRatio||1,w=canvas.clientWidth,h=canvas.clientHeight;if(canvas.width!==Math.round(w*ratio)||canvas.height!==Math.round(h*ratio)){canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);}ctx.setTransform(ratio,0,0,ratio,0,0);ctx.fillStyle='#142431';ctx.fillRect(0,0,w,h);ctx.imageSmoothingEnabled=false;if(image.complete && image.naturalWidth)ctx.drawImage(image,panX,panY,image.width*zoom,image.height*zoom);
 const simulation=state.fleet_simulation;
 const surveyPreview=state.collection?.profile==='moving_loop_demo' ? state.survey_route_preview : null;
 const preview=state.vehicle_connected===false ? [] : surveyPreview||[];
 if(preview.length){ctx.strokeStyle="#e9a23b";ctx.lineWidth=2;ctx.setLineDash([7,5]);ctx.beginPath();preview.forEach((p,i)=>{const v=screen(...p);i?ctx.lineTo(...v):ctx.moveTo(...v);});ctx.stroke();ctx.setLineDash([]);ctx.fillStyle="#ffda9b";ctx.font="12px Microsoft YaHei";ctx.fillText("橙色虚线：采集路线示意",16,54);}
 const showRoute=state.vehicle_connected!==false&&['departing','navigating','returning','retreating','docking','settling','settling_return','settling_departure'].includes(state.phase);
 const path=showRoute ? state.health.path||[] : [];if(path.length){ctx.strokeStyle=simulation?.running?pairColors[state.fleet_number===1?'A':'B']:'#5d9be9';ctx.lineWidth=2;ctx.beginPath();path.forEach((p,i)=>{const s=screen(...p);i?ctx.lineTo(...s):ctx.moveTo(...s);});ctx.stroke();}
 if(simulation?.running&&simulation.peer_path.length){ctx.strokeStyle=pairColors[state.fleet_number===1?'B':'A'];ctx.lineWidth=2;ctx.beginPath();simulation.peer_path.forEach((p,i)=>{const s=screen(...p);i?ctx.lineTo(...s):ctx.moveTo(...s);});ctx.stroke();}
 const track=showRoute ? state.health.track||[] : [];if(track.length>1){ctx.strokeStyle='#087f89';ctx.lineWidth=2;ctx.beginPath();track.forEach((p,i)=>{const s=screen(...p);i?ctx.lineTo(...s):ctx.moveTo(...s);});ctx.stroke();}
 ctx.fillStyle='#f45555';(state.health.laser||[]).forEach(p=>{const s=screen(...p);ctx.fillRect(s[0]-1,s[1]-1,2,2);});
 for(const {number,bounds} of state.fleet_zones||[]){
  const [left,top]=screen(bounds[1],bounds[3]),[right,bottom]=screen(bounds[0],bounds[2]);
  ctx.strokeStyle=number===state.fleet_number?'#087f89':'#8055bf';ctx.lineWidth=2;
  ctx.setLineDash([6,5]);ctx.strokeRect(left,top,right-left,bottom-top);ctx.setLineDash([]);
 }
 const docks=state.fleet_docks||(state.dock&&state.health.dock_calibrated!==false?[{pose:state.dock}]:[]);
 for(const {pose,number} of docks)drawCharger(pose,number);
 if(simulation&&showSimulationPoints)drawPairPoints(simulation);
 else state.points.forEach((p,i)=>{const s=screen(p.x,p.y);ctx.fillStyle=p.id===selected?'#13a6aa':'#fff';ctx.strokeStyle=p.enabled?'#087f89':'#96a5ab';ctx.lineWidth=p.id===selected?3:2;ctx.beginPath();ctx.arc(...s,11,0,Math.PI*2);ctx.fill();ctx.stroke();ctx.fillStyle=p.id===selected?'#fff':'#087f89';ctx.font='bold 11px Microsoft YaHei';ctx.textAlign='center';ctx.fillText(i+1,s[0],s[1]+4);ctx.textAlign='left';if(!p.id.startsWith("north-loop-"))arrow(p.x,p.y,p.yaw,p.enabled?'#087f89':'#96a5ab',30);});
 const colors=state.fleet_number===2?['#a16eff','#08c7cf']:['#08c7cf','#a16eff'];
 const cars=[[state.health.pose,colors[0],state.fleet_number],[state.fleet_peer,colors[1],3-state.fleet_number]];
 cars.forEach(([pose,color,number])=>{
  if(pose)drawChassis(pose,color,number);
 });
 if(gesture && mode!=='pan'){const p=world(gesture.x,gesture.y),end=world(gesture.nowX,gesture.nowY),angle=Math.atan2(end[1]-p[1],end[0]-p[0]);arrow(...p,angle,'#edb545',35);}
}
function renderVehicleConnection(disconnected) {
 const reconnecting=state.connection_status==='reconnecting';
 $('vehicle-offline').hidden = !disconnected;
 if(disconnected)$('vehicle-offline').textContent=reconnecting?'正在重连车端':'实车离线 · 已保存地图';
 $('points-empty-title').textContent = reconnecting?'等待车端更新':disconnected?'点位离线':'暂无导航点';
 if (disconnected) {
  if(!reconnecting)collectionDirty=false;
  if (mode !== 'pan') { gesture = null; setMode('pan'); }
 }
 $('stop').disabled=disconnected&&!state.stop_token;
 ['add-tool','pose-tool','collection-export','export','import','anchor-notes','anchor-height']
  .forEach(id => { $(id).disabled = disconnected; });
 for (const [link, href] of vehicleLinks) {
  if (disconnected && link.hasAttribute('href')) {
   link.removeAttribute('href');
   link.setAttribute('aria-disabled','true');
  } else if (!disconnected && !link.hasAttribute('href')) {
   link.setAttribute('href',href);
   link.removeAttribute('aria-disabled');
  }
 }
}

function render(){
 syncMapImage();
 const h=state.health,disconnected=state.vehicle_connected===false,sim=state.fleet_simulation;
 const locked=!quiet()||!!sim?.running;
 const fleetActive=state.fleet_paired??h.fleet?.active??!!h.fleet;
 const fleetBlock=state.fleet_control??(fleetActive?(h.fleet?.motion_reason||(!h.fleet?.leased?'等待双车面板确认运行许可':'')):'');
 renderVehicleConnection(disconnected);
 $('simulation-points').hidden=!sim;
 $('simulation-points').setAttribute('aria-pressed',String(showSimulationPoints));
 $('simulation-points').classList.toggle('active',showSimulationPoints);
 $('simulation-points').title=sim?`统一朝向 ${Math.round(sim.points[0].yaw*180/Math.PI)}°`:'';
 canvas.setAttribute('aria-label',sim&&showSimulationPoints?'50 个同朝向采集点与双车实时位置':'场景导航地图');
 if(sim?.running){
  $('add-tool').disabled=$('pose-tool').disabled=true;
  if(mode!=='pan'){gesture=null;setMode('pan');}
 }
 $('fleet-open').hidden=window.location.hostname!=='127.0.0.1'||window.location.port!=='8891';
 const browserUrl=new URL(location.href),local=['127.0.0.1','localhost','[::1]'].includes(browserUrl.hostname);
 browserUrl.port=state.workspace?.site_port||8896;browserUrl.pathname='/';browserUrl.search=browserUrl.hash='';
 const siteUrl=state.site_url||(!local&&h.mode==='robot'?browserUrl.href:'');
 $('site-open').hidden=!siteUrl||!!state.fleet_number;
 if(siteUrl)$('site-open').href=siteUrl;
 $('fleet-wait').hidden=!fleetBlock;
 $('fleet-wait').textContent=fleetBlock;
 $('vehicle-title').textContent=state.fleet_number===1?'一号车':state.fleet_number===2?'二号车':'机器人巡航';
 $('offline-banner').hidden=h.mode!=='offline';
 $('map-live').textContent=state.connection_status==='reconnecting'?'正在重连':disconnected?'已保存':'演练';
 $('map-live').hidden=!disconnected&&h.mode!=='offline';
 const dockStatus=h.dock_localization?.status;
 const calibrating=dockStatus==='applying'&&!String(h.readiness||'').includes('停车锁定');
 const dockReview=h.charging_stable&&['alignment_required','attention'].includes(dockStatus);
 $('phase').textContent=state.phase==='idle'&&h.charging_stable?'桩上充电':phaseNames[state.phase]||state.phase;
 $('message').textContent=state.message;
 $('message').hidden=disconnected||!state.message||state.message==='请添加导航点，核对定位后开始'||
  (h.charging_stable&&(state.message.includes('已按保存的充电桩位自动校准定位')||state.message.includes('已用雷达复核定位')));
 $('voltage').textContent=h.voltage==null?'—':h.voltage.toFixed(2)+' V';
 $('charging').textContent=h.charging==null?'—':h.charging?'充电中':'未充电';
 $('charge-current').textContent=h.charging&&h.current!=null?h.current.toFixed(2)+' A':'';
 $('remaining').textContent=state.remaining==null?'—':Math.ceil(state.remaining)+' 秒';
 $('progress').hidden=disconnected||!state.run.length;
 $('progress').textContent='点位 '+Math.min(state.index+1,state.run.length)+' / '+state.run.length;
 $('readiness').hidden=disconnected||(!calibrating&&h.ready);
 $('readiness').textContent=calibrating?'正在校准桩位…':h.readiness;
 $('loc-badge').textContent=dockReview?'桩位待复核':h.localization_confirmed?(dockStatus==='confirmed'?'桩位已校准':'已定位'):'待定位';
 $('loc-badge').className='small-badge'+(dockReview?' attention':h.localization_confirmed?' confirmed':'');
 $('map-version').textContent=Number((state.map.resolution*100).toFixed(3))+' cm / 格';
 $('map-version').title=state.map.version;
 $('point-count').textContent=disconnected&&state.connection_status!=='reconnecting'?'—':state.points.length;
 $('empty-points').hidden=state.points.length>0;
 $('start').disabled=locked||!h.ready||!h.localization_confirmed||!state.points.some(p=>p.enabled);$('confirm').disabled=locked||!h.ready;$('initial-dock').disabled=locked;$('pause').disabled=!['navigating','returning','departing','dwelling','collecting','docking','settling','settling_return','settling_departure'].includes(state.phase);$('resume').disabled=!state.phase.startsWith('paused_')||!h.ready||!h.localization_confirmed;$('return').disabled=state.phase==='canceling'||(!h.charging&&(!h.ready||!h.localization_confirmed));$('add-point').disabled=locked;$('apply-heading').disabled=locked;$('recover-points').disabled=locked||!state.checkpoint_available;$('shared-heading').disabled=locked;$('auto-return').disabled=locked;
 const c=state.collection||{mode:'timed',stage:'disabled'}, c5=state.c5||{};
 const repeatMode=['timed','broadcast_station'].includes(c.mode)&&!c5.enabled;
 $('repeat').disabled=locked||!repeatMode;
 if(locked)$('repeat').checked=!!state.repeat;
 else if(!repeatMode)$('repeat').checked=false;
 $('endurance-status').hidden=!state.recharge_resume_pending&&!state.recharge_resume_enabled;
 $('endurance-status').textContent=state.recharge_resume_pending?'充电后续采':
  state.recharge_resume_repeat_only?'循环时自动回充续采':'低电量回充续采已启用';
 if(!disconnected&&state.repeat&&state.run.length)$('progress').textContent='第 '+state.lap+' 轮 · '+$('progress').textContent;
 if(!collectionDirty){$('collection-mode').value=c.mode;$('collection-device').value=c.device||'auto';}
 const selectedCollection=$('collection-mode').value;
 const fas=state.fas||{};
 $('dual-preflight').textContent=fas.motion_only?'本车处于无采集测试模式 · 双频发包关闭':
   fas.car&&fas.protocol?'本车已配置双频实验 · 基站收包待静止联调验证':'等待配置车端双频实验';
 const activeCollection=c.mode!=='timed'||!!c5.enabled;
 $('collection-summary').textContent=state.connection_status==='reconnecting'?'等待更新':disconnected?'离线':collectionDirty?'待应用':
  [collectionNames[c.mode],c5.enabled?'C5 CSI':null].filter(Boolean).join(' + ')||'关闭';
 $('collection-summary').classList.toggle('confirmed',!disconnected&&activeCollection&&!collectionDirty);
 $('serial-field').hidden=selectedCollection==='timed';
 $('collection-ports').innerHTML=(c.ports||[]).map(p=>'<option value="'+esc(p)+'"></option>').join('');
 $('collection-mode').disabled=$('collection-device').disabled=locked||h.mode==='offline'||!!c5.active;
 $('collection-apply').disabled=$('collection-mode').disabled||(!collectionDirty&&!activeCollection);
 $('collection-apply').textContent=!collectionDirty&&activeCollection?'重新连接':
  activeCollection?(selectedCollection==='timed'?'关闭采集':'应用设置'):'启用采集';
 $('collection-apply').classList.toggle('primary',collectionDirty);
 $('collection-status').hidden=disconnected||(!collectionDirty&&c.mode==='timed'&&!c.motion_blocked);
 $('collection-status').textContent=c.mode==='timed'?'':c.error?'WS63：'+c.error:c.stage==='sampling'?'采集中 · '+Math.floor(c.elapsed||0)+' 秒':c.connected?'WS63 串口已连接':'WS63 串口未连接';
 ['survey-load','survey-restore','anchor-save'].forEach(k=>$(k).disabled=locked);
 const loopReady=!state.points.some(p=>p.id.startsWith("north-loop-"))||c.profile==="moving_loop_demo";
 const c5Ready=!c5.active&&(!c5.enabled||['ready','done','canceled'].includes(c5.stage));
 const broadcastMode=['broadcast','broadcast_station'].includes(c.mode);
 const radioReady=c.connected&&(['ready','done'].includes(c.stage)||(c.mode==='broadcast_station'&&(c.stage==='canceled'||c.retryable)));
 const collectionReady=loopReady&&!collectionDirty&&!c.motion_blocked&&c5Ready&&(!broadcastMode||c.beacon_recent)&&(!['ws63','broadcast','broadcast_station','servo_car_v2'].includes(c.mode)||radioReady);
 $('return').disabled=$('return').disabled||c.motion_blocked||!!c5.active||(c5.enabled&&['collecting','paused_collection'].includes(state.phase));
 if(c.mode==='broadcast'&&!collectionDirty)$('collection-status').textContent=c.error?'WS63：'+c.error:c.connected?(c.beacon_recent?'基站1 '+c.last_rssi+' dBm · 沿途有效 '+(c.samples||0)+' 条 · '+(c.measured_points||0)+' 个热力格' :'Client 已连接，等待固定基站广播'):'广播 Client：'+c.stage;
 if(c.mode==='broadcast_station'&&!collectionDirty)$('collection-status').textContent=c.error?'WS63：'+c.error:c.connected?(c.beacon_recent?'基站 1 '+c.last_rssi+' dBm · 本站 '+(c.samples||0)+' 条 · 已完成 '+(c.measured_points||0)+' 站次':'接收板已连接，等待固定基站广播'):'广播接收端：'+c.stage;
 if(c.mode==='servo_car_v2')$('collection-status').textContent=servoStatus(c);
 else if(c.motion_blocked)$('collection-status').textContent='有未放行的旋转轮次。请切到旋转定点模式恢复连接。';
 if(collectionDirty)$('collection-status').textContent='设置尚未应用';
 if(fleetActive){
  autoReturnBeforePair??=$('auto-return').checked;
  $('auto-return').checked=false;$('auto-return').disabled=true;
 }else if(autoReturnBeforePair!==undefined){
  $('auto-return').checked=autoReturnBeforePair;autoReturnBeforePair=undefined;
 }
 $('initial-dock').disabled=$('initial-dock').disabled||h.dock_calibrated===false;
 if($('align-pose'))$('align-pose').disabled=locked||!h.stationary||!h.pose||h.mode!=='robot';
 $('return').disabled=$('return').disabled||fleetActive||!!fleetBlock||h.dock_calibrated===false;
 const cancelReturn=canCancelReturn();
 $('return').lastChild.nodeValue=cancelReturn?'取消回充':'主动回充';
 if(cancelReturn) $('return').disabled=disconnected;

 $('resume').disabled=$('resume').disabled||!!fleetBlock;
 $('start').disabled=$('start').disabled||!collectionReady||!!fleetBlock;
 $('recover-points').hidden=!state.checkpoint_available;
 renderPoints(locked);document.querySelectorAll('#points .go').forEach(b=>b.disabled=locked||!h.ready||!h.localization_confirmed||!collectionReady||!!fleetBlock);$('events').innerHTML=state.events.slice(0,6).map(e=>'<div class="activity-item"><span class="activity-time">'+esc(e.time)+'</span><span>'+esc(e.text)+'</span></div>').join('')||'<div class="hint">暂无记录</div>';document.dispatchEvent(new CustomEvent('cruise-state',{detail:state}));draw();}
function renderPoints(locked){const key=JSON.stringify(state.points)+selected+locked;if(pointsKey===key || (!locked && $('points').contains(document.activeElement) && document.activeElement.tagName==='INPUT'))return;pointsKey=key;$('points').innerHTML=state.points.map((p,i)=>'<div class="point-card '+(selected===p.id?'selected':'')+'" data-id="'+esc(p.id)+'"><div class="point-head"><span class="point-index">'+(i+1)+'</span><input class="point-name" aria-label="点位名称" data-field="name" value="'+esc(p.name)+'" '+(locked?'disabled':'')+'><input class="point-check" aria-label="加入巡航" type="checkbox" data-field="enabled" '+(p.enabled?'checked ':'')+(locked?'disabled':'')+'></div><div class="point-position">x '+p.x.toFixed(2)+' m · y '+p.y.toFixed(2)+' m</div><div class="point-fields"><label>停留（秒）<input type="number" data-field="wait" min="0" max="3600" step="1" value="'+p.wait+'" '+(locked?'disabled':'')+'></label><label>到达朝向（度）<input type="number" data-field="yaw" min="-180" max="180" step="0.1" value="'+Number((p.yaw*180/Math.PI).toFixed(1))+'" '+(locked?'disabled':'')+'></label></div><div class="point-actions"><button data-action="up" '+(locked||i===0?'disabled':'')+'>↑</button><button data-action="down" '+(locked||i===state.points.length-1?'disabled':'')+'>↓</button><button data-action="move" '+(locked?'disabled':'')+'>地图移动</button><button data-action="delete" '+(locked?'disabled':'')+'>删除</button><button class="go" data-action="go" '+(locked||!state.health.ready||!state.health.localization_confirmed?'disabled':'')+'>前往 ↗</button></div></div>').join('');}
$('points').addEventListener('click',e=>{const card=e.target.closest('.point-card');if(!card)return;selected=card.dataset.id;draw();const action=e.target.dataset.action;if(!action){pointsKey='';return;}const points=structuredClone(state.points),i=points.findIndex(p=>p.id===selected);if(action==='go')command({action:'go',id:selected});else if(action==='move')setMode('move');else if(action==='delete')command({action:'save_points',points:points.filter(p=>p.id!==selected)});else{const j=action==='up'?i-1:i+1;[points[i],points[j]]=[points[j],points[i]];command({action:'save_points',points});}});
$('points').addEventListener('change',e=>{const card=e.target.closest('.point-card'),field=e.target.dataset.field;if(!card||!field)return;const points=structuredClone(state.points),p=points.find(p=>p.id===card.dataset.id);p[field]=field==='enabled'?e.target.checked:field==='name'?e.target.value:field==='yaw'?Number(e.target.value)*Math.PI/180:Number(e.target.value);command({action:'save_points',points});});
function local(e){const b=canvas.getBoundingClientRect();return [e.clientX-b.left,e.clientY-b.top];}
canvas.addEventListener('pointerdown',e=>{if(!state)return;const [x,y]=local(e);gesture={x,y,nowX:x,nowY:y,panX,panY};canvas.setPointerCapture(e.pointerId);});
canvas.addEventListener('pointermove',e=>{if(!state)return;const [x,y]=local(e),p=world(x,y);$('coordinates').textContent='x '+p[0].toFixed(2)+' m · y '+p[1].toFixed(2)+' m';if(gesture){gesture.nowX=x;gesture.nowY=y;if(mode==='pan'){panX=gesture.panX+x-gesture.x;panY=gesture.panY+y-gesture.y;}draw();}});
canvas.addEventListener('pointerup',e=>{if(!gesture)return;const g=gesture;gesture=null;const [x,y]=local(e),distance=Math.hypot(x-g.x,y-g.y);if(mode==='pan'){if(distance<5){const p=state.points.find(v=>{const s=screen(v.x,v.y);return Math.hypot(x-s[0],y-s[1])<16;});selected=p?.id||null;pointsKey='';render();}return;}if(!quiet()){toast('请先停止任务，再修改点位或定位');setMode('pan');return;}const start=world(g.x,g.y),end=world(x,y),angle=distance>8?Math.atan2(end[1]-start[1],end[0]-start[0]):state.health.pose?.[2]||0;if(mode==='pose'){command({action:'initial_pose',pose:[...start,angle]});setMode('pan');return;}const points=structuredClone(state.points);if(mode==='move'){const p=points.find(p=>p.id===selected);if(!p)return;p.x=start[0];p.y=start[1];if(distance>8)p.yaw=angle;}else{const p={id:(Date.now().toString(36)+Math.random().toString(36).slice(2,12)),name:'点位 '+(points.length+1),x:start[0],y:start[1],yaw:angle,wait:10,enabled:true};points.push(p);selected=p.id;}command({action:'save_points',points});draw();});
canvas.addEventListener('wheel',e=>{e.preventDefault();const [x,y]=local(e),factor=Math.exp(-e.deltaY*.001);const newZoom=Math.min(18,Math.max(.25,zoom*factor)),ratio=newZoom/zoom;panX=x-(x-panX)*ratio;panY=y-(y-panY)*ratio;zoom=newZoom;draw();},{passive:false});
$('pan-tool').onclick=()=>setMode('pan');$('add-tool').onclick=$('add-point').onclick=()=>setMode('add');$('pose-tool').onclick=()=>setMode('pose');$('fit').onclick=()=>fit(false);$('full').onclick=()=>fit(true);
$('simulation-points').onclick=()=>{
 showSimulationPoints=!showSimulationPoints;
 gesture=null;setMode('pan');render();fit(false);
};
['stop','pause','resume','confirm'].forEach(id=>$(id).onclick=()=>command({action:id}));
$('return').onclick=()=>command({action:canCancelReturn()?'stop':'return'});
$('initial-dock').onclick=()=>command({action:'initial_dock'});$('start').onclick=()=>command({action:'start',auto_return:$('auto-return').checked,repeat:$('repeat').checked});
if($('align-pose'))$('align-pose').onclick=()=>command({action:'align_pose'});
$('export').onclick=async()=>{const data=await(await fetch('/api/export')).json();const link=document.createElement('a'),url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'}));link.href=url;link.download='实验室导航点.json';link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};$('import').onclick=()=>$('file').click();$('file').onchange=async e=>{try{const data=JSON.parse(await e.target.files[0].text());command({action:'import',map_id:data.map_id,points:data.points});}catch(err){toast('点位文件无法读取：'+err.message);}e.target.value='';};
function updateCollectionForm(){
 if(!state)return;
 const mode=$('collection-mode').value,c=state.collection||{mode:'timed',device:'auto'};
 const device=$('collection-device').value.trim()||'auto';
 collectionDirty=mode!==c.mode||!!state.c5?.enabled||
  (mode!=='timed'&&device!==(c.device||'auto'));
 render();
}
$('collection-device').oninput=updateCollectionForm;
$('collection-mode').onchange=()=>{
 updateCollectionForm();
};
$('collection-apply').onclick=async()=>{
 const data={action:'collection_config',mode:$('collection-mode').value,device:$('collection-device').value.trim()||'auto',c5:false};
 if(await command(data)){collectionDirty=false;render();}
};
$('survey-load').onclick=async()=>{await command({action:'survey_load_north'});fit(false);};
$('survey-restore').onclick=async()=>{await command({action:'survey_restore_points'});fit(false);};
$('anchor-save').onclick=()=>command({action:'survey_anchor',notes:$('anchor-notes').value,height_m:$('anchor-height').value?Number($('anchor-height').value):null});
function c5Status(c5) {
 if(c5.error)return 'C5：'+c5.error;
 if(c5.stage==='starting')return 'C5：'+c5.point_name+' 已停稳，正在启动采集';
 if(c5.stage==='sampling')return 'C5：'+c5.point_name+' 正在采集 · '+(c5.elapsed||0)+' 秒';
 if(c5.stage==='done')return 'C5：'+c5.point_name+' 已保存 · 有效 CSI '+c5.result.valid_csi+' 条';
 if(c5.stage==='canceling')return 'C5：正在停止并保存已采数据';
 if(c5.stage==='canceled')return 'C5：本轮已取消';
 return c5.enabled?'C5 采集端就绪 · '+(c5.plan==='variable'?'可变状态扫描':'固定状态采集'):'';
}
function servoStatus(collection) {
 if(collection.error)return collection.error;
 if(collection.stage==='sampling'){
  const stage=collection.waiting_move?'已写盘，等待移动许可':'已收 '+(collection.node_count||0)+'/2 个角度';
  return '点位 '+collection.position_id+' · 第 '+collection.round_id+' 轮 · '+stage+' · '+Math.floor(collection.elapsed||0)+' 秒';
 }
 if(collection.stage==='done')return '两个角度均已保存，收到移动许可';
 if(collection.stage==='submitting')return '小车已停稳，正在触发旋转测量';
 if(collection.stage==='ready')return 'SERVO_CAR_2 串口和 SLE 链路已就绪';
 if(collection.stage==='waiting_peer')return collection.link_ready?'等待 Client 完成上一轮':'等待 Client 连接旋转基站';
 if(collection.stage==='canceling')return '正在停止本地采集';
 return '正在检查 SERVO_CAR_2 串口';
}
$('collection-export').onclick=()=>{
 const a=document.createElement('a'),mode=state.collection?.mode;
 if(mode==='servo_car_v2'){
  a.href='/api/collection/servo/export';a.download='WS63旋转定点采集与地图.zip';
 }else{
  a.href=mode==='broadcast'?'/api/broadcast/export':'/api/collection/bundle';
  a.download=mode==='broadcast'?'场景RSSI热力图与数据.zip':'WS63采集位置与地图.zip';
 }
 a.click();
};
new ResizeObserver(()=>{if(!fitted)fitted=fit(false);else draw();}).observe(canvas);
window.addEventListener('keydown',e=>{if(e.key==='Escape' && state)command({action:'stop'});});poll();

$('apply-heading').onclick=()=>{const degrees=Number($('shared-heading').value);if(!Number.isFinite(degrees)||degrees < -180||degrees > 180){toast('请输入-180～180度的朝向');return;}command({action:'save_points',points:state.points.map(p=>({...p,yaw:degrees*Math.PI/180}))});};

$('recover-points').onclick=()=>command({action:'recover_points'});
