'use strict';
// Presentation only. This file never sends robot commands.
(() => {
  const root=document.body;
  try {root.dataset.bsTheme=localStorage.getItem('cruise-theme')||'light';} catch (_) {}
  const toggle=document.getElementById('theme-toggle');
  const updateIcon=()=>{toggle.querySelector('use').setAttribute('href','/vendor/icons.svg#'+(root.dataset.bsTheme==='dark'?'sun':'moon'));};
  updateIcon();
  toggle.addEventListener('click',()=>{
    root.dataset.bsTheme=root.dataset.bsTheme==='dark'?'light':'dark';updateIcon();
    try {localStorage.setItem('cruise-theme',root.dataset.bsTheme);} catch (_) {}
  });
  let last=0;
  const connection=document.getElementById('connection-state');
  document.addEventListener('cruise-state',event=>{
    last=performance.now();const h=event.detail.health;
    const disconnected=event.detail.vehicle_connected===false;
    const reconnecting=event.detail.connection_status==='reconnecting';
    connection.classList.toggle('online',!disconnected&&!!h.ready);
    connection.classList.toggle('offline-mode',disconnected||h.mode==='offline');
    connection.querySelector('span').textContent=reconnecting?'正在重连':disconnected?'离线':h.mode==='offline'?'演练':h.ready?'在线':'准备中';
  });
  setInterval(()=>{
    const age=performance.now()-last;
    if(last && age>2500){
      connection.classList.remove('online');
      connection.querySelector('span').textContent=age>=6000?'面板未连接':'面板重连中';
    }
  },1000);
})();
