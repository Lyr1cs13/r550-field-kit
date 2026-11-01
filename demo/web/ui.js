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
    last=Date.now();const h=event.detail.health;
    connection.classList.toggle('online',!!h.ready);
    connection.classList.toggle('offline-mode',h.mode==='offline');
    connection.querySelector('span').textContent=h.mode==='offline'?'模拟器就绪':h.ready?'系统就绪':'系统准备中';
  });
  setInterval(()=>{
    if(last && Date.now()-last>2500){connection.classList.remove('online');connection.querySelector('span').textContent='连接中断';}
  },1000);
})();
