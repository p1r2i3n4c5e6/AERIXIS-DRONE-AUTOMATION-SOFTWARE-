/* Visual accents, vector task icons and entrance motion only. */
(() => {
 'use strict';
 const accents=['#78a5ff','#64d4a5','#b49aff','#ef8294','#54cddd','#c38ae7'];
 function init(){
  const panels=document.querySelectorAll('.sidebar > .card,.glass-panel,.mission-capabilities article');
  panels.forEach((panel,i)=>{panel.style.setProperty('--panel-accent',accents[i%accents.length]);panel.classList.add('spectrum-panel');});
  for(const [i,button]of [...document.querySelectorAll('.operations-nav .view-tab-btn')].entries())button.style.setProperty('--nav-accent',accents[i%accents.length]);
  const tasks=['radio','shield','radio','takeoff','route','sequence'];
  [...document.querySelectorAll('.sidebar>.card')].filter(card=>!card.hidden&&getComputedStyle(card).display!=='none').forEach((card,i)=>{
   const h=card.querySelector('.card-head h2');if(!h)return;h.textContent=h.textContent.replace(/^[^A-Za-z]+/,'').replace(/^Vehicle Status$/,'Local vehicle status').replace(/^Flight Commands$/,'Local flight commands');const type=tasks[i%tasks.length],symbol=document.createElement('span');symbol.className='panel-task-icon';symbol.append(window.GCSTaskIcons.element(type,'',type==='sequence'?'Control':''));h.prepend(symbol);
  });
  const localIcons=['action','drone','shield'];document.querySelectorAll('.pipeline-tier-box .tier-icon').forEach((n,i)=>n.replaceChildren(window.GCSTaskIcons.element(localIcons[i]||'radio')));
  document.querySelectorAll('.hud-card small').forEach((n,i)=>n.textContent=['Local altitude','Local mode','Gateway link'][i]||n.textContent);
  const radar=document.querySelector('.radar svg');if(radar)radar.replaceWith(window.GCSTaskIcons.element('drone'));
  // Animate only when a pane is actually opened, not when live values refresh.
  const arrive=node=>{node.classList.remove('panel-arrive');void node.offsetWidth;node.classList.add('panel-arrive');};
  document.querySelectorAll('.detail-panel').forEach(panel=>new MutationObserver(records=>{if(records.some(r=>r.attributeName==='hidden')&&!panel.hidden)arrive(panel);}).observe(panel,{attributes:true,attributeFilter:['hidden']}));
  document.querySelectorAll('dialog').forEach(panel=>new MutationObserver(()=>{if(panel.open)arrive(panel);}).observe(panel,{attributes:true,attributeFilter:['open']}));
  const studio=document.getElementById('view-tree');new MutationObserver(()=>{const pane=studio.dataset.mode==='live'?studio.querySelector('.control-board'):studio.querySelector('.radio-plan-panel');if(pane)arrive(pane);}).observe(studio,{attributes:true,attributeFilter:['data-mode']});
 }
 if(document.readyState==='complete')init();else document.addEventListener('DOMContentLoaded',init,{once:true});
})();
