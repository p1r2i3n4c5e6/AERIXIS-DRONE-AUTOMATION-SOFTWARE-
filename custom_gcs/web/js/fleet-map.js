/* Read-only fleet map. Uses existing refreshes; never polls or commands an aircraft. */
(() => {
 'use strict';
 const colors=['#6fa5ff','#64d4a5','#ee8096','#b49aff','#54d4e4','#f0aa79'],M=window.GCSMapPosition;
 const rows=new Map(),tracks=new Map(),make=(tag,cls,text)=>{const n=document.createElement(tag);n.className=cls||'';if(text!=null)n.textContent=text;return n;};
 let map,local=null,localAt=0,fleet=[],fleetAt=0,auto=true,lastFit=0,signature='',list,status,follow,resizeObserver;
 function iconMarkup(color){return `<div class="aircraft-map-symbol" style="--aircraft-color:${color}"><svg viewBox="0 0 96 96" fill="none" aria-hidden="true"><path class="drone-shadow" d="M30 30 66 66M66 30 30 66"/><path class="drone-arm" d="M32 31 64 65M64 31 32 65"/><g class="drone-rotors">${[[23,23],[73,23],[23,73],[73,73]].map(([x,y])=>`<ellipse cx="${x}" cy="${y}" rx="18" ry="15"/><path d="M${x-13} ${y-3}q13-8 26 6M${x-13} ${y+3}q13 8 26-6"/><circle cx="${x}" cy="${y}" r="4"/>`).join('')}</g><path class="drone-body" d="m48 26 12 14 2 17-14 12-14-12 2-17z"/><path class="drone-nose" d="m48 29 7 11H41z"/><path class="drone-body-detail" d="M40 49h16M41 56h14M39 65l-6 9m24-9 6 9"/><circle class="drone-camera" cx="48" cy="62" r="4"/></svg></div>`;}
 function init(){
  const host=document.querySelector('#view-map .content-area');if(!host)return;
  const bar=make('div','fleet-map-toolbar'),title=make('div','fleet-map-title');title.append(make('strong','','Fleet positioning'));status=make('small','','Waiting for live GPS');title.append(status);
  const fit=make('button','','Fit aircraft');fit.type='button';fit.onclick=()=>fitAll(true);follow=make('button','active','Auto-fit fleet');follow.type='button';follow.setAttribute('aria-pressed','true');follow.onclick=()=>{auto=!auto;updateFollow();if(auto)fitAll(true);};bar.append(title,fit,follow);host.append(bar);
  for(const id of ['draw-area-btn','add-waypoints-btn','load-kml-btn'])document.getElementById(id)?.addEventListener('click',()=>{auto=false;updateFollow();});
  const panel=make('section','fleet-map-roster');panel.setAttribute('aria-label','Aircraft GPS status');panel.append(make('h3','','Aircraft positions'));list=make('div','fleet-map-rows');panel.append(list);host.append(panel);
  new MutationObserver(()=>{if(document.getElementById('view-map').classList.contains('active')&&map){requestAnimationFrame(()=>{map.invalidateSize();if(auto)fitAll(true);});}}).observe(document.getElementById('view-map'),{attributes:true,attributeFilter:['class']});
  try{if(typeof mapInstance!=='undefined'&&mapInstance)attach(mapInstance);}catch{}
  update();setInterval(update,1000);
 }
 function updateFollow(){follow?.classList.toggle('active',auto);follow?.setAttribute('aria-pressed',String(auto));}
 function attach(instance){if(map===instance)return;map=instance;map.on('dragstart',()=>{auto=false;updateFollow();});map.on('zoomstart',e=>{if(e.originalEvent){auto=false;updateFollow();}});map.getContainer().addEventListener('wheel',()=>{auto=false;updateFollow();},{passive:true});resizeObserver=new ResizeObserver(()=>{map.invalidateSize();});resizeObserver.observe(map.getContainer());update();}
 function fitAll(force=false){if(!map||!tracks.size||!document.getElementById('view-map').classList.contains('active'))return;if(!force&&Date.now()-lastFit<3000)return;lastFit=Date.now();map.invalidateSize();const points=[...tracks.values()].map(t=>t.marker.getLatLng());map.fitBounds(L.latLngBounds(points),{paddingTopLeft:[55,110],paddingBottomRight:[55,map.getSize().y<500?80:210],maxZoom:18,animate:!matchMedia('(prefers-reduced-motion: reduce)').matches,duration:.4});}
 function remove(id){const t=tracks.get(id);if(!t)return;map.removeLayer(t.marker);map.removeLayer(t.trail);tracks.delete(id);}
 function updateTrack(id,label,p,color){
  if(!map)return;let t=tracks.get(id);const point=[p.lat,p.lon];
  if(!t){const marker=L.marker(point,{icon:L.divIcon({className:'fleet-aircraft-marker',html:iconMarkup(color),iconSize:[60,60],iconAnchor:[30,30]}),title:label,keyboard:true}).addTo(map);const tooltip=make('div','aircraft-map-label',label);marker.bindTooltip(tooltip,{permanent:true,direction:'bottom',offset:[0,23],className:'aircraft-map-tooltip'});const trail=L.polyline([],{color,weight:2,opacity:.55}).addTo(map);t={marker,trail,points:[]};tracks.set(id,t);}
  const old=t.points.at(-1);if(!old||old[0]!==p.lat||old[1]!==p.lon){t.points.push(point);if(t.points.length>80)t.points.shift();t.marker.setLatLng(point);t.trail.setLatLngs(t.points);}
  const symbol=t.marker.getElement()?.querySelector('.aircraft-map-symbol');if(symbol){symbol.style.transform=`rotate(${p.heading??0}deg)`;symbol.classList.toggle('heading-known',p.heading!==null);}
  const labelNode=t.marker.getTooltip().getContent();labelNode.textContent=label;
  const popup=make('div','aircraft-position-popup');popup.append(make('strong','',label),make('p','',`${p.lat.toFixed(7)}, ${p.lon.toFixed(7)}`),make('small','',`GPS age ${p.age.toFixed(1)}s${p.altitude===null?'':` · ${p.altitude.toFixed(1)} m reported altitude`}${p.heading===null?' · Heading unavailable':` · Heading ${p.heading.toFixed(0)}°`}`));t.marker.bindPopup(popup);
 }
 function update(){if(!list)return;
  const fresh=Date.now()-fleetAt<5000,localFresh=Date.now()-localAt<3500;
  const liveLocal=localFresh&&!!local?.connected;const hudLink=document.getElementById('hud-link');if(hudLink){hudLink.textContent=liveLocal?'ONLINE':'OFFLINE';hudLink.style.color=liveLocal?'var(--green)':'var(--red)';}
  if(!liveLocal){const alt=document.getElementById('hud-alt'),mode=document.getElementById('hud-mode');if(alt)alt.textContent='—';if(mode)mode.textContent='UNKNOWN';}
  const entries=fleet.map((d,i)=>({id:d.drone_id,label:d.label||d.drone_id,color:colors[i%colors.length],p:fresh?M.fleetPosition(d):null,connection:fresh?d.connection:'STALE',fresh:fresh&&d.fresh}));
  // Local gateway is separately identified, never assigned to an arbitrary fleet drone.
  const localPoint=localFresh?M.localPosition(local):null;if(localPoint)entries.push({id:'__local_gateway__',label:'Local gateway aircraft',color:'#54d4e4',p:localPoint,connection:'CONNECTED',fresh:true});
  const valid=entries.filter(e=>e.p),ids=new Set(valid.map(e=>e.id));for(const id of tracks.keys())if(!ids.has(id))remove(id);
  for(const e of valid)updateTrack(e.id,e.label,e.p,e.color);
  try{if(typeof kmlMapMode!=='undefined'&&kmlMapMode){auto=false;updateFollow();}}catch{}
  const newSignature=JSON.stringify(valid.map(e=>e.id).sort());const changed=newSignature!==signature;signature=newSignature;
  status.textContent=valid.length?`${valid.length} live GPS ${valid.length===1?'track':'tracks'} · WGS84`:'No fresh aircraft GPS · no live markers';
  for(const id of rows.keys())if(!entries.some(e=>e.id===id)){rows.get(id).remove();rows.delete(id);}
  for(const e of entries){let row=rows.get(e.id);if(!row){row=make('button','fleet-map-aircraft');row.type='button';const symbol=make('span','roster-aircraft-symbol');symbol.innerHTML=iconMarkup(e.color);row.append(symbol,make('strong',''),make('small',''));list.append(row);rows.set(e.id,row);}row.querySelector('strong').textContent=e.label;row.querySelector('small').textContent=e.p?`${e.p.lat.toFixed(6)}, ${e.p.lon.toFixed(6)}`:e.fresh?'Connected · GPS unavailable':`${e.connection||'DISCONNECTED'} · no live position`;row.disabled=!e.p;row.dataset.live=e.p?'true':'false';row.onclick=()=>{if(!e.p||!map)return;auto=false;updateFollow();map.setView([e.p.lat,e.p.lon],18,{animate:!matchMedia('(prefers-reduced-motion: reduce)').matches});tracks.get(e.id)?.marker.openPopup();};}
  let empty=list.querySelector('.map-gps-empty');if(!entries.length&&!empty){empty=make('p','map-gps-empty','Connect aircraft to see their GPS status. Markers appear after a fresh position fix is received.');list.append(empty);}else if(entries.length)empty?.remove();
  if(auto&&valid.length)fitAll(changed);
 }
 window.GCSFleetMap={attach,updateFleet(drones){fleet=drones||[];fleetAt=Date.now();update();},updateLocal(s){local=s;localAt=Date.now();update();},localUnavailable(){localAt=0;update();}};
 if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',init);else init();
})();
