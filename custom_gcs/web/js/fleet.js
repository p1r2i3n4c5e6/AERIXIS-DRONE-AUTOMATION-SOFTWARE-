/* Fleet operations: laptop planning, independent onboard execution. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const el = (tag, cls, text) => { const n = document.createElement(tag); if (cls) n.className = cls; if (text !== undefined) n.textContent = text; return n; };
  let drones = [], selected = '', busy = false, polling = false, apiOK = false;
  const drafts = new Map(), seen = new Set(), connections = new Map();
  const planned = new Map();
  let activity = [], treeKey = '', transform = {x: 20, y: 20, scale: 1};
  const current = () => drones.find(d => d.drone_id === selected);
  async function api(path, body) {
    const response = await fetch(path, {method: body ? 'POST' : 'GET', cache: 'no-store',
      headers: {'Content-Type': 'application/json', 'X-AS2-GCS': '1'},
      body: body ? JSON.stringify(body) : undefined, signal: AbortSignal.timeout(body ? 65000 : 12000)});
    const data = await response.json();
    if (!response.ok || data.error) throw new Error(data.error || `GCS HTTP ${response.status}`);
    return data;
  }
  function toast(message, level = 'INFO') {
    const node = el('div', `gcs-toast ${level}`);
    node.append(el('span', '', level === 'ERROR' ? '!' : '✓'), el('span', '', message));
    const close = el('button', '', '×'); close.setAttribute('aria-label', 'Dismiss notification'); close.onclick = () => node.remove(); node.append(close);
    $('gcs-toasts').append(node); setTimeout(() => node.remove(), 7000);
    while ($('gcs-toasts').children.length > 4) $('gcs-toasts').firstChild.remove();
  }
  function record(drone, level, message, id, time = Date.now()/1000) {
    if (id && seen.has(id)) return;
    if (id) {seen.add(id); if (seen.size > 10000) seen.delete(seen.values().next().value);}
    activity.unshift({drone, level, message, time}); activity = activity.slice(0, 400);
  }
  function error(message, context = selected || 'GCS') {
    $('error-title').textContent = 'Operation needs attention'; $('error-message').textContent = String(message);
    $('error-context').textContent = context; record(context, 'ERROR', String(message));
    if (!$('fleet-error-dialog').open) $('fleet-error-dialog').showModal(); renderActivity();
  }
  function badge(node, status) {
    node.className = `connection-badge ${status}`; node.replaceChildren();
    if (status === 'CONNECTING') node.append(el('i', 'spinner'));
    else if (status === 'CONNECTED') node.append(el('i', 'signal-dot'));
    node.append(document.createTextNode(status));
  }
  function select(id) {
    if($('operator-load-check'))$('operator-load-check').checked=false;
    if($('operator-payload-id'))$('operator-payload-id').value='';
    if (selected) drafts.set(selected, {xml:$('fleet-xml').value, label:$('xml-file-label').textContent});
    selected = id; const draft = drafts.get(id) || {xml:'',label:'No file selected'};
    $('fleet-xml').value = draft.xml; $('xml-file-label').textContent = draft.label;
    treeKey = ''; render();
  }
  function renderActivity() {
    const filter = $('activity-filter').value;
    const entries = activity.filter(e => filter === 'all' || (filter === 'errors' ? ['ERROR','WARN'].includes(e.level) : e.drone === selected));
    const fragment = document.createDocumentFragment();
    for (const item of entries.slice(0, 100)) {
      const row = el('article', `activity-entry ${item.level}`), meta = el('div', 'activity-meta');
      meta.append(el('i', 'event-dot'), el('span', '', item.drone || 'GCS'), el('time', '', new Date(item.time*1000).toLocaleTimeString()));
      row.append(meta, el('p', '', item.message)); fragment.append(row);
    }
    if (!entries.length) fragment.append(el('p', 'small-empty', 'Mission events and connection updates appear here.'));
    $('fleet-activity').replaceChildren(fragment);
  }
  function render() {
    try { window.GCSFleetMap?.updateFleet(drones); } catch {}
    try { window.GCSLinkDisplay?.updateFleet(drones, selected); } catch {}
    try { window.GCSLive?.updateFleet(drones, selected); } catch (error) { console.warn('Execution display unavailable', error); }
    const fresh = drones.filter(d => d.fresh), ros = fresh.filter(d => d.data?.ros?.available);
    $('fleet-connected').replaceChildren(document.createTextNode(`${fresh.length} `), el('em', '', `/ ${drones.length}`));
    $('fleet-ros-total').textContent = ros.length ? ros.reduce((a,d) => a+d.data.ros.count, 0) + (ros.length < drones.length ? '+' : '') : '—';
    $('fleet-bt-total').textContent = fresh.length ? fresh.reduce((a,d) => a+d.data.bt_running_count, 0) + (fresh.length < drones.length ? '+' : '') : '—';
    $('fleet-attention').textContent = drones.filter(d => !d.fresh || d.data?.hold_lockout || ['FAILURE','ERROR','INTERRUPTED'].includes(d.data?.execution?.state)).length;
    $('aircraft-count').textContent = `${drones.length} REGISTERED`;
    const cards = document.createDocumentFragment();
    for (const d of drones) {
      const card = el('button', `aircraft-card${d.drone_id === selected ? ' selected' : ''}`);
      card.onclick = () => select(d.drone_id); card.setAttribute('aria-pressed', String(d.drone_id === selected));
      const top = el('div', 'aircraft-card-top'), names = el('div'); names.append(el('h3','',d.label), el('small','',d.drone_id));
      const glyph=el('div','aircraft-glyph');if(window.GCSTaskIcons)glyph.append(window.GCSTaskIcons.element('drone'));
      top.append(glyph,names); const connection = el('span'); badge(connection, d.connection);
      const counts = el('div','card-node-counts'); counts.append(el('span','',`ROS ${d.fresh && d.data?.ros?.available ? d.data.ros.count : '—'}`),el('span','',`BT running ${d.fresh ? d.data.bt_running_count : '—'}`));
      card.append(top,connection,counts); cards.append(card);
    }
    if (!drones.length) cards.append(el('p','small-empty','No aircraft added yet.'));
    $('fleet-aircraft-list').replaceChildren(cards);
    const d = current(); $('fleet-empty').hidden = !!d; $('fleet-selected').hidden = !d;
    if (!d) return renderActivity();
    const data = d.data || {}, mission = data.mission, execution = data.execution;
    $('selected-label').textContent = d.label; $('selected-endpoint').textContent = `${d.drone_id} · ${d.endpoint}`;
    badge($('selected-connection'),d.connection);
    $('selected-ros-count').textContent = d.fresh && data.ros?.available ? data.ros.count : '—';
    $('selected-bt-count').textContent = d.fresh ? `${data.bt_running_count} / ${(data.bt_nodes || []).length}` : '—';
    $('selected-mission-state').textContent = !d.fresh ? 'UNKNOWN' : execution?.state || (mission ? 'STAGED' : 'No mission');
    $('selected-age').textContent = d.age_s == null ? 'Never' : `${d.age_s.toFixed(1)}s ago`;
    $('selected-revision').textContent = mission ? mission.revision.slice(0,16) : 'Nothing uploaded';
    $('selected-revision').title = mission?.revision || '';
    $('selected-policy').textContent = data.flight_enabled ? 'Flight enabled onboard' : 'Observation only';
    const rescue=data.ros?.rescue;
    if(!$('payload-truth')){const node=el('p','mission-notice');node.id='payload-truth';$('rescue-health-strip').after(node);}
    $('payload-truth').textContent=d.fresh&&rescue?.fresh?'Payload: '+(rescue.data.payload?.truth||'UNKNOWN'):'Payload: UNKNOWN · stale aircraft state';
    $('rescue-health-strip').textContent = d.fresh && rescue?.fresh ? 'Rescue: '+(rescue.data.active_operation || 'monitoring')+' · '+Object.entries(rescue.data.flags || {}).filter(([k,v])=>v===false && ['CameraFresh','GimbalStable','TrackConfirmed','PayloadLoaded','PeersFresh','SeparationSafe'].includes(k)).map(([k])=>k+' unavailable').join(' · ') : 'Rescue supervisor unavailable or stale.';
    const notice = $('mission-notice'); notice.classList.toggle('warning', !d.fresh || !!data.hold_lockout);
    notice.textContent = !d.fresh ? `Status unavailable. ${d.error || 'Waiting for the onboard service.'} The onboard mission may still be running.` :
      data.hold_lockout ? 'Ground-state reconciliation required before another flight. Cancelling a tree does not verify a physical stop.' :
      data.active ? 'This tree is executing onboard. Disconnecting the GCS does not stop it.' :
      mission ? `${mission.node_count} nodes validated onboard. Start applies to revision ${mission.revision.slice(0,12)}; editor changes need another upload.` : 'Upload and start are separate actions. Choose an XML mission to validate onboard.';
    $('stage-mission').disabled = busy || !d.fresh || data.active || !$('fleet-xml').value.trim();
    $('start-onboard').disabled = busy || !d.fresh || !data.can_start;
    $('cancel-onboard').disabled = busy || !d.fresh || !data.active;
    $('reconcile-onboard').hidden = !data.hold_lockout;
    $('reconcile-onboard').disabled = busy || !d.fresh || data.active;
    const nodes = document.createDocumentFragment();
    if (d.fresh && data.ros?.available) for (const name of data.ros.nodes) {
      const row = el('div','ros-node-row'); row.append(el('i','signal-dot'),el('span','',name),el('small','','DISCOVERED')); nodes.append(row);
    } else nodes.append(el('p','small-empty','A fresh ROS graph is not available.'));
    $('live-ros-nodes').replaceChildren(nodes);
    $('bt-registry-counts').textContent = d.fresh && data.registry_counts ? 'Available onboard: '+Object.entries(data.registry_counts).map(([k,v])=>`${v} ${k}`).join(' · ') : 'Onboard node registry unavailable.';
    renderTree(d); renderActivity();
  }
  function renderTree(d) {
    window.FleetTree3D?.update(d);
    const nodes = d.data?.bt_nodes || [], key = JSON.stringify([d.drone_id,d.fresh,nodes]);
    if (key === treeKey) return;
    const changedRun = !$('live-tree-canvas').dataset.run || $('live-tree-canvas').dataset.run !== `${d.drone_id}:${d.data?.execution?.mission_id}`;
    treeKey = key; const canvas = $('live-tree-canvas');
    canvas.dataset.run = `${d.drone_id}:${d.data?.execution?.mission_id}`;
    if (!d.fresh || !nodes.length) { canvas.replaceChildren(el('p','quiet-empty',!d.fresh ? 'Live tree unavailable — aircraft status is stale.' : 'The actual tree appears when execution starts.')); return; }
    const svgEl = (tag, attrs, text) => {const n = document.createElementNS('http://www.w3.org/2000/svg',tag); for (const [k,v] of Object.entries(attrs || {})) n.setAttribute(k,v); if (text != null) n.textContent = text; return n;};
    const svg = svgEl('svg', {role:'img','aria-label':`Behavior tree for ${d.label}`}), group = svgEl('g',{id:'fleet-tree-transform'});
    const byId = new Map(nodes.map(n => [n.node_uid,n])), childIds = new Set(nodes.flatMap(n => n.children_uids || []));
    const positions = new Map(); let leaf = 0;
    function place(n, depth, visited = new Set()) {
      if (visited.has(n.node_uid) || positions.has(n.node_uid)) return; visited.add(n.node_uid);
      const children = (n.children_uids || []).map(id => byId.get(id)).filter(Boolean);
      children.forEach(c => place(c, depth+1, visited));
      const points = children.map(c => positions.get(c.node_uid)).filter(Boolean);
      const x = points.length ? points.reduce((a,p)=>a+p.x,0)/points.length : leaf++ * 170;
      positions.set(n.node_uid,{x,y:depth*90});
    }
    nodes.filter(n=>!childIds.has(n.node_uid)).forEach(n=>place(n,0)); nodes.forEach(n=>place(n,0));
    for (const n of nodes) for (const id of n.children_uids || []) {
      const a = positions.get(n.node_uid), b = positions.get(id); if (!b) continue;
      group.append(svgEl('path',{class:'bt-edge',d:`M${a.x+75},${a.y+48} C${a.x+75},${a.y+69} ${b.x+75},${b.y-20} ${b.x+75},${b.y}`}));
    }
    for (const n of nodes) {
      const p = positions.get(n.node_uid), status = n.status === 'RUNNING' && d.data.active ? 'RUNNING' : ({success:'SUCCESS',failed:'FAILURE'}[n.display_status] || 'IDLE');
      const g = svgEl('g',{class:`bt-live-node ${status}`,transform:`translate(${p.x},${p.y})`});
      g.append(svgEl('title',{},`${n.node_path}\n${n.node_type}: ${n.status} · last result ${n.display_status}`),svgEl('rect',{class:'bt-box',width:150,height:48,rx:7}),svgEl('text',{x:10,y:18},n.node_name.length>21?n.node_name.slice(0,20)+'…':n.node_name),svgEl('text',{x:10,y:35,class:'bt-kind'},`${n.node_category || n.node_type} · ${n.display_status}`));group.append(g);
    }
    if (changedRun) transform = {x:20,y:20,scale:Math.min(1, Math.max(.25, (canvas.clientWidth-40)/Math.max(170,leaf*170)))};
    svg.append(group);canvas.replaceChildren(svg); applyTransform();
  }
  function applyTransform() {const g=$('fleet-tree-transform'); if(g)g.setAttribute('transform',`translate(${transform.x},${transform.y}) scale(${transform.scale})`);}
  async function refresh() {
    if (polling) return; polling = true;
    try {
      const response = await api('/api/fleet'); drones = response.drones; apiOK = true;
      $('fleet-api-state').replaceChildren(el('i','signal-dot'),document.createTextNode('GCS online'));
      for (const d of drones) {
        const previous = connections.get(d.drone_id);
        if (previous !== d.connection) {
          record(d.drone_id,d.connection === 'CONNECTED' ? 'INFO' : 'WARN',`${d.connection}${d.error ? ': '+d.error : ''}`);
          if (previous === 'CONNECTED' && !d.fresh) toast(`${d.label}: connection lost. Onboard execution may continue.`, 'ERROR');
          connections.set(d.drone_id,d.connection);
        }
        for (const event of d.data?.logs || []) {
          const id = `${d.drone_id}:${event.id}`, isNew = !seen.has(id);
          record(d.drone_id,event.level,event.message,id,event.time);
          if (isNew && event.level === 'ERROR' && Date.now()/1000-event.time<5) toast(`${d.label}: ${event.message}`,'ERROR');
        }
      }
      if (!current() && drones.length) select(drones[0].drone_id);
      $('fleet-update-time').textContent = `UPDATED ${new Date().toLocaleTimeString()}`; render();
    } catch (e) {
      if (apiOK) toast('GCS connection lost. Aircraft state is unknown.','ERROR'); apiOK = false;
      drones = drones.map(d=>({...d,fresh:false,connection:'STALE',error:'Laptop GCS service unavailable'}));
      $('fleet-api-state').textContent = 'GCS unavailable'; render();
    } finally {polling = false;}
  }
  async function command(action) {
    const d=current(); if (!d || busy) return; const mission=d.data?.mission, execution=d.data?.execution;
    const payload = {drone_id:d.drone_id,action};
    busy=true;render();
    try {
      if (action === 'stage') {
        payload.xml=$('fleet-xml').value;
        const plan=planned.get(d.drone_id);
        if(plan && plan.xml===payload.xml)payload.route_bundle=plan.bundle;
        if($('operator-load-check')?.checked)payload.operator_payload={confirmed_loaded:true,payload_id:$('operator-payload-id').value.trim()};
        payload.sha256=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',new TextEncoder().encode(payload.xml))),b=>b.toString(16).padStart(2,'0')).join('');
      } else if (action === 'start') Object.assign(payload,{mission_id:mission.id,revision:mission.revision,command_id:crypto.randomUUID()});
      else if (action === 'cancel') Object.assign(payload,{mission_id:execution.mission_id,revision:execution.revision});
      try { if (action === 'stage') window.GCSLinkDisplay?.uploadStarted(d); } catch {}
      await api('/api/fleet/command',payload);
      try { if (action === 'stage') window.GCSLinkDisplay?.uploadFinished(d, true); } catch {}
      toast(`${d.label}: ${action === 'stage' ? 'XML received and validated onboard' : action === 'start' ? 'Start acknowledged onboard' : action === 'cancel' ? 'Cancellation requested; awaiting result' : 'Ground state verified'}.`);
      await refresh();
    } catch(e) {
      try { if (action === 'stage') window.GCSLinkDisplay?.uploadFinished(d, false, e.message); } catch {}
      error(e.message,`${d.label} · ${action}`);}
    finally {busy=false;render();}
  }
  function useStudio() {
    if (!current()) return error('Add and select an aircraft before assigning a studio tree.');
    try {if (typeof generateBtXml !== 'function') throw new Error('BT studio is unavailable'); $('fleet-xml').value=generateBtXml();$('xml-file-label').textContent='Studio draft';render();}
    catch(e){error(e.message);}
  }
  function init() {
    const load=el('div','operator-load-controls'),label=el('label','',' I confirm this identified aid package is loaded before takeoff (no load sensors).');
    const check=el('input');check.type='checkbox';check.id='operator-load-check';label.prepend(check);
    const payloadId=el('input');payloadId.id='operator-payload-id';payloadId.placeholder='Unique package ID, e.g. kit_001';payloadId.setAttribute('aria-label','Physical package ID');
    load.append(label,payloadId,el('p','nodes-note','A command sent to the servo does not prove delivery. Confirmation is bound to this uploaded mission.'));
    $('detail-mission').insertBefore(load,$('detail-mission').querySelector('.mission-buttons'));
    for (const id of ['add-aircraft','empty-add-aircraft']) $(id).onclick=()=>$('aircraft-dialog').showModal();
    document.querySelectorAll('[data-close]').forEach(b=>b.onclick=()=>$(b.dataset.close).close());
    $('view-error-activity').onclick=()=>{$('fleet-error-dialog').close();switchView('fleet');$('activity-filter').value='errors';renderActivity();$('fleet-activity').scrollIntoView({behavior:'smooth',block:'nearest'});};
    $('aircraft-transport').onchange=()=>{const radio=$('aircraft-transport').value==='mavlink';$('aircraft-endpoint-label').hidden=radio;$('aircraft-system-label').hidden=!radio;$('aircraft-form').elements.endpoint.required=!radio;$('aircraft-form').elements.system_id.required=radio;};
    $('aircraft-form').onsubmit=async e=>{e.preventDefault();const form=e.currentTarget,button=form.querySelector('[type=submit]');button.disabled=true;button.textContent='Connecting…';$('aircraft-form-error').textContent='';
      try {await api('/api/fleet/profile',Object.fromEntries(new FormData(form)));form.reset();$('aircraft-transport').onchange();$('aircraft-dialog').close();await refresh();}
      catch(err){$('aircraft-form-error').textContent=err.message;}
      finally{button.disabled=false;button.textContent='Connect aircraft ↗';}};
    document.querySelectorAll('[data-panel]').forEach(b=>b.onclick=()=>{document.querySelectorAll('[data-panel]').forEach(x=>{const active=x===b;x.classList.toggle('active',active);x.setAttribute('aria-selected',String(active));$('detail-'+x.dataset.panel).hidden=!active;});if(b.dataset.panel==='tree'){treeKey='';$('live-tree-canvas').dataset.run='';render();}});
    $('activity-filter').onchange=renderActivity;$('clear-activity').onclick=()=>{activity=[];renderActivity();};
    $('choose-mission').onclick=()=>$('mission-file').click();$('use-studio-mission').onclick=useStudio;
    $('mission-file').onchange=async e=>{const file=e.target.files[0];if(!file)return;try{if(file.size>1000000)throw new Error('XML exceeds the 1 MB limit');$('fleet-xml').value=await file.text();$('xml-file-label').textContent=file.name;render();}catch(err){error(err.message);}finally{e.target.value='';}};
    $('fleet-xml').oninput=render;
    for(const [id,action] of [['stage-mission','stage'],['start-onboard','start'],['cancel-onboard','cancel'],['reconcile-onboard','reconcile']]) $(id).onclick=()=>command(action);
    const canvas=$('live-tree-canvas');let drag=null;
    canvas.onpointerdown=e=>{drag={x:e.clientX,y:e.clientY};canvas.setPointerCapture(e.pointerId);};
    canvas.onpointermove=e=>{if(!drag)return;transform.x+=e.clientX-drag.x;transform.y+=e.clientY-drag.y;drag={x:e.clientX,y:e.clientY};applyTransform();};
    canvas.onpointerup=canvas.onpointercancel=()=>{drag=null;};
    canvas.addEventListener('wheel',e=>{e.preventDefault();transform.scale=Math.max(.15,Math.min(3,transform.scale*(e.deltaY>0?.9:1.1)));applyTransform();},{passive:false});
    refresh();setInterval(refresh,1500);
  }
  window.FleetUI={refresh,useStudio,error,toast,getSelected:current,setXml:(xml,label)=>{if(!current())throw new Error('Add and select an aircraft in Fleet first.');$('fleet-xml').value=xml;$('xml-file-label').textContent=label;render();}};
  window.FleetUI.selectAircraft=select;
  window.FleetUI.getAircraft=()=>drones.map(d=>({drone_id:d.drone_id,label:d.label}));
  window.FleetUI.setPlannedMission=(id,xml,bundle)=>{
    if(!drones.some(d=>d.drone_id===id))throw new Error('Register this aircraft in Fleet first.');
    planned.set(id,{xml,bundle});drafts.set(id,{xml,label:'KML partition · '+id});
    if(id===selected){$('fleet-xml').value=xml;$('xml-file-label').textContent='KML partition · '+id;render();}
  };
  if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',init);else init();
})();
