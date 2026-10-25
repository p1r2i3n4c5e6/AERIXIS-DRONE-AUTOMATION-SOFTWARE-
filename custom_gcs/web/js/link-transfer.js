/* Presentation only. Consumes existing refreshes; never opens a connection or sends a request. */
(() => {
 'use strict';
 const make=(tag,cls,text)=>{const n=document.createElement(tag);n.className=cls||'';if(text!=null)n.textContent=text;return n;};
 let local=null,received=0,unavailable=false,fleet=[],selected='',fleetAt=0;
 const uploads=new Map();let dialog,trigger,summary,localStatus,localDetail,as2,px4,tx,rx,mission,aircraft,upload;
 const number=v=>typeof v==='number'&&Number.isFinite(v)?Math.max(0,v):0;
 function rateRow(cls,label){const row=make('div','transfer-row '+cls),name=make('div','transfer-label',label),track=make('div','packet-track');track.setAttribute('aria-hidden','true');for(let i=0;i<3;i++)track.append(make('i'));const value=make('strong','transfer-rate','0.0 KB/s'),total=make('small','transfer-total','No telemetry');row.append(name,value,track,total);return row;}
 function init(){
  trigger=make('button','link-monitor-trigger');trigger.type='button';trigger.setAttribute('aria-haspopup','dialog');trigger.append(make('i',''),make('span','link-trigger-label','Links & data'),make('small','link-trigger-state','Waiting'));
  dialog=make('dialog','link-monitor');dialog.setAttribute('aria-labelledby','link-monitor-title');
  const head=make('div','link-monitor-head'),title=make('div');title.append(make('span','eyebrow','COMMUNICATIONS'),make('h2','','Links & data'));title.lastChild.id='link-monitor-title';const close=make('button','link-close','×');close.type='button';close.setAttribute('aria-label','Close link monitor');close.onclick=()=>dialog.close();head.append(title,close);
  summary=make('p','link-summary','Waiting for connection status');
  const pipeline=make('div','link-pipeline');for(const label of ['GCS','AERIXIS','PX4']){const node=make('div','link-endpoint');node.append(make('i','link-orbit'),make('strong','',label),make('small','','Waiting'));pipeline.append(node);if(label!=='PX4')pipeline.append(make('span','link-bridge','↔'));}as2=pipeline.children[2];px4=pipeline.children[4];
  const localSection=make('section','link-section'),localHead=make('div','link-section-title');localStatus=make('span','link-status');localHead.append(make('h3','','Local telemetry link'),localStatus);localDetail=make('p','link-detail');tx=rateRow('outgoing','↑ Transmit to gateway');rx=rateRow('incoming','↓ Receive from gateway');mission=make('p','transfer-message');localSection.append(localHead,localDetail,tx,rx,mission);
  const fleetSection=make('section','link-section aircraft-link-section');fleetSection.append(make('h3','','Selected aircraft · onboard executor'));aircraft=make('div','aircraft-link-status');upload=make('div','upload-operation');upload.setAttribute('role','status');fleetSection.append(aircraft,upload,make('p','link-detail','Aircraft upload confirmation is shown after the onboard service replies. Per-aircraft byte rates are not supplied by this endpoint.'));
  dialog.append(head,summary,pipeline,localSection,fleetSection);document.body.append(dialog);document.querySelector('body > header').insertBefore(trigger,document.getElementById('interface-tools'));
  trigger.onclick=()=>{draw();dialog.showModal();};dialog.addEventListener('click',e=>{if(e.target===dialog){const r=dialog.getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)dialog.close();}});
  draw();setInterval(draw,1000);
 }
 function endpoint(node,online,label){node.dataset.state=online?'connected':'offline';node.querySelector('small').textContent=label;}
 function drawRate(row,rate,kb,packets,active){row.classList.toggle('flowing',active&&rate>0);row.querySelector('.transfer-rate').textContent=(active?rate:0).toFixed(1)+' KB/s';row.querySelector('.transfer-total').textContent=local?`${kb.toFixed(1)} KB · ${packets.toLocaleString()} packets`:'No telemetry';}
 function draw(){if(!dialog)return;
  const fresh=received>0&&!unavailable&&Date.now()-received<3500,connected=fresh&&!!local?.connected;
  const state=!fresh?'unavailable':connected?'connected':'waiting';
  const label=!fresh?'Service unavailable':connected?'Gateway connected':'Awaiting heartbeat';
  trigger.dataset.state=state;trigger.querySelector('small').textContent=label;dialog.dataset.state=state;
  summary.textContent=connected?'Live telemetry from the local gateway':fresh?'Listening for the AERIXIS gateway heartbeat':'Local GCS state is unavailable. Live data movement is paused.';
  localStatus.textContent=label;localStatus.dataset.state=state;
  localDetail.textContent=fresh?(local.gateway_detail||'Local MAVLink transport'):'Waiting for a fresh GCS update';
  const gcs=dialog.querySelector('.link-endpoint');endpoint(gcs,fresh,fresh?'Service reachable':'No fresh state');endpoint(as2,connected&&!!local.as2_link?.connected,connected&&local.as2_link?.connected?'Core online':'Core unconfirmed');endpoint(px4,connected&&!!local.px4_link?.connected,connected&&local.px4_link?.connected?'Autopilot online':'Autopilot unconfirmed');
  const t=local?.traffic||{};drawRate(tx,number(t.tx_rate),number(t.tx_kb),number(t.tx_packets),connected);drawRate(rx,number(t.rx_rate),number(t.rx_kb),number(t.rx_packets),connected);
  const message=fresh?local.mission_message||'No mission transfer reported':'Mission transfer status unavailable';mission.textContent=message;mission.classList.toggle('processing',fresh&&/^(Uploading \d+ MAVLink items|Sending item \d+\/\d+)/.test(message));
  const fleetFresh=Date.now()-fleetAt<5000,d=fleet.find(d=>d.drone_id===selected),online=fleetFresh&&!!d?.fresh;
  const aircraftKey=JSON.stringify([d?.drone_id,d?.label,d?.connection,online,fleetFresh]);
  if(aircraft.dataset.key!==aircraftKey){aircraft.dataset.key=aircraftKey;aircraft.replaceChildren();aircraft.dataset.state=online?'connected':d?.connection==='CONNECTING'&&fleetFresh?'connecting':'offline';
  aircraft.append(make('i','link-orbit'),make('strong','',d?.label||'No aircraft selected'),make('span','',d?(fleetFresh?d.connection||'UNKNOWN':'State unavailable'):'Add an aircraft from Fleet overview'));}
  const record=d&&uploads.get(d.drone_id),uploadKey=JSON.stringify([d?.drone_id,record,online]);
  if(upload.dataset.key!==uploadKey){upload.dataset.key=uploadKey;upload.replaceChildren();upload.dataset.state=record?.state||'idle';
  if(record){upload.append(make('i','upload-indicator'),make('strong','',record.state==='pending'?'Uploading mission · awaiting onboard validation':record.state==='success'?'Mission received & validated onboard':'Upload not confirmed'),make('small','',record.state==='pending'?(online?'Request in progress. Waiting for the aircraft reply.':'Request pending; aircraft telemetry is unavailable.'):record.detail||'Onboard service acknowledged the mission XML.'));}
  else upload.append(make('span','','No mission upload in this session'));}
  trigger.classList.toggle('upload-pending',[...uploads.values()].some(r=>r.state==='pending'));
 }
 window.GCSLinkDisplay={
  updateLocal(s){local=s;received=Date.now();unavailable=false;draw();},
  localUnavailable(){unavailable=true;draw();},
  updateFleet(drones,id){fleet=drones||[];selected=id;fleetAt=Date.now();draw();},
  uploadStarted(d){uploads.set(d.drone_id,{state:'pending'});draw();},
  uploadFinished(d,ok,detail){uploads.set(d.drone_id,{state:ok?'success':'error',detail:detail||''});draw();}
 };
 if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',init);else init();
})();
