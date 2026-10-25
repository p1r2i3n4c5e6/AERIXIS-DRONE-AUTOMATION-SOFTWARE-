/* KML -> one validated XML + route bundle per drone. Trees build automatically when a KML
   is loaded or the mission type changes; the operator still reviews, assigns and uploads. */
(() => {
  'use strict';
  const make=(tag,text)=>{const n=document.createElement(tag);if(text)n.textContent=text;return n;};
  const store={get:k=>{try{return localStorage.getItem(k);}catch(e){return null;}},set:(k,v)=>{try{localStorage.setItem(k,v);}catch(e){}}};
  const MISSIONS={
    'Rescue missions (A8 mini tracker)':{
      human_track_rtl:'1 · Track & target a person — return to launch if lost',
      human_orbit:'2 · Orbit the detected person, then return to launch',
      human_geotag_resume:'3 · Geotag every person, release lock, resume search',
      human_geotag_drop_resume:'4 · Geotag + drop payload over person, resume search',
      swarm_coverage:'5 · Swarm area coverage only'},
    'Other templates':{survey:'Surveillance',search_geotag:'Search and geotag (calibrated geolocation)',
      rescue_delivery:'Rescue delivery (beside person)',waypoint_delivery:'Waypoint delivery',
      swarm_search:'Swarm search',swarm_delivery:'Swarm delivery'}};
  const HELP={
    human_track_rtl:'Searches the KML area. When a person is detected the drone stops, faces them and keeps the gimbal locked. If the person is lost for longer than the lost timeout it flies straight back to launch and lands.',
    human_orbit:'Searches until a person is detected, then circles them at the orbit radius for the chosen laps with the gimbal locked, then returns to launch.',
    human_geotag_resume:'Searches the area. For each new person: pause, fly until the gimbal looks straight down with the person centred, geotag latitude/longitude, release the lock and resume the search. Repeats for every person, then returns to launch.',
    human_geotag_drop_resume:'Same as mission 3, plus one payload release over the first person while the gimbal is at nadir. Needs the operator load confirmation and a verified overhead envelope onboard.',
    swarm_coverage:'Each drone flies its share of the KML area with mesh freshness and separation gates. No person detection.'};
  const VISION=['human_track_rtl','human_orbit','human_geotag_resume','human_geotag_drop_resume'];
  const DEFAULT_KEY='as2.defaultMission';

  function init(){
    const parent=document.querySelector('.designer-workspace');if(!parent)return;
    const panel=make('section');panel.className='glass-panel fleet-planner';
    panel.append(make('h2','Plan a fleet mission from KML'),make('p','Load a KML: each drone’s behaviour tree is generated automatically for the selected mission. Review, assign and upload stay separate steps.'));
    const form=make('form'),fields={};
    function field(name,title,value,type='number',parentNode=form){
      const label=make('label',title),input=make('input');input.type=type;input.value=value;input.name=name;input.step='any';
      input.required=type!=='text';label.append(input);parentNode.append(label);fields[name]=input;return input;
    }
    // Mission type first: it decides which options matter.
    const label=make('label','Mission type'),mode=make('select');
    for(const [group,items] of Object.entries(MISSIONS)){
      const optgroup=document.createElement('optgroup');optgroup.label=group;
      for(const [value,text] of Object.entries(items)){const option=make('option',text);option.value=value;optgroup.append(option);}
      mode.append(optgroup);
    }
    mode.value=store.get(DEFAULT_KEY)||'human_geotag_resume';if(!mode.value)mode.value='human_geotag_resume';
    label.append(mode);form.append(label);
    const help=make('p');help.className='planner-mission-help';form.append(help);
    const defaultRow=make('label','Use this mission type as the default'),makeDefault=make('input');makeDefault.type='checkbox';defaultRow.prepend(makeDefault);form.append(defaultRow);
    const file=field('kml','KML area','','file');file.accept='.kml,application/vnd.google-earth.kml+xml';
    field('area','Area name (only for multi-area KML)','','text');
    field('altitude_m','Flight altitude above take-off (m)','15');field('spacing_m','Survey lane spacing (m)','15');
    field('speed_mps','Survey speed (m/s)','1.5');field('corridor_m','Separation between drones (m)','10');field('heading_deg','Survey heading (degrees)','0');
    const options=make('details');options.className='planner-mission-options';options.append(make('summary','Mission options'));form.append(options);
    const visionFields={search_pitch_deg:['Gimbal search pitch (deg, -90 = down)','-45'],lost_timeout_s:['Person lost timeout (s)','3'],
      track_duration_s:['Mission 1: track duration (s, 0 = until lost)','0'],orbit_radius_m:['Mission 2: orbit radius (m)','10'],
      orbit_laps:['Mission 2: orbit laps','2'],approach_speed:['Missions 3–4: approach speed (m/s)','1'],rtl_speed:['Return-to-launch speed (m/s)','2']};
    for(const [name,[title,value]] of Object.entries(visionFields))field(name,title,value,'number',options);
    const payloadLabel=make('label','Payload confirmation'),confirmation=make('select');
    for(const [value,text] of [['operator','Operator confirms load (servo drop, delivery not sensed)'],['sensors','Physical load/release sensors']]){const option=make('option',text);option.value=value;confirmation.append(option);}
    payloadLabel.append(confirmation);options.append(payloadLabel);
    const overheadLabel=make('label','Direct-over-person delivery for “Other templates” (needs a calibrated payload envelope)'),overhead=make('input');overhead.type='checkbox';overheadLabel.prepend(overhead);options.append(overheadLabel);

    const aircraft=make('div');aircraft.className='planner-aircraft';
    const refresh=make('button','Reload registered aircraft');refresh.type='button';
    function loadAircraft(){
      const previous={};for(const n of aircraft.querySelectorAll('input[data-drone]'))previous[n.dataset.drone]=n.value;
      aircraft.replaceChildren();
      for(const d of (window.FleetUI&&window.FleetUI.getAircraft?window.FleetUI.getAircraft():[])){
        const row=make('label',d.label+' · endurance (minutes)'),input=make('input');input.type='number';input.min='1';input.step='0.1';
        input.value=previous[d.drone_id]||store.get('as2.endurance.'+d.drone_id)||'20';input.required=true;input.dataset.drone=d.drone_id;
        input.onchange=()=>store.set('as2.endurance.'+d.drone_id,input.value);
        row.append(input);aircraft.append(row);
      }
      if(!aircraft.children.length)aircraft.append(make('p','Add aircraft in Fleet first; plans are made per registered drone.'));
    }
    refresh.onclick=loadAircraft;
    const submit=make('button','Generate per-drone trees');submit.type='submit';submit.className='fleet-primary';
    const status=make('p');status.setAttribute('role','status');const results=make('div');results.className='planner-results';
    form.append(refresh,aircraft,submit);panel.append(form,status,results);parent.insertBefore(panel,parent.querySelector('.designer-grid'));

    function describe(){
      help.textContent=HELP[mode.value]||'';makeDefault.checked=(store.get(DEFAULT_KEY)||'human_geotag_resume')===mode.value;
      if(mode.value==='human_geotag_drop_resume')confirmation.value='operator';
      options.classList.toggle('vision-mission',VISION.includes(mode.value));
    }
    makeDefault.onchange=()=>{if(makeDefault.checked){store.set(DEFAULT_KEY,mode.value);window.FleetUI&&window.FleetUI.toast('Default mission: '+mode.selectedOptions[0].textContent);}};
    let planning=false;
    async function plan(auto){
      if(planning)return;
      if(!file.files[0]){if(!auto)status.textContent='Choose a KML file first.';return;}
      if(!aircraft.querySelector('input[data-drone]'))loadAircraft();
      const fleet=[...aircraft.querySelectorAll('input[data-drone]')].map(n=>({drone_id:n.dataset.drone,endurance_s:Number(n.value)*60,reserve_fraction:.25,coverage:true}));
      if(!fleet.length){status.textContent='Add aircraft in Fleet first, then the trees are generated automatically.';return;}
      planning=true;submit.disabled=true;submit.textContent='Generating trees…';results.replaceChildren();
      try{
        if(file.files[0].size>1000000)throw new Error('Choose a KML file up to 1 MB.');
        const kml=await file.files[0].text();
        const request={mission_id:crypto.randomUUID().replaceAll('-',''),template:mode.value,payload_confirmation:confirmation.value,fleet,kml};
        request.drop_mode=mode.value==='human_geotag_drop_resume'||overhead.checked?'overhead':'beside';request.overhead_authorized=request.drop_mode==='overhead';
        for(const [key,input] of Object.entries(fields)){
          if(key==='kml')continue;
          if(key in visionFields&&!VISION.includes(mode.value))continue;
          if(input.type==='number'){if(input.value!=='')request[key]=Number(input.value);}else if(input.value)request[key]=input.value;
        }
        const response=await fetch('/api/plan/kml',{method:'POST',headers:{'Content-Type':'application/json','X-AS2-GCS':'1'},body:JSON.stringify(request)});
        const result=await response.json();if(!response.ok||result.error)throw new Error(result.error||'Planning failed');
        render(result);
      }catch(e){status.textContent=e.message;if(!auto&&window.FleetUI)window.FleetUI.error(e.message,'Fleet planner');}
      finally{planning=false;submit.disabled=false;submit.textContent='Generate per-drone trees';}
    }
    function render(result){
      const name=mode.selectedOptions[0].textContent;
      status.textContent=result.complete?`${name}: ${result.bundles.length} tree(s) generated. Check origin, altitude and corridors before assigning.`:'Plan needs correction: '+result.reason;
      results.append(make('p',`Common origin: ${result.origin.latitude.toFixed(8)}, ${result.origin.longitude.toFixed(8)}. Every aircraft must use this same origin.`));
      result.bundles.forEach((bundle,index)=>{
        const row=make('article');row.append(make('h3',bundle.drone_id),make('p',`${bundle.coverage.waypoints.length} waypoints · ${Math.round(bundle.assignment.estimated_duration_s/60)} estimated minutes`));
        const preview=make('button','Preview tree');preview.type='button';preview.onclick=()=>window.MissionDesigner.importXml(result.xml[bundle.drone_id]);
        const assign=make('button','Assign XML + route to aircraft');assign.type='button';assign.disabled=!result.complete;
        assign.onclick=()=>{try{window.FleetUI.setPlannedMission(bundle.drone_id,result.xml[bundle.drone_id],bundle);window.FleetUI.toast(bundle.drone_id+': XML and route assigned for review.');}catch(e){window.FleetUI.error(e.message);}};
        row.append(preview,assign);results.append(row);
        if(index===0&&window.MissionDesigner){try{window.MissionDesigner.importXml(result.xml[bundle.drone_id]);}catch(e){}}
      });
      const assignAll=make('button','Assign all drones');assignAll.type='button';assignAll.className='fleet-primary';assignAll.disabled=!result.complete;
      assignAll.onclick=()=>{let done=0;for(const bundle of result.bundles){try{window.FleetUI.setPlannedMission(bundle.drone_id,result.xml[bundle.drone_id],bundle);done++;}catch(e){window.FleetUI.error(e.message,bundle.drone_id);}}
        if(done)window.FleetUI.toast(`${done} drone(s): XML and route assigned for review.`);};
      const download=make('button','Download fleet manifest and plans');download.type='button';download.onclick=()=>{
        const url=URL.createObjectURL(new Blob([JSON.stringify(result,null,2)],{type:'application/json'})),link=make('a');link.href=url;link.download='aerixis-fleet-plan.json';link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
      };results.append(assignAll,download);
    }
    form.onsubmit=event=>{event.preventDefault();plan(false);};
    file.onchange=()=>{loadAircraft();plan(true);};
    mode.onchange=()=>{describe();if(file.files[0])plan(true);};
    describe();setTimeout(loadAircraft,500);
  }
  if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',init);else init();
})();
