/* Read-only 3D view of actual per-aircraft BT instances, keyed by node UID. */
(() => {
  'use strict';
  let renderer, scene, camera, group, last='', current, enabled=false;
  let angle=.15, elevation=.25, distance=40;
  const host=()=>document.getElementById('fleet-tree-3d');
  function paint(){
    if(!renderer||!enabled||!host().clientWidth)return;
    const width=host().clientWidth,height=host().clientHeight;
    renderer.setSize(width,height,false);camera.aspect=width/height;camera.updateProjectionMatrix();
    camera.position.set(Math.sin(angle)*distance,Math.sin(elevation)*distance,Math.cos(angle)*distance);
    camera.lookAt(0,-5,0);renderer.render(scene,camera);
  }
  function setup(){
    if(renderer)return true;
    if(!window.THREE){host().textContent='3D is unavailable. Use the 2D live tree.';return false;}
    try{
      renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));
      scene=new THREE.Scene();scene.background=new THREE.Color('#07121c');camera=new THREE.PerspectiveCamera(48,1,.1,5000);
      host().replaceChildren(renderer.domElement);let drag;
      host().onpointerdown=e=>{drag=[e.clientX,e.clientY];host().setPointerCapture(e.pointerId);};
      host().onpointermove=e=>{if(!drag)return;angle+=(e.clientX-drag[0])*.007;elevation=Math.max(-.7,Math.min(1,(elevation+(e.clientY-drag[1])*.007)));drag=[e.clientX,e.clientY];paint();};
      host().onpointerup=host().onpointercancel=()=>{drag=null;};
      host().addEventListener('wheel',e=>{e.preventDefault();distance=Math.max(8,Math.min(2000,distance*(e.deltaY>0?1.1:.9)));paint();},{passive:false});
      window.addEventListener('resize',paint);return true;
    }catch(e){host().textContent='3D is unavailable on this device. Use the 2D live tree.';return false;}
  }
  function update(drone){
    current=drone;if(!enabled||!setup())return;
    const nodes=drone?.fresh?(drone.data?.bt_nodes||[]):[];
    const key=JSON.stringify([drone?.drone_id,drone?.fresh,drone?.data?.active,nodes]);
    if(key===last){paint();return;}last=key;
    if(group){group.traverse(o=>{o.geometry?.dispose();if(o.material){o.material.map?.dispose();o.material.dispose();}});scene.remove(group);}
    group=new THREE.Group();scene.add(group);
    document.getElementById('fleet-3d-status').textContent=!drone?.fresh?'Aircraft state is stale. Live graph cleared.':nodes.length?`${drone.label} · ${nodes.length} actual node instances`:'Waiting for the actual onboard tree.';
    if(!nodes.length){paint();return;}
    const byId=new Map(nodes.map(n=>[n.node_uid,n])),children=new Set(nodes.flatMap(n=>n.children_uids||[])),positions=new Map();let leaf=0,depthMax=0;
    function layout(n,depth,seen=new Set()){
      if(positions.has(n.node_uid)||seen.has(n.node_uid))return;seen.add(n.node_uid);depthMax=Math.max(depthMax,depth);
      const kids=(n.children_uids||[]).map(id=>byId.get(id)).filter(Boolean);kids.forEach(c=>layout(c,depth+1,seen));
      const points=kids.map(c=>positions.get(c.node_uid)).filter(Boolean);
      positions.set(n.node_uid,{x:points.length?points.reduce((s,p)=>s+p.x,0)/points.length:leaf++*4,y:-depth*2.8});
    }
    nodes.filter(n=>!children.has(n.node_uid)).forEach(n=>layout(n,0));nodes.forEach(n=>layout(n,0));
    const center=(leaf-1)*2;
    for(const node of nodes){
      const p=positions.get(node.node_uid),running=node.status==='RUNNING'&&drone.data.active;
      const status=running?'RUNNING':({success:'SUCCESS',failed:'FAILURE'}[node.display_status]||'IDLE');
      const color={RUNNING:'#61c8e8',SUCCESS:'#68d9b4',FAILURE:'#ec8491',IDLE:'#536a7b'}[status];
      const box=new THREE.Mesh(new THREE.BoxGeometry(3.6,1.2,.25),new THREE.MeshBasicMaterial({color,transparent:true,opacity:.5}));box.position.set(p.x-center,p.y,0);group.add(box);
      const canvas=document.createElement('canvas');canvas.width=512;canvas.height=128;const ctx=canvas.getContext('2d');
      ctx.textAlign='center';ctx.fillStyle='#eefaff';ctx.font='bold 27px sans-serif';ctx.fillText(String(node.node_name).slice(0,29),256,49);
      ctx.font='18px sans-serif';ctx.fillText(`${node.node_category} · ${status} · #${node.node_uid}`,256,85);
      const label=new THREE.Sprite(new THREE.SpriteMaterial({map:new THREE.CanvasTexture(canvas),transparent:true}));label.scale.set(3.5,.9,1);label.position.set(p.x-center,p.y,.2);group.add(label);
      for(const uid of node.children_uids||[]){const b=positions.get(uid);if(!b)continue;group.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(p.x-center,p.y-.6,0),new THREE.Vector3(b.x-center,b.y+.6,0)]),new THREE.LineBasicMaterial({color:'#365969'})));}
    }
    distance=Math.max(25,Math.min(1900,Math.max(leaf*4,depthMax*3)*1.3));paint();
  }
  function init(){
    for(const mode of ['2d','3d'])document.getElementById('fleet-show-'+mode).onclick=()=>{
      enabled=mode==='3d';host().hidden=!enabled;document.getElementById('fleet-3d-status').hidden=!enabled;
      document.getElementById('live-tree-canvas').hidden=enabled;
      for(const m of ['2d','3d'])document.getElementById('fleet-show-'+m).setAttribute('aria-pressed',String(m===mode));
      update(current||window.FleetUI?.getSelected());
    };
  }
  window.FleetTree3D={update};
  if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',init);else init();
})();
