/* One editable hierarchy supplies XML, the node inspector and the 3D graph. */
(() => {
  'use strict';
  const $=id=>document.getElementById(id);
  const el=(tag,cls,text)=>{const n=document.createElement(tag);if(cls)n.className=cls;if(text!=null)n.textContent=text;return n;};
  let catalog,definitions={},main='Mission',active='Mission',selected=null,counter=0;
  let renderer,scene,camera,group,angle=.25,elevation=.25,distance=35,meshes=[],lastGraph='';
  const clone=x=>JSON.parse(JSON.stringify(x));
  function identify(n){n.uid=++counter;n.ports=n.ports||{};n.children=n.children||[];n.children.forEach(identify);return n;}
  function walk(n,fn,parent=null){if(!n)return;fn(n,parent);n.children.forEach(c=>walk(c,fn,n));}
  function locate(){let found=null;walk(definitions[active],(n,p)=>{if(n.uid===selected)found={node:n,parent:p};});return found;}
  const escape=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&apos;'}[c]));
  function xml(){
    function output(n,depth){const indent='  '.repeat(depth),attrs=Object.entries(n.ports).filter(([,v])=>v!=='').map(([k,v])=>` ${k}="${escape(v)}"`).join('');return n.children.length?`${indent}<${n.type}${attrs}>\n${n.children.map(c=>output(c,depth+1)).join('\n')}\n${indent}</${n.type}>`:`${indent}<${n.type}${attrs}/>`;}
    return `<?xml version="1.0" encoding="UTF-8"?>\n<root BTCPP_format="4" main_tree_to_execute="${escape(main)}">\n`+Object.entries(definitions).map(([id,n])=>`  <BehaviorTree ID="${escape(id)}">\n${output(n,2)}\n  </BehaviorTree>`).join('\n')+'\n</root>';
  }
  function issues(){const errors=[];for(const tree of Object.values(definitions))walk(tree,n=>{const spec=catalog.nodes[n.type];if(!spec){errors.push(`Unregistered node ${n.type}`);return;}const count=n.children.length;if(['Action','Condition'].includes(spec.category)&&count)errors.push(`${n.type} cannot have children`);if(spec.category==='Decorator'&&count!==1)errors.push(`${n.type} needs exactly one child`);if(spec.category==='Control'&&!count)errors.push(`${n.type} needs children`);if(n.type==='RecoverThenFail'&&count!==2)errors.push('RecoverThenFail needs task + recovery');if(!spec.upload_supported)errors.push(`${n.type} is restricted by the upload profile`);for(const [name,p] of Object.entries(spec.ports))if(p.direction===0&&!p.has_default&&!n.ports[name]&&!['server_name','server_timeout'].includes(name))errors.push(`${n.type}: enter ${name}`);if(n.type==='SubTree'&&!definitions[n.ports.ID])errors.push('SubTree needs an existing definition ID');});return errors;}
  function notify(message,error=false){$('designer-notice').textContent=message;$('designer-notice').classList.toggle('tree-validation-error',error);}
  function renderLibrary(){
    const search=$('designer-search').value.toLowerCase(),category=$('designer-category').value,fragment=document.createDocumentFragment();
    for(const [name,spec] of Object.entries(catalog.nodes).sort(([a],[b])=>a.localeCompare(b))){
      if(category!=='all'&&spec.category!==category)continue;if(!`${name} ${spec.description}`.toLowerCase().includes(search))continue;
      const card=el('article','library-entry'+(!spec.upload_supported?' unsupported':''));card.append(el('small','',spec.category),el('h3','',name),el('p','',spec.description));
      const add=el('button','',spec.category==='Decorator'?'Wrap selected':'Add child');add.disabled=!spec.upload_supported;add.onclick=()=>addNode(name);card.append(add);
      const help=el('button','quiet-button','Help');help.onclick=()=>renderProperties({type:name,ports:{},children:[]},true);card.append(help);fragment.append(card);
    }
    $('designer-library').replaceChildren(fragment);
  }
  function addNode(type){
    const found=locate();if(!found)return;
    const spec=catalog.nodes[type],node=identify({type,ports:{},children:[]});
    if(spec.category==='Decorator'){
      node.children=[found.node];if(found.parent)found.parent.children[found.parent.children.indexOf(found.node)]=node;else definitions[active]=node;
    }else{
      const parentSpec=catalog.nodes[found.node.type];
      if(!['Control','Decorator'].includes(parentSpec?.category))return notify('Select a Control node before adding children, or wrap this node first.',true);
      if(parentSpec.category==='Decorator'&&found.node.children.length)return notify('A Decorator has one child. Select its child to wrap or edit it.',true);
      found.node.children.push(node);
    }
    selected=node.uid;render();
  }
  function renderProperties(n,helpOnly=false){
    const spec=catalog.nodes[n.type];const container=$('designer-properties');container.replaceChildren();if(!spec)return;
    container.append(el('h3','',n.type),el('p','',spec.description),el('p','node-safety',spec.safety));
    if(!spec.upload_supported)container.append(el('p','tree-validation-error','Listed for reference. This node is restricted by the current onboard validator.'));
    if(helpOnly){container.append(el('p','','Select a tree instance to edit its ports.'));return;}
    const ports={name:{type:'string',description:'Instance label shown in execution logs',has_default:true,default:''},...spec.ports};
    if(n.type==='SubTree')ports.ID={type:'string',description:'BehaviorTree definition ID',has_default:false};
    for(const [key,port] of Object.entries(ports)){
      const label=el('label','',key+(port.direction===1?' · output':''));const input=el('input');input.value=n.ports[key]??(port.has_default?port.default:'');
      input.placeholder=port.has_default?'Default: '+port.default:'Required';input.setAttribute('aria-label',`${n.type} ${key}`);
      input.onchange=()=>{if(input.value==='')delete n.ports[key];else n.ports[key]=input.value;renderTree();updateXml();};
      label.append(input,el('small','',`${port.type||'string'} · ${port.description||'Typed onboard port'}`));container.append(label);
    }
  }
  function renderTree(){
    const root=definitions[active],fragment=document.createDocumentFragment();let count=0;
    function row(n,depth){count++;const spec=catalog.nodes[n.type]||{category:'Unknown'};const button=el('button','designer-tree-row'+(n.uid===selected?' selected':''));button.style.marginLeft=`${depth*14}px`;button.style.width=`calc(100% - ${depth*14}px)`;button.setAttribute('role','treeitem');button.setAttribute('aria-level',depth+1);button.setAttribute('aria-selected',String(n.uid===selected));button.append(el('i','node-kind-dot '+spec.category),el('span','',n.ports.name||n.type),el('small','',spec.category));button.onclick=()=>{selected=n.uid;renderTree();renderProperties(n);};fragment.append(button);n.children.forEach(c=>row(c,depth+1));}
    if(root)row(root,0);$('designer-tree').replaceChildren(fragment);$('designer-instance-count').textContent=`${count} INSTANCES`;
    draw3D();
  }
  function updateXml(){const errors=issues();$('designer-xml').value=xml();notify(errors.length?errors.slice(0,4).join(' · '):'Local structure checks passed. Upload performs authoritative onboard validation; hardware readiness is checked separately.',!!errors.length);$('designer-to-fleet').disabled=!!errors.length;}
  function render(){
    const options=Object.keys(definitions).map(id=>{const option=el('option','',id+(id===main?' · main':''));option.value=id;return option;});$('designer-definition').replaceChildren(...options);$('designer-definition').value=active;
    renderTree();const found=locate();if(found)renderProperties(found.node);updateXml();
  }
  function applyTemplate(){
    const template=clone(catalog.templates[$('designer-template').value]);const swarm=crypto.randomUUID();walk(template,n=>{if(n.ports.swarm_id)n.ports.swarm_id=swarm;});definitions={Mission:identify(template)};main=active='Mission';selected=definitions.Mission.uid;render();
  }
  function move(delta){const f=locate();if(!f?.parent)return;const list=f.parent.children,i=list.indexOf(f.node),j=i+delta;if(j<0||j>=list.length)return;[list[i],list[j]]=[list[j],list[i]];render();}
  function importXml(text){
    if(/<!DOCTYPE|<!ENTITY/i.test(text))throw new Error('XML entities and external documents are not allowed');
    const doc=new DOMParser().parseFromString(text,'application/xml');if(doc.querySelector('parsererror'))throw new Error('Invalid XML');
    const root=doc.documentElement;if(root.tagName!=='root'||root.getAttribute('BTCPP_format')!=='4')throw new Error('BehaviorTree.CPP v4 XML required');
    const next={};let count=0;
    function read(element,depth=0){if(++count>500||depth>40)throw new Error('Tree size/depth limit exceeded');let type=element.tagName;const ports=Object.fromEntries([...element.attributes].map(a=>[a.name,a.value]));if(['Action','Condition','Decorator','Control'].includes(type)){type=ports.ID;delete ports.ID;}return {type,ports,children:[...element.children].map(c=>read(c,depth+1))};}
    for(const def of root.children){if(def.tagName==='TreeNodesModel')continue;if(def.tagName!=='BehaviorTree')throw new Error('External tree includes are not supported');const id=def.getAttribute('ID');if(!id||next[id]||def.children.length!==1)throw new Error('Unique tree IDs and one root per definition are required');next[id]=identify(read(def.children[0]));}
    const mainId=root.getAttribute('main_tree_to_execute')||Object.keys(next)[0];if(!next[mainId])throw new Error('Main tree missing');definitions=next;main=active=mainId;selected=next[active].uid;render();
  }
  // Presentation only: mission model, XML and validation are unchanged.
  function draw3D(){
    const host=$('designer-3d');if(!window.GCSFlowCanvas||!definitions[active])return;
    host.className='behavior-network';const nodes=[];walk(definitions[active],n=>nodes.push({id:n.uid,label:n.ports.name||n.type,task:n.type,kind:catalog.nodes[n.type]?.category||'Node',children:n.children.map(c=>c.uid)}));
    window.GCSFlowCanvas.render(host,{nodes,selected,onSelect:n=>{selected=n.id;renderTree();const f=locate();if(f)renderProperties(f.node);}});
  }
  function resize(){/* The SVG viewBox fits the available workspace automatically. */}
  async function init(){
    try{const response=await fetch('/data/bt_catalog.json',{cache:'no-store'});if(!response.ok)throw new Error('Node catalog unavailable');catalog=await response.json();$('designer-node-count').textContent=`${Object.keys(catalog.nodes).length} TYPES`;$('designer-registry-source').textContent='Installed registry · '+catalog.registry_hash.slice(0,12);renderLibrary();applyTemplate();}
    catch(e){notify(e.message,true);return;}
    $('designer-search').oninput=renderLibrary;$('designer-category').onchange=renderLibrary;$('designer-load-template').onclick=applyTemplate;
    $('designer-definition').onchange=()=>{active=$('designer-definition').value;selected=definitions[active].uid;render();};
    $('designer-up').onclick=()=>move(-1);$('designer-down').onclick=()=>move(1);
    $('designer-remove').onclick=()=>{const f=locate();if(!f?.parent)return notify('Keep one root. Select a child to remove, or load another template.',true);f.parent.children.splice(f.parent.children.indexOf(f.node),1);selected=f.parent.uid;render();};
    $('designer-wrap').onclick=()=>{const f=locate();if(!f)return;const n=identify({type:'Sequence',ports:{},children:[]});n.children=[f.node];if(f.parent)f.parent.children[f.parent.children.indexOf(f.node)]=n;else definitions[active]=n;selected=n.uid;render();};
    $('designer-import').onclick=()=>$('designer-file').click();$('designer-file').onchange=async e=>{try{const file=e.target.files[0];if(!file)return;if(file.size>1000000)throw new Error('XML exceeds 1 MB');importXml(await file.text());}catch(err){notify(err.message,true);}finally{e.target.value='';}};
    $('designer-download').onclick=()=>{const url=URL.createObjectURL(new Blob([xml()],{type:'application/xml'}));const a=el('a');a.href=url;a.download='aerixis-mission.xml';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
    $('designer-copy').onclick=async()=>{try{await navigator.clipboard.writeText(xml());notify('Mission XML copied.');}catch(e){notify('Clipboard unavailable; use Save XML.',true);}};
    $('designer-to-fleet').onclick=()=>{try{const errors=issues();if(errors.length)throw new Error(errors[0]);window.FleetUI.setXml(xml(),'Mission Designer');switchView('fleet');}catch(e){window.FleetUI.error(e.message,'Mission Designer');}};
    $('designer-use-waypoints').onclick=()=>{try{if(typeof missionTreeNodes==='undefined')throw new Error('Map plan unavailable');const points=missionTreeNodes.filter(n=>n.type==='GoTo');if(!points.length)throw new Error('Create map waypoints first');let replaced=false;walk(definitions[active],n=>{if(n.ports.name==='CheckpointedCoverage'){n.children=points.map((p,i)=>identify({type:'SurveyWaypoint',ports:{segment_id:`leg_${i}`,pose:[p.x,p.y,p.z].join(';'),speed:String(p.speed||.5),timeout_s:'120'},children:[]}));replaced=true;}});if(!replaced)throw new Error('This tree has no CheckpointedCoverage group');render();}catch(e){notify(e.message,true);}};
    window.addEventListener('resize',resize);
  }
  window.MissionDesigner={resize,xml,importXml};
  if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',init);else init();
})();
