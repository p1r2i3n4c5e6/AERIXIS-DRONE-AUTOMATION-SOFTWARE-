/* Read-only presentation of executor facts. No commands, timers or outcome inference. */
(function(root){
 'use strict';
 const activeStates=new Set(['STARTING','RUNNING','CANCELLING']);
 function runKey(d){const e=d?.data?.execution;if(!e?.mission_id||!e.revision||!(e.command_id||e.started_at))return null;return [d.drone_id,e.mission_id,e.revision,e.command_id||e.started_at].join(':');}
 function nodeState(n,d){
  if(!d?.fresh)return 'UNKNOWN';
  if(n.status==='RUNNING')return d.data?.active===true?'RUNNING':'UNKNOWN';
  if(n.status==='SUCCESS'||n.display_status==='success')return 'SUCCESS';
  if(n.status==='FAILURE'||n.display_status==='failed')return 'FAILURE';
  if(n.status==='SKIPPED'||n.display_status==='skipped')return 'SKIPPED';
  if(['unknown','cancelled'].includes(n.display_status))return n.display_status.toUpperCase();
  if(n.display_status==='running')return 'UNKNOWN';
  return 'PENDING';
 }
 function summarize(d){const e=d?.data?.execution||{},nodes=d?.data?.bt_nodes||[];const actions=nodes.filter(n=>n.node_category==='Action'||n.node_type==='Action');const steps=actions.length?actions:nodes.filter(n=>!(n.children_uids||[]).length);const state=!d?.fresh?'UNKNOWN':e.state||'WAITING';return {state,key:runKey(d),steps:steps.map(n=>({...n,uiState:nodeState(n,d)})),nodes:nodes.map(n=>({...n,uiState:nodeState(n,d)})),running:steps.filter(n=>nodeState(n,d)==='RUNNING'),done:steps.filter(n=>nodeState(n,d)==='SUCCESS').length,total:steps.length,success:!!d?.fresh&&state==='SUCCESS'&&d.data?.active===false,active:!!d?.fresh&&activeStates.has(state)&&d.data?.active===true};}
 class CompletionTracker{
  constructor(seen=[]){this.observed=new Set();this.seen=new Set((Array.isArray(seen)?seen:[]).slice(-300));}
  update(d){const s=summarize(d);if(!s.key||!d?.fresh)return false;if(s.active){this.observed.add(s.key);while(this.observed.size>500)this.observed.delete(this.observed.values().next().value);}const e=d.data.execution;if(s.success&&Number.isFinite(e.finished_at)&&this.observed.has(s.key)&&!this.seen.has(s.key)){this.seen.add(s.key);this.observed.delete(s.key);while(this.seen.size>300)this.seen.delete(this.seen.values().next().value);return true;}return false;}
 }
 const api={runKey,nodeState,summarize,CompletionTracker};if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.GCSExecutionModel=api;
})(typeof window!=='undefined'?window:globalThis);
