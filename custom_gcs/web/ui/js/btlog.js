// Behaviour-tree execution log per aircraft, built in the GCS from the node states that already
// arrive with every status poll (no extra radio traffic). Each poll is compared with the last one;
// changes become timeline entries. Poll period is 1 s in flight, so a node that starts and
// finishes between two polls appears with its final state only.
import { h, fill, store, badge } from './core.js';

const last = new Map();          // drone_id -> Map(uid -> display status)
const runs = new Map();          // drone_id -> "mission:state"
export const btLog = new Map();  // drone_id -> [{t, kind, text, status}]

const label = n => n.node_name && n.node_name !== n.node_type ? `${n.node_name} (${n.node_type})` : n.node_type;
const statusOf = n => (n.status !== 'IDLE' ? n.status : (n.display_status || 'pending').toUpperCase());

function push(id, entry) {
  const list = btLog.get(id) || [];
  list.push(entry);
  if (list.length > 600) list.splice(0, list.length - 600);
  btLog.set(id, list);
}

export function track(s) {
  const now = Date.now();
  for (const d of s.fleet) {
    const data = d.data; if (!data) continue;
    const exec = data.execution || {}, runKey = `${exec.mission_id || ''}:${exec.state || ''}`;
    if (runs.get(d.drone_id) !== runKey) {
      if (runs.has(d.drone_id) && exec.state) push(d.drone_id, { t: now, kind: 'mission', status: exec.state,
        text: `Mission ${String(exec.mission_id || '').slice(0, 8)} ${exec.state}${exec.detail ? ' — ' + exec.detail : ''}` });
      runs.set(d.drone_id, runKey);
    }
    const nodes = data.bt_nodes || [];
    const prev = last.get(d.drone_id), next = new Map();
    for (const n of nodes) {
      const st = statusOf(n); next.set(n.node_uid, st);
      if (prev && prev.has(n.node_uid) && prev.get(n.node_uid) !== st && st !== 'IDLE' && st !== 'PENDING')
        push(d.drone_id, { t: now, kind: n.node_category || 'Node', status: st, text: label(n), from: prev.get(n.node_uid) });
    }
    // A new tree (other uids) starts a new baseline rather than flooding the log.
    last.set(d.drone_id, next);
  }
}

const KIND = { RUNNING: 'warn', SUCCESS: 'ok', FAILURE: 'bad', FAILED: 'bad', CANCELLED: 'bad', SKIPPED: '', ERROR: 'bad', INTERRUPTED: 'bad' };

export function btLogView(id, { limit = 200 } = {}) {
  const list = (btLog.get(id) || []).slice(-limit).reverse();
  if (!list.length) return h('p', { class: 'faint', style: { margin: '6px 0' } }, 'No tree activity seen yet. Node changes appear here while a mission runs.');
  return h('div', { class: 'btlog' }, ...list.map(e => h('div', { class: `bt-row ${e.kind === 'mission' ? 'mission' : ''}` },
    h('span', { class: 'faint mono' }, new Date(e.t).toLocaleTimeString([], { hour12: false })),
    badge(e.status, KIND[e.status] ?? ''),
    h('span', { class: 'bt-kind' }, e.kind === 'mission' ? 'MISSION' : e.kind),
    h('span', {}, e.text, e.from ? h('span', { class: 'faint' }, `  ← ${e.from.toLowerCase()}`) : null))));
}

export function renderBtLog(box, id, opts) { fill(box, [btLogView(id, opts)]); }
export { store };
