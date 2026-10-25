// Automatic ground reconcile after a flight. The mission manager clears its ground lockout by itself
// only after a mission that ended SUCCESS; a run where a leg failed but the aircraft still returned
// and landed (or an onboard abort latch) waited for the operator's Reconcile button every time.
// Here the GCS does it for the operator, but only when it is safe: no mission running, PX4 reports
// the aircraft disarmed, fresh status, and no flight-controller fault or unhealthy sensor. The
// aircraft checks again itself (it refuses unless PX4 says disarmed on the ground). A pop-up names
// each aircraft and what is being reset; a faulty FC is never cleared automatically.
import { h, api, toast, notes, isFresh, rescueOf, linksOf, prefs } from './core.js';

export const autoReconcile = { get: () => prefs.get('autoReconcile', true), set: v => prefs.set('autoReconcile', !!v) };

const RETRY_MS = 5000;
const tried = new Map();      // drone_id -> last attempt time
const warned = new Map();     // drone_id -> fault text already reported
const items = new Map();      // drone_id -> {label, what, state: 'busy'|'done'|'wait', note}
let pop = null, hideTimer = null;

/** What has to be reset on this aircraft after its flight, or '' if nothing. */
function needs(d) {
  const data = d.data || {}, f = rescueOf(d)?.flags || {}, exec = data.execution || {};
  const out = [];
  if (data.hold_lockout) out.push(`ground lockout after the mission (${String(exec.state || 'ended').toLowerCase()}${exec.detail ? ': ' + String(exec.detail).slice(0, 80) : ''})`);
  if (f.AbortRequested) out.push(`onboard abort latch (${f.reason || 'no reason given'})`);
  return out.join(' and ');
}

function faultsOf(d) {
  const fc = linksOf(d).fc || {};
  return [...(fc.sensors || []).map(x => `${x} sensor unhealthy`), ...(fc.faults || [])];
}

/** Called on every poll by main.js. */
export function watchReconcile(s) {
  if (!autoReconcile.get() || !s.stateOk) return;
  const now = Date.now();
  for (const d of s.fleet) {
    const id = d.drone_id, what = needs(d), v = rescueOf(d)?.vehicle || {};
    if (!what) { warned.delete(id); continue; }
    if (d.data?.active || !isFresh(d) || v.armed !== false) continue;   // still flying / landing / unknown
    const faults = faultsOf(d);
    if (faults.length) {
      if (warned.get(id) !== faults.join('|')) {
        warned.set(id, faults.join('|'));
        toast(`${d.label}: not reset automatically`, `Flight controller reports: ${faults.join('; ')}. Check the aircraft, then press Reconcile on its card (Upload & start).`, 'warn', 0);
      }
      continue;
    }
    if (now - (tried.get(id) || 0) < RETRY_MS) continue;
    tried.set(id, now);
    run(d, what);
  }
}

async function run(d, what) {
  const id = d.drone_id;
  items.set(id, { label: d.label, what, state: 'busy', note: 'resetting…' });
  render();
  try {
    await api('/api/fleet/command', { drone_id: id, action: 'reconcile' });
    items.set(id, { label: d.label, what, state: 'done', note: 'ready for the next mission' });
    notes.add('ok', `${d.label}: reset after the flight`, what);
  } catch (e) {
    // Usually "not disarmed on the ground yet": retried quietly on the next polls.
    items.set(id, { label: d.label, what, state: 'wait', note: `waiting — ${e.message}` });
  }
  render();
}

function render() {
  if (!pop) {
    pop = { list: h('div', { class: 'ar-list' }) };
    pop.el = h('div', { class: 'ar-pop', role: 'status', 'aria-live': 'polite', hidden: true },
      h('div', { class: 'ar-head' }, h('b', {}, 'Reloading after the flight'),
        h('button', { class: 'ar-x', 'aria-label': 'Close', onclick: () => { items.clear(); render(); } }, '×')),
      pop.list);
    document.body.append(pop.el);
  }
  pop.list.replaceChildren(...[...items.values()].map(it => h('div', { class: `ar-item ${it.state}` },
    h('span', { class: 'ar-ic' }, it.state === 'done' ? '✓' : it.state === 'wait' ? '…' : ''),
    h('div', {}, h('b', {}, it.label), h('p', {}, `${it.what} — ${it.note}`)))));
  pop.el.hidden = items.size === 0;
  clearTimeout(hideTimer);
  if (items.size && [...items.values()].every(it => it.state === 'done')) hideTimer = setTimeout(() => { items.clear(); render(); }, 4000);
}
