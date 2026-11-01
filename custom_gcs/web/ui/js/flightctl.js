// Sidebar flight controls, always visible on every page: safety switch, AI avoidance, RTL, LAND, DISARM.
// AI avoidance: each drone bends its legs around the others (learned brain + ORCA shield onboard);
// OFF = the usual straight legs and stop rules. Every drone starts OFF after a restart.
// RTL / LAND go over the telemetry radio to each drone's FC guardian (PX4's own modes, works
// with AERIXIS core down); DISARM goes to PX4 through AERIXIS core and PX4 refuses it in flight.
// The safety switch lives in the GCS server, so SAFE blocks mission Start from every window.
import { h, icon, api, store, toast, fill, linksOf } from './core.js';

// Flight mode panel (2026-10-25): PX4's mode per aircraft, set from the GCS.
// Offboard -> AERIXIS core (it streams PX4's setpoints); Position / Altitude hold -> PX4 directly
// (PX4 answers; manual modes need RC sticks or a joystick); RTL / Land -> FC guardian (as below).
const MODES = [
  { id: 'OFFBOARD', label: 'Offboard', ic: 'bolt', nav: ['OFFBOARD'], hint: 'AERIXIS core flies: missions, go-to, hover.' },
  { id: 'POSCTL', label: 'Position hold', ic: 'gps', nav: ['POSCTL'], hint: 'PX4 holds position; RC sticks move it.' },
  { id: 'ALTCTL', label: 'Altitude hold', ic: 'wave', nav: ['ALTCTL'], hint: 'PX4 holds height; RC sticks fly it.' },
  { id: 'RTL', label: 'RTL', ic: 'home', nav: ['RTL'], emergency: true, hint: 'Fly home and land (FC guardian).' },
  { id: 'LAND', label: 'Land', ic: 'land', nav: ['LAND', 'DESCEND'], emergency: true, hint: 'Land where it is (FC guardian).' },
];
const radioDrones = () => (store.fleet || []).filter(d => String(d.endpoint || '').startsWith('mavlink://'));
const systemOf = d => String(d.endpoint).slice('mavlink://'.length);
const navOf = d => linksOf(d).fc?.status?.nav || null;

const ACTIONS = {
  RTL: { label: 'RTL', ic: 'home', kind: 'warn', title: 'Return to launch',
    text: 'Each selected drone flies home and lands (PX4 RTL through its FC guardian). The running mission stops.' },
  LAND: { label: 'Land', ic: 'land', kind: 'warn', title: 'Land now',
    text: 'Each selected drone lands where it is (PX4 LAND through its FC guardian). The running mission stops.' },
  DISARM: { label: 'Disarm', ic: 'power', kind: 'danger', title: 'Disarm motors',
    text: 'Only for drones on the ground: PX4 refuses to disarm a flying drone. Use Land first.' },
};

let root = null, safetyBtn = null, aiBtn = null, modeBtn = null, modeDialog = null;

export function mountFlightControls(parent) {
  safetyBtn = h('button', { class: 'fc-btn fc-safety', onclick: toggleSafety, title: 'Safety switch' });
  aiBtn = h('button', { class: 'fc-btn fc-ai', onclick: toggleAi, title: 'AI avoidance' });
  modeBtn = h('button', { class: 'fc-btn fc-mode', onclick: openModes, title: 'Flight mode' });
  const action = (id, extra = '') => { const a = ACTIONS[id];
    return h('button', { class: `fc-btn fc-${a.kind} ${extra}`, title: a.title, onclick: () => send(id) },
      h('span', { class: 'fc-ic', html: icon(a.ic) }), h('span', { class: 'fc-text' }, a.label)); };
  aiBtn.classList.add('fc-half'); modeBtn.classList.add('fc-half'); safetyBtn.classList.add('fc-wide');
  // Four rows at the bottom of the menu (2026-10-30): AI + Mode, RTL + Land, Safety, Disarm.
  root = h('div', { class: 'fc-group' },
    h('div', { class: 'side-div', role: 'separator', 'aria-label': 'Flight controls' }),
    h('div', { class: 'fc-grid' }, aiBtn, modeBtn, action('RTL', 'fc-half'), action('LAND', 'fc-half'), safetyBtn, action('DISARM', 'fc-wide')));
  parent.append(root);
  update(store);
}

// What the drones themselves report (the runtime's switch), not what was last clicked.
function aiState(s) {
  const radio = (s.fleet || []).filter(d => String(d.endpoint || '').startsWith('mavlink://'));
  const on = radio.filter(d => d.data?.ros?.rescue?.data?.avoidance?.enabled === true).length;
  return { total: radio.length, on, state: !radio.length || on === 0 ? 'off' : on === radio.length ? 'on' : 'mixed' };
}

export function update(s) {
  if (!safetyBtn) return;
  updateModeButton();
  if (modeDialog) modeDialog.refresh();
  const ai = aiState(s);
  aiBtn.classList.toggle('is-on', ai.state === 'on'); aiBtn.classList.toggle('is-mixed', ai.state === 'mixed');
  aiBtn.title = ai.state === 'on' ? 'AI avoidance ON on every drone: legs bend around other drones. Click to switch off.'
    : ai.state === 'mixed' ? `AI avoidance on ${ai.on} of ${ai.total} drones. Click to switch it on for all.`
    : 'AI avoidance off: usual straight legs and stop rules. Click to switch on.';
  fill(aiBtn, [h('span', { class: 'fc-ic', html: icon('ai') }),
    h('span', { class: 'fc-text' }, ai.state === 'on' ? 'AI: ON' : ai.state === 'mixed' ? `AI: ${ai.on}/${ai.total}` : 'AI: off')]);
  const safe = s.state?.safety_safe !== false;
  safetyBtn.classList.toggle('is-safe', safe); safetyBtn.classList.toggle('is-ready', !safe);
  safetyBtn.title = safe ? 'Safety switch SAFE: mission Start is blocked. Click to switch to READY.'
    : 'Safety switch READY: mission Start is allowed. Click to switch to SAFE.';
  fill(safetyBtn, [h('span', { class: 'fc-ic', html: icon(safe ? 'lock' : 'unlock') }),
    h('span', { class: 'fc-text' }, safe ? 'Safety: SAFE' : 'Safety: READY')]);
}

async function toggleSafety() {
  const safe = store.state?.safety_safe !== false;
  if (safe && !(await pick('Switch to READY?', 'Mission Start becomes possible from every GCS window. RTL, Land and Disarm work in both positions.', null, 'Switch to READY', 'primary'))) return;
  try { const r = await api('/api/safety', { safe: !safe }); store.state.safety_safe = r.safe; update(store);
    toast(r.safe ? 'Safety switch SAFE' : 'Safety switch READY', r.safe ? 'Mission Start is blocked.' : 'Mission Start is allowed.', r.safe ? 'ok' : 'warn'); }
  catch (e) { toast('Safety switch not changed', e.message, 'bad'); }
}

async function toggleAi() {
  const ai = aiState(store);
  const enable = ai.state !== 'on';
  if (!ai.total) return toast('No aircraft', 'Add aircraft on the Connect page first.', 'warn');
  if (enable && !(await pick('Switch AI avoidance on?',
    'Every drone flies its legs with the learned avoidance brain and the ORCA shield: near another drone it bends its path, passes on the right and rejoins its line. Takes effect from each drone\'s next leg. The 10 m separation stop, holds and return heights stay active. Tested in simulation only.',
    null, 'Switch AI on', 'primary'))) return;
  try {
    const r = await api('/api/avoidance', { enabled: enable });
    const failed = Object.entries(r.results || {}).filter(([, v]) => v !== 'ok');
    toast(enable ? 'AI avoidance ON sent' : 'AI avoidance off sent',
      failed.length ? 'Not reached: ' + failed.map(([k, v]) => `${k} (${v})`).join(', ') : 'Each drone confirms in its status within a few seconds.',
      failed.length ? 'bad' : enable ? 'warn' : 'ok', 9000);
  } catch (e) { toast('AI avoidance not changed', e.message, 'bad'); }
}

async function send(action) {
  const a = ACTIONS[action];
  const drones = store.fleet.filter(d => String(d.endpoint || '').startsWith('mavlink://'));  // radio aircraft
  if (!drones.length) return toast('No aircraft', 'Add aircraft on the Connect page first.', 'warn');
  const chosen = await pick(a.title, a.text, drones, a.title, a.kind === 'danger' ? 'danger' : 'warn');
  if (!chosen || !chosen.length) return;
  try {
    await api('/api/emergency', { drone_ids: chosen, action, reason: 'menu button' });
    toast(`${a.title}: sent`, `${chosen.join(', ')} — acknowledgement shows in the drone cards and the log.`, 'warn');
  } catch (e) { toast(`${a.title} failed`, e.message, 'bad', 9000); }
}

// Confirm dialog; with `drones` it also asks which aircraft (all ticked by default) and resolves
// to the chosen drone IDs, otherwise to true/false.
function pick(title, text, drones, ok, kind) {
  return new Promise(resolve => {
    const boxes = (drones || []).map(d => h('input', { type: 'checkbox', checked: true, value: d.drone_id }));
    const list = drones ? h('div', { class: 'fc-pick' }, ...drones.map((d, i) =>
      h('label', { class: 'check' }, boxes[i], h('span', {}, h('b', {}, d.label || d.drone_id),
        h('small', { class: 'faint' }, ` system ${String(d.endpoint).slice('mavlink://'.length)}`))))) : null;
    const dialog = h('dialog', { class: 'glass-dialog' },
      h('div', { class: 'card-head' }, h('div', { class: 'card-icon', html: icon('warn') }), h('h3', {}, title)),
      h('p', { class: 'muted' }, text), list,
      h('div', { class: 'row end', style: { marginTop: '16px' } },
        h('button', { class: 'btn', onclick: () => done(null) }, 'Cancel'),
        h('button', { class: `btn ${kind}`, onclick: () => done(drones ? boxes.filter(b => b.checked).map(b => b.value) : true) }, ok)));
    const done = value => { dialog.close(); dialog.remove(); resolve(value); };
    dialog.addEventListener('cancel', () => done(null));
    document.body.append(dialog); dialog.showModal();
  });
}

// ---------- flight mode ----------
function updateModeButton() {
  const drones = radioDrones();
  const navs = [...new Set(drones.map(navOf).filter(Boolean))];
  // Half-width button: the mode name alone (icon says "mode"); the tooltip has the full text.
  const text = !drones.length ? 'Mode' : navs.length === 1 ? navs[0] : navs.length ? `${navs.length} modes` : 'Mode —';
  modeBtn.title = drones.length ? `Flight mode (PX4): ${navs.length ? navs.join(', ') : 'unknown'}. Click to change it.` : 'Flight mode: add aircraft on the Connect page first.';
  fill(modeBtn, [h('span', { class: 'fc-ic', html: icon('sitemap') }), h('span', { class: 'fc-text' }, text)]);
}

function requestState(d) {
  const r = (store.state?.mode_cmds || {})[systemOf(d)];
  if (!r || Date.now() / 1000 - r.t > 30) return null;
  const words = { sending: 'sending…', sent: 'sent to PX4…', accepted: 'accepted', refused: 'refused by PX4', 'no-answer': 'no answer' };
  return { text: `${r.mode} ${words[r.state] || r.state}`, kind: r.state === 'accepted' ? 'ok' : r.state === 'refused' || r.state === 'no-answer' ? 'bad' : 'warn' };
}

function openModes() {
  const drones = radioDrones();
  if (!drones.length) return toast('No aircraft', 'Add aircraft on the Connect page first.', 'warn');
  const boxes = drones.map(d => h('input', { type: 'checkbox', checked: true, value: d.drone_id, onchange: () => refresh() }));
  const rows = drones.map((d, i) => h('label', { class: 'check fm-drone' }, boxes[i],
    h('span', { class: 'fm-name' }, h('b', {}, d.label || d.drone_id), h('small', { class: 'faint' }, ` system ${systemOf(d)}`)),
    h('span', { class: 'fm-now' }), h('span', { class: 'fm-req' })));
  const buttons = MODES.map(m => h('button', { class: `fm-mode${m.emergency ? ' fm-emergency' : ''}`, title: m.hint, onclick: () => choose(m) },
    h('span', { class: 'fm-ic', html: icon(m.ic) }), h('b', {}, m.label), h('small', {}, m.hint)));
  const dialog = h('dialog', { class: 'glass-dialog fm-dialog' },
    h('div', { class: 'card-head' }, h('div', { class: 'card-icon', html: icon('sitemap') }), h('h3', {}, 'Flight mode')),
    h('p', { class: 'muted' }, 'Tick the aircraft, then pick a mode. The highlighted mode is what PX4 reports now.'),
    h('div', { class: 'fm-drones' }, ...rows),
    h('div', { class: 'fm-grid' }, ...buttons),
    h('div', { class: 'row end' }, h('button', { class: 'btn', onclick: () => close() }, 'Close')));
  const chosen = () => boxes.filter(b => b.checked).map(b => b.value);
  function refresh() {
    const live = radioDrones();
    drones.forEach((d, i) => {
      const now = live.find(x => x.drone_id === d.drone_id) || d, nav = navOf(now), req = requestState(now);
      fill(rows[i].querySelector('.fm-now'), [nav ? h('span', { class: 'badge' }, nav) : h('span', { class: 'faint' }, 'no FC data')]);
      fill(rows[i].querySelector('.fm-req'), [req ? h('span', { class: `badge ${req.kind}` }, req.text) : '']);
    });
    const picked = new Set(chosen()), navs = new Set(live.filter(d => picked.has(d.drone_id)).map(navOf).filter(Boolean));
    MODES.forEach((m, i) => buttons[i].classList.toggle('is-current', m.nav.some(n => navs.has(n))));
  }
  async function choose(m) {
    const ids = chosen();
    if (!ids.length) return toast('No aircraft ticked', 'Tick at least one aircraft.', 'warn');
    try {
      if (m.emergency) await api('/api/emergency', { drone_ids: ids, action: m.id, reason: 'flight mode panel' });
      else await api('/api/mode', { drone_ids: ids, mode: m.id });
      toast(`${m.label}: sent`, `${ids.join(', ')} — PX4's answer shows here and in the log.`, m.emergency ? 'warn' : 'ok');
    } catch (e) { toast(`${m.label} not sent`, e.message, 'bad', 9000); }
  }
  function close() { modeDialog = null; dialog.close(); dialog.remove(); }
  dialog.addEventListener('cancel', () => close());
  modeDialog = { refresh };
  document.body.append(dialog); dialog.showModal(); refresh();
}
