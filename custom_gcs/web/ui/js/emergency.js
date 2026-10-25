// Emergency view. Opens full screen by itself only when AERIXIS core has failed on an aircraft in
// the air (not running on the Pi, or the FC guardian took over after losing it): then only a direct
// command to the flight controller can bring it back. Everything else (onboard abort, PX4 failsafe,
// link loss, critical battery, mission failure, the operator's own RTL/LAND) is a toast only.
// The Emergency button opens it at any time, for every aircraft. Shows which aircraft and why, and lets the
// operator send RTL or LAND to that aircraft or to all of them. The default (highlighted) choice is
// the mission's emergency setting. Commands go through the GCS server as MAVLink COMMAND_LONG
// NAV_RETURN_TO_LAUNCH / NAV_LAND (one per system ID, retried until acknowledged); the onboard FC
// guardian forwards them to PX4 even when the AERIXIS core is down. Hiding the view keeps a red banner.
import { h, $, icon, badge, api, store, toast, notes, fill, isFresh, rescueOf, linksOf, linkUp } from './core.js';
import { sysId, guardianOf, guardianText } from './links.js';
import { needs, missionName } from './missions.js';

const since = new Map();     // reason key -> first time seen (debounce)
const known = new Set();     // reason keys already shown (the view reopens only for new ones)
let incidents = [];          // critical only (AERIXIS core failed in the air): [{d, reasons:[{key,text}]}]
let notices = [];            // every reason, shown in the view when the operator opens it
let wasCritical = false;
let manual = false, open = false, ui = null;
// fc_guardian states (as2_rescue_runtime/guardian.py) that mean it is acting or PX4 is already in failsafe.
const ACK_TEXT = { 4: 'guardian not running or no PX4 link', 3: 'not supported by this gateway' };

function held(key, active, ms) {
  if (!active) { since.delete(key); return false; }
  if (!since.has(key)) since.set(key, Date.now());
  return Date.now() - since.get(key) >= ms;
}

function reasonsFor(d) {
  const id = d.drone_id, data = d.data || {}, r = rescueOf(d) || {}, v = r.vehicle || {}, f = r.flags || {};
  const fc = linksOf(d).fc || {}, gd = guardianOf(d), exec = data.execution || {};
  // Values may be the last ones received before a link loss: still the best knowledge we have.
  const airborne = !!(v.armed || data.active || gd?.airborne);
  // critical: AERIXIS core failed in the air -> the view opens by itself; others are toasts only.
  const out = [], add = (key, text, critical = false) => out.push({ key: `${id}:${key}`, text, critical });
  if (airborne && f.AbortRequested) add('abort', `Onboard abort: ${f.reason || 'reason not reported'}`);
  if (airborne && (fc.faults || []).length) add('failsafe', `PX4 failsafe: ${fc.faults.join(', ')}`);
  if (held(`${id}:bat`, airborne && v.battery != null && v.battery < 0.2, 3000)) add('bat', `Battery critical: ${Math.round(v.battery * 100)} %`);
  if (held(`${id}:link`, airborne && !isFresh(d), 5000)) add('link', `Link loss: no status from the aircraft for ${d.age_s != null ? d.age_s.toFixed(0) + ' s' : 'a while'} (${d.error || d.connection})`);
  if (held('radio', airborne && store.stateOk && !linkUp(), 6000)) add('radio', 'Telemetry radio: no MAVLink heartbeat');
  // The guardian has an emergency (from the GCS or after AERIXIS core loss): always shown here.
  if (airborne && gd?.airborne && gd.as2_alive === false && !gd.emergency) add('as2', `AERIXIS core not running onboard (PX4 ${gd.nav || 'mode unknown'}, guardian ${guardianText(gd.state)}): command the flight controller directly`, true);
  const ge = gd?.emergency;
  if (ge) add('guardian', ge.state === 'STOOD_DOWN' ? `FC guardian ${ge.action}: stood down — pilot or another mode took over`
    : `FC guardian ${ge.action} ${ge.state === 'EMERGENCY_ACTIVE' ? 'active (PX4 confirmed)' : 'commanding PX4'}${ge.source === 'as2_loss' ? ' — AERIXIS core lost' : ge.source === 'gcs' ? ' — GCS command' : ''}${gd.detail ? ': ' + gd.detail : ''}`, ge.source === 'as2_loss');
  if (v.armed && ['FAILURE', 'ERROR'].includes(exec.state)) add('failed', `Mission ${exec.state.toLowerCase()} while airborne: ${exec.detail || 'see BT execution'}`);
  return out;
}

/** Called on every poll by main.js. */
export function watchEmergency(s) {
  const all = s.fleet.map(d => ({ d, reasons: reasonsFor(d) })).filter(x => x.reasons.length);
  notices = all;
  incidents = all.map(x => ({ d: x.d, reasons: x.reasons.filter(r => r.critical) })).filter(x => x.reasons.length);
  const keys = new Set(all.flatMap(x => x.reasons.map(r => r.key)));
  const fresh = [...keys].filter(k => !known.has(k));
  for (const k of known) if (!keys.has(k)) known.delete(k);   // a recurrence alerts again
  if (fresh.length) {
    fresh.forEach(k => known.add(k));
    let critical = false;
    for (const x of all) for (const r of x.reasons) if (fresh.includes(r.key)) {
      critical = critical || r.critical;
      // Non-critical ones are toasted once by alerts.js; only a real emergency is added here.
      if (r.critical) notes.add('bad', `EMERGENCY ${x.d.label}`, r.text);
    }
    if (critical) show(false);
  }
  // Opened by itself and every emergency is over (e.g. the guardian's RTL landed): close it again.
  if (open && !manual && wasCritical && !incidents.length) {
    hide();
    toast('Emergency over', 'No aircraft with a failed AERIXIS core in the air any more.', 'ok', 8000);
  }
  wasCritical = incidents.length > 0;
  render();
}

export function activeEmergency() { return incidents.length > 0; }
export function emergencyShown() { return open || activeEmergency(); }
export function openEmergency() { show(true); }

function show(byOperator) {
  build();
  manual = manual || byOperator;
  open = true; ui.overlay.hidden = false; ui.banner.hidden = true;
  render();
}
function hide() { open = false; manual = false; ui.overlay.hidden = true; render(); }

function build() {
  if (ui) return;
  ui = {};
  ui.list = h('div', { class: 'em-list' });
  ui.all = h('div', { class: 'em-all' });
  ui.sub = h('p');
  ui.overlay = h('div', { class: 'em-overlay', hidden: true, role: 'alertdialog', 'aria-label': 'Emergency' },
    h('div', { class: 'em-panel' },
      h('div', { class: 'em-head' }, h('div', { class: 'em-ic', html: icon('warn') }),
        h('div', {}, h('h2', {}, 'EMERGENCY'), ui.sub),
        h('button', { class: 'btn', onclick: hide }, 'Hide (keep banner)')),
      ui.list, ui.all));
  ui.bannerText = h('span');
  ui.banner = h('div', { class: 'em-banner', hidden: true }, h('span', { html: icon('warn') }), ui.bannerText,
    h('button', { class: 'btn sm', onclick: () => show(false) }, 'Open'));
  document.body.append(ui.overlay, ui.banner);
  document.addEventListener('keydown', e => { if (e.key === 'Escape' && open) hide(); });
}

async function send(ids, action, reason) {
  try {
    const res = await api('/api/emergency', { drone_ids: ids, action, reason });
    toast(`${action} sent`, `System ${res.systems.join(', ')} — waiting for acknowledgement`, 'warn', 8000);
  } catch (e) { toast(`${action} not sent`, e.message, 'bad', 15000); }
  render();
}

function commandOf(d) {
  const c = store.state?.emergency?.[String(sysId(d))];
  if (!c || Date.now() / 1000 - c.t > 900) return null;
  const text = { sending: `${c.action} sending (${c.attempts}/5)`, accepted: `${c.action} accepted by the FC guardian`, 'on-ground': `${c.action}: on ground, nothing to do`,
    refused: `${c.action} REFUSED — ${ACK_TEXT[c.result] || 'MAV_RESULT ' + c.result}`, 'no-ack': `${c.action} — no acknowledgement` }[c.state] || c.state;
  return badge(text, c.state === 'accepted' ? 'ok' : c.state === 'sending' ? 'warn' : c.state === 'on-ground' ? 'info' : 'bad');
}

function actionButtons(ids, label, reason, preferred) {
  return ['RTL', 'LAND'].map(a => h('button', { class: `btn ${a === preferred ? 'danger em-pref' : ''}`, onclick: () => send(ids, a, reason()),
    title: a === preferred ? 'Mission emergency setting' : '' },
    a === 'RTL' ? h('span', { html: icon('home') }) : h('span', { html: icon('download') }),
    `${a === 'RTL' ? 'Return to launch' : 'Land now'}${label ? ' · ' + label : ''}`));
}

// Rows are kept per aircraft and only their text is refreshed: buttons survive the 1 s polls.
const rowsUi = new Map();
function rowFor(d, preferred) {
  let row = rowsUi.get(d.drone_id);
  if (!row || row.preferred !== preferred) {
    row = { preferred, who: h('div', { class: 'em-who' }), why: h('div', { class: 'em-why' }) };
    row.el = h('div', { class: 'em-row' }, row.who, row.why,
      h('div', { class: 'em-act' }, ...actionButtons([d.drone_id], '', () => row.reasonText, preferred)));
    rowsUi.set(d.drone_id, row);
  }
  return row;
}

function render() {
  if (!ui) { if (incidents.length) build(); else return; }
  const preferred = needs().emergency || 'RTL';
  const rows = manual ? store.fleet.map(d => notices.find(x => x.d.drone_id === d.drone_id) || { d, reasons: [{ key: 'op', text: 'Operator emergency' }] }) : incidents;
  if (open) {
    ui.sub.textContent = !incidents.length && rows.length ? `Operator emergency — choose return or land per aircraft or for all · mission ${missionName()} · emergency setting: ${preferred === 'RTL' ? 'return to launch' : 'land'}`
      : rows.length ? `${incidents.length} aircraft with an emergency · mission ${missionName()} · emergency setting: ${preferred === 'RTL' ? 'return to launch' : 'land'}`
      : store.fleet.length ? 'No active emergency.' : 'No aircraft registered.';
    fill(ui.list, rows.map(({ d, reasons }) => {
      const r = rescueOf(d) || {}, v = r.vehicle || {}, fc = linksOf(d).fc || {}, row = rowFor(d, preferred);
      const facts = [v.armed ? 'ARMED' : 'disarmed', fc.status?.nav || '—', v.position ? `alt ${v.position[2].toFixed(1)} m` : 'alt —',
        v.battery != null ? `battery ${Math.round(v.battery * 100)} %` : 'battery —', isFresh(d) ? `status ${(d.age_s ?? 0).toFixed(1)} s` : 'NO STATUS'];
      row.reasonText = reasons.map(x => x.text).join('; ');
      fill(row.who, [h('b', {}, d.label), h('small', {}, `${d.drone_id} · system ${sysId(d) ?? '—'}`), commandOf(d)]);
      fill(row.why, [...reasons.map(x => h('div', {}, x.text)), h('small', {}, facts.join(' · '))]);
      return row.el;
    }));
    const allKey = `${preferred}:${rows.length}:${store.fleet.length}:${incidents.length}`;
    if (allKey !== ui.allKey) {
      ui.allKey = allKey;
      fill(ui.all, rows.length > 1 || (incidents.length && store.fleet.length > 1) ? [h('b', {}, `All ${store.fleet.length} aircraft`),
        ...actionButtons('all', 'all', () => 'operator: all aircraft', preferred)] : []);
    }
  }
  // Banner only for a real emergency the operator hid; command progress is shown in the view.
  const showBanner = !open && incidents.length > 0;
  ui.banner.hidden = !showBanner;
  if (showBanner) ui.bannerText.textContent = `EMERGENCY · ${incidents.map(x => `${x.d.label}: ${x.reasons[0].text}`).join(' · ')}`;
}
