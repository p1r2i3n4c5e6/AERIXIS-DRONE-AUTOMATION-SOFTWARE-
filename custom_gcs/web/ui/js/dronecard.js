// Compact per-aircraft status card shared by Readiness and the Operations console.
// Everything comes from the onboard status reply (/api/fleet): AERIXIS runtime state, the
// flight-controller health tap (fc/health: GPS fix, satellites, HDOP, PX4 faults and warnings)
// and the link summaries (camera/tracker, mesh). Nothing here commands the aircraft.
import { h, badge, fill, isFresh, rescueOf, linksOf, stripAnsi } from './core.js';
import { needs } from './missions.js';
import { droneReady } from './preflight.js';

const NAV_OK = new Set(['OFFBOARD', 'POSCTL', 'ALTCTL', 'LOITER', 'MISSION', 'TAKEOFF', 'LAND', 'RTL']);

export function gnss(d) {
  const fc = linksOf(d).fc?.gps, r = rescueOf(d) || {};
  if (fc) {
    const good = fc.fix_type >= 3 && fc.sats >= 6 && (fc.hdop == null || fc.hdop <= 2.5);
    return { state: fc.fix_type >= 3 ? (good ? 'ok' : 'warn') : 'bad', fix: fc.fix, sats: fc.sats, hdop: fc.hdop, eph: fc.eph,
      text: `${fc.fix} · ${fc.sats} sat · HDOP ${fc.hdop ?? '—'}`, jam: fc.jamming >= 2 };
  }
  return r.gps ? { state: 'ok', fix: 'fix', text: 'fix (no FC detail)' } : { state: 'bad', fix: 'none', text: 'no fix' };
}

/** Pre-flight items. need=false items are shown but do not block Start. */
export function checks(d) {
  const r = rescueOf(d) || {}, v = r.vehicle || {}, f = r.flags || {}, L = linksOf(d), vis = L.vision, sw = L.swarm, fc = L.fc || {}, data = d.data || {};
  const fresh = isFresh(d), g = gnss(d);
  const item = (key, label, state, value, need = true) => ({ key, label, state, value, need });
  const sensors = fc.sensors || [], faults = fc.faults || [];
  const offboardIdle = fc.status?.nav === 'OFFBOARD' && fc.status?.arming !== 'armed';
  const cam = needs().camera;
  return [
    item('link', 'Status link', fresh ? 'ok' : 'bad', fresh ? `${(d.age_s ?? 0).toFixed(1)} s` : d.connection),
    item('as2', 'AERIXIS core', (data.ros?.count || 0) > 5 ? 'ok' : 'bad', `${data.ros?.count ?? 0}`),
    item('fc', 'Flight ctrl', v.connected ? 'ok' : 'bad', v.connected ? (fc.status?.nav || 'connected') : 'no data'),
    // Disarmed in OFFBOARD, PX4 reports preflight failing only because no setpoint stream is
    // flowing yet (the core starts it at take-off): informational there; unhealthy sensors always block.
    item('pre', 'PX4 preflight', !fc.status ? 'warn' : sensors.length ? 'bad' : fc.status.preflight_ok ? 'ok' : offboardIdle ? 'warn' : 'bad',
      !fc.status ? 'no tap' : sensors.length ? sensors.join(' ') : fc.status.preflight_ok ? 'pass' : offboardIdle ? 'offboard idle' : 'FAIL',
      !!fc.status && (sensors.length > 0 || !offboardIdle)),
    item('gnss', 'GNSS', g.state, g.fix + (g.sats != null ? ` ${g.sats}sat` : '')),
    item('pos', 'Position', v.position_valid ? 'ok' : 'bad', v.position_valid ? 'valid' : 'invalid'),
    item('bat', 'Battery', v.battery == null ? 'bad' : v.battery >= .3 ? 'ok' : v.battery >= .25 ? 'warn' : 'bad', v.battery == null ? '—' : `${Math.round(v.battery * 100)}%`),
    item('health', 'Healthy', f.VehicleHealthy ? 'ok' : 'bad', f.VehicleHealthy ? 'yes' : (f.reason || 'no').slice(0, 22)),
    item('motion', 'Motion', r.motion_enabled ? 'ok' : 'warn', r.motion_enabled ? 'enabled' : 'bench'),
    item('flight', 'Flight allow', data.flight_enabled ? 'ok' : 'warn', data.flight_enabled ? 'yes' : 'observe'),
    // Camera counts only when the selected mission detects people (Plan / custom mission builder).
    ...(cam ? [item('cam', 'Camera', vis?.fresh ? 'ok' : 'bad', vis?.fresh ? `${Math.round(vis.fps || 0)} fps` : 'offline'),
      item('gimbal', 'Gimbal', vis?.gimbal_pitch != null ? 'ok' : 'warn', vis?.gimbal_pitch != null ? `${vis.gimbal_pitch.toFixed(0)}°` : '—', false)]
      : [item('cam', 'Camera', vis?.fresh ? 'ok' : 'idle', 'not used', false)]),
    item('mesh', 'Mesh', !sw?.required ? 'ok' : sw.peers_fresh ? 'ok' : 'bad', !sw?.required ? 'single' : sw.peers_fresh ? `${(sw.peers || []).length} peers` : 'peer lost', !!sw?.required),
    // Swarm gate: 10 m plus both aircraft's GNSS uncertainty (≈ 15 m when still).
    ...(sw?.required ? [item('sep', 'Separation', sw.separation_safe ? 'ok' : 'bad', sw.separation_safe ? 'safe' : 'too close')] : []),
    // The manager latches hold_lockout for the whole execution; it only means "reconcile" once landed.
    item('lock', 'Lockout', f.AbortRequested || (data.hold_lockout && !data.active) ? 'bad' : 'ok',
      f.AbortRequested ? 'abort' : data.active ? 'in flight' : data.hold_lockout ? 'reconcile' : 'clear'),
  ].concat(faults.length ? [item('fs', 'PX4 failsafe', 'bad', faults[0])] : []);
}
/** Same verdict as the Swarm start page (preflight.js): selected checks pass and no error. */
export function readyToFly(d) { return droneReady(d); }

/** Everything an operator must read: FC faults, unhealthy sensors, PX4 warnings, aborts, link errors. */
export function problems(d) {
  const r = rescueOf(d) || {}, fc = linksOf(d).fc || {}, out = [];
  if (d.error) out.push(['LINK', d.error, 'bad']);
  if (r.flags?.AbortRequested) out.push(['AERIXIS', `Abort: ${r.flags.reason || 'latched'}`, 'bad']);
  else if (r.flags && !r.flags.VehicleHealthy && r.flags.reason) out.push(['AERIXIS', r.flags.reason, 'warn']);
  if (d.data?.hold_lockout && !d.data?.active) out.push(['AERIXIS', 'Ground lockout — reconcile on the ground', 'warn']);
  for (const s of fc.sensors || []) out.push(['FC', `${s} sensor unhealthy${s.startsWith('MAG') ? ' — check compass calibration / interference' : ''}`, 'bad']);
  for (const f of fc.faults || []) out.push(['FC', f, 'bad']);
  if (fc.status && !fc.status.preflight_ok && !(fc.status.nav === 'OFFBOARD' && fc.status.arming !== 'armed'))
    out.push(['FC', 'PX4 preflight checks failing (see PX4 messages)', 'bad']);
  if (fc.gps?.jamming >= 2) out.push(['GPS', 'Jamming detected', 'bad']);
  const now = Date.now() / 1000;
  for (const m of (fc.messages || []).slice(-3).reverse()) if (now - m.time < 600) out.push(['PX4', m.text, m.severity <= 3 ? 'bad' : 'warn']);
  return out;
}

function meter(label, value, cls = '') { return h('div', {}, h('small', {}, label), h('b', { class: cls }, value)); }

/**
 * Build or refresh a compact card. Keep the returned element between polls (pass it back as
 * `el`): only its contents are replaced. `actions` is {key, build()} for the button row.
 */
export function droneCard(d, { el, selected = false, onSelect, logLines = 0, actions = null } = {}) {
  if (!el) {
    el = h('section', { class: 'dc' });
    el._head = h('div', { class: `dc-head ${onSelect ? 'click' : ''}`, onclick: onSelect || null });
    el._body = h('div', { class: 'dc-body' });
    // Buttons live in their own slot that is rebuilt only when `actions.key` changes, so a
    // click is never lost to the once-a-second refresh of the values around it.
    el._actions = h('div');
    el.append(el._head, el._body, el._actions);
  }
  if (actions?.key !== el._actionsKey) {
    el._actionsKey = actions?.key;
    fill(el._actions, actions ? [actions.build()] : []);
    el._actions.className = actions ? 'dc-body' : '';
    el._actions.style.paddingTop = '0';
  }
  el.classList.toggle('sel', selected);
  const r = rescueOf(d) || {}, v = r.vehicle || {}, fresh = isFresh(d), data = d.data || {}, exec = data.execution || {}, fc = linksOf(d).fc || {};
  const g = gnss(d), list = checks(d), ok = droneReady(d);
  const state = data.active ? ['RUNNING', 'warn'] : !fresh ? [d.connection || 'OFFLINE', 'bad'] : ok ? ['READY', 'ok'] : ['NOT READY', 'bad'];
  fill(el._head, [h('b', {}, d.label), h('span', { class: 'id' }, `${d.drone_id} · SYS ${String(d.endpoint || '').replace('mavlink://', '') || '—'}`),
    h('span', { class: 'right' }, v.armed ? badge('ARMED', 'warn') : null, fc.status?.nav ? badge(fc.status.nav, NAV_OK.has(fc.status.nav) ? '' : 'warn') : null, badge(state[0], state[1]))]);
  const bat = v.battery == null ? null : Math.round(v.battery * 100);
  const meters = h('div', { class: 'meters' },
    meter('Battery', bat == null ? '—' : `${bat}%`, bat == null ? '' : bat < 25 ? 'bad' : bat < 30 ? 'warn' : ''),
    meter('GNSS', g.sats != null ? `${g.sats} sat` : g.fix, g.state === 'ok' ? '' : g.state),
    meter('HDOP', g.hdop != null ? g.hdop.toFixed(1) : '—', g.hdop > 2.5 ? 'warn' : ''),
    meter('Alt / Spd', v.position ? `${v.position[2].toFixed(1)}m ${(v.speed || 0).toFixed(1)}` : '—'));
  const matrix = h('div', { class: 'matrix' }, ...list.map(c => h('div', { class: `mx ${c.state}`, title: c.need ? '' : 'informational' },
    h('span', { class: 'm' }), h('span', { class: 'k' }, c.label), h('span', { class: 'v' }, c.value))));
  const issues = problems(d);
  const faults = issues.length ? h('div', { class: `faults ${issues.some(i => i[2] === 'bad') ? '' : 'warn'}` },
    ...issues.slice(0, 5).map(([src, text]) => h('div', {}, h('span', { class: 'src' }, src), h('span', {}, text)))) : null;
  // Fleet de-confliction: the person this aircraft claimed, or gave way to a peer.
  const person = r.claim ? h('span', { class: 'ok' }, `· responding to ${r.claim.label} (claimed)`)
    : r.yielded && r.active_operation ? h('span', { class: 'warn' }, `· ${r.yielded.label} left to ${r.yielded.owner}`) : null;
  // Mission 2 watch moved away from (or holding clear of) a peer's orbit.
  const watching = r.watch && r.watch.note && r.watch.note !== 'watching planned station' &&
    ['ScanArea', 'SurveyWaypoint'].includes(r.active_operation)
    ? h('span', { class: ['holding', 'stopped short'].includes(r.watch.state) ? 'warn' : 'ok' }, `· ${r.watch.note}`) : null;
  // Approach/orbit hovering because a peer is too close (or peer data is stale).
  const held = r.hold && r.hold.note ? h('span', { class: 'warn' }, `· ${r.hold.note}`) : null;
  // AI avoidance flying this leg around another drone (or clear of all).
  const av = r.avoidance;
  const ai = av && av.enabled ? h('span', { class: av.deviating ? 'warn' : 'faint' },
    `· AI ${av.deviating ? `avoiding (nearest ${av.nearest} m)` : 'on'}`) : null;
  const op = r.active_operation || r.claim ? h('div', { class: 'row', style: { fontSize: '12px' } }, h('span', { class: 'kicker' }, 'Now'), h('b', {}, r.active_operation || 'between steps'),
    exec.state ? h('span', { class: 'faint' }, `· ${exec.state}`) : null, person, watching, held, ai) : null;
  const logs = logLines ? (data.logs || []).slice(-logLines).reverse() : [];
  const log = logs.length ? h('div', { class: 'mini-log' }, ...logs.map(l => h('div', { class: l.level, title: stripAnsi(l.message) },
    `${new Date(l.time * 1000).toLocaleTimeString([], { hour12: false })} ${stripAnsi(l.message)}`))) : null;
  fill(el._body, [meters, op, matrix, faults, log]);
  el._ready = ok;
  return el;
}
