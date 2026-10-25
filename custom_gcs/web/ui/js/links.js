// Per-aircraft system architecture: every hop from this GCS to the PX4 flight controller with its
// live state, the local vehicle status and the telemetry / mission data each aircraft used.
// Read-only: all values come from /api/state (per-system link counters) and /api/fleet.
import { h, icon, badge, fill, isFresh, rescueOf, linksOf, store } from './core.js';

export const sysId = d => Number(String(d?.endpoint || '').replace('mavlink://', '')) || null;
export const linkOf = d => store.state?.systems?.[String(sysId(d))] || null;
/** FC guardian state: the status poll (needs the mission manager) or, when that is missing or older,
 *  the gateway's 'LINK|GUARDIAN|…' STATUSTEXT (works with the AERIXIS core down). Same shape either way. */
export function guardianOf(d) {
  const polled = d?.data?.ros?.links?.guardian, text = store.state?.guardian?.[String(sysId(d))];
  if (polled && (isFresh(d) || !text)) return polled;
  if (!text) return polled || null;
  if (!text.up) return { state: 'DOWN', detail: text.detail || 'fc_guardian not running', emergency: null, as2_alive: null, via: 'radio' };
  return { state: text.state, action: text.action, detail: text.detail, as2_alive: null, via: 'radio',
    emergency: text.emergency ? { action: text.emergency.action, state: text.emergency.state, source: null } : null };
}

/** Guardian state for display: 'WAITING_FOR_AS2' (onboard state name) → 'waiting for core'. */
export const guardianText = state => state === 'WAITING_FOR_AS2' ? 'waiting for core' : String(state || '—').toLowerCase().replace(/_/g, ' ');

export const CORE = [['platform', 'platform'], ['state_estimator', 'estimator'], ['controller_manager', 'controller'],
  ['GoToBehavior', 'go-to'], ['TakeoffBehavior', 'takeoff'], ['LandBehavior', 'land'], ['rescue_runtime', 'rescue runtime']];

/** Ordered hops: [title, subtitle, icon, colour, state('ok'|'warn'|'bad'|'idle'), badge text, link label to next]. */
export function hops(d) {
  const s = store.state || {}, conn = s.connection || {}, link = linkOf(d), data = d.data || {}, ros = data.ros || {};
  const r = rescueOf(d) || {}, v = r.vehicle || {}, L = linksOf(d), fc = L.fc || {}, vis = L.vision, sw = L.swarm;
  const nodes = (ros.nodes || []).map(n => n.split('/').pop());
  const missing = ros.nodes ? CORE.filter(([n]) => !nodes.includes(n)).map(([, label]) => label) : [];
  const hbOk = link && link.heartbeat_age != null && link.heartbeat_age < 5;
  const fresh = isFresh(d), offboardIdle = fc.status?.nav === 'OFFBOARD' && fc.status?.arming !== 'armed';
  return [
    ['GCS Studio (this laptop)', conn.type ? `${conn.type.toUpperCase()} ${conn.type === 'udp' ? 'listening :' + conn.listen_port : conn.target + ' @ ' + conn.baud}` : 'link closed',
      'laptop', 'blue', !store.stateOk ? 'bad' : conn.type ? 'ok' : 'warn', !store.stateOk ? 'OFFLINE' : conn.type ? 'ACTIVE' : 'NO LINK',
      `MAVLink telemetry · ${link?.heartbeat_hz ? link.heartbeat_hz + ' Hz' : 'no heartbeat'}`],
    ['AERIXIS MAVLink gateway', hbOk ? `system ${sysId(d)} · heartbeat ${link.heartbeat_age}s ago` : `system ${sysId(d) ?? '—'} · no heartbeat from this aircraft`,
      'radio', 'violet', hbOk ? 'ok' : link ? 'bad' : 'idle', hbOk ? 'CONNECTED' : 'NO HEARTBEAT', 'Authenticated mission relay'],
    ['Onboard mission manager', fresh ? `status ${(d.age_s ?? 0).toFixed(1)}s old · flight ${data.flight_enabled ? 'allowed' : 'observe only'}` : (d.error || d.connection || 'not answering'),
      'shield', 'teal', fresh ? 'ok' : d.connection === 'CONNECTING' ? 'warn' : 'bad', fresh ? (data.active ? 'EXECUTING' : 'READY') : (d.connection || 'OFFLINE'), 'ROS 2 graph'],
    ['AERIXIS core', ros.available ? (missing.length ? `missing: ${missing.join(', ')}` : `${ros.count} nodes · platform, estimator, controller, behaviours`) : (ros.error || 'ROS graph unavailable'),
      'drone', 'green', !ros.available ? 'bad' : missing.length ? 'warn' : 'ok', !ros.available ? 'DOWN' : missing.length ? 'INCOMPLETE' : 'RUNNING',
      `Micro-XRCE-DDS bridge · ${v.connected ? 'PX4 data flowing' : 'no PX4 data'}`],
    ['PX4 flight controller', fc.status ? `${fc.status.nav} · ${fc.status.arming} · preflight ${fc.status.preflight_ok ? 'pass' : offboardIdle ? 'waits for offboard stream' : 'FAIL'}${(fc.sensors || []).length ? ' · ' + fc.sensors.join(' ') + ' unhealthy' : ''}` : v.connected ? 'connected (no health data)' : 'no data over DDS',
      'chip', 'amber', !v.connected ? 'bad' : (fc.sensors || []).length || (fc.faults || []).length ? 'bad' : fc.status && !fc.status.preflight_ok && !offboardIdle ? 'warn' : 'ok',
      !v.connected ? 'NO DATA' : v.armed ? 'ARMED' : 'SYNCHRONIZED', `Health tap ${fc.mavlink || 'off'}`],
    ['A8 camera / tracker', vis ? (vis.fresh ? `${vis.source || 'camera'} ${Math.round(vis.fps || 0)} fps · ${vis.locked ? 'locked ' + vis.label : vis.lock_state || 'searching'}` : 'stream stale') : 'tracker not running',
      'camera', 'pink', vis?.fresh ? 'ok' : vis ? 'bad' : 'idle', vis?.fresh ? (vis.locked ? 'LOCKED' : 'LIVE') : 'OFF', null],
    ['Drone-to-drone mesh', !sw?.required ? 'single aircraft (no mesh needed)' : `${(sw.peers || []).map(p => `${p.owner} ${p.age_s != null ? p.age_s.toFixed(1) + 's' : '—'}`).join(' · ')} · separation ${sw.separation_safe ? 'safe' : 'TOO CLOSE'}`,
      'mesh', 'cyan', !sw?.required ? 'idle' : sw.peers_fresh && sw.separation_safe ? 'ok' : 'bad', !sw?.required ? 'N/A' : sw.peers_fresh ? (sw.separation_safe ? 'LINKED' : 'TOO CLOSE') : 'PEER LOST', null],
  ];
}

export function archChain(d) {
  const list = hops(d);
  return h('div', { class: 'chain' }, ...list.flatMap(([title, sub, ic, color, state, label, next], i) => [
    // Camera and mesh hang off the aircraft rather than continuing the chain.
    i === 5 ? h('div', { class: 'chain-split kicker' }, 'Onboard links') : null,
    h('div', { class: `hop ${state} c-${color}` },
      h('div', { class: 'hop-ic', html: icon(ic) }),
      h('div', { class: 'hop-txt' }, h('b', {}, title), h('small', {}, sub)),
      badge(label, state === 'idle' ? '' : state)),
    next && i < 4 ? h('div', { class: `hop-link ${list[i + 1][4] === 'ok' && state === 'ok' ? 'on' : ''}` }, h('span', {}, next)) : null]));
}

export function vehicleStatus(d) {
  const r = rescueOf(d) || {}, v = r.vehicle || {}, fc = linksOf(d).fc || {}, g = r.gps;
  const row = (k, val) => [h('dt', {}, k), h('dd', {}, val)];
  return h('dl', { class: 'kv' },
    ...row('Armed state', v.armed == null ? '—' : v.armed ? 'ARMED' : 'DISARMED'),
    ...row('Flight mode', fc.status?.nav || (v.connected ? 'unknown' : '—')),
    ...row('Altitude (above take-off)', v.position ? `${v.position[2].toFixed(2)} m` : '—'),
    ...row('Ground speed', v.speed != null ? `${v.speed.toFixed(2)} m/s` : '—'),
    ...row('Heading', v.yaw != null ? `${((v.yaw * 180 / Math.PI + 360) % 360).toFixed(0)}°` : '—'),
    ...row('GPS coordinates', g ? `${g.latitude.toFixed(6)}, ${g.longitude.toFixed(6)}` : '—'),
    ...row('GNSS', fc.gps ? `${fc.gps.fix} · ${fc.gps.sats} sat · HDOP ${fc.gps.hdop}` : '—'),
    ...row('Battery', v.battery != null ? `${Math.round(v.battery * 100)} % (${v.battery_source || 'PX4'})` : '—'));
}

export function dataTransfer(d) {
  const l = linkOf(d) || {}, xml = d.xml_bytes;
  return h('div', {},
    h('div', { class: 'xfer' },
      h('div', { class: 'x-up' }, h('small', {}, '↑ Upload (TX)'), h('b', {}, `${l.tx_kb ?? 0} KB`), h('span', {}, `${l.tx_packets ?? 0} packets · ${l.tx_rate ?? 0} KB/s`)),
      h('div', { class: 'x-down' }, h('small', {}, '↓ Download (RX)'), h('b', {}, `${l.rx_kb ?? 0} KB`), h('span', {}, `${l.rx_packets ?? 0} packets · ${l.rx_rate ?? 0} KB/s`))),
    h('dl', { class: 'kv', style: { marginTop: '8px' } },
      h('dt', {}, 'Mission BT XML payload'), h('dd', {}, xml ? `${(xml / 1024).toFixed(2)} KB` + (d.bundle_bytes ? ` + route ${(d.bundle_bytes / 1024).toFixed(1)} KB` : '') : 'not uploaded yet'),
      h('dt', {}, 'Telemetry heartbeat'), h('dd', {}, l.heartbeat_hz ? `${l.heartbeat_hz} Hz (active)` : 'none'),
      h('dt', {}, 'Status poll age'), h('dd', {}, d.age_s != null ? `${d.age_s.toFixed(1)} s` : '—')));
}

/** The three panels for one aircraft, kept as one element and refreshed in place. */
export function archPanel(d, el) {
  if (!el) {
    el = h('div', { class: 'arch' });
    el._chain = h('div'); el._veh = h('div'); el._xfer = h('div');
    el.append(
      h('section', { class: 'card c-pink' }, h('div', { class: 'card-head' }, h('div', { class: 'card-icon', html: icon('sitemap') }), h('div', {}, h('h3', {}, 'System architecture link'), el._sub = h('p'))), el._chain),
      h('div', { class: 'arch-side' },
        h('section', { class: 'card c-teal' }, h('div', { class: 'card-head' }, h('div', { class: 'card-icon', html: icon('gps') }), h('div', {}, h('h3', {}, 'Local vehicle status')), el._armed = h('div', { class: 'right' })), el._veh),
        h('section', { class: 'card c-violet' }, h('div', { class: 'card-head' }, h('div', { class: 'card-icon', html: icon('wave') }), h('div', {}, h('h3', {}, 'Telemetry & mission data')), h('div', { class: 'right' }, badge('MAVLink v2', 'info'))), el._xfer)));
  }
  el._sub.textContent = `${d.label} · ${d.drone_id} · GCS → gateway → AERIXIS core → XRCE-DDS → PX4`;
  const v = rescueOf(d)?.vehicle;
  fill(el._armed, [badge(v?.armed ? 'ARMED' : v ? 'DISARMED' : 'NO DATA', v?.armed ? 'warn' : v ? 'ok' : 'bad')]);
  fill(el._chain, [archChain(d)]);
  fill(el._veh, [vehicleStatus(d)]);
  fill(el._xfer, [dataTransfer(d)]);
  return el;
}
