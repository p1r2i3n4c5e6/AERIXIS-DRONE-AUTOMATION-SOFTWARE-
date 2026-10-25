// Swarm start checks: one pass/fail list per aircraft, grouped by link (telemetry radio, gateway &
// manager, AERIXIS core nodes, flight controller, GNSS, mesh, camera, payload, guardian), each with the
// reason when it fails. Everything comes from data the GCS already polls (/api/state per-system
// heartbeats, /api/fleet onboard status deltas); no extra radio traffic.
// Mandatory checks always count. Optional checks count only when the operator selects them.
// The camera and payload checks apply only when the selected mission uses them.
import { store, prefs, isFresh, rescueOf, linksOf, linkUp } from './core.js';
import { gnss, problems } from './dronecard.js';
import { CORE, sysId, linkOf, guardianOf, guardianText } from './links.js';
import { needs } from './missions.js';

// key -> [label, default selected]. Not listed here = mandatory.
export const OPTIONAL = {
  flight: ['Flight allowed onboard', true],
  motion: ['Motion enabled (not bench)', true],
  gimbal: ['Gimbal attitude feedback', true],
  guardian: ['FC guardian ready', true],
  hdop: ['GNSS quality (≥ 6 sat, HDOP ≤ 2.5)', true],
  payload_loaded: ['Package confirmed loaded', false],
};
export function selected(key) {
  const sel = prefs.get('startChecks', {});
  return key in sel ? !!sel[key] : OPTIONAL[key][1];
}
export function setSelected(key, on) { const sel = prefs.get('startChecks', {}); sel[key] = !!on; prefs.set('startChecks', sel); }

const GROUPS = [
  ['radio', 'Telemetry radio', 'radio', 'blue'],
  ['manager', 'Gateway & mission manager', 'shield', 'violet'],
  ['as2', 'AERIXIS core', 'drone', 'green'],
  ['fc', 'Flight controller (PX4)', 'chip', 'amber'],
  ['gnss', 'GNSS & position', 'gps', 'teal'],
  ['mesh', 'Drone-to-drone mesh', 'mesh', 'cyan'],
  ['camera', 'Camera & gimbal', 'camera', 'pink'],
  ['payload', 'Payload', 'gift', 'coral'],
  ['guardian', 'FC guardian', 'shield', 'indigo'],
];

/** [{id, title, ic, color, items:[{key,label,state:'ok'|'warn'|'bad'|'na',value,why,need,optional}]}] */
export function linkChecks(d) {
  const s = store.state || {}, n = needs(), r = rescueOf(d) || {}, v = r.vehicle || {}, f = r.flags || {}, data = d.data || {};
  const L = linksOf(d), fc = L.fc || {}, vis = L.vision, sw = L.swarm, mesh = L.mesh, ros = data.ros || {};
  const fresh = isFresh(d), mav = String(d.endpoint || '').startsWith('mavlink://'), sid = sysId(d), link = linkOf(d);
  const out = Object.fromEntries(GROUPS.map(([id, title, ic, color]) => [id, { id, title, ic, color, items: [] }]));
  const add = (group, key, label, ok, value, why, { optional = false, warn = false, na = false } = {}) =>
    out[group].items.push({ key, label, value, optional, need: na ? false : optional ? selected(key) : true,
      state: na ? 'na' : ok ? 'ok' : warn ? 'warn' : 'bad', why: ok || na ? '' : why });

  // Telemetry radio: the GCS port/socket and this aircraft's own heartbeat on it.
  const conn = s.connection || {};
  add('radio', 'radio', 'GCS radio link', store.stateOk && linkUp(), !store.stateOk ? 'GCS offline' : linkUp() ? `${conn.type === 'serial' ? conn.target + ' @ ' + conn.baud : 'UDP :' + conn.listen_port}` : 'no heartbeat',
    !store.stateOk ? 'The GCS server does not answer.' : 'No MAVLink heartbeat on the telemetry link — check radio power, antenna, port and baud (Connect).');
  if (mav) add('radio', 'hb', `Heartbeat from system ${sid}`, link?.heartbeat_age != null && link.heartbeat_age < 5,
    link?.heartbeat_age != null ? `${link.heartbeat_age} s` : 'never', `This aircraft's gateway is silent${link?.heartbeat_age != null ? ` for ${link.heartbeat_age} s` : ''} — Pi powered? gateway running? system ID ${sid} correct?`);
  else add('radio', 'hb', 'Aircraft heartbeat', true, 'HTTPS bench link', '', { na: true });

  // Gateway & mission manager (the onboard status reply over the radio).
  add('manager', 'link', 'Onboard mission manager answers', fresh, fresh ? `${(d.age_s ?? 0).toFixed(1)} s` : (d.connection || 'offline'),
    d.error ? `No fresh status: ${d.error}` : 'No fresh status reply from the onboard manager.');
  const latched = f.AbortRequested || (data.hold_lockout && !data.active);
  add('manager', 'lock', 'No abort / ground lockout', !latched, f.AbortRequested ? 'abort' : latched ? 'lockout' : 'clear',
    f.AbortRequested ? `Abort latched (${f.reason || 'no reason'}) — check the aircraft, then press Reconcile on its card in Upload & start (landed, disarmed).`
      : (data.execution?.state === 'SUCCESS' ? 'Last flight ended normally — clears by itself once the flight controller reports the aircraft disarmed on the ground.'
        : `Last flight ended ${String(data.execution?.state || 'abnormally').toLowerCase()} — check the aircraft, then press Reconcile on its card in Upload & start (landed, disarmed).`));
  add('manager', 'flight', 'Flight allowed onboard', !!data.flight_enabled, data.flight_enabled ? 'yes' : 'observe only',
    'The onboard manager runs in observe-only mode (allow_flight off in the aircraft profile).', { optional: true });
  add('manager', 'motion', 'Motion enabled', !!r.motion_enabled, r.motion_enabled ? 'enabled' : 'bench',
    'Runtime is in bench mode: motion commands are disabled in the aircraft profile.', { optional: true });

  // AERIXIS core nodes.
  const names = (ros.nodes || []).map(x => x.split('/').pop());
  const missing = ros.nodes ? CORE.filter(([x]) => !names.includes(x)).map(([, label]) => label) : null;
  add('as2', 'as2', 'Core nodes running', ros.available && missing && !missing.length,
    !ros.available ? 'ROS down' : missing ? (missing.length ? `${missing.length} missing` : `${ros.count} nodes`) : '—',
    !ros.available ? (ros.error || 'The ROS graph is not available onboard.') : `Missing: ${(missing || []).join(', ') || 'node list not received yet'}.`);
  add('as2', 'health', 'Vehicle healthy (AERIXIS core)', !!f.VehicleHealthy, f.VehicleHealthy ? 'yes' : 'no', `Runtime says unhealthy: ${f.reason || 'no reason reported'}.`);

  // Flight controller: XRCE-DDS data, PX4 arming checks, sensors, failsafe, battery.
  const sensors = fc.sensors || [], faults = fc.faults || [], offboardIdle = fc.status?.nav === 'OFFBOARD' && fc.status?.arming !== 'armed';
  add('fc', 'xrce', 'Micro-XRCE-DDS: PX4 data', !!v.connected, v.connected ? 'flowing' : 'no data',
    'No PX4 data over Micro-XRCE-DDS — agent running? FC serial cable / baud? (UDP 8888 in SITL)');
  add('fc', 'pre', 'PX4 arming checks', !!fc.status && (fc.status.preflight_ok || offboardIdle), !fc.status ? 'no tap' : fc.status.preflight_ok ? 'pass' : offboardIdle ? 'offboard idle' : 'FAIL',
    !fc.status ? 'Flight-controller health tap (fc_health) reports nothing.' : 'PX4 preflight checks fail — see PX4 messages below.');
  add('fc', 'sensors', 'Sensors healthy', !!fc.status && !sensors.length, sensors.length ? sensors.join(' ') : fc.status ? 'all ok' : '—',
    sensors.length ? `Unhealthy: ${sensors.join(', ')}${sensors.some(x => x.startsWith('MAG')) ? ' — calibrate compass / move away from metal' : ''}.` : 'No sensor health data yet.');
  add('fc', 'fs', 'No PX4 failsafe', !faults.length, faults.length ? faults[0] : 'none', `PX4 failsafe active: ${faults.join(', ')}.`);
  const bat = v.battery;
  add('fc', 'bat', 'Battery ≥ 30 %', bat != null && bat >= .3, bat == null ? '—' : `${Math.round(bat * 100)} %`,
    bat == null ? 'No battery reading.' : 'Charge or swap the battery (25 % is the return reserve).');

  // GNSS & position.
  const g = gnss(d);
  add('gnss', 'fix', 'GNSS 3D fix', g.state !== 'bad', g.fix + (g.sats != null ? ` · ${g.sats} sat` : ''), 'No 3D fix — wait outdoors with a clear sky view.');
  add('gnss', 'hdop', 'GNSS quality', g.state === 'ok', g.hdop != null ? `HDOP ${g.hdop}` : g.text, `Weak fix (${g.text}). Geotags and separation margins get worse.`,
    { optional: true, warn: true });
  if (g.jam) add('gnss', 'jam', 'No GNSS jamming', false, 'jamming', 'PX4 reports GNSS jamming.');
  add('gnss', 'pos', 'Position valid', !!v.position_valid, v.position_valid ? 'valid' : 'invalid', 'The estimator has no valid position.');

  // Mesh (fleet only).
  const fleet = store.fleet.length > 1 || !!sw?.required;
  if (fleet) {
    const peers = (sw?.peers || []).map(p => `${p.owner} ${p.age_s != null ? p.age_s.toFixed(1) + ' s' : '—'}`).join(', ');
    add('mesh', 'mesh', 'Peers fresh on the mesh', !!sw?.peers_fresh, sw?.peers_fresh ? `${(sw.peers || []).length} peer(s)` : 'peer lost',
      `${peers ? 'Peer data too old: ' + peers : 'No peers heard'}${mesh?.error ? ' · ' + mesh.error : ''} — mesh radio / base station on?`);
    add('mesh', 'sep', 'Separation ≥ 10 m + GNSS', !!sw?.separation_safe, sw?.separation_safe ? 'safe' : 'too close',
      'Another aircraft is closer than 10 m plus both GNSS uncertainties (≈ 15 m) — move the launch points apart.');
  } else add('mesh', 'mesh', 'Mesh', true, 'single aircraft', '', { na: true });

  // Camera: only when the selected mission detects people.
  if (n.camera) {
    add('camera', 'cam', 'A8 stream & tracker', !!vis?.fresh, vis?.fresh ? `${Math.round(vis.fps || 0)} fps` : vis ? 'stale' : 'not running',
      vis ? 'Camera stream is stale — check the A8 Ethernet link (192.168.144.25) and the tracker.' : 'Tracker not running onboard.');
    add('camera', 'gimbal', 'Gimbal attitude feedback', vis?.gimbal_pitch != null, vis?.gimbal_pitch != null ? `${vis.gimbal_pitch.toFixed(0)}°` : 'none',
      'No gimbal attitude from the A8 — SIYI SDK link (UDP 37260)?', { optional: true });
  } else add('camera', 'cam', 'Camera', true, 'not used by this mission', '', { na: true });

  // Payload: only when the mission drops.
  if (n.payload) {
    add('payload', 'servo', 'Payload servo adapter', names.includes('payload_aux_adapter'), names.includes('payload_aux_adapter') ? 'running' : 'missing',
      'payload_aux_adapter is not running onboard (FC AUX servo).');
    add('payload', 'payload_loaded', 'Package confirmed loaded', !!r.payload?.operator_confirmed, r.payload?.operator_confirmed ? (r.payload.payload_id || 'yes') : 'at upload',
      'Confirm the package in Upload & start (the servo cannot sense the load).', { optional: true, warn: true });
  } else add('payload', 'servo', 'Payload', true, 'not used by this mission', '', { na: true });

  // FC guardian (onboard RTL/LAND path that works when the AERIXIS core is down), once the aircraft reports it.
  const gd = guardianOf(d);
  if (gd) {
    // ON_GROUND / MONITORING: ready to act. NO_FC: it cannot see PX4 and could not act.
    const ok = ['ON_GROUND', 'MONITORING'].includes(gd.state);
    add('guardian', 'guardian', 'Guardian ready', ok, `${guardianText(gd.state)}${gd.action ? ' · ' + gd.action : ''}`,
      `Guardian is ${guardianText(gd.state)}${gd.detail ? ': ' + gd.detail : ''}.`, { optional: true });
  } else add('guardian', 'guardian', 'Guardian', true, 'not reported', '', { na: true });

  return GROUPS.map(([id]) => out[id]).filter(gr => gr.items.length);
}

export function flatChecks(d) { return linkChecks(d).flatMap(g => g.items.map(i => ({ ...i, group: g.title }))); }

/** Errors that block "ready" although every check passes (recent PX4 errors, link errors). */
export function errorsOf(d) {
  const now = Date.now() / 1000, fc = linksOf(d).fc || {};
  const recent = (fc.messages || []).filter(m => m.severity <= 3 && now - m.time < 60).map(m => `PX4: ${m.text}`);
  return [...problems(d).filter(([src, , k]) => k === 'bad' && src !== 'PX4').map(([src, text]) => `${src}: ${text}`), ...recent];
}

export function blockers(d) { return flatChecks(d).filter(c => c.need && c.state !== 'ok'); }
export function droneReady(d) { return !d.data?.active && blockers(d).length === 0 && errorsOf(d).length === 0; }

/** Whole-fleet verdict for the start page and Upload & start. */
export function fleetStatus(s = store) {
  const out = [];
  if (!s.stateOk) out.push('GCS server offline');
  else if (!linkUp()) out.push('Telemetry link: no heartbeat');
  if (!s.fleet.length) out.push('No aircraft registered');
  const notReady = s.fleet.filter(d => !droneReady(d));
  for (const d of notReady) {
    const b = blockers(d), e = errorsOf(d);
    out.push(`${d.label}: ${d.data?.active ? 'already flying' : [...b.map(c => c.label), ...e].slice(0, 4).join(', ')}${b.length + e.length > 4 ? ` +${b.length + e.length - 4}` : ''}`);
  }
  return { ok: out.length === 0, ready: s.fleet.length - notReady.length, total: s.fleet.length, blockers: out };
}
