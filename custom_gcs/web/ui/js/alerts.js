// Watches every poll for events an operator must not miss and raises each once as a small toast in
// the bottom-right corner (errors stay longer, nothing covers the screen); everything also lands
// in the bell's notification history. Only a failure of AERIXIS core on an aircraft in the air
// (not running on the Pi, or the FC guardian took over after losing it) also opens the full-screen
// emergency view (emergency.js); the Emergency button opens it at any time. Onboard log lines and GCS errors can be shown as toasts too (bell → setting).
import { toast, isFresh, rescueOf, linksOf, linkUp, prefs, stripAnsi } from './core.js';
import { guardianOf, guardianText } from './links.js';

const seen = new Map();   // key -> last value, so each condition alerts once until it clears
const since = new Map();  // key -> first time a condition was observed (debounce)
const ERR = 20000, WARN = 10000;

function edge(key, active, delayMs = 0) {
  const now = Date.now();
  if (!active) { since.delete(key); const was = seen.get(key); seen.set(key, false); return was === true ? 'cleared' : null; }
  if (!since.has(key)) since.set(key, now);
  if (now - since.get(key) < delayMs || seen.get(key) === true) return null;
  seen.set(key, true); return 'raised';
}

// Log lines as toasts: 'warnings' (errors + warnings) | 'errors' | 'off'. At most one toast per
// aircraft every 4 s; the rest are counted into the next one and kept in the bell history.
export const LOG_TOASTS = [['warnings', 'Errors + warnings'], ['errors', 'Errors only'], ['off', 'Off']];
export const logToasts = { get: () => prefs.get('logToasts', 'warnings'), set: v => prefs.set('logToasts', v) };
const logCursor = new Map(), logGate = new Map();
function logLines(id, name, lines, quiet) {
  const last = logCursor.get(id);
  if (lines.length) logCursor.set(id, Math.max(last ?? 0, ...lines.map(l => l.time)));
  if (last === undefined || quiet) return;   // first sight of this aircraft: history, not news
  const mode = logToasts.get();
  if (mode === 'off') return;
  const wanted = lines.filter(l => l.time > last && (l.level === 'ERROR' || l.level === 'FATAL' || (mode === 'warnings' && (l.level === 'WARN' || l.level === 'WARNING'))));
  if (!wanted.length) return;
  const gate = logGate.get(id) || { t: 0, skipped: 0 };
  if (Date.now() - gate.t < 4000) { gate.skipped += wanted.length; logGate.set(id, gate); return; }
  const l = wanted[wanted.length - 1], bad = l.level === 'ERROR' || l.level === 'FATAL';
  const more = wanted.length - 1 + gate.skipped;
  toast(`${name} onboard ${bad ? 'error' : 'warning'}`, stripAnsi(l.message).slice(0, 220) + (more ? `  (+${more} more)` : ''), bad ? 'bad' : 'warn', bad ? ERR : WARN);
  logGate.set(id, { t: Date.now(), skipped: 0 });
}

let primed = false, gcsLogT = null;
export function watch(s) {
  // The first poll only records the current state: no flood of toasts when the page opens.
  if (!s.stateOk && !primed) return;
  const quiet = !primed;
  const raise = (fn) => { if (!quiet) fn(); };

  let e = edge('gcs-offline', !s.stateOk, 3000);
  if (e === 'raised') raise(() => toast('GCS server not responding', 'The ground station backend stopped answering. Check the "Custom GCS" terminal; nothing can be sent until it is back.', 'bad', 0));
  if (e === 'cleared') raise(() => toast('GCS server back online', '', 'ok'));

  const hadLink = seen.get('link-ever');
  if (linkUp()) seen.set('link-ever', true);
  e = edge('link-lost', hadLink && !linkUp(), 6000);
  if (e === 'raised') raise(() => toast('Telemetry link lost', 'No MAVLink heartbeat for several seconds. Aircraft keep executing their onboard tree, but you cannot see or command them. Check the radio / antenna / serial cable.', 'bad', 0));
  if (e === 'cleared') raise(() => toast('Telemetry link restored', '', 'ok'));

  // GCS server errors (e.g. an emergency command that was not acknowledged).
  if (s.stateOk) {
    const gcsLogs = (s.state?.logs || []).filter(l => l.severity === 'ERROR');
    const lastGcs = gcsLogs[gcsLogs.length - 1], key = lastGcs ? `${lastGcs.time}${lastGcs.message}` : '';
    if (lastGcs && gcsLogT !== null && key !== gcsLogT) raise(() => toast('GCS error', lastGcs.message, 'bad', ERR));
    gcsLogT = key;
  }

  for (const d of s.fleet) {
    const id = d.drone_id, name = d.label, data = d.data || {}, r = rescueOf(d) || {}, v = r.vehicle || {};
    const fc = linksOf(d).fc || {}, sw = linksOf(d).swarm || {}, active = !!data.active, exec = data.execution || {};
    if (isFresh(d)) seen.set(`ever:${id}`, true);
    e = edge(`lost:${id}`, seen.get(`ever:${id}`) && !isFresh(d), active ? 5000 : 12000);
    if (e === 'raised') raise(() => toast(`${name}: ${active ? 'no status while flying' : 'status lost'}`, active ? `The onboard manager has not answered (${d.error || d.connection}). The aircraft continues its mission and its own safety checks.` : (d.error || d.connection), active ? 'bad' : 'warn', active ? 0 : WARN));
    if (e === 'cleared') raise(() => toast(`${name}: status restored`, '', 'ok'));

    e = edge(`abort:${id}`, !!r.flags?.AbortRequested);
    if (e === 'raised') raise(() => toast(`${name}: ABORT`, `Onboard safety aborted the mission: ${r.flags.reason || 'reason not reported'}. It returns/lands; reconcile on the ground afterwards.`, 'bad', 0));

    const runEnd = exec.mission_id && ['FAILURE', 'ERROR', 'CANCELLED', 'INTERRUPTED', 'SUCCESS'].includes(exec.state) ? `${exec.mission_id}:${exec.state}` : null;
    const prevEnd = seen.get(`end:${id}`);
    if (runEnd && prevEnd !== runEnd) {
      if (prevEnd !== undefined) raise(() => exec.state === 'SUCCESS' ? toast(`${name}: mission completed`, exec.detail || '', 'ok', WARN)
        : toast(`${name}: mission ${exec.state.toLowerCase()}`, exec.detail || 'The behaviour tree ended without success. See Operations → BT execution and the onboard log for the failing node.', 'bad', 0));
      seen.set(`end:${id}`, runEnd);
    } else if (!runEnd && prevEnd === undefined) seen.set(`end:${id}`, null);

    const faults = [...(fc.sensors || []).map(x => `${x} sensor unhealthy${x.startsWith('MAG') ? ' (compass calibration / interference)' : ''}`), ...(fc.faults || [])];
    const fkey = faults.join('|'), fprev = seen.get(`faults:${id}`);
    if (fkey && fkey !== fprev) raise(() => toast(`${name}: flight controller fault`, `Do not fly until resolved: ${faults.join('; ')}`, 'bad', 0));
    seen.set(`faults:${id}`, fkey);

    for (const m of fc.messages || []) {
      const k = `msg:${id}:${m.time}:${m.text}`;
      if (seen.has(k)) continue; seen.set(k, true);
      if (Date.now() / 1000 - m.time < 120) raise(() => toast(`${name} PX4: ${m.text}`, '', m.severity <= 3 ? 'bad' : 'warn', m.severity <= 3 ? ERR : WARN));
    }

    const bat = v.battery;
    e = edge(`bat:${id}`, bat != null && bat < 0.3, 3000);
    if (e === 'raised') raise(() => toast(`${name}: battery ${Math.round(bat * 100)} %`, bat < 0.25 ? 'Below the 25 % reserve: onboard safety returns the aircraft.' : 'Battery low. Plan to recover this aircraft.', bat < 0.25 ? 'bad' : 'warn', ERR));

    e = edge(`gnss:${id}`, active && fc.gps && fc.gps.fix_type < 3, 2000);
    if (e === 'raised') raise(() => toast(`${name}: GNSS fix lost in flight`, `Fix: ${fc.gps.fix}, ${fc.gps.sats} satellites. Position-based operations and geotags are unreliable.`, 'bad', 0));

    e = edge(`sep:${id}`, active && sw.required && sw.peers_fresh && !sw.separation_safe, 1500);
    if (e === 'raised') raise(() => toast(`${name}: too close to another aircraft`, ['TrackTarget', 'OrbitTarget', 'ApproachOverhead'].includes(r.active_operation)
      ? `Separation below 10 m plus GNSS uncertainty while ${r.active_operation}. Tracking and orbit are NOT separation-gated — watch both aircraft; use Emergency if needed.`
      : 'Separation below 10 m plus GNSS uncertainty. The search/watch gate stops this aircraft and it returns.', 'warn', ERR));

    // FC guardian warnings while airborne (an emergency after AERIXIS core loss opens the Emergency view).
    const gd = guardianOf(d), airborne = !!(v.armed || active || gd?.airborne), gs = gd?.state;
    const gwarn = { PX4_FAILSAFE: 'AERIXIS core lost — PX4 is already returning/landing by its own failsafe.', NO_FC: 'Guardian cannot see PX4: it could not act if the AERIXIS core fails.',
      WAITING_FOR_AS2: 'Airborne but the AERIXIS core has not been seen by the guardian yet.', DOWN: 'fc_guardian is not running: no RTL/LAND path without the AERIXIS core.' }[gs];
    e = edge(`guard:${id}:${gs}`, airborne && !!gwarn, 1500);
    if (e === 'raised') raise(() => toast(`${name}: FC guardian ${guardianText(gs)}`, gwarn, 'bad', 0));

    logLines(id, name, data.logs || [], quiet);
  }
  primed = true;
}
