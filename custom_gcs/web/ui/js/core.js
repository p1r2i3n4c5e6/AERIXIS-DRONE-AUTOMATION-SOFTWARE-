// Shared helpers, icons, API access and the polled store. No flight logic lives here:
// every action is the same backend call the GCS server already validates.

export const $ = (sel, root = document) => root.querySelector(sel);
export const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];

export function h(tag, attrs = {}, ...children) {
  const node = document.createElement(tag);
  for (const [key, value] of Object.entries(attrs || {})) {
    if (value === undefined || value === null || value === false) continue;
    if (key === 'class') node.className = value;
    else if (key === 'html') node.innerHTML = value;
    else if (key === 'style' && typeof value === 'object') {
      // Custom properties (--accent, --step) need setProperty; Object.assign ignores them.
      for (const [k, v] of Object.entries(value)) k.startsWith('--') ? node.style.setProperty(k, v) : (node.style[k] = v);
    }
    else if (key.startsWith('on') && typeof value === 'function') node.addEventListener(key.slice(2), value);
    else if (value === true) node.setAttribute(key, '');
    else node.setAttribute(key, value);
  }
  for (const child of children.flat()) {
    if (child === null || child === undefined || child === false) continue;
    node.append(child instanceof Node ? child : document.createTextNode(String(child)));
  }
  return node;
}

const P = {
  link: '<path d="M10 14a4 4 0 0 0 5.7 0l3-3a4 4 0 0 0-5.7-5.7l-1 1"/><path d="M14 10a4 4 0 0 0-5.7 0l-3 3a4 4 0 0 0 5.7 5.7l1-1"/>',
  drone: '<circle cx="5" cy="5" r="2.5"/><circle cx="19" cy="5" r="2.5"/><circle cx="5" cy="19" r="2.5"/><circle cx="19" cy="19" r="2.5"/><path d="M7 7l3 3M17 7l-3 3M7 17l3-3M17 17l-3-3"/><rect x="9.5" y="9.5" width="5" height="5" rx="1.5"/>',
  check: '<path d="M20 6 9 17l-5-5"/>',
  shield: '<path d="M12 3 5 6v6c0 4.5 3 7.5 7 9 4-1.5 7-4.5 7-9V6z"/><path d="m9 12 2 2 4-4"/>',
  map: '<path d="M9 4 3 6v14l6-2 6 2 6-2V4l-6 2z"/><path d="M9 4v14M15 6v14"/>',
  upload: '<path d="M12 16V4M7 9l5-5 5 5"/><path d="M4 16v3a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2v-3"/>',
  play: '<path d="M7 4v16l13-8z"/>',
  stop: '<rect x="6" y="6" width="12" height="12" rx="2"/>',
  live: '<circle cx="12" cy="12" r="3"/><path d="M6.3 6.3a8 8 0 0 0 0 11.4M17.7 6.3a8 8 0 0 1 0 11.4M3.5 3.5a12 12 0 0 0 0 17M20.5 3.5a12 12 0 0 1 0 17"/>',
  logs: '<path d="M8 6h12M8 12h12M8 18h12"/><circle cx="4" cy="6" r="1"/><circle cx="4" cy="12" r="1"/><circle cx="4" cy="18" r="1"/>',
  sim: '<rect x="3" y="4" width="18" height="12" rx="2"/><path d="M8 20h8M12 16v4"/><path d="m9 9 2 2 4-4"/>',
  radio: '<path d="M4 20 12 4l8 16"/><path d="M7 14h10"/><circle cx="12" cy="4" r="1.5"/>',
  mesh: '<circle cx="5" cy="12" r="2.5"/><circle cx="19" cy="6" r="2.5"/><circle cx="19" cy="18" r="2.5"/><path d="M7.3 11 16.7 7M7.3 13l9.4 4M19 8.5v7"/>',
  camera: '<path d="M4 8h3l2-3h6l2 3h3v11H4z"/><circle cx="12" cy="13" r="3.5"/>',
  person: '<circle cx="12" cy="6" r="3"/><path d="M6 21v-5a6 6 0 0 1 12 0v5"/>',
  orbit: '<circle cx="12" cy="12" r="2.5"/><ellipse cx="12" cy="12" rx="9" ry="5" transform="rotate(-20 12 12)"/><circle cx="20" cy="9" r="1.5"/>',
  pin: '<path d="M12 21s7-6.2 7-12a7 7 0 0 0-14 0c0 5.8 7 12 7 12z"/><circle cx="12" cy="9" r="2.5"/>',
  gift: '<rect x="3" y="9" width="18" height="12" rx="2"/><path d="M3 13h18M12 9v12M12 9C10 5 6 5 7 8s5 1 5 1 4 2 5-1-3-3-5 1"/>',
  grid: '<rect x="3" y="3" width="7" height="7" rx="1.5"/><rect x="14" y="3" width="7" height="7" rx="1.5"/><rect x="3" y="14" width="7" height="7" rx="1.5"/><rect x="14" y="14" width="7" height="7" rx="1.5"/>',
  tree: '<rect x="9" y="3" width="6" height="4" rx="1"/><rect x="3" y="16" width="6" height="4" rx="1"/><rect x="15" y="16" width="6" height="4" rx="1"/><path d="M12 7v4M6 16v-3h12v3"/>',
  file: '<path d="M6 3h8l4 4v14H6z"/><path d="M14 3v4h4"/>',
  sun: '<circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M2 12h2M20 12h2M4.9 4.9l1.4 1.4M17.7 17.7l1.4 1.4M4.9 19.1l1.4-1.4M17.7 6.3l1.4-1.4"/>',
  palette: '<path d="M12 3a9 9 0 1 0 0 18c1.2 0 1.7-1 1.2-1.9-.5-1-.1-2.1 1.2-2.1H17a4 4 0 0 0 4-4c0-5.5-4-10-9-10z"/><circle cx="7.5" cy="11" r="1.1"/><circle cx="10" cy="7" r="1.1"/><circle cx="14.5" cy="7" r="1.1"/><circle cx="17" cy="11" r="1.1"/>',
  moon: '<path d="M20 14.5A8 8 0 0 1 9.5 4 8 8 0 1 0 20 14.5z"/>',
  info: '<circle cx="12" cy="12" r="9"/><path d="M12 11v5M12 8h.01"/>',
  warn: '<path d="M12 3 2 20h20z"/><path d="M12 10v4M12 17h.01"/>',
  arrow: '<path d="M5 12h14M13 6l6 6-6 6"/>',
  refresh: '<path d="M20 11a8 8 0 0 0-14.9-3M4 13a8 8 0 0 0 14.9 3"/><path d="M4 4v4h4M20 20v-4h-4"/>',
  plus: '<path d="M12 5v14M5 12h14"/>',
  download: '<path d="M12 4v12M7 11l5 5 5-5"/><path d="M4 19h16"/>',
  home: '<path d="M3 11 12 4l9 7"/><path d="M5 10v10h14V10"/>',
  bolt: '<path d="M13 2 4 14h7l-1 8 9-12h-7z"/>',
  battery: '<rect x="2" y="7" width="17" height="10" rx="2"/><path d="M22 11v2"/><path d="M5 10v4M8 10v4"/>',
  gps: '<circle cx="12" cy="12" r="3"/><circle cx="12" cy="12" r="8"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3"/>',
  reconcile: '<path d="M4 12a8 8 0 0 1 14-5.3L20 9"/><path d="M20 4v5h-5"/><path d="M20 12a8 8 0 0 1-14 5.3L4 15"/><path d="M4 20v-5h5"/>',
  wave: '<path d="M2 12h3l3-8 4 16 3-8h7"/>',
  bell: '<path d="M6 8a6 6 0 0 1 12 0c0 7 3 9 3 9H3s3-2 3-9"/><path d="M10.3 21a1.9 1.9 0 0 0 3.4 0"/>',
  sitemap: '<rect x="9" y="3" width="6" height="5" rx="1"/><rect x="2" y="16" width="6" height="5" rx="1"/><rect x="16" y="16" width="6" height="5" rx="1"/><path d="M12 8v4M5 16v-4h14v4"/>',
  code: '<path d="m8 7-5 5 5 5M16 7l5 5-5 5"/>',
  doc: '<path d="M6 3h9l4 4v14H6z"/><path d="M9 12h7M9 16h7M9 8h3"/>',
  chip: '<rect x="6" y="6" width="12" height="12" rx="2"/><path d="M9 2v4M15 2v4M9 18v4M15 18v4M2 9h4M2 15h4M18 9h4M18 15h4"/>',
  satellite: '<path d="m13 7 4 4-6 6-4-4z"/><path d="m16 4 4 4-2 2-4-4zM6 14l4 4-2 2-4-4z"/><path d="M14 21a7 7 0 0 0 7-7"/>',
  laptop: '<rect x="4" y="5" width="16" height="11" rx="1.5"/><path d="M2 19h20"/>',
  lock: '<rect x="5" y="11" width="14" height="10" rx="2"/><path d="M8 11V8a4 4 0 0 1 8 0v3"/>',
  unlock: '<rect x="5" y="11" width="14" height="10" rx="2"/><path d="M8 11V8a4 4 0 0 1 7.5-2"/>',
  land: '<path d="M12 3v11M7 9l5 5 5-5"/><path d="M4 20h16"/>',
  power: '<path d="M12 3v9"/><path d="M6.3 7.3a8 8 0 1 0 11.4 0"/>',
  menu: '<path d="M4 6h16M4 12h16M4 18h16"/>',
  pen: '<path d="M4 20l4.5-1L19 8.5 15.5 5 5 15.5 4 20z"/><path d="M13.5 7l3.5 3.5"/>',
  undo: '<path d="M9 14L4 9l5-5"/><path d="M4 9h10.5a5.5 5.5 0 0 1 0 11H11"/>',
  ai: '<circle cx="6" cy="7" r="2"/><circle cx="18" cy="7" r="2"/><circle cx="12" cy="17" r="2"/><path d="M8 7h8M7 9l4 6M17 9l-4 6"/>',
};
export function icon(name, cls = '') {
  return `<svg class="${cls}" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round">${P[name] || P.info}</svg>`;
}
export function iconEl(name) { const span = document.createElement('span'); span.innerHTML = icon(name); return span.firstChild; }

/** Replace children only when they actually changed. */
export function fill(parent, nodes) {
  const list = nodes.filter(Boolean);
  const current = [...parent.children];
  if (current.length === list.length && current.every((c, i) => c === list[i])) return parent;
  parent.replaceChildren(...list);
  return parent;
}

/** Glass panel. `accent` (cyan, blue, violet, pink, amber, green, coral, indigo, teal) colours its
 *  edge, icon and header wash, so every card on a page reads differently. */
export function card({ title, sub, ic = 'info', right, cls = '', accent = '' }, ...body) {
  const head = h('div', { class: 'card-head' },
    h('div', { class: 'card-icon', html: icon(ic) }),
    h('div', {}, h('h3', {}, title), sub ? h('p', {}, sub) : null),
    right ? h('div', { class: 'right' }, right) : null);
  return h('section', { class: `card ${accent ? 'c-' + accent : ''} ${cls}` }, head, ...body);
}
export function badge(text, kind = '') { return h('span', { class: `badge ${kind}` }, text); }
export function dot(kind, live = false) { return h('span', { class: `dot ${kind} ${live ? 'live' : ''}` }); }
export function hint(text, kind = '') { return h('div', { class: `hint ${kind}`, html: icon(kind === 'bad' || kind === 'warn' ? 'warn' : 'info') }, h('div', {}, text)); }
export function emptyState(ic, title, text, action) {
  return h('div', { class: 'empty' }, h('div', { class: 'e-ic', html: icon(ic) }), h('b', {}, title), h('p', {}, text), action || null);
}
export function fmtAge(s) { if (s === null || s === undefined || !isFinite(s)) return '—'; return s < 1 ? 'now' : s < 60 ? `${s.toFixed(0)} s ago` : `${(s / 60).toFixed(0)} min ago`; }
export const stripAnsi = (text) => String(text || '').replace(/\x1b\[[0-9;]*m/g, '');

// ---------- notification centre ----------
// Every toast and alert is kept here (newest first) for the bell panel; survives a reload.
export const notes = {
  list: (() => { try { return JSON.parse(sessionStorage.getItem('as2gcs.notes') || '[]'); } catch { return []; } })(),
  unread: 0, listeners: new Set(),
  add(kind, title, text = '') {
    const last = this.list[0];
    if (last && last.title === title && last.text === text && Date.now() - last.t < 15000) { last.n = (last.n || 1) + 1; last.t = Date.now(); }
    else this.list.unshift({ t: Date.now(), kind, title, text });
    this.list.length = Math.min(this.list.length, 150);
    if (kind !== 'info') this.unread++;
    try { sessionStorage.setItem('as2gcs.notes', JSON.stringify(this.list)); } catch { /* storage blocked */ }
    this.listeners.forEach(fn => fn());
  },
  clear() { this.list = []; this.unread = 0; try { sessionStorage.removeItem('as2gcs.notes'); } catch { } this.listeners.forEach(fn => fn()); },
};

// ---------- alert pop-up (one at a time, queued) ----------
const alertQueue = [];
let alertOpen = false;
/** Important problem: modal pop-up + notification. Resolves with true for the primary action. */
export function alertBox(title, text, { kind = 'bad', items = [], ok = 'Understood', cancel = null } = {}) {
  notes.add(kind, title, text + (items.length ? ' — ' + items.join('; ') : ''));
  return new Promise(resolve => { alertQueue.push({ title, text, kind, items, ok, cancel, resolve }); pumpAlerts(); });
}
function pumpAlerts() {
  if (alertOpen || !alertQueue.length) return;
  alertOpen = true;
  const a = alertQueue.shift();
  const dialog = h('dialog', { class: `glass-dialog alert ${a.kind}` },
    h('div', { class: 'card-head' }, h('div', { class: 'card-icon', html: icon(a.kind === 'ok' ? 'check' : 'warn') }), h('h3', {}, a.title)),
    h('p', {}, a.text),
    a.items.length ? h('ul', { class: 'alert-items' }, ...a.items.map(x => h('li', {}, x))) : null,
    h('div', { class: 'row end' }, a.cancel ? h('button', { class: 'btn', onclick: () => done(false) }, a.cancel) : null,
      h('button', { class: `btn ${a.kind === 'bad' ? 'danger' : 'primary'}`, onclick: () => done(true) }, a.ok)));
  const done = value => { dialog.close(); dialog.remove(); alertOpen = false; a.resolve(value); setTimeout(pumpAlerts, 150); };
  dialog.addEventListener('cancel', e => { e.preventDefault(); done(false); });
  document.body.append(dialog); dialog.showModal();
}

// ---------- toasts ----------
export function toast(title, text = '', kind = 'info', ms = 6000) {
  notes.add(kind, title, text);
  const box = $('#toasts');
  const node = h('div', { class: `toast ${kind}`, html: icon(kind === 'ok' ? 'check' : kind === 'bad' || kind === 'warn' ? 'warn' : 'info') },
    h('div', {}, h('b', {}, title), text ? h('p', {}, text) : null),
    h('button', { 'aria-label': 'Dismiss', onclick: () => close() }, '×'));
  const close = () => { node.classList.add('out'); setTimeout(() => node.remove(), 300); };
  box.append(node);
  while (box.children.length > 4) box.firstChild.remove();
  if (ms) setTimeout(close, ms);
}

// ---------- API ----------
export async function api(path, body) {
  const options = body === undefined ? {} : {
    method: 'POST', headers: { 'Content-Type': 'application/json', 'X-AS2-GCS': '1' }, body: JSON.stringify(body) };
  const response = await fetch(path, options);
  let data = {};
  try { data = await response.json(); } catch { /* empty body */ }
  if (!response.ok || data.error) throw new Error(data.error || `${path}: HTTP ${response.status}`);
  return data;
}
/** Radio frames an object needs: the codec sends zlib-compressed JSON in 180-byte chunks. */
export async function radioFrames(value) {
  const raw = new TextEncoder().encode(JSON.stringify({ request_id: '0'.repeat(32), drone_id: 'drone0', path: '/api/mission/stage', body: value }));
  try {
    const stream = new Blob([raw]).stream().pipeThrough(new CompressionStream('deflate'));
    return Math.ceil((await new Response(stream).arrayBuffer()).byteLength / 180);
  } catch { return Math.ceil(raw.length / 4 / 180); }
}
export async function sha256(text) {
  const digest = await crypto.subtle.digest('SHA-256', new TextEncoder().encode(text));
  return [...new Uint8Array(digest)].map(b => b.toString(16).padStart(2, '0')).join('');
}

// ---------- persistent preferences (per browser) ----------
export const prefs = {
  get(key, fallback) { try { const v = localStorage.getItem('as2gcs.' + key); return v === null ? fallback : JSON.parse(v); } catch { return fallback; } },
  set(key, value) { try { localStorage.setItem('as2gcs.' + key, JSON.stringify(value)); } catch { /* storage blocked */ } },
};

// ---------- store: one poll loop for every page ----------
export const store = {
  state: null, fleet: [], ports: [], planner: { available: true },
  stateOk: false, fleetOk: false, history: [],
  plan: null,                 // last planning result from /api/plan/kml
  missionType: prefs.get('mission', 'human_geotag_resume'),
  mode: prefs.get('mode', 'sim'),
  // drone_id -> {xml, bundle, label, source}; kept for the browser session so a reload does not
  // lose trees that are already validated onboard.
  assigned: (() => { try { return new Map(JSON.parse(sessionStorage.getItem('as2gcs.assigned') || '[]')); } catch { return new Map(); } })(),
  saveAssigned() { try { sessionStorage.setItem('as2gcs.assigned', JSON.stringify([...this.assigned])); } catch { /* quota */ } },
  listeners: new Set(),
  on(fn) { this.listeners.add(fn); return () => this.listeners.delete(fn); },
  emit() { for (const fn of this.listeners) { try { fn(this); } catch (e) { console.error(e); } } },
  drone(id) { return this.fleet.find(d => d.drone_id === id); },
};

async function pollState() {
  try {
    store.state = await api('/api/state'); store.stateOk = true;
    const t = store.state.traffic || {};
    store.history.push({ tx: t.tx_rate || 0, rx: t.rx_rate || 0 });
    if (store.history.length > 60) store.history.shift();
  } catch { store.stateOk = false; }
}
async function pollFleet() {
  try { store.fleet = (await api('/api/fleet')).drones || []; store.fleetOk = true; }
  catch { store.fleetOk = false; }
}
export async function refreshPorts() {
  try {
    const ports = (await api('/api/ports')).serial_ports || [];
    // USB radios / UARTs first; the many legacy /dev/ttyS* entries last.
    const rank = p => (p.is_usb || /USB|ACM|AMA|serial/.test(p.device)) ? 0 : 1;
    store.ports = ports.sort((a, b) => rank(a) - rank(b) || a.device.localeCompare(b.device, undefined, { numeric: true }));
  } catch { store.ports = []; }
  return store.ports;
}
export function startPolling() {
  // Local HTTP only (the radio is polled by the GCS server at its own pace). 1 s is enough for
  // an operator; a hidden tab drops to one refresh every 5 s to save CPU.
  let tick = 0;
  const loop = async () => {
    await Promise.all([pollState(), tick % 2 === 0 || document.hidden ? pollFleet() : null]);
    tick++; store.emit();
    setTimeout(loop, document.hidden ? 5000 : 1000);
  };
  document.addEventListener('visibilitychange', () => { if (!document.hidden) store.emit(); });
  api('/api/plan/status').then(v => { store.planner = v; }).catch(() => {});
  refreshPorts();
  loop();
}

// ---------- derived helpers used by several pages ----------
export function rescueOf(drone) { return drone?.data?.ros?.rescue?.data || null; }
export function linksOf(drone) { return drone?.data?.ros?.links || {}; }
/** The drone's payload set-up as it reports it (2026-11-01): {capacity, loaded, remaining, servos,
 *  truth}, or null when the drone does not report one (older software, or not connected yet). */
/** The drone camera's field of view as its tracker reports it: {hfov, vfov} in degrees, or null. */
export function cameraOf(drone) {
  const v = linksOf(drone).vision || {};
  return Number.isFinite(v.hfov_deg) && Number.isFinite(v.vfov_deg) ? { hfov: v.hfov_deg, vfov: v.vfov_deg } : null;
}
export function payloadOf(drone) {
  const p = linksOf(drone).payload;
  return p && Number.isFinite(p.capacity) ? p : null;
}
export function isFresh(drone) { return drone?.connection === 'CONNECTED' && drone?.fresh; }
export function linkUp() { const s = store.state; return !!(s && s.pipeline?.link1_mavlink?.status === 'connected' && s.heartbeat_age < 7); }

// ---------- confirmation dialog ----------
export function confirmDialog(title, text, { ok = 'Confirm', kind = 'primary' } = {}) {
  return new Promise(resolve => {
    const dialog = h('dialog', { class: 'glass-dialog' },
      h('div', { class: 'card-head' }, h('div', { class: 'card-icon', html: icon(kind === 'danger' ? 'warn' : 'play') }), h('h3', {}, title)),
      h('p', { class: 'muted' }, text),
      h('div', { class: 'row end', style: { marginTop: '16px' } },
        h('button', { class: 'btn', onclick: () => done(false) }, 'Cancel'),
        h('button', { class: `btn ${kind}`, onclick: () => done(true) }, ok)));
    const done = value => { dialog.close(); dialog.remove(); resolve(value); };
    dialog.addEventListener('cancel', () => done(false));
    document.body.append(dialog); dialog.showModal();
  });
}
