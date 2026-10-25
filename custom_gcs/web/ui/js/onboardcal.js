// Onboard calibration: the drone's own onboard panel (http://<onboard-computer>:8090) full window,
// opened from the menu after "Logs & data". Everything else is hidden behind it; the back button
// (top left) or Esc return to the GCS exactly as it was. The panel is loaded in a fresh iframe each
// time (no browser-history entries), so Back never has to walk through the panel's own pages.
// The GCS keeps no file link with the onboard software: this is only a web page served by the
// drone, reached over the drone's network (Wi-Fi / hotspot / Ethernet), not over the telemetry radio.
import { h, $, icon, store, prefs } from './core.js';

const COLOR = '#16a596';
const PORT = 8090;
let view = null, frame = null, droneSel = null, addr = null, status = null, empty = null, open = false, probeId = 0;

const savedAddr = id => (prefs.get('onboardAddr', {}) || {})[id] || '';
function rememberAddr(id, value) {
  const all = prefs.get('onboardAddr', {}) || {};
  all[id] = value; prefs.set('onboardAddr', all);
}

// "10.0.0.5", "10.0.0.5:8090" or "http://10.0.0.5:8090/" -> "http://10.0.0.5:8090"
function panelUrl(text) {
  let t = String(text || '').trim();
  if (!t) return '';
  if (!/^https?:\/\//i.test(t)) t = 'http://' + t;
  try {
    const u = new URL(t);
    if (!u.port) u.port = String(PORT);
    return u.origin;
  } catch { return ''; }
}

// Reachability: an opaque (no-cors) request resolves when the panel answers, rejects when not.
async function reachable(url) {
  const ctl = new AbortController(), t = setTimeout(() => ctl.abort(), 3500);
  try { await fetch(url + '/api/state', { mode: 'no-cors', cache: 'no-store', signal: ctl.signal }); return true; }
  catch { return false; } finally { clearTimeout(t); }
}

function setStatus(kind, text) {
  status.className = 'oc-status ' + kind;
  status.replaceChildren(h('span', { class: 'dot ' + (kind === 'ok' ? 'ok' : kind === 'bad' ? 'bad' : 'warn') }), text);
}

// The GCS server scans its networks for onboard panels (the Pi's IP changes with every hotspot).
// The panel whose drone ID matches the selected aircraft wins, else the first one found.
async function findPanel(fresh, hint) {
  try {
    const q = new URLSearchParams(); if (fresh) q.set('fresh', '1'); if (hint) q.set('hint', hint);
    const r = await fetch('/api/onboard/discover?' + q, { cache: 'no-store' });
    const panels = (await r.json()).panels || [];
    return panels.find(p => p.drone_id === droneSel.value) || panels[0] || null;
  } catch { return null; }
}

async function load(fresh = false) {
  let url = panelUrl(addr.value);
  const id = ++probeId;
  let ok = false;
  if (url && !fresh) {
    setStatus('warn', 'Connecting to ' + url.replace(/^https?:\/\//, '') + '…');
    ok = await reachable(url);
    if (id !== probeId) return;                   // a newer request won
  }
  if (!ok) {                                      // nothing saved, or it moved: search the network
    setStatus('warn', 'Searching the network for the onboard computer…');
    const found = await findPanel(Boolean(url) || fresh, url ? new URL(url).hostname : '');
    if (id !== probeId) return;
    if (found) {
      addr.value = found.address;
      url = panelUrl(found.address);
      ok = await reachable(url);
      if (id !== probeId) return;
    }
  }
  if (!ok) {
    frame.hidden = true; empty.hidden = false;
    empty.querySelector('.oc-why').textContent = url ? `${url.replace(/^https?:\/\//, '')} does not answer, and no onboard computer was found on this network.`
      : 'No onboard computer was found on this network.';
    setStatus('bad', 'Onboard computer not found');
    return;
  }
  rememberAddr(droneSel.value, addr.value.trim());
  empty.hidden = true;
  const target = url + '/#calib';
  if (frame.dataset.src !== target || frame.hidden) newFrame(target);
  setStatus('ok', 'Connected · ' + url.replace(/^https?:\/\//, ''));
}

// A new iframe with its src set before it is attached: its first load adds no history entry.
function newFrame(src) {
  const next = h('iframe', { class: 'oc-frame', title: 'Onboard calibration', allow: 'fullscreen', src });
  next.dataset.src = src;
  if (frame) frame.replaceWith(next); else view.querySelector('.oc-body').prepend(next);
  frame = next;
}

function dropFrame() {
  const blank = h('iframe', { class: 'oc-frame', title: 'Onboard calibration', hidden: true });
  frame.replaceWith(blank);
  frame = blank;
}

function fillDrones() {
  const drones = store.fleet || [];
  const ids = drones.map(d => d.drone_id).join();
  if (ids === droneSel.dataset.ids) return;
  droneSel.dataset.ids = ids;
  const keep = droneSel.value;
  droneSel.replaceChildren(...(drones.length ? drones.map(d => h('option', { value: d.drone_id }, d.label || d.drone_id))
    : [h('option', { value: '_' }, 'Onboard computer')]));
  if ([...droneSel.options].some(o => o.value === keep)) droneSel.value = keep;
}

function build() {
  droneSel = h('select', { class: 'oc-drone', 'aria-label': 'Aircraft', onchange: () => { addr.value = savedAddr(droneSel.value); load(); } });
  addr = h('input', { class: 'oc-addr', placeholder: 'Found automatically, or type the IP', spellcheck: 'false', 'aria-label': 'Onboard computer address',
    onkeydown: e => { if (e.key === 'Enter') load(); } });
  status = h('div', { class: 'oc-status' });
  frame = h('iframe', { class: 'oc-frame', title: 'Onboard calibration', hidden: true, allow: 'fullscreen' });
  empty = h('div', { class: 'oc-empty', hidden: true },
    h('div', { class: 'oc-empty-ic', html: icon('gps') }),
    h('h2', {}, 'Connect to the onboard computer'),
    h('p', { class: 'oc-why' }, ''),
    h('ol', {}, h('li', {}, 'Put this laptop on the same Wi-Fi / hotspot as the drone.'),
      h('li', {}, 'Check the onboard computer is powered (panel port 8090).'),
      h('li', {}, 'Search again, or type its IP above.')),
    h('button', { class: 'btn primary', style: { '--accent': COLOR }, onclick: () => load(true) }, h('span', { html: icon('refresh') }), 'Search again'));
  view = h('div', { class: 'oc-view', id: 'onboard-cal', role: 'dialog', 'aria-modal': 'true', 'aria-label': 'Onboard calibration', hidden: true },
    h('header', { class: 'oc-bar' },
      h('button', { class: 'oc-back', title: 'Back to mission control (Esc)', onclick: () => close() },
        h('span', { html: '<svg viewBox="0 0 24 24"><path d="M15 5l-7 7 7 7"/></svg>' }), h('b', {}, 'Back')),
      h('div', { class: 'oc-title' }, h('span', { class: 'oc-ic', html: icon('gps') }), h('div', {}, h('b', {}, 'Onboard calibration'), h('small', {}, 'Sensors · gimbal · links · camera'))),
      h('div', { class: 'oc-tools' }, droneSel, addr,
        h('button', { class: 'btn primary oc-go', style: { '--accent': COLOR }, onclick: () => load() }, 'Connect'),
        h('button', { class: 'icon-btn', title: 'Search the network again', onclick: () => { frame.hidden = true; load(true); }, html: icon('refresh') }),
        h('button', { class: 'icon-btn', title: 'Open in a new tab', onclick: () => { const u = panelUrl(addr.value); if (u) window.open(u + '/#calib', '_blank', 'noopener'); }, html: icon('arrow') })),
      status),
    h('div', { class: 'oc-body' }, frame, empty));
  document.body.append(view);
  addEventListener('keydown', e => { if (open && e.key === 'Escape' && document.activeElement !== frame) close(); });
  addEventListener('hashchange', () => { if (open) close(); });     // browser Back / another page
}

export function openOnboardCal() {
  if (!view) build();
  if (open) return;
  open = true;
  fillDrones();
  if (!addr.value) addr.value = savedAddr(droneSel.value);
  view.hidden = false;
  document.documentElement.classList.add('oc-open');
  requestAnimationFrame(() => requestAnimationFrame(() => view.classList.add('in')));
  load();
  setTimeout(() => (addr.value ? view.querySelector('.oc-back') : addr).focus(), 50);
}

function close() {
  if (!open) return;
  open = false;
  view.classList.remove('in');
  document.documentElement.classList.remove('oc-open');
  const done = () => { if (!open) { view.hidden = true; dropFrame(); } };
  matchMedia('(prefers-reduced-motion: reduce)').matches ? done() : setTimeout(done, 320);
  document.querySelector('.step[data-page="onboard-cal"]')?.focus();
}

// Menu entry, styled like the workspace steps; not a step itself (no "Next", no guard).
export function onboardCalButton() {
  return h('button', { class: 'step oc-step', 'data-page': 'onboard-cal', style: { '--step': COLOR }, title: 'Onboard calibration — sensors, gimbal, links on the drone',
    onclick: () => openOnboardCal() },
    h('span', { class: 'step-ic', html: icon('gps') }), h('span', {}, h('b', {}, 'Onboard calibration'), h('small', {}, 'Sensors · gimbal · links')),
    h('span', { class: 'tick', html: icon('arrow') }));
}
