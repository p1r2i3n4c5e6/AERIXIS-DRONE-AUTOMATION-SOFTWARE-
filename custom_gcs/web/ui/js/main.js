// App shell: workspace navigation (coloured icons), theme, notification bell, alert watcher,
// "before you continue" guard between steps, routing.
import { h, $, icon, store, prefs, startPolling, linkUp, isFresh, fill, notes, alertBox } from './core.js';
import { page as connect } from './page-connect.js';
import { page as ready } from './page-readiness.js';
import { page as plan } from './page-plan.js';
import { page as upload } from './page-upload.js';
import { page as live } from './page-live.js';
import { page as logs } from './page-logs.js';
import { watch, LOG_TOASTS, logToasts } from './alerts.js';
import { watchEmergency, openEmergency, emergencyShown } from './emergency.js';
import { loadMissions, catalog } from './missions.js';
import { track } from './btlog.js';
import { watchReconcile, autoReconcile } from './autoreconcile.js';
import { mountFlightControls, update as updateFlightControls } from './flightctl.js?v=15';
import { onboardCalButton } from './onboardcal.js?v=14';

const PAGES = [connect, ready, plan, upload, live, logs];
// Each workspace has its own colour (icon, active card, page accents, primary buttons).
const COLOR = { connect: '#4f8df7', ready: '#22b07d', plan: '#9b6cf6', upload: '#f0a12c', live: '#f2556b', logs: '#22b5c9' };
const ICON = { connect: 'link', ready: 'shield', plan: 'map', upload: 'upload', live: 'live', logs: 'doc' };

function applyTheme(theme) {
  document.documentElement.dataset.theme = theme;
  prefs.set('theme', theme);
  $('#theme').innerHTML = icon(theme === 'dark' ? 'sun' : 'moon');
  $('#theme').title = theme === 'dark' ? 'Light theme' : 'Dark theme';
}

function build() {
  const nav = $('#steps');
  PAGES.forEach((p, i) => {
    nav.append(h('button', { class: 'step', 'data-page': p.id, style: { '--step': COLOR[p.id] }, title: `Step ${i + 1}: ${p.title} — ${p.sub}`, onclick: () => { location.hash = '#' + p.id; } },
      h('span', { class: 'step-ic', html: icon(ICON[p.id]) }), h('span', {}, h('b', {}, p.title), h('small', {}, p.sub)), h('span', { class: 'tick', html: icon('check') })));
    const next = PAGES[i + 1];
    const section = h('section', { class: `page ${p.full ? 'full' : ''}`, id: 'page-' + p.id, style: { '--accent': COLOR[p.id] } });
    if (!p.full) section.append(h('div', { class: 'page-intro' }, h('div', { class: 'pi-ic', html: icon(ICON[p.id]) }),
      h('div', {}, h('h2', {}, p.title), h('p', {}, p.intro))));
    const body = h('div');
    section.append(body);
    if (next && !p.full) section.append(p.nextBar = h('div', { class: 'next-bar' }, h('span', { class: 'muted' }, ''),
      h('button', { class: 'btn primary', style: { '--accent': COLOR[next.id] }, onclick: () => { location.hash = '#' + next.id; } }, `Next: ${next.title}`, h('span', { html: icon('arrow') }))));
    $('#pages').append(section);
    p.mount(body);
  });
  // After Logs & data: the drone's onboard panel, full window (not a workflow step).
  nav.append(onboardCalButton());
  mountFlightControls($('#flight-controls'));
  $('#foot').innerHTML = '';
  $('#foot').append(h('small', { id: 'foot-sub' }, 'Plan locally · execute onboard'),
    h('button', { class: 'side-btn', onclick: () => { location.hash = '#connect'; } }, 'Configure link ↗'),
    h('a', { href: '/classic.html', class: 'side-link' }, 'Classic interface (BT editor)'));
}

// ---------- routing with a "before you continue" check ----------
let current = null, bypass = false, guarding = false;
async function route() {
  if (guarding) { if (current) history.replaceState(null, '', '#' + current.id); return; }  // one guard pop-up at a time
  const id = (location.hash || '#connect').slice(1);
  const target = PAGES.find(p => p.id === id) || PAGES[0];
  // Never cover the emergency view with the step guard.
  if (current && target !== current && !bypass && !emergencyShown() && PAGES.indexOf(target) > PAGES.indexOf(current) && current.leaveWarnings) {
    const items = current.leaveWarnings(store) || [];
    if (items.length) {
      const from = current;
      history.replaceState(null, '', '#' + from.id);           // stay until the operator decides
      guarding = true;
      const go = await alertBox(`Before you leave ${from.title}`, 'Some important items are not finished. You can continue, but check these first:',
        { kind: 'warn', items, ok: `Continue to ${target.title}`, cancel: 'Stay and fix' });
      guarding = false;
      if (go) { bypass = true; location.hash = '#' + target.id; }
      return;
    }
  }
  bypass = false;
  current = target;
  document.querySelectorAll('.page').forEach(s => s.classList.toggle('active', s.id === 'page-' + target.id));
  document.querySelectorAll('.step').forEach(s => s.classList.toggle('active', s.dataset.page === target.id));
  const i = PAGES.indexOf(target);
  document.documentElement.style.setProperty('--page', COLOR[target.id]);
  $('#page-wrap').classList.toggle('flush', !!target.full);
  $('#pages').style.height = target.full ? '100%' : '';
  $('#crumb-step').textContent = `Step ${i + 1} of ${PAGES.length}`;
  $('#crumb-title').textContent = target.title;
  $('#page-wrap').scrollTop = 0;
  window.__page = target.id;
  try { target.update?.(store); } catch (e) { console.error(e); }
  target.shown?.();
}

const NEXT_TEXT = {
  connect: p => p.done ? 'Link and aircraft are live.' : 'Open the link and add at least one aircraft that answers.',
  ready: p => p.done ? 'Whole swarm is ready to fly.' : 'Some selected checks fail — the reason is shown next to each one.',
  plan: p => p.done ? 'Trees generated and assigned.' : 'Choose a mission, accept its conditions, load a KML area.',
  upload: p => p.done ? 'Mission running or finished.' : 'Upload & validate each tree, then Start.',
  live: () => 'Records, BT execution and geotags are in Logs & data.',
};

// ---------- notification bell ----------
function renderNotes() {
  const unread = notes.unread;
  $('#bell-count').textContent = unread ? (unread > 9 ? '9+' : unread) : '';
  $('#notif-sub').textContent = notes.list.length ? `${notes.list.length} messages` : '';
  if ($('#notif-panel').hidden) return;
  fill($('#notif-list'), notes.list.length ? notes.list.map(n => h('div', { class: `note ${n.kind}` },
    h('span', { class: 'n-dot' }), h('div', {}, h('b', {}, n.title, n.n > 1 ? h('span', { class: 'faint' }, ` ×${n.n}`) : null), n.text ? h('p', {}, n.text) : null),
    h('time', {}, new Date(n.t).toLocaleTimeString([], { hour12: false })))) : [h('p', { class: 'faint', style: { padding: '14px' } }, 'No notifications.')]);
}

let ticks = 0;
function tick(s) {
  ticks++;
  try { track(s); } catch (e) { console.error('btlog', e); }
  try { watch(s); } catch (e) { console.error('alerts', e); }
  try { watchEmergency(s); } catch (e) { console.error('emergency', e); }
  try { watchReconcile(s); } catch (e) { console.error('auto reconcile', e); }
  for (const p of PAGES) {
    const visible = window.__page === p.id;
    if (visible || (!p.full && ticks % 5 === 0)) { try { p.update(s); } catch (e) { console.error(p.id, e); } }
    document.querySelector(`.step[data-page="${p.id}"]`)?.classList.toggle('done', !!p.done);
    if (p.nextBar && NEXT_TEXT[p.id]) p.nextBar.firstChild.textContent = NEXT_TEXT[p.id](p);
  }
  const up = linkUp(), fresh = s.fleet.filter(isFresh).length, running = s.fleet.filter(d => d.data?.active).length;
  fill($('#status-pill'), [h('span', { class: `dot ${!s.stateOk ? 'bad' : up ? 'ok' : 'warn'}` }),
    !s.stateOk ? 'GCS server offline' : up ? `Link OK · ${fresh}/${s.fleet.length} aircraft${running ? ` · ${running} flying` : ''}` : 'Link: no heartbeat']);
  $('#mode-pill').textContent = store.mode === 'sim' ? 'Simulation' : 'Field operation';
  try { updateFlightControls(s); } catch (e) { console.error('flight controls', e); }
  const sub = document.getElementById('foot-sub');
  if (sub) sub.textContent = `${s.fleet.length} aircraft · ${running} flying · ${s.state?.gateway_detail || 'no link yet'}`;
}

// ---------- menu bar: hide / show ----------
// Wide screens: the menu collapses to an icon rail (choice remembered). Narrow screens (phone,
// tablet): the menu is a drawer over the page, closed after choosing a workspace.
// The menu button lives in the sidebar (next to the logo); the top-bar one is only for narrow
// screens, where the sidebar is hidden until opened.
function setupMenu() {
  const app = $('.app'), buttons = [$('#menu-btn'), $('#side-menu-btn')];
  const narrow = () => matchMedia('(max-width: 860px)').matches;
  const paint = () => {
    const shown = narrow() ? app.classList.contains('side-open') : !app.classList.contains('side-collapsed');
    for (const btn of buttons) { btn.innerHTML = icon('menu'); btn.title = shown ? 'Hide menu' : 'Show menu'; btn.setAttribute('aria-expanded', String(shown)); }
  };
  if (prefs.get('sideCollapsed', false)) app.classList.add('side-collapsed');
  for (const btn of buttons) btn.onclick = e => {
    e.stopPropagation();
    if (narrow()) app.classList.toggle('side-open');
    else { app.classList.toggle('side-collapsed'); prefs.set('sideCollapsed', app.classList.contains('side-collapsed')); }
    paint();
  };
  $('#side-scrim').onclick = () => { app.classList.remove('side-open'); paint(); };
  $('#steps').addEventListener('click', () => { if (narrow()) { app.classList.remove('side-open'); paint(); } });
  matchMedia('(max-width: 860px)').addEventListener('change', paint);
  paint();
}

// ---------- About (brand tile) ----------
function openAbout() {
  const dialog = h('dialog', { class: 'glass-dialog', style: { '--c': '#04A9B3' } },
    h('div', { style: { padding: '20px 22px 18px' } },
      h('img', { class: 'about-logo light', src: '/ui/brand/aerixis-logo.svg', alt: 'AERIXIS' }),
      h('img', { class: 'about-logo dark', src: '/ui/brand/aerixis-logo-dark.svg', alt: 'AERIXIS' }),
      h('p', {}, h('b', {}, 'AERIXIS Mission Control'), ' — ground control for flood-rescue drone swarms: plan on this laptop, execute onboard each aircraft.'),
      h('p', { class: 'muted', style: { fontSize: '12px' } }, 'Built on the open-source AeroStack2 framework (BSD licence) and PX4.'),
      h('div', { class: 'row end', style: { marginTop: '14px' } }, h('button', { class: 'btn', onclick: () => { dialog.close(); dialog.remove(); } }, 'Close'))));
  dialog.addEventListener('close', () => dialog.remove());
  document.body.append(dialog);
  dialog.showModal();
}

// ---------- loading splash (ui/js/splash.js): report the real loading steps ----------
const splash = window.AerixisSplash;
function splashSteps() {
  if (!splash) return;
  splash.step('ui', 'ok', `${PAGES.length} workspaces`);
  splash.step('server', 'run', location.host);
  let serverFails = 0, firstFleet = true, off = null;
  const onTick = s => {
    if (!splash.open) { off?.(); return; }
    if (s.stateOk) splash.step('server', 'ok', location.host);
    else if (++serverFails >= 8) splash.fail('Ground station server not answering', `${location.host}/api/state does not reply. Start it with start_gcs.sh, then Retry.`);
    else splash.step('server', 'run', 'retrying…');
    if (s.fleetOk && firstFleet) {
      firstFleet = false;
      const fresh = s.fleet.filter(isFresh).length;
      splash.step('fleet', 'ok', s.fleet.length ? `${fresh}/${s.fleet.length} answering · ${linkUp() ? 'link up' : 'no heartbeat'}` : `none added · ${linkUp() ? 'link up' : 'no heartbeat'}`);
    } else if (firstFleet && s.stateOk) splash.step('fleet', 'run', 'polling…');
  };
  off = store.on(onTick);
  splash.step('missions', 'run');
  catalog.listeners.add(function once() {
    catalog.listeners.delete(once);
    const n = catalog.builtin.size, c = catalog.custom.length;
    splash.step('missions', catalog.error ? 'warn' : 'ok', catalog.error ? 'built-in only (' + catalog.error.slice(0, 40) + ')' : `${n} built-in · ${c} custom`);
  });
  if (!window.L) { splash.step('maps', 'warn', 'map library missing'); return; }
  // One imagery tile tells whether online base maps load; offline the browser shows cached tiles only.
  splash.step('maps', 'run', 'checking tiles…');
  const views = document.querySelectorAll('.leaflet-container').length, img = new Image();
  const done = (state, text) => { clearTimeout(t); img.onload = img.onerror = null; splash.step('maps', state, `${views} map views · ${text}`); };
  const t = setTimeout(() => done('warn', 'offline, cached tiles only'), 5000);
  img.onload = () => done('ok', 'imagery online');
  img.onerror = () => done('warn', 'offline, cached tiles only');
  img.src = 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/1/0/0?probe=' + Date.now();
}

document.addEventListener('DOMContentLoaded', () => {
  const saved = prefs.get('theme', null);
  applyTheme(saved || (matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light'));
  $('#theme').onclick = () => { applyTheme(document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark'); PAGES.forEach(p => p.map?.resize?.()); };
  $('#bell').innerHTML = icon('bell') + '<span class="count" id="bell-count"></span>';
  $('#bell').onclick = e => { e.stopPropagation(); const p = $('#notif-panel'); p.hidden = !p.hidden; if (!p.hidden) notes.unread = 0; renderNotes(); };
  $('#notif-clear').onclick = () => notes.clear();
  $('#em-btn').innerHTML = icon('warn') + '<span>Emergency</span>';
  $('#em-btn').onclick = () => openEmergency();
  const logSel = h('select', { onchange: e => logToasts.set(e.target.value) }, ...LOG_TOASTS.map(([v, t]) => h('option', { value: v, selected: logToasts.get() === v }, t)));
  $('#notif-set').append(h('span', {}, 'Onboard log lines as pop-ups'), logSel);
  const arSel = h('select', { onchange: e => autoReconcile.set(e.target.value === 'on') },
    h('option', { value: 'on', selected: autoReconcile.get() }, 'Automatic'), h('option', { value: 'off', selected: !autoReconcile.get() }, 'Manual (Reconcile button)'));
  $('#notif-set').after(h('div', { class: 'notif-set' }, h('span', {}, 'Reset after each flight'), arSel));
  $('#notif-panel').addEventListener('click', e => e.stopPropagation());
  document.addEventListener('click', () => { if (!$('#notif-panel').hidden) { $('#notif-panel').hidden = true; } });
  notes.listeners.add(renderNotes);
  build();
  setupMenu();
  $('#brand').onclick = openAbout;
  window.addEventListener('hashchange', route);
  store.on(tick);
  route();
  renderNotes();
  splashSteps();
  startPolling();
  loadMissions();
});
