// Step 5 — Operations console: full-size map, every aircraft's card on the left (AERIXIS runtime,
// flight-controller health, links, last log lines), selected aircraft's live tree and details
// in a drawer under the map.
import { h, icon, badge, hint, api, store, toast, fill, confirmDialog, emptyState, isFresh, rescueOf, linksOf, stripAnsi, prefs } from './core.js';
import { createMap, gcsPosition } from './mapview.js';
import { sim } from './simarena.js';
import { fromLive, treeViewer, legend } from './tree.js';
import { droneCard, gnss } from './dronecard.js';
import { btLogView } from './btlog.js';
import { archPanel } from './links.js';

export const OPS = { SurveyWaypoint: 'Flying survey lane', ApproachOverhead: 'Approaching person (gimbal → nadir)', GeotagOverhead: 'Geotagging person',
  DropPayload: 'Releasing payload', TrackTarget: 'Tracking person', OrbitTarget: 'Orbiting person', ReturnToLaunch: 'Returning to launch',
  ManagedTakeoff: 'Taking off', ManagedLand: 'Landing', PauseCoverage: 'Holding', ResumeCoverage: 'Resuming search', ScanArea: 'Watching area (yaw sweep)',
  SetSearchArea: 'Loading search area', SetGimbalSearch: 'Gimbal to search pose', ReleaseTargetLock: 'Releasing camera lock', RequestRecovery: 'Recovery hold' };

function gimbalGauge(pitch, yaw) {
  const p = pitch == null ? 0 : Math.max(-90, Math.min(25, pitch)), a = (-p) * Math.PI / 180;
  const x = 60 + 44 * Math.cos(a), y = 20 + 44 * Math.sin(a), nadir = pitch != null && pitch <= -84;
  const g = h('div', { class: 'gauge', style: { height: '80px', width: '120px' } });
  g.innerHTML = `<svg viewBox="0 0 120 80"><path d="M16 20 A44 44 0 0 1 60 64" fill="none" stroke="var(--line)" stroke-width="6"/>
    <line x1="60" y1="20" x2="${x}" y2="${y}" stroke="${nadir ? 'var(--ok)' : 'var(--accent)'}" stroke-width="3"/><circle cx="60" cy="20" r="5" fill="var(--muted)"/></svg>
    <div class="g-val" style="place-items:start end;padding:2px 4px;font-size:13px"><div>${pitch == null ? '—' : pitch.toFixed(0) + '°'}<small>${nadir ? 'nadir' : 'pitch'} · yaw ${yaw == null ? '—' : yaw.toFixed(0)}°</small></div></div>`;
  return g;
}

export const page = {
  id: 'live', title: 'Operations', sub: 'Map · fleet · live tree', ic: 'live', full: true,
  intro: '',
  mount(root) {
    this.root = root; this.selected = null; this.cards = new Map();
    this.summary = h('span', { class: 'faint', style: { fontSize: '11.5px' } });
    this.list = h('div', { class: 'ops-list' });
    const side = h('div', { class: 'ops-side' }, h('div', { class: 'bar' }, h('span', { class: 'kicker' }, 'Fleet'), this.summary), this.list);
    this.mapBox = h('div', { class: 'map' });
    const overlay = h('div', { class: 'ops-overlay' },
      h('button', { class: 'btn sm', onclick: () => { this.map?.resetView(); this.map?.follow(this.points || []); } }, 'Follow fleet'),
      h('button', { class: 'btn sm', onclick: () => this.map?.clearTrails() }, 'Clear trails'));
    this.drawerTitle = h('b', { style: { fontSize: '12px' } }, '—');
    this.treeBadge = badge('—');
    this.tab = prefs.get('opsTab', 'tree');
    this.tabBar = h('div', { class: 'tabs', style: { margin: 0 } }, ...[['tree', 'Live tree'], ['bt', 'BT execution'], ['links', 'Links & data']].map(([id, t]) =>
      h('button', { 'data-tab': id, class: id === this.tab ? 'active' : '', onclick: () => this.setTab(id) }, t)));
    this.drawer = h('div', { class: `ops-drawer ${prefs.get('drawer', true) ? '' : 'closed'}` },
      h('div', { class: 'bar' }, this.drawerTitle, this.treeBadge, this.tabBar, h('span', { style: { flex: 1 } }),
        h('button', { class: 'btn sm', onclick: () => { this.drawer.classList.toggle('closed'); prefs.set('drawer', !this.drawer.classList.contains('closed')); this.map?.resize(); } }, 'Show / hide')),
      this.bodyTree = h('div', { class: 'body' }, this.treeBox = h('div'), this.detail = h('div', { class: 'ops-detail' })),
      this.bodyBt = h('div', { class: 'body-scroll' }),
      this.bodyLinks = h('div', { class: 'body-scroll' }));
    this.archPanels = new Map();
    const main = h('div', { class: 'ops-main' }, h('div', { class: 'ops-map' }, this.mapBox, overlay), this.drawer);
    root.append(h('div', { class: 'ops' }, side, main));
    this.map = createMap(this.mapBox);
    this.viewer = treeViewer(this.treeBox);
  },
  shown() { this.map?.resize(); this.planShown = null; this.setTab(this.tab); },
  setTab(id) {
    this.tab = id; prefs.set('opsTab', id);
    this.tabBar.querySelectorAll('button').forEach(b => b.classList.toggle('active', b.dataset.tab === id));
    this.bodyTree.style.display = id === 'tree' ? '' : 'none';
    this.bodyBt.style.display = id === 'bt' ? '' : 'none';
    this.bodyLinks.style.display = id === 'links' ? '' : 'none';
    this.update(store);
  },
  select(id) { this.selected = id; const d = store.drone(id), g = rescueOf(d)?.gps; if (g) this.map?.focus([g.latitude, g.longitude]); this.update(store); },
  async cancel(d) {
    if (!(await confirmDialog(`Cancel ${d.label}?`, 'The onboard tree is halted and the aircraft holds position. It does not land by itself.', { ok: 'Cancel mission', kind: 'danger' }))) return;
    try { await api('/api/fleet/command', { drone_id: d.drone_id, action: 'cancel', mission_id: d.data.execution.mission_id, revision: d.data.execution.revision }); toast(`${d.label}: cancel requested`, '', 'warn'); }
    catch (e) { toast('Cancel failed', e.message, 'bad'); }
  },
  update(s) {
    const drones = s.fleet;
    if (!drones.length) { fill(this.list, [emptyState('drone', 'No aircraft', 'Add aircraft in step 1.')]); return; }
    if (!this.selected || !drones.some(d => d.drone_id === this.selected)) this.selected = drones[0].drone_id;
    const flying = drones.filter(d => d.data?.active).length, tags = drones.reduce((n, d) => n + (rescueOf(d)?.geotags || []).length, 0);
    this.summary.textContent = `${drones.filter(isFresh).length}/${drones.length} online · ${flying} flying · ${tags} geotags`;
    // cards
    fill(this.list, drones.map(d => {
      const data = d.data || {}, L = linksOf(d), vis = L.vision || {}, id = d.drone_id;
      const chips = [[`CAM ${vis.fresh ? (vis.locked ? 'LOCK ' + vis.label : 'ok') : 'off'}`, vis.fresh ? (vis.locked ? 'info' : 'ok') : 'bad'],
        [`FC ${rescueOf(d)?.vehicle?.connected ? 'ok' : 'off'}`, rescueOf(d)?.vehicle?.connected ? 'ok' : 'bad'],
        [`TAP ${L.fc?.mavlink || 'off'}`, L.fc?.mavlink === 'ok' ? 'ok' : ''],
        L.swarm?.required ? [`MESH ${L.swarm.peers_fresh ? 'ok' : 'lost'}`, L.swarm.peers_fresh ? 'ok' : 'bad'] : null].filter(Boolean);
      const actions = { key: JSON.stringify([chips, !!data.active]), build: () => h('div', { class: 'row', style: { gap: '4px' } },
        ...chips.map(([t, k]) => badge(t, k)),
        data.active ? h('button', { class: 'btn sm danger', style: { marginLeft: 'auto' }, onclick: e => { e.stopPropagation(); this.cancel(store.drone(id)); } }, 'Cancel') : null) };
      const el = droneCard(d, { el: this.cards.get(d.drone_id), selected: d.drone_id === this.selected, onSelect: () => this.select(id), logLines: 3, actions });
      this.cards.set(d.drone_id, el);
      return el;
    }));
    // map
    if (store.plan && this.planShown !== store.plan) { this.planShown = store.plan; this.map?.showPlan(store.plan); }
    const points = this.map?.showDrones(drones.map((d, index) => { const r = rescueOf(d) || {};
      return { id: d.drone_id, label: d.label, index, latlon: r.gps ? [r.gps.latitude, r.gps.longitude] : null, yawDeg: (r.vehicle?.yaw || 0) * 180 / Math.PI,
        alt: r.vehicle?.position?.[2], selected: d.drone_id === this.selected, stale: !isFresh(d) }; }), id => this.select(id)) || [];
    this.map?.showArena(store.mode === 'sim' ? sim.arena : null);
    this.map?.showGeotags(drones.flatMap(d => (rescueOf(d)?.geotags || []).map(t => ({ ...t, drone: d.label }))));
    this.points = points;
    if (flying) this.map?.follow(points);
    else if (!this.centered && points.length && !gcsPosition.fix) { this.centered = true; this.map?.follow(points); }
    // drawer: selected aircraft
    const d = store.drone(this.selected), data = d.data || {}, r = rescueOf(d) || {}, vis = linksOf(d).vision || {}, fc = linksOf(d).fc || {}, exec = data.execution || {};
    this.drawerTitle.textContent = d.label;
    this.treeBadge.className = `badge ${data.active ? 'warn' : exec.state === 'SUCCESS' ? 'ok' : exec.state ? 'bad' : ''}`;
    this.treeBadge.textContent = (data.bt_nodes || []).length ? (data.active ? `RUNNING · ${data.bt_running_count || 0} nodes` : exec.state || 'idle') : 'no tree run yet';
    if (!this.drawer.classList.contains('closed') && this.tab === 'bt') fill(this.bodyBt, [btLogView(d.drone_id, { limit: 150 })]);
    if (!this.drawer.classList.contains('closed') && this.tab === 'links') {
      const el = archPanel(d, this.archPanels.get(d.drone_id)); this.archPanels.set(d.drone_id, el); fill(this.bodyLinks, [el]);
    }
    if (!this.drawer.classList.contains('closed') && this.tab === 'tree') {
      const tree = fromLive(data.bt_nodes), key = `${d.drone_id}:${exec.mission_id || ''}`;
      const sig = (data.bt_nodes || []).map(n => n.status[0] + (n.display_status || ' ')[0]).join('');
      if (key !== this.treeKey || sig !== this.treeSig) { this.viewer.update(tree, { keepView: key === this.treeKey }); this.treeKey = key; this.treeSig = sig; }
      const g = gnss(d), op = r.active_operation;
      fill(this.detail, [
        h('div', { class: 'kicker' }, 'Now'), h('div', { style: { fontWeight: 600, margin: '2px 0 8px' } }, op ? (OPS[op] || op) : data.active ? 'Between operations' : 'Not flying'),
        h('div', { style: { display: 'grid', gridTemplateColumns: '120px 1fr', gap: '10px', alignItems: 'start' } }, gimbalGauge(vis.gimbal_pitch, vis.gimbal_yaw),
          h('dl', { class: 'kv' }, h('dt', {}, 'Camera'), h('dd', {}, vis.fresh ? `${vis.source || ''} ${Math.round(vis.fps || 0)} fps` : 'offline'),
            h('dt', {}, 'Lock'), h('dd', {}, vis.locked ? vis.label : vis.lock_state || '—'),
            h('dt', {}, 'GNSS'), h('dd', {}, g.text), h('dt', {}, 'Mode'), h('dd', {}, fc.status ? `${fc.status.nav} · ${fc.status.arming}` : '—'))),
        r.flags?.AbortRequested ? hint(`Abort: ${r.flags.reason || 'latched'} — returning to launch and landing.`, 'bad') : null,
        h('div', { class: 'kicker', style: { marginTop: '8px' } }, `Geotags (${(r.geotags || []).length})`),
        ...(r.geotags || []).slice(-5).reverse().map(t => h('div', { class: 'mono', style: { fontSize: '11px' } }, `${t.label}  ${t.latitude.toFixed(7)}, ${t.longitude.toFixed(7)}  ±${t.sigma_m} m`)),
        h('div', { class: 'kicker', style: { marginTop: '8px' } }, 'PX4 messages'),
        ...((fc.messages || []).length ? fc.messages.slice(-4).reverse().map(m => h('div', { class: 'mono', style: { fontSize: '11px', color: m.severity <= 3 ? 'var(--bad)' : 'var(--warn)' } }, m.text))
          : [h('div', { class: 'faint', style: { fontSize: '11px' } }, fc.mavlink === 'ok' ? 'none' : `tap ${fc.mavlink || 'off'}`)]),
        h('div', { class: 'kicker', style: { marginTop: '8px' } }, 'Onboard log'),
        ...(data.logs || []).slice(-8).reverse().map(l => h('div', { class: 'mono', style: { fontSize: '11px', color: l.level === 'ERROR' ? 'var(--bad)' : l.level === 'WARN' ? 'var(--warn)' : '' } },
          `${new Date(l.time * 1000).toLocaleTimeString([], { hour12: false })} ${stripAnsi(l.message).slice(0, 140)}`)),
        legend()]);
    }
  },
};
