// Step 2 — Swarm start: the go / no-go page. Fleet verdict ("Whole swarm is ready to fly"), the
// checks the operator selected, and per aircraft a pass/fail list grouped by link with the reason
// for every failure (preflight.js). The compact status cards of the earlier Readiness page stay
// available under "Status cards". Camera/payload checks follow the mission chosen in Plan.
import { h, icon, api, store, toast, emptyState, fill, isFresh, linksOf, prefs, card, badge } from './core.js';
import { droneCard, gnss } from './dronecard.js';
import { linkChecks, fleetStatus, droneReady, errorsOf, blockers, OPTIONAL, selected, setSelected } from './preflight.js';
import { needs, missionName, catalog } from './missions.js';
import { sysId } from './links.js';

export { checks } from './dronecard.js';
export { droneReady as readyToFly } from './preflight.js';

export const page = {
  id: 'ready', title: 'Swarm start', sub: 'Pre-flight · ready to fly', ic: 'shield',
  intro: 'Every aircraft and every link is checked here. When all selected checks pass and nothing is in error, the whole swarm is ready to fly.',
  mount(root) {
    this.root = root; this.cards = new Map(); this.lists = new Map();
    this.view = prefs.get('startView', 'checks');
    this.hero = h('section', { class: 'go-hero' });
    this.strip = h('div', { class: 'strip' });
    this.tabs = h('div', { class: 'tabs', style: { margin: 0 } }, ...[['checks', 'Checklist by link'], ['cards', 'Status cards']].map(([id, t]) =>
      h('button', { 'data-view': id, class: id === this.view ? 'active' : '', onclick: () => this.setView(id) }, t)));
    this.selBox = h('div', { class: 'sel-checks' });
    this.selCard = card({ title: 'Checks counted', sub: 'mandatory checks are always counted', ic: 'check', accent: 'indigo' }, this.selBox);
    this.grid = h('div', { class: 'go-grid' });
    this.cardGrid = h('div', { class: 'dc-grid' });
    root.append(this.hero, this.strip, h('div', { class: 'row', style: { margin: '14px 0 10px' } }, this.tabs, h('span', { style: { flex: 1 } }), this.missionChip = h('div', { class: 'row' })),
      h('div', { class: 'go-layout' }, h('div', {}, this.grid, this.cardGrid), this.selCard));
    this.renderSelection();
    this.setView(this.view);
    catalog.listeners.add(() => this.update(store));
  },
  setView(id) {
    this.view = id; prefs.set('startView', id);
    this.tabs.querySelectorAll('button').forEach(b => b.classList.toggle('active', b.dataset.view === id));
    this.grid.style.display = id === 'checks' ? '' : 'none';
    this.cardGrid.style.display = id === 'cards' ? '' : 'none';
    this.update(store);
  },
  renderSelection() {
    const mandatory = ['GCS radio link and aircraft heartbeat', 'Onboard manager answers, no abort/lockout', 'AERIXIS core nodes, vehicle healthy',
      'XRCE-DDS data, PX4 arming checks, sensors, no failsafe, battery ≥ 30 %', 'GNSS 3D fix, valid position', 'Fleet: mesh peers fresh, separation',
      'Camera stream — only if the mission uses the camera', 'Payload servo — only if the mission drops'];
    fill(this.selBox, [
      ...Object.entries(OPTIONAL).map(([key, [label]]) => h('label', { class: 'check' },
        h('input', { type: 'checkbox', checked: selected(key), onchange: e => { setSelected(key, e.target.checked); this.update(store); } }), label)),
      h('div', { class: 'kicker', style: { marginTop: '10px' } }, 'Always counted'),
      ...mandatory.map(t => h('div', { class: 'locked', html: icon('shield') + `<span>${t}</span>` }))]);
  },
  async reconcile(d, button) {
    button.classList.add('busy');
    try { await api('/api/fleet/command', { drone_id: d.drone_id, action: 'reconcile' }); toast(`${d.label}: ground state verified`, 'Lockout cleared; ready for a new mission.', 'ok'); }
    catch (e) { toast(`${d.label}: reconcile refused`, e.message, 'bad'); } finally { button.classList.remove('busy'); }
  },
  heroView(st) {
    const n = needs();
    const head = st.ok
      ? [h('div', { class: 'go-ic ok', html: icon('check') }), h('div', {}, h('h2', {}, 'Whole swarm is ready to fly'),
          h('p', {}, `${st.ready}/${st.total} aircraft ready · every selected check passes · no errors`))]
      : [h('div', { class: 'go-ic bad', html: icon('warn') }), h('div', {}, h('h2', {}, st.total && st.ready ? `${st.ready}/${st.total} aircraft ready — swarm not ready yet` : 'Not ready to fly'),
          h('p', {}, `${st.blockers.length} item(s) to fix before Start:`))];
    return [h('div', { class: 'go-head' }, ...head, h('span', { style: { flex: 1 } }),
      st.ok ? h('button', { class: 'btn go', onclick: () => { location.hash = '#upload'; } }, h('span', { html: icon('play') }), 'Upload & start')
        : h('button', { class: 'btn', onclick: () => { location.hash = '#connect'; } }, h('span', { html: icon('link') }), 'Connect')),
      st.ok ? null : h('ul', { class: 'go-blockers' }, ...st.blockers.slice(0, 8).map(b => h('li', {}, b)), st.blockers.length > 8 ? h('li', { class: 'faint' }, `+${st.blockers.length - 8} more`) : null),
      h('div', { class: 'row go-chips' }, badge(`Camera ${n.camera ? 'required' : 'not used'}`, n.camera ? 'info' : ''),
        badge(`Payload ${n.payload ? 'drop' : 'none'}`, n.payload ? 'info' : ''), badge(`Emergency → ${n.emergency}`, 'warn'),
        badge(store.fleet.length > 1 ? `Fleet of ${store.fleet.length}: mesh required` : 'Single aircraft'))];
  },
  checklist(d) {
    let el = this.lists.get(d.drone_id);
    if (!el) {
      el = h('section', { class: 'card go-card' });
      el._head = h('div', { class: 'card-head' }); el._body = h('div', { class: 'go-groups' }); el._foot = h('div');
      el.append(el._head, el._body, el._foot);
      this.lists.set(d.drone_id, el);
    }
    const groups = linkChecks(d), errors = errorsOf(d), ready = droneReady(d), b = blockers(d), active = !!d.data?.active;
    el.classList.toggle('ready', ready); el.classList.toggle('blocked', !ready && !active);
    fill(el._head, [h('div', { class: 'card-icon', html: icon('drone') }),
      h('div', {}, h('h3', {}, d.label), h('p', {}, `${d.drone_id} · system ${sysId(d) ?? '—'} · ${gnss(d).text}`)),
      h('div', { class: 'right' }, badge(active ? 'FLYING' : ready ? 'READY TO FLY' : `${b.length + errors.length} TO FIX`, active ? 'warn' : ready ? 'ok' : 'bad'))]);
    fill(el._body, groups.map(g => {
      const counted = g.items.filter(i => i.need), bad = counted.filter(i => i.state !== 'ok').length, na = g.items.every(i => i.state === 'na');
      return h('div', { class: `go-group c-${g.color} ${na ? 'na' : bad ? 'bad' : 'ok'}` },
        h('div', { class: 'go-gh' }, h('span', { class: 'gi', html: icon(g.ic) }), h('b', {}, g.title),
          h('span', { class: 'gs' }, na ? 'n/a' : bad ? `${bad} failing` : 'pass')),
        ...g.items.map(i => h('div', { class: `go-item ${i.state} ${i.need ? '' : 'info'}`, title: i.need ? '' : i.state === 'na' ? 'not applicable' : 'not counted (optional, not selected)' },
          h('span', { class: 'm' }), h('span', { class: 'k' }, i.label, i.optional ? h('i', {}, i.need ? ' · selected' : ' · optional') : null), h('span', { class: 'v' }, i.value),
          i.why ? h('div', { class: 'why' }, i.why) : null)));
    }));
    const lock = (d.data?.hold_lockout && !active) || d.data?.ros?.rescue?.data?.flags?.AbortRequested;
    const key = `${!!lock}:${active}:${errors.join('|')}`;
    if (el._footKey !== key) {
      el._footKey = key;
      fill(el._foot, [errors.length ? h('div', { class: 'faults', style: { marginTop: '10px' } }, ...errors.map(e => h('div', {}, e))) : null,
        lock ? h('div', { class: 'row', style: { marginTop: '10px' } },
          h('button', { class: 'btn sm', disabled: active, onclick: e => this.reconcile(store.drone(d.drone_id), e.currentTarget), html: icon('reconcile') + 'Reconcile on ground' }),
          h('span', { class: 'faint', style: { fontSize: '11px' } }, 'aircraft disarmed and still')) : null]);
    }
    return el;
  },
  update(s) {
    if (!this.hero) return;
    const drones = s.fleet, st = fleetStatus(s);
    // Rebuilt only when the content changes, so a click is never lost to the 1 s refresh.
    const heroKey = JSON.stringify([st, needs(), missionName(), store.fleet.length]);
    if (heroKey !== this.heroKey) {
      this.heroKey = heroKey;
      this.hero.className = `go-hero ${st.ok ? 'ok' : 'bad'}`;
      fill(this.hero, this.heroView(st));
      fill(this.missionChip, [h('span', { class: 'faint', style: { fontSize: '12px' } }, 'Mission'), h('b', { style: { fontSize: '12.5px' } }, missionName()),
        h('button', { class: 'btn sm ghost', onclick: () => { location.hash = '#plan'; } }, 'Change in Plan')]);
    }
    const cell = (label, value, cls = '') => h('div', {}, h('small', {}, label), h('b', { class: cls }, value));
    const gps = drones.map(gnss);
    fill(this.strip, [cell('Aircraft', drones.length), cell('Ready to fly', `${st.ready}/${drones.length}`, drones.length && st.ok ? 'ok' : drones.length ? 'warn' : ''),
      cell('Status fresh', `${drones.filter(isFresh).length}/${drones.length}`),
      cell('GNSS 3D fix', `${gps.filter(g => g.state !== 'bad').length}/${drones.length}`),
      cell('Cameras', needs().camera ? `${drones.filter(d => linksOf(d).vision?.fresh).length}/${drones.length}` : 'not used'),
      cell('Mesh', drones.some(d => linksOf(d).swarm?.required) ? `${drones.filter(d => linksOf(d).swarm?.peers_fresh).length} linked` : 'single')]);
    if (!drones.length) {
      fill(this.grid, [this.empty ||= emptyState('drone', 'No aircraft', 'Add aircraft in step 1 — Connect.')]);
      fill(this.cardGrid, [this.empty2 ||= emptyState('drone', 'No aircraft', 'Add aircraft in step 1 — Connect.')]);
      this.done = false; return;
    }
    if (this.view === 'checks') fill(this.grid, drones.map(d => this.checklist(d)));
    else fill(this.cardGrid, drones.map(d => {
      const lockout = (d.data?.hold_lockout && !d.data?.active) || d.data?.ros?.rescue?.data?.flags?.AbortRequested;
      const id = d.drone_id;
      const actions = lockout ? { key: `lock:${!!d.data?.active}`, build: () => h('div', { class: 'row' },
        h('button', { class: 'btn sm', disabled: !!d.data?.active, onclick: e => this.reconcile(store.drone(id), e.currentTarget), html: icon('reconcile') + 'Reconcile on ground' }),
        h('span', { class: 'faint', style: { fontSize: '11px' } }, 'aircraft disarmed and still')) } : null;
      const el = droneCard(d, { el: this.cards.get(d.drone_id), logLines: 2, actions });
      this.cards.set(d.drone_id, el);
      return el;
    }));
    this.done = st.ok;
  },
  leaveWarnings(s) {
    return s.fleet.filter(d => !droneReady(d) && !d.data?.active).map(d => `${d.label}: ${[...blockers(d).map(c => `${c.label} (${c.value})`), ...errorsOf(d)].join(', ')}`);
  },
};
