// Step 3 — Plan: mission type and its conditions -> KML area -> only that mission's options ->
// one tree per drone, assigned straight to Upload (the trees are reviewed there).
import { h, icon, card, badge, hint, api, store, prefs, toast, fill, isFresh, linksOf, radioFrames, confirmDialog } from './core.js';
import { createMap } from './mapview.js';
import { parseXml, countNodes, loadCatalog } from './tree.js';
import { gnss } from './dronecard.js';
import { catalog, isCustom, customById, emergencyAction, setEmergencyAction, deleteMission } from './missions.js';
import { openBuilder } from './builder.js';

// Option catalogue: [key, label, default, min, max]
const O = {
  altitude_m: ['altitude_m', 'Altitude above take-off (m)', 15, 1, 50],
  spacing_m: ['spacing_m', 'Lane spacing (m)', 15, 2, 200],
  speed_mps: ['speed_mps', 'Survey speed (m/s)', 1.5, 0.1, 2],
  transit_mps: ['speed_mps', 'Transit speed to station (m/s)', 1.5, 0.1, 2],
  heading_deg: ['heading_deg', 'Lane heading (°)', 0, -180, 180],
  corridor_m: ['corridor_m', 'Separation between drones (m)', 10, 0, 100],
  search_pitch_deg: ['search_pitch_deg', 'Gimbal search pitch (°)', -45, -90, 0],
  lost_timeout_s: ['lost_timeout_s', 'Person lost after (s)', 3, 0.5, 30],
  rtl_speed: ['rtl_speed', 'Return speed (m/s)', 2, 0.3, 3],
  track_duration_s: ['track_duration_s', 'Track for (s, 0 = until lost)', 0, 0, 1700],
  approach_speed: ['approach_speed', 'Approach speed (m/s)', 1, 0.2, 2],
  orbit_radius_m: ['orbit_radius_m', 'Orbit radius (m)', 10, 3, 100],
  orbit_laps: ['orbit_laps', 'Orbit laps', 2, 0.25, 20],
  orbit_speed: ['orbit_speed', 'Orbit speed (m/s)', 1, 0.2, 3],
  search_time_s: ['search_time_s', 'Watch window (s)', 600, 30, 1740],
  scan_rate_dps: ['scan_rate_dps', 'Yaw sweep rate (°/s)', 12, 2, 30],
};

const VISION_REQ = [
  ['cam', 'Camera / tracker streaming on every aircraft', d => linksOf(d).vision?.fresh, d => linksOf(d).vision?.fresh ? `${Math.round(linksOf(d).vision.fps || 0)} fps` : 'offline'],
  ['gimbal', 'Gimbal attitude feedback', d => linksOf(d).vision?.gimbal_pitch != null, d => linksOf(d).vision?.gimbal_pitch != null ? 'ok' : 'none'],
];
const GNSS_REQ = ['gnss', 'GNSS 3D fix, ≥ 6 satellites, HDOP ≤ 2.5', d => gnss(d).state === 'ok', d => gnss(d).text];

export const MISSIONS = [
  { id: 'human_track_rtl', n: 1, title: 'Track & target', ic: 'person', summary: 'Survey the area; lock and track the first person; return to launch when lost.',
    options: ['altitude_m', 'spacing_m', 'speed_mps', 'heading_deg', 'corridor_m', 'search_pitch_deg', 'lost_timeout_s', 'track_duration_s', 'rtl_speed'],
    requires: [...VISION_REQ],
    does: ['Flies survey lanes inside the KML area only.', 'Only a person standing inside the area counts as a detection.',
      'Holds position, faces the person and keeps the gimbal lock.', 'Returns to launch and lands when the person is lost for the set time (or the track time ends).'],
    limits: ['One person per flight; the tracker picks the person nearest the image centre.', 'Ground assumed flat at take-off height for range estimates.'] },
  { id: 'human_orbit', n: 2, title: 'Orbit person', ic: 'orbit', summary: 'No survey lanes. Hold a station inside the area and watch; orbit the person found.',
    options: ['altitude_m', 'transit_mps', 'search_time_s', 'scan_rate_dps', 'search_pitch_deg', 'orbit_radius_m', 'orbit_laps', 'orbit_speed', 'lost_timeout_s', 'rtl_speed'],
    requires: [...VISION_REQ],
    does: ['No waypoints are generated: each aircraft flies straight to one station in the middle of its share of the area.',
      'Holds the station and sweeps yaw slowly for the watch window, gimbal at the search pitch.',
      'A person detected inside the KML area interrupts the watch; people outside it are ignored.',
      'Follows the person and circles them at the set radius, camera kept on them, for the set laps.',
      'Then returns to launch and lands. Person lost → returns to launch.'],
    limits: ['Orbit radius 3–100 m; the circle may extend beyond the KML edge by up to the radius.', 'Orbit speed ≤ 3 m/s.',
      'Keep the radius larger than obstacles around the person (trees, poles).'] },
  { id: 'human_geotag_resume', n: 3, title: 'Geotag & resume', ic: 'pin', summary: 'Survey; fly overhead each person, geotag, release the lock, resume.',
    options: ['altitude_m', 'spacing_m', 'speed_mps', 'heading_deg', 'corridor_m', 'search_pitch_deg', 'lost_timeout_s', 'approach_speed', 'rtl_speed'],
    requires: [...VISION_REQ, GNSS_REQ],
    does: ['Flies survey lanes inside the KML area.', 'For each new person inside the area: approaches until the gimbal looks straight down, records latitude/longitude.',
      'Releases the lock, looks away, resumes the lane from the interruption point.', 'People within 6 m of an earlier geotag are not tagged twice.'],
    limits: ['Geotag accuracy depends on GNSS quality and flat ground (shown as ± metres).'] },
  { id: 'human_geotag_drop_resume', n: 4, title: 'Geotag + drop', ic: 'gift', summary: 'Mission 3 plus one payload release over the first person at nadir.',
    options: ['altitude_m', 'spacing_m', 'speed_mps', 'heading_deg', 'corridor_m', 'search_pitch_deg', 'lost_timeout_s', 'approach_speed', 'rtl_speed'],
    requires: [...VISION_REQ, GNSS_REQ,
      ['flight', 'Flight allowed and overhead drop envelope verified in the aircraft profile', d => !!d.data?.flight_enabled, d => d.data?.flight_enabled ? 'yes' : 'no']],
    does: ['Everything in mission 3.', 'Over the first geotagged person: releases the payload once, only when steady and inside the drop height window.',
      'A release is never repeated automatically.'],
    limits: ['Operator must confirm the package is loaded at upload (servo command does not prove delivery).', 'Drop height window comes from the aircraft profile.',
      'Releasing over a person is a deliberate choice: make sure the package is safe to drop.'] },
  { id: 'swarm_coverage', n: 5, title: 'Swarm coverage', ic: 'mesh', summary: 'Every aircraft flies its share of the area. No detection.',
    options: ['altitude_m', 'spacing_m', 'speed_mps', 'heading_deg', 'corridor_m', 'rtl_speed'],
    requires: [],
    does: ['Splits the area into one contiguous band of lanes per aircraft.', 'Flies the lanes, returns to launch, lands.'],
    limits: ['No camera use.'] },
];
const MCOLOR = ['#4f8df7', '#9b6cf6', '#ec5fa0', '#f0a12c', '#22b07d'];
const PAYLOAD_REQ = ['flight', 'Flight allowed and overhead drop envelope verified in the aircraft profile', d => !!d.data?.flight_enabled, d => d.data?.flight_enabled ? 'yes' : 'no'];

/** Parameters a custom mission uses, from its features (same catalogue as the built-ins). */
export function optionsFor(f) {
  const o = ['altitude_m'];
  if (f.type === 'watch') o.push('transit_mps', 'search_time_s', 'scan_rate_dps'); else o.push('spacing_m', 'speed_mps', 'heading_deg', 'corridor_m');
  if (f.detect) o.push('search_pitch_deg', 'lost_timeout_s');
  if (f.type === 'track') o.push('track_duration_s');
  if (f.geotag) o.push('approach_speed');
  if (f.orbit) o.push('orbit_radius_m', 'orbit_laps', 'orbit_speed');
  o.push('rtl_speed');
  return o;
}
export { O as OPTION_CATALOGUE };

/** Mission descriptor for a saved custom mission (same shape as MISSIONS entries). */
function customMission(rec) {
  const f = rec.features;
  return { id: rec.id, n: 'C', custom: true, record: rec, title: rec.name, ic: f.orbit ? 'orbit' : f.geotag ? 'pin' : f.type === 'track' ? 'person' : 'grid',
    summary: summaryOf(f), options: optionsFor(f), features: f,
    requires: [...(f.detect ? VISION_REQ : []), ...(f.geotag ? [GNSS_REQ] : []), ...(f.drop ? [PAYLOAD_REQ] : [])],
    does: rec._does || [summaryOf(f)],
    limits: [...(f.detect ? ['Only people inside the KML area count; the tracker picks the person nearest the image centre.'] : ['No camera use.']),
      ...(f.orbit ? ['Orbit radius 3–100 m; the circle may extend beyond the KML edge by up to the radius. Orbit and tracking are not separation-gated.'] : []),
      ...(f.geotag ? ['Geotag accuracy depends on GNSS quality and flat ground (shown as ± metres).'] : []),
      ...(f.drop ? ['Confirm the package is loaded at upload; the servo command does not prove delivery. A release is never repeated automatically.'] : []),
      ...(f.finish === 'LAND' ? ['Lands where the mission ends — make sure that place is safe to land (not water).'] : []),
      ...(rec.notes ? [`Notes: ${rec.notes}`] : [])] };
}
export function summaryOf(f) {
  const type = { coverage: 'Survey lanes', watch: 'Watch stations', track: 'Survey, then track a person' }[f.type];
  const acts = [f.geotag && 'geotag', f.drop && 'drop', f.orbit && 'orbit'].filter(Boolean);
  return `${type}${f.detect ? (acts.length ? ` · ${acts.join(' + ')}` : '') + (f.resume ? ' · resume' : ' · then end') : ' · no camera'} · finish ${f.finish}`;
}

const COMMON_REQ = [
  ['fresh', 'Aircraft status fresh over the link', d => isFresh(d), d => isFresh(d) ? 'fresh' : d.connection],
];

export const page = {
  id: 'plan', title: 'Plan', sub: 'Mission & area', ic: 'map',
  intro: 'Choose the mission, read and accept its conditions, load the KML area. One tree per aircraft is generated and assigned to Upload.',
  mount(root) {
    this.root = root; this.kmlText = null; this.kmlName = ''; this.inputs = {};
    this.list = h('div', { class: 'mission-list' });
    this.renderList();
    catalog.listeners.add(() => {
      this.renderList();
      const id = store.missionType, rec = customById(id);
      // A deleted mission falls back to the default; changed features need new options and a new plan.
      if (isCustom(id) && !rec) this.pick('human_geotag_resume', true);
      else if (isCustom(id) && this.shownFeatures !== JSON.stringify(rec.features)) this.pick(id, true);
      else this.renderTerms();
    });
    this.terms = h('div', { class: 'terms' });
    this.accept = h('input', { type: 'checkbox', onchange: () => this.updateButtons() });
    this.missionCard = card({ title: 'Mission', sub: 'built-in or your own · last choice is remembered', ic: 'grid', accent: 'violet',
      right: h('button', { class: 'btn sm primary', onclick: () => this.build({}) }, h('span', { html: icon('plus') }), 'New mission') }, this.list);
    this.termsCard = card({ title: 'Conditions', ic: 'shield', accent: 'amber', right: this.termsBadge = badge('—') }, this.terms);
    // area
    this.fileInput = h('input', { type: 'file', accept: '.kml', style: { display: 'none' }, onchange: e => this.loadFile(e.target.files[0]) });
    this.drop = h('div', { class: 'drop', onclick: () => this.fileInput.click() },
      h('b', {}, 'Load KML area — drop the file here or click'), h('small', {}, 'Polygon; holes and no-fly zones are respected'), this.fileInput);
    ['dragenter', 'dragover'].forEach(ev => this.drop.addEventListener(ev, e => { e.preventDefault(); this.drop.classList.add('over'); }));
    ['dragleave', 'drop'].forEach(ev => this.drop.addEventListener(ev, e => { e.preventDefault(); this.drop.classList.remove('over'); }));
    this.drop.addEventListener('drop', e => this.loadFile(e.dataTransfer.files[0]));
    this.mapBox = h('div', { class: 'map' });
    this.areaCard = card({ title: 'Search area', ic: 'map', accent: 'teal', right: this.areaBadge = badge('no KML') }, this.drop, this.mapBox);
    // options + generate
    this.optionBox = h('div', { class: 'fields' });
    this.fleetBox = h('div', { class: 'fields' });
    this.genBtn = h('button', { class: 'btn primary', onclick: () => this.generate() }, h('span', { html: icon('bolt') }), 'Generate & assign trees');
    this.toUpload = h('button', { class: 'btn', disabled: true, onclick: () => { location.hash = '#upload'; } }, 'Review in Upload', h('span', { html: icon('arrow') }));
    this.progress = h('div', { class: 'progress', style: { display: 'none' } }, h('span'));
    this.result = h('div');
    this.emBox = h('div', { class: 'seg' });
    this.saveVals = h('button', { class: 'btn sm', style: { display: 'none' }, onclick: () => this.saveValues() }, 'Save values to this mission');
    this.optCard = card({ title: 'Parameters', sub: 'only what this mission uses', ic: 'bolt', accent: 'pink', right: this.saveVals }, this.optionBox,
      h('div', { class: 'row', style: { marginTop: '12px' } }, h('span', { class: 'kicker' }, 'On emergency'), this.emBox,
        h('span', { class: 'faint', style: { fontSize: '11.5px' } }, 'recovery branch of every tree + default in the Emergency view')),
      h('div', { class: 'kicker', style: { marginTop: '12px' } }, 'Measured endurance per aircraft (min)'), this.fleetBox,
      h('div', { class: 'row', style: { marginTop: '12px' } }, this.genBtn, this.toUpload, this.genNote = h('span', { class: 'faint', style: { fontSize: '11.5px' } })), this.progress, this.result);
    root.append(h('div', { class: 'cols-plan' },
      h('div', { class: 'grid', style: { alignContent: 'start' } }, this.missionCard, this.termsCard),
      h('div', { class: 'grid', style: { alignContent: 'start' } }, this.areaCard, this.optCard)));
    this.map = createMap(this.mapBox);
    loadCatalog();
    this.pick(MISSIONS.some(m => m.id === store.missionType) || isCustom(store.missionType) ? store.missionType : 'human_geotag_resume', true);
  },
  shown() { this.map?.resize(); this.renderFleet(); },
  mission() {
    if (isCustom(store.missionType)) {
      const rec = customById(store.missionType);
      if (rec) return customMission(rec);
      // Saved missions are still loading (or this one was deleted): show it as a placeholder.
      return { id: store.missionType, n: 'C', custom: true, title: 'Saved mission (loading…)', options: ['altitude_m'], requires: [], does: [], limits: [], features: null };
    }
    return MISSIONS.find(m => m.id === store.missionType) || MISSIONS[2];
  },
  renderList() {
    const id = store.missionType;
    const row = (m, color, acts) => h('div', { class: `mrow ${m.id === id ? 'selected' : ''}`, 'data-mission': m.id, style: { '--m': color } },
      h('button', { class: 'mpick', onclick: () => this.pick(m.id) }, h('span', { class: 'n' }, m.n), h('span', {}, h('b', {}, m.title), h('small', {}, m.summary))),
      h('div', { class: 'acts' }, ...acts));
    const act = (ic, title, fn, cls = '') => h('button', { class: `icon-btn sm ${cls}`, title, 'aria-label': title, html: icon(ic), onclick: e => { e.stopPropagation(); fn(); } });
    const custom = catalog.custom;
    fill(this.list, [
      h('div', { class: 'kicker' }, 'Built-in · always available, never deleted'),
      ...MISSIONS.map((m, i) => row(m, MCOLOR[i], [act('file', 'Edit a copy', () => this.build({ copyOf: m }))])),
      h('div', { class: 'kicker', style: { marginTop: '8px' } }, `My missions · saved on this GCS (${custom.length})`),
      ...(custom.length ? custom.map((rec, i) => row(customMission(rec), ['#22b5c9', '#5b6cf0', '#f2556b', '#3fae5a'][i % 4], [
        act('code', 'Edit', () => this.build({ record: rec })),
        act('file', 'Edit a copy', () => this.build({ record: rec, copy: true })),
        act('stop', 'Delete', () => this.remove(rec), 'del')]))
        : [h('p', { class: 'faint', style: { margin: '2px 4px', fontSize: '12px' } }, catalog.error ? `Saved missions unavailable: ${catalog.error}` : 'None yet — press "New mission" or edit a copy of a built-in one.')])]);
  },
  build(arg) {
    const values = {};
    for (const input of Object.values(this.inputs)) if (input.dataset.key) values[input.dataset.key] = +input.value;
    openBuilder({ ...arg, values, fleetSize: store.fleet.length || 1, onSaved: rec => this.pick(rec.id) });
  },
  async remove(rec) {
    if (!(await confirmDialog(`Delete "${rec.name}"?`, 'The saved mission is removed from this GCS. Built-in missions and trees already uploaded to aircraft are not affected.', { ok: 'Delete', kind: 'danger' }))) return;
    try { await deleteMission(rec.id); toast('Mission deleted', rec.name, 'ok'); } catch (e) { toast('Delete failed', e.message, 'bad'); }
  },
  async saveValues() {
    const m = this.mission(); if (!m.custom || !m.record) return;
    const options = { ...m.record.options };
    for (const input of this.optionBox.querySelectorAll('input')) options[input.dataset.key] = +input.value;
    try {
      const { saveMission } = await import('./missions.js');
      await saveMission({ id: m.record.id, name: m.record.name, features: { ...m.record.features, emergency: emergencyAction() }, options, notes: m.record.notes, based_on: m.record.based_on });
      toast('Values saved', m.record.name, 'ok');
    } catch (e) { toast('Save failed', e.message, 'bad'); }
  },
  renderEmergency() {
    const cur = emergencyAction();
    fill(this.emBox, ['RTL', 'LAND'].map(a => h('button', { class: cur === a ? 'active' : '', onclick: () => { setEmergencyAction(a); this.renderEmergency(); this.renderTerms(); } },
      a === 'RTL' ? 'Return to launch' : 'Land in place')));
  },
  pick(id, silent) {
    const changed = store.missionType !== id;
    store.missionType = id; prefs.set('mission', id);
    this.list.querySelectorAll('.mrow').forEach(b => b.classList.toggle('selected', b.dataset.mission === id));
    if (changed || silent) this.accept.checked = false;
    const m = this.mission();
    if (m.custom && m.record && changed) {
      // A saved mission brings its own parameter values.
      for (const [key, value] of Object.entries(m.record.options || {})) {
        prefs.set('opt.' + key, value);
        for (const input of Object.values(this.inputs)) if (input.dataset.key === key) input.value = value;
      }
    }
    if (m.custom && m.record && !m.record._does) import('./missions.js').then(x => x.describeMission(m.record.features, store.fleet.length || 1))
      .then(r => { m.record._does = r.does; if (store.missionType === m.id) this.renderTerms(); }).catch(() => {});
    this.saveVals.style.display = m.custom && m.record ? '' : 'none';
    this.renderEmergency();
    this.shownFeatures = JSON.stringify(m.features || null);
    fill(this.optionBox, m.options.map(name => {
      const [key, label, def, min, max] = O[name];
      const input = this.inputs[key + ':' + name] ||= h('input', { type: 'number', step: 'any', min, max, value: prefs.get('opt.' + key, def),
        onchange: () => { prefs.set('opt.' + key, +input.value); } });
      input.dataset.key = key;
      return h('label', { class: 'field' }, label, input, h('small', {}, `${min} … ${max}`));
    }));
    this.renderTerms();
    store.plan = null; this.map?.showPlan(null); fill(this.result, []); this.toUpload.disabled = true; this.done = false;
    if (!silent && changed) toast(m.custom ? `My mission: ${m.title}` : `Mission ${m.n}: ${m.title}`, 'Read and accept its conditions before generating.');
  },
  requirements() {
    const m = this.mission(), drones = store.fleet;
    const rows = [['KML search area loaded', !!this.kmlText, this.kmlText ? this.kmlName : 'none', true],
      ['At least one aircraft registered', drones.length > 0, `${drones.length}`, true]];
    for (const [, label, test, value] of [...COMMON_REQ, ...m.requires]) {
      const pass = drones.filter(test);
      rows.push([label, drones.length > 0 && pass.length === drones.length, drones.length ? drones.map(d => `${d.drone_id}: ${value(d)}`).join(' · ') : '—', false]);
    }
    if (drones.length > 1) {
      const meshOk = drones.every(d => !linksOf(d).swarm?.required || linksOf(d).swarm?.peers_fresh);
      rows.push(['Mesh link between all aircraft (fleet > 1)', meshOk, meshOk ? 'peers fresh' : 'peer missing', false]);
      const close = drones.filter(d => linksOf(d).swarm?.required && !linksOf(d).swarm?.separation_safe).map(d => d.drone_id);
      rows.push(['Aircraft ≥ 15 m apart (10 m + GNSS uncertainty)', !close.length, close.length ? `too close: ${close.join(', ')}` : 'separated', false]);
    }
    return rows;
  },
  renderTerms() {
    const m = this.mission(), rows = this.requirements();
    if (m.custom && m.record?._does) m.does = m.record._does;
    const blocking = rows.filter(r => r[3] && !r[1]).length, pending = rows.filter(r => !r[3] && !r[1]).length;
    this.termsBadge.className = `badge ${blocking ? 'bad' : pending ? 'warn' : 'ok'}`;
    this.termsBadge.textContent = blocking ? `${blocking} missing` : pending ? `${pending} not met yet` : 'all met';
    fill(this.terms, [
      h('h4', {}, 'Requirements'), h('div', { style: { padding: '2px 0 6px' } }, ...rows.map(([label, ok, value, hard]) =>
        h('div', { class: `req ${ok ? 'ok' : hard ? 'bad' : 'warn'}`, title: hard ? 'required to plan' : 'required before Start (checked again onboard)' },
          h('span', { class: 'm' }), h('span', {}, label), h('span', { class: 'v' }, value)))),
      h('h4', {}, 'What the aircraft will do'), h('ul', {}, ...m.does.map(t => h('li', {}, t))),
      h('h4', {}, 'Limits and operator duties'), h('ul', {}, ...m.limits.map(t => h('li', {}, t)),
        h('li', {}, 'A geofence and RC/manual override must be configured on the flight controller.'),
        store.fleet.length > 1 ? h('li', {}, 'Fleet: launch points at least 15 m apart. A swarm aircraft stops its search and returns if another comes closer than 10 m plus GNSS uncertainty.') : null,
        h('li', {}, `Low battery, lost position or an unhealthy vehicle aborts the mission: ${emergencyAction() === 'LAND' ? 'land in place (operator setting)' : 'return to launch and land'}.`)),
      h('label', { class: 'accept' }, this.accept, h('span', {}, `I have read the conditions of ${m.custom ? `"${m.title}"` : `mission ${m.n}`} and the area is clear for flight.`))]);
    this.updateButtons();
  },
  updateButtons() {
    const hard = this.requirements().filter(r => r[3] && !r[1]);
    this.genBtn.disabled = !this.accept.checked || hard.length > 0;
    this.genNote.textContent = hard.length ? hard.map(r => r[0]).join('; ') : !this.accept.checked ? 'Accept the conditions first' : '';
  },
  renderFleet() {
    const drones = store.fleet;
    fill(this.fleetBox, drones.length ? drones.map(d => {
      const key = 'endurance.' + d.drone_id;
      const input = this.inputs[key] ||= h('input', { type: 'number', min: 1, step: 'any', value: prefs.get(key, 20), onchange: () => prefs.set(key, +input.value) });
      return h('label', { class: 'field' }, d.label, input);
    }) : [h('p', { class: 'muted', style: { margin: 0 } }, 'Add aircraft in step 1 — one tree is made per registered aircraft.')]);
  },
  async loadFile(file) {
    if (!file) return;
    if (file.size > 1e6) return toast('KML too large', 'Choose a file up to 1 MB.', 'bad');
    this.kmlText = await file.text(); this.kmlName = file.name;
    this.areaBadge.className = 'badge info'; this.areaBadge.textContent = `${file.name} · ${(file.size / 1024).toFixed(1)} KB`;
    this.map?.resetView(); this.renderTerms();
    // Preview the area right away (no assignment until the conditions are accepted).
    this.generate(true);
  },
  async generate(preview) {
    if (!this.kmlText) return toast('Load a KML area first', '', 'warn');
    this.renderFleet();
    // Plan only for aircraft that answer right now: an offline aircraft in the plan would make
    // every other tree wait for a mesh partner that never comes.
    const drones = store.fleet.filter(isFresh), offline = store.fleet.filter(d => !isFresh(d));
    if (!drones.length) return toast('No aircraft answering', 'Connect at least one aircraft (step 1) before planning.', 'warn');
    if (offline.length && !preview) toast(`Planning without ${offline.map(d => d.label).join(', ')}`, 'Not answering right now, so left out of this plan. Re-plan once they are connected.', 'warn', 10000);
    const m = this.mission();
    if (m.custom && !m.features) return toast('Saved missions not loaded yet', 'Wait a moment and try again.', 'warn');
    const request = { mission_id: crypto.randomUUID().replaceAll('-', ''), template: m.custom ? 'custom' : m.id, kml: this.kmlText, payload_confirmation: 'operator',
      emergency_action: emergencyAction(), ...(m.custom ? { features: m.features } : {}),
      fleet: drones.map(d => ({ drone_id: d.drone_id, endurance_s: (+this.inputs['endurance.' + d.drone_id]?.value || 20) * 60, reserve_fraction: .25, coverage: true })) };
    for (const input of this.optionBox.querySelectorAll('input')) {
      const v = +input.value, min = +input.min, max = +input.max;
      if (!isFinite(v) || v < min || v > max) return toast('Parameter out of range', `${input.closest('label').firstChild.textContent}: ${min} … ${max}`, 'bad');
      request[input.dataset.key] = v;
    }
    const drops = m.id === 'human_geotag_drop_resume' || !!m.features?.drop;
    request.drop_mode = drops ? 'overhead' : 'beside'; request.overhead_authorized = drops;
    this.genBtn.classList.add('busy'); this.progress.style.display = ''; this.progress.classList.add('indeterminate');
    try {
      const plan = await api('/api/plan/kml', request);
      store.plan = plan; this.map?.showPlan(plan);
      if (preview) { fill(this.result, [h('div', { class: 'faint', style: { marginTop: '10px', fontSize: '12px' } }, 'Area preview only. Accept the conditions and press Generate to assign trees.')]); return; }
      if (!plan.complete) { fill(this.result, [hint(plan.reason, 'warn')]); toast('Plan needs attention', plan.reason, 'warn', 9000); return; }
      for (const b of plan.bundles) store.assigned.set(b.drone_id, { xml: plan.xml[b.drone_id], bundle: b, label: `${m.custom ? '' : 'M' + m.n + ' '}${m.title} · ${this.kmlName}`, source: 'plan', template: plan.template, emergency_action: plan.emergency_action || emergencyAction() });
      store.saveAssigned();
      const rows = await Promise.all(plan.bundles.map(async b => {
        const xml = plan.xml[b.drone_id], frames = await radioFrames({ xml, route_bundle: b });
        let nodes = 0; try { nodes = countNodes(parseXml(xml)); } catch { /* shown in Upload */ }
        const pts = b.coverage.waypoints.length;
        return h('tr', {}, h('td', {}, h('b', {}, b.drone_id)), h('td', { class: 'mono' }, plan.stations ? 'station' : `${pts} wp`),
          h('td', { class: 'mono' }, `${nodes}`), h('td', { class: 'mono' }, `${(xml.length / 1024).toFixed(1)} KB`),
          h('td', { class: 'mono' }, `${frames} frames`), h('td', { class: 'mono' }, `~${Math.round(b.assignment.estimated_duration_s / 60)} min`));
      }));
      fill(this.result, [h('table', { class: 'table', style: { marginTop: '10px' } },
        h('thead', {}, h('tr', {}, ...['Aircraft', 'Route', 'Nodes', 'XML', 'Radio upload', 'Flight'].map(x => h('th', {}, x)))), h('tbody', {}, ...rows)),
        h('div', { class: 'faint', style: { fontSize: '11.5px', marginTop: '6px' } }, `Common origin ${plan.origin.latitude.toFixed(7)}, ${plan.origin.longitude.toFixed(7)} — applied onboard automatically.`)]);
      this.toUpload.disabled = false; this.done = true;
      toast('Trees assigned', `${plan.bundles.length} aircraft. Review and upload them in step 4.`, 'ok');
    } catch (e) { toast('Planning failed', e.message, 'bad', 9000); }
    finally { this.genBtn.classList.remove('busy'); this.progress.style.display = 'none'; }
  },
  leaveWarnings(s) {
    const out = [];
    if (!this.kmlText) out.push('No KML search area loaded.');
    if (!this.accept.checked) out.push(`Conditions of ${this.mission().custom ? `"${this.mission().title}"` : `mission ${this.mission().n}`} not accepted.`);
    if (!this.done) out.push('Trees have not been generated and assigned for this mission.');
    // Requirement rows beyond the two already covered above (KML, aircraft count).
    for (const r of this.requirements().slice(2).filter(r => !r[1])) out.push(`${r[0]}: ${r[2]}`);
    return out;
  },
  update(s) {
    if (this.fleetCount !== s.fleet.length) { this.fleetCount = s.fleet.length; this.renderFleet(); }
    const now = Date.now(); if (now - (this.termsAt || 0) > 2000) { this.termsAt = now; this.renderTerms(); }
  },
};
