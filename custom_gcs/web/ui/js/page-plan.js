// Step 3 — Plan: mission type and its conditions -> KML area -> only that mission's options ->
// one tree per drone, assigned straight to Upload (the trees are reviewed there).
import { sim, arenaKml } from './simarena.js';
import { h, icon, card, badge, hint, api, store, prefs, toast, fill, isFresh, linksOf, radioFrames, confirmDialog } from './core.js';
import { createMap } from './mapview.js';
import { createAreaDrawer, kmlFromPoints, polygonStats, selfIntersects, fmtArea, fmtLen } from './areadraw.js';
import { parseXml, countNodes, loadCatalog } from './tree.js';
import { gnss } from './dronecard.js';
import { catalog, isCustom, customById, emergencyAction, setEmergencyAction, deleteMission } from './missions.js';
import { openBuilder } from './builder.js';

// Option catalogue: [key, label, default, min, max, remembered-as (default: key)]
const O = {
  altitude_m: ['altitude_m', 'Altitude above take-off (m)', 15, 1, 50],
  spacing_m: ['spacing_m', 'Lane spacing (m)', 15, 2, 200],
  speed_mps: ['speed_mps', 'Survey speed (m/s)', 1.5, 0.1, 2],
  transit_mps: ['speed_mps', 'Transit speed to station (m/s)', 1.5, 0.1, 2],
  heading_deg: ['heading_deg', 'Lane heading (°)', 0, -180, 180],
  corridor_m: ['corridor_m', 'Separation between drones (m)', 10, 0, 100],
  search_pitch_deg: ['search_pitch_deg', 'Gimbal search pitch (°)', -45, -90, 0],
  // Geotag missions search straight down, the camera swinging to both sides of the lane
  // (2026-10-31); remembered apart from the forward-looking pitch of track/orbit missions.
  search_pitch_nadir: ['search_pitch_deg', 'Gimbal search pitch (°, -90 = straight down, sweeping sideways)', -90, -90, 0, 'search_pitch_nadir'],
  lost_timeout_s: ['lost_timeout_s', 'Person lost after (s)', 3, 0.5, 30],
  rtl_speed: ['rtl_speed', 'Return speed (m/s)', 2, 0.3, 3],
  track_duration_s: ['track_duration_s', 'Track for (s, 0 = until lost)', 0, 0, 1700],
  approach_speed: ['approach_speed', 'Approach speed (m/s)', 1, 0.2, 2],
  delivery_wait_s: ['delivery_wait_s', 'Delivery drones wait for a geotag up to (s)', 1500, 10, 1800],
  delivery_speed: ['delivery_speed', 'Delivery flight speed (m/s)', 1.5, 0.3, 3],
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
    options: ['altitude_m', 'spacing_m', 'speed_mps', 'heading_deg', 'corridor_m', 'search_pitch_nadir', 'lost_timeout_s', 'approach_speed', 'rtl_speed'],
    requires: [...VISION_REQ, GNSS_REQ],
    does: ['Flies survey lanes inside the KML area.', 'For each new person inside the area: approaches until the gimbal looks straight down, records latitude/longitude.',
      'Releases the lock, looks away, resumes the lane from the interruption point.', 'People within 6 m of an earlier geotag are not tagged twice.'],
    limits: ['Geotag accuracy depends on GNSS quality and flat ground (shown as ± metres).'] },
  { id: 'human_geotag_drop_resume', n: 4, title: 'Geotag + drop', ic: 'gift', summary: 'Mission 3 plus one payload release over the first person at nadir.',
    options: ['altitude_m', 'spacing_m', 'speed_mps', 'heading_deg', 'corridor_m', 'search_pitch_nadir', 'lost_timeout_s', 'approach_speed', 'rtl_speed'],
    requires: [...VISION_REQ, GNSS_REQ,
      ['flight', 'Flight allowed and overhead drop envelope verified in the aircraft profile', d => !!d.data?.flight_enabled, d => d.data?.flight_enabled ? 'yes' : 'no']],
    does: ['Everything in mission 3.', 'Over the first geotagged person: releases the payload once, only when steady and inside the drop height window.',
      'A release is never repeated automatically.'],
    limits: ['Operator must confirm the package is loaded at upload (servo command does not prove delivery).', 'Drop height window comes from the aircraft profile.',
      'Releasing over a person is a deliberate choice: make sure the package is safe to drop.'] },
  { id: 'search_orbit', n: 6, title: 'Search & orbit', ic: 'orbit', auto: true, orbitChoice: true,
    summary: 'Just give the area: drones plan their own lanes, geotag each person, and the drones you choose also orbit them.',
    // Planned as a custom mission: every drone searches and geotags; the chosen drones also orbit.
    features: { camera: true, payload: false, type: 'coverage', detect: true, geotag: true, orbit: false, drop: false, resume: true, finish: 'RTL', emergency: 'RTL' },
    orbitFeatures: { camera: true, payload: false, type: 'coverage', detect: true, geotag: true, orbit: true, drop: false, resume: true, finish: 'RTL', emergency: 'RTL' },
    options: ['altitude_m', 'spacing_m', 'speed_mps', 'heading_deg', 'corridor_m', 'search_pitch_nadir', 'lost_timeout_s', 'approach_speed', 'orbit_radius_m', 'orbit_laps', 'orbit_speed', 'rtl_speed'],
    requires: [...VISION_REQ, GNSS_REQ],
    does: ['No Generate step: as soon as the area is set (file, drawn, or the arena) and the conditions are accepted, the lanes are planned and assigned automatically.',
      'Every aircraft flies its share of the area, camera straight down and sweeping sideways.',
      'For each new person beside its lane: moves over them, geotags, then rejoins the lane right there.',
      'The aircraft you allow to orbit also circle that person at the set radius for the set laps before rejoining the lane.',
      'With several aircraft you choose which ones may orbit; the others only geotag and keep searching.'],
    limits: ['Orbit radius 3–100 m; the circle may extend beyond the area edge by up to the radius.', 'Orbit speed ≤ 3 m/s.',
      'Keep the radius larger than obstacles around the person (trees, poles).',
      'Geotag accuracy depends on GNSS quality and flat ground (shown as ± metres).'] },
  { id: 'search_deliver', n: 7, title: 'Search, geotag & deliver', ic: 'gift', auto: true, roleChoice: true, delivers: true,
    summary: 'Search drones geotag people; delivery drones get the geotags over the mesh, split them and drop a package to each.',
    features: { camera: true, payload: false, type: 'coverage', detect: true, geotag: true, orbit: false, drop: false, resume: true, finish: 'RTL', emergency: 'RTL' },
    options: ['altitude_m', 'spacing_m', 'speed_mps', 'heading_deg', 'corridor_m', 'search_pitch_nadir', 'lost_timeout_s', 'approach_speed', 'delivery_wait_s', 'delivery_speed', 'rtl_speed'],
    requires: [...VISION_REQ, GNSS_REQ],
    does: ['No Generate step: set the area, accept the conditions and choose each aircraft\'s role — it plans and assigns by itself.',
      'Search drones share the area, camera straight down and sweeping sideways, and geotag every person beside their lanes (as mission 3).',
      'Every geotag goes over the drone mesh to the delivery drones.',
      'Delivery drones wait on the ground. Each claims the nearest geotagged person nobody else serves — two delivery drones never take the same person; if one takes person A, the other takes person B.',
      'The delivery drone takes off, flies over the person at mission height, holds steady, releases its package once, returns and lands.'],
    limits: ['One package per delivery drone; people within 8 m of each other count as one target.',
      'The drop is over the geotag position (no camera check at the drop): accuracy is the geotag\'s (± metres shown).',
      'Drop height must be inside the drop window of the aircraft profile; the package must be safe to drop.',
      'Operator must confirm each delivery drone\'s package at upload (servo command does not prove delivery).'] },
  { id: 'swarm_coverage', n: 5, title: 'Swarm coverage', ic: 'mesh', summary: 'Every aircraft flies its share of the area. No detection.',
    options: ['altitude_m', 'spacing_m', 'speed_mps', 'heading_deg', 'corridor_m', 'rtl_speed'],
    requires: [],
    does: ['Splits the area into one contiguous band of lanes per aircraft.', 'Flies the lanes, returns to launch, lands.'],
    limits: ['No camera use.'] },
];
const MCOLOR = ['#4f8df7', '#9b6cf6', '#ec5fa0', '#f0a12c', '#16a596', '#f2556b', '#22b07d'];
const PAYLOAD_REQ = ['flight', 'Flight allowed and overhead drop envelope verified in the aircraft profile', d => !!d.data?.flight_enabled, d => d.data?.flight_enabled ? 'yes' : 'no'];

/** Parameters a custom mission uses, from its features (same catalogue as the built-ins). */
export function optionsFor(f) {
  const o = ['altitude_m'];
  if (f.type === 'watch') o.push('transit_mps', 'search_time_s', 'scan_rate_dps'); else o.push('spacing_m', 'speed_mps', 'heading_deg', 'corridor_m');
  if (f.detect) o.push(f.geotag ? 'search_pitch_nadir' : 'search_pitch_deg', 'lost_timeout_s');
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
  intro: 'Choose the mission, read and accept its conditions, draw the search area on the map or load a KML file. One tree per aircraft is generated and assigned to Upload.',
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
    this.accept = h('input', { type: 'checkbox', onchange: () => { this.updateButtons(); this.autoPlan(); } });
    this.missionCard = card({ title: 'Mission', sub: 'built-in or your own · last choice is remembered', ic: 'grid', accent: 'violet',
      right: h('button', { class: 'btn sm primary', onclick: () => this.build({}) }, h('span', { html: icon('plus') }), 'New mission') }, this.list);
    this.termsCard = card({ title: 'Conditions', ic: 'shield', accent: 'amber', right: this.termsBadge = badge('—') }, this.terms);
    // area
    this.fileInput = h('input', { type: 'file', accept: '.kml', style: { display: 'none' }, onchange: e => this.loadFile(e.target.files[0]) });
    this.drop = h('div', { class: 'drop', onclick: () => this.fileInput.click() },
      h('b', {}, 'Load KML area — drop the file here or click'), h('small', {}, 'or draw the area on the map below · polygon; holes and no-fly zones in a file are respected'), this.fileInput);
    ['dragenter', 'dragover'].forEach(ev => this.drop.addEventListener(ev, e => { e.preventDefault(); this.drop.classList.add('over'); }));
    ['dragleave', 'drop'].forEach(ev => this.drop.addEventListener(ev, e => { e.preventDefault(); this.drop.classList.remove('over'); }));
    this.drop.addEventListener('drop', e => this.loadFile(e.dataTransfer.files[0]));
    this.mapBox = h('div', { class: 'map' });
    // Simulation: the whole rescue arena (10 m inside its edges) as the search area, one click.
    this.arenaBtn = h('button', { class: 'btn sm', style: { display: 'none', marginTop: '8px' }, onclick: () => this.useArena() },
      h('span', { html: icon('grid') }), 'Use arena as search area');
    // Draw the area on the map (Simulation and Field): toggle, corners by clicking, finish -> KML.
    this.drawToggle = h('input', { type: 'checkbox', onchange: e => (e.target.checked ? this.drawer?.start() : this.drawer?.stop()) });
    this.drawInfo = h('span', { class: 'draw-info' });
    const dbtn = (ic, label, title, fn, cls = 'btn sm') => h('button', { class: cls, title, onclick: fn }, h('span', { html: icon(ic) }), label);
    this.drawBtns = {
      undo: dbtn('undo', 'Undo', 'Remove the last corner (Ctrl+Z)', () => this.drawer?.undo()),
      clear: dbtn('stop', 'Clear', 'Remove every corner', () => this.drawer?.clear()),
      fit: dbtn('map', 'Zoom to it', 'Show the whole drawn area', () => this.drawer?.fit()),
      save: dbtn('download', 'Save .kml', 'Save the drawn area as a KML file (to reuse or share)', () => this.saveDrawn()),
      use: dbtn('check', 'Use as search area', 'Finish the polygon and plan the mission in it (Enter)', () => this.useDrawn(this.drawer?.points || []), 'btn sm primary'),
    };
    this.drawHelp = h('div', { class: 'draw-help faint' }, 'Click the map to add corners · drag a corner to move it · drag a small dot to add a corner between two · right-click a corner to remove it · click the first corner, double-click or press Enter to finish · Esc stops drawing.');
    this.drawBar = h('div', { class: 'draw-bar' },
      h('label', { class: 'switch draw-switch', title: 'Draw the search area on the map instead of loading a file' }, this.drawToggle, h('span', { class: 'slider' }),
        h('b', {}, h('span', { html: icon('pen') }), 'Draw area on map')),
      this.drawInfo,
      h('div', { class: 'draw-actions' }, ...Object.values(this.drawBtns)),
      this.drawHelp);
    this.areaCard = card({ title: 'Search area', ic: 'map', accent: 'teal', right: this.areaBadge = badge('no KML') }, this.drop, this.arenaBtn, this.mapBox, this.drawBar);
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
      this.orbitBox = h('div', { class: 'orbit-pick', style: { display: 'none' } }),
      h('div', { class: 'kicker', style: { marginTop: '12px' } }, 'Measured endurance per aircraft (min)'), this.fleetBox,
      h('div', { class: 'row', style: { marginTop: '12px' } }, this.genBtn, this.toUpload, this.genNote = h('span', { class: 'faint', style: { fontSize: '11.5px' } })), this.progress, this.result);
    root.append(h('div', { class: 'cols-plan' },
      h('div', { class: 'grid', style: { alignContent: 'start' } }, this.missionCard, this.termsCard),
      h('div', { class: 'grid', style: { alignContent: 'start' } }, this.areaCard, this.optCard)));
    this.map = createMap(this.mapBox);
    this.setupDrawing();
    loadCatalog();
    this.pick(MISSIONS.some(m => m.id === store.missionType) || isCustom(store.missionType) ? store.missionType : 'human_geotag_resume', true);
  },
  shown() { this.map?.resize(); this.renderFleet(); this.renderOrbit(); },
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
      const [key, label, def, min, max, remembered = key] = O[name];
      const input = this.inputs[key + ':' + name] ||= h('input', { type: 'number', step: 'any', min, max, value: prefs.get('opt.' + remembered, def),
        onchange: () => { prefs.set('opt.' + remembered, +input.value); } });
      input.dataset.key = key;
      return h('label', { class: 'field' }, label, input, h('small', {}, `${min} … ${max}`));
    }));
    this.renderTerms();
    this.renderOrbit();
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
  // Mission 6 "Search & orbit": which aircraft may orbit a person (the others geotag only).
  orbitDrones(drones = store.fleet.filter(isFresh)) {
    if (drones.length === 1) return [drones[0].drone_id];
    const chosen = prefs.get('orbitDrones', []) || [];
    return drones.map(d => d.drone_id).filter(id => chosen.includes(id));
  },
  // Mission 7: role of each aircraft ('search' | 'delivery'); default first half search.
  droneRoles(drones = store.fleet.filter(isFresh)) {
    const saved = prefs.get('droneRoles', {}) || {}, half = Math.ceil(drones.length / 2);
    return Object.fromEntries(drones.map((d, i) => [d.drone_id, saved[d.drone_id] || (i < half ? 'search' : 'delivery')]));
  },
  rolesValid(roles) {
    const v = Object.values(roles);
    return v.includes('search') && v.includes('delivery');
  },
  renderRoles(drones) {
    const roles = this.droneRoles(drones);
    const n = r => Object.values(roles).filter(x => x === r).length;
    fill(this.orbitBox, [
      h('div', { class: 'kicker' }, 'Role of each aircraft'),
      h('div', { class: 'orbit-list' }, ...drones.map(d => {
        const role = roles[d.drone_id];
        return h('button', { class: 'orbit-chip role-chip on ' + role, title: 'Click to switch between search and delivery',
          onclick: () => { prefs.set('droneRoles', { ...(prefs.get('droneRoles', {}) || {}), [d.drone_id]: role === 'search' ? 'delivery' : 'search' });
            this.orbitBox.classList.remove('ask'); this.renderOrbit(); this.autoPlan(); } },
          h('span', { html: icon(role === 'search' ? 'pin' : 'gift') }), `${d.label} · ${role === 'search' ? 'Search' : 'Delivery'}`);
      })),
      h('small', { class: 'faint' }, this.rolesValid(roles) ? `${n('search')} search, ${n('delivery')} delivery. Click an aircraft to switch its role.`
        : drones.length < 2 ? 'Mission 7 needs at least two aircraft (one searches, one delivers).' : 'Needs at least one search drone and one delivery drone.')]);
  },
  renderOrbit() {
    if (!this.orbitBox) return;
    const m = this.mission(), drones = store.fleet.filter(isFresh);
    this.orbitBox.style.display = m.orbitChoice || m.roleChoice ? '' : 'none';
    if (m.roleChoice) return this.renderRoles(drones);
    if (!m.orbitChoice) return;
    const chosen = this.orbitDrones(drones);
    fill(this.orbitBox, [
      h('div', { class: 'kicker' }, drones.length > 1 ? 'Which aircraft may orbit a person?' : 'Orbit'),
      drones.length > 1
        ? h('div', { class: 'orbit-list' }, ...drones.map(d => h('label', { class: 'orbit-chip' + (chosen.includes(d.drone_id) ? ' on' : '') },
            h('input', { type: 'checkbox', checked: chosen.includes(d.drone_id), onchange: e => {
              const set = new Set(prefs.get('orbitDrones', []) || []);
              e.target.checked ? set.add(d.drone_id) : set.delete(d.drone_id);
              prefs.set('orbitDrones', [...set]); this.orbitBox.classList.remove('ask'); this.renderOrbit(); this.autoPlan();
            } }), h('span', { html: icon('orbit') }), d.label)))
        : h('p', { class: 'faint', style: { margin: '2px 0 0', fontSize: '12px' } }, drones.length ? `${drones[0].label} geotags and orbits each person.` : 'Connect an aircraft first.'),
      drones.length > 1 ? h('small', { class: 'faint' }, chosen.length ? `${chosen.length} of ${drones.length} orbit; the others geotag and keep searching.` : 'None chosen yet: tick at least one.') : null]);
  },
  askOrbit() {
    this.renderOrbit(); this.orbitBox.classList.add('ask');
    this.orbitBox.scrollIntoView({ behavior: 'smooth', block: 'center' });
  },
  /** Missions with auto planning (6): plan and assign as soon as everything needed is there. */
  autoPlan() {
    const m = this.mission();
    if (!m.auto || !this.kmlText || !this.accept.checked) return;
    if (this.requirements().some(r => r[3] && !r[1])) return;
    const drones = store.fleet.filter(isFresh);
    if (m.orbitChoice && drones.length > 1 && !this.orbitDrones(drones).length) return this.askOrbit();
    if (m.roleChoice && !this.rolesValid(this.droneRoles(drones))) return this.askOrbit();
    clearTimeout(this.autoTimer);
    this.autoTimer = setTimeout(() => this.generate(false), 400);
  },
  renderFleet() {
    const drones = store.fleet;
    fill(this.fleetBox, drones.length ? drones.map(d => {
      const key = 'endurance.' + d.drone_id;
      const input = this.inputs[key] ||= h('input', { type: 'number', min: 1, step: 'any', value: prefs.get(key, 20), onchange: () => prefs.set(key, +input.value) });
      return h('label', { class: 'field' }, d.label, input);
    }) : [h('p', { class: 'muted', style: { margin: 0 } }, 'Add aircraft in step 1 — one tree is made per registered aircraft.')]);
  },
  setupDrawing() {
    if (!this.map) return;
    this.drawer = createAreaDrawer(this.map, { onChange: s => this.renderDraw(s), onFinish: pts => this.useDrawn(pts) });
    // The same toggle as a button on the map itself.
    const plan = this;
    const Draw = L.Control.extend({ options: { position: 'topleft' }, onAdd() {
      const b = L.DomUtil.create('a', 'draw-ctl'); b.href = '#'; b.title = 'Draw the search area on the map';
      b.innerHTML = `${icon('pen')}<span>Draw area</span>`;
      L.DomEvent.disableClickPropagation(b);
      L.DomEvent.on(b, 'click', e => { L.DomEvent.preventDefault(e); plan.drawer.toggle(); });
      const wrap = L.DomUtil.create('div', 'leaflet-bar draw-ctl-wrap'); wrap.append(b); plan.drawCtl = b; return wrap; } });
    new Draw().addTo(this.map.map);
    this.renderDraw({ active: false, points: this.drawer.points, used: false });
  },
  renderDraw({ active, points }) {
    const n = points.length, st = polygonStats(points), bad = n >= 4 && selfIntersects(points);
    this.drawToggle.checked = active;
    this.drawCtl?.classList.toggle('on', active);
    this.drawBar.classList.toggle('on', active);
    this.drawHelp.style.display = active ? '' : 'none';
    this.drawInfo.className = `draw-info ${bad ? 'bad' : ''}`;
    this.drawInfo.textContent = !n ? (active ? 'Click the map to place the first corner' : '')
      : bad ? `${n} corners · edges cross — move a corner so the outline does not cross itself`
        : `${n} corner${n > 1 ? 's' : ''}${n >= 3 ? ` · ${fmtArea(st.area_m2)} · perimeter ${fmtLen(st.perimeter_m)}` : ` · ${3 - n} more to make an area`}`;
    const shape = n >= 3 && !bad;
    this.drawBtns.undo.disabled = !n; this.drawBtns.clear.disabled = !n; this.drawBtns.fit.disabled = !n;
    this.drawBtns.save.disabled = !shape; this.drawBtns.use.disabled = !shape;
    this.drawBtns.undo.style.display = this.drawBtns.clear.style.display = active || n ? '' : 'none';
    this.drawBtns.fit.style.display = this.drawBtns.save.style.display = this.drawBtns.use.style.display = n ? '' : 'none';
  },
  drawnCheck(pts) {
    if (pts.length < 3) { toast('Not an area yet', 'Place at least 3 corners.', 'warn'); return false; }
    if (selfIntersects(pts)) { toast('Outline crosses itself', 'Move a corner so the edges do not cross, then try again.', 'bad', 8000); return false; }
    const st = polygonStats(pts);
    if (st.area_m2 < 100) { toast('Area too small', `${fmtArea(st.area_m2)} — draw at least 10 × 10 m.`, 'warn'); return false; }
    // Simulation: the drones only have ground (and people) inside the 300 m arena.
    const a = sim.arena;
    if (a && store.mode === 'sim') {
      const o = a.home, [[w, s], [e, n]] = a.bounds_enu, k = 111320, c = Math.cos(o.latitude_deg * Math.PI / 180);
      const out = pts.filter(([lat, lon]) => { const x = (lon - o.longitude_deg) * k * c, y = (lat - o.latitude_deg) * k; return x < w || x > e || y < s || y > n; }).length;
      if (out) toast('Partly outside the arena', `${out} corner${out > 1 ? 's are' : ' is'} outside the orange arena outline: there is no ground or people there in the simulation.`, 'warn', 9000);
    }
    return true;
  },
  useDrawn(pts) {
    if (!this.drawnCheck(pts)) return;
    const st = polygonStats(pts), name = `drawn_area_${pts.length}pts_${fmtArea(st.area_m2).replace(/[^0-9a-z.]+/gi, '')}.kml`;
    this.loadFile(new File([kmlFromPoints(pts, 'AERIXIS drawn search area')], name, { type: 'application/vnd.google-earth.kml+xml' }));
    this.drawer.markUsed();
    toast('Search area set', `${pts.length} corners · ${fmtArea(st.area_m2)}. Turn on Draw area again to change it.`, 'ok');
  },
  saveDrawn() {
    const pts = this.drawer?.points || [];
    if (!this.drawnCheck(pts)) return;
    const url = URL.createObjectURL(new Blob([kmlFromPoints(pts, 'AERIXIS drawn search area')], { type: 'application/vnd.google-earth.kml+xml' }));
    const a = h('a', { href: url, download: `aerixis_area_${new Date().toISOString().slice(0, 16).replace(/[-:T]/g, '')}.kml` });
    document.body.append(a); a.click(); a.remove(); setTimeout(() => URL.revokeObjectURL(url), 2000);
  },
  useArena() {
    if (!sim.arena) return toast('No arena running', 'Start the simulation on the Connect page first.', 'warn');
    this.loadFile(new File([arenaKml(sim.arena)], 'rescue_arena_sim.kml', { type: 'application/vnd.google-earth.kml+xml' }));
  },
  async loadFile(file) {
    if (!file) return;
    if (file.size > 1e6) return toast('KML too large', 'Choose a file up to 1 MB.', 'bad');
    this.kmlText = await file.text(); this.kmlName = file.name;
    this.areaBadge.className = 'badge info'; this.areaBadge.textContent = `${file.name} · ${(file.size / 1024).toFixed(1)} KB`;
    this.map?.resetView(); this.renderTerms();
    // Preview the area right away (no assignment until the conditions are accepted); missions
    // that plan by themselves (6) then plan and assign at once if everything is ready.
    await this.generate(true);
    this.autoPlan();
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
    // Mission 6: a custom mission underneath, with the orbit only for the drones chosen to orbit.
    const orbiters = m.orbitChoice ? this.orbitDrones(drones) : null;
    if (m.orbitChoice && !preview && !orbiters.length) { this.askOrbit(); return toast('Choose the orbit drones', 'Tick which aircraft may orbit a person (the others geotag and keep searching).', 'warn', 9000); }
    // Mission 7: each aircraft searches or delivers; at least one of each.
    const roles = m.roleChoice ? this.droneRoles(drones) : null;
    if (roles && !preview && !this.rolesValid(roles)) { this.askOrbit(); return toast('Choose the roles', 'Mission 7 needs at least one search drone and one delivery drone.', 'warn', 9000); }
    const request = { mission_id: crypto.randomUUID().replaceAll('-', ''), template: m.custom || m.features ? 'custom' : m.id, kml: this.kmlText, payload_confirmation: 'operator',
      emergency_action: emergencyAction(), ...(m.custom || m.features ? { features: m.features } : {}),
      fleet: drones.map(d => ({ drone_id: d.drone_id, endurance_s: (+this.inputs['endurance.' + d.drone_id]?.value || 20) * 60, reserve_fraction: .25, coverage: true,
        ...(orbiters?.includes(d.drone_id) ? { features: m.orbitFeatures } : {}),
        ...(roles ? { role: roles[d.drone_id] } : {}) })) };
    for (const input of this.optionBox.querySelectorAll('input')) {
      const v = +input.value, min = +input.min, max = +input.max;
      if (!isFinite(v) || v < min || v > max) return toast('Parameter out of range', `${input.closest('label').firstChild.textContent}: ${min} … ${max}`, 'bad');
      request[input.dataset.key] = v;
    }
    const drops = m.id === 'human_geotag_drop_resume' || !!m.features?.drop || !!m.delivers;
    request.drop_mode = drops ? 'overhead' : 'beside'; request.overhead_authorized = drops;
    this.genBtn.classList.add('busy'); this.progress.style.display = ''; this.progress.classList.add('indeterminate');
    try {
      const plan = await api('/api/plan/kml', request);
      store.plan = plan; this.map?.showPlan(plan);
      if (preview) { fill(this.result, [h('div', { class: 'faint', style: { marginTop: '10px', fontSize: '12px' } }, m.auto ? 'Area preview. Accept the conditions (and choose the orbit drones) — planning then happens by itself.' : 'Area preview only. Accept the conditions and press Generate to assign trees.')]); return; }
      if (!plan.complete) { fill(this.result, [hint(plan.reason, 'warn')]); toast('Plan needs attention', plan.reason, 'warn', 9000); return; }
      // usesCamera: Simulation opens the drone camera window when such a mission starts.
      const usesCamera = m.custom ? !!m.features?.detect : (m.requires || []).length > 0 && m.id !== 'swarm_coverage';
      for (const b of plan.bundles) store.assigned.set(b.drone_id, { xml: plan.xml[b.drone_id], bundle: b, label: `${m.custom ? '' : 'M' + m.n + ' '}${m.title}${orbiters?.includes(b.drone_id) ? ' (orbits)' : ''}${roles ? (roles[b.drone_id] === 'delivery' ? ' (delivery)' : ' (search)') : ''} · ${this.kmlName}`, source: 'plan', template: plan.template, usesCamera, emergency_action: plan.emergency_action || emergencyAction() });
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
      const roleText = roles ? ` · delivery: ${Object.keys(roles).filter(id => roles[id] === 'delivery').join(', ')}` : '';
      toast(m.auto ? 'Planned automatically' : 'Trees assigned', `${plan.bundles.length} aircraft${orbiters ? ` · orbit: ${orbiters.length ? orbiters.join(', ') : 'none'}` : ''}${roleText}. Review and upload them in step 4.`, 'ok');
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
    const arena = store.mode === 'sim' ? sim.arena : null;
    this.map?.showArena(arena); this.arenaBtn.style.display = arena ? '' : 'none';
    const now = Date.now(); if (now - (this.termsAt || 0) > 2000) { this.termsAt = now; this.renderTerms(); }
  },
};
