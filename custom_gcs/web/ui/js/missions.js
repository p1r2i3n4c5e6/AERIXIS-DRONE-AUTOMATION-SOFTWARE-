// Mission catalogue shared by Plan, Swarm start and Emergency.
// Built-in missions (the five A8 missions) are never deletable; the operator edits a copy.
// Custom missions are operator choices (feature spec + parameters + name) saved on the GCS server
// (config/missions/*.json); the planner composes their tree every time they are planned.
import { api, store, prefs } from './core.js';

// Same as as2_rescue_planner.features.BUILTIN_FEATURES; used until /api/missions answers (or if
// the planner is not installed) so readiness still knows whether the camera is needed.
const F = (o) => ({ camera: true, payload: false, type: 'coverage', detect: true, geotag: false, orbit: false, drop: false, resume: true, finish: 'RTL', emergency: 'RTL', ...o });
export const BUILTIN_FEATURES = {
  human_track_rtl: F({ type: 'track', resume: false }),
  human_orbit: F({ type: 'watch', orbit: true, resume: false }),
  human_geotag_resume: F({ geotag: true }),
  human_geotag_drop_resume: F({ payload: true, geotag: true, drop: true }),
  swarm_coverage: F({ camera: false, detect: false }),
};

export const catalog = {
  builtin: new Map(Object.entries(BUILTIN_FEATURES)), custom: [], loaded: false, error: '',
  listeners: new Set(),
  emit() { this.listeners.forEach(fn => { try { fn(); } catch (e) { console.error(e); } }); },
};

export async function loadMissions() {
  try {
    const c = await api('/api/missions');
    for (const b of c.builtin || []) catalog.builtin.set(b.id, b.features);
    catalog.custom = c.custom || []; catalog.error = c.error || ''; catalog.loaded = true;
  } catch (e) { catalog.error = e.message; }
  catalog.emit();
  return catalog;
}

export const isCustom = id => /^c_[0-9a-f]{12}$/.test(String(id || ''));
export const customById = id => catalog.custom.find(m => m.id === id) || null;

/** Feature spec of the selected mission (built-in or custom). */
export function currentFeatures(id = store.missionType) {
  if (isCustom(id)) return customById(id)?.features || null;
  return catalog.builtin.get(id) || null;
}

/** RTL or LAND on an emergency: a mission setting (custom: in the spec; built-in: remembered). */
export function emergencyAction(id = store.missionType) {
  if (store.plan?.emergency_action && store.plan.template === (isCustom(id) ? 'custom' : id)) return store.plan.emergency_action;
  if (isCustom(id)) return prefs.get('emergency.' + id, customById(id)?.features?.emergency || 'RTL');
  return prefs.get('emergency.' + id, 'RTL');
}
export function setEmergencyAction(action, id = store.missionType) { prefs.set('emergency.' + id, action === 'LAND' ? 'LAND' : 'RTL'); }

/** What the selected mission needs onboard, for readiness: camera only when it detects people. */
export function needs(id = store.missionType) {
  const f = currentFeatures(id);
  return { known: !!f, camera: !!f?.detect, gnss: !!f?.geotag, payload: !!f?.drop, emergency: emergencyAction(id) };
}

export function missionName(id = store.missionType) {
  if (isCustom(id)) return customById(id)?.name || 'custom mission';
  return { human_track_rtl: 'M1 Track & target', human_orbit: 'M2 Orbit person', human_geotag_resume: 'M3 Geotag & resume',
    human_geotag_drop_resume: 'M4 Geotag + drop', swarm_coverage: 'M5 Swarm coverage' }[id] || id;
}

export async function saveMission(record) {
  const saved = await api('/api/missions/save', record);
  await loadMissions();
  return saved;
}
export async function deleteMission(id) {
  await api('/api/missions/delete', { id });
  await loadMissions();
}
export function describeMission(features, fleetSize = 1) { return api('/api/missions/describe', { features, fleet_size: fleetSize }); }
