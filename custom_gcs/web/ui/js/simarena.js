// Simulation mode: the PX4 SITL rescue arena (custom_gcs/simulation). Status of the simulation
// (started from Connect), the arena layout for the maps (outline + the 50 ground-truth people)
// and helpers: the arena as a KML search area, and the distance of a geotag to the nearest
// ground-truth person (how good the geotagging was). Only used while store.mode === 'sim'.
import { store } from './core.js';

export const sim = { status: null, arena: null, error: '', listeners: new Set() };

async function getJson(path) {
  const r = await fetch(path);
  const data = await r.json().catch(() => ({}));
  if (!r.ok || data.error) throw new Error(data.error || `${path}: HTTP ${r.status}`);
  return data;
}

export async function refreshSim() {
  if (store.mode !== 'sim') { if (sim.arena || sim.status) { sim.arena = null; sim.status = null; sim.listeners.forEach(f => f()); } return; }
  try {
    sim.status = await getJson('/api/sim/status'); sim.error = '';
    sim.arena = sim.status.components?.px4 || sim.status.state === 'starting' ? (await getJson('/api/sim/arena')).arena : null;
  } catch (e) { sim.error = e.message; }
  sim.listeners.forEach(f => f());
}

let timer = null;
export function watchSim() {
  if (timer) return;
  refreshSim();
  timer = setInterval(refreshSim, 3000);
}

// Metres east/north of the arena centre -> [lat, lon] (same flat-earth step as the maps).
export function arenaLatLon(arena, east, north) {
  const o = arena.home;
  return [o.latitude_deg + north / 111320,
    o.longitude_deg + east / (111320 * Math.max(0.01, Math.cos(o.latitude_deg * Math.PI / 180)))];
}

/** The arena (inset by `inset` metres from its edges) as a KML polygon for the Plan page. */
export function arenaKml(arena, inset = 10) {
  const [[w, s], [e, n]] = arena.bounds_enu;
  const ring = [[w + inset, s + inset], [e - inset, s + inset], [e - inset, n - inset], [w + inset, n - inset], [w + inset, s + inset]]
    .map(([x, y]) => { const [lat, lon] = arenaLatLon(arena, x, y); return `${lon.toFixed(8)},${lat.toFixed(8)},0`; }).join(' ');
  return `<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2"><Document><name>AERIXIS rescue arena (simulation)</name>
<Placemark><name>Rescue arena search area</name><Polygon><outerBoundaryIs><LinearRing><coordinates>${ring}</coordinates></LinearRing></outerBoundaryIs></Polygon></Placemark>
</Document></kml>
`;
}

function metres(a, b) {
  const k = 111320, dLat = (a[0] - b[0]) * k, dLon = (a[1] - b[1]) * k * Math.cos(a[0] * Math.PI / 180);
  return Math.hypot(dLat, dLon);
}

/** Nearest ground-truth person to a geotag: { name, dist } or null without an arena. */
export function nearestTruth(lat, lon) {
  if (!sim.arena?.humans?.length) return null;
  let best = null;
  for (const p of sim.arena.humans) {
    const d = metres([lat, lon], [p.latitude, p.longitude]);
    if (!best || d < best.dist) best = { name: p.name, dist: d };
  }
  return best;
}
