// Leaflet map helper. Plan coordinates are metres east/north of the KML plan origin; the
// onboard AERIXIS earth frame uses the same origin once the plan is staged.
import { icon, prefs } from './core.js';
import { arenaLatLon, nearestTruth } from './simarena.js';

const LANE_COLORS = ['#2f6aa8', '#b8501f', '#3d8b5a', '#7a4fa3', '#9a7b12', '#2a8a91'];
export const laneColor = i => LANE_COLORS[i % LANE_COLORS.length];

export function enuToLatLon(origin, east, north) {
  const lat = origin.latitude + north / 111320;
  const lon = origin.longitude + east / (111320 * Math.max(0.01, Math.cos(origin.latitude * Math.PI / 180)));
  return [lat, lon];
}

// Base maps. Satellite imagery: Esri World Imagery; labels for the hybrid view: Esri reference
// layers. Tiles need internet; the browser caches what was already viewed.
const ESRI = 'https://server.arcgisonline.com/ArcGIS/rest/services/';
function baseLayers() {
  const street = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
    { maxZoom: 21, maxNativeZoom: 19, className: 'tiles-street', attribution: '© OpenStreetMap contributors' });
  const imagery = () => L.tileLayer(ESRI + 'World_Imagery/MapServer/tile/{z}/{y}/{x}',
    { maxZoom: 21, maxNativeZoom: 19, attribution: 'Imagery © Esri, Maxar, Earthstar Geographics' });
  const labels = () => [L.tileLayer(ESRI + 'Reference/World_Transportation/MapServer/tile/{z}/{y}/{x}', { maxZoom: 21, maxNativeZoom: 19 }),
    L.tileLayer(ESRI + 'Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}', { maxZoom: 21, maxNativeZoom: 19 })];
  return { Street: street, Satellite: imagery(), Hybrid: L.layerGroup([imagery(), ...labels()]) };
}

// Where the operator (this GCS laptop) is: browser geolocation, shared by every map.
export const gcsPosition = { fix: prefs.get('gcsPos', null), listeners: new Set(), watching: false, error: '' };
export function startGeolocation() {
  if (gcsPosition.watching || !navigator.geolocation) return;
  gcsPosition.watching = true;
  navigator.geolocation.watchPosition(p => {
    gcsPosition.fix = { lat: p.coords.latitude, lon: p.coords.longitude, acc: p.coords.accuracy, t: Date.now() };
    gcsPosition.error = '';
    prefs.set('gcsPos', gcsPosition.fix);
    gcsPosition.listeners.forEach(fn => fn());
  }, e => { gcsPosition.error = e.message || 'location unavailable'; gcsPosition.watching = false; gcsPosition.listeners.forEach(fn => fn()); },
  { enableHighAccuracy: true, maximumAge: 10000, timeout: 20000 });
}
const INDIA = [22.35, 78.67];
// Same order as the drone cards: drone 1 blue, 2 pink, 3 teal, 4 violet, 5 amber, 6 coral.
const DRONE_COLORS = ['#4f8df7', '#ec5fa0', '#16a596', '#9b6cf6', '#f0a12c', '#f27a5e'];

export function createMap(element, { center, zoom } = {}) {
  if (!window.L) { element.textContent = 'Map library unavailable'; return null; }
  // Start where the operator was last seen (India by default), not at PX4's SITL home.
  const start = center || (gcsPosition.fix ? [gcsPosition.fix.lat, gcsPosition.fix.lon] : INDIA);
  const map = L.map(element, { zoomControl: true, attributionControl: true, fadeAnimation: false, zoomAnimation: true, markerZoomAnimation: false })
    .setView(start, zoom || (gcsPosition.fix ? 16 : 5));
  map.attributionControl.setPrefix('');
  const bases = baseLayers();
  const chosen = bases[prefs.get('mapBase', 'Hybrid')] ? prefs.get('mapBase', 'Hybrid') : 'Hybrid';
  bases[chosen].addTo(map);
  L.control.layers(bases, null, { position: 'topright', collapsed: false }).addTo(map);
  map.on('baselayerchange', e => prefs.set('mapBase', e.name));
  L.control.scale({ imperial: false, position: 'bottomleft' }).addTo(map);
  // "You are here": accuracy circle + dot, updated live.
  const me = L.layerGroup().addTo(map);
  let meCentered = false;
  const drawMe = () => {
    me.clearLayers();
    const f = gcsPosition.fix; if (!f) return;
    L.circle([f.lat, f.lon], { radius: Math.min(f.acc || 0, 2000), color: '#4f8df7', weight: 1, fillOpacity: .08, interactive: false }).addTo(me);
    L.marker([f.lat, f.lon], { icon: L.divIcon({ html: '<div class="gcs-dot"></div>', className: '', iconSize: [18, 18], iconAnchor: [9, 9] }), zIndexOffset: 500 })
      .addTo(me).bindTooltip(`You (GCS) · ±${Math.round(f.acc || 0)} m`, { direction: 'top', offset: [0, -8] });
  };
  gcsPosition.listeners.add(() => { drawMe(); if (!meCentered && !userMoved && gcsPosition.fix && !markers.size) { meCentered = true; map.setView([gcsPosition.fix.lat, gcsPosition.fix.lon], 16); } });
  drawMe();
  const Locate = L.Control.extend({ options: { position: 'topleft' }, onAdd() {
    const b = L.DomUtil.create('a', 'leaflet-bar-part locate-btn'); b.href = '#'; b.title = 'Show my location (GCS)'; b.innerHTML = icon('gps');
    L.DomEvent.on(b, 'click', e => { L.DomEvent.preventDefault(e); startGeolocation();
      if (gcsPosition.fix) { userMoved = true; map.setView([gcsPosition.fix.lat, gcsPosition.fix.lon], Math.max(map.getZoom(), 17)); } });
    const wrap = L.DomUtil.create('div', 'leaflet-bar'); wrap.append(b); return wrap; } });
  new Locate().addTo(map);
  if (prefs.get('locate', true)) startGeolocation();
  const layers = { arena: L.layerGroup().addTo(map), area: L.layerGroup().addTo(map), lanes: L.layerGroup().addTo(map), trails: L.layerGroup().addTo(map),
    drones: L.layerGroup().addTo(map), geotags: L.layerGroup().addTo(map) };
  // Remember where the operator looks: Connect -> Simulation uses it as the arena location.
  map.on('moveend', () => { const c = map.getCenter(); prefs.set('mapCenter', [+c.lat.toFixed(7), +c.lng.toFixed(7)]); });
  const markers = new Map(), trails = new Map();
  let userMoved = false, tagKey = '', arenaKey = '';
  map.on('dragstart', () => { userMoved = true; });
  map.on('zoomstart', e => { if (e.originalEvent) userMoved = true; });
  element.addEventListener('wheel', () => { userMoved = true; }, { passive: true });
  const api = {
    map, layers,
    resize() { setTimeout(() => map.invalidateSize(), 60); },
    showPlan(plan) {
      layers.area.clearLayers(); layers.lanes.clearLayers();
      if (!plan?.origin) return;
      const o = plan.origin, shell = plan.region?.shell || [];
      if (shell.length) {
        const poly = L.polygon(shell.map(([e, n]) => enuToLatLon(o, e, n)), { color: '#2f6aa8', weight: 2, fillColor: '#2f6aa8', fillOpacity: .07, dashArray: '5 4' }).addTo(layers.area);
        for (const hole of [...(plan.region.holes || []), ...(plan.region.exclusions || [])])
          L.polygon(hole.map(([e, n]) => enuToLatLon(o, e, n)), { color: '#c62828', weight: 1.5, fillOpacity: .15 }).addTo(layers.area);
        if (!userMoved) map.fitBounds(poly.getBounds(), { padding: [30, 30] });
      }
      (plan.bundles || []).forEach((b, i) => {
        const pts = (b.coverage?.waypoints || []).map(w => enuToLatLon(o, w.pose[0], w.pose[1]));
        if (!pts.length) return;
        if (plan.stations) {
          L.marker(pts[0], { icon: L.divIcon({ html: '<div class="station-marker"></div>', className: '', iconSize: [14, 14], iconAnchor: [7, 7] }) })
            .addTo(layers.lanes).bindTooltip(`${b.drone_id} watch station`, { permanent: true, direction: 'right', offset: [8, 0] });
        } else {
          L.polyline(pts, { color: laneColor(i), weight: 2.5, opacity: .9 }).addTo(layers.lanes).bindTooltip(b.drone_id, { sticky: true });
          L.circleMarker(pts[0], { radius: 4, color: '#fff', weight: 1.5, fillColor: laneColor(i), fillOpacity: 1 }).addTo(layers.lanes).bindTooltip(`${b.drone_id} start`);
        }
      });
    },
    /** drones: [{id, label, latlon, yawDeg, alt, selected, stale}] — markers are moved, not rebuilt. */
    showDrones(drones, onClick) {
      const seen = new Set(), bounds = [];
      for (const d of drones) {
        if (!d.latlon) continue;
        seen.add(d.id); bounds.push(d.latlon);
        const rot = Math.round((-(d.yawDeg || 0) + 90) / 3) * 3;
        const color = DRONE_COLORS[(d.index ?? 0) % DRONE_COLORS.length];
        const look = `${rot}|${d.selected}|${d.stale}|${color}`;
        let m = markers.get(d.id);
        if (!m) {
          m = L.marker(d.latlon, { zIndexOffset: 1000 }).addTo(layers.drones);
          m.bindTooltip('', { permanent: true, direction: 'top', offset: [0, -14] });
          if (onClick) m.on('click', () => onClick(d.id));
          markers.set(d.id, m);
          trails.set(d.id, L.polyline([], { color, weight: 2, opacity: .85 }).addTo(layers.trails));
        }
        m.setLatLng(d.latlon);
        if (m._look !== look) {
          m._look = look;
          m.setIcon(L.divIcon({ html: `<div class="drone-marker ${d.selected ? 'sel' : ''} ${d.stale ? 'stale' : ''}" style="transform:rotate(${rot}deg);--dc:${color}">${icon('drone')}</div>`,
            className: '', iconSize: [28, 28], iconAnchor: [14, 14] }));
        }
        const f = gcsPosition.fix, far = f ? map.distance([f.lat, f.lon], d.latlon) : null;
        m.setTooltipContent(`${d.label}${d.alt != null ? ' · ' + d.alt.toFixed(1) + ' m' : ''}${far != null ? ` · ${far < 1000 ? Math.round(far) + ' m' : (far / 1000).toFixed(1) + ' km'} from you` : ''}`);
        const trail = trails.get(d.id), pts = trail.getLatLngs(), last = pts[pts.length - 1];
        if (!last || map.distance(last, d.latlon) > 0.8) { pts.push(L.latLng(d.latlon)); if (pts.length > 300) pts.shift(); trail.setLatLngs(pts); }
      }
      for (const [id, m] of markers) if (!seen.has(id)) { layers.drones.removeLayer(m); markers.delete(id); layers.trails.removeLayer(trails.get(id)); trails.delete(id); }
      return bounds;
    },
    clearTrails() { for (const t of trails.values()) t.setLatLngs([]); },
    showGeotags(tags) {
      const key = tags.map(t => t.id || `${t.latitude},${t.longitude}`).join('|') + '|' + arenaKey;
      if (key === tagKey) return; tagKey = key;
      layers.geotags.clearLayers();
      for (const t of tags) {
        // Simulation arena: how far the geotag is from the nearest real (ground-truth) person.
        const truth = nearestTruth(t.latitude, t.longitude);
        const check = truth ? `<br><span class="geo-truth">Arena check: ${truth.dist.toFixed(1)} m from ${truth.name}</span>` : '';
        L.marker([t.latitude, t.longitude], { icon: L.divIcon({ html: '<div class="geo-marker"></div>', className: '', iconSize: [18, 18], iconAnchor: [9, 18] }) })
          .addTo(layers.geotags).bindTooltip(`${t.label}${truth ? ` · ${truth.dist.toFixed(1)} m off` : ''}`, { direction: 'right', offset: [8, -10] })
          .bindPopup(`<b>${t.label}</b> · ${t.drone || ''}<br>${t.latitude.toFixed(7)}, ${t.longitude.toFixed(7)}<br>±${t.sigma_m} m${check}`);
      }
    },
    /** Simulation: the rescue arena outline and its ground-truth people (null clears it). */
    showArena(arena) {
      const key = arena ? `${arena.home.latitude_deg},${arena.home.longitude_deg},${arena.humans.length}` : '';
      if (key === arenaKey) return; arenaKey = key; tagKey = '';
      layers.arena.clearLayers();
      if (!arena) return;
      const [[w, s], [e, n]] = arena.bounds_enu;
      const outline = L.rectangle([arenaLatLon(arena, w, s), arenaLatLon(arena, e, n)],
        { color: '#f0a12c', weight: 2, dashArray: '8 6', fillColor: '#f0a12c', fillOpacity: .04, interactive: false }).addTo(layers.arena);
      L.marker(arenaLatLon(arena, w, n), { icon: L.divIcon({ html: '<div class="arena-tag">Simulation arena · 300 m</div>', className: '', iconSize: [150, 20], iconAnchor: [-4, -4] }), interactive: false }).addTo(layers.arena);
      for (const p of arena.humans)
        L.circleMarker([p.latitude, p.longitude], { radius: 4, color: '#fff', weight: 1, fillColor: '#f0a12c', fillOpacity: .95 })
          .addTo(layers.arena).bindTooltip(`${p.name} (ground truth)`, { direction: 'top' });
      if (!userMoved && !markers.size) map.fitBounds(outline.getBounds(), { padding: [20, 20] });
    },
    follow(points) { if (!userMoved && points.length) map.fitBounds(L.latLngBounds(points).pad(.6), { maxZoom: 19, animate: false }); },
    focus(latlon) { if (latlon) { userMoved = true; map.setView(latlon, Math.max(map.getZoom(), 19)); } },
    resetView() { userMoved = false; },
  };
  return api;
}
