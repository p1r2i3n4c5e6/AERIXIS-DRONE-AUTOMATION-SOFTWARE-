// Draw the search area on the map instead of loading a KML file (2026-10-31).
// Click the map to add corners, drag a corner to move it, right-click a corner to remove it,
// click the first corner (or Enter) to finish. The polygon becomes a normal KML search area, the
// same as a loaded file, for Simulation and Field missions alike. The last drawing is remembered.
import { prefs } from './core.js';

const M_PER_DEG = 111320;

/** Local metres (east, north) of [lat, lon] around ref. */
function enu(ref, p) {
  return [(p[1] - ref[1]) * M_PER_DEG * Math.cos(ref[0] * Math.PI / 180), (p[0] - ref[0]) * M_PER_DEG];
}

export function polygonStats(pts) {
  if (pts.length < 2) return { area_m2: 0, perimeter_m: 0 };
  const xy = pts.map(p => enu(pts[0], p));
  let area = 0, per = 0;
  for (let i = 0; i < xy.length; i++) {
    const a = xy[i], b = xy[(i + 1) % xy.length];
    if (pts.length >= 3 || i < xy.length - 1) per += Math.hypot(b[0] - a[0], b[1] - a[1]);
    area += a[0] * b[1] - b[0] * a[1];
  }
  return { area_m2: pts.length >= 3 ? Math.abs(area) / 2 : 0, perimeter_m: per };
}

/** True when two non-adjacent edges of the closed polygon cross (a bow-tie is not a valid area). */
export function selfIntersects(pts) {
  const n = pts.length; if (n < 4) return false;
  const xy = pts.map(p => enu(pts[0], p));
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const hit = (p1, p2, p3, p4) => {
    const d1 = cross(p3, p4, p1), d2 = cross(p3, p4, p2), d3 = cross(p1, p2, p3), d4 = cross(p1, p2, p4);
    return ((d1 > 0) !== (d2 > 0)) && ((d3 > 0) !== (d4 > 0));
  };
  for (let i = 0; i < n; i++)
    for (let j = i + 1; j < n; j++) {
      if (j === i + 1 || (i === 0 && j === n - 1)) continue;      // neighbouring edges share a corner
      if (hit(xy[i], xy[(i + 1) % n], xy[j], xy[(j + 1) % n])) return true;
    }
  return false;
}

export function kmlFromPoints(pts, name = 'AERIXIS drawn search area') {
  const ring = [...pts, pts[0]].map(([lat, lon]) => `${lon.toFixed(8)},${lat.toFixed(8)},0`).join(' ');
  const safe = name.replace(/[<&>]/g, '');
  return `<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2"><Document><name>${safe}</name>
<Placemark><name>${safe}</name><Polygon><outerBoundaryIs><LinearRing><coordinates>${ring}</coordinates></LinearRing></outerBoundaryIs></Polygon></Placemark>
</Document></kml>
`;
}

export const fmtArea = m2 => m2 >= 10000 ? `${(m2 / 10000).toFixed(2)} ha` : `${Math.round(m2)} m²`;
export const fmtLen = m => m >= 1000 ? `${(m / 1000).toFixed(2)} km` : `${Math.round(m)} m`;

/**
 * Polygon drawing on a createMap() map. onChange({ active, points, closed }) after every change;
 * onFinish(points) when the operator closes the polygon (first corner, double-click or Enter).
 */
export function createAreaDrawer(mapApi, { onChange = () => {}, onFinish = () => {} } = {}) {
  const map = mapApi.map, layer = L.layerGroup().addTo(map);
  // Rubber-band line from the last corner to the mouse (own layer: moving the mouse never rebuilds the corners).
  const rubber = L.polyline([], { color: '#f2556b', weight: 1.5, dashArray: '4 5', interactive: false }).addTo(map);
  let pts = (prefs.get('drawnArea', []) || []).filter(p => Array.isArray(p) && p.length === 2);
  let active = false, used = false, hover = null, dragging = false;

  const save = () => prefs.set('drawnArea', pts);
  const changed = () => { save(); redraw(); onChange(state()); };
  const state = () => ({ active, points: pts.slice(), used });

  const vertexIcon = (first) => L.divIcon({ className: '', iconSize: [16, 16], iconAnchor: [8, 8],
    html: `<div class="draw-vertex ${first ? 'first' : ''} ${first && active && pts.length >= 3 ? 'can-close' : ''}"></div>` });

  function redraw() {
    layer.clearLayers(); rubber.setLatLngs([]);
    if (!pts.length || (!active && used)) return;      // once used, the plan's own outline shows it
    const style = { color: '#f2556b', weight: 2.5, fillColor: '#f2556b', fillOpacity: active ? .12 : .06, dashArray: active ? null : '6 5' };
    if (pts.length >= 3) L.polygon(pts, { ...style, interactive: false }).addTo(layer);
    else if (pts.length === 2) L.polyline(pts, { ...style, interactive: false }).addTo(layer);
    drawRubber();
    if (!active) return;
    pts.forEach((p, i) => {
      const m = L.marker(p, { icon: vertexIcon(i === 0), draggable: true, keyboard: false, zIndexOffset: 2000 }).addTo(layer);
      m.bindTooltip(i === 0 && pts.length >= 3 ? 'Click to finish' : `Corner ${i + 1} · drag to move · right-click to remove`, { direction: 'top', offset: [0, -8] });
      m.on('dragstart', () => { dragging = true; rubber.setLatLngs([]); });
      m.on('drag', e => { pts[i] = [e.latlng.lat, e.latlng.lng]; const shape = layer.getLayers().find(l => l instanceof L.Polygon || l instanceof L.Polyline); shape?.setLatLngs(pts); });
      m.on('dragend', () => { dragging = false; used = false; changed(); });
      m.on('click', () => { if (i === 0 && pts.length >= 3) finish(); });
      m.on('contextmenu', e => { L.DomEvent.preventDefault(e.originalEvent || e); pts.splice(i, 1); used = false; changed(); });
    });
    // Midpoint handles: drag one to add a corner between two others.
    if (pts.length >= 2) pts.forEach((p, i) => {
      if (pts.length < 3 && i === pts.length - 1) return;
      const q = pts[(i + 1) % pts.length], mid = [(p[0] + q[0]) / 2, (p[1] + q[1]) / 2];
      const m = L.marker(mid, { icon: L.divIcon({ className: '', iconSize: [12, 12], iconAnchor: [6, 6], html: '<div class="draw-mid"></div>' }),
        draggable: true, keyboard: false, zIndexOffset: 1900 }).addTo(layer);
      m.bindTooltip('Drag to add a corner here', { direction: 'top', offset: [0, -6] });
      m.on('dragend', e => { pts.splice(i + 1, 0, [e.target.getLatLng().lat, e.target.getLatLng().lng]); used = false; changed(); });
    });
  }

  function onClick(e) {
    if (!active) return;
    pts.push([e.latlng.lat, e.latlng.lng]); used = false; changed();
  }
  function drawRubber() {
    rubber.setLatLngs(active && !dragging && hover && pts.length ? [pts[pts.length - 1], hover, ...(pts.length >= 2 ? [pts[0]] : [])] : []);
  }
  function onMove(e) { if (!active || !pts.length) return; hover = [e.latlng.lat, e.latlng.lng]; drawRubber(); }
  function onKey(e) {
    if (!active || /input|textarea|select/i.test(e.target.tagName)) return;
    if (e.key === 'Escape') { e.stopPropagation(); stop(); }
    else if (e.key === 'Enter' && pts.length >= 3) finish();
    else if ((e.key === 'z' && (e.ctrlKey || e.metaKey)) || e.key === 'Backspace') { e.preventDefault(); undo(); }
  }
  map.on('click', onClick);
  map.on('mousemove', onMove);
  map.on('mouseout', () => { hover = null; drawRubber(); });
  map.on('dblclick', e => { if (active && pts.length >= 3) { L.DomEvent.stop(e); finish(); } });
  document.addEventListener('keydown', onKey, true);

  function start() {
    if (active) return;
    active = true; hover = null;
    map.doubleClickZoom.disable();
    map.getContainer().classList.add('drawing');
    redraw(); onChange(state());
  }
  function stop() {
    if (!active) return;
    active = false; hover = null;
    map.doubleClickZoom.enable();
    map.getContainer().classList.remove('drawing');
    redraw(); onChange(state());
  }
  function finish() {
    if (pts.length < 3) return;
    // A double-click also adds two clicks' worth of corners: drop near-duplicates at the end.
    while (pts.length > 3 && map.distance(pts[pts.length - 1], pts[pts.length - 2]) < 0.5) pts.pop();
    save(); onFinish(pts.slice());
  }
  function undo() { if (pts.length) { pts.pop(); used = false; changed(); } }
  function clear() { pts = []; used = false; changed(); }

  redraw();
  return {
    start, stop, undo, clear, finish,
    toggle() { active ? stop() : start(); },
    markUsed() { used = true; stop(); redraw(); onChange(state()); },
    fit() { if (pts.length) map.fitBounds(L.latLngBounds(pts).pad(.3), { maxZoom: 19 }); },
    get active() { return active; },
    get points() { return pts.slice(); },
  };
}
