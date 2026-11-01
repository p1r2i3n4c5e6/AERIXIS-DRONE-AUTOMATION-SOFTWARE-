// Step 6 — Logs & data: onboard and GCS logs, geotag export, link traffic, raw XML.
import { h, icon, card, badge, store, fill, emptyState, rescueOf, linksOf, stripAnsi, api, toast } from './core.js';
import { btLogView } from './btlog.js';
import { archPanel } from './links.js';

function download(name, text, type) {
  const url = URL.createObjectURL(new Blob([text], { type }));
  const a = h('a', { href: url, download: name }); document.body.append(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
function sparkline(history) {
  const max = Math.max(1, ...history.flatMap(p => [p.tx, p.rx]));
  const path = key => history.map((p, i) => `${i ? 'L' : 'M'}${(i / Math.max(1, history.length - 1)) * 300},${68 - (p[key] / max) * 62}`).join(' ');
  const box = h('div'); box.innerHTML = `<svg class="sparkline" viewBox="0 0 300 70" preserveAspectRatio="none"><path class="rx" d="${path('rx')}"/><path class="tx" d="${path('tx')}"/></svg>`;
  return box;
}

export const page = {
  id: 'logs', title: 'Logs & data', sub: 'Records & export', ic: 'logs',
  intro: 'Everything recorded during the mission: onboard executor log, GCS log, geotags to export, link traffic and the exact XML each drone received.',
  mount(root) {
    this.root = root; this.tab = 'bt'; this.drone = null; this.archPanels = new Map();
    this.tabBar = h('div', { class: 'tabs' }, ...[['archive', 'Mission archive'], ['bt', 'BT execution'], ['onboard', 'Onboard log'], ['fc', 'Flight controller'], ['links', 'Links & data'], ['gcs', 'GCS log'], ['geotags', 'Geotags'], ['traffic', 'Link traffic'], ['xml', 'Tree XML']].map(([id, t]) =>
      h('button', { 'data-tab': id, class: id === this.tab ? 'active' : '', onclick: () => { this.tab = id; this.root.querySelectorAll('[data-tab]').forEach(b => b.classList.toggle('active', b.dataset.tab === id)); this.update(store, true); } }, t)));
    this.body = h('div');
    this.droneSel = h('select', { onchange: () => { this.drone = this.droneSel.value; this.update(store, true); } });
    root.append(card({ title: 'Mission records', sub: 'pick an aircraft and a view', ic: 'logs', accent: 'cyan', right: h('label', { class: 'field' }, this.droneSel) }, this.tabBar, this.body));
  },
  update(s, force) {
    const drones = s.fleet;
    if (drones.length && (!this.drone || !drones.some(d => d.drone_id === this.drone))) this.drone = drones[0].drone_id;
    const options = drones.map(d => d.drone_id).join();
    if (options !== this.options) { this.options = options; this.droneSel.replaceChildren(...drones.map(d => h('option', { value: d.drone_id }, d.label))); }
    this.droneSel.value = this.drone || '';
    const now = Date.now(); if (!force && now - (this.last || 0) < 1500) return; this.last = now;
    const d = store.drone(this.drone);
    let content;
    if (this.tab === 'archive') {
      content = this.archiveView();
    } else if (this.tab === 'bt') {
      content = h('div', {}, h('p', { class: 'faint', style: { margin: '0 0 8px', fontSize: '12px' } }, 'Every node change seen in this aircraft’s status updates (1 s while flying), newest first. The complete onboard event journal stays on the aircraft.'),
        d ? btLogView(d.drone_id, { limit: 400 }) : emptyState('tree', 'No aircraft', ''));
    } else if (this.tab === 'links') {
      content = d ? (() => { const el = archPanel(d, this.archPanels.get(d.drone_id)); this.archPanels.set(d.drone_id, el); return el; })() : emptyState('link', 'No aircraft', '');
    } else if (this.tab === 'onboard') {
      const logs = (d?.data?.logs || []).slice(-150).reverse();
      content = logs.length ? h('div', { class: 'log' }, ...logs.map(l => h('div', { class: l.level }, h('span', { class: 'faint' }, new Date(l.time * 1000).toLocaleTimeString()), h('b', {}, l.level), h('span', {}, stripAnsi(l.message)))))
        : emptyState('logs', 'No onboard log yet', 'The executor log appears once a mission is uploaded or run.');
    } else if (this.tab === 'fc') {
      const fc = linksOf(d).fc;
      content = !fc ? emptyState('drone', 'No flight-controller health data', 'The onboard fc_health node publishes it (AERIXIS onboard launch).') : h('div', { class: 'grid g2' },
        h('dl', { class: 'kv' }, h('dt', {}, 'GNSS'), h('dd', {}, fc.gps ? `${fc.gps.fix} · ${fc.gps.sats} sat · HDOP ${fc.gps.hdop} · eph ${fc.gps.eph ?? '—'} m` : '—'),
          h('dt', {}, 'Jamming / spoofing'), h('dd', {}, fc.gps ? `${fc.gps.jamming ?? '—'} / ${fc.gps.spoofing ?? '—'}` : '—'),
          h('dt', {}, 'Mode / arming'), h('dd', {}, fc.status ? `${fc.status.nav} · ${fc.status.arming}` : '—'),
          h('dt', {}, 'Preflight checks'), h('dd', {}, fc.status ? (fc.status.preflight_ok ? 'pass' : 'FAIL') : '—'),
          h('dt', {}, 'Failsafe'), h('dd', {}, fc.status ? (fc.status.failsafe ? 'ACTIVE' : 'no') : '—'),
          h('dt', {}, 'Unhealthy sensors'), h('dd', {}, fc.sensors == null ? 'tap ' + fc.mavlink : fc.sensors.join(', ') || 'none'),
          h('dt', {}, 'Active faults'), h('dd', {}, (fc.faults || []).join(', ') || 'none')),
        h('div', {}, h('div', { class: 'kicker' }, 'PX4 warnings (STATUSTEXT)'), (fc.messages || []).length ? h('div', { class: 'log' }, ...fc.messages.slice().reverse().map(m =>
          h('div', { class: m.severity <= 3 ? 'ERROR' : 'WARN' }, h('span', { class: 'faint' }, new Date(m.time * 1000).toLocaleTimeString()), h('b', {}, m.severity <= 3 ? 'ERROR' : 'WARN'), h('span', {}, m.text))))
          : h('p', { class: 'faint' }, fc.mavlink === 'ok' ? 'No warnings.' : `MAVLink tap ${fc.mavlink}: set fc_mavlink_url (Pi: FC USB /dev/ttyACM0).`)));
    } else if (this.tab === 'gcs') {
      const logs = (s.state?.logs || []).slice().reverse();
      content = logs.length ? h('div', { class: 'log' }, ...logs.map(l => h('div', { class: l.severity }, h('span', { class: 'faint' }, l.time), h('b', {}, l.severity), h('span', {}, l.message)))) : emptyState('logs', 'No GCS log', '');
    } else if (this.tab === 'geotags') {
      const tags = drones.flatMap(x => (rescueOf(x)?.geotags || []).map(t => ({ ...t, drone: x.drone_id })));
      const csv = () => ['drone,label,latitude,longitude,sigma_m,height_m,time', ...tags.map(t => [t.drone, t.label, t.latitude, t.longitude, t.sigma_m, (t.height_above_takeoff_m || 0).toFixed(1), new Date((t.wall_time || 0) * 1000).toISOString()].join(','))].join('\n');
      const kml = () => `<?xml version="1.0" encoding="UTF-8"?><kml xmlns="http://www.opengis.net/kml/2.2"><Document><name>AERIXIS geotags</name>${tags.map(t => `<Placemark><name>${t.drone} ${t.label}</name><description>±${t.sigma_m} m</description><Point><coordinates>${t.longitude},${t.latitude},0</coordinates></Point></Placemark>`).join('')}</Document></kml>`;
      const geojson = () => JSON.stringify({ type: 'FeatureCollection', features: tags.map(t => ({ type: 'Feature', geometry: { type: 'Point', coordinates: [t.longitude, t.latitude] }, properties: { drone: t.drone, label: t.label, sigma_m: t.sigma_m } })) }, null, 1);
      content = h('div', {}, h('div', { class: 'row', style: { marginBottom: '10px' } }, badge(`${tags.length} people geotagged`, 'violet'),
        h('button', { class: 'btn sm', disabled: !tags.length, onclick: () => download('geotags.csv', csv(), 'text/csv'), html: icon('download') + 'CSV' }),
        h('button', { class: 'btn sm', disabled: !tags.length, onclick: () => download('geotags.kml', kml(), 'application/vnd.google-earth.kml+xml'), html: icon('download') + 'KML' }),
        h('button', { class: 'btn sm', disabled: !tags.length, onclick: () => download('geotags.geojson', geojson(), 'application/geo+json'), html: icon('download') + 'GeoJSON' })),
        tags.length ? h('table', { class: 'table' }, h('thead', {}, h('tr', {}, ...['Drone', 'Person', 'Latitude', 'Longitude', 'Accuracy', 'Height', 'Time'].map(x => h('th', {}, x)))),
          h('tbody', {}, ...tags.map(t => h('tr', {}, h('td', {}, t.drone), h('td', {}, h('b', {}, t.label)), h('td', { class: 'mono' }, t.latitude.toFixed(7)), h('td', { class: 'mono' }, t.longitude.toFixed(7)),
            h('td', {}, `±${t.sigma_m} m`), h('td', {}, `${(t.height_above_takeoff_m || 0).toFixed(1)} m`), h('td', {}, t.wall_time ? new Date(t.wall_time * 1000).toLocaleTimeString() : '—')))))
          : emptyState('pin', 'No geotags yet', 'Missions 3 and 4 geotag every person they find.'));
    } else if (this.tab === 'traffic') {
      const t = s.state?.traffic || {};
      content = h('div', { class: 'grid g2' },
        h('div', {}, h('div', { class: 'row' }, badge('↓ receive', 'info'), badge('↑ transmit', 'violet')), sparkline(store.history)),
        h('dl', { class: 'kv' }, h('dt', {}, 'Received'), h('dd', {}, `${t.rx_kb ?? 0} KB · ${t.rx_packets ?? 0} packets`), h('dt', {}, 'Sent'), h('dd', {}, `${t.tx_kb ?? 0} KB · ${t.tx_packets ?? 0} packets`),
          h('dt', {}, 'Rate'), h('dd', {}, `↓ ${t.rx_rate ?? 0} · ↑ ${t.tx_rate ?? 0} KB/s`), h('dt', {}, 'Heartbeat'), h('dd', {}, `${s.state?.heartbeat_hz ?? 0} Hz`),
          h('dt', {}, 'Link'), h('dd', {}, s.state?.gateway_detail || '—')));
    } else {
      const a = store.assigned.get(this.drone);
      content = a ? h('div', {}, h('div', { class: 'row', style: { marginBottom: '8px' } }, badge(a.label, 'violet'),
        h('button', { class: 'btn sm', onclick: () => download(`${this.drone}.xml`, a.xml, 'application/xml'), html: icon('download') + 'Download XML' })),
        h('textarea', { class: 'code', readonly: true, rows: 22 }, a.xml)) : emptyState('file', 'No tree for this drone', 'Generate one in step 3.');
    }
    fill(this.body, [content]);
  },
  /** Saved missions (GCS mission archive): mission time, results, folder. */
  archiveView() {
    if (Date.now() - (this.archiveAt || 0) > 4000) {
      this.archiveAt = Date.now();
      api('/api/archive').then(r => { this.archive = r; if (this.tab === 'archive') this.update(store, true); }).catch(() => {});
    }
    const r = this.archive, open = folder => api('/api/archive/open', folder ? { folder } : {}).catch(e => toast('Folder not opened', e.message, 'bad'));
    if (!r) return h('p', { class: 'faint' }, 'Loading the mission archive…');
    const cur = r.current || {}, rows = r.history || [];
    const head = h('div', { class: 'row', style: { marginBottom: '10px', flexWrap: 'wrap', gap: '8px' } },
      h('span', { class: 'faint', style: { fontSize: '12px' } }, `Every mission is saved in ${cur.root || '~/AERIXIS_Missions'} — Simulation/ and Real_Hardware/, one Mission_N folder per run, one folder per drone (track, geotags, events, uploaded tree${cur.copying ? '; copying simulation logs now…' : '; in simulation also the onboard and PX4 logs'}).`),
      h('button', { class: 'btn sm', onclick: () => open(null), html: icon('folder') + 'Open archive folder' }));
    const running = cur.running ? h('div', { class: 'archive-row' }, h('b', {}, `Mission ${cur.mission}`),
      h('span', {}, badge('RUNNING', 'warn'), ` ${cur.mode} · ${cur.drones.map(d => `${d.label} ${d.duration}`).join(' · ')}`),
      h('span', { class: 'mt-clock' }, cur.elapsed)) : null;
    const list = rows.map(m => h('div', { class: 'archive-row' },
      h('div', {}, h('b', {}, `Mission ${m.mission}`), h('div', { class: 'faint', style: { fontSize: '11px' } }, m.started)),
      h('div', {}, badge(m.mode === 'Simulation' ? 'SIM' : 'REAL', m.mode === 'Simulation' ? 'info' : 'warn'),
        ` ${m.geotags} geotags · ${m.packages_delivered} packages · `,
        ...m.drones.map(d => h('span', { class: 'mt-drone', style: { marginRight: '4px' } }, `${d.label} ${d.duration} `, badge(d.result || '—', d.result === 'SUCCESS' ? 'ok' : 'bad')))),
      h('div', { class: 'row', style: { gap: '8px' } }, h('span', { class: 'mt-clock', title: 'Mission time: first drone start to last drone end' }, m.mission_time),
        h('button', { class: 'btn sm', onclick: () => open(m.folder) }, 'Open folder'))));
    return h('div', {}, head, running, ...(list.length ? list : [running ? null : emptyState('logs', 'No saved missions yet', 'A mission is saved here when its drones finish.')]));
  },
};
