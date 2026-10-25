// Step 1 — Connect. One chain, top to bottom: open the GCS telemetry link (serial radio in the
// field, UDP in SITL), then register the aircraft that answer on that link.
import { h, icon, card, badge, hint, api, store, prefs, toast, fill, refreshPorts, linkUp, isFresh, rescueOf, fmtAge, confirmDialog } from './core.js';
import { archPanel } from './links.js';

const BAUDS = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600];

export const page = {
  id: 'connect', title: 'Connect', sub: 'Link & aircraft', ic: 'link',
  intro: 'Open the telemetry link first, then add the aircraft on it. Every aircraft shares the one link and is told apart by its MAVLink system ID.',
  mount(root) {
    this.root = root;
    // --- chain overview
    this.pipe = h('div', { class: 'pipeline' });
    // --- link
    this.modeSeg = h('div', { class: 'seg' },
      h('button', { 'data-mode': 'sim', onclick: () => this.setMode('sim') }, h('span', { html: icon('sim') }), 'Simulation (SITL)'),
      h('button', { 'data-mode': 'real', onclick: () => this.setMode('real') }, h('span', { html: icon('radio') }), 'Field radio'));
    this.port = h('select'); this.baud = h('select', {}, ...BAUDS.map(b => h('option', { value: b }, b)));
    this.baud.value = prefs.get('baud', 57600);
    this.udpHost = h('input', { value: prefs.get('udpHost', '127.0.0.1') });
    this.udpPort = h('input', { type: 'number', value: prefs.get('udpPort', 14555) });
    this.listen = h('input', { type: 'number', value: prefs.get('listenPort', 14551) });
    this.serialFields = h('div', { class: 'fields' },
      h('label', { class: 'field' }, 'Radio serial port', this.port),
      h('label', { class: 'field' }, 'Baud rate', this.baud, h('small', {}, 'Same on both radios')));
    this.udpFields = h('div', { class: 'fields' },
      h('label', { class: 'field' }, 'Onboard host', this.udpHost),
      h('label', { class: 'field' }, 'Onboard port', this.udpPort, h('small', {}, 'simulation: 14555')),
      h('label', { class: 'field' }, 'GCS listen port', this.listen, h('small', {}, 'default 14551')));
    this.connectBtn = h('button', { class: 'btn primary', onclick: () => this.connect() }, h('span', { html: icon('link') }), 'Open link');
    this.disconnectBtn = h('button', { class: 'btn', onclick: () => this.disconnect() }, 'Close');
    this.linkStatus = h('dl', { class: 'kv' });
    this.modeHelp = h('div');
    this.linkCard = card({ title: '1 · Telemetry link', sub: 'GCS ↔ onboard gateway', ic: 'radio', accent: 'violet',
      right: h('button', { class: 'btn sm ghost', title: 'Rescan serial ports', onclick: async () => { await refreshPorts(); this.fillPorts(); }, html: icon('refresh') }) },
      this.modeSeg, this.serialFields, this.udpFields, h('div', { class: 'row' }, this.connectBtn, this.disconnectBtn),
      h('div', { class: 'sep', style: { margin: '2px 12px' } }), this.linkStatus, this.modeHelp);
    for (const el of [this.serialFields, this.udpFields]) el.style.marginTop = '10px';
    // --- aircraft
    this.simBox = h('div');
    const f = this.form = {
      id: h('input', { placeholder: 'drone0', value: 'drone0' }), label: h('input', { placeholder: 'Rescue 1' }),
      sys: h('input', { type: 'number', min: 1, max: 254, value: 1 }), token: h('input', { placeholder: 'Onboard GUI → Pairing token', type: 'password' }) };
    this.addBtn = h('button', { class: 'btn', onclick: () => this.addAircraft() }, h('span', { html: icon('plus') }), 'Add');
    this.manual = h('details', { open: !store.fleet.length },
      h('summary', { class: 'kicker', style: { cursor: 'pointer', padding: '4px 0' } }, 'Add aircraft manually'),
      h('div', { class: 'fields', style: { marginTop: '6px' } },
        h('label', { class: 'field' }, 'Drone ID', f.id, h('small', {}, 'ROS namespace')), h('label', { class: 'field' }, 'Label', f.label),
        h('label', { class: 'field' }, 'System ID', f.sys, h('small', {}, 'PX4 MAV_SYS_ID')), h('label', { class: 'field', style: { gridColumn: 'span 2' } }, 'Pairing token', f.token)),
      h('div', { class: 'row', style: { marginTop: '8px' } }, this.addBtn));
    this.table = h('div');
    this.aircraftCard = card({ title: '2 · Aircraft on this link', sub: 'ID · system ID · onboard status', ic: 'drone', accent: 'teal' }, this.simBox, this.table, h('div', { class: 'sep' }), this.manual);
    // Per-aircraft architecture: every hop GCS -> gateway -> AERIXIS core -> XRCE-DDS -> PX4, vehicle, data.
    this.archTabs = h('div', { class: 'seg' }); this.archBox = h('div'); this.archPanels = new Map();
    this.archCard = h('div', { class: 'arch-wrap' }, h('div', { class: 'row', style: { margin: '18px 0 10px' } },
      h('h3', { class: 'section-title' }, 'System architecture — per aircraft'), this.archTabs), this.archBox);
    root.append(h('section', { class: 'card c-blue', style: { marginBottom: '14px' } }, h('div', { class: 'card-head' }, h('div', { class: 'card-icon', html: icon('wave') }), h('div', {}, h('h3', {}, 'Connection chain'), h('p', {}, 'fleet overview — each hop turns green once confirmed live'))), this.pipe),
      h('div', { class: 'cols-connect' }, this.linkCard, this.aircraftCard), this.archCard);
    this.fillPorts(); this.setMode(store.mode, true);
  },
  fillPorts() {
    const keep = this.port.value || prefs.get('port', '');
    this.port.replaceChildren(...(store.ports.length ? store.ports : [{ device: '/dev/ttyUSB0', description: 'not detected' }]).map(p =>
      h('option', { value: p.device }, `${p.device}${p.description && p.description !== 'n/a' ? ' — ' + p.description : ''}`)));
    if (keep) this.port.value = keep;
  },
  setMode(mode, silent) {
    store.mode = mode; prefs.set('mode', mode);
    this.modeSeg.querySelectorAll('button').forEach(b => b.classList.toggle('active', b.dataset.mode === mode));
    const sim = mode === 'sim';
    this.serialFields.style.display = sim ? 'none' : ''; this.udpFields.style.display = sim ? '' : 'none';
    fill(this.modeHelp, [sim
      ? hint(h('span', {}, 'Start the simulation with ', h('b', { class: 'mono' }, 'launch/sim.sh start N'), '. All N simulated aircraft answer on UDP 14555.'))
      : hint(h('span', {}, 'Plug in the GCS radio. On each Pi open the onboard GUI, connect its FC and GCS radio, and read its ', h('b', {}, 'pairing token'), '.'))]);
    this.simChecked = 0; this.update(store);
    if (!silent) toast(sim ? 'Simulation mode' : 'Field mode', sim ? 'Link over local UDP.' : 'Link over the serial telemetry radio.');
  },
  async connect() {
    const b = this.connectBtn; b.classList.add('busy');
    try {
      const body = store.mode === 'sim'
        ? { type: 'udp', host: this.udpHost.value.trim(), port: +this.udpPort.value, listen_port: +this.listen.value }
        : { type: 'serial', port: this.port.value, baud: +this.baud.value };
      prefs.set('udpHost', body.host || '127.0.0.1'); prefs.set('udpPort', +this.udpPort.value); prefs.set('listenPort', +this.listen.value);
      prefs.set('port', this.port.value); prefs.set('baud', +this.baud.value);
      await api('/api/connect', body);
      toast('Link opened', body.type === 'udp' ? `UDP ${body.host}:${body.port}, listening on ${body.listen_port}` : `${body.port} @ ${body.baud} baud`, 'ok');
    } catch (e) { toast('Could not open link', e.message, 'bad', 9000); } finally { b.classList.remove('busy'); }
  },
  async disconnect() { try { await api('/api/disconnect', {}); toast('Link closed'); } catch (e) { toast('Close failed', e.message, 'bad'); } },
  async addAircraft() {
    const f = this.form, b = this.addBtn; b.classList.add('busy');
    try {
      await api('/api/fleet/profile', { drone_id: f.id.value.trim(), label: f.label.value.trim() || f.id.value.trim(), transport: 'mavlink', system_id: +f.sys.value, token: f.token.value.trim() });
      toast('Aircraft added', `${f.id.value} · system ID ${f.sys.value}`, 'ok'); f.token.value = '';
    } catch (e) { toast('Could not add aircraft', e.message, 'bad'); } finally { b.classList.remove('busy'); }
  },
  async remove(d) {
    if (!(await confirmDialog(`Remove ${d.label}?`, 'The GCS forgets this aircraft and its token. Nothing is sent to the aircraft.', { ok: 'Remove', kind: 'danger' }))) return;
    try { await api('/api/fleet/remove', { drone_id: d.drone_id }); toast(`${d.label} removed`); } catch (e) { toast('Remove failed', e.message, 'bad'); }
  },
  refreshSim() {
    // Simulated aircraft are added exactly like real ones (ID, system ID, pairing token): the GCS
    // never reads the onboard computer's files, so the same steps work with a Pi over the radio.
    if (store.mode !== 'sim') { fill(this.simBox, []); this.simKey = null; return; }
    if (this.simKey === 'hint') return; this.simKey = 'hint';
    fill(this.simBox, [h('div', { class: 'row', style: { marginBottom: '8px', flexWrap: 'wrap' } }, h('span', { class: 'kicker' }, 'SITL'),
      h('span', { class: 'faint' }, 'Add each simulated drone below with its ID, system ID and token — print them with ',
        h('b', { class: 'mono' }, 'launch/sim.sh pairing'), ' on the simulation computer.'))]);
  },
  update(s) {
    const st = s.state || {}, t = st.traffic || {}, conn = st.connection || {}, up = linkUp(), drones = s.fleet;
    // Radio capacity: 10 bits per byte on a UART; UDP has none worth showing.
    const capacity = conn.type === 'serial' && conn.baud ? conn.baud / 10 / 1024 : null;
    const load = capacity ? Math.round(100 * ((t.rx_rate || 0) + (t.tx_rate || 0)) / capacity) : null;
    fill(this.linkStatus, [
      h('dt', {}, 'State'), h('dd', {}, h('span', { class: 'row', style: { justifyContent: 'flex-end', gap: '6px' } }, h('span', { class: `dot ${up ? 'ok' : s.stateOk ? 'warn' : 'bad'}` }), up ? 'heartbeat OK' : s.stateOk ? 'no heartbeat' : 'GCS server offline')),
      h('dt', {}, 'Transport'), h('dd', {}, conn.type ? `${conn.type.toUpperCase()} ${conn.target || ''}${conn.baud ? ' @ ' + conn.baud : ''}${conn.listen_port ? ' · rx :' + conn.listen_port : ''}` : 'closed'),
      h('dt', {}, 'Heartbeat'), h('dd', {}, `${st.heartbeat_hz ?? 0} Hz · ${st.heartbeat_age ?? '—'} s`),
      h('dt', {}, 'Traffic ↓ / ↑'), h('dd', {}, `${t.rx_rate ?? 0} / ${t.tx_rate ?? 0} KB/s`),
      capacity ? h('dt', {}, 'Radio load') : null, capacity ? h('dd', { style: { color: load > 80 ? 'var(--bad)' : load > 60 ? 'var(--warn)' : '' } }, `${load}% of ${capacity.toFixed(1)} KB/s`) : null]);
    const anyFresh = drones.some(isFresh), anyRos = drones.some(d => (d.data?.ros?.count || 0) > 0), anyPx4 = drones.some(d => rescueOf(d)?.vehicle?.connected);
    const nodes = [
      ['GCS server', s.stateOk ? 'running' : 'offline', s.stateOk ? 'on' : 'off'],
      [store.mode === 'sim' ? 'UDP link' : 'Telemetry radio', up ? `${st.heartbeat_hz} Hz heartbeat` : 'no heartbeat', up ? 'on' : s.stateOk ? 'half' : ''],
      ['Mission managers', drones.length ? `${drones.filter(isFresh).length}/${drones.length} answering` : 'no aircraft', anyFresh ? (drones.every(isFresh) ? 'on' : 'half') : drones.length ? 'off' : ''],
      ['AERIXIS core', anyRos ? `${drones.filter(d => (d.data?.ros?.count || 0) > 5).length}/${drones.length} up` : '—', anyRos ? 'on' : ''],
      ['PX4', anyPx4 ? `${drones.filter(d => rescueOf(d)?.vehicle?.connected).length}/${drones.length} connected` : '—', anyPx4 ? 'on' : ''],
    ];
    fill(this.pipe, nodes.map(([b, small, cls]) => h('div', { class: `pnode ${cls}` }, h('b', {}, b), h('small', {}, small))));
    // Rows are kept per aircraft and only their cells refreshed: the remove button stays put.
    this.rows ||= new Map();
    const rows = drones.map(d => {
      const fresh = isFresh(d), r = rescueOf(d), id = d.drone_id;
      let row = this.rows.get(id);
      if (!row) {
        row = h('tr'); row._cells = [0, 1, 2, 3, 4, 5].map(() => h('td'));
        row._remove = h('button', { class: 'btn sm ghost', title: 'Remove from GCS', onclick: () => this.remove(store.drone(id)) }, '✕');
        row.append(...row._cells, h('td', {}, row._remove)); this.rows.set(id, row);
      }
      const c = row._cells;
      fill(c[0], [h('b', {}, d.label), ' ', h('span', { class: 'faint mono' }, id), d.error ? h('div', { style: { color: 'var(--bad)', fontSize: '11px' } }, d.error) : null]);
      c[1].className = 'mono'; c[1].textContent = String(d.endpoint || '').replace('mavlink://', '');
      fill(c[2], [badge(fresh ? 'CONNECTED' : d.connection, fresh ? 'ok' : d.connection === 'ERROR' || d.connection === 'DISCONNECTED' ? 'bad' : 'warn')]);
      c[3].className = 'mono'; c[3].textContent = d.data?.ros?.count ?? '—';
      fill(c[4], [r?.vehicle ? badge(r.vehicle.armed ? 'ARMED' : r.vehicle.connected ? 'ok' : 'no FC', r.vehicle.armed ? 'warn' : r.vehicle.connected ? 'ok' : 'bad') : '—']);
      c[5].className = 'mono faint'; c[5].textContent = fmtAge(d.age_s);
      row._remove.disabled = !!d.data?.active;
      return row;
    });
    for (const id of [...this.rows.keys()]) if (!drones.some(d => d.drone_id === id)) this.rows.delete(id);
    if (!this.tableEl) { this.tbody = h('tbody'); this.tableEl = h('table', { class: 'table' }, h('thead', {}, h('tr', {}, ...['Aircraft', 'SYS', 'Onboard', 'Core', 'PX4', 'Age', ''].map(x => h('th', {}, x)))), this.tbody); }
    fill(this.tbody, rows);
    fill(this.table, [drones.length ? this.tableEl : (this.noneEl ||= h('p', { class: 'muted', style: { margin: '4px 0' } }, 'No aircraft yet — add simulated aircraft above, or one with its pairing token below.'))]);
    this.refreshSim();
    // architecture panel for the selected aircraft
    if (!drones.some(d => d.drone_id === this.archSel)) this.archSel = drones[0]?.drone_id;
    const tabKey = drones.map(d => d.drone_id + isFresh(d)).join() + this.archSel;
    if (tabKey !== this.archTabKey) {
      this.archTabKey = tabKey;
      fill(this.archTabs, drones.map(d => h('button', { class: d.drone_id === this.archSel ? 'active' : '', onclick: () => { this.archSel = d.drone_id; this.update(store); } },
        h('span', { class: `dot ${isFresh(d) ? 'ok' : 'bad'}` }), d.label)));
    }
    const sel = store.drone(this.archSel);
    if (sel) { const el = archPanel(sel, this.archPanels.get(sel.drone_id)); this.archPanels.set(sel.drone_id, el); fill(this.archBox, [el]); }
    else fill(this.archBox, [h('p', { class: 'faint' }, 'Add an aircraft to see its link chain.')]);
    this.done = up && anyFresh;
  },
  leaveWarnings(s) {
    const out = [];
    if (!s.stateOk) out.push('The GCS server is not responding.');
    else if (!linkUp()) out.push('Telemetry link has no heartbeat — open the link first.');
    if (!s.fleet.length) out.push('No aircraft added yet.');
    for (const d of s.fleet) if (!isFresh(d)) out.push(`${d.label}: onboard status not answering (${d.error || d.connection}).`);
    return out;
  },
};
