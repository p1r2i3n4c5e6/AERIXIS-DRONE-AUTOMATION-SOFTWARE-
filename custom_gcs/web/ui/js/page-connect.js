// Step 1 — Connect. One chain, top to bottom: open the GCS telemetry link (serial radio in the
// field, UDP in SITL), then register the aircraft that answer on that link.
import { h, icon, card, badge, hint, api, store, prefs, toast, fill, refreshPorts, linkUp, isFresh, rescueOf, fmtAge, confirmDialog } from './core.js';
import { archPanel } from './links.js';
import { sim, refreshSim, watchSim } from './simarena.js';
import { createSimOverlay } from './simoverlay.js';
import { gcsPosition } from './mapview.js';

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
    // Simulation: the arena panel on top; the UDP fields are technical, so they fold away.
    this.simHost = h('div');
    this.udpBox = h('details', { class: 'link-details' }, h('summary', { class: 'kicker' }, 'Link settings (UDP)'), this.udpFields);
    this.linkCard = card({ title: '1 · Telemetry link', sub: 'GCS ↔ onboard gateway', ic: 'radio', accent: 'violet',
      right: h('button', { class: 'btn sm ghost', title: 'Rescan serial ports', onclick: async () => { await refreshPorts(); this.fillPorts(); }, html: icon('refresh') }) },
      this.modeSeg, this.simHost, this.serialFields, this.udpBox, h('div', { class: 'row' }, this.connectBtn, this.disconnectBtn),
      h('div', { class: 'sep', style: { margin: '2px 12px' } }), this.linkStatus, this.modeHelp);
    for (const el of [this.serialFields, this.udpBox]) el.style.marginTop = '10px';
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
    this.manualSep = h('div', { class: 'sep' });
    this.aircraftCard = card({ title: '2 · Aircraft on this link', sub: 'ID · system ID · onboard status', ic: 'drone', accent: 'teal' }, this.simBox, this.table, this.manualSep, this.manual);
    // Per-aircraft architecture: every hop GCS -> gateway -> AERIXIS core -> XRCE-DDS -> PX4, vehicle, data.
    this.archTabs = h('div', { class: 'seg' }); this.archBox = h('div'); this.archPanels = new Map();
    this.archCard = h('div', { class: 'arch-wrap' }, h('div', { class: 'row', style: { margin: '18px 0 10px' } },
      h('h3', { class: 'section-title' }, 'System architecture — per aircraft'), this.archTabs), this.archBox);
    root.append(h('section', { class: 'card c-blue', style: { marginBottom: '14px' } }, h('div', { class: 'card-head' }, h('div', { class: 'card-icon', html: icon('wave') }), h('div', {}, h('h3', {}, 'Connection chain'), h('p', {}, 'fleet overview — each hop turns green once confirmed live'))), this.pipe),
      h('div', { class: 'cols-connect' }, this.linkCard, this.aircraftCard), this.archCard);
    this.buildSimPanel();
    this.fillPorts(); this.setMode(store.mode, true);
  },
  // ---------- Simulation arena (Simulation mode only, started from this GCS computer) ----------
  buildSimPanel() {
    const saved = prefs.get('simArena', {});
    this.simN = h('select', {}, ...[1, 2, 3, 4].map(n => h('option', { value: n, selected: (saved.drones || 1) === n }, n)));
    this.simLat = h('input', { type: 'number', step: '0.000001', value: saved.lat ?? '', placeholder: 'latitude' });
    this.simLon = h('input', { type: 'number', step: '0.000001', value: saved.lon ?? '', placeholder: 'longitude' });
    this.sim3d = h('input', { type: 'checkbox', checked: saved.show3d ?? true });
    this.simCam = h('input', { type: 'checkbox', checked: saved.cameraOnMission ?? true });
    const setLoc = (lat, lon, what) => { this.simLat.value = (+lat).toFixed(6); this.simLon.value = (+lon).toFixed(6); toast('Arena location', `${what}: ${(+lat).toFixed(5)}, ${(+lon).toFixed(5)}`); };
    this.simStart = h('button', { class: 'btn primary', onclick: async () => { if (await this.startSim(!sim.status?.components?.px4)) this.overlay.open(true); } }, h('span', { html: icon('play') }), 'Start simulation');
    this.simAddBtn = h('button', { class: 'btn', title: 'One more drone in the same arena (same Gazebo world), up to 4', onclick: () => this.overlay.add() }, h('span', { html: icon('plus') }), 'Add drone');
    this.sim3dBtn = h('button', { class: 'btn', title: 'Open the Gazebo 3D window of the running arena', onclick: () => this.openGui() }, h('span', { html: icon('grid') }), '3D window');
    this.simStop = h('button', { class: 'btn', onclick: () => this.stopSim() }, h('span', { html: icon('stop') }), 'Stop');
    this.simCamBtn = h('button', { class: 'btn', title: 'Open the window with every simulated drone camera', onclick: () => this.openCameras(true) }, h('span', { html: icon('camera') }), 'Camera window');
    this.simState = h('div', { class: 'sim-state' });
    this.simLog = h('pre', { class: 'sim-log' });
    this.simPanel = h('div', { class: 'sim-panel' },
      h('div', { class: 'sim-title' }, h('b', {}, 'Simulation arena'), h('span', { class: 'faint' }, '300 × 300 m · 50 people · 180 trees')),
      h('div', { class: 'fields', style: { marginTop: '8px' } },
        h('label', { class: 'field sim-n' }, 'Drones', this.simN, h('small', {}, '1 to 4, 20 m apart in the arena centre')),
        h('label', { class: 'field' }, 'Latitude', this.simLat), h('label', { class: 'field' }, 'Longitude', this.simLon)),
      h('div', { class: 'row', style: { marginTop: '6px', flexWrap: 'wrap' } },
        h('span', { class: 'faint', style: { fontSize: '11.5px' } }, 'Arena centre:'),
        h('button', { class: 'btn sm', onclick: () => { const c = prefs.get('mapCenter', null); c ? setLoc(c[0], c[1], 'Map centre') : toast('No map yet', 'Open the Plan or Operations map and move it to your area first.', 'warn'); } }, h('span', { html: icon('map') }), 'Map centre'),
        h('button', { class: 'btn sm', onclick: () => { const f = gcsPosition.fix; f ? setLoc(f.lat, f.lon, 'Your location') : toast('Location unknown', 'Allow location in the browser, or use the map centre.', 'warn'); } }, h('span', { html: icon('gps') }), 'My location')),
      h('div', { class: 'row', style: { marginTop: '8px', flexWrap: 'wrap', gap: '14px' } },
        h('label', { class: 'check' }, this.sim3d, h('span', {}, '3D world window (Gazebo)')),
        h('label', { class: 'check' }, this.simCam, h('span', {}, 'Camera window when a camera mission starts'))),
      h('div', { class: 'row', style: { marginTop: '10px', flexWrap: 'wrap' } }, this.simStart, this.simAddBtn, this.sim3dBtn, this.simCamBtn, this.simStop),
      this.simState,
      h('details', {}, h('summary', { class: 'kicker', style: { cursor: 'pointer', marginTop: '6px' } }, 'Launcher output'), this.simLog));
    sim.listeners.add(() => this.renderSim());
    // The big start window (setup -> live stages -> ready); the work is done by this page.
    this.overlay = createSimOverlay({
      values: () => this.simValues(),
      setDrones: n => { this.simN.value = n; this.simValues(); },
      location: () => this.simLocation(true),
      useMyLocation: () => { const f = gcsPosition.fix; if (f) setLoc(f.lat, f.lon, 'Your location'); else toast('Location unknown', 'Allow location for this page in the browser, or use the map centre.', 'warn'); },
      useMapCentre: () => { const c = prefs.get('mapCenter', null); if (c) setLoc(c[0], c[1], 'Map centre'); else toast('No map position yet', 'Open the Plan or Operations map once, or type latitude/longitude in the Simulation panel.', 'warn'); },
      checkbox3d: () => h('input', { type: 'checkbox', checked: this.sim3d.checked, onchange: e => { this.sim3d.checked = e.target.checked; this.simValues(); } }),
      start: () => this.startSim(true),
      stop: () => this.stopSim(),
      add: () => this.addDrone(),
      gui: () => this.openGui(),
    });
  },
  simValues() {
    const lat = parseFloat(this.simLat.value), lon = parseFloat(this.simLon.value);
    const v = { drones: +this.simN.value, lat, lon, show3d: this.sim3d.checked, cameraOnMission: this.simCam.checked };
    prefs.set('simArena', v);
    return v;
  },
  /** Arena centre: typed, else this laptop's location, else the map centre, else the arena's own
   *  default (PX4's test field), so Start never fails for want of a location. */
  simLocation(withSource) {
    let lat = parseFloat(this.simLat.value), lon = parseFloat(this.simLon.value), source = 'chosen in the Simulation panel';
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) {
      const f = gcsPosition.fix, c = prefs.get('mapCenter', null);
      if (f) { [lat, lon, source] = [f.lat, f.lon, 'your location (this laptop)']; }
      else if (c) { [lat, lon, source] = [c[0], c[1], 'where the map is looking']; }
      else { [lat, lon, source] = [47.397742, 8.545594, 'arena default (no location yet: press My location or Map centre)']; }
    }
    return withSource ? { lat: +lat, lon: +lon, source } : [+lat, +lon];
  },
  async startSim(auto) {
    const [lat, lon] = this.simLocation(), v = this.simValues();
    const running = sim.status?.components?.px4;
    if (!auto && running && !(await confirmDialog('Restart the simulation?', `The running simulation stops and starts again with ${v.drones} drone(s) at ${lat.toFixed(5)}, ${lon.toFixed(5)}.`, { ok: 'Restart' }))) return false;
    try {
      // The camera window opens later, when a mission that uses the camera starts (Upload & start).
      await api('/api/sim/start', { drones: v.drones, home: [lat, lon, 0], show3d: v.show3d, cameras: false });
      this.simWasReady = false; this.simPaired = false; this.overlay?.setDrones(v.drones);
      if (!this.overlay?.isOpen) toast('Starting the simulation', `${v.drones} drone(s) at ${lat.toFixed(4)}, ${lon.toFixed(4)} — about 1–3 minutes.`, 'ok', 8000);
      refreshSim();
      return true;
    } catch (e) { toast('Simulation not started', e.message, 'bad', 9000); return false; }
  },
  async stopSim() {
    if (!(await confirmDialog('Stop the simulation?', 'PX4, Gazebo, the simulated onboard software and the camera window are closed.', { ok: 'Stop', kind: 'danger' }))) return false;
    try { await api('/api/sim/stop', {}); toast('Stopping the simulation'); refreshSim(); return true; } catch (e) { toast('Stop failed', e.message, 'bad'); return false; }
  },
  async pairSim(quiet) {
    try { const r = await api('/api/sim/pair', {}); this.simPaired = true;
      if (!quiet) toast('Simulated drones added', `${r.added.join(', ')} — in the simulation aircraft list only.`, 'ok');
      if (!linkUp()) this.connect(); }
    catch (e) { if (!quiet) toast('Simulated drones not added', e.message, 'warn', 9000); }
  },
  async addDrone() {
    try {
      const r = await api('/api/sim/add', {});
      this.simN.value = r.drones; this.simValues(); this.overlay.setDrones(r.drones);
      toast(`Adding drone ${r.drones}`, 'It joins the same arena and Gazebo window; the others wait on the ground.', 'ok');
      refreshSim(); return true;
    } catch (e) { toast('Drone not added', e.message, 'warn', 9000); return false; }
  },
  async openGui() {
    try { await api('/api/sim/gui', {}); toast('Gazebo 3D window', 'Opening (it can take a few seconds).', 'ok'); }
    catch (e) { toast('3D window', e.message, 'warn'); }
  },
  async openCameras(manual) {
    try { const r = await api('/api/sim/cameras', {}); if (manual || r.result === 'opened') toast('Drone camera window', r.result === 'opened' ? 'Opening in its own window.' : 'Already open.', 'ok'); }
    catch (e) { if (manual) toast('Camera window', e.message, 'warn'); }
  },
  renderSim() {
    if (!this.simPanel) return;
    const st = sim.status, comps = st?.components || {}, state = st?.state || (sim.error ? 'unavailable' : '…');
    const busy = state === 'starting' || state === 'stopping' || state === 'adding';
    this.simStart.disabled = busy; this.simStop.disabled = busy || !(comps.px4 || state === 'failed'); this.simCamBtn.disabled = !comps.px4;
    this.simStart.lastChild.textContent = comps.px4 ? 'Restart' : 'Start simulation';
    this.simAddBtn.disabled = busy || state === 'adding' || !comps['sitl-onboard'] || +(st?.settings?.drones || 1) >= 4;
    this.sim3dBtn.disabled = !comps.px4 || !!comps['gazebo-gui'];
    const kind = state === 'running' ? 'ok' : state === 'failed' || state === 'unavailable' ? 'bad' : busy ? 'warn' : '';
    const parts = comps.px4 ? [`PX4 + Gazebo ${comps.px4 ? 'on' : 'off'}`, `onboard ${comps['sitl-onboard'] ? 'on' : 'off'}`, `cameras ${comps['sim-cameras'] ? 'open' : 'closed'}`] : [];
    fill(this.simState, [h('div', { class: 'row', style: { marginTop: '8px', gap: '8px', flexWrap: 'wrap' } },
      h('span', { class: `dot ${kind}` }), h('b', {}, { running: 'Running', starting: 'Starting…', adding: 'Adding a drone…', stopping: 'Stopping…', failed: 'Failed', idle: 'Not running' }[state] || state),
      h('span', { class: 'muted', style: { fontSize: '12px' } }, sim.error || st?.message || ''),
      parts.length ? h('span', { class: 'faint', style: { fontSize: '11.5px' } }, parts.join(' · ')) : null),
      comps.px4 ? hint(h('span', {}, 'Next: Plan → ', h('b', {}, 'Use arena as search area'), ', pick a mission, then Upload & start. The camera window opens with a camera mission.')) : null]);
    this.simLog.textContent = (st?.log || []).join('\n');
    // Ready (or found running after a GCS restart): add the simulated drones to the simulation
    // list and open the UDP link, once, without any typing.
    const ready = state === 'running' && !!comps['sitl-onboard'];
    // Another GCS window switched the server to Field mode: never touch the link from here.
    const fieldElsewhere = store.state?.link_mode === 'real';
    if (fieldElsewhere && store.mode === 'sim') this.simState.append(hint('The GCS link is in Field mode (another GCS window chose Field radio). Click Simulation (SITL) here to use the simulation again.', 'warn'));
    if (ready && store.mode === 'sim' && !fieldElsewhere) {
      const ids = new Set(store.fleet.map(d => d.drone_id));
      const want = +(st?.settings?.drones || 0);
      const wanted = new Set([...Array(want).keys()].map(i => 'drone' + i));
      const missing = !want || [...wanted].some(id => !ids.has(id)) || [...ids].some(id => !wanted.has(id));
      if (missing && !this.simPairing && Date.now() - (this.simPairedAt || 0) > 10000) { this.simPairedAt = Date.now(); this.simPairing = true; this.pairSim(true).finally(() => { this.simPairing = false; }); }
      else {
        // The simulated radio is UDP 14555. A link on 14550 (PX4's own port, the server default)
        // shows PX4's heartbeat and looks fine, but no drone answers there.
        const conn = store.state?.connection || {};
        const wrong = conn.type !== 'udp' || !/:14555$/.test(String(conn.target || ''));
        if ((wrong || !linkUp()) && !this.simLinking) {
          this.simLinking = true; this.udpHost.value = '127.0.0.1'; this.udpPort.value = 14555;
          this.connect().finally(() => { setTimeout(() => { this.simLinking = false; }, 8000); });
        }
      }
    }
    this.simWasReady = ready;
  },
  fillPorts() {
    const keep = this.port.value || prefs.get('port', '');
    this.port.replaceChildren(...(store.ports.length ? store.ports : [{ device: '/dev/ttyUSB0', description: 'not detected' }]).map(p =>
      h('option', { value: p.device }, `${p.device}${p.description && p.description !== 'n/a' ? ' — ' + p.description : ''}`)));
    if (keep) this.port.value = keep;
  },
  setMode(mode, silent) {
    store.mode = mode; prefs.set('mode', mode);
    // Simulation and Field have separate aircraft lists AND links on the GCS server. Only a click
    // changes the server's mode; a page that loads adopts the server's mode (see update()), so an old
    // tab can never close the field radio link.
    if (!silent) api('/api/fleet/use', { mode }).then(r => {
      if (r.changed) toast(mode === 'sim' ? 'Simulation aircraft' : 'Field aircraft', `Using the ${mode === 'sim' ? 'simulation' : 'field'} aircraft list.`);
      if (r.link_closed) toast('Link closed', `${mode === 'sim' ? 'Simulation' : 'Field'} mode closed the ${r.link_closed}.`, 'warn', 8000);
    }).catch(e => toast('Mode not switched on the GCS', e.message, 'warn', 9000));
    this.modeSeg.querySelectorAll('button').forEach(b => b.classList.toggle('active', b.dataset.mode === mode));
    const sim = mode === 'sim';
    this.serialFields.style.display = sim ? 'none' : ''; this.udpBox.style.display = sim ? '' : 'none';
    // Simulated drones are added by the simulation itself: no ID / token form in Simulation.
    this.manual.style.display = this.manualSep.style.display = sim ? 'none' : '';
    if (sim && +this.udpPort.value !== 14555) { this.udpPort.value = 14555; this.udpHost.value = '127.0.0.1'; }
    fill(this.simHost, sim ? [this.simPanel] : []);
    fill(this.modeHelp, sim ? [] : [hint(h('span', {}, 'Plug in the GCS radio. On each Pi open the onboard GUI, connect its FC and GCS radio, and read its ', h('b', {}, 'pairing token'), '.'))]);
    this.simChecked = 0; this.update(store);
    if (sim) { watchSim(); if (!silent) refreshSim().then(async () => {
      const c = sim.status?.components || {}, state = sim.status?.state;
      if (!c.px4 && state !== 'starting' && state !== 'stopping') { if (await this.startSim(true)) this.overlay.open(true); }
      else this.overlay.open();
    }); }
    if (!silent) toast(sim ? 'Simulation mode' : 'Field mode', sim ? 'Simulated drones on this laptop.' : 'Link over the serial telemetry radio.');
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
    fill(this.simBox, []); this.simKey = null;
  },
  update(s) {
    // Follow the GCS server's mode (set by the last click in any GCS window, or start_field.sh).
    // A server that has no mode yet takes this page's saved one.
    const serverMode = s.state?.link_mode;
    if (s.stateOk && serverMode === undefined && !this.modeClaimed) { this.modeClaimed = true; api('/api/fleet/use', { mode: store.mode }).catch(() => {}); }
    else if (serverMode && serverMode !== store.mode && this.root) { this.setMode(serverMode, true); return; }
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
    const none = store.mode === 'sim' ? 'The simulated drones appear here by themselves once the simulation is ready.' : 'No aircraft yet — add one with its pairing token below.';
    if (!this.noneEl) this.noneEl = h('p', { class: 'muted', style: { margin: '4px 0' } });
    this.noneEl.textContent = none;
    fill(this.table, [drones.length ? this.tableEl : this.noneEl]);
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
