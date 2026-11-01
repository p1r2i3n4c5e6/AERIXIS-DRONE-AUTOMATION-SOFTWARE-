// Step 4 — Upload & start: send each drone its tree, validate onboard, then start it.
import { h, icon, card, badge, hint, api, store, toast, fill, sha256, confirmDialog, emptyState, isFresh, rescueOf, radioFrames, alertBox , prefs, payloadOf } from './core.js';
import { parseXml, treeViewer, countNodes, loadCatalog } from './tree.js';
import { readyToFly } from './page-readiness.js';

export const page = {
  id: 'upload', title: 'Upload & start', sub: 'Send trees onboard', ic: 'upload',
  intro: 'Each drone validates its tree onboard before it may start. Start is only offered when the onboard manager says the drone can fly.',
  mount(root) {
    this.root = root; this.cards = new Map(); this.busy = new Set(); this.started = new Set();  // started from here; status may lag
    this.uploadAll = h('button', { class: 'btn primary', onclick: () => this.all('stage') }, h('span', { html: icon('upload') }), 'Upload all');
    this.startAll = h('button', { class: 'btn go', onclick: () => this.all('start') }, h('span', { html: icon('play') }), 'Start all');
    this.top = card({ title: 'Fleet actions', sub: 'or work aircraft by aircraft below', ic: 'bolt', accent: 'amber', cls: 'span-all',
      right: h('div', { class: 'row' }, this.uploadAll, this.startAll) }, this.topInfo = h('div'));
    this.grid = h('div', { class: 'grid g2' });
    root.append(h('div', { class: 'grid' }, this.top, this.grid));
    loadCatalog();
  },
  cardFor(d, i) {
    if (this.cards.has(d.drone_id)) return this.cards.get(d.drone_id);
    const ui = {};
    ui.source = h('div', { class: 'row' });
    ui.file = h('input', { type: 'file', accept: '.xml', style: { display: 'none' }, onchange: async e => {
      // Hand-written files lose comments and indentation: same tree, fewer radio frames.
      const f = e.target.files[0]; if (!f) return;
      const xml = (await f.text()).replace(/<!--[\s\S]*?-->/g, '').replace(/>\s+</g, '><').trim();
      store.assigned.set(d.drone_id, { xml, label: f.name, source: 'file' }); store.saveAssigned(); e.target.value = ''; this.update(store); } });
    ui.treeBox = h('div'); ui.tree = null;
    ui.payloadCheck = h('input', { type: 'checkbox' }); ui.payloadId = h('input', { placeholder: 'Package ID (new for every load)', title: 'A new ID for every load: the drone refuses an ID used in an earlier mission' });
    // Several packages per drone (2026-11-01): how many are loaded, up to what the drone reports.
    ui.payloadCount = h('input', { type: 'number', min: 1, max: 20, step: 1, value: 1, style: { width: '80px' } });
    ui.payloadCap = h('small', { class: 'faint' });
    ui.payload = h('div', { style: { marginTop: '12px', display: 'none' } },
      hint('This tree releases payloads. Confirm the packages are physically loaded — the servo command does not prove delivery.', 'warn'),
      h('div', { class: 'row', style: { marginTop: '8px' } }, h('label', { class: 'check' }, ui.payloadCheck, 'Packages loaded'),
        h('label', { class: 'field' }, 'How many', ui.payloadCount), h('label', { class: 'field', style: { flex: 1 } }, ui.payloadId)), ui.payloadCap);
    ui.stepper = h('div', { class: 'stepper' });
    ui.status = h('div', { class: 'kv', style: { marginTop: '10px' } });
    ui.stage = h('button', { class: 'btn primary', onclick: () => this.act(d.drone_id, 'stage') }, h('span', { html: icon('upload') }), 'Upload & validate');
    ui.start = h('button', { class: 'btn go', onclick: () => this.act(d.drone_id, 'start') }, h('span', { html: icon('play') }), 'Start');
    ui.cancel = h('button', { class: 'btn danger', onclick: () => this.act(d.drone_id, 'cancel') }, h('span', { html: icon('stop') }), 'Cancel');
    ui.reconcile = h('button', { class: 'btn', onclick: () => this.act(d.drone_id, 'reconcile'), html: icon('reconcile') + 'Reconcile' });
    ui.progress = h('div', { class: 'progress indeterminate', style: { display: 'none', marginTop: '10px' } }, h('span'));
    ui.card = card({ title: d.label, sub: d.drone_id, ic: 'drone', accent: ['blue', 'pink', 'teal', 'violet', 'green', 'coral'][i % 6],
      right: h('div', { class: 'row' }, ui.badge = badge('…'), h('button', { class: 'btn sm', onclick: () => ui.file.click(), html: icon('file') + 'XML file' }), ui.file) },
      ui.stepper, ui.source, h('div', { style: { marginTop: '10px' } }, ui.treeBox), ui.payload, ui.status,
      h('div', { class: 'row', style: { marginTop: '14px' } }, ui.stage, ui.start, ui.cancel, ui.reconcile), ui.progress);
    ui.viewer = treeViewer(ui.treeBox, { collapsedDepth: 3 }); ui.treeBox.classList.add('small');
    this.cards.set(d.drone_id, ui);
    return ui;
  },
  async act(id, action) {
    const d = store.drone(id), ui = this.cards.get(id); if (!d || this.busy.has(id)) return;
    const data = d.data || {}, body = { drone_id: id, action };
    try {
      if (action === 'stage') {
        const a = store.assigned.get(id); if (!a) throw new Error('No tree for this drone yet — plan one in step 3 or choose an XML file.');
        body.xml = a.xml; body.sha256 = await sha256(a.xml);
        if (a.bundle) body.route_bundle = a.bundle;
        // The mission's RTL/LAND choice for the onboard FC guardian (handed over at start).
        if (a.emergency_action) body.emergency_action = a.emergency_action;
        if (ui.payloadVisible) {
          if (!ui.payloadCheck.checked || !ui.payloadId.value.trim()) throw new Error('Confirm the loaded packages and enter an ID first.');
          const count = Math.round(+ui.payloadCount.value), cap = payloadOf(d)?.capacity;
          if (!(count >= 1 && count <= 20)) throw new Error('Packages loaded: 1 to 20.');
          if (cap != null && count > cap) throw new Error(`${d.label} has servo steps for ${cap} package${cap === 1 ? '' : 's'} (set on the drone's control panel).`);
          body.operator_payload = { confirmed_loaded: true, payload_id: ui.payloadId.value.trim(), count };
        }
      } else if (action === 'start') {
        if (!(await confirmDialog(`Start ${d.label}?`, 'The drone will arm, take off and fly its mission autonomously. Make sure the area is clear.', { ok: 'Start mission', kind: 'go' }))) return;
        Object.assign(body, { mission_id: data.mission.id, revision: data.mission.revision, command_id: crypto.randomUUID() });
      } else if (action === 'cancel') {
        Object.assign(body, { mission_id: data.execution.mission_id, revision: data.execution.revision });
      }
      this.busy.add(id); ui.progress.style.display = ''; this.update(store);
      await api('/api/fleet/command', body);
      toast(`${d.label}: ${{ stage: 'tree validated onboard', start: 'mission started', cancel: 'cancel requested', reconcile: 'ground state verified' }[action]}`, action === 'start' ? 'Follow it in Operations (step 5).' : '', 'ok');
      if (action === 'start') { this.started.add(id); this.simCameras([id]); setTimeout(() => { location.hash = '#live'; }, 800); }
    } catch (e) { toast(`${d.label}: ${action} failed`, e.message, 'bad', 10000); }
    finally { this.busy.delete(id); ui.progress.style.display = 'none'; this.update(store); }
  },
  /** Why an aircraft cannot start, in operator words ('' = it can). */
  blocker(d) {
    const data = d.data || {}, ui = this.cards.get(d.drone_id), m = data.mission;
    if (!isFresh(d)) return `no onboard status (${d.error || d.connection})`;
    if (data.active) return 'already executing';
    if (!store.assigned.get(d.drone_id)) return 'no tree assigned';
    if (!(m && ui?.assignedSha && m.sha256 === ui.assignedSha)) return 'tree not uploaded & validated';
    if (data.hold_lockout) return 'ground lockout after the last flight — reconcile on the ground';
    if (rescueOf(d)?.flags?.AbortRequested) return `abort latched (${rescueOf(d).flags.reason || 'reason not reported'}) — reconcile on the ground`;
    if (!data.flight_enabled) return 'flight not allowed by the onboard manager (observation only)';
    if (!data.can_start) return 'not on the ground / not disarmed and still';
    return '';
  },
  async all(action) {
    const ids = store.fleet.map(d => d.drone_id).filter(id => {
      const d = store.drone(id); return action === 'stage' ? store.assigned.has(id) : d?.data?.can_start; });
    if (action === 'start') {
      const blocked = store.fleet.filter(d => !d.data?.can_start).map(d => [d, this.blocker(d)]).filter(([, why]) => why);
      if (blocked.length) {
        const lock = blocked.filter(([, why]) => why.includes('reconcile')).map(([d]) => d);
        const fix = await alertBox(ids.length ? `${blocked.length} aircraft cannot start` : 'No aircraft can start yet', ids.length ? `${ids.length} aircraft are ready; these are not:` : 'Fix these first:',
          { kind: 'warn', items: blocked.map(([d, why]) => `${d.label}: ${why}`), ok: lock.length ? `Reconcile ${lock.length} on ground` : 'Understood', cancel: lock.length ? 'Close' : null });
        if (fix && lock.length) for (const d of lock) await this.act(d.drone_id, 'reconcile');
        if (!ids.length) return;
      }
    }
    if (!ids.length) return toast('Nothing to upload', 'Plan trees in step 3 first.', 'warn');
    if (action === 'start' && !(await confirmDialog(`Start ${ids.length} drone(s)?`, 'All selected drones arm, take off and fly autonomously.', { ok: 'Start all', kind: 'go' }))) return;
    for (const id of ids) { if (action === 'start') { const d = store.drone(id); await api('/api/fleet/command', { drone_id: id, action, mission_id: d.data.mission.id, revision: d.data.mission.revision, command_id: crypto.randomUUID() }).then(() => { this.started.add(id); toast(`${id}: started`, '', 'ok'); }).catch(e => toast(`${id}: start failed`, e.message, 'bad')); } else await this.act(id, action); }
    if (action === 'start') this.simCameras(ids.filter(id => this.started.has(id)));
  },
  /** Simulation: open the drone camera window when a started mission uses the camera. */
  simCameras(ids) {
    if (store.mode !== 'sim' || !(prefs.get('simArena', {}).cameraOnMission ?? true)) return;
    const camera = ids.some(id => { const a = store.assigned.get(id); return a && (a.usesCamera ?? a.template !== 'swarm_coverage'); });
    if (camera) api('/api/sim/cameras', {}).then(r => { if (r.result === 'opened') toast('Drone camera window', 'Opening in its own window for this camera mission.', 'ok'); }).catch(() => {});
  },
  leaveWarnings(s) {
    const out = [];
    for (const d of s.fleet) {
      const ui = this.cards.get(d.drone_id), a = store.assigned.get(d.drone_id), m = d.data?.mission;
      if (!a) out.push(`${d.label}: no tree assigned (plan in step 3 or choose an XML file).`);
      else if (!(m && ui?.assignedSha && m.sha256 === ui.assignedSha)) out.push(`${d.label}: tree not uploaded & validated onboard.`);
      else if (!d.data?.active && d.data?.execution?.mission_id !== m.id && !this.started.has(d.drone_id)) out.push(`${d.label}: validated but not started.`);
    }
    return out;
  },
  update(s) {
    const drones = s.fleet;
    if (!drones.length) { if (!this.empty) this.empty = card({ title: 'No aircraft', ic: 'drone', cls: 'span-all' }, emptyState('drone', 'Add an aircraft first', 'Step 1 — Connect.'));
      fill(this.grid, [this.empty]); this.done = false; return; }
    let running = 0, staged = 0, finished = 0;
    fill(this.grid, drones.map((d, i) => {
      const ui = this.cardFor(d, i), data = d.data || {}, mission = data.mission, exec = data.execution || {};
      const a = store.assigned.get(d.drone_id), fresh = isFresh(d), busy = this.busy.has(d.drone_id);
      // tree source + preview
      if (a !== ui.shownAssignment) {
        ui.shownAssignment = a;
        let tree = null; try { tree = a ? parseXml(a.xml) : null; } catch (e) { toast(`${d.label}: XML invalid`, e.message, 'bad'); }
        ui.viewer.update(tree, { keepView: false });
        ui.payloadVisible = !!a && /<DropPayload|<ReleasePayload|<DropAtTarget/.test(a.xml);
        ui.payload.style.display = ui.payloadVisible ? '' : 'none';
        const pay = payloadOf(d);
        // Every load needs a new package ID (the drone refuses one used in an earlier mission).
        if (ui.payloadVisible && !ui.payloadId.value.trim()) {
          const t = new Date(), p2 = n => String(n).padStart(2, '0');
          ui.payloadId.value = `kit_${d.drone_id}_${t.getFullYear()}${p2(t.getMonth() + 1)}${p2(t.getDate())}_${p2(t.getHours())}${p2(t.getMinutes())}`;
        }
        if (ui.payloadVisible && pay?.capacity) ui.payloadCount.value = Math.min(pay.capacity, Math.max(1, +ui.payloadCount.value === 1 ? pay.capacity : +ui.payloadCount.value));
        ui.payloadCap.textContent = pay ? (pay.capacity ? `Drone set-up: ${pay.capacity} package${pay.capacity === 1 ? '' : 's'} on ${pay.servos.length} servo${pay.servos.length === 1 ? '' : 's'} (${pay.servos.map(s => `set ${s.set} × ${s.drops}`).join(', ')}).` : 'This drone reports no payload servo: set one on its control panel.') : 'Payload set-up not reported by this drone yet.';
        ui.assignedSha = null; if (a) sha256(a.xml).then(v => { ui.assignedSha = v; this.update(store); });
        const cost = h('span', { class: 'mono faint' });
        if (a) radioFrames({ xml: a.xml, route_bundle: a.bundle || null }).then(n => {
          const baud = store.state?.connection?.baud;
          cost.textContent = `${(a.xml.length / 1024).toFixed(1)} KB · ${n} radio frames${baud ? ` · ~${Math.ceil(n * 230 / (baud / 10)) + 1} s @ ${baud}` : ''}`; });
        fill(ui.source, [a ? badge(a.source === 'plan' ? 'PLANNED' : 'XML FILE', 'info') : badge('NO TREE', 'warn'),
          h('span', { class: 'faint' }, a ? `${a.label} · ${countNodes(tree)} nodes` : 'Plan in step 3, or choose an XML file'), cost]);
      }
      // The staged onboard mission counts only if it is exactly this drone's assigned tree.
      const validated = !!(mission && a && ui.assignedSha && mission.sha256 === ui.assignedSha);
      const thisRun = validated && exec.mission_id === mission.id;
      const active = !!data.active && thisRun, done = thisRun && ['SUCCESS', 'FAILURE', 'CANCELLED', 'ERROR', 'INTERRUPTED'].includes(exec.state);
      if (validated) staged++; if (data.active) running++; if (done) finished++;
      const stepState = [a ? 'done' : 'now', validated ? 'done' : a ? 'now' : '', active || done ? 'done' : validated && data.can_start ? 'now' : '', done ? 'done' : active ? 'now' : ''];
      fill(ui.stepper, ['Tree', 'Validated onboard', 'Started', 'Finished'].flatMap((label, j) => [j ? h('span', { class: 'bar' }) : null,
        h('span', { class: `s ${stepState[j]}` }, h('i', {}, stepState[j] === 'done' ? '✓' : j + 1), label)]));
      ui.badge.className = `badge ${active ? 'warn' : exec.state === 'SUCCESS' && done ? 'ok' : done ? 'bad' : fresh ? 'info' : 'bad'}`;
      ui.badge.textContent = active ? `RUNNING ${data.bt_running_count || ''}` : done ? exec.state : fresh ? (readyToFly(d) ? 'READY' : 'CHECKS') : d.connection;
      fill(ui.status, [h('dt', {}, 'Onboard mission'), h('dd', {}, mission ? `${mission.id.slice(0, 8)} · ${mission.node_count} nodes · ${mission.state}` : '—'),
        h('dt', {}, 'Execution'), h('dd', {}, exec.state ? `${exec.state}${exec.detail ? ' · ' + exec.detail : ''}` : '—'),
        h('dt', {}, 'Runtime'), h('dd', {}, rescueOf(d)?.active_operation || (rescueOf(d)?.flags?.VehicleHealthy ? 'idle · healthy' : rescueOf(d)?.flags?.reason || 'idle'))]);
      ui.stage.disabled = busy || !fresh || !!data.active || !a;
      ui.start.disabled = busy || !fresh || !data.can_start || !validated;
      ui.start.title = !validated ? 'Upload & validate this tree first' : data.can_start ? '' : 'Needs disarmed ground state and flight allowed onboard';
      ui.cancel.style.display = data.active ? '' : 'none'; ui.cancel.disabled = busy;
      ui.reconcile.style.display = data.hold_lockout || rescueOf(d)?.flags?.AbortRequested ? '' : 'none'; ui.reconcile.disabled = busy || !!data.active;
      return ui.card;
    }));
    fill(this.topInfo, [h('div', { class: 'row' }, badge(`${store.assigned.size} tree(s) assigned`, 'info'), badge(`${staged} validated onboard`, 'info'), badge(`${running} running`, running ? 'warn' : ''),
      store.plan ? h('span', { class: 'faint' }, `Plan: ${store.plan.template} · ${store.plan.area || ''}`) : h('span', { class: 'faint' }, 'No plan yet — step 3'))]);
    this.done = running > 0 || finished > 0;
  },
};
