// AERIXIS Copilot — floating offline mission planner (2026-11-01). A robot-drone button in the
// bottom-right corner opens a chat on every page. The operator describes the mission in their own
// language; the local model (llama.cpp on this laptop) asks for what is missing and proposes a plan
// shown as an EDITABLE card (mission, parameters, each drone's role / AI avoidance / endurance).
// "Instant plan" plans at once from the KML and the drones' own set-up, without the model.
// "Apply" fills the normal Plan form; the operator still accepts the conditions and the existing
// planner makes the behaviour trees. The model never makes trees or commands a drone.
import { h, icon, badge, api, store, prefs, toast, fill, isFresh, payloadOf, cameraOf } from './core.js';
import { polygonStats } from './areadraw.js';
import { catalog, emergencyAction } from './missions.js';
import { setAiDrones } from './flightctl.js';

/** Area of the first polygon of a KML text: { area_ha, width_m, height_m } (null if none). */
export function kmlArea(text) {
  const m = /<coordinates>([\s\S]*?)<\/coordinates>/i.exec(text || '');
  if (!m) return null;
  const pts = m[1].trim().split(/\s+/).map(t => t.split(',').map(Number)).filter(p => p.length >= 2 && p.every(Number.isFinite)).map(([lon, lat]) => [lat, lon]);
  if (pts.length < 3) return null;
  const st = polygonStats(pts), lat0 = pts[0][0] * Math.PI / 180;
  const xs = pts.map(p => p[1] * 111320 * Math.cos(lat0)), ys = pts.map(p => p[0] * 110540);
  return { area_ha: +(st.area_m2 / 1e4).toFixed(2), width_m: Math.round(Math.max(...xs) - Math.min(...xs)), height_m: Math.round(Math.max(...ys) - Math.min(...ys)) };
}

// A friendly robot drone: round body with eyes, four rotors, a little antenna.
const ROBOT = `<svg viewBox="0 0 64 64" class="robot" aria-hidden="true">
  <defs><linearGradient id="rb" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="#7c5cff"/><stop offset="1" stop-color="#22b5c9"/></linearGradient></defs>
  <g class="rotors" fill="none" stroke="#cfe9ff" stroke-width="2" stroke-linecap="round">
    <line x1="10" y1="16" x2="22" y2="16"/><line x1="42" y1="16" x2="54" y2="16"/>
    <line x1="18" y1="22" x2="22" y2="26"/><line x1="46" y1="22" x2="42" y2="26"/></g>
  <g stroke="#cfe9ff" stroke-width="2"><line x1="16" y1="16" x2="16" y2="22"/><line x1="48" y1="16" x2="48" y2="22"/></g>
  <line x1="32" y1="14" x2="32" y2="20" stroke="#cfe9ff" stroke-width="2"/><circle cx="32" cy="12" r="2.6" fill="#ffd166"/>
  <rect x="14" y="20" width="36" height="26" rx="13" fill="url(#rb)"/>
  <rect x="19" y="26" width="26" height="13" rx="6.5" fill="#0d1530"/>
  <circle class="eye" cx="26" cy="32.5" r="3" fill="#7ef0ff"/><circle class="eye" cx="38" cy="32.5" r="3" fill="#7ef0ff"/>
  <path d="M27 42 q5 3 10 0" stroke="#e8f6ff" stroke-width="2" fill="none" stroke-linecap="round"/>
  <g stroke="#cfe9ff" stroke-width="2" stroke-linecap="round"><line x1="22" y1="46" x2="19" y2="52"/><line x1="42" y1="46" x2="45" y2="52"/></g>
</svg>`;

export function createAiPlanner(plan, { MISSIONS, O }) {
  const messages = prefs.get('aiChat', []) || [];
  let busy = false, lastPlan = prefs.get('aiPlan', null), open = prefs.get('aiOpen', false);
  const status = h('span', { class: 'ai-status' }, 'Offline planner');
  const log = h('div', { class: 'ai-log' });
  const input = h('textarea', { class: 'ai-input', rows: 2, placeholder: 'Apni bhasha mein mission batayein… e.g. "4 drone, 2 search aur 2 packet drop, speed 2"' });
  const send = h('button', { class: 'btn primary ai-send', title: 'Send (Enter)', onclick: () => ask() }, h('span', { html: icon('arrow') }));
  input.addEventListener('keydown', e => { if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); ask(); } });

  const save = () => { prefs.set('aiChat', messages.slice(-30).map(({ form, ...m }) => m)); prefs.set('aiPlan', lastPlan); };
  const say = (content, extra = {}) => { messages.push({ role: 'assistant', content, ...extra }); save(); render(); };
  const fleet = () => store.fleet.filter(d => isFresh(d) || d.data);

  function context() {
    const drones = store.fleet.map(d => {
      const cam = cameraOf(d), pay = payloadOf(d);
      return { id: d.drone_id, label: d.label, online: isFresh(d), camera: cam ? { hfov_deg: cam.hfov, vfov_deg: cam.vfov } : null,
        payload: pay ? { capacity: pay.capacity, servos: pay.servos } : null, endurance_min: prefs.get('endurance.' + d.drone_id, 20) };
    });
    const area = plan.kmlText ? { loaded: true, name: plan.kmlName, ...(kmlArea(plan.kmlText) || {}) } : { loaded: false };
    const params = Object.fromEntries(Object.entries(O).map(([name, [key, label, def, min, max, remembered = key]]) =>
      [name, { label, default: def, min, max, value: prefs.get('opt.' + remembered, def) }]));
    return { mode: store.mode === 'sim' ? 'simulation' : 'real drones', fleet: drones, area, params,
      missions: MISSIONS.map(m => ({ id: m.id, title: m.title, summary: m.summary, options: m.options })),
      saved: catalog.custom.map(r => ({ id: r.id, name: r.name })), current_mission: store.missionType,
      roles: plan.droneRoles(), emergency: emergencyAction() };
  }

  // ---------- Instant plan: from the KML and each drone's own set-up, no model ----------
  function instantPlan() {
    const drones = store.fleet.filter(d => isFresh(d));
    if (!drones.length) return { error: 'No drone online yet: connect them first (Connect page).' };
    if (!plan.kmlText) return { error: 'Load the search area first: a KML file, a drawn area, or the arena (simulation).' };
    const withCam = drones.filter(d => cameraOf(d)), withPkg = drones.filter(d => (payloadOf(d)?.capacity || 0) > 0);
    const altitude = 15, hfov = Math.min(...withCam.map(d => cameraOf(d).hfov), 65);
    const spacing = Math.max(5, Math.floor(2 * altitude * Math.tan(hfov * Math.PI / 360) / 1.1));
    const params = { altitude_m: altitude, spacing_m: spacing, speed_mps: 2, approach_speed: 1, rtl_speed: 2, search_pitch_nadir: -90 };
    let mission = 'human_geotag_resume', roles = {};
    if (drones.length >= 2 && withPkg.length && withCam.length) {
      // Payload drones deliver; at least one camera drone searches (half the fleet, at least one).
      const deliver = withPkg.slice(-Math.max(1, Math.min(withPkg.length, Math.floor(drones.length / 2))));
      if (drones.some(d => !deliver.includes(d) && cameraOf(d))) {
        mission = 'search_deliver';
        roles = Object.fromEntries(drones.map(d => [d.drone_id, deliver.includes(d) ? 'delivery' : 'search']));
        Object.assign(params, { delivery_speed: 2, delivery_wait_s: 1500, airborne_wait_s: 600, delivery_altitude_m: 0 });
      }
    } else if (!withCam.length) mission = 'swarm_coverage';
    const ai_drones = Object.entries(roles).filter(([, r]) => r === 'delivery').map(([id]) => id);
    return { mission, params, roles, orbit_drones: [], emergency: 'RTL', ai_drones,
      endurance: Object.fromEntries(drones.map(d => [d.drone_id, prefs.get('endurance.' + d.drone_id, 20) < 30 && store.mode === 'sim' ? 60 : prefs.get('endurance.' + d.drone_id, 20)])) };
  }

  function instant() {
    const p = instantPlan();
    messages.push({ role: 'user', content: '⚡ Instant plan' });
    if (p.error) return say(p.error, { actions: actionsForMissing() });
    lastPlan = { plan: p, warnings: [], status: 'ready' };
    say(`Instant plan ready: ${MISSIONS.find(m => m.id === p.mission)?.title}. ${Object.keys(p.roles).length
      ? Object.entries(p.roles).map(([id, r]) => `${store.drone(id)?.label || id} ${r}`).join(', ') + '. ' : ''}Lanes ${p.params.spacing_m} m apart at ${p.params.altitude_m} m (the camera covers that without gaps), 2 m/s. AI avoidance on the delivery drones. Change anything below, then Apply.`, { plan: p });
  }

  function actionsForMissing() {
    const out = [];
    if (!plan.kmlText && store.mode === 'sim') out.push(['Use arena as search area', () => { plan.useArena(); say('Arena set as the search area. Now press ⚡ Instant plan or tell me the mission.'); }]);
    if (!plan.kmlText) out.push(['Open Plan to load a KML', () => { location.hash = '#plan'; }]);
    return out;
  }

  // ---------- chat with the model ----------
  async function ask(text) {
    const q = (text ?? input.value).trim();
    if (!q || busy) return;
    messages.push({ role: 'user', content: q }); input.value = ''; busy = true; save(); render();
    try {
      const r = await api('/api/llm/chat', { messages: messages.map(({ role, content }) => ({ role, content })), context: context() });
      lastPlan = r.plan?.mission ? { plan: r.plan, warnings: r.warnings, status: r.status } : lastPlan;
      messages.push({ role: 'assistant', content: r.reply, plan: r.plan?.mission ? r.plan : null, warnings: r.warnings,
        actions: plan.kmlText ? undefined : actionsForMissing() });
      const t = r.timing || {};
      status.textContent = `Answered in ${t.seconds} s · ${t.tokens_per_s} words/s`;
    } catch (e) {
      messages.push({ role: 'assistant', content: `⚠ ${e.message}`, error: true });
    } finally { busy = false; save(); render(); }
  }

  // ---------- editable plan card ----------
  function planCard(p, warnings = []) {
    const m = MISSIONS.find(x => x.id === p.mission);
    const form = { mission: p.mission, params: { ...(p.params || {}) }, roles: { ...(p.roles || {}) },
      ai: new Set(p.ai_drones || Object.entries(p.roles || {}).filter(([, r]) => r === 'delivery').map(([id]) => id)),
      endurance: { ...(p.endurance || {}) }, emergency: p.emergency || 'RTL' };
    const missionSel = h('select', { onchange: () => { form.mission = missionSel.value; rebuild(); } },
      ...MISSIONS.map(x => h('option', { value: x.id, selected: x.id === p.mission }, `${x.n} · ${x.title}`)),
      ...(p.mission === 'custom' ? [h('option', { value: 'custom', selected: true }, `New: ${p.custom_name || 'custom mission'}`)] : []));
    const body = h('div');
    const rebuild = () => {
      const mm = MISSIONS.find(x => x.id === form.mission);
      const names = (mm?.options || Object.keys(form.params)).filter(n => O[n]);
      const roleful = form.mission === 'search_deliver';
      fill(body, [
        h('div', { class: 'ai-grid' }, ...names.map(n => {
          const [key, label, def, min, max, remembered = key] = O[n];
          const value = form.params[n] ?? prefs.get('opt.' + remembered, def);
          form.params[n] = value;
          return h('label', { class: 'ai-field', title: `${label} (${min} … ${max})` }, label.replace(/ \(.*$/, ''),
            h('input', { type: 'number', step: 'any', min, max, value, onchange: e => { form.params[n] = Math.min(max, Math.max(min, +e.target.value)); e.target.value = form.params[n]; } }));
        })),
        h('div', { class: 'ai-drones' }, h('div', { class: 'ai-drow head' }, h('span', {}, 'Drone'), h('span', {}, roleful ? 'Role' : ''), h('span', { title: 'AI avoidance' }, 'AI'), h('span', {}, 'Endurance')),
          ...store.fleet.map(d => {
            const id = d.drone_id, pay = payloadOf(d);
            form.endurance[id] ??= prefs.get('endurance.' + id, 20);
            return h('div', { class: 'ai-drow' },
              h('span', {}, h('b', {}, d.label), h('small', { class: 'faint' }, ` ${cameraOf(d) ? 'cam' : 'no cam'} · ${pay?.capacity ?? 0} pkg`)),
              roleful ? h('select', { onchange: e => { form.roles[id] = e.target.value; } },
                ...['search', 'delivery'].map(r => h('option', { value: r, selected: (form.roles[id] || 'search') === r }, r))) : h('span'),
              h('input', { type: 'checkbox', checked: form.ai.has(id), title: 'AI avoidance for this drone', onchange: e => { e.target.checked ? form.ai.add(id) : form.ai.delete(id); } }),
              h('input', { type: 'number', min: 1, step: 1, value: form.endurance[id], title: 'Endurance (min)', onchange: e => { form.endurance[id] = +e.target.value; } }));
          })),
        ...(warnings || []).map(w => h('div', { class: 'ai-warn' }, '⚠ ' + w))]);
    };
    rebuild();
    return h('div', { class: 'ai-plan' },
      h('div', { class: 'row', style: { gap: '6px', alignItems: 'center' } }, h('b', {}, 'Plan'), missionSel),
      body,
      h('div', { class: 'row', style: { gap: '6px', marginTop: '8px', flexWrap: 'wrap' } },
        h('button', { class: 'btn sm primary', onclick: () => apply({ ...p, mission: form.mission, params: form.params, roles: form.roles,
          ai_drones: [...form.ai], endurance: form.endurance, emergency: form.emergency }) }, h('span', { html: icon('check') }), 'Apply to plan'),
        h('small', { class: 'faint' }, m ? m.summary : '')));
  }

  async function apply(p) {
    try {
      if (location.hash !== '#plan') location.hash = '#plan';
      for (const [id, v] of Object.entries(p.endurance || {})) if (v > 0) prefs.set('endurance.' + id, v);
      await plan.applyAiPlan(p);
      plan.renderFleet?.();
      if (p.ai_drones) {
        const r = await setAiDrones(p.ai_drones).catch(e => ({ error: e.message }));
        if (r.error) toast('AI avoidance not set', r.error, 'bad');
      }
      say(`Applied ✓ ${MISSIONS.find(m => m.id === p.mission)?.title || 'Mission'} is in the Plan form${p.ai_drones?.length ? `; AI avoidance on ${p.ai_drones.map(id => store.drone(id)?.label || id).join(', ')}` : ''}. Read the conditions and tick "I have read…" — the trees are made, then Upload & start.`);
    } catch (e) { toast('Plan not applied', e.message, 'bad'); }
  }

  // ---------- rendering ----------
  function render() {
    fill(log, [
      h('div', { class: 'ai-msg assistant' }, 'Namaste! 👋 I plan AERIXIS missions. Load the area (KML), then press ⚡ Instant plan — or tell me what you need: which drones search, which drop packages, how fast, how much time.'),
      ...messages.map(m => h('div', { class: `ai-msg ${m.role}${m.error ? ' error' : ''}` }, m.content,
        ...(m.plan ? [planCard(m.plan, m.warnings)] : []),
        ...(m.actions ? [h('div', { class: 'row', style: { gap: '6px', marginTop: '6px', flexWrap: 'wrap' } }, ...m.actions.map(([t, fn]) => h('button', { class: 'btn sm', onclick: fn }, t)))] : [])))]);
    if (busy) log.append(h('div', { class: 'ai-msg assistant thinking' }, '…'));
    log.scrollTop = log.scrollHeight;
    send.disabled = busy;
  }

  async function warm() {
    try {
      const s = await api('/api/llm/status');
      if (!s.installed) { status.textContent = 'Chat model not installed (~/aerixis_llm) · Instant plan works'; return; }
      status.textContent = s.ready ? 'Ready · offline' : 'Loading the model on the GPU…';
      if (!s.ready) { await api('/api/llm/start', {}); status.textContent = 'Ready · offline'; }
    } catch (e) { status.textContent = 'Chat unavailable (' + e.message + ') · Instant plan works'; }
  }

  // ---------- floating shell ----------
  const panel = h('section', { class: 'copilot-panel', role: 'dialog', 'aria-label': 'AERIXIS Copilot' },
    h('header', {}, h('span', { class: 'copilot-avatar', html: ROBOT }),
      h('div', { class: 'copilot-title' }, h('b', {}, 'AERIXIS Copilot'), status),
      h('button', { class: 'icon-btn', title: 'New chat', onclick: () => { messages.length = 0; lastPlan = null; save(); render(); } }, h('span', { html: icon('plus') })),
      h('button', { class: 'icon-btn', title: 'Hide', onclick: () => toggle(false) }, '✕')),
    h('div', { class: 'copilot-quick' },
      h('button', { class: 'btn sm primary', onclick: instant, title: 'Plan now from the area and the drones\' own set-up' }, '⚡ Instant plan'),
      h('button', { class: 'btn sm', onclick: () => ask('Ek fast search-and-deliver mission plan karo poore area ke liye') }, 'Search + deliver'),
      h('button', { class: 'btn sm', onclick: () => ask('Sab drone sirf search aur geotag karein') }, 'Search only')),
    log, h('div', { class: 'ai-row' }, input, send));
  const bubble = h('div', { class: 'copilot-bubble', onclick: () => toggle(true) }, 'Hey! 👋 Use me to plan the mission');
  const fab = h('button', { class: 'copilot-fab', title: 'AERIXIS Copilot — plan a mission', onclick: () => toggle(!open), html: ROBOT });
  const root = h('div', { class: 'copilot' }, panel, bubble, fab);
  function toggle(on) {
    open = on; prefs.set('aiOpen', on);
    root.classList.toggle('open', on); bubble.classList.add('gone');
    if (on) { warm(); render(); setTimeout(() => input.focus(), 50); }
  }
  root.classList.toggle('open', open);
  setTimeout(() => bubble.classList.add('show'), 1200);
  setTimeout(() => bubble.classList.add('gone'), 12000);
  render();
  if (open) warm();
  return { root, warm, ask, instant, context };
}
