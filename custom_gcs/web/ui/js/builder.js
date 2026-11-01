// Custom mission builder: four simple steps, no behaviour-tree nodes shown.
//   1 Aircraft carries (camera, payload servo)  2 Mission type (coverage, watch stations, track)
//   3 When a person is found (detection, geotag, orbit radius/laps, payload drop, continue)
//   4 Finish & safety (RTL/LAND on finish and on emergency, name)
// The planner (as2_rescue_planner.features) checks the choices and composes the tree with the
// mandatory safety parts, which are shown here but cannot be switched off.
import { h, icon, toast, fill, badge } from './core.js';
import { BUILTIN_FEATURES, saveMission, describeMission } from './missions.js';

const STEPS = [['Aircraft carries', 'drone'], ['Mission type', 'map'], ['When a person is found', 'person'], ['Finish & safety', 'shield']];

/** Make the choices consistent (same rules as the planner, applied as the operator clicks). */
function settle(f) {
  if (!f.camera) { f.detect = f.geotag = f.orbit = f.drop = false; if (f.type !== 'coverage') f.type = 'coverage'; }
  if (f.type === 'watch' || f.type === 'track') f.detect = true;
  if (!f.detect) f.geotag = f.orbit = f.drop = false;
  if (f.type === 'track') f.geotag = f.orbit = f.drop = false;
  if (!f.payload) f.drop = false;
  if (f.drop) f.geotag = true;
  return f;
}

export function openBuilder({ copyOf = null, record = null, copy = false, values = {}, fleetSize = 1, onSaved } = {}) {
  const base = record ? record.features : copyOf ? BUILTIN_FEATURES[copyOf.id] : { camera: true, payload: false, type: 'coverage', detect: true, geotag: true, orbit: false, drop: false, resume: true, finish: 'RTL', emergency: 'RTL' };
  const f = settle({ ...base });
  const editing = record && !copy ? record : null;
  const opts = { orbit_radius_m: record?.options?.orbit_radius_m ?? values.orbit_radius_m ?? 10, orbit_laps: record?.options?.orbit_laps ?? values.orbit_laps ?? 2 };
  let step = 0, error = '', describeSeq = 0;
  // Delivery drones (2026-11-01): a fleet choice, kept outside the feature spec.
  let delivery = !!record?.delivery_drones;
  const name = h('input', { maxlength: 60, placeholder: 'e.g. Night coverage without camera', value: editing ? record.name : record ? `${record.name} (copy)` : copyOf ? `${copyOf.title} (my copy)` : '' });
  const notes = h('input', { maxlength: 500, placeholder: 'optional — what this mission is for', value: record?.notes || '' });
  const radius = h('input', { type: 'number', min: 3, max: 100, step: 'any', value: opts.orbit_radius_m });
  const laps = h('input', { type: 'number', min: .25, max: 20, step: 'any', value: opts.orbit_laps });

  const tabs = h('div', { class: 'b-steps' });
  const body = h('div', { class: 'b-body' });
  const side = h('div', { class: 'b-side' });
  const back = h('button', { class: 'btn', onclick: () => go(step - 1) }, 'Back');
  const next = h('button', { class: 'btn primary', onclick: () => go(step + 1) }, 'Next', h('span', { html: icon('arrow') }));
  const save = h('button', { class: 'btn go', onclick: () => doSave(false) }, h('span', { html: icon('check') }), editing ? 'Save changes' : 'Save mission');
  const saveNew = editing ? h('button', { class: 'btn', onclick: () => doSave(true) }, 'Save as new') : null;
  const dialog = h('dialog', { class: 'glass-dialog builder c-violet' },
    h('div', { class: 'card-head' }, h('div', { class: 'card-icon', html: icon('tree') }),
      h('div', {}, h('h3', {}, editing ? `Edit "${record.name}"` : copyOf ? `New mission from "${copyOf.title}"` : 'New mission'),
        h('p', {}, 'Choose what the aircraft carries and does. The safety parts are added automatically.')),
      h('div', { class: 'right' }, h('button', { class: 'icon-btn', 'aria-label': 'Close', onclick: () => close(), html: '×' }))),
    tabs, h('div', { class: 'b-grid' }, body, side),
    h('div', { class: 'row end', style: { marginTop: '14px' } }, back, next, saveNew, save));
  const close = () => { dialog.close(); dialog.remove(); };
  dialog.addEventListener('cancel', e => { e.preventDefault(); close(); });

  const choice = (label, sub, ic, on, onclick, disabled = '') => h('button', { class: `choice ${on ? 'on' : ''}`, disabled: !!disabled, title: disabled || '', onclick },
    h('span', { class: 'ci', html: icon(ic) }), h('span', {}, h('b', {}, label), h('small', {}, disabled || sub)), h('span', { class: 'tick', html: icon('check') }));
  const toggle = (label, sub, key, disabled = '', extra = null) => h('div', { class: `tgl ${f[key] ? 'on' : ''} ${disabled ? 'dis' : ''}` },
    h('label', {}, h('input', { type: 'checkbox', checked: f[key], disabled: !!disabled, onchange: e => set(key, e.target.checked) }),
      h('span', {}, h('b', {}, label), h('small', {}, disabled || sub))), f[key] && extra ? extra : null);
  const set = (key, value) => {
    f[key] = value;
    if (key === 'detect' && value && !f.geotag && !f.orbit && f.type !== 'track') f.geotag = true;
    if (key === 'drop' && value) f.payload = true;
    settle(f); render();
  };

  function pane() {
    if (step === 0) return [
      h('h4', {}, 'Camera'), h('div', { class: 'choices' },
        choice('A8 gimbal camera', 'Person detection, tracking, geotag and orbit are possible', 'camera', f.camera, () => set('camera', true)),
        choice('No camera', 'Area coverage only; readiness does not check the camera', 'grid', !f.camera, () => set('camera', false))),
      h('h4', {}, 'Payload'), h('div', { class: 'choices' },
        choice('Payload servo', 'A package can be released over a geotagged person', 'gift', f.payload, () => set('payload', true), f.camera ? '' : 'A drop needs the camera to find the person'),
        choice('No payload', 'Nothing is released', 'stop', !f.payload, () => set('payload', false)))];
    if (step === 1) return [h('div', { class: 'choices col' },
      choice('Area coverage', 'Survey lanes over each aircraft\'s share of the KML area', 'grid', f.type === 'coverage', () => set('type', 'coverage')),
      choice('Watch stations', 'Each aircraft holds one station inside the area and sweeps yaw', 'gps', f.type === 'watch', () => set('type', 'watch'), f.camera ? '' : 'Needs the camera'),
      choice('Track person', 'Survey until a person is found, then follow them with the camera', 'person', f.type === 'track', () => set('type', 'track'), f.camera ? '' : 'Needs the camera'))];
    if (step === 2) {
      if (!f.camera) return [h('p', { class: 'muted' }, 'No camera on board: the aircraft only covers the area. Nothing to choose here.')];
      const track = f.type === 'track' ? 'Not combined with tracking' : '';
      return [
        toggle('Human detection', 'Only a person inside the KML area counts', 'detect', f.type !== 'coverage' ? 'Always on for this mission type' : ''),
        h('div', { class: 'kicker', style: { margin: '10px 0 4px' } }, 'Then'),
        toggle('Geotag', 'Fly overhead until the gimbal looks straight down; record latitude/longitude', 'geotag', !f.detect ? 'Needs detection' : track || (f.drop ? 'Needed for the payload drop' : '')),
        toggle('Payload drop', 'Release once, overhead, when steady and in the drop height window', 'drop', !f.detect ? 'Needs detection' : track || (!f.payload ? 'No payload servo (step 1)' : '')),
        toggle('Orbit the person', 'Circle the person, camera kept on them', 'orbit', !f.detect ? 'Needs detection' : track,
          h('div', { class: 'fields', style: { marginTop: '6px' } }, h('label', { class: 'field' }, 'Radius (m)', radius, h('small', {}, '3 … 100')),
            h('label', { class: 'field' }, 'Laps', laps, h('small', {}, '0.25 … 20')))),
        toggle('Continue searching', 'After each person: release the lock and resume (otherwise the mission ends)', 'resume', !f.detect ? 'Needs detection' : ''),
        h('div', { class: 'kicker', style: { margin: '10px 0 4px' } }, 'Fleet'),
        (() => {
          const dis = f.type !== 'coverage' || !f.geotag ? 'Needs area coverage with geotag' : '';
          return h('div', { class: `tgl ${delivery && !dis ? 'on' : ''} ${dis ? 'dis' : ''}` },
            h('label', {}, h('input', { type: 'checkbox', checked: delivery && !dis, disabled: !!dis, onchange: e => { delivery = e.target.checked; render(); } }),
              h('span', {}, h('b', {}, 'Delivery drones'), h('small', {}, dis || 'Other aircraft wait with packages and fly to every geotag, one package per person, until their packages run out. You choose which aircraft search and which deliver when planning.'))));
        })()];
    }
    return [
      h('h4', {}, 'When the mission ends'), h('div', { class: 'choices' },
        choice('Return to launch', 'Fly home and land', 'home', f.finish === 'RTL', () => set('finish', 'RTL')),
        choice('Land in place', 'Land where the mission ends', 'download', f.finish === 'LAND', () => set('finish', 'LAND'))),
      h('h4', {}, 'On an emergency'), h('div', { class: 'choices' },
        choice('Return to launch', 'Unhealthy vehicle, low battery or abort: fly home', 'home', f.emergency === 'RTL', () => set('emergency', 'RTL')),
        choice('Land in place', 'Land immediately where it is', 'download', f.emergency === 'LAND', () => set('emergency', 'LAND'))),
      h('h4', {}, 'Always included (cannot be removed)'), h('div', { class: 'locks' },
        ...['Wait until the vehicle is healthy', 'Health gate around the whole flight', 'Fleet: wait for the mesh, separation & peer gates',
          'Recovery branch: return / land, then hold', 'Return or land and disarm at the end'].map(t => h('span', { class: 'locked', html: icon('shield') + `<span>${t}</span>` }))),
      h('div', { class: 'fields', style: { marginTop: '12px' } }, h('label', { class: 'field', style: { gridColumn: 'span 2' } }, 'Mission name', name),
        h('label', { class: 'field', style: { gridColumn: 'span 2' } }, 'Notes', notes))];
  }

  async function describe() {
    const seq = ++describeSeq;
    try {
      const r = await describeMission(f, fleetSize);
      if (seq !== describeSeq) return;
      error = '';
      fill(side, [h('div', { class: 'kicker' }, 'What the aircraft will do'), h('ul', {}, ...r.does.map(t => h('li', {}, t))),
        h('div', { class: 'kicker' }, 'Needs on board'), h('div', { class: 'row', style: { gap: '5px' } },
          ...(r.requires.length ? r.requires.map(x => badge(x.replace('_', ' '), 'info')) : [badge('flight controller + GNSS only')]))]);
    } catch (e) {
      if (seq !== describeSeq) return;
      error = e.message;
      fill(side, [h('div', { class: 'hint bad', html: icon('warn') }, h('div', {}, error))]);
    }
    save.disabled = !!error; if (saveNew) saveNew.disabled = !!error;
  }

  function render() {
    fill(tabs, STEPS.map(([t, ic], i) => h('button', { class: `${i === step ? 'active' : ''} ${i < step ? 'done' : ''}`, onclick: () => go(i) },
      h('span', { class: 'bi', html: icon(ic) }), h('span', {}, `${i + 1} · ${t}`))));
    fill(body, pane());
    back.disabled = step === 0; next.style.display = step === STEPS.length - 1 ? 'none' : '';
    save.style.display = step === STEPS.length - 1 ? '' : 'none'; if (saveNew) saveNew.style.display = save.style.display;
    describe();
  }
  function go(i) { step = Math.max(0, Math.min(STEPS.length - 1, i)); render(); }

  async function doSave(asNew) {
    const r = +radius.value, l = +laps.value;
    if (f.orbit && !(r >= 3 && r <= 100 && l >= .25 && l <= 20)) return toast('Orbit out of range', 'Radius 3–100 m, laps 0.25–20.', 'bad');
    if (!name.value.trim()) { go(3); name.focus(); return toast('Give the mission a name', '', 'warn'); }
    const options = { ...(record?.options || {}), ...values, ...(f.orbit ? { orbit_radius_m: r, orbit_laps: l } : {}) };
    save.classList.add('busy');
    try {
      const rec = await saveMission({ id: editing && !asNew ? record.id : undefined, name: name.value.trim(), features: f, options,
        notes: notes.value.trim(), based_on: copyOf?.id || record?.based_on || '',
        delivery_drones: delivery && f.type === 'coverage' && f.geotag });
      toast('Mission saved', `${rec.name} — selected in Plan`, 'ok');
      close(); onSaved?.(rec);
    } catch (e) { toast('Not saved', e.message, 'bad', 10000); }
    finally { save.classList.remove('busy'); }
  }

  document.body.append(dialog); dialog.showModal();
  render();
}
