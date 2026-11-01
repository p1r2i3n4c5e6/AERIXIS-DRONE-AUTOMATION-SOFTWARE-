// Simulation loading window (2026-10-31): a large panel over the blurred GCS while the simulation
// starts (Connect -> Simulation (SITL)) or a drone joins (Add drone): live stages (arena, Gazebo,
// PX4 per drone, onboard software, drones added, link) -> ready summary with Add drone / 3D window.
// The work itself is done by the Connect page (start, add, pairing, link); this only shows it.
import { h, icon, store, fill, linkUp, isFresh } from './core.js';
import { sim, refreshSim } from './simarena.js';

const STEPS = [
  ['arena', 'Rescue arena at your location', '300 × 300 m · 50 people · 180 trees'],
  ['gazebo', 'Gazebo 3D world', 'physics, sensors, cameras'],
  ['px4', 'PX4 flight controllers', 'one per drone'],
  ['onboard', 'Onboard software', 'AERIXIS core, mission manager, tracker, gateway'],
  ['added', 'Drones added to the GCS', 'simulation aircraft list'],
  ['link', 'Link open', 'UDP 14555 · heartbeats from every drone'],
];

export function createSimOverlay(ctl) {
  let root = null, card = null, phase = 'closed', startedAt = 0, timer = null, wantDrones = 1;

  function stepStates() {
    const p = sim.status?.progress || {}, n = p.drones || wantDrones;
    const ids = [...Array(n).keys()].map(i => 'drone' + i);
    const fleet = new Map(store.fleet.map(d => [d.drone_id, d]));
    const added = ids.every(id => fleet.has(id));
    const fresh = ids.filter(id => fleet.has(id) && isFresh(fleet.get(id))).length;
    const st = {
      arena: p.arena ? 'done' : 'run',
      gazebo: p.gazebo ? 'done' : p.arena ? 'run' : 'wait',
      px4: p.px4_ready ? 'done' : p.gazebo ? 'run' : 'wait',
      onboard: p.onboard ? 'done' : p.px4_ready ? 'run' : 'wait',
      added: added && p.onboard ? 'done' : p.onboard ? 'run' : 'wait',
      link: added && p.onboard && linkUp() && fresh === n ? 'done' : added && p.onboard ? 'run' : 'wait',
    };
    const detail = {
      px4: `${p.px4_spawned || 0} of ${n} started`,
      added: added ? ids.map(id => fleet.get(id)?.label || id).join(', ') : `0 of ${n}`,
      link: `${fresh} of ${n} answering`,
      onboard: p.onboard_waiting ? 'starting on each drone… (the longest step)' : null,
    };
    if (p.failed || sim.status?.state === 'failed') for (const k in st) if (st[k] === 'run') st[k] = 'bad';
    return { st, detail, n, fresh };
  }

  function progressView() {
    const { st, detail, n, fresh } = stepStates();
    const done = Object.values(st).filter(s => s === 'done').length, failed = Object.values(st).includes('bad');
    const ready = done === STEPS.length;
    if (ready && phase === 'progress') phase = 'ready';
    const secs = Math.round((Date.now() - (startedAt || Date.now())) / 1000);
    const adding = !!sim.status?.progress?.adding;
    const title = failed ? (adding ? 'The drone was not added' : 'Simulation did not start') : ready ? 'Simulation ready'
      : adding ? `Adding drone ${n} to the arena…` : 'Starting the simulation…';
    const sub = failed ? 'See the launcher output below. Retry, or Stop and check the logs.'
      : ready ? `${n} drone${n > 1 ? 's' : ''} connected in the same arena, ready to plan.`
        : `${n} drone${n > 1 ? 's' : ''} · ${adding ? 'about 1–2 minutes' : 'about 1–3 minutes'} · ${Math.floor(secs / 60)}:${String(secs % 60).padStart(2, '0')}`;
    const rows = STEPS.map(([key, name, hint]) => {
      const s = st[key], extra = detail[key];
      return h('div', { class: `simov-step ${s}` },
        h('span', { class: 'simov-dot', html: s === 'done' ? icon('check') : s === 'bad' ? icon('warn') : '' }),
        h('div', {}, h('b', {}, name), h('small', {}, extra && s !== 'wait' ? extra : hint)));
    });
    const log = h('pre', { class: 'sim-log simov-log' }, (sim.status?.log || []).join('\n'));
    return [
      h('div', { class: 'simov-head' }, h('div', { class: `simov-ic ${ready ? 'ok' : failed ? 'bad' : 'spin'}`, html: icon(ready ? 'check' : failed ? 'warn' : 'sim') }),
        h('div', {}, h('h2', {}, title), h('p', {}, sub))),
      h('div', { class: 'progress simov-bar' }, h('span', { style: { width: `${Math.round(100 * done / STEPS.length)}%` } })),
      h('div', { class: 'simov-steps' }, ...rows),
      h('details', { class: 'simov-more', open: failed }, h('summary', { class: 'kicker' }, 'Launcher output'), log),
      h('div', { class: 'simov-actions' },
        ready ? h('button', { class: 'btn', onclick: () => ctl.gui(), title: 'Open the Gazebo 3D window' }, h('span', { html: icon('grid') }), '3D window') : null,
        ready && n < 4 ? h('button', { class: 'btn', onclick: () => add(), title: 'One more drone in the same arena' }, h('span', { html: icon('plus') }), 'Add drone') : null,
        ready ? h('button', { class: 'btn', onclick: close }, 'Close')
          : h('button', { class: 'btn', onclick: close, title: 'Keep going in the background' }, 'Run in background'),
        failed ? h('button', { class: 'btn primary', title: adding ? 'Finish adding this drone (the others keep flying)' : 'Start the simulation again',
          onclick: async () => { if (await (adding ? ctl.add() : ctl.start())) { startedAt = Date.now(); phase = 'progress'; render(); } } }, h('span', { html: icon('refresh') }), 'Retry')
          : ready ? h('button', { class: 'btn primary simov-go', onclick: () => { close(); location.hash = '#plan'; } }, 'Go to Plan', h('span', { html: icon('arrow') }))
            : h('button', { class: 'btn danger', onclick: async () => { if (await ctl.stop()) close(); } }, h('span', { html: icon('stop') }), 'Stop')),
      ready && fresh === n ? h('div', { class: 'faint simov-foot' }, 'Next: Plan → Use arena as search area → Generate → Upload & start.') : null,
    ];
  }

  function render() {
    if (!root || phase === 'closed') return;
    fill(card, progressView());
  }

  async function add() {
    if (await ctl.add()) { wantDrones += 1; open(true); }
  }

  /** Show the loading window. fresh: a start or an add was just requested (restart the timer). */
  function open(fresh) {
    phase = 'progress';
    if (fresh || !startedAt) startedAt = Date.now();
    if (!root) {
      card = h('div', { class: 'simov-card', role: 'dialog', 'aria-modal': 'true', 'aria-label': 'Simulation' });
      root = h('div', { class: 'simov' }, card);
      document.body.append(root);
      sim.listeners.add(render);
      store.on(render);
      document.addEventListener('keydown', e => { if (e.key === 'Escape' && phase !== 'closed') close(); });
    }
    root.hidden = false;
    timer ||= setInterval(render, 1000);
    refreshSim();
    render();
  }

  function close() {
    phase = 'closed';
    if (root) root.hidden = true;
    clearInterval(timer); timer = null;
  }

  return { open, close, add, setDrones: n => { wantDrones = n; }, get isOpen() { return phase !== 'closed'; } };
}
