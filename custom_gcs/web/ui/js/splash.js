// AERIXIS loading splash controller (classic script, runs before the app modules).
// The app reports real loading steps with AerixisSplash.step(key, 'run'|'ok'|'warn'|'fail', detail);
// the splash fades out once every step is ok/warn (warn = usable but degraded, e.g. maps offline).
// A step that fails, a script that does not load, or a 25 s timeout shows an error card with
// Retry / Open anyway, so the operator is never locked out of the emergency controls.
(function () {
  const el = document.getElementById('splash');
  if (!el) return;
  const t0 = performance.now();
  let quick = false;                       // reloads in the same browser session skip the intro
  try { quick = sessionStorage.getItem('aerixis.splash') === '1'; sessionStorage.setItem('aerixis.splash', '1'); } catch (e) { }
  if (quick) el.classList.add('quick');
  // ?splash=hold keeps the splash up after loading (for checking its look); Esc still closes it.
  const hold = /[?&]splash=hold\b/.test(location.search);
  const MIN_MS = quick ? 350 : 2200, TIMEOUT_MS = 25000;
  const steps = new Map();
  el.querySelectorAll('[data-step]').forEach(row => steps.set(row.dataset.step, { row, state: 'wait', label: row.firstChild.textContent.trim() }));
  const bar = el.querySelector('.sp-bar i'), status = el.querySelector('.sp-status b'), count = el.querySelector('.sp-status span');
  let finished = false, closed = false, failed = false;

  function render() {
    const all = [...steps.values()], done = all.filter(s => s.state === 'ok' || s.state === 'warn').length;
    bar.style.width = Math.max(4, Math.round(100 * done / all.length)) + '%';
    count.textContent = `${done}/${all.length}`;
    const running = all.find(s => s.state === 'run') || all.find(s => s.state === 'wait');
    if (!failed) status.textContent = done === all.length ? 'Ready' : running ? running.label + '…' : 'Loading…';
    if (done === all.length && !failed) finish();
  }
  function step(key, state, detail) {
    const s = steps.get(key);
    if (!s || closed || s.state === state && detail === undefined) return;
    s.state = state;
    s.row.className = state;
    if (detail !== undefined) s.row.querySelector('small').textContent = detail;
    if (state === 'fail') fail(`${s.label} failed`, detail || '');
    render();
  }
  function finish() {
    if (finished) return;
    finished = true;
    clearTimeout(timer);
    if (!hold) setTimeout(close, Math.max(0, MIN_MS - (performance.now() - t0)));
  }
  function close() {
    if (closed) return;
    closed = true;
    clearTimeout(timer);
    el.classList.add('leaving');
    document.documentElement.classList.remove('splashing');
    window.dispatchEvent(new Event('aerixis:ready'));
    setTimeout(() => el.remove(), 500);
  }
  function fail(title, text) {
    if (closed) return;
    failed = true;
    el.classList.add('error');
    status.textContent = 'Could not finish loading';
    el.querySelector('.sp-error b').textContent = title;
    el.querySelector('.sp-error p').textContent = text;
    el.querySelector('[data-act=open]').focus();
  }
  const timer = setTimeout(() => {
    const pending = [...steps.values()].filter(s => s.state === 'wait' || s.state === 'run').map(s => s.label.toLowerCase());
    fail('Loading is taking too long', `Still waiting for: ${pending.join(', ') || 'the interface'}. Check that start_gcs.sh is running, then Retry — or open the GCS anyway.`);
  }, TIMEOUT_MS);
  window.addEventListener('error', e => {
    if (steps.get('ui')?.state !== 'ok') fail('The interface could not start', String(e.message || e.filename || 'script error'));
  });
  el.querySelector('[data-act=retry]').onclick = () => location.reload();
  el.querySelector('[data-act=open]').onclick = close;
  el.querySelector('[data-act=skip]').onclick = close;
  document.addEventListener('keydown', e => { if (e.key === 'Escape' && !closed) close(); });
  render();
  window.AerixisSplash = { step, fail, close, get open() { return !closed; } };
})();
