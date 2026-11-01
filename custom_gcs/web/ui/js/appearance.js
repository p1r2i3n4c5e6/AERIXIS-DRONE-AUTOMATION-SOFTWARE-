// Appearance (2026-10-30): mode (Light / Dark / Midnight / System), ready-made themes like a desktop
// OS, and the accent colour (Multicolour = each page its own colour, as before, or one colour for
// the whole GCS). Saved per browser (prefs 'appearance'); index.html applies it before first paint.
// Only colours change: no layout, no flight logic.
import { h, $, icon, prefs } from './core.js';

const MODES = [
  { id: 'light', name: 'Light' },
  { id: 'dark', name: 'Dark' },
  { id: 'midnight', name: 'Midnight' },
  { id: 'system', name: 'System' },
];

const MULTI = 'multi';
const RAINBOW = 'linear-gradient(90deg,#4f8df7,#22b07d,#f0a12c,#f2556b,#9b6cf6)';

// Ready-made themes: a mode plus an accent, like the themes of a desktop OS.
const THEMES = [
  { id: 'aerixis', name: 'AERIXIS', mode: 'light', accent: MULTI },
  { id: 'aerixis-dark', name: 'AERIXIS Dark', mode: 'dark', accent: MULTI },
  { id: 'midnight', name: 'Midnight', mode: 'midnight', accent: MULTI },
  { id: 'sky', name: 'Sky', mode: 'light', accent: '#0078D4' },
  { id: 'ocean', name: 'Ocean', mode: 'dark', accent: '#00B7C3' },
  { id: 'forest', name: 'Forest', mode: 'light', accent: '#10893E' },
  { id: 'sunset', name: 'Sunset', mode: 'dark', accent: '#F7630C' },
  { id: 'rose', name: 'Rose', mode: 'light', accent: '#E3008C' },
  { id: 'glow', name: 'Glow', mode: 'dark', accent: '#8764B8' },
  { id: 'graphite', name: 'Graphite', mode: 'dark', accent: '#69797E' },
  { id: 'gold', name: 'Gold', mode: 'midnight', accent: '#FFB900' },
  { id: 'ruby', name: 'Ruby', mode: 'midnight', accent: '#E81123' },
];

// Accent swatches (desktop OS accent palette plus the AERIXIS blue).
const ACCENTS = [
  ['#0078D4', 'Blue'], ['#0063B1', 'Navy blue'], ['#4F8DF7', 'AERIXIS blue'], ['#6B69D6', 'Indigo'],
  ['#8764B8', 'Purple'], ['#B146C2', 'Orchid'], ['#E3008C', 'Pink'], ['#EA005E', 'Raspberry'],
  ['#E81123', 'Red'], ['#CA5010', 'Rust'], ['#F7630C', 'Orange'], ['#FFB900', 'Gold'],
  ['#10893E', 'Green'], ['#107C10', 'Forest green'], ['#00B294', 'Seafoam'], ['#00B7C3', 'Teal'],
  ['#038387', 'Dark teal'], ['#69797E', 'Graphite'],
];

const SURFACE = {   // mini-preview colours: page background and sidebar per mode
  light: ['#e9eff7', '#0d1833'], dark: ['#131c31', '#121b31'], midnight: ['#0a1226', '#0d1833'],
};

const media = matchMedia('(prefers-color-scheme: dark)');
let state = load();
let panel = null;
let onChange = () => {};

function load() {
  const saved = prefs.get('appearance', null);
  if (saved && typeof saved === 'object') return { mode: saved.mode || 'system', accent: saved.accent || MULTI };
  // Before 2026-10-30 only light/dark was stored; the old dark was the deep one, now "Midnight",
  // but the user asked for a standard dark, so an old "dark" opens in the new Dark.
  const old = prefs.get('theme', null);
  return { mode: old === 'light' || old === 'dark' ? old : 'system', accent: MULTI };
}

const resolved = mode => (mode === 'system' ? (media.matches ? 'dark' : 'light') : mode);

/** Text colour that stays readable on the accent (dark text on gold / light colours). */
function onAccent(hex) {
  const v = hex.replace('#', '').match(/../g).map(x => parseInt(x, 16) / 255)
    .map(c => (c <= 0.03928 ? c / 12.92 : ((c + 0.055) / 1.055) ** 2.4));
  return 0.2126 * v[0] + 0.7152 * v[1] + 0.0722 * v[2] > 0.45 ? '#16181d' : '#ffffff';
}

function apply() {
  const root = document.documentElement, mode = resolved(state.mode);
  root.dataset.theme = mode === 'light' ? 'light' : 'dark';
  if (mode === 'midnight') root.dataset.shade = 'midnight'; else delete root.dataset.shade;
  if (state.accent === MULTI) {
    root.dataset.accent = 'multi';
    root.style.removeProperty('--theme-accent');
    root.style.removeProperty('--on-accent');
  } else {
    root.dataset.accent = 'single';
    root.style.setProperty('--theme-accent', state.accent);
    root.style.setProperty('--on-accent', onAccent(state.accent));
  }
  const meta = document.querySelector('meta[name=theme-color]');
  if (meta) meta.content = getComputedStyle(root).getPropertyValue('--bg').trim() || '#0B2347';
  prefs.set('appearance', state);
  prefs.set('theme', root.dataset.theme);      // older code paths read this
  const btn = $('#theme');
  if (btn) { btn.innerHTML = icon('palette'); btn.title = 'Appearance: theme and colours'; }
  if (panel && !panel.hidden) render();
  onChange();
}

function set(change) { state = { ...state, ...change }; apply(); }

function mini(mode, accent) {
  const [bg, side] = SURFACE[resolved(mode)] || SURFACE.light;
  const single = accent !== MULTI;
  const sideTint = single ? `linear-gradient(180deg,color-mix(in srgb,${accent} 22%,${side}),${side})` : side;
  return h('span', { class: 'ap-mini', style: { background: single ? `radial-gradient(60px 40px at 30% 0%,color-mix(in srgb,${accent} 35%,transparent),transparent),${bg}` : bg } },
    h('i', { style: { background: sideTint } }),
    h('u', { style: { background: single ? accent : RAINBOW } }),
    h('s', { style: { background: single ? `color-mix(in srgb,${accent} 45%,${bg})` : 'color-mix(in srgb,#9b6cf6 45%,transparent)' } }));
}

function render() {
  const sameTheme = t => t.mode === state.mode && t.accent.toLowerCase() === String(state.accent).toLowerCase();
  const custom = state.accent !== MULTI && !ACCENTS.some(([c]) => c.toLowerCase() === state.accent.toLowerCase());
  panel.replaceChildren(
    h('div', { class: 'ap-head' }, h('b', {}, 'Appearance'), h('span', { class: 'faint' }, 'saved in this browser')),
    h('h4', {}, 'Mode'),
    h('div', { class: 'ap-modes', role: 'radiogroup', 'aria-label': 'Mode' }, ...MODES.map(m => {
      const preview = m.id === 'system'
        ? h('span', { class: 'ap-mini', style: { background: `linear-gradient(110deg,${SURFACE.light[0]} 50%,${SURFACE.dark[0]} 50%)` } },
          h('i', { style: { background: SURFACE.dark[1] } }))
        : mini(m.id, state.accent);
      return h('button', { class: 'ap-mode' + (state.mode === m.id ? ' on' : ''), role: 'radio', 'aria-checked': String(state.mode === m.id), onclick: () => set({ mode: m.id }) },
        preview, m.name);
    })),
    h('h4', {}, 'Themes'),
    h('div', { class: 'ap-themes' }, ...THEMES.map(t =>
      h('button', { class: 'ap-theme' + (sameTheme(t) ? ' on' : ''), title: `${t.name}: ${MODES.find(m => m.id === t.mode).name}, ${t.accent === MULTI ? 'multicolour' : 'one colour'}`, onclick: () => set({ mode: t.mode, accent: t.accent }) },
        mini(t.mode, t.accent), t.name))),
    h('h4', {}, 'Accent colour'),
    h('button', { class: 'ap-multi' + (state.accent === MULTI ? ' on' : ''), onclick: () => set({ accent: MULTI }) },
      h('span', { class: 'ap-rainbow' }),
      h('span', {}, h('b', {}, 'Multicolour'), h('small', {}, 'Each page and card keeps its own colour'))),
    h('div', { class: 'ap-swatches', role: 'radiogroup', 'aria-label': 'Accent colour' }, ...ACCENTS.map(([c, name]) =>
      h('button', { class: 'ap-sw' + (state.accent.toLowerCase() === c.toLowerCase() ? ' on' : ''), role: 'radio', 'aria-checked': String(state.accent.toLowerCase() === c.toLowerCase()),
        title: name, 'aria-label': name, style: { background: c, color: onAccent(c) }, html: icon('check'), onclick: () => set({ accent: c }) }))),
    h('label', { class: 'ap-custom' }, 'Custom colour',
      h('input', { type: 'color', value: state.accent === MULTI ? '#4f8df7' : state.accent, oninput: e => set({ accent: e.target.value }) }),
      custom ? h('code', {}, state.accent.toUpperCase()) : null));
}

/** Wire the top-bar theme button to the Appearance panel. onResize: called after every change. */
export function setupAppearance({ onResize } = {}) {
  onChange = onResize || (() => {});
  const btn = $('#theme');
  panel = h('section', { class: 'ap-panel', id: 'ap-panel', hidden: true, role: 'dialog', 'aria-label': 'Appearance' });
  btn.after(panel);
  btn.setAttribute('aria-label', 'Appearance');
  btn.onclick = e => {
    e.stopPropagation();
    const notif = $('#notif-panel'); if (notif) notif.hidden = true;
    panel.hidden = !panel.hidden;
    if (!panel.hidden) render();
  };
  panel.addEventListener('click', e => e.stopPropagation());
  document.addEventListener('click', () => { panel.hidden = true; });
  document.addEventListener('keydown', e => { if (e.key === 'Escape') panel.hidden = true; });
  media.addEventListener('change', () => { if (state.mode === 'system') apply(); });
  apply();
}
