// Behaviour-tree viewer: top-to-bottom layout, pan/zoom, collapse, live status colours.
// Display only — it never edits or sends a tree.
import { h, icon } from './core.js';

let catalog = null;
export async function loadCatalog() {
  if (catalog) return catalog;
  try { catalog = (await (await fetch('/data/bt_catalog.json')).json()).nodes || {}; } catch { catalog = {}; }
  return catalog;
}
const CATEGORY_COLORS = { Control: '#8b5cf6', Decorator: '#f59e0b', Action: '#3b82f6', Condition: '#10b981', SubTree: '#ec4899' };
const BUILTIN = { Sequence: 'Control', Fallback: 'Control', ReactiveFallback: 'Control', ReactiveSequence: 'Control',
  SequenceWithMemory: 'Control', Parallel: 'Control', RecoverThenFail: 'Control', Timeout: 'Decorator',
  RetryUntilSuccessful: 'Decorator', ForceSuccess: 'Decorator', ForceFailure: 'Decorator', Inverter: 'Decorator',
  Delay: 'Decorator', Repeat: 'Decorator', StableFor: 'Decorator', SafetyGate: 'Decorator' };
export function categoryOf(type) { return catalog?.[type]?.category || BUILTIN[type] || 'Action'; }

/** XML text -> {type, name, ports, children}. */
export function parseXml(xml) {
  const doc = new DOMParser().parseFromString(xml, 'application/xml');
  if (doc.querySelector('parsererror')) throw new Error('Invalid behaviour tree XML');
  const root = doc.documentElement;
  const main = root.getAttribute('main_tree_to_execute');
  const trees = [...root.querySelectorAll(':scope > BehaviorTree')];
  const tree = trees.find(t => t.getAttribute('ID') === main) || trees[0];
  if (!tree || !tree.firstElementChild) throw new Error('No behaviour tree found in XML');
  let uid = 0;
  const walk = el => {
    const ports = {};
    for (const a of el.attributes) if (a.name !== 'name' && a.name !== 'ID') ports[a.name] = a.value;
    const type = el.getAttribute('ID') && ['Action', 'Condition', 'Decorator', 'Control'].includes(el.tagName) ? el.getAttribute('ID') : el.tagName;
    return { uid: ++uid, type, name: el.getAttribute('name') || '', ports, children: [...el.children].map(walk) };
  };
  return walk(tree.firstElementChild);
}

/** bt_nodes from the onboard executor -> same shape, with live status. */
export function fromLive(nodes) {
  if (!nodes?.length) return null;
  const byId = new Map(nodes.map(n => [n.node_uid, n]));
  const childIds = new Set(nodes.flatMap(n => n.children_uids || []));
  const rootNode = nodes.find(n => !childIds.has(n.node_uid)) || nodes[0];
  const walk = n => ({ uid: n.node_uid, type: n.node_type, name: n.node_name === n.node_type ? '' : n.node_name,
    status: n.status === 'IDLE' ? (n.display_status || '').toUpperCase() || 'IDLE' : n.status,
    category: n.node_category, children: (n.children_uids || []).map(id => byId.get(id)).filter(Boolean).map(walk) });
  return walk(rootNode);
}

export function countNodes(tree) { let c = 0; const w = n => { c++; n.children.forEach(w); }; if (tree) w(tree); return c; }
export function summarize(tree) {
  const counts = { Control: 0, Decorator: 0, Action: 0, Condition: 0 };
  const w = n => { counts[n.category || categoryOf(n.type)] = (counts[n.category || categoryOf(n.type)] || 0) + 1; n.children.forEach(w); };
  if (tree) w(tree);
  return counts;
}

const W = 150, HN = 46, GX = 18, GY = 64;
/** Mount an interactive viewer inside `box`. Returns {update(tree)}. */
export function treeViewer(box, { collapsedDepth = 99 } = {}) {
  box.classList.add('tree-box');
  const svgNS = 'http://www.w3.org/2000/svg';
  const svg = document.createElementNS(svgNS, 'svg');
  const g = document.createElementNS(svgNS, 'g');
  svg.append(g);
  const tools = h('div', { class: 'tree-tools' },
    h('button', { class: 'btn sm', onclick: () => fit() }, 'Fit'),
    h('button', { class: 'btn sm', onclick: () => zoom(1.2) }, '+'),
    h('button', { class: 'btn sm', onclick: () => zoom(1 / 1.2) }, '−'));
  box.replaceChildren(tools, svg);
  let view = { x: 20, y: 20, k: 1 }, tree = null, collapsed = new Set(), size = { w: 0, h: 0 }, fitted = false;
  const apply = () => g.setAttribute('transform', `translate(${view.x},${view.y}) scale(${view.k})`);
  const zoom = f => { const r = box.getBoundingClientRect(); const cx = r.width / 2, cy = r.height / 2;
    view.x = cx - (cx - view.x) * f; view.y = cy - (cy - view.y) * f; view.k *= f; apply(); };
  const fit = () => { const r = box.getBoundingClientRect(); if (!size.w || !r.width) return;
    const k = Math.min(1.3, (r.width - 40) / size.w, (r.height - 60) / size.h);
    view = { k, x: (r.width - size.w * k) / 2, y: 40 }; apply(); };
  let drag = null;
  svg.addEventListener('pointerdown', e => { drag = { x: e.clientX, y: e.clientY }; svg.setPointerCapture(e.pointerId); box.style.cursor = 'grabbing'; });
  svg.addEventListener('pointermove', e => { if (!drag) return; view.x += e.clientX - drag.x; view.y += e.clientY - drag.y; drag = { x: e.clientX, y: e.clientY }; apply(); });
  svg.addEventListener('pointerup', () => { drag = null; box.style.cursor = ''; });
  box.addEventListener('wheel', e => { e.preventDefault(); zoom(e.deltaY > 0 ? 0.9 : 1.1); }, { passive: false });

  function layout(node, depth, next) {
    node._d = depth;
    const kids = collapsed.has(node.uid) ? [] : node.children;
    if (!kids.length) { node._x = next.x; next.x += W + GX; }
    else { kids.forEach(c => layout(c, depth + 1, next)); node._x = (kids[0]._x + kids[kids.length - 1]._x) / 2; }
    node._y = depth * (HN + GY);
    node._kids = kids;
  }
  function draw() {
    g.replaceChildren();
    if (!tree) return;
    const next = { x: 0 }; layout(tree, 0, next);
    let maxY = 0;
    const edges = document.createElementNS(svgNS, 'g'), nodes = document.createElementNS(svgNS, 'g');
    const walk = n => {
      maxY = Math.max(maxY, n._y);
      for (const c of n._kids) {
        const p = document.createElementNS(svgNS, 'path');
        const x1 = n._x + W / 2, y1 = n._y + HN, x2 = c._x + W / 2, y2 = c._y, my = (y1 + y2) / 2;
        p.setAttribute('d', `M${x1},${y1} C${x1},${my} ${x2},${my} ${x2},${y2}`);
        p.setAttribute('class', `edge ${c.status === 'RUNNING' ? 'RUNNING' : ''}`);
        edges.append(p); walk(c);
      }
      const cat = n.category || categoryOf(n.type), color = CATEGORY_COLORS[cat] || '#3b82f6';
      const node = document.createElementNS(svgNS, 'g');
      node.setAttribute('class', `tn ${n.status || ''}`);
      node.setAttribute('transform', `translate(${n._x},${n._y})`);
      node.innerHTML = `<rect class="box" width="${W}" height="${HN}" rx="11"/>
        <rect class="cat" x="0" y="0" width="5" height="${HN}" rx="2.5" fill="${color}"/>
        <text x="14" y="19">${esc(trim(n.name || n.type, 19))}</text>
        <text class="sub" x="14" y="35">${esc(cat)}${n.name ? ' · ' + esc(trim(n.type, 14)) : ''}${n.children.length ? ` · ${n.children.length}` : ''}</text>
        ${n.status && n.status !== 'IDLE' ? `<circle cx="${W - 13}" cy="13" r="5" fill="${n.status === 'SUCCESS' ? '#10b981' : n.status === 'FAILURE' ? '#e5484d' : n.status === 'RUNNING' ? '#f59e0b' : '#8a95ad'}"/>` : ''}
        ${n.children.length ? `<text class="sub" x="${W - 18}" y="38">${collapsed.has(n.uid) ? '＋' : '－'}</text>` : ''}`;
      const title = document.createElementNS(svgNS, 'title');
      title.textContent = `${n.type}${n.name ? ' "' + n.name + '"' : ''}\n${Object.entries(n.ports || {}).map(([k, v]) => `${k} = ${v}`).join('\n')}`;
      node.append(title);
      if (n.children.length) { node.style.cursor = 'pointer'; node.addEventListener('click', e => { e.stopPropagation(); collapsed.has(n.uid) ? collapsed.delete(n.uid) : collapsed.add(n.uid); draw(); }); }
      nodes.append(node);
    };
    walk(tree);
    g.append(edges, nodes);
    size = { w: Math.max(next.x - GX, W), h: maxY + HN };
    if (!fitted) { fitted = true; requestAnimationFrame(fit); }
    apply();
  }
  return {
    update(newTree, { keepView = true } = {}) {
      const sameShape = tree && newTree && countNodes(tree) === countNodes(newTree) && tree.type === newTree.type;
      if (!sameShape) { collapsed = new Set(); fitted = false;
        const collapseDeep = n => { if (n._depthHint >= collapsedDepth) collapsed.add(n.uid); n.children.forEach(c => { c._depthHint = (n._depthHint || 0) + 1; collapseDeep(c); }); };
        if (newTree) collapseDeep(newTree); }
      tree = newTree; draw(); if (!keepView) fit();
    },
    fit,
  };
}
const esc = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const trim = (s, n) => (s.length > n ? s.slice(0, n - 1) + '…' : s);
export function legend() {
  return h('div', { class: 'row', style: { gap: '14px', fontSize: '12px', marginTop: '8px' } },
    ...Object.entries(CATEGORY_COLORS).slice(0, 4).map(([k, c]) => h('span', { class: 'row', style: { gap: '6px' } },
      h('span', { style: { width: '10px', height: '10px', borderRadius: '3px', background: c, display: 'inline-block' } }), k)),
    h('span', { class: 'faint' }, 'Click a node with children to fold it · drag to pan · wheel to zoom'));
}
