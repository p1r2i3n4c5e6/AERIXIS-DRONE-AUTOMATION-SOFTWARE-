/**
 * Behavior Tree Live Log & Execution Trace Viewer
 */

let btLogsCache = [];
let btLogFilter = 'ALL';
let btLogSearch = '';
let btAutoScroll = true;

function renderBtLogs(logs) {
  if (Array.isArray(logs)) {
    btLogsCache = logs;
  }
  const container = document.getElementById('bt-log-table-body');
  if (!container) return;

  const filtered = btLogsCache.filter(item => {
    // Level / type filter
    if (btLogFilter === 'TICKS' && !item.message.toLowerCase().includes('bt') && !item.message.toLowerCase().includes('tree')) return false;
    if (btLogFilter === 'ERRORS' && item.severity !== 'ERROR' && item.severity !== 'WARN') return false;
    if (btLogFilter === 'ACTIONS' && !item.message.toLowerCase().includes('action') && !item.message.toLowerCase().includes('waypoint') && !item.message.toLowerCase().includes('takeoff') && !item.message.toLowerCase().includes('land')) return false;

    // Search query
    if (btLogSearch) {
      const q = btLogSearch.toLowerCase();
      return (item.message && item.message.toLowerCase().includes(q)) ||
             (item.time && item.time.toLowerCase().includes(q)) ||
             (item.severity && item.severity.toLowerCase().includes(q));
    }
    return true;
  });

  container.replaceChildren();

  filtered.forEach(log => {
    const tr = document.createElement('tr');
    tr.className = `log-row ${log.severity || 'INFO'}`;

    // Time
    const tdTime = document.createElement('td');
    tdTime.className = 'log-col-time';
    tdTime.textContent = log.time || '--:--:--';

    // Badge
    const tdBadge = document.createElement('td');
    tdBadge.className = 'log-col-badge';
    const badge = document.createElement('span');
    badge.className = `log-badge ${log.severity || 'INFO'}`;
    badge.textContent = log.severity || 'INFO';
    tdBadge.appendChild(badge);

    // Tag / Action
    const tdTag = document.createElement('td');
    tdTag.className = 'log-col-tag';
    if (log.message.includes('Takeoff') || log.message.includes('Taking Off')) {
      tdTag.innerHTML = '<span class="tag-pill tag-takeoff">🛫 Takeoff</span>';
    } else if (log.message.includes('Land') || log.message.includes('Landing')) {
      tdTag.innerHTML = '<span class="tag-pill tag-land">🛬 Land</span>';
    } else if (log.message.includes('waypoint') || log.message.includes('GoTo')) {
      tdTag.innerHTML = '<span class="tag-pill tag-nav">📍 GoTo</span>';
    } else if (log.message.includes('Offboard')) {
      tdTag.innerHTML = '<span class="tag-pill tag-offboard">⚡ Offboard</span>';
    } else if (log.message.includes('Arm')) {
      tdTag.innerHTML = '<span class="tag-pill tag-arm">🛡️ Arm</span>';
    } else {
      tdTag.innerHTML = '<span class="tag-pill">BT Engine</span>';
    }

    // Message
    const tdMsg = document.createElement('td');
    tdMsg.className = 'log-col-msg';
    tdMsg.textContent = log.message;

    tr.append(tdTime, tdBadge, tdTag, tdMsg);
    container.appendChild(tr);
  });

  const countBadge = document.getElementById('bt-log-total-count');
  if (countBadge) countBadge.textContent = `${filtered.length} / ${btLogsCache.length} events`;

  if (btAutoScroll) {
    const scrollBox = document.getElementById('bt-log-scroll-container');
    if (scrollBox) scrollBox.scrollTop = scrollBox.scrollHeight;
  }
}

function setBtLogFilter(filterName) {
  btLogFilter = filterName;
  ['ALL', 'TICKS', 'ACTIONS', 'ERRORS'].forEach(f => {
    const btn = document.getElementById(`bt-filter-btn-${f.toLowerCase()}`);
    if (btn) btn.classList.toggle('active', f === filterName);
  });
  renderBtLogs();
}

function filterBtLogsSearch(query) {
  btLogSearch = query;
  renderBtLogs();
}

function toggleBtAutoScroll() {
  btAutoScroll = !btAutoScroll;
  const btn = document.getElementById('bt-autoscroll-toggle');
  if (btn) {
    btn.classList.toggle('active', btAutoScroll);
    btn.textContent = btAutoScroll ? '⬇️ Auto-Scroll: ON' : '⏸️ Auto-Scroll: OFF';
  }
}

function clearBtLogs() {
  btLogsCache = [];
  renderBtLogs();
}

function exportBtLogs() {
  const text = btLogsCache.map(l => `[${l.time}] [${l.severity}] ${l.message}`).join('\n');
  const blob = new Blob([text], { type: 'text/plain' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `as2_behavior_tree_${Date.now()}.log`;
  a.click();
}
