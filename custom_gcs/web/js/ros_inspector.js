/**
 * ROS 2 Stack & Controller Monitor (Interactive Window)
 * Fetches /api/ros/nodes and updates pills with green (running), blue (offline), red (error).
 */

let rosMonitorTimer = null;

function openRosInspector() {
  const modal = document.getElementById('ros-inspector-modal');
  if (modal) {
    modal.classList.add('open');
    fetchRosStatus();
    if (!rosMonitorTimer) {
      rosMonitorTimer = setInterval(fetchRosStatus, 1500);
    }
  }
}

function closeRosInspector() {
  const modal = document.getElementById('ros-inspector-modal');
  if (modal) {
    modal.classList.remove('open');
    if (rosMonitorTimer) {
      clearInterval(rosMonitorTimer);
      rosMonitorTimer = null;
    }
  }
}

async function fetchRosStatus() {
  try {
    const res = await fetch('/api/ros/nodes');
    if (!res.ok) throw new Error('HTTP ' + res.status);
    const data = await res.json();
    renderRosPills(data);
  } catch (err) {
    const statusText = document.getElementById('ros-modal-status-text');
    if (statusText) {
      statusText.textContent = 'ROS: Link offline';
      statusText.style.color = '#ef4444';
    }
  }
}

function renderRosPills(data) {
  const statusText = document.getElementById('ros-modal-status-text');
  if (statusText && data.summary) {
    statusText.textContent = `ROS 2 Active: ${data.summary.running_nodes} Nodes · ${data.summary.running_controllers} Controllers`;
    statusText.style.color = data.summary.running_nodes > 0 ? '#38bdf8' : '#f59e0b';
  }

  // 1. Render Nodes Grid
  const nodesGrid = document.getElementById('ros-nodes-grid');
  if (nodesGrid && Array.isArray(data.nodes)) {
    nodesGrid.replaceChildren();
    data.nodes.forEach(node => {
      const pill = document.createElement('div');
      pill.className = `ros-pill ${node.status || 'offline'}`;
      pill.title = `Type: ${node.type || 'Node'} | Status: ${node.status.toUpperCase()}`;
      
      const nameSpan = document.createElement('span');
      nameSpan.textContent = node.name;
      
      const dot = document.createElement('span');
      dot.style.fontSize = '9px';
      dot.textContent = node.status === 'running' ? '●' : (node.status === 'error' ? '▲' : '○');

      pill.append(nameSpan, dot);
      nodesGrid.appendChild(pill);
    });
  }

  // 2. Render Controllers Grid
  const ctrlGrid = document.getElementById('ros-controllers-grid');
  if (ctrlGrid && Array.isArray(data.controllers)) {
    ctrlGrid.replaceChildren();
    data.controllers.forEach(ctrl => {
      const pill = document.createElement('div');
      pill.className = `ros-pill ${ctrl.status || 'offline'}`;
      pill.title = `Controller: ${ctrl.name} | Status: ${ctrl.status.toUpperCase()}`;
      
      const nameSpan = document.createElement('span');
      nameSpan.textContent = ctrl.name;
      
      const dot = document.createElement('span');
      dot.style.fontSize = '9px';
      dot.textContent = ctrl.status === 'running' ? '●' : (ctrl.status === 'error' ? '▲' : '○');

      pill.append(nameSpan, dot);
      ctrlGrid.appendChild(pill);
    });
  }
}
