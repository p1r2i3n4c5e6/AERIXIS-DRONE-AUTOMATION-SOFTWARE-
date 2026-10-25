function showGcsError(message) { if (window.FleetUI) window.FleetUI.error(message, 'Local telemetry link'); else alert(message); }
/**
 * Custom GCS Main Application Controller
 */

let mapInstance = null;
let droneMarkerLayer = null;
let droneTrailLayer = null;
let droneCoordinatesTrail = [];

// ==========================================
// 1. VIEW SWITCHING
// ==========================================
function switchView(viewKey) {
  document.body.classList.toggle('fleet-view', ['fleet','designer'].includes(viewKey));
  ['fleet', 'designer', 'map', 'tree', 'logs', 'xml'].forEach(k => {
    const el = document.getElementById('view-' + k);
    const btn = document.getElementById('tab-' + k + '-btn');
    if (el) el.classList.remove('active');
    if (btn) btn.classList.remove('active');
  });
  const targetEl = document.getElementById('view-' + viewKey);
  const targetBtn = document.getElementById('tab-' + viewKey + '-btn');
  if (targetEl) targetEl.classList.add('active');
  if (targetBtn) targetBtn.classList.add('active');

  if (viewKey === 'designer' && window.MissionDesigner) {
    window.MissionDesigner.resize();
  } else if (viewKey === 'tree') {
    setTimeout(() => {
      if (!btThreeInitialized) {
        initThreeBtStudio();
      } else {
        onBtResize();
        rebuild3DBtTree();
      }
    }, 50);
  } else if (viewKey === 'map' && mapInstance) {
    setTimeout(() => {
      try { mapInstance.invalidateSize(); } catch (e) {}
    }, 50);
  } else if (viewKey === 'fleet' && window.FleetUI) {
    window.FleetUI.refresh();
  } else if (viewKey === 'logs') {
    renderBtLogs();
  }
}

// ==========================================
// 2. LEAFLET MAP CONTROLLER
// ==========================================
function initLeafletMap() {
  if (mapInstance) return;
  try {
    if (typeof L === 'undefined') {
      console.warn('Leaflet is not defined');
      return;
    }
    const container = document.getElementById('map');
    if (!container) return;

    mapInstance = L.map('map', {
      zoomControl: true,
      attributionControl: true
    }).setView([20, 0], 2);

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 20,
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap contributors</a>'
    }).addTo(mapInstance);

    // Live aircraft layers are presentation-owned and require fresh GPS.
    try { window.GCSFleetMap?.attach(mapInstance); } catch {}

    // Click handler for drawing KML polygon area or clicking waypoints
    mapInstance.on('click', function(e) {
      if (kmlMapMode === 'draw_area') {
        kmlPolygonPoints.push([e.latlng.lat, e.latlng.lng]);
        redrawKmlLayers();
      } else if (kmlMapMode === 'add_waypoints') {
        kmlWaypoints.push([e.latlng.lat, e.latlng.lng]);
        redrawKmlLayers();
        updateKmlBadge();
        syncKmlToBehaviorTree();
      }
    });

    setTimeout(() => {
      try { mapInstance.invalidateSize(); } catch (e) {}
    }, 150);
  } catch (err) {
    console.warn('Map initialization notice:', err);
  }
}

function updateLeafletDrone(lat, lon) {
  try {
    if (!mapInstance) initLeafletMap();
    if (!mapInstance || typeof lat !== 'number' || typeof lon !== 'number') return;
    if (droneMarkerLayer) droneMarkerLayer.setLatLng([lat, lon]);
    droneCoordinatesTrail.push([lat, lon]);
    if (droneCoordinatesTrail.length > 500) droneCoordinatesTrail.shift();
    if (droneTrailLayer) droneTrailLayer.setLatLngs(droneCoordinatesTrail);
  } catch (e) {
    console.warn('Map update notice:', e);
  }
}

// ==========================================
// 3. FLIGHT MODES & COMMANDS
// ==========================================
async function sendMode(mode) {
  try {
    const res = await fetch('/api/mode', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode })
    });
    const data = await res.json();
    if (data.ok) {
      updateModeUI(data.mode);
    } else if (data.error) {
      showGcsError('Mode switch failed: ' + data.error);
    }
  } catch (e) {
    showGcsError('Failed to send mode switch to server.');
  }
}

function updateModeUI(modeName) {
  const mode = (modeName || 'UNKNOWN').toUpperCase();
  ['offboard', 'position', 'altitude', 'manual'].forEach(m => {
    const btn = document.getElementById(`mode-${m}-btn`);
    if (btn) btn.classList.toggle('active', mode === m.toUpperCase() || (m === 'position' && mode === 'POSCTL'));
  });

  const isOffboard = (mode === 'OFFBOARD');
  const dispMode = document.getElementById('disp-mode-text');
  const hudMode = document.getElementById('hud-mode');
  const dispAs2 = document.getElementById('disp-as2-text');
  const authBadge = document.getElementById('authority-badge');
  const warnBox = document.getElementById('mode-warning-box');

  if (dispMode) dispMode.textContent = mode;
  if (hudMode) hudMode.textContent = mode;

  if (dispAs2) {
    dispAs2.textContent = isOffboard ? 'ENABLED' : 'LOCKED (INACTIVE)';
    dispAs2.style.color = isOffboard ? 'var(--green)' : 'var(--amber)';
  }
  if (authBadge) {
    authBadge.textContent = isOffboard ? 'OFFBOARD ACTIVE' : 'MANUAL HOLD';
    authBadge.style.color = isOffboard ? 'var(--green)' : 'var(--amber)';
  }
  if (warnBox) {
    warnBox.classList.toggle('visible', !isOffboard);
  }

  // Interlock AERIXIS action buttons
  document.querySelectorAll('.as2-action-btn').forEach(btn => {
    btn.disabled = !isOffboard;
  });
}

async function sendCmd(action, alt) {
  try {
    const res = await fetch('/api/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ action, altitude: alt })
    });
    const data = await res.json();
    if (!data.ok && data.error) {
      showGcsError('Command error: ' + data.error);
    }
  } catch (e) {
    showGcsError('Failed to send flight command.');
  }
}

// ==========================================
// 4. ONE-CLICK AUTONOMOUS DEMO RUNNER
// ==========================================
async function runAutonomousDemo() {
  await executeCurrentMission();
}

async function executeCurrentMission() {
  switchView('fleet');
  if (window.FleetUI) window.FleetUI.useStudio();
}

// ==========================================
// 5. CONNECTION MODAL
// ==========================================
function openConnModal() { document.getElementById('conn-modal').classList.add('open'); refreshPorts(); }
function closeConnModal() { document.getElementById('conn-modal').classList.remove('open'); }
function setModalTab(tab) {
  document.getElementById('modal-tab-serial').className = (tab === 'serial' ? 'primary' : '');
  document.getElementById('modal-tab-udp').className = (tab === 'udp' ? 'primary' : '');
  document.getElementById('modal-pane-serial').style.display = (tab === 'serial' ? 'flex' : 'none');
  document.getElementById('modal-pane-udp').style.display = (tab === 'udp' ? 'flex' : 'none');
}

async function refreshPorts() {
  const sel = document.getElementById('serial-port-select');
  if (!sel) return;
  sel.innerHTML = '<option>Scanning available ports...</option>';
  try {
    const res = await fetch('/api/ports');
    const data = await res.json();
    sel.innerHTML = '';
    if (data.serial_ports && data.serial_ports.length > 0) {
      data.serial_ports.forEach(p => {
        const opt = document.createElement('option');
        opt.value = p.device;
        opt.textContent = `${p.device} (${p.description || 'Serial Radio'})`;
        sel.appendChild(opt);
      });
    } else {
      sel.innerHTML = '<option value="/dev/ttyUSB0">/dev/ttyUSB0 (Telemetry Radio)</option><option value="/dev/ttyACM0">/dev/ttyACM0 (Pixhawk USB)</option>';
    }
  } catch (e) {
    sel.innerHTML = '<option value="/dev/ttyUSB0">/dev/ttyUSB0 (Telemetry Radio)</option>';
  }
}

async function connectSerial() {
  const port = document.getElementById('serial-port-select').value;
  const baud = parseInt(document.getElementById('serial-baud-select').value, 10);
  const res = await fetch('/api/connect', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ type: 'serial', port, baud })
  });
  const data = await res.json();
  if (data.ok) {
    closeConnModal();
    document.getElementById('conn-summary-text').textContent = `Serial: ${port} @ ${baud}`;
  } else {
    showGcsError('Failed to connect: ' + (data.error || 'Port busy or not found'));
  }
}

async function connectUdp() {
  const host = document.getElementById('udp-host').value;
  const port = parseInt(document.getElementById('udp-port').value, 10);
  const listen_port = parseInt(document.getElementById('udp-listen').value, 10);
  const res = await fetch('/api/connect', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ type: 'udp', host, port, listen_port })
  });
  const data = await res.json();
  if (data.ok) {
    closeConnModal();
    document.getElementById('conn-summary-text').textContent = `UDP: ${host}:${port}`;
  }
}

// ==========================================
// 6. LIVE STATE POLLING
// ==========================================
async function pollState() {
  try {
    const res = await fetch('/api/state');
    if (!res.ok) throw new Error('HTTP ' + res.status);
    const s = await res.json();
    try { window.GCSFleetMap?.updateLocal(s); } catch {}
    try { window.GCSLinkDisplay?.updateLocal(s); } catch (error) { console.warn('Link display unavailable', error); }

    // 1. Flight mode UI
    updateModeUI(s.flight_mode || 'UNKNOWN');

    // 2. Armed state
    const armedText = document.getElementById('armed-text');
    const armedBadge = document.getElementById('armed-badge');
    if (armedText) {
      armedText.textContent = s.armed ? 'ARMED' : 'DISARMED';
      armedText.style.color = s.armed ? 'var(--green)' : 'var(--muted)';
    }
    if (armedBadge) {
      armedBadge.textContent = s.armed ? 'ARMED' : 'DISARMED';
      armedBadge.style.color = s.armed ? 'var(--green)' : 'var(--muted)';
    }

    // 3. Altitude
    const altVal = (typeof s.altitude === 'number' ? s.altitude : 0).toFixed(2) + ' m';
    const altText = document.getElementById('alt-text');
    const hudAlt = document.getElementById('hud-alt');
    if (altText) altText.textContent = altVal;
    if (hudAlt) hudAlt.textContent = altVal;

    // 4. GPS & Map
    if (typeof s.latitude === 'number' && typeof s.longitude === 'number') {
      const gpsText = document.getElementById('gps-text');
      if (gpsText) gpsText.textContent = `${s.latitude.toFixed(5)}, ${s.longitude.toFixed(5)}`;
      // Fleet map consumes the full state so unconfirmed GPS is never shown as live.
    }

    // Traffic / Data Transfer Card
    if (s.traffic) {
      const txKb = document.getElementById('disp-tx-kb');
      const rxKb = document.getElementById('disp-rx-kb');
      const txPkts = document.getElementById('disp-tx-packets');
      const rxPkts = document.getElementById('disp-rx-packets');
      const txRate = document.getElementById('tx-rate-badge');
      const rxRate = document.getElementById('rx-rate-badge');
      const xmlSize = document.getElementById('disp-bt-xml-size');
      const hbHz = document.getElementById('disp-hb-hz');

      if (txKb) txKb.innerHTML = (s.traffic.tx_kb || 0).toFixed(1) + ' <span style="font-size:11px;font-weight:normal;color:#94a3b8;">KB</span>';
      if (rxKb) rxKb.innerHTML = (s.traffic.rx_kb || 0).toFixed(1) + ' <span style="font-size:11px;font-weight:normal;color:#94a3b8;">KB</span>';
      if (txPkts) txPkts.textContent = (s.traffic.tx_packets || 0).toLocaleString() + ' pkts sent';
      if (rxPkts) rxPkts.textContent = (s.traffic.rx_packets || 0).toLocaleString() + ' pkts rcvd';
      if (txRate) txRate.textContent = (s.traffic.tx_rate || 0).toFixed(1) + ' KB/s';
      if (rxRate) rxRate.textContent = (s.traffic.rx_rate || 0).toFixed(1) + ' KB/s';
      if (xmlSize) xmlSize.textContent = s.traffic.bt_xml_bytes ? ((s.traffic.bt_xml_bytes / 1024).toFixed(1) + ' KB') : '0.0 KB';
      if (hbHz) hbHz.textContent = (s.heartbeat_hz || 0).toFixed(1) + ' Hz (' + (s.heartbeat_age < 7 ? 'Active' : 'Offline') + ')';
    }


    // 5. Link statuses
    const as2Conn = !!(s.as2_link && s.as2_link.connected);
    const px4Conn = !!(s.px4_link && s.px4_link.connected);
    const gwLink = document.getElementById('gw-link-val');
    const px4Link = document.getElementById('px4-link-val');
    const hudLink = document.getElementById('hud-link');
    const stackBadge = document.getElementById('stack-overall-badge');

    if (gwLink) {
      gwLink.textContent = as2Conn ? 'ONLINE' : 'OFFLINE';
      gwLink.style.color = as2Conn ? 'var(--green)' : 'var(--red)';
    }
    if (px4Link) {
      px4Link.textContent = px4Conn ? 'ONLINE' : 'OFFLINE';
      px4Link.style.color = px4Conn ? 'var(--green)' : 'var(--red)';
    }
    if (hudLink) {
      hudLink.textContent = (s.connected || as2Conn) ? 'ONLINE' : 'DISCONNECTED';
      hudLink.style.color = (s.connected || as2Conn) ? 'var(--green)' : 'var(--muted)';
    }
    if (stackBadge) {
      const allOk = (s.connected && as2Conn && px4Conn);
      stackBadge.textContent = allOk ? 'CONNECTED' : (s.connected ? 'PARTIAL' : 'DISCONNECTED');
      stackBadge.style.color = allOk ? 'var(--green)' : (s.connected ? 'var(--amber)' : 'var(--red)');
    }

    // Update Animated Decoupled 3-Tier Pipeline Card
    try { updatePipelineCard(s); } catch (e) { console.warn('Pipeline update error:', e); }

    // 6. Mission state
    const missionBadge = document.getElementById('mission-status-badge');
    const missionMsg = document.getElementById('mission-msg-text');
    const missionSeq = document.getElementById('mission-seq-text');
    if (missionBadge) {
      missionBadge.textContent = s.mission_active ? 'EXECUTING' : 'STANDBY';
      missionBadge.style.color = s.mission_active ? 'var(--cyan)' : 'var(--muted)';
    }
    if (missionMsg) missionMsg.textContent = s.mission_message || 'Idle';
    if (missionSeq) missionSeq.textContent = s.mission_total ? `${s.mission_sequence} of ${s.mission_total}` : 'None';

    // Live Step-by-Step BT Mission Tracker
    if (Array.isArray(s.mission_steps) && s.mission_steps.length > 0) {
      const container = document.getElementById('live-steps-container');
      if (container) {
        container.replaceChildren();
        s.mission_steps.forEach((st, idx) => {
          const div = document.createElement('div');
          div.className = `live-step-item ${st.status || 'pending'}`;
          
          const left = document.createElement('span');
          left.style.display = 'flex';
          left.style.alignItems = 'center';
          left.style.gap = '6px';
          left.textContent = `${st.icon || '📍'} ${idx + 1}. ${st.name}`;

          const right = document.createElement('span');
          right.className = 'step-badge';
          if (st.status === 'running') right.textContent = 'RUNNING ⚡';
          else if (st.status === 'success') right.textContent = 'DONE ✓';
          else right.textContent = ({failed:'FAILED',cancelled:'CANCELLED',unknown:'UNKNOWN',skipped:'SKIPPED'})[st.status] || 'WAITING';

          div.append(left, right);
          container.appendChild(div);
        });
      }

      // Update progress bar
      const totalSteps = s.mission_steps.length;
      const completedSteps = s.mission_steps.filter(st => st.status === 'success').length;
      const currentStep = Math.min(totalSteps, (s.active_step_index !== undefined ? s.active_step_index + 1 : completedSteps + 1));
      const pct = Math.round((completedSteps / totalSteps) * 100);

      const bar = document.getElementById('bt-progress-bar');
      const stepTxt = document.getElementById('bt-progress-step-text');
      const pctTxt = document.getElementById('bt-progress-pct-text');
      if (bar) bar.style.width = `${pct}%`;
      if (stepTxt) stepTxt.textContent = s.mission_active ? `Step ${currentStep} of ${totalSteps}` : (completedSteps === totalSteps ? 'Mission Completed ✓' : `Plan Ready (${totalSteps} Steps)`);
      if (pctTxt) pctTxt.textContent = `${pct}%`;
    }

    // Update Planned Trajectory on Map from server state
    if (Array.isArray(s.trajectory) && s.trajectory.length > 0 && (!kmlWaypoints || kmlWaypoints.length === 0)) {
      kmlWaypoints = s.trajectory.map(p => [p.lat, p.lon]);
      redrawKmlLayers();
      updateKmlBadge();
    }

    // Sync with 3D Behavior Tree Studio active node
    if (typeof activeExecutingNodeIndex !== 'undefined' && s.active_step_index !== undefined) {
      if (activeExecutingNodeIndex !== s.active_step_index) {
        activeExecutingNodeIndex = s.active_step_index;
        if (typeof rebuild3DBtTree === 'function' && btThreeInitialized) {
          rebuild3DBtTree();
        }
        if (typeof renderBtNodeList === 'function') {
          renderBtNodeList();
        }
      }
    }

    // 7. System & BT Logs
    if (Array.isArray(s.logs)) {
      const logBox = document.getElementById('console-list');
      if (logBox) {
        logBox.replaceChildren();
        s.logs.slice(-25).reverse().forEach(l => {
          const li = document.createElement('li');
          li.className = l.severity || 'INFO';
          li.textContent = `${l.time || ''} ${l.message || ''}`;
          logBox.append(li);
        });
      }
      const logCount = document.getElementById('log-count');
      if (logCount) logCount.textContent = `${s.logs.length} messages`;

      // Update dedicated BT Log Viewer
      renderBtLogs(s.logs);
    }

    // 8. Header pill dot
    const mavDot = document.getElementById('mav-dot');
    if (mavDot) mavDot.className = 'dot' + (s.connected ? ' live' : ' warn');

  } catch (err) {
    try { window.GCSFleetMap?.localUnavailable(); } catch {}
    try { window.GCSLinkDisplay?.localUnavailable(); } catch {}
    const mavDot = document.getElementById('mav-dot');
    if (mavDot) mavDot.className = 'dot';
  }
}


// ==========================================
// SIMULATION MODE CONTROLLER
// ==========================================
let isSimulationModeActive = true;

async function onSimulationToggleChanged(enableSim) {
  isSimulationModeActive = enableSim;
  const pill = document.getElementById('sim-toggle-pill');
  const title = document.getElementById('sim-mode-status-text');
  const sub = document.getElementById('sim-mode-sub-text');
  const connSummary = document.getElementById('conn-summary-text');

  if (enableSim) {
    if (pill) pill.classList.remove('sim-off');
    if (title) { title.textContent = 'SIMULATION: ON'; title.style.color = '#34d399'; }
    if (sub) sub.textContent = 'Connected to SITL / Local AERIXIS';
    try {
      await fetch('/api/connect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ type: 'udp', host: '127.0.0.1', port: 14550, listen_port: 14551 })
      });
      if (connSummary) connSummary.textContent = 'SITL: 127.0.0.1:14550';
    } catch (e) {
      console.warn('Connect to SITL notice:', e);
    }
  } else {
    if (pill) pill.classList.add('sim-off');
    if (title) { title.textContent = 'HARDWARE / RADIO: ACTIVE'; title.style.color = 'var(--amber)'; }
    if (sub) sub.textContent = 'Disconnected from SITL';
    try {
      await fetch('/api/disconnect', { method: 'POST' });
      if (connSummary) connSummary.textContent = 'Disconnected (Select Radio Port)';
    } catch (e) {
      console.warn('Disconnect notice:', e);
    }
    // Prompt user to pick real telemetry radio device
    openConnModal();
    setModalTab('serial');
  }
}

// =============================================================================
// POLYGON AREA COVERAGE (LAWNMOWER / BOUSTROPHEDON) TRAJECTORY PLANNER
// Automatically generates internal scanning sweeps inside any KML boundary fence
// =============================================================================
function generateAreaCoverageGrid(polyLatLngs, spacingMeters = 6.0, alt = 2.5) {
  if (!polyLatLngs || polyLatLngs.length < 3) return { enu: [], gps: [] };

  const homeLat = 47.397742;
  const homeLon = 8.545594;
  const R = 6378137.0;

  // 1. Convert GPS polygon to local metric ENU coordinates
  const polyEnu = polyLatLngs.map(p => {
    const lat = p[0];
    const lon = p[1];
    const dLat = (lat - homeLat) * (Math.PI / 180.0);
    const dLon = (lon - homeLon) * (Math.PI / 180.0);
    const north = dLat * R;
    const east = dLon * R * Math.cos(homeLat * (Math.PI / 180.0));
    return { x: east, y: north };
  });

  // Ensure closed ring
  const closed = [...polyEnu];
  const first = closed[0];
  const last = closed[closed.length - 1];
  if (Math.hypot(first.x - last.x, first.y - last.y) > 0.05) {
    closed.push({ x: first.x, y: first.y });
  }

  const xs = closed.map(p => p.x);
  const ys = closed.map(p => p.y);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);

  const gridPtsEnu = [];
  let y = minY + spacingMeters * 0.6;
  let direction = 0; // 0: left->right, 1: right->left

  // 2. Generate parallel sweep scan lines intersecting polygon edges
  while (y <= maxY) {
    const intersections = [];
    for (let i = 0; i < closed.length - 1; i++) {
      const p1 = closed[i];
      const p2 = closed[i + 1];
      if ((p1.y <= y && y <= p2.y) || (p2.y <= y && y <= p1.y)) {
        if (Math.abs(p2.y - p1.y) > 1e-6) {
          const xInt = p1.x + (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y);
          intersections.push(xInt);
        }
      }
    }

    intersections.sort((a, b) => a - b);

    // 3. Pair intersections to create continuous boustrophedon sweep line segments
    for (let j = 0; j < intersections.length - 1; j += 2) {
      const xLeft = intersections[j];
      const xRight = intersections[j + 1];
      if (xRight - xLeft > 1.2) { // Minimum sweep segment length
        if (direction === 0) {
          gridPtsEnu.push({ x: Math.round(xLeft * 10) / 10, y: Math.round(y * 10) / 10, z: alt });
          gridPtsEnu.push({ x: Math.round(xRight * 10) / 10, y: Math.round(y * 10) / 10, z: alt });
        } else {
          gridPtsEnu.push({ x: Math.round(xRight * 10) / 10, y: Math.round(y * 10) / 10, z: alt });
          gridPtsEnu.push({ x: Math.round(xLeft * 10) / 10, y: Math.round(y * 10) / 10, z: alt });
        }
        direction = 1 - direction;
      }
    }
    y += spacingMeters;
  }

  // 4. Convert scan waypoints back to GPS latitude/longitude
  const scanGps = gridPtsEnu.map(p => {
    const dLat = (p.y / R) * (180.0 / Math.PI);
    const dLon = (p.x / (R * Math.cos(homeLat * (Math.PI / 180.0)))) * (180.0 / Math.PI);
    return [
      Math.round((homeLat + dLat) * 1e7) / 1e7,
      Math.round((homeLon + dLon) * 1e7) / 1e7
    ];
  });

  return { enu: gridPtsEnu, gps: scanGps };
}

function generateAreaCoverageFromBoundary() {
  if (!kmlPolygonPoints || kmlPolygonPoints.length < 3) {
    alert('Please draw a fence boundary on the map (min 3 points) or load a KML boundary first.');
    return;
  }
  const spacing = parseFloat(document.getElementById('survey-spacing')?.value) || 6.0;
  const alt = parseFloat(document.getElementById('mission-alt')?.value) || 2.5;

  const res = generateAreaCoverageGrid(kmlPolygonPoints, spacing, alt);
  if (res.gps.length > 0) {
    kmlWaypoints = res.gps;
    redrawKmlLayers();
    updateKmlBadge();
    syncKmlToBehaviorTree();
    alert(`Generated ${kmlWaypoints.length} Lawnmower Scan Waypoints inside the boundary (${spacing}m lane spacing)!`);
  } else {
    alert('Could not generate scan lines inside this boundary. Try making the boundary larger or reducing lane spacing.');
  }
}

function autoRegenerateCoverageIfActive() {
  if (kmlPolygonPoints.length >= 3 && kmlWaypoints.length > 0) {
    const spacing = parseFloat(document.getElementById('survey-spacing')?.value) || 6.0;
    const alt = parseFloat(document.getElementById('mission-alt')?.value) || 2.5;
    const res = generateAreaCoverageGrid(kmlPolygonPoints, spacing, alt);
    if (res.gps.length > 0) {
      kmlWaypoints = res.gps;
      redrawKmlLayers();
      updateKmlBadge();
      syncKmlToBehaviorTree();
    }
  }
}

// ==========================================
// KML FILE & MISSION PLANNER
// ==========================================
let kmlPolygonPoints = [];
let kmlWaypoints = [];
let kmlMapMode = ''; // '' | 'draw_area' | 'add_waypoints'
let kmlPolygonLayer = null;
let kmlRouteLayer = null;
let kmlWaypointMarkers = [];

function toggleDrawKmlArea() {
  if (kmlMapMode === 'draw_area') {
    kmlMapMode = '';
    const btn = document.getElementById('draw-area-btn');
    if (btn) btn.classList.remove('primary');
  } else {
    kmlMapMode = 'draw_area';
    const btn1 = document.getElementById('draw-area-btn');
    const btn2 = document.getElementById('add-waypoints-btn');
    if (btn1) btn1.classList.add('primary');
    if (btn2) btn2.classList.remove('primary');
  }
}

function toggleAddKmlWaypoints() {
  if (kmlMapMode === 'add_waypoints') {
    kmlMapMode = '';
    const btn = document.getElementById('add-waypoints-btn');
    if (btn) btn.classList.remove('primary');
  } else {
    kmlMapMode = 'add_waypoints';
    const btn1 = document.getElementById('add-waypoints-btn');
    const btn2 = document.getElementById('draw-area-btn');
    if (btn1) btn1.classList.add('primary');
    if (btn2) btn2.classList.remove('primary');
  }
}

function clearKmlPlan() {
  kmlPolygonPoints = [];
  kmlWaypoints = [];
  kmlMapMode = '';
  const btn1 = document.getElementById('draw-area-btn');
  const btn2 = document.getElementById('add-waypoints-btn');
  if (btn1) btn1.classList.remove('primary');
  if (btn2) btn2.classList.remove('primary');
  redrawKmlLayers();
  updateKmlBadge();
}

function updateKmlBadge() {
  const badge = document.getElementById('kml-wp-count-badge');
  if (badge) {
    if (kmlPolygonPoints.length >= 3 && kmlWaypoints.length > 0) {
      badge.textContent = `${kmlPolygonPoints.length} Fence · ${kmlWaypoints.length} Scan WPs`;
    } else if (kmlPolygonPoints.length >= 3) {
      badge.textContent = `${kmlPolygonPoints.length} Fence Vertices`;
    } else {
      badge.textContent = `${kmlWaypoints.length} Waypoints`;
    }
  }
}

function redrawKmlLayers() {
  if (!mapInstance) return;

  if (kmlPolygonLayer) {
    try { mapInstance.removeLayer(kmlPolygonLayer); } catch(e){}
    kmlPolygonLayer = null;
  }
  if (kmlRouteLayer) {
    try { mapInstance.removeLayer(kmlRouteLayer); } catch(e){}
    kmlRouteLayer = null;
  }
  kmlWaypointMarkers.forEach(m => {
    try { mapInstance.removeLayer(m); } catch(e){}
  });
  kmlWaypointMarkers = [];

  // Redraw Geofence Area Boundary Polygon
  if (kmlPolygonPoints.length >= 3) {
    kmlPolygonLayer = L.polygon(kmlPolygonPoints, {
      color: '#10b981', // Emerald green fence
      weight: 2.5,
      dashArray: '6, 6',
      fillColor: '#10b981',
      fillOpacity: 0.12
    }).addTo(mapInstance);
    kmlPolygonLayer.bindPopup(`<b>Survey Geofence Area Boundary</b><br>${kmlPolygonPoints.length} Vertices<br>Full Interior Area Scan Active`);
  }

  // Redraw Lawnmower Scan Trajectory Polyline
  if (kmlWaypoints.length > 0) {
    kmlRouteLayer = L.polyline(kmlWaypoints, {
      color: '#38bdf8', // Cyan scan lines
      weight: 3,
      opacity: 0.9,
      dashArray: '8, 4'
    }).addTo(mapInstance);

    const alt = parseFloat(document.getElementById('mission-alt')?.value) || 2.5;
    kmlWaypoints.forEach((pt, idx) => {
      const marker = L.circleMarker(pt, {
        radius: 6,
        fillColor: '#38bdf8',
        color: '#ffffff',
        weight: 2,
        fillOpacity: 1.0
      }).addTo(mapInstance);
      marker.bindPopup(`<b>Scan Waypoint ${idx + 1}</b><br>Lat: ${pt[0].toFixed(6)}<br>Lon: ${pt[1].toFixed(6)}<br>Alt: ${alt}m AGL`);
      kmlWaypointMarkers.push(marker);
    });
  }
}

function triggerKmlFileInput() {
  const input = document.getElementById('kml-file-input');
  if (input) input.click();
}

async function handleKmlFileUpload(event) {
  const file = event.target.files?.[0];
  if (!file) return;

  try {
    const text = await file.text();
    const xml = new DOMParser().parseFromString(text, 'application/xml');

    const parseCoords = str => str.trim().split(/\s+/).map(v => {
      const parts = v.split(',');
      const lon = parseFloat(parts[0]);
      const lat = parseFloat(parts[1]);
      return (!isNaN(lat) && !isNaN(lon)) ? [lat, lon] : null;
    }).filter(Boolean);

    // 1. Check Polygon
    const linearRings = xml.querySelectorAll('LinearRing coordinates');
    if (linearRings.length > 0) {
      const polyPts = parseCoords(linearRings[0].textContent);
      if (polyPts.length >= 3) {
        kmlPolygonPoints = polyPts;
      }
    }

    // 2. Check LineString or Placemark Points
    const wps = [];
    const lineStrings = xml.querySelectorAll('LineString coordinates');
    lineStrings.forEach(ls => {
      const pts = parseCoords(ls.textContent);
      pts.forEach(p => wps.push(p));
    });

    if (wps.length === 0) {
      const marks = xml.querySelectorAll('Placemark Point coordinates');
      marks.forEach(m => {
        const pts = parseCoords(m.textContent);
        if (pts.length) wps.push(pts[0]);
      });
    }

    if (wps.length > 0) {
      kmlWaypoints = wps;
    } else if (kmlPolygonPoints.length >= 3) {
      const spacing = parseFloat(document.getElementById('survey-spacing')?.value) || 6.0;
      const alt = parseFloat(document.getElementById('mission-alt')?.value) || 2.5;
      const scanRes = generateAreaCoverageGrid(kmlPolygonPoints, spacing, alt);
      if (scanRes.gps && scanRes.gps.length > 0) {
        kmlWaypoints = scanRes.gps;
      } else {
        kmlWaypoints = [...kmlPolygonPoints];
      }
    }

    redrawKmlLayers();
    updateKmlBadge();

    const allPts = [...kmlPolygonPoints, ...kmlWaypoints];
    if (allPts.length > 0 && mapInstance) {
      mapInstance.fitBounds(L.latLngBounds(allPts), { padding: [50, 50] });
    }

    syncKmlToBehaviorTree();
    alert(`Loaded KML Survey Area successfully!\nFence Boundary Vertices: ${kmlPolygonPoints.length}\nAuto-Generated Interior Scan Waypoints: ${kmlWaypoints.length}\n3D Behavior Tree Area Coverage Trajectory updated.`);
  } catch (err) {
    alert('Error reading KML file: ' + err.message);
  } finally {
    event.target.value = '';
  }
}

function exportKmlFile() {
  if (kmlPolygonPoints.length < 3 && kmlWaypoints.length === 0) {
    alert('Please load a KML file or draw points on the map first.');
    return;
  }
  const alt = parseFloat(document.getElementById('mission-alt').value) || 2.5;
  const polyStr = kmlPolygonPoints.map(p => `${p[1]},${p[0]},${alt}`).concat(kmlPolygonPoints.length ? [`${kmlPolygonPoints[0][1]},${kmlPolygonPoints[0][0]},${alt}`] : []).join(' ');
  const marks = kmlWaypoints.map((p, i) =>
    `    <Placemark><name>WP ${i+1}</name><Point><coordinates>${p[1]},${p[0]},${alt}</coordinates></Point></Placemark>`
  ).join('\n');

  const kml = `<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>AERIXIS Mission Plan</name>
    ${kmlPolygonPoints.length >= 3 ? `
    <Placemark>
      <name>Boundary Area</name>
      <Polygon>
        <outerBoundaryIs>
          <LinearRing>
            <coordinates>${polyStr}</coordinates>
          </LinearRing>
        </outerBoundaryIs>
      </Polygon>
    </Placemark>` : ''}
${marks}
  </Document>
</kml>`;

  const blob = new Blob([kml], { type: 'application/vnd.google-earth.kml+xml' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'as2_kml_mission.kml';
  a.click();
}

function syncKmlToBehaviorTree() {
  if (kmlWaypoints.length === 0) return;
  const alt = parseFloat(document.getElementById('mission-alt').value) || 2.5;

  const homeLat = 47.397742;
  const homeLon = 8.545594;
  const R = 6378137.0;

  const enuWaypoints = kmlWaypoints.map(p => {
    const dLat = (p[0] - homeLat) * (Math.PI / 180.0);
    const dLon = (p[1] - homeLon) * (Math.PI / 180.0);
    const north = dLat * R;
    const east = dLon * R * Math.cos(homeLat * (Math.PI / 180.0));
    return { x: Math.round(east * 10) / 10, y: Math.round(north * 10) / 10 };
  });

  if (typeof loadKmlIntoBtStudio === 'function') {
    loadKmlIntoBtStudio(enuWaypoints, alt);
  }
}

async function convertKmlToBehaviorTreeAndExecute() {
  if (kmlWaypoints.length === 0) {
    alert('Please load a KML file or add waypoints to the map first.');
    return;
  }
  syncKmlToBehaviorTree();
  const confirmed = confirm(`Execute KML Mission with ${kmlWaypoints.length} waypoints on AERIXIS in OFFBOARD mode?`);
  if (!confirmed) return;

  await sendMode('offboard');
  await sendCmd('arm');
  await executeCurrentMission();
}

// Update Decoupled 3-Tier Architecture Pipeline Card (GCS -> AERIXIS -> Micro-XRCE-DDS -> PX4)
function updatePipelineCard(s) {
  if (!s) return;
  const isMavOk = !!s.connected;
  const as2Ok = !!(s.as2_link && s.as2_link.connected);
  const px4Ok = !!(s.px4_link && s.px4_link.connected);
  const hbHz = (s.heartbeat_hz !== undefined && s.heartbeat_hz !== null) ? Number(s.heartbeat_hz).toFixed(1) : (isMavOk ? '1.0' : '0.0');

  // Overall Pipeline Banner
  const banner = document.getElementById('pipeline-banner');
  const bannerTitle = document.getElementById('pipeline-banner-title');
  const bannerDetail = document.getElementById('pipeline-banner-detail');
  const bannerBadge = document.getElementById('pipeline-banner-badge');
  const bannerIcon = document.getElementById('pipeline-banner-icon');

  if (banner && bannerTitle) {
    if (isMavOk && as2Ok && px4Ok) {
      banner.className = 'pipeline-overall-banner status-full-sync';
      if (bannerIcon) bannerIcon.textContent = '⚡';
      bannerTitle.textContent = 'FULL PIPELINE SYNCHRONIZED';
      if (bannerDetail) bannerDetail.textContent = 'GCS ➔ AERIXIS Core ➔ Micro-XRCE-DDS ➔ PX4 Autopilot Live';
      if (bannerBadge) bannerBadge.textContent = 'ALL SYSTEMS GO';
    } else if (isMavOk && as2Ok && !px4Ok) {
      banner.className = 'pipeline-overall-banner status-as2-only';
      if (bannerIcon) bannerIcon.textContent = '🚁';
      bannerTitle.textContent = 'AERIXIS READY · AWAITING AUTOPILOT';
      if (bannerDetail) bannerDetail.textContent = 'AERIXIS Core ready. Launch Gazebo simulation (./3_launch_simulation.sh)';
      if (bannerBadge) bannerBadge.textContent = 'DDS PENDING';
    } else if (isMavOk && !as2Ok) {
      banner.className = 'pipeline-overall-banner status-gateway-only';
      if (bannerIcon) bannerIcon.textContent = '📡';
      bannerTitle.textContent = 'AERIXIS GATEWAY CONNECTED';
      if (bannerDetail) bannerDetail.textContent = 'Gateway live; waiting for AERIXIS platform & estimator nodes';
      if (bannerBadge) bannerBadge.textContent = 'INIT NODES';
    } else {
      banner.className = 'pipeline-overall-banner status-offline';
      if (bannerIcon) bannerIcon.textContent = '🔍';
      bannerTitle.textContent = 'SEARCHING FOR AERIXIS CORE';
      if (bannerDetail) bannerDetail.textContent = 'GCS listening on UDP 14551. Launch AERIXIS stack (./2_launch_as2.sh)';
      if (bannerBadge) bannerBadge.textContent = 'OFFLINE';
    }
  }

  // Tier 1: GCS Studio
  const badgeGcs = document.getElementById('tier-badge-gcs');
  if (badgeGcs) {
    badgeGcs.textContent = 'STANDALONE ACTIVE';
    badgeGcs.className = 'tier-badge badge-active';
  }

  // Link 1: MAVLink Telemetry
  const chan1 = document.getElementById('channel-link1');
  const rate1 = document.getElementById('link1-rate-badge');
  const det1 = document.getElementById('link1-detail');
  if (chan1) {
    if (isMavOk) {
      chan1.className = 'pipeline-link-channel link-active-heartbeat';
      if (rate1) {
        rate1.textContent = hbHz + ' Hz';
        rate1.className = 'channel-rate-badge badge-rate-green';
      }
      if (det1) det1.textContent = 'MAVLink Heartbeat Active · Latency < 1ms';
    } else {
      chan1.className = 'pipeline-link-channel link-searching';
      if (rate1) {
        rate1.textContent = '0.0 Hz';
        rate1.className = 'channel-rate-badge badge-rate-muted';
      }
      if (det1) det1.textContent = 'Scanning for AERIXIS on UDP 127.0.0.1:14550...';
    }
  }

  // Tier 2: AERIXIS Core
  const boxAs2 = document.getElementById('tier-box-as2');
  const badgeAs2 = document.getElementById('tier-badge-as2');
  const extraAs2 = document.getElementById('tier-extra-as2');
  if (boxAs2 && badgeAs2) {
    if (as2Ok) {
      boxAs2.className = 'pipeline-tier-box tier-as2 tier-active';
      badgeAs2.textContent = '● AERIXIS CORE ACTIVE';
      badgeAs2.className = 'tier-badge badge-active';
      if (extraAs2) extraAs2.textContent = s.as2_link?.detail || 'Platform, Estimator & Behaviors ready';
    } else if (isMavOk) {
      boxAs2.className = 'pipeline-tier-box tier-as2 tier-waiting';
      badgeAs2.textContent = '◌ INITIALIZING';
      badgeAs2.className = 'tier-badge badge-waiting';
      if (extraAs2) extraAs2.textContent = s.as2_link?.detail || 'Waiting for AERIXIS components';
    } else {
      boxAs2.className = 'pipeline-tier-box tier-as2 tier-offline';
      badgeAs2.textContent = '✕ OFFLINE';
      badgeAs2.className = 'tier-badge badge-offline';
      if (extraAs2) extraAs2.textContent = 'Run ./2_launch_as2.sh 1';
    }
  }

  // Link 2: Micro-XRCE-DDS Bridge
  const chan2 = document.getElementById('channel-link2');
  const det2 = document.getElementById('link2-detail');
  if (chan2) {
    if (px4Ok) {
      chan2.className = 'pipeline-link-channel link-active-dds';
      if (det2) det2.textContent = 'Synchronized (/fmu/out/vehicle_odometry @ 50 Hz)';
    } else if (as2Ok) {
      chan2.className = 'pipeline-link-channel link-waiting-dds';
      if (det2) det2.textContent = 'Agent active on UDP 8888; waiting for PX4 client...';
    } else {
      chan2.className = 'pipeline-link-channel link-offline';
      if (det2) det2.textContent = 'Micro-XRCE-DDS middleware bridge inactive';
    }
  }

  // Tier 3: PX4 Autopilot
  const boxPx4 = document.getElementById('tier-box-px4');
  const badgePx4 = document.getElementById('tier-badge-px4');
  const extraPx4 = document.getElementById('tier-extra-px4');
  if (boxPx4 && badgePx4) {
    if (px4Ok) {
      boxPx4.className = 'pipeline-tier-box tier-px4 tier-active';
      badgePx4.textContent = '● SYNCHRONIZED';
      badgePx4.className = 'tier-badge badge-active';
      if (extraPx4) extraPx4.textContent = 'Connected strictly via AERIXIS Micro-XRCE-DDS';
    } else if (as2Ok) {
      boxPx4.className = 'pipeline-tier-box tier-px4 tier-waiting';
      badgePx4.textContent = '◌ WAITING FOR CLIENT';
      badgePx4.className = 'tier-badge badge-waiting';
      if (extraPx4) extraPx4.textContent = 'Launch ./3_launch_simulation.sh';
    } else {
      boxPx4.className = 'pipeline-tier-box tier-px4 tier-offline';
      badgePx4.textContent = '✕ DISCONNECTED';
      badgePx4.className = 'tier-badge badge-offline';
      if (extraPx4) extraPx4.textContent = 'Autopilot offline (via AERIXIS Micro-XRCE-DDS)';
    }
  }
}

// Start Application
function startGcsApplication() {
  try { initLeafletMap(); } catch (e) { console.warn(e); }
  try { renderBtNodeList(); } catch (e) { console.warn(e); }
  try { updateBtXml(); } catch (e) { console.warn(e); }
  try { pollState(); setInterval(pollState, 500); } catch (e) { console.warn(e); }
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', startGcsApplication);
} else {
  startGcsApplication();
}
