/**
 * Enhanced 3D Behavior Tree Studio (Powered by Three.js)
 * Manages 3D tree visualization, locked mandatory prerequisites, and real-time XML generation.
 */

const MANDATORY_PREREQUISITES = [
  {
    id: 'node_offboard',
    type: 'Offboard',
    name: 'Set Offboard Mode',
    service_name: 'set_offboard_mode',
    locked: true,
    icon: '⚡',
    desc: 'Activates PX4 Offboard mode to transfer flight control authority to AERIXIS.'
  },
  {
    id: 'node_arm',
    type: 'Arm',
    name: 'Arm Drone',
    service_name: 'set_arming_state',
    locked: true,
    icon: '🛡️',
    desc: 'Arms motors after Offboard mode authority is verified.'
  },
  {
    id: 'node_takeoff',
    type: 'TakeOff',
    name: 'Autonomous Takeoff',
    locked: true,
    height: 2.0,
    speed: 0.5,
    icon: '🛫',
    desc: 'Ascends to safe initial operational altitude.'
  }
];

let missionTreeNodes = [
  ...JSON.parse(JSON.stringify(MANDATORY_PREREQUISITES)),
  { id: 'node_goto_1', type: 'GoTo', name: 'Waypoint Alpha', x: 5.0, y: 0.0, z: 2.0, speed: 1.5, yaw: 0.0, icon: '📍' },
  { id: 'node_goto_2', type: 'GoTo', name: 'Waypoint Bravo', x: 5.0, y: 5.0, z: 2.0, speed: 1.5, yaw: 0.0, icon: '📍' },
  { id: 'node_goto_3', type: 'GoTo', name: 'Waypoint Charlie', x: 0.0, y: 5.0, z: 2.0, speed: 1.5, yaw: 0.0, icon: '📍' },
  { id: 'node_goto_4', type: 'GoTo', name: 'Return Home Pose', x: 0.0, y: 0.0, z: 2.0, speed: 1.5, yaw: 0.0, icon: '📍' },
  { id: 'node_land', type: 'Land', name: 'Autonomous Land', speed: 0.5, icon: '🛬' },
  { id: 'node_disarm', type: 'Disarm', name: 'Disarm Motors', service_name: 'set_arming_state', icon: '🛑' }
];

let selectedBTNodeId = null;
let activeExecutingNodeIndex = -1;
let trajSubtreeExpanded = true;

// Three.js Scene Variables
let btScene, btCamera, btRenderer, btTreeGroup;
let btNodeMeshes = [];
let btIsDragging = false;
let btPrevMouse = { x: 0, y: 0 };
let btCameraAngle = { theta: 0.0, phi: 0.35, radius: 26.0 };
let btCameraTarget = { x: 0, y: -3.0, z: 0 };
let btThreeInitialized = false;

function initThreeBtStudio() {
  if (btThreeInitialized) return;
  try {
    const canvas = document.getElementById('threejs-canvas');
    if (!canvas) return;
    const container = canvas.parentElement;
    const w = container.clientWidth || 800;
    const h = container.clientHeight || 500;

    if (typeof THREE === 'undefined') {
      console.warn('Three.js not available yet');
      return;
    }

    btScene = new THREE.Scene();
    btScene.background = new THREE.Color(0x060911);

    btCamera = new THREE.PerspectiveCamera(45, w / h, 0.1, 1000);
    updateBtCameraPosition();

    btRenderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true, powerPreference: 'default' });
    btRenderer.setSize(w, h);
    btRenderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));

    // Ambient + Directional + Point Lights
    btScene.add(new THREE.AmbientLight(0xffffff, 0.7));
    const dir = new THREE.DirectionalLight(0x38bdf8, 0.85);
    dir.position.set(10, 20, 15);
    btScene.add(dir);

    const point = new THREE.PointLight(0x10b981, 1.2, 50);
    point.position.set(0, 5, 5);
    btScene.add(point);

    // Dark Tactical Floor Grid
    const grid = new THREE.GridHelper(50, 50, 0x1f344d, 0x0f1c2d);
    grid.position.y = -12;
    btScene.add(grid);

    btTreeGroup = new THREE.Group();
    btScene.add(btTreeGroup);

    // Mouse Navigation Listeners
    canvas.addEventListener('mousedown', onBtMouseDown);
    window.addEventListener('mousemove', onBtMouseMove);
    window.addEventListener('mouseup', onBtMouseUp);
    canvas.addEventListener('wheel', onBtWheel, { passive: false });
    window.addEventListener('resize', onBtResize);

    btThreeInitialized = true;
    rebuild3DBtTree();
    animateBtScene();
  } catch (err) {
    console.error('Three.js BT Studio init error:', err);
  }
}

function updateBtCameraPosition() {
  if (!btCamera) return;
  const x = btCameraTarget.x + btCameraAngle.radius * Math.sin(btCameraAngle.theta) * Math.cos(btCameraAngle.phi);
  const y = btCameraTarget.y + btCameraAngle.radius * Math.sin(btCameraAngle.phi);
  const z = btCameraTarget.z + btCameraAngle.radius * Math.cos(btCameraAngle.theta) * Math.cos(btCameraAngle.phi);
  btCamera.position.set(x, y, z);
  btCamera.lookAt(btCameraTarget.x, btCameraTarget.y, btCameraTarget.z);
}

function onBtMouseDown(e) {
  btIsDragging = true;
  btPrevMouse = { x: e.clientX, y: e.clientY };

  if (!btRenderer || !btCamera) return;
  const rect = btRenderer.domElement.getBoundingClientRect();
  const mouse = new THREE.Vector2(
    ((e.clientX - rect.left) / rect.width) * 2 - 1,
    -((e.clientY - rect.top) / rect.height) * 2 + 1
  );
  const raycaster = new THREE.Raycaster();
  raycaster.setFromCamera(mouse, btCamera);
  const intersects = raycaster.intersectObjects(btNodeMeshes, true);
  if (intersects.length > 0) {
    let obj = intersects[0].object;
    while (obj && !obj.userData.nodeId && obj.parent) obj = obj.parent;
    if (obj && obj.userData.nodeId) {
      selectBtNode(obj.userData.nodeId);
    }
  }
}

function onBtMouseMove(e) {
  if (!btIsDragging) return;
  const dx = e.clientX - btPrevMouse.x;
  const dy = e.clientY - btPrevMouse.y;
  btPrevMouse = { x: e.clientX, y: e.clientY };

  if (e.buttons === 1) {
    btCameraAngle.theta -= dx * 0.007;
    btCameraAngle.phi = Math.max(0.05, Math.min(Math.PI / 2 - 0.05, btCameraAngle.phi + dy * 0.007));
  } else if (e.buttons === 2 || e.shiftKey) {
    btCameraTarget.x -= dx * 0.03;
    btCameraTarget.y += dy * 0.03;
  }
  updateBtCameraPosition();
}

function onBtMouseUp() { btIsDragging = false; }

function onBtWheel(e) {
  e.preventDefault();
  btCameraAngle.radius = Math.max(8.0, Math.min(60.0, btCameraAngle.radius + e.deltaY * 0.025));
  updateBtCameraPosition();
}

function reset3DCamera() {
  btCameraAngle = { theta: 0.0, phi: 0.35, radius: 26.0 };
  btCameraTarget = { x: 0, y: -3.0, z: 0 };
  updateBtCameraPosition();
}

function setTopDownView() {
  btCameraAngle = { theta: 0.0, phi: 1.50, radius: 26.0 };
  updateBtCameraPosition();
}

function setIsoView() {
  btCameraAngle = { theta: 0.7, phi: 0.55, radius: 28.0 };
  updateBtCameraPosition();
}

function onBtResize() {
  const container = document.getElementById('threejs-canvas')?.parentElement;
  if (!container || !btRenderer || !btCamera) return;
  const w = container.clientWidth || 800;
  const h = container.clientHeight || 500;
  btCamera.aspect = w / h;
  btCamera.updateProjectionMatrix();
  btRenderer.setSize(w, h);
}

function createBtNodeTexture(title, subtitle, type, isMandatory, isSelected, isActive) {
  const c = document.createElement('canvas');
  c.width = 512; c.height = 256;
  const ctx = c.getContext('2d');

  ctx.fillStyle = isActive ? '#1e3a2f' : (isMandatory ? '#0b241b' : (isSelected ? '#0f2942' : '#0c1626'));
  ctx.fillRect(0, 0, 512, 256);

  ctx.lineWidth = isActive ? 14 : (isSelected ? 10 : 6);
  ctx.strokeStyle = isActive ? '#34d399' : (isMandatory ? '#10b981' : (isSelected ? '#38bdf8' : (type === 'Sequence' ? '#0ea5e9' : '#1e3852')));
  ctx.strokeRect(6, 6, 500, 244);

  // Status tag
  ctx.font = 'bold 22px Inter, sans-serif';
  if (isMandatory) {
    ctx.fillStyle = '#34d399';
    ctx.fillText('🔒 MANDATORY PREREQUISITE', 24, 46);
  } else {
    ctx.fillStyle = type === 'Sequence' ? '#38bdf8' : '#8b9eb5';
    ctx.fillText(type.toUpperCase(), 24, 46);
  }

  // Title
  ctx.font = 'bold 36px Inter, sans-serif';
  ctx.fillStyle = '#f0f6fc';
  ctx.fillText(title, 24, 115);

  // Subtitle
  ctx.font = '24px "JetBrains Mono", monospace';
  ctx.fillStyle = '#94a3b8';
  ctx.fillText(subtitle, 24, 180);

  // Status indicator ring
  ctx.fillStyle = isActive ? '#10b981' : (isMandatory ? '#10b981' : '#38bdf8');
  ctx.beginPath();
  ctx.arc(460, 42, 10, 0, Math.PI * 2);
  ctx.fill();

  const texture = new THREE.CanvasTexture(c);
  texture.needsUpdate = true;
  return texture;
}

function rebuild3DBtTree() {
  if (!btScene || !btTreeGroup) return;

  while (btTreeGroup.children.length > 0) {
    const obj = btTreeGroup.children[0];
    btTreeGroup.remove(obj);
  }
  btNodeMeshes = [];

  const rootPos = new THREE.Vector3(0, 7.2, 0);
  const seqPos = new THREE.Vector3(0, 3.8, 0);

  // 1. Root Node
  const rootMesh = create3DBtNodeMesh('MainTree', 'ID="MainTree"', 'BehaviorTree', false, false, false, 'root');
  rootMesh.position.copy(rootPos);
  btTreeGroup.add(rootMesh);

  // 2. Main Flight Sequence
  const isSeqActive = (activeExecutingNodeIndex >= 0);
  const seqMesh = create3DBtNodeMesh('FlightSequence', '<Sequence>', 'Sequence', false, false, isSeqActive, 'seq');
  seqMesh.position.copy(seqPos);
  btTreeGroup.add(seqMesh);

  // Link Root -> Sequence
  btTreeGroup.add(create3DLink(rootPos, seqPos, isSeqActive ? 0x34d399 : 0x0ea5e9));

  // Separate nodes into Pre-flight, Trajectory waypoints, and Post-flight
  const preNodes = [];
  const trajNodes = [];
  const postNodes = [];

  missionTreeNodes.forEach((node, idx) => {
    const item = { ...node, originalIndex: idx };
    if (node.type === 'Offboard' || node.type === 'Arm' || node.type === 'TakeOff') {
      preNodes.push(item);
    } else if (node.type === 'Land' || node.type === 'Disarm') {
      postNodes.push(item);
    } else {
      trajNodes.push(item);
    }
  });

  if (trajNodes.length > 0) {
    // Level 1 Children under FlightSequence: Pre-flight + TrajectorySequence + Post-flight
    const level1Items = [];
    preNodes.forEach(p => level1Items.push({ type: 'pre', data: p }));
    level1Items.push({ type: 'trajectory_branch', data: trajNodes });
    postNodes.forEach(p => level1Items.push({ type: 'post', data: p }));

    const l1Count = level1Items.length;
    const l1Spacing = 5.4;
    const l1StartX = -((l1Count - 1) * l1Spacing) / 2;

    let trajBranchPos = null;

    level1Items.forEach((item, l1Idx) => {
      const branchPos = new THREE.Vector3(l1StartX + l1Idx * l1Spacing, 0.2, 0);

      if (item.type === 'trajectory_branch') {
        trajBranchPos = branchPos;
        const isTrajActive = trajNodes.some(tn => tn.originalIndex === activeExecutingNodeIndex);
        const isTrajSelected = trajNodes.some(tn => tn.id === selectedBTNodeId);
        
        const trajMesh = create3DBtNodeMesh(
          'AreaCoverageTrajectory',
          '<Sequence> (' + trajNodes.length + ' WPs)',
          'Sequence',
          false,
          isTrajSelected,
          isTrajActive,
          'branch_trajectory'
        );
        trajMesh.position.copy(branchPos);
        btTreeGroup.add(trajMesh);
        btNodeMeshes.push(trajMesh);

        // Link FlightSequence -> AreaCoverageTrajectory
        const linkColor = isTrajActive ? 0x34d399 : 0x0ea5e9;
        btTreeGroup.add(create3DLink(seqPos, branchPos, linkColor));
      } else {
        const node = item.data;
        let sub = '';
        if (node.type === 'Offboard') sub = 'set_offboard_mode';
        else if (node.type === 'Arm') sub = 'set_arming_state';
        else if (node.type === 'TakeOff') sub = 'height: ' + (node.height || 2.0) + 'm';
        else if (node.type === 'Land') sub = 'speed: ' + (node.speed || 0.5) + 'm/s';
        else if (node.type === 'Disarm') sub = 'set_arming_state';

        const isActive = (node.originalIndex === activeExecutingNodeIndex);
        const mesh = create3DBtNodeMesh(node.name, sub, node.type, !!node.locked, node.id === selectedBTNodeId, isActive, node.id);
        mesh.position.copy(branchPos);
        btTreeGroup.add(mesh);
        btNodeMeshes.push(mesh);

        const linkColor = isActive ? 0x34d399 : (node.locked ? 0x10b981 : 0x38bdf8);
        btTreeGroup.add(create3DLink(seqPos, branchPos, linkColor));
      }
    });

    // Level 2: Scan Waypoint Leaf Nodes under AreaCoverageTrajectory!
    if (trajBranchPos) {
      const tCount = trajNodes.length;
      const tSpacing = 4.0;
      const tStartX = trajBranchPos.x - ((tCount - 1) * tSpacing) / 2;

      trajNodes.forEach((tn, tIdx) => {
        const childPos = new THREE.Vector3(
          tStartX + tIdx * tSpacing,
          -4.2,
          (tIdx % 2 === 0 ? 0 : 0.8)
        );

        let sub = '';
        if (tn.type === 'GoTo') sub = '[' + tn.x + ', ' + tn.y + ', ' + tn.z + ']';
        else if (tn.type === 'Wait') sub = (tn.duration || 5) + 's dwell';
        else if (tn.type === 'FollowPath') sub = 'follow_path';

        const isActive = (tn.originalIndex === activeExecutingNodeIndex);
        const mesh = create3DBtNodeMesh(tn.name, sub, tn.type, false, tn.id === selectedBTNodeId, isActive, tn.id);
        mesh.position.copy(childPos);
        btTreeGroup.add(mesh);
        btNodeMeshes.push(mesh);

        // LINK GOES FROM AreaCoverageTrajectory BRANCH DOWN TO WAYPOINT LEAF!
        const linkColor = isActive ? 0x34d399 : 0x38bdf8;
        btTreeGroup.add(create3DLink(trajBranchPos, childPos, linkColor));
      });
    }

  } else {
    // Fallback if no trajectory waypoints
    const total = missionTreeNodes.length;
    const spacing = 4.5;
    const startX = -((total - 1) * spacing) / 2;

    missionTreeNodes.forEach((node, idx) => {
      const childPos = new THREE.Vector3(startX + idx * spacing, -3.5, 0);
      let sub = '';
      if (node.type === 'Offboard') sub = 'set_offboard_mode';
      else if (node.type === 'Arm') sub = 'set_arming_state';
      else if (node.type === 'TakeOff') sub = 'height: ' + (node.height || 2.0) + 'm';
      else if (node.type === 'Land') sub = 'speed: ' + (node.speed || 0.5) + 'm/s';
      else if (node.type === 'Disarm') sub = 'set_arming_state';

      const isActive = (idx === activeExecutingNodeIndex);
      const mesh = create3DBtNodeMesh(node.name, sub, node.type, !!node.locked, node.id === selectedBTNodeId, isActive, node.id);
      mesh.position.copy(childPos);
      btTreeGroup.add(mesh);
      btNodeMeshes.push(mesh);

      const linkColor = isActive ? 0x34d399 : (node.locked ? 0x10b981 : 0x38bdf8);
      btTreeGroup.add(create3DLink(seqPos, childPos, linkColor));
    });
  }
}

function create3DBtNodeMesh(title, sub, type, isMandatory, isSelected, isActive, nodeId) {
  const group = new THREE.Group();
  group.userData = { nodeId };

  const geom = new THREE.BoxGeometry(3.6, 1.8, 0.35);
  const texture = createBtNodeTexture(title, sub, type, isMandatory, isSelected, isActive);
  const faceMat = new THREE.MeshStandardMaterial({ map: texture, roughness: 0.3, metalness: 0.1 });
  const sideMat = new THREE.MeshStandardMaterial({
    color: isActive ? 0x10b981 : (isMandatory ? 0x059669 : (isSelected ? 0x0284c7 : 0x162638)),
    roughness: 0.4
  });

  const materials = [sideMat, sideMat, sideMat, sideMat, faceMat, sideMat];
  const box = new THREE.Mesh(geom, materials);
  group.add(box);

  const wireGeom = new THREE.EdgesGeometry(geom);
  const wireMat = new THREE.LineBasicMaterial({
    color: isActive ? 0x34d399 : (isMandatory ? 0x10b981 : (isSelected ? 0x38bdf8 : 0x224163)),
    linewidth: isActive ? 3 : 2
  });
  group.add(new THREE.LineSegments(wireGeom, wireMat));
  return group;
}

function create3DLink(start, end, colorHex) {
  const mid = new THREE.Vector3((start.x + end.x) / 2, (start.y + end.y) / 2 - 0.5, (start.z + end.z) / 2);
  const curve = new THREE.QuadraticBezierCurve3(start, mid, end);
  const tubeGeom = new THREE.TubeGeometry(curve, 24, 0.05, 8, false);
  const mat = new THREE.MeshStandardMaterial({ color: colorHex, emissive: colorHex, emissiveIntensity: 0.6, roughness: 0.2 });
  return new THREE.Mesh(tubeGeom, mat);
}

function animateBtScene() {
  requestAnimationFrame(animateBtScene);
  if (btRenderer && btScene && btCamera) {
    btRenderer.render(btScene, btCamera);
  }
}

// Preset Loader
function loadBtPreset(presetKey) {
  const base = JSON.parse(JSON.stringify(MANDATORY_PREREQUISITES));
  if (presetKey === 'survey') {
    missionTreeNodes = [
      ...base,
      { id: 'node_goto_1', type: 'GoTo', name: 'Survey Point 1', x: 5.0, y: 0.0, z: 2.0, speed: 1.5, yaw: 0.0, icon: '📍' },
      { id: 'node_goto_2', type: 'GoTo', name: 'Survey Point 2', x: 5.0, y: 5.0, z: 2.0, speed: 1.5, yaw: 0.0, icon: '📍' },
      { id: 'node_goto_3', type: 'GoTo', name: 'Survey Point 3', x: 0.0, y: 5.0, z: 2.0, speed: 1.5, yaw: 0.0, icon: '📍' },
      { id: 'node_goto_4', type: 'GoTo', name: 'Return Home Pose', x: 0.0, y: 0.0, z: 2.0, speed: 1.5, yaw: 0.0, icon: '📍' },
      { id: 'node_land', type: 'Land', name: 'Autonomous Land', speed: 0.5, icon: '🛬' },
      { id: 'node_disarm', type: 'Disarm', name: 'Disarm Motors', service_name: 'set_arming_state', icon: '🛑' }
    ];
  } else if (presetKey === 'inspection') {
    missionTreeNodes = [
      ...base,
      { id: 'node_goto_insp', type: 'GoTo', name: 'Target Structure', x: 8.0, y: 4.0, z: 3.5, speed: 1.2, yaw: 1.57, icon: '📍' },
      { id: 'node_wait_insp', type: 'Wait', name: 'Dwell & Inspect', duration: 10, icon: '⏳' },
      { id: 'node_goto_ret', type: 'GoTo', name: 'Return Home', x: 0.0, y: 0.0, z: 2.0, speed: 1.5, yaw: 0.0, icon: '📍' },
      { id: 'node_land', type: 'Land', name: 'Autonomous Land', speed: 0.5, icon: '🛬' },
      { id: 'node_disarm', type: 'Disarm', name: 'Disarm Motors', service_name: 'set_arming_state', icon: '🛑' }
    ];
  } else if (presetKey === 'patrol') {
    missionTreeNodes = [
      ...base,
      { id: 'node_path_1', type: 'FollowPath', name: 'Perimeter Path', speed: 1.8, icon: '🛣️' },
      { id: 'node_land', type: 'Land', name: 'Autonomous Land', speed: 0.5, icon: '🛬' },
      { id: 'node_disarm', type: 'Disarm', name: 'Disarm Motors', service_name: 'set_arming_state', icon: '🛑' }
    ];
  } else if (presetKey === 'custom') {
    missionTreeNodes = [
      ...base,
      { id: 'node_land', type: 'Land', name: 'Autonomous Land', speed: 0.5, icon: '🛬' }
    ];
  }
  renderBtNodeList();
  if (btThreeInitialized) rebuild3DBtTree();
  updateBtXml();
}

function generateBtXml() {
  let preActionsXML = '';
  let trajectoryActionsXML = '';
  let postActionsXML = '';

  missionTreeNodes.forEach((node, idx) => {
    if (node.type === 'Offboard') {
      preActionsXML += `            <Action ID="Offboard" service_name="${node.service_name || 'set_offboard_mode'}"/>\n`;
    } else if (node.type === 'Arm') {
      preActionsXML += `            <Action ID="Arm" service_name="${node.service_name || 'set_arming_state'}"/>\n`;
    } else if (node.type === 'TakeOff') {
      preActionsXML += `            <Action ID="TakeOff" height="${node.height || 2.0}" speed="${node.speed || 0.5}"/>\n`;
    } else if (node.type === 'GoTo') {
      trajectoryActionsXML += `                <Action ID="GoTo" max_speed="${node.speed || 1.5}" pose="${node.x || 0};${node.y || 0};${node.z || 2}" yaw_angle="${node.yaw || 0.0}" yaw_mode="1" frame_id="earth"/>\n`;
    } else if (node.type === 'FollowPath') {
      trajectoryActionsXML += `                <Action ID="FollowPath"/>\n`;
    } else if (node.type === 'Wait') {
      trajectoryActionsXML += `                <Sleep msec="${Math.round(Number(node.duration ?? 5) * 1000)}"/>\n`;
    } else if (node.type === 'Land') {
      postActionsXML += `            <Action ID="Land" speed="${node.speed || 0.5}"/>\n`;
    } else if (node.type === 'Disarm') {
      postActionsXML += `            <Action ID="Disarm" service_name="${node.service_name || 'set_arming_state'}"/>\n`;
    }
  });

  let middleXML = '';
  if (trajectoryActionsXML) {
    middleXML = `            <!-- AREA COVERAGE SCAN TRAJECTORY CHILD OF FLIGHT SEQUENCE -->\n` +
                `            <Sequence name="AreaCoverageScanTrajectory">\n` +
                trajectoryActionsXML +
                `            </Sequence>\n`;
  }

  return `<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence name="FlightMissionSequence">
${(preActionsXML + middleXML + postActionsXML).trimEnd()}
        </Sequence>
    </BehaviorTree>
    <TreeNodesModel>
        <Action ID="Offboard">
            <input_port name="service_name" default="set_offboard_mode">Offboard mode service</input_port>
        </Action>
        <Action ID="Arm">
            <input_port name="service_name" default="set_arming_state">Arming service</input_port>
        </Action>
        <Action ID="TakeOff">
            <input_port name="height" default="2.0">Takeoff height (m)</input_port>
            <input_port name="speed" default="0.5">Takeoff speed (m/s)</input_port>
        </Action>
        <Action ID="GoTo">
            <input_port name="pose">Target position x;y;z</input_port>
            <input_port name="max_speed" default="1.5">Maximum speed (m/s)</input_port>
            <input_port name="yaw_angle" default="0.0">Yaw angle (rad)</input_port>
            <input_port name="yaw_mode" default="1">Yaw mode</input_port>
            <input_port name="frame_id" default="earth">Reference coordinate frame</input_port>
        </Action>
        <Action ID="FollowPath"/>
        <Action ID="Sleep">
            <input_port name="msec">Dwell time in milliseconds</input_port>
        </Action>
        <Action ID="Land">
            <input_port name="speed" default="0.5">Landing speed (m/s)</input_port>
        </Action>
        <Action ID="Disarm">
            <input_port name="service_name" default="set_arming_state">Disarming service</input_port>
        </Action>
    </TreeNodesModel>
</root>`;
}

function updateBtXml() {
  const xml = generateBtXml();
  const mini = document.getElementById('mini-xml-preview');
  const full = document.getElementById('full-xml-editor');
  if (mini) mini.value = xml;
  if (full) full.value = xml;
}

function renderBtNodeList() {
  const container = document.getElementById('node-sequence-list');
  if (!container) return;
  container.replaceChildren();

  // Separate nodes into pre-flight, trajectory waypoints, and post-flight
  const preNodes = [];
  const trajNodes = [];
  const postNodes = [];

  missionTreeNodes.forEach((node, index) => {
    const item = { ...node, originalIndex: index };
    if (node.type === 'Offboard' || node.type === 'Arm' || node.type === 'TakeOff') {
      preNodes.push(item);
    } else if (node.type === 'Land' || node.type === 'Disarm') {
      postNodes.push(item);
    } else {
      trajNodes.push(item);
    }
  });

  const renderSingleNode = (node, index, isIndented = false) => {
    const item = document.createElement('div');
    const isNodeActive = (index === activeExecutingNodeIndex);
    item.className = 'node-item' + (node.locked ? ' mandatory' : '') + (node.id === selectedBTNodeId ? ' selected' : '') + (isNodeActive ? ' node-active-exec' : '');
    if (isIndented) {
      item.style.marginLeft = '16px';
      item.style.borderLeft = isNodeActive ? '3px solid var(--green)' : '2px solid rgba(56, 189, 248, 0.4)';
    }
    item.onclick = () => selectBtNode(node.id);

    const head = document.createElement('div');
    head.className = 'node-head';

    const titleGroup = document.createElement('div');
    titleGroup.className = 'node-title-group';

    const icon = document.createElement('span');
    icon.className = 'node-icon';
    icon.textContent = node.icon || '📦';

    const nameGroup = document.createElement('div');
    const name = document.createElement('div');
    name.className = 'node-name';
    name.textContent = (isIndented ? '↳ ' : (index + 1) + '. ') + node.name;

    const typeLabel = document.createElement('div');
    typeLabel.className = 'node-type-label';
    typeLabel.textContent = isNodeActive ? '⚡ EXECUTING NOW' : ('Action: <' + node.type + '>');
    if (isNodeActive) typeLabel.style.color = 'var(--green)';

    nameGroup.append(name, typeLabel);
    titleGroup.append(icon, nameGroup);

    const actions = document.createElement('div');
    actions.className = 'node-actions';

    if (node.locked) {
      const lockedBadge = document.createElement('span');
      lockedBadge.className = 'mandatory-pill';
      lockedBadge.textContent = '🔒 MANDATORY';
      actions.append(lockedBadge);
    } else {
      if (index > 3) {
        const upBtn = document.createElement('button');
        upBtn.className = 'btn-icon';
        upBtn.textContent = '↑';
        upBtn.title = 'Move Up';
        upBtn.onclick = (e) => { e.stopPropagation(); moveBtNode(index, -1); };
        actions.append(upBtn);
      }
      if (index < missionTreeNodes.length - 1) {
        const dnBtn = document.createElement('button');
        dnBtn.className = 'btn-icon';
        dnBtn.textContent = '↓';
        dnBtn.title = 'Move Down';
        dnBtn.onclick = (e) => { e.stopPropagation(); moveBtNode(index, 1); };
        actions.append(dnBtn);
      }
      const delBtn = document.createElement('button');
      delBtn.className = 'btn-icon';
      delBtn.style.color = 'var(--red)';
      delBtn.textContent = '✕';
      delBtn.title = 'Delete Node';
      delBtn.onclick = (e) => { e.stopPropagation(); deleteBtNode(index); };
      actions.append(delBtn);
    }

    head.append(titleGroup, actions);
    item.append(head);

    if (node.type === 'GoTo') {
      const params = document.createElement('div');
      params.className = 'node-params';
      params.innerHTML = `
        <div class="param-field"><label>X (m)</label><input type="number" step="0.5" value="${node.x}" onchange="updateBtNodeParam('${node.id}', 'x', parseFloat(this.value))"></div>
        <div class="param-field"><label>Y (m)</label><input type="number" step="0.5" value="${node.y}" onchange="updateBtNodeParam('${node.id}', 'y', parseFloat(this.value))"></div>
        <div class="param-field"><label>Z (m)</label><input type="number" step="0.5" value="${node.z}" onchange="updateBtNodeParam('${node.id}', 'z', parseFloat(this.value))"></div>
        <div class="param-field"><label>Speed</label><input type="number" step="0.1" value="${node.speed}" onchange="updateBtNodeParam('${node.id}', 'speed', parseFloat(this.value))"></div>
      `;
      item.append(params);
    } else if (node.type === 'Wait') {
      const params = document.createElement('div');
      params.className = 'node-params';
      params.innerHTML = `
        <div class="param-field"><label>Duration (s)</label><input type="number" value="${node.duration}" onchange="updateBtNodeParam('${node.id}', 'duration', parseFloat(this.value))"></div>
      `;
      item.append(params);
    } else if (node.type === 'Land') {
      const params = document.createElement('div');
      params.className = 'node-params';
      params.innerHTML = `
        <div class="param-field"><label>Speed (m/s)</label><input type="number" step="0.1" value="${node.speed}" onchange="updateBtNodeParam('${node.id}', 'speed', parseFloat(this.value))"></div>
      `;
      item.append(params);
    }

    return item;
  };

  // 1. Render Pre-flight nodes (Offboard, Arm, Takeoff)
  preNodes.forEach(n => {
    container.append(renderSingleNode(n, n.originalIndex, false));
  });

  // 2. Render Trajectory Subtree Branch (AreaCoverageScanTrajectory)
  if (trajNodes.length > 0) {
    const isTrajActive = trajNodes.some(tn => tn.originalIndex === activeExecutingNodeIndex);
    const branchBox = document.createElement('div');
    branchBox.className = 'node-item trajectory-branch-card';
    branchBox.style.background = isTrajActive ? 'rgba(52, 211, 153, 0.12)' : 'rgba(14, 165, 233, 0.08)';
    branchBox.style.border = isTrajActive ? '1px solid var(--green)' : '1px solid rgba(56, 189, 248, 0.35)';
    branchBox.style.borderRadius = '6px';
    branchBox.style.padding = '8px 10px';
    branchBox.style.marginBottom = '6px';

    const bHead = document.createElement('div');
    bHead.style.display = 'flex';
    bHead.style.alignItems = 'center';
    bHead.style.justifyContent = 'space-between';
    bHead.style.cursor = 'pointer';
    bHead.onclick = () => {
      trajSubtreeExpanded = !trajSubtreeExpanded;
      renderBtNodeList();
    };

    const bLeft = document.createElement('div');
    bLeft.style.display = 'flex';
    bLeft.style.alignItems = 'center';
    bLeft.style.gap = '8px';
    bLeft.innerHTML = `
      <span style="font-size:16px;">🗺️</span>
      <div>
        <div style="font-weight:700;font-size:12.5px;color:#f0f6fc;">4. Area Coverage Trajectory</div>
        <div style="font-size:10px;color:var(--cyan);font-family:monospace;">&lt;Sequence name="AreaCoverageScanTrajectory"&gt;</div>
      </div>
    `;

    const bRight = document.createElement('div');
    bRight.style.display = 'flex';
    bRight.style.alignItems = 'center';
    bRight.style.gap = '6px';
    bRight.innerHTML = `
      <span style="font-size:10.5px;padding:2px 8px;border-radius:12px;background:rgba(56,189,248,0.2);color:var(--cyan);font-weight:600;">
        ${trajNodes.length} Waypoints
      </span>
      <span style="font-size:12px;color:#94a3b8;">${trajSubtreeExpanded ? '▼' : '▶'}</span>
    `;

    bHead.append(bLeft, bRight);
    branchBox.append(bHead);

    if (trajSubtreeExpanded) {
      const leavesCont = document.createElement('div');
      leavesCont.style.marginTop = '8px';
      leavesCont.style.paddingLeft = '8px';
      leavesCont.style.display = 'flex';
      leavesCont.style.flexDirection = 'column';
      leavesCont.style.gap = '6px';

      trajNodes.forEach(tn => {
        leavesCont.append(renderSingleNode(tn, tn.originalIndex, true));
      });
      branchBox.append(leavesCont);
    }

    container.append(branchBox);
  }

  // 3. Render Post-flight nodes (Land, Disarm)
  postNodes.forEach((n, idx) => {
    const displayIndex = (preNodes.length + (trajNodes.length > 0 ? 1 : 0) + idx);
    const rendered = renderSingleNode(n, n.originalIndex, false);
    const nameEl = rendered.querySelector('.node-name');
    if (nameEl) nameEl.textContent = `${displayIndex + 1}. ${n.name}`;
    container.append(rendered);
  });
}

function selectBtNode(id) {
  selectedBTNodeId = id;
  renderBtNodeList();
  if (btThreeInitialized) rebuild3DBtTree();
}

function updateBtNodeParam(id, key, val) {
  const node = missionTreeNodes.find(n => n.id === id);
  if (node) {
    node[key] = val;
    updateBtXml();
    if (btThreeInitialized) rebuild3DBtTree();
  }
}

function moveBtNode(index, delta) {
  const newIdx = index + delta;
  if (newIdx < 3 || newIdx >= missionTreeNodes.length) return;
  const item = missionTreeNodes.splice(index, 1)[0];
  missionTreeNodes.splice(newIdx, 0, item);
  renderBtNodeList();
  if (btThreeInitialized) rebuild3DBtTree();
  updateBtXml();
}

function deleteBtNode(index) {
  if (index < 3) return;
  missionTreeNodes.splice(index, 1);
  renderBtNodeList();
  if (btThreeInitialized) rebuild3DBtTree();
  updateBtXml();
}

function openAddBtNodeMenu() {
  const type = prompt('Select Node Type to Add:\\n1. GoTo (Waypoint)\\n2. Wait (Dwell)\\n3. Land\\n4. FollowPath\\n5. Disarm', '1');
  const count = missionTreeNodes.length + 1;
  if (type === '1' || (type && type.toLowerCase().includes('goto'))) {
    missionTreeNodes.push({ id: `node_goto_${Date.now()}`, type: 'GoTo', name: `Waypoint ${count}`, x: 5.0, y: 5.0, z: 2.0, speed: 1.5, yaw: 0.0, icon: '📍' });
  } else if (type === '2' || (type && type.toLowerCase().includes('wait'))) {
    missionTreeNodes.push({ id: `node_wait_${Date.now()}`, type: 'Wait', name: `Hover Dwell`, duration: 5, icon: '⏳' });
  } else if (type === '3' || (type && type.toLowerCase().includes('land'))) {
    missionTreeNodes.push({ id: `node_land_${Date.now()}`, type: 'Land', name: `Autonomous Land`, speed: 0.5, icon: '🛬' });
  } else if (type === '4' || (type && type.toLowerCase().includes('path'))) {
    missionTreeNodes.push({ id: `node_path_${Date.now()}`, type: 'FollowPath', name: `Path Trajectory`, speed: 1.5, icon: '🛣️' });
  } else if (type === '5' || (type && type.toLowerCase().includes('disarm'))) {
    missionTreeNodes.push({ id: `node_disarm_${Date.now()}`, type: 'Disarm', name: `Disarm Motors`, service_name: 'set_arming_state', icon: '🛑' });
  }
  renderBtNodeList();
  if (btThreeInitialized) rebuild3DBtTree();
  updateBtXml();
}

function copyBtXml() {
  navigator.clipboard.writeText(generateBtXml());
  alert('BehaviorTree.CPP v4 XML copied to clipboard!');
}

function downloadBtXml() {
  const blob = new Blob([generateBtXml()], { type: 'application/xml' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'as2_mission_tree.xml';
  a.click();
}

// KML Trajectory to Behavior Tree Converter
function loadKmlIntoBtStudio(waypoints, altitude = 2.5) {
  const base = JSON.parse(JSON.stringify(MANDATORY_PREREQUISITES));
  base[2].height = altitude;

  const kmlNodes = [...base];
  waypoints.forEach((wp, idx) => {
    kmlNodes.push({
      id: `node_scan_wp_${idx + 1}`,
      type: 'GoTo',
      name: `Scan WP ${idx + 1}`,
      x: wp.x,
      y: wp.y,
      z: altitude,
      speed: 1.5,
      yaw: 0.0,
      icon: '📍'
    });
  });

  kmlNodes.push({ id: 'node_land', type: 'Land', name: 'Autonomous Land', speed: 0.5, icon: '🛬' });
  kmlNodes.push({ id: 'node_disarm', type: 'Disarm', name: 'Disarm Motors', service_name: 'set_arming_state', icon: '🛑' });

  missionTreeNodes = kmlNodes;

  const presetSel = document.getElementById('preset-selector');
  if (presetSel) {
    let opt = document.getElementById('preset-kml-opt');
    if (!opt) {
      opt = document.createElement('option');
      opt.id = 'preset-kml-opt';
      opt.value = 'kml_imported';
      presetSel.appendChild(opt);
    }
    opt.textContent = `📐 Area Coverage (${waypoints.length} Scan WPs)`;
    presetSel.value = 'kml_imported';
  }

  renderBtNodeList();
  if (btThreeInitialized) rebuild3DBtTree();
  updateBtXml();
}
