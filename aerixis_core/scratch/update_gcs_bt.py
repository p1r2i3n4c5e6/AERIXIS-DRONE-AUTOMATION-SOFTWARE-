from pathlib import Path

index_path = Path('/home/prince/custom_gcs/web/index.html')
with open(index_path, 'r', encoding='utf-8') as f:
    content = f.read()

# Replace button area if run-bt-mission not present
if 'id="run-bt-mission"' not in content:
    target = '<button id="upload-mission" class="primary" style="width:100%;margin-top:9px" disabled>Upload + execute through AS2</button>'
    replacement = '''<button id="upload-mission" class="primary" style="width:100%;margin-top:9px" disabled>Upload + Execute Trajectory (AS2)</button>
        <button id="run-bt-mission" class="secondary" style="width:100%;margin-top:7px;background:linear-gradient(135deg,#312e81,#4338ca);border:1px solid #6366f1;color:#fff;font-weight:600;" disabled>Execute AS2 Behavior Tree</button>'''
    content = content.replace(target, replacement)

# Add event handler and disable logic
if "run-bt-mission').onclick" not in content:
    bt_script = '''
    if ($('run-bt-mission')) {
      $('run-bt-mission').onclick = async () => {
        const alt = Number($('mission-alt').value);
        if (waypoints.length === 0) {
          notice('Please load KML or add waypoints first.', true);
          return;
        }
        const droneId = $('swarm-select') ? $('swarm-select').value : 'drone0';
        notice('Compiling & launching AS2 Behavior Tree for ' + droneId + ' at ' + alt + 'm...');
        try {
          const res = await fetch('/api/tree/run', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({
              waypoints,
              altitude: alt,
              drone_id: droneId
            })
          });
          const d = await res.json();
          if (!res.ok) throw new Error(d.error || 'Failed to start AS2 Behavior Tree');
          notice('AS2 Behavior Tree running: ' + d.count + ' waypoints loaded into BehaviorTree.CPP sequence.');
        } catch (err) {
          notice(err.message, true);
        }
      };
    }
'''
    upload_click = "    $('upload-mission').onclick=async()=>{"
    content = content.replace(upload_click, bt_script + "\n" + upload_click)

# Ensure run-bt-mission disabled state syncs with upload-mission
if "if($('run-bt-mission')) $('run-bt-mission').disabled" not in content:
    content = content.replace(
        "$('upload-mission').disabled=!(waypoints.length>0&&stackReady);",
        "$('upload-mission').disabled=!(waypoints.length>0&&stackReady);\n      if($('run-bt-mission')) $('run-bt-mission').disabled=$('upload-mission').disabled;"
    )
    content = content.replace(
        "$('upload-mission').disabled=!(waypoints.length>0&&vehicleState.connected&&vehicleState.as2_link?.connected===true&&vehicleState.px4_link?.connected===true);",
        "$('upload-mission').disabled=!(waypoints.length>0&&vehicleState.connected&&vehicleState.as2_link?.connected===true&&vehicleState.px4_link?.connected===true);\n      if($('run-bt-mission')) $('run-bt-mission').disabled=$('upload-mission').disabled;"
    )

with open(index_path, 'w', encoding='utf-8') as f:
    f.write(content)

print('Updated index.html cleanly')
