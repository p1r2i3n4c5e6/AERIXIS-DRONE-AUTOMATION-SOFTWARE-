"""Exercise the installed GCS BT path without opening its network transports."""
import importlib.util
import os
import sys
import threading
import time

os.environ.update(ROS_DOMAIN_ID='91', ROS_LOCALHOST_ONLY='1',
                  ROS_LOG_DIR='/tmp/as2_bt_phase1_test_logs')
sys.path.insert(0, '/home/prince/custom_gcs')
spec = importlib.util.spec_from_file_location('gcs_smoke_target', '/home/prince/custom_gcs/server.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
bridge = module.MavlinkBridge.__new__(module.MavlinkBridge)
bridge.home = (0.0, 0.0, 0.0)
bridge.state_lock = threading.RLock()
bridge.bt_run_lock = threading.Lock()
bridge.bt_active_run = None
bridge.bt_cancel_pending = False
bridge.state = {'mission_active': False, 'gateway_heartbeat': time.monotonic()}
bridge.bt_xml_upload_bytes = 0
bridge._append_log = lambda *args: None


def run(body):
    bridge.state['gateway_heartbeat'] = time.monotonic()
    bridge.run_behavior_tree({'drone_id': 'phase1_test', 'tree_xml':
                             f'<root BTCPP_format="4"><BehaviorTree ID="Test">{body}</BehaviorTree></root>'})


def wait():
    deadline = time.monotonic() + 10
    while bridge.bt_run_lock.locked() and time.monotonic() < deadline:
        time.sleep(0.02)
    assert not bridge.bt_run_lock.locked(), bridge.state


run('<Sequence><AlwaysFailure/><AlwaysSuccess/></Sequence>')
wait()
assert bridge.state['mission_result'] == 'FAILURE', bridge.state
assert [s['status'] for s in bridge.state['mission_steps']] == ['failed', 'pending'], bridge.state
print('Installed GCS: failure preserved; unticked action stays pending')
run('<Sleep msec="10000"/>')
try:
    run('<AlwaysSuccess/>')
    raise AssertionError('Overlapping mission accepted')
except ValueError:
    pass
deadline = time.monotonic() + 5
while not any(s['status'] == 'running' for s in bridge.state['mission_steps']):
    assert time.monotonic() < deadline
    time.sleep(0.02)
bridge.stop_mission()
wait()
assert bridge.state['mission_result'] == 'CANCELLED', bridge.state
print('Installed GCS: overlapping mission rejected; stop reaches actual BT process')
