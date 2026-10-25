"""Run harmless synthetic BTs through the actual executable; no AS2 flight actions."""
import os
import subprocess
import threading
from bt_execution import LocalTreeRun

os.environ['ROS_DOMAIN_ID'] = '91'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/as2_bt_phase1_test_logs'

cases = [('success', '<AlwaysSuccess/>', 'SUCCESS', 0),
         ('failure', '<AlwaysFailure/>', 'FAILURE', 1),
         ('unknown_node', '<NoSuchNode/>', 'ERROR', 3),
         ('cancel', '<Sleep msec="10000"/>', 'CANCELLED', 2)]
for name, body, expected, code in cases:
    run = LocalTreeRun('phase1_test', f'<root BTCPP_format="4"><BehaviorTree ID="Test">{body}</BehaviorTree></root>')
    proc = run.start()
    # Bound only this synthetic test's child lifetime; no vehicle actions are in these trees.
    timeout = threading.Timer(15, proc.kill)
    timeout.start()
    output = []
    try:
        for line in proc.stdout:
            output.append(line)
            run.state.accept(line.strip())
            if name == 'cancel' and any(n['status'] == 'RUNNING' for n in run.state.nodes.values()):
                run.cancel()
        actual = proc.wait()
        run.state.finish_process(actual)
        assert (run.state.outcome, actual) == (expected, code), ''.join(output)
        print(f'{name}: {run.state.outcome}, exit={actual}')
    finally:
        timeout.cancel()
