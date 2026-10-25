"""Actual registered C++ nodes talking to the Python runtime; no hardware drivers enabled."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import unittest
import rclpy
from ament_index_python.packages import get_package_prefix
from as2_rescue_runtime.runtime_node import RescueRuntime

class BridgeTests(unittest.TestCase):
    def test_real_nodes_and_runtime(self):
        runner=Path(get_package_prefix('as2_behavior_tree'))/'lib/as2_behavior_tree/as2_behavior_tree_node'
        with tempfile.TemporaryDirectory() as tmp:
            rclpy.init(args=['--ros-args','-r','__ns:=/rescue_bench','-p','state_dir:='+tmp])
            node=RescueRuntime();stop=threading.Event()
            def spin():
                while not stop.is_set():rclpy.spin_once(node,timeout_sec=.02)
            worker=threading.Thread(target=spin,daemon=True);worker.start()
            def run(body):
                path=Path(tmp)/'tree.xml';path.write_text('<root BTCPP_format="4"><BehaviorTree ID="Test">'+body+'</BehaviorTree></root>')
                return subprocess.run([str(runner),'--ros-args','-r','__ns:=/rescue_bench','-p','mission_id:=bridge_test','-p','tree:='+str(path)],capture_output=True,text=True,timeout=8)
            try:
                result=run('<ReportObservation timeout_s="3"/>');self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                result=run('<SafetyGate flag="VehicleHealthy"><AlwaysSuccess/></SafetyGate>');self.assertEqual(result.returncode,1)
                result=run('<Timeout msec="350"><RetryUntilSuccessful num_attempts="-1"><Delay delay_msec="50"><StableFor seconds=".1"><AlwaysFailure/></StableFor></Delay></RetryUntilSuccessful></Timeout>');self.assertEqual(result.returncode,1,result.stdout)
                result=run('<StableFor seconds="0.1"><AlwaysSuccess/></StableFor>');self.assertEqual(result.returncode,0,result.stdout)
                result=run('<RecoverThenFail><AlwaysFailure/><Sleep msec="100"/></RecoverThenFail>');self.assertEqual(result.returncode,1,result.stdout)
                result=run('<ReleasePayload timeout_s="2" target_id="zone"/>');self.assertEqual(result.returncode,1,result.stdout)
                self.assertFalse(node.driver.enabled);self.assertEqual(node.engine.journal.state['releases'],{})
            finally:stop.set();worker.join();node.owner_lock.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':unittest.main()
