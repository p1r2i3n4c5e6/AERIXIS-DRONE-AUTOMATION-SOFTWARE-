import hashlib
from http.server import ThreadingHTTPServer
from pathlib import Path
import tempfile
import threading
import time
import unittest
from as2_mission_manager.manager import MissionManager,load_registry
from as2_mission_manager.server import make_handler
from as2_rescue_runtime.radio_rpc import RadioRelay,RadioClient

class RadioTests(unittest.TestCase):
    def test_authenticated_radio_upload_start_and_status(self):
        from ament_index_python.packages import get_package_prefix
        runner=Path(get_package_prefix('as2_behavior_tree'))/'lib/as2_behavior_tree/as2_behavior_tree_node'
        with tempfile.TemporaryDirectory() as tmp:
            manager=MissionManager('radio_bench',tmp,runner,load_registry(runner))
            token='radio-bench-private-key-'*3
            server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(manager,token))
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            relay=None
            client=RadioClient(lambda system,frame:relay.receive(frame))
            relay=RadioRelay('radio_bench',token.encode(),lambda frame:client.receive(7,frame),f'http://127.0.0.1:{server.server_port}')
            profile=dict(drone_id='radio_bench',system_id=7,token=token)
            try:
                state=client.request(profile,'/api/status');self.assertFalse(state['active'])
                xml='<root BTCPP_format="4"><BehaviorTree ID="Radio"><AlwaysSuccess/></BehaviorTree></root>'
                mission=client.request(profile,'/api/mission/stage',dict(drone_id='radio_bench',xml=xml,sha256=hashlib.sha256(xml.encode()).hexdigest()))['result']
                self.assertFalse(manager.active())
                command=dict(mission_id=mission['id'],revision=mission['revision'],command_id='radio-bench-start')
                client.request(profile,'/api/mission/start',command)
                end=time.monotonic()+5
                while manager.active() and time.monotonic()<end:time.sleep(.02)
                self.assertEqual(client.request(profile,'/api/status')['execution']['state'],'SUCCESS')
                self.assertEqual(client.request(profile,'/api/mission/start',command)['result']['state'],'SUCCESS')
            finally:relay.close();server.shutdown();server.server_close();thread.join();manager.close()

if __name__=='__main__':unittest.main()
