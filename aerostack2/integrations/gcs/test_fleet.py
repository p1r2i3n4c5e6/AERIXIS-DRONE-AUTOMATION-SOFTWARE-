"""Exercise the laptop HTTP bridge against real, harmless onboard executors."""
import hashlib
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
import urllib.request
import urllib.error

sys.path.insert(0,'/home/prince/custom_gcs')
from fleet import Fleet
from server import make_handler as gcs_handler
from as2_mission_manager.manager import MissionManager, load_registry
from as2_mission_manager.server import make_handler

RUNNER=Path('/home/prince/aerostack2/install/as2_behavior_tree/lib/as2_behavior_tree/as2_behavior_tree_node')

class FleetTests(unittest.TestCase):
    def test_full_upload_start_status_cancel_flow_and_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            registry=load_registry(RUNNER)
            managers=[MissionManager(f'bench{i}',Path(tmp)/str(i),RUNNER,registry) for i in range(2)]
            servers=[];threads=[];token='local-test-token-'*3
            fleet=Fleet(Path(tmp)/'fleet.json')
            for manager in managers:
                server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(manager,token));servers.append(server)
                thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start();threads.append(thread)
            gcs=ThreadingHTTPServer(('127.0.0.1',0),gcs_handler(None,fleet));servers.append(gcs)
            thread=threading.Thread(target=gcs.serve_forever,daemon=True);thread.start();threads.append(thread)
            url=f'http://127.0.0.1:{gcs.server_port}'
            def request(path,data=None,header=True):
                headers={'Content-Type':'application/json'}
                if header:headers['X-AS2-GCS']='1'
                req=urllib.request.Request(url+path,data=json.dumps(data).encode() if data is not None else None,headers=headers)
                with urllib.request.urlopen(req,timeout=10) as response:return json.load(response)
            try:
                for i in range(2):request('/api/fleet/profile',dict(drone_id=f'bench{i}',endpoint=f'http://127.0.0.1:{servers[i].server_port}',token=token))
                self.assertEqual((Path(tmp)/'fleet.json').stat().st_mode & 0o777,0o600)
                with self.assertRaises(urllib.error.HTTPError):request('/api/fleet/profile',{},header=False)
                xml='<root BTCPP_format="4"><BehaviorTree ID="Bench"><Sleep msec="30000"/></BehaviorTree></root>'
                staged=[]
                for i in range(2):
                    response=request('/api/fleet/command',dict(drone_id=f'bench{i}',action='stage',xml=xml,sha256=hashlib.sha256(xml.encode()).hexdigest()))
                    staged.append(response['result']);self.assertFalse(managers[i].active())
                for i,mission in enumerate(staged):
                    request('/api/fleet/command',dict(drone_id=f'bench{i}',action='start',mission_id=mission['id'],revision=mission['revision'],command_id=f'bench-start-{i}'))
                deadline=time.monotonic()+8
                while time.monotonic()<deadline:
                    for profile in list(fleet.profiles.values()):fleet._one(profile)
                    data=request('/api/fleet')
                    if all(d['data']['bt_running_count'] for d in data['drones']):break
                    time.sleep(.05)
                self.assertTrue(all(d['fresh'] and d['data']['bt_running_count']==1 for d in data['drones']))
                self.assertNotIn(token,json.dumps(data))
                for i,mission in enumerate(staged):request('/api/fleet/command',dict(drone_id=f'bench{i}',action='cancel',mission_id=mission['id'],revision=mission['revision']))
                deadline=time.monotonic()+8
                while any(m.active() for m in managers) and time.monotonic()<deadline:time.sleep(.02)
                self.assertTrue(all(m.snapshot()['execution']['state']=='CANCELLED' for m in managers))
                # A wrongly paired endpoint must never receive a mutation.
                request('/api/fleet/profile',dict(drone_id='wrong_drone',endpoint=f'http://127.0.0.1:{servers[0].server_port}',token=token))
                with self.assertRaises(urllib.error.HTTPError) as error:request('/api/fleet/command',dict(drone_id='wrong_drone',action='reconcile'))
                self.assertIn('different drone ID',error.exception.read().decode())
                for path in ['/','/js/fleet.js','/css/fleet.css']:
                    with urllib.request.urlopen(url+path) as response:self.assertEqual(response.status,200)
            finally:
                fleet.close()
                for server in servers:server.shutdown();server.server_close()
                for thread in threads:thread.join()
                for manager in managers:manager.close()

if __name__=='__main__':unittest.main()
