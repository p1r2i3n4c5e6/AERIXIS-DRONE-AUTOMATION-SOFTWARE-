"""Harmless real-executor integration; source the AS2 environment first."""
import hashlib
from http.server import ThreadingHTTPServer
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
import urllib.request
import urllib.error

from as2_mission_manager.manager import MissionManager, load_registry
from as2_mission_manager.server import make_handler
from as2_mission_manager.validation import validate_xml

RUNNER = Path(os.environ.get('AS2_TEST_RUNNER', '/home/prince/aerostack2/install/as2_behavior_tree/lib/as2_behavior_tree/as2_behavior_tree_node'))

def tree(body):
    return '<root BTCPP_format="4"><BehaviorTree ID="Bench">'+body+'</BehaviorTree></root>'


class ManagerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.registry = load_registry(RUNNER)

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.manager = MissionManager('bench0',self.directory.name,RUNNER,self.registry)

    def tearDown(self):
        self.manager.close();self.directory.cleanup()

    def stage(self, body, manager=None):
        manager = manager or self.manager
        xml = tree(body)
        return manager.stage(dict(drone_id=manager.drone_id,xml=xml,sha256=hashlib.sha256(xml.encode()).hexdigest()))

    def start(self, mission, manager=None):
        return (manager or self.manager).start(dict(mission_id=mission['id'],revision=mission['revision'],command_id='test-command-1'))

    def finish(self, manager=None):
        manager = manager or self.manager
        end = time.monotonic()+12
        while manager.active() and time.monotonic()<end: time.sleep(.02)
        self.assertFalse(manager.active(), 'Executor did not finish')
        return manager.snapshot()

    def test_upload_does_not_start_and_checksum(self):
        self.stage('<AlwaysSuccess/>')
        self.assertIsNone(self.manager.snapshot()['execution'])
        with self.assertRaisesRegex(ValueError,'checksum'):
            self.manager.stage(dict(drone_id='bench0',xml=tree('<AlwaysSuccess/>'),sha256='bad'))
        with self.assertRaisesRegex(ValueError,'another drone'):
            self.manager.stage(dict(drone_id='bench1',xml=tree('<AlwaysSuccess/>')))

    def test_emergency_action_option_reaches_guardian(self):
        xml = tree('<AlwaysSuccess/>')
        base = dict(drone_id='bench0',xml=xml,sha256=hashlib.sha256(xml.encode()).hexdigest())
        with self.assertRaisesRegex(ValueError,'RTL or LAND'):
            self.manager.stage(dict(base,emergency_action='hover'))
        class Observer:
            actions = []
            def set_emergency_action(self, action): self.actions.append(action)
            def snapshot(self): return {}
        directory = tempfile.TemporaryDirectory()
        manager = MissionManager('bench0',directory.name,RUNNER,self.registry,observer=Observer())
        try:
            mission = manager.stage(dict(base,emergency_action='land'))
            self.assertEqual(mission['emergency_action'],'LAND')
            self.start(mission,manager)
            self.assertEqual(Observer.actions,['LAND'])
            self.finish(manager)
            plain = self.stage('<AlwaysSuccess/>')          # no option: the guardian keeps its parameter
            self.assertNotIn('emergency_action',plain)
        finally:
            manager.close();directory.cleanup()

    def test_success_tree_graph_and_idempotent_start(self):
        mission=self.stage('<Sequence><AlwaysSuccess/><AlwaysSuccess/></Sequence>')
        first=self.start(mission)
        self.assertEqual(self.start(mission)['pid'],first['pid'])
        result=self.finish()
        self.assertEqual(result['execution']['state'],'SUCCESS')
        self.assertEqual(len(result['bt_nodes']),3)
        self.assertEqual(sorted(len(n['children_uids']) for n in result['bt_nodes']),[0,0,2])
        self.assertEqual(result['bt_running_count'],0)
        with self.assertRaisesRegex(ValueError,'already been started'):
            self.manager.start(dict(mission_id=mission['id'],revision=mission['revision'],command_id='different-command'))

    def test_lockout_clears_itself_only_after_success_on_the_ground(self):
        class Observer:
            ground = False
            def safe_on_ground(self): return self.ground
            def reconcile_runtime(self): return 'runtime reconciled'
            def snapshot(self): return {}
        for body, outcome, released in [('<AlwaysSuccess/>', 'SUCCESS', True), ('<AlwaysFailure/>', 'FAILURE', False)]:
            directory = tempfile.TemporaryDirectory()
            observer = Observer()
            manager = MissionManager('bench0',directory.name,RUNNER,self.registry,observer=observer)
            try:
                self.start(self.stage(body,manager),manager)
                self.assertEqual(self.finish(manager)['execution']['state'],outcome)
                manager.record['hold_lockout'] = True        # as after a flight (requires_flight)
                self.assertTrue(manager.snapshot()['hold_lockout'], 'still airborne: lockout kept')
                observer.ground = True                       # FC: disarmed and landed
                self.assertEqual(not manager.snapshot()['hold_lockout'], released, outcome)
            finally:
                manager.close();directory.cleanup()

    def test_cancel_in_flight_asks_the_guardian_to_land_or_return(self):
        class Observer:
            ground = False
            def __init__(self): self.requests = []
            def safe_on_ground(self): return self.ground
            def request_emergency(self, action, source): self.requests.append((action, source))
            def set_emergency_action(self, action): pass
            def reconcile_runtime(self): return ''
            def snapshot(self): return {}
        xml = tree('<Sleep msec="30000"/>')
        for ground, expected in [(False, [('LAND', 'cancel')]), (True, [])]:
            directory = tempfile.TemporaryDirectory()
            observer = Observer(); observer.ground = ground
            manager = MissionManager('bench0',directory.name,RUNNER,self.registry,observer=observer)
            try:
                mission = manager.stage(dict(drone_id='bench0',xml=xml,sha256=hashlib.sha256(xml.encode()).hexdigest(),
                                             emergency_action='LAND'))
                self.start(mission,manager)
                end = time.monotonic()+5
                while not manager.snapshot()['bt_running_count'] and time.monotonic()<end: time.sleep(.02)
                manager.cancel(dict(mission_id=mission['id'],revision=mission['revision']))
                self.assertEqual(observer.requests, expected)
                self.finish(manager)
            finally:
                manager.close();directory.cleanup()

    def test_runtime_abort_blocks_start(self):
        # SITL 2026-10-23: a runtime abort survived a restart; the manager still said can_start and
        # the drone armed before the tree failed on the ground.
        class Observer:
            abort = 'RESTART_RECONCILIATION_REQUIRED'
            def safe_on_ground(self): return True
            def runtime_abort(self): return self.abort
            def snapshot(self): return {}
        directory = tempfile.TemporaryDirectory()
        observer = Observer()
        manager = MissionManager('bench0',directory.name,RUNNER,self.registry,observer=observer,allow_flight=True)
        try:
            mission = self.stage('<AlwaysSuccess/>',manager)
            manager.record['mission']['requires_flight'] = True   # as for a real flight tree
            self.assertFalse(manager.snapshot()['can_start'])
            with self.assertRaisesRegex(ValueError,'holds an abort'):
                self.start(mission,manager)
            observer.abort = ''
            self.assertTrue(manager.snapshot()['can_start'])
        finally:
            manager.close();directory.cleanup()

    def test_avoidance_switch_starts_off_and_reaches_the_runtime(self):
        class Observer:
            def __init__(self): self.sent = []
            def set_avoidance(self, enabled): self.sent.append(enabled)
            def snapshot(self): return {}
        directory = tempfile.TemporaryDirectory()
        observer = Observer()
        manager = MissionManager('bench0',directory.name,RUNNER,self.registry,observer=observer)
        try:
            self.assertEqual(observer.sent, [False])                 # every start: OFF
            self.assertEqual(manager.set_avoidance(dict(enabled=True)), dict(enabled=True))
            self.assertTrue(manager.snapshot()['avoidance'])
            with self.assertRaisesRegex(ValueError, 'true or false'):
                manager.set_avoidance(dict(enabled='yes'))
            manager.close()
            manager = MissionManager('bench0',directory.name,RUNNER,self.registry,observer=observer)
            self.assertEqual(observer.sent[-1], False)               # a restart switches it off again
            self.assertFalse(manager.snapshot()['avoidance'])
        finally:
            manager.close();directory.cleanup()

    def test_failure_is_not_success(self):
        self.start(self.stage('<AlwaysFailure/>'))
        self.assertEqual(self.finish()['execution']['state'],'FAILURE')

    def test_cancel_and_exclusivity(self):
        mission=self.stage('<Sleep msec="30000"/>');self.start(mission)
        end=time.monotonic()+5
        while not self.manager.snapshot()['bt_running_count'] and time.monotonic()<end:time.sleep(.02)
        self.assertGreater(self.manager.snapshot()['bt_running_count'],0)
        with self.assertRaisesRegex(ValueError,'running'):self.stage('<AlwaysSuccess/>')
        with self.assertRaisesRegex(ValueError,'revision'):
            self.manager.cancel(dict(mission_id=mission['id'],revision='wrong'))
        self.manager.cancel(dict(mission_id=mission['id'],revision=mission['revision']))
        self.assertEqual(self.finish()['execution']['state'],'CANCELLED')

    def test_independent_drones(self):
        with tempfile.TemporaryDirectory() as second:
            manager=MissionManager('bench1',second,RUNNER,self.registry)
            try:
                one=self.stage('<Sleep msec="500"/>');two=self.stage('<AlwaysFailure/>',manager)
                self.start(one);self.start(two,manager)
                self.assertEqual(self.finish(manager)['execution']['state'],'FAILURE')
                self.assertEqual(self.finish()['execution']['state'],'SUCCESS')
                self.assertNotEqual(one['id'],two['id'])
                self.assertTrue(all(n['node_uid'] is not None for n in manager.snapshot()['bt_nodes']))
            finally:manager.close()

    def test_stale_revision_tamper_and_flight_disabled(self):
        old=self.stage('<AlwaysSuccess/>');new=self.stage('<AlwaysFailure/>')
        with self.assertRaisesRegex(ValueError,'revision'):self.start(old)
        (Path(self.directory.name)/(new['id']+'.xml')).write_text(tree('<AlwaysSuccess/>'))
        with self.assertRaisesRegex(ValueError,'checksum'):self.start(new)
        flight=self.stage('<TakeOff height="2" speed="0.5"/>')
        with self.assertRaisesRegex(ValueError,'flight is disabled'):self.start(flight)

    def test_restart_lockout_and_directory_owner(self):
        with self.assertRaisesRegex(RuntimeError,'Another'):MissionManager('bench0',self.directory.name,RUNNER,self.registry)
        self.manager.close()
        path=Path(self.directory.name)/'state.json'
        path.write_text(json.dumps(dict(drone_id='bench0',mission=None,execution={'state':'RUNNING'},hold_lockout=False)))
        self.manager=MissionManager('bench0',self.directory.name,RUNNER,self.registry)
        self.assertEqual(self.manager.snapshot()['execution']['state'],'INTERRUPTED')
        self.assertTrue(self.manager.snapshot()['hold_lockout'])
        with self.assertRaisesRegex(ValueError,'ground'):self.manager.reconcile()

    def test_schema_rejections(self):
        for body in ['<InventedNode/>','<Script code="x:=1"/>','<Sleep/>','<Sleep msec="-1"/>',
                     '<TakeOff height="nan" speed="1"/>','<Sequence/>','<AlwaysSuccess><AlwaysFailure/></AlwaysSuccess>',
                     '<IfThenElse><AlwaysSuccess/></IfThenElse>']:
            with self.subTest(body=body),self.assertRaises(ValueError):validate_xml(tree(body),self.registry)
        with self.assertRaises(ValueError):validate_xml('<!DOCTYPE root>'+tree('<AlwaysSuccess/>'),self.registry)

    def test_http_authentication_and_api(self):
        token='test-only-token-'*3
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(self.manager,token))
        worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
        url=f'http://127.0.0.1:{server.server_port}'
        try:
            with self.assertRaises(urllib.error.HTTPError) as error:urllib.request.urlopen(url+'/api/status')
            self.assertEqual(error.exception.code,401)
            request=urllib.request.Request(url+'/api/status',headers={'Authorization':'Bearer '+token})
            with urllib.request.urlopen(request) as response:self.assertEqual(json.load(response)['drone_id'],'bench0')
        finally:server.shutdown();server.server_close();worker.join()


if __name__=='__main__':unittest.main()
