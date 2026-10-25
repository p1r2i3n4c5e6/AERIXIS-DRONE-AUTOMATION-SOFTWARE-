"""External mode change: PX4 leaves OFFBOARD in flight (RC pilot, FC guardian, GCS RTL/LAND,
PX4 failsafe). The running operation fails and motion stops; a later switch back to OFFBOARD
never resumes the old leg."""
import json
from pathlib import Path
import tempfile
import unittest
from as2_rescue_runtime.safety import Evidence
from as2_rescue_runtime.durable import Journal
from as2_rescue_runtime.engine import Engine,REANCHOR_S

class Clock:
    def __init__(self):self.value=1000.
    def __call__(self):return self.value

class Driver:
    idle=True;hover_future=None
    def __init__(self):self.calls=[];self.result=None
    def stop(self):self.calls.append('stop')
    def hold(self):return True
    def reanchor(self):self.calls.append('reanchor')
    def track(self,target):self.calls.append(('track',target))
    def move(self,pose,frame,speed,facing=None):self.calls.append(('move',pose));self.result='RUNNING'
    def vertical(self,*args):self.calls.append(('vertical',args));self.result='RUNNING'
    def velocity(self,*args):self.calls.append(('velocity',args));return True
    def motion_result(self):return self.result
    def report(self,*args):self.calls.append(('report',args))
    def request_route(self,command,pose,frame):
        self.evidence.put('route',dict(approved=True,path_clear=True,mission_id=command['mission_id'],
            command_id=command['command_id'],target_pose=pose,frame_id=frame))

FLYING,LANDED=3,1

class ModeChangeTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.clock=Clock();self.e=Evidence(self.clock)
        self.d=Driver();self.d.evidence=self.e
        self.engine=Engine(self.e,Journal(Path(self.tmp.name)/'journal.json'),self.d,self.clock)
        self.count=0
    def tearDown(self):self.tmp.cleanup()
    def step(self,dt=.05,offboard=True,armed=True,state=FLYING,mode=None,platform=True):
        """Advance the clock and feed fresh vehicle (and optionally platform/info) evidence."""
        self.clock.value+=dt
        self.e.put('vehicle',dict(position=[0,0,10],yaw=0.,frame_id='earth',position_valid=True,connected=True,speed=0,
            tilt_deg=0,body_rate_dps=0,vibration_rms=.1,battery=.8,hold_ack=True,motion_idle=True,platform_state=state,
            armed=armed))
        if platform:
            self.e.put('platform',dict(armed=armed,offboard=offboard,connected=True,state=state,px4_mode=mode))
    def command(self,operation,**extra):
        self.count+=1
        value=dict(schema_version=1,stamp=self.clock(),mission_id='mission',run_id='run',command_id='cmd-%d'%self.count,
            operation=operation,timeout_s=60,**extra)
        self.engine.submit(value);return value
    def moves(self):return [c for c in self.d.calls if isinstance(c,tuple) and c[0] in ('move','velocity','vertical')]
    def start_leg(self):
        self.step();self.command('SurveyWaypoint',segment_id='lane-1',pose='40;0;10',speed=1.0)
        self.assertIsNone(self.engine.tick());self.assertEqual(self.moves(),[('move',[40.,0.,10.])])
        self.d.result='RUNNING';self.d.idle=False

    def test_takeover_mid_leg_fails_the_leg_and_stops_motion(self):
        self.start_leg();self.d.calls.clear()
        self.step(offboard=False,mode='RTL');result=self.engine.tick()
        self.assertEqual(result['status'],'FAILURE');self.assertEqual(result['operation'],'SurveyWaypoint')
        self.assertIn('EXTERNAL_MODE_CHANGE',result['reason']);self.assertIn('now RTL',result['reason'])
        self.assertIn('stop',self.d.calls);self.assertIn(('track',None),self.d.calls)
        self.assertIsNone(self.engine.active);self.assertTrue(self.engine.external)
        self.assertEqual(self.moves(),[])
        flags=self.engine.flags()
        self.assertTrue(flags['AbortRequested']);self.assertFalse(flags['VehicleHealthy'])
        self.assertIn('EXTERNAL_MODE_CHANGE',flags['reason'])

    def test_back_to_offboard_never_resumes_the_old_leg(self):
        self.start_leg();self.step(offboard=False);self.engine.tick()
        self.d.idle=True;self.d.result=None;self.d.calls.clear()
        for _ in range(20):self.step(offboard=False);self.engine.tick()
        self.step(offboard=True);self.engine.tick()
        self.assertTrue(self.d.calls.count('reanchor')>=1)  # hover re-frozen where the aircraft is
        self.assertFalse(self.engine.external)
        for _ in range(20):self.step();self.engine.tick()
        self.assertEqual(self.moves(),[])
        # The BT (or a replayed command) asking for a new leg is refused: the mission stays aborted.
        self.command('SurveyWaypoint',segment_id='lane-2',pose='80;0;10',speed=1.0);result=self.engine.tick()
        self.assertEqual(result['status'],'FAILURE');self.assertIn('EXTERNAL_MODE_CHANGE',result['reason'])
        self.command('ResumeCoverage',speed=1.0);result=self.engine.tick()
        self.assertEqual(result['status'],'FAILURE');self.assertEqual(self.moves(),[])
        # Only return/land are accepted once AS2 flies again.
        self.command('ReturnToLaunch',speed=1.0);self.assertIsNone(self.engine.tick())
        self.assertEqual(self.moves(),[('move',[0.,0.,10.])])

    def test_no_motion_of_any_kind_while_another_mode_flies(self):
        self.start_leg();self.step(offboard=False);self.engine.tick();self.d.idle=True;self.d.result=None
        self.d.calls.clear()
        for operation,extra in (('ReturnToLaunch',{}),('ManagedLand',dict(speed=.5)),('RequestRecovery',{}),
                ('PauseCoverage',{}),('OrbitTarget',{}),('TrackTarget',{}),('SurveyWaypoint',dict(segment_id='x',pose='1;1;10'))):
            self.step(offboard=False);self.command(operation,**extra);result=self.engine.tick()
            self.assertEqual(result['status'],'FAILURE',operation);self.assertIn('EXTERNAL_MODE_CHANGE',result['reason'],operation)
        self.assertEqual(self.moves(),[])
        # A pure report still goes through.
        self.step(offboard=False);self.command('ReportObservation');self.assertEqual(self.engine.tick()['status'],'SUCCESS')

    def test_takeover_between_operations_blocks_the_next_one(self):
        self.step();self.engine.tick()  # flying in OFFBOARD, nothing active
        self.step(offboard=False,mode='POSCTL');self.engine.tick()
        self.assertTrue(self.engine.external);self.assertIn('now POSCTL',self.engine.mode_change)
        self.step(offboard=False);self.command('SurveyWaypoint',segment_id='lane-1',pose='40;0;10')
        self.assertEqual(self.engine.tick()['status'],'FAILURE');self.assertEqual(self.moves(),[])

    def test_hover_reference_follows_the_aircraft_while_external(self):
        self.start_leg();self.step(offboard=False);self.engine.tick();self.d.calls.clear()
        for _ in range(int(round(7.0/.05))):self.step(offboard=False);self.engine.tick()
        self.assertEqual(self.d.calls.count('reanchor'),int(7.0/REANCHOR_S))

    def test_stale_platform_keeps_the_decision(self):
        self.start_leg();self.step(offboard=False);self.engine.tick()
        self.clock.value+=5;self.step(platform=False)  # platform/info silent: still external
        self.assertTrue(self.engine.mode_guard())
        self.command('ReturnToLaunch');self.assertEqual(self.engine.tick()['status'],'FAILURE')

    def test_no_false_takeover_on_the_ground_or_at_disarm(self):
        # Offboard requested before arming, armed on the ground, PX4 switching modes while landed.
        self.step(offboard=False,armed=False,state=0);self.engine.tick()
        self.step(offboard=True,armed=False,state=0);self.engine.tick()
        self.step(offboard=True,armed=True,state=LANDED);self.engine.tick()
        self.step(offboard=False,armed=True,state=LANDED);self.engine.tick()
        self.assertFalse(self.engine.external);self.assertIsNone(self.engine.mode_change)
        # Normal flight, AS2 land, PX4 disarms (offboard flag drops in the same sample).
        self.step(offboard=True,armed=True,state=FLYING);self.engine.tick()
        self.step(offboard=True,armed=True,state=4);self.engine.tick()
        self.step(offboard=False,armed=False,state=LANDED);self.engine.tick()
        self.assertFalse(self.engine.external);self.assertIsNone(self.engine.mode_change);self.assertFalse(self.engine.aborted)
        # Armed again in a pilot mode that never was OFFBOARD this flight: not AS2's flight to lose.
        self.step(offboard=False,armed=True,state=FLYING);self.engine.tick()
        self.assertFalse(self.engine.external)

    def test_no_platform_evidence_changes_nothing(self):
        self.step(platform=False);self.command('SurveyWaypoint',segment_id='lane-1',pose='40;0;10')
        self.assertIsNone(self.engine.tick());self.assertEqual(self.moves(),[('move',[40.,0.,10.])])
        self.assertFalse(self.engine.external)

    def test_ground_reconcile_clears_the_latch(self):
        self.start_leg();self.step(offboard=False);self.engine.tick()
        self.d.idle=True;self.d.result=None
        self.step(offboard=False,armed=False,state=0);self.engine.tick()
        self.assertFalse(self.engine.external);self.assertTrue(self.engine.aborted)
        self.engine.reconcile_ground()
        self.assertIsNone(self.engine.mode_change);self.assertFalse(self.engine.aborted)


class RuntimeWiringTests(unittest.TestCase):
    """platform/info and fc_guardian/state reach the engine through the ROS node."""
    @classmethod
    def setUpClass(cls):
        import rclpy
        from as2_rescue_runtime.runtime_node import RescueRuntime
        cls.rclpy=rclpy;cls.tmp=tempfile.TemporaryDirectory()
        rclpy.init(args=['--ros-args','-r','__ns:=/mode_change_test','-p','state_dir:='+cls.tmp.name])
        cls.node=RescueRuntime()
    @classmethod
    def tearDownClass(cls):
        cls.node.owner_lock.close();cls.node.destroy_node();cls.rclpy.shutdown();cls.tmp.cleanup()
    def platform(self,offboard,armed=True,state=FLYING):
        from as2_msgs.msg import PlatformInfo
        msg=PlatformInfo();msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.connected=True;msg.armed=armed;msg.offboard=offboard;msg.status.state=state
        self.node.raw_sample('platform',msg)
    def test_platform_info_and_guardian_mode_feed_the_guard(self):
        from std_msgs.msg import String
        node=self.node
        self.platform(True);self.assertFalse(node.engine.mode_guard())
        node.guardian_state(String(data=json.dumps(dict(state='MONITORING',px4=dict(nav='LAND')))))
        self.platform(False)
        self.assertTrue(node.engine.mode_guard());self.assertIn('now LAND',node.engine.mode_change)
        self.assertTrue(node.driver.stopping)
    def test_reanchor_requests_a_new_hover(self):
        driver=self.node.driver;driver.hover_ok=True;driver.stopping=False
        driver.reanchor();self.assertFalse(driver.hover_ok);self.assertTrue(driver.stopping)

if __name__=='__main__':unittest.main()
