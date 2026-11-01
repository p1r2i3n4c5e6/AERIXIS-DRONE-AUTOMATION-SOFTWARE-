import json
from pathlib import Path
import tempfile
import unittest
from as2_rescue_runtime.safety import Evidence,Safety
from as2_rescue_runtime.siyi import encode,decode,attitude,rotation,CenteringController
from as2_rescue_runtime.durable import Journal
from as2_rescue_runtime.engine import Engine
from as2_rescue_runtime.swarm import AssignmentAuthority,separation_ok
from as2_rescue_runtime.radio import encode as radio_encode,Receiver
from as2_rescue_runtime.geolocation import locate

class Clock:
    def __init__(self):self.value=1000.
    def __call__(self):return self.value

class Driver:
    idle=True
    def __init__(self):self.calls=[];self.result=None
    def stop(self):self.calls.append('stop')
    def hold(self):return True
    def track(self,target):self.calls.append(('track',target))
    def move(self,pose,frame,speed,facing=None):self.calls.append(('move',pose));self.result='RUNNING'
    def vertical(self,*args):self.calls.append(('vertical',args));self.result='RUNNING'
    def motion_result(self):return self.result
    def release(self,*args):self.calls.append(('release',args))
    def report(self,*args):self.calls.append(('report',args))
    def request_route(self,command,pose,frame):
        self.evidence.put('route',dict(approved=True,path_clear=True,mission_id=command['mission_id'],
            command_id=command['command_id'],target_pose=pose,frame_id=frame))
    def assignment(self,*args):self.calls.append(('assignment',args))

class Tests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.clock=Clock();self.e=Evidence(self.clock)
        self.j=Journal(Path(self.tmp.name)/'journal.json');self.d=Driver();self.engine=Engine(self.e,self.j,self.d,self.clock)
        self.d.evidence=self.e
    def tearDown(self):self.tmp.cleanup()
    def healthy(self,gimbal_rate=0):
        self.e.put('vehicle',dict(position=[0,0,5],frame_id='earth',position_valid=True,connected=True,speed=0,tilt_deg=0,body_rate_dps=0,vibration_rms=.1,battery=.8,hold_ack=True,motion_idle=True,platform_state=3))
        self.e.put('gimbal',dict(verified=True,yaw=0,pitch=-90,yaw_rate=gimbal_rate,pitch_rate=0,oscillating=False,tracking=True,track_id=1,center_error=[0,0]))
        self.e.put('track',dict(track_id=1,confirmed=True,predicted=False,confidence=.9))
        self.e.put('fix',dict(valid=True,sigma_m=.2,track_id=1,frame_id='earth',position=[5,0,0],target_id='target'))
        self.e.put('zone',dict(clear=True,approved=True,person_clearance_m=5,frame_id='earth',target_id='target',approach_position=[0,0,5],path_clear=True))
        self.e.put('payload',dict(verified=True,present=True,latched=True,payload_id='kit-1'))
        self.e.put('swarm',dict(owned=True,mission_id='group',target_id='target',peers_fresh=True,separation_safe=True))
    def command(self,operation,**extra):
        value=dict(schema_version=1,stamp=self.clock(),mission_id='mission',run_id='run',command_id='cmd-'+str(len(self.engine.results))+operation,operation=operation,timeout_s=10,target_id='target',swarm_id='group',**extra)
        self.engine.submit(value);return value
    def test_unknown_and_stale_evidence_fail_closed(self):
        self.assertFalse(self.engine.flags()['VehicleHealthy']);self.healthy();self.assertTrue(self.engine.flags()['VehicleHealthy'])
        self.clock.value+=1.1;self.assertFalse(self.engine.flags()['VehicleHealthy']);self.assertFalse(self.engine.flags()['SafeToRelease'])
    def test_arm_pause_on_the_ground_is_tolerated_but_not_in_the_air(self):
        # AS2's platform pauses its topics while PX4 arms; on a loaded computer the gap passes 1 s
        # (SITL 2026-10-23: SafetyGate failed the flight 0.65 s into ManagedTakeoff).
        self.healthy();self.clock.value+=.01;self.e.put('vehicle',dict(self.e.get('vehicle'),platform_state=1))
        self.clock.value+=1.5;flags=self.engine.flags()
        self.assertTrue(flags['VehicleHealthy'],flags['health'])
        self.clock.value+=2.0;flags=self.engine.flags()
        self.assertFalse(flags['VehicleHealthy']);self.assertIn('no fresh flight-controller data',flags['health'])
        self.clock.value+=.01;self.healthy()                       # airborne (platform_state 3)
        self.clock.value+=1.5;flags=self.engine.flags()
        self.assertFalse(flags['VehicleHealthy']);self.assertIn('older than 1.0 s',flags['health'])
    def test_data_gap_while_take_off_spools_up_on_the_ground(self):
        # SITL 2026-11-01: a 1 s simulation stall cancelled a take-off still standing on the ground.
        self.healthy();base=dict(self.e.get('vehicle'),platform_state=2)
        self.clock.value+=.01;self.e.put('vehicle',dict(base,speed=0.05))
        self.clock.value+=1.5;flags=self.engine.flags()
        self.assertTrue(flags['VehicleHealthy'],flags['health'])
        self.clock.value+=.01;self.e.put('vehicle',dict(base,speed=0.5))   # climbing
        self.clock.value+=1.5;flags=self.engine.flags()
        self.assertFalse(flags['VehicleHealthy']);self.assertIn('older than 1.0 s',flags['health'])
    def test_health_reason_names_the_instability(self):
        self.healthy();self.clock.value+=.01;v=dict(self.e.get('vehicle'),tilt_deg=40);self.e.put('vehicle',v)
        self.engine.flags();self.clock.value+=.6;self.e.put('vehicle',dict(v));flags=self.engine.flags()
        self.assertFalse(flags['VehicleHealthy']);self.assertIn('AIRCRAFT_UNSTABLE',flags['health'])
    def test_replay_future_and_invalid_samples(self):
        self.assertTrue(self.e.put('x',{},1000));self.assertFalse(self.e.put('x',{},1000));self.assertFalse(self.e.put('x',{},1001))
    def test_gimbal_instability_stops_approach(self):
        self.healthy(gimbal_rate=20);self.command('ApproachDeliveryZone');result=self.engine.tick()
        self.assertEqual(result['status'],'FAILURE');self.assertIn('stop',self.d.calls);self.assertFalse(any(isinstance(x,tuple) and x[0]=='move' for x in self.d.calls))
    def test_aircraft_instability_latches_abort(self):
        self.healthy();self.clock.value+=.01;v=dict(self.e.get('vehicle'),vibration_rms=10);self.e.put('vehicle',v)
        flags=self.engine.flags();self.assertFalse(flags['AbortRequested'])  # not yet persistent
        self.clock.value+=.6;self.e.put('vehicle',dict(v));self.assertTrue(self.engine.flags()['AbortRequested'])
        self.clock.value+=.01;self.healthy();self.assertTrue(self.engine.flags()['AbortRequested'])
    def test_touchdown_vibration_does_not_abort_the_landing(self):
        # SITL 2026-10-25: the landing bump aborted ManagedLand as AIRCRAFT_UNSTABLE.
        self.healthy();self.clock.value+=.01;v=dict(self.e.get('vehicle'),vibration_rms=12.5,platform_state=4)
        self.e.put('vehicle',v);self.engine.flags();self.clock.value+=.6;self.e.put('vehicle',dict(v))
        flags=self.engine.flags();self.assertFalse(flags['AbortRequested']);self.assertTrue(flags['VehicleHealthy'])
        # Tilt still counts while landing.
        self.clock.value+=.01;t=dict(v,tilt_deg=80);self.e.put('vehicle',t);self.engine.flags()
        self.clock.value+=.6;self.e.put('vehicle',dict(t));self.assertTrue(self.engine.flags()['AbortRequested'])
    def test_brief_instability_spike_does_not_latch(self):
        self.healthy();self.clock.value+=.01;self.e.put('vehicle',dict(self.e.get('vehicle'),body_rate_dps=90,tilt_rate_dps=90))
        self.assertTrue(self.engine.flags()['VehicleHealthy'])  # transient tolerated
        self.clock.value+=.2;self.healthy();flags=self.engine.flags()
        self.assertTrue(flags['VehicleHealthy']);self.assertFalse(flags['AbortRequested'])
    def test_no_motion_before_cancel_termination(self):
        # The new leg waits (bounded by its timeout) and never commands motion meanwhile.
        self.healthy();self.d.idle=False;self.command('ApproachDeliveryZone');result=self.engine.tick()
        self.assertIsNone(result);self.assertIn('stop',self.d.calls)
        self.assertFalse(any(isinstance(x,tuple) and x[0]=='move' for x in self.d.calls))
    def test_release_requires_continuous_stability_and_sensors(self):
        self.healthy();self.command('ReleasePayload');result=self.engine.tick();self.assertEqual(result['status'],'FAILURE')
        self.assertFalse(any(isinstance(x,tuple) and x[0]=='release' for x in self.d.calls))
    def test_release_once_and_physical_confirmation(self):
        for _ in range(25):self.healthy();self.engine.flags();self.clock.value+=.1
        self.healthy();cmd=self.command('ReleasePayload');self.assertIsNone(self.engine.tick())
        self.assertEqual(len([x for x in self.d.calls if isinstance(x,tuple) and x[0]=='release']),1)
        self.engine.tick();self.assertEqual(len([x for x in self.d.calls if isinstance(x,tuple) and x[0]=='release']),1)
        self.clock.value+=.01;self.e.put('payload',dict(verified=True,present=False,released=True,latched=False,payload_id='kit-1',transaction_id=self.engine.last_release))
        result=self.engine.tick();self.assertEqual(result['status'],'SUCCESS');self.assertEqual(self.engine.submit(cmd),result)
    def test_release_restart_never_retries(self):
        fresh,_=self.j.reserve_release('m','t','p','tx');self.assertTrue(fresh)
        restarted=Journal(self.j.path);fresh,old=restarted.reserve_release('m','t','p','tx');self.assertFalse(fresh);self.assertEqual(old['state'],'UNCERTAIN')
        with self.assertRaises(ValueError):restarted.reserve_release('m','t','p','new')
    def test_checkpoint_persists(self):
        self.j.checkpoint('m','leg2',[1,2,3],True);self.assertIn('leg2',Journal(self.j.path).state['missions']['m']['completed'])
    def test_shared_assignment_excludes_second_drone(self):
        a=AssignmentAuthority(Path(self.tmp.name)/'assign.json',self.clock)
        self.assertTrue(a.claim('group','person','drone0'));self.assertFalse(a.claim('group','person','drone1'))
        self.clock.value+=6;self.assertFalse(a.claim('group','person','drone1'));self.assertFalse(a.claim('group','person','drone0'))
        self.assertEqual(next(iter(a.items.values()))['state'],'UNRESOLVED')
    def test_assignment_restart_is_uncertain(self):
        path=Path(self.tmp.name)/'assign.json';a=AssignmentAuthority(path,self.clock);a.claim('g','t','a')
        self.assertFalse(AssignmentAuthority(path,self.clock).claim('g','t','b'))
    def test_peer_frames_freshness_and_separation(self):
        peer=dict(position=[30,0,0],frame_id='earth',stamp=1000,sigma_m=1)
        self.assertTrue(separation_ok([0,0,0],[peer],1000,'earth'))
        self.assertFalse(separation_ok([0,0,0],[peer],1001,'earth'))
        self.assertFalse(separation_ok([0,0,0],[peer],1000,'other'))
        self.assertFalse(separation_ok([25,0,0],[peer],1000,'earth'))
    def test_radio_reorder_replay_corruption_and_missing_fragment(self):
        import random,string
        rng=random.Random(5);data=dict(xml=''.join(rng.choice(string.ascii_letters) for _ in range(8000)))
        key=b'k'*32;frames=radio_encode(data,key,1000);receiver=Receiver(key,self.clock)
        for frame in frames[:-1]:self.assertIsNone(receiver.accept(frame))
        bad=bytearray(frames[-1]);bad[-1]^=1;self.assertIsNone(receiver.accept(bytes(bad)))
        self.assertEqual(receiver.accept(frames[-1]),data);self.assertIsNone(receiver.accept(frames[-1]))
        receiver=Receiver(key,self.clock);result=None
        for frame in reversed(frames):result=receiver.accept(frame)
        self.assertEqual(result,data)
    def test_radio_wrong_key_and_expiry(self):
        frame=radio_encode({'a':1},b'k'*32,1000)[0]
        self.assertIsNone(Receiver(b'x'*32,self.clock).accept(frame));self.clock.value+=61
        self.assertIsNone(Receiver(b'k'*32,self.clock).accept(frame))
    def test_siyi_manufacturer_vectors_and_crc(self):
        self.assertEqual(encode(0x0d).hex(),'556601000000000de805')
        self.assertEqual(encode(7,rotation(100,100)).hex(),'556601020000000764643dcf')
        self.assertEqual(decode(encode(13))['command'],13)
        with self.assertRaises(ValueError):decode(encode(13)[:-1]+b'\x00')
    def test_gimbal_slew_deadband_and_stale_stop(self):
        c=CenteringController();self.assertEqual(c.update((1,1)),(3,3));self.assertEqual(c.update((1,1)),(6,6));self.assertEqual(c.update((1,1),False),(0,0));self.assertEqual(c.update((.01,.01)),(0,0))
    def test_runtime_restart_requires_ground_reconciliation(self):
        self.healthy();self.command('ApproachDeliveryZone');self.engine.tick()
        self.assertTrue(self.j.state['runtime']['needs_reconcile'])
        restarted=Engine(self.e,Journal(self.j.path),Driver(),self.clock)
        self.assertFalse(restarted.flags()['VehicleHealthy'])
        with self.assertRaises(ValueError):restarted.reconcile_ground()
        self.clock.value+=.01
        self.e.put('vehicle',dict(self.e.get('vehicle'),armed=False,platform_state=0))
        restarted.reconcile_ground()
        self.assertFalse(restarted.recovery_required)
        self.assertFalse(Journal(self.j.path).state['runtime']['needs_reconcile'])
    def test_ground_reconciliation_retains_uncertain_payload(self):
        self.j.reserve_release('m','t','p','tx');self.j.release_result('tx')
        self.healthy();self.clock.value+=.01
        self.e.put('vehicle',dict(self.e.get('vehicle'),armed=False,platform_state=0))
        self.engine.reconcile_ground()
        self.assertEqual(self.j.state['releases']['tx']['state'],'UNCERTAIN')
        with self.assertRaises(ValueError):self.j.reserve_release('m','t','p','another')
    def test_radio_agent_cannot_renew_after_supervisor_loss(self):
        from as2_rescue_runtime.swarm import supervisor_allows_renewal
        state=dict(stamp=self.clock(),mission_id='m',flags=dict(VehicleHealthy=True,AbortRequested=False))
        self.assertTrue(supervisor_allows_renewal(state,self.clock()))
        self.assertFalse(supervisor_allows_renewal(state,self.clock()+.6))
        state['flags']['AbortRequested']=True
        self.assertFalse(supervisor_allows_renewal(state,self.clock()))
        self.assertFalse(supervisor_allows_renewal(None,self.clock()))
    def test_release_timeout_latches_abort(self):
        for _ in range(25):self.healthy();self.engine.flags();self.clock.value+=.1
        self.healthy();self.command('ReleasePayload');self.engine.tick()
        self.clock.value+=11;result=self.engine.tick()
        self.assertEqual(result['status'],'FAILURE')
        self.assertTrue(self.engine.flags()['AbortRequested'])
        self.assertEqual(self.j.state['releases'][self.engine.last_release]['state'],'UNCERTAIN')
    def test_calibrated_geolocation_and_invalid_geometry(self):
        args=([320,240],[500,500,320,240],[[1,0,0],[0,-1,0],[0,0,-1]],[1,2,10],0,.1,.01,.1)
        result=locate(*args,surface_verified=True);self.assertEqual(result['position'],[1.,2.,0.]);self.assertGreater(result['sigma_m'],0)
        with self.assertRaises(ValueError):locate(*args)
        with self.assertRaises(ValueError):locate([320,240],[500,500,320,240],[[1,0,0],[0,1,0],[0,0,1]],[0,0,10],0,.1,.1,.1,True)

if __name__=='__main__':unittest.main()


class MeshWireTests(unittest.TestCase):
    def test_peer_frame_round_trip_and_fits_one_radio_frame(self):
        import time as _time
        from as2_rescue_runtime.radio import encode
        from as2_rescue_runtime.swarm import compact_peer, expand_peer
        frame = dict(owner='drone0', position=[-12.34, 45.67, 12.01], frame_id='earth', stamp=round(_time.time(), 3),
                     sigma_m=2.5, speed=1.53, claim=dict(label='L12', since=round(_time.time(), 2), xy=[-5.12, 17.93]),
                     tags=[[6.1, -28.2], [-4.9, 16.4]])
        self.assertEqual(expand_peer(compact_peer(frame)), frame)
        self.assertEqual(expand_peer(dict(frame)), frame)  # long form from an older drone still works
        parts = encode(dict(sender='drone0', kind='peer', payload=compact_peer(frame)), b'k' * 32, _time.time(), ttl=5)
        self.assertEqual(len(parts), 1)  # responding drones keep 2 updates/s at 2 frames/s
