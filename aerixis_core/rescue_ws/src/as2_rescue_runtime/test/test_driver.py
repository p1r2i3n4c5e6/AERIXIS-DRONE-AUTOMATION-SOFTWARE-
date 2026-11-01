"""Ownership tests for delayed AS2 goal acceptance and cancellation results."""
from types import SimpleNamespace
import unittest
from as2_rescue_runtime.runtime_node import RosDriver

class Future:
    def __init__(self,value=None):self.value=value;self.ready=False
    def done(self):return self.ready
    def result(self):return self.value

class DriverTests(unittest.TestCase):
    def test_late_goal_acceptance_is_cancelled_without_early_handover(self):
        driver=RosDriver.__new__(RosDriver)
        driver.goal=None;driver.result=None;driver.cancel_future=None
        driver.hover_future=None;driver.hover_ok=True;driver.outcome=None;driver.stopping=False
        driver.result_field='go_to_success'
        terminal=Future(SimpleNamespace(status=5,result=SimpleNamespace(go_to_success=False)))
        cancel=Future();calls=[]
        goal=SimpleNamespace(accepted=True,get_result_async=lambda:terminal,
            cancel_goal_async=lambda:(calls.append('cancel') or cancel))
        driver.sending=Future(goal)
        driver.stop();self.assertFalse(driver.idle)
        driver.sending.ready=True;driver.poll()
        self.assertEqual(calls,['cancel']);self.assertFalse(driver.idle)
        cancel.ready=True;driver.poll();self.assertFalse(driver.idle)
        terminal.ready=True;driver.poll();self.assertTrue(driver.idle)
        self.assertEqual(driver.motion_result(),'FAILURE')
    def test_velocity_waits_for_its_own_speed_ack_and_sends_the_real_position(self):
        # SITL 2026-10-23: right after a stop the cached controller mode still said SPEED, the yaw
        # pose went out with position (0, 0, 0) while the controller was back in POSITION mode, and
        # PX4 got a velocity setpoint towards the origin (41 m/s for one cycle).
        from as2_msgs.msg import ControlMode
        sent=[];requests=[]
        driver=RosDriver.__new__(RosDriver)
        driver.enabled=True;driver.goal=None;driver.sending=None;driver.hover_future=None;driver.stopping=False
        driver.result=None;driver.cancel_future=None
        driver.hover_ok=False;driver.velocity_error='';driver.speed_future=None;driver.velocity_active=True
        driver.speed_acked=True
        driver.controller_mode=SimpleNamespace(control_mode=ControlMode.SPEED,yaw_mode=ControlMode.YAW_ANGLE)
        driver.pose_pub=SimpleNamespace(publish=lambda m:sent.append(('pose',m)))
        driver.twist_pub=SimpleNamespace(publish=lambda m:sent.append(('twist',m)))
        ack=Future(SimpleNamespace(success=True))
        driver.hover=SimpleNamespace(service_is_ready=lambda:True,call_async=lambda r:(requests.append(r) or ack))
        here=SimpleNamespace(x=-38.7,y=0.02,z=15.0)
        driver.node=SimpleNamespace(raw={'pose':(SimpleNamespace(header=SimpleNamespace(frame_id='earth'),
                                                                 pose=SimpleNamespace(position=here)),0.0)},
                                    get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=0,to_msg=__import__('builtin_interfaces.msg',fromlist=['Time']).Time)))
        driver.stop()                                         # leg ends: HOVER, cached mode still SPEED
        sent.clear()
        self.assertFalse(driver.velocity([1.0,0.0,0.0],3.1,'earth'))
        self.assertEqual(sent,[]);self.assertEqual(len(requests),1)
        self.assertFalse(driver.velocity([1.0,0.0,0.0],3.1,'earth'));self.assertEqual(len(requests),1)
        ack.ready=True;driver.poll()
        self.assertTrue(driver.velocity([1.0,0.0,0.0],3.1,'earth'))
        pose=[m for k,m in sent if k=='pose'][0].pose.position
        self.assertEqual((pose.x,pose.y,pose.z),(-38.7,0.02,15.0))
        # No measured position in that frame: nothing is sent.
        sent.clear()
        self.assertFalse(driver.velocity([1.0,0.0,0.0],3.1,'drone1/odom'));self.assertEqual(sent,[])
    def test_velocity_reference_is_acceleration_limited(self):
        # SITL 2026-11-01: a step from hover to 2.8 m/s tilted the drone past 45 deg/s (abort).
        from as2_rescue_runtime.runtime_node import VELOCITY_ACCEL_MPS2
        clock=SimpleNamespace(t=100.0)
        driver=RosDriver.__new__(RosDriver)
        driver.node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(clock.t*1e9))),
                                    evidence=SimpleNamespace(get=lambda key,age=None:dict(velocity=[0.5,0.0,0.0])))
        driver.ramp=None
        self.assertEqual(driver.ramped([2.0,2.0,0.3]),[0.5,0.0,0.3])     # starts from the measured velocity
        speeds=[]
        for _ in range(60):
            clock.t+=0.05;speeds.append(driver.ramped([2.0,2.0,0.3]))
        steps=[((b[0]-a[0])**2+(b[1]-a[1])**2)**.5 for a,b in zip(speeds,speeds[1:])]
        self.assertLessEqual(max(steps),VELOCITY_ACCEL_MPS2*0.05+1e-9)
        self.assertEqual(speeds[-1],[2.0,2.0,0.3])                      # reaches the command
        clock.t+=5.0                                                     # a long pause: one bounded step
        before=list(speeds[-1]);after=driver.ramped([-2.0,-2.0,0.0])
        self.assertLessEqual(((after[0]-before[0])**2+(after[1]-before[1])**2)**.5,VELOCITY_ACCEL_MPS2*0.25+1e-9)
        driver.ramp=None;driver.node.evidence=None                       # stop() resets; unknown speed = 0
        self.assertEqual(driver.ramped([1.0,0.0,0.0]),[0.0,0.0,0.0])
    def test_repeated_stop_preserves_successful_hold_ack(self):
        driver=RosDriver.__new__(RosDriver);driver.stopping=False;driver.hover_ok=True
        driver.stop();self.assertFalse(driver.hover_ok)
        driver.hover_ok=True;driver.stop();self.assertTrue(driver.hover_ok)
