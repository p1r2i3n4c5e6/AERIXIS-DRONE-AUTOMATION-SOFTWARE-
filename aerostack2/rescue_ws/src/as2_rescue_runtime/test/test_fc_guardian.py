"""FC guardian: decision logic (guardian.py) and the ROS node that commands PX4 (fc_guardian.py).

Software checks with synthetic PX4/AS2 messages; no aircraft, no PX4. The SITL check (kill AS2
mid-flight, PX4 must RTL/LAND) is described in docs/RASPI5_ONBOARD_GUIDE.md.
"""
import json
import time
import unittest

import rclpy
from std_msgs.msg import String

from as2_rescue_runtime import guardian as g
from as2_rescue_runtime.guardian import Config, GuardianCore

OFFBOARD, HOLD, RTL, LAND, POSCTL, MISSION = 14, 4, 5, 18, 2, 3


class Flight:
    """Drives a GuardianCore with a simulated clock and PX4/AS2 inputs."""

    def __init__(self, **config):
        config.setdefault('as2_timeout_s', 3.0)
        self.core = GuardianCore(Config(**config))
        self.t = 100.0
        self.armed, self.nav, self.height, self.takeoff_time = False, OFFBOARD, 0.0, 0
        self.as2_alive, self.as2_state = True, 3
        self.sent = []

    def tick(self, seconds=0.1, steps=None):
        for _ in range(steps or max(1, int(round(seconds / 0.1)))):
            self.t += 0.1
            self.core.on_status(self.t, self.armed, self.nav, self.takeoff_time)
            self.core.on_local_position(self.t, -self.height)
            if self.as2_alive:
                self.core.on_platform(self.t, self.as2_state)
                self.core.on_rescue(self.t)
            self.sent += self.core.step(self.t)
        return self.core.state

    def take_off(self, height=10.0):
        self.tick(0.5)                      # ground level recorded while disarmed
        self.armed = True
        self.tick(0.5)
        self.takeoff_time, self.height = 123456, height
        return self.tick(0.5)

    def px4_follows(self):
        """PX4 switches to the commanded mode."""
        last = self.sent[-1]['action']
        self.nav = g.TARGET_NAV[last]


class DecisionTests(unittest.TestCase):
    def test_monitoring_while_as2_alive(self):
        f = Flight()
        self.assertEqual(f.take_off(), g.MONITORING)
        self.assertEqual(f.tick(10), g.MONITORING)
        self.assertEqual(f.sent, [])

    def test_never_acts_on_the_ground(self):
        f = Flight()
        f.tick(1)
        f.as2_alive = False
        self.assertEqual(f.tick(10), g.ON_GROUND)       # disarmed
        f.armed = True
        self.assertEqual(f.tick(10), g.ON_GROUND)       # armed, no takeoff detected
        f.takeoff_time = 1                              # PX4 saw a takeoff but we are on the ground again
        self.assertEqual(f.tick(10), g.ON_GROUND)
        self.assertEqual(f.sent, [])
        accepted, reason = f.core.request(f.t, 'RTL')
        self.assertFalse(accepted)
        self.assertIn('not airborne', reason)

    def test_land_detected_wins_when_px4_bridges_it(self):
        f = Flight()
        f.take_off()
        f.core.on_land_detected(f.t, True)
        self.assertFalse(f.core.airborne(f.t))
        f.core.on_land_detected(f.t, False)
        self.assertTrue(f.core.airborne(f.t))

    def test_as2_loss_commands_rtl_once_px4_confirms(self):
        f = Flight(action='RTL')
        f.take_off()
        f.as2_alive = False
        self.assertEqual(f.tick(2.5), g.MONITORING)     # inside the timeout: nothing
        self.assertEqual(f.sent, [])
        self.assertEqual(f.tick(1.0), g.COMMANDING)
        self.assertEqual([c['command'] for c in f.sent], [20])
        f.px4_follows()
        self.assertEqual(f.tick(0.2), g.EMERGENCY_ACTIVE)
        f.tick(30)
        self.assertEqual(len(f.sent), 1, 'no second command after PX4 confirmed')
        self.assertEqual(f.core.snapshot(f.t)['emergency']['source'], 'as2_loss')

    def test_as2_loss_commands_land_when_chosen(self):
        f = Flight(action='LAND')
        f.take_off()
        f.as2_alive = False
        f.tick(3.5)
        self.assertEqual(f.sent[0]['command'], 21)
        f.px4_follows()
        self.assertEqual(f.tick(0.2), g.EMERGENCY_ACTIVE)

    def test_action_can_change_in_flight(self):
        f = Flight(action='RTL')
        f.take_off()
        self.assertTrue(f.core.set_action('land'))
        f.as2_alive = False
        f.tick(3.5)
        self.assertEqual(f.sent[0]['action'], 'LAND')
        with self.assertRaises(ValueError):
            f.core.set_action('hover')

    def test_rescue_state_silence_alone_is_as2_loss(self):
        f = Flight()
        f.take_off()
        f.core.on_platform(f.t, 3)
        for _ in range(40):                  # platform alive, runtime silent
            f.t += 0.1
            f.core.on_status(f.t, True, OFFBOARD, 1)
            f.core.on_platform(f.t, 3)
            f.sent += f.core.step(f.t)
        self.assertEqual(f.core.state, g.COMMANDING)
        self.assertIn('rescue/state', f.core.emergency['reason'])

    def test_retries_until_mode_changes(self):
        f = Flight(action='LAND', retry_period_s=1.0)
        f.take_off()
        f.as2_alive = False
        f.tick(3.1)
        f.tick(3.0)                          # PX4 does not switch: resend every second
        self.assertEqual([c['attempt'] for c in f.sent], [1, 2, 3, 4])
        f.px4_follows()
        f.tick(5)
        self.assertEqual(len(f.sent), 4)
        self.assertEqual(f.core.state, g.EMERGENCY_ACTIVE)

    def test_refused_rtl_falls_back_to_land(self):
        f = Flight(action='RTL', rtl_fallback_after=3)
        f.take_off()
        f.as2_alive = False
        f.tick(3.1)
        for _ in range(3):
            f.core.on_ack(f.t, 20, 2)        # DENIED (e.g. no home position)
            f.tick(1.0)
        self.assertEqual(f.sent[-1]['command'], 21)
        self.assertEqual([c['command'] for c in f.sent], [20, 20, 20, 21])
        self.assertEqual(f.core.emergency['refused'], 0)   # counter restarts for LAND
        f.px4_follows()
        self.assertEqual(f.tick(0.2), g.EMERGENCY_ACTIVE)

    def test_landing_as2_finishes_with_land_not_rtl(self):
        f = Flight(action='RTL')
        f.take_off(height=3.0)
        f.as2_state = 4                      # PlatformStatus.LANDING
        f.tick(0.5)
        f.as2_alive = False
        f.tick(3.5)
        self.assertEqual(f.sent[0]['action'], 'LAND')

    def test_px4_failsafe_already_returning_is_not_doubled(self):
        f = Flight()
        f.take_off()
        f.as2_alive = False
        f.nav = LAND                          # PX4 offboard-loss failsafe acted first
        self.assertEqual(f.tick(5), g.PX4_FAILSAFE)
        self.assertEqual(f.sent, [])

    def test_hold_after_offboard_loss_is_acted_on(self):
        f = Flight()
        f.take_off()
        f.as2_alive = False
        f.nav = HOLD                          # COM_OBL_RC_ACT = Hold
        f.tick(3.5)
        self.assertEqual(f.core.state, g.COMMANDING)

    def test_pilot_mode_is_never_overridden(self):
        f = Flight()
        f.take_off()
        f.nav = POSCTL
        f.as2_alive = False
        self.assertEqual(f.tick(5), g.PILOT_CONTROL)
        self.assertEqual(f.sent, [])

    def test_pilot_takeover_during_emergency_stands_down(self):
        f = Flight(action='LAND')
        f.take_off()
        f.as2_alive = False
        f.tick(3.5)
        f.px4_follows()
        f.tick(0.2)
        f.nav = POSCTL                        # RC pilot takes over during the land
        self.assertEqual(f.tick(0.2), g.STOOD_DOWN)
        f.nav = HOLD                          # pilot hands over to HOLD, AS2 still dead
        f.tick(10)
        self.assertEqual(f.core.state, g.STOOD_DOWN)
        self.assertEqual(len(f.sent), 1)

    def test_pilot_takeover_while_commanding_stops_resends(self):
        f = Flight(action='LAND')
        f.take_off()
        f.as2_alive = False
        f.tick(3.1)
        f.nav = POSCTL
        f.tick(5)
        self.assertEqual(len(f.sent), 1)
        self.assertEqual(f.core.state, g.STOOD_DOWN)

    def test_other_px4_modes_not_owned_by_as2(self):
        f = Flight()
        f.take_off()
        f.nav = MISSION
        f.as2_alive = False
        self.assertEqual(f.tick(5), g.OTHER_MODE)
        self.assertEqual(f.sent, [])

    def test_waits_for_as2_before_supervising(self):
        f = Flight()
        f.as2_alive = False                  # guardian started, AS2 never came up
        f.take_off()
        self.assertEqual(f.tick(10), g.WAITING_FOR_AS2)
        self.assertEqual(f.sent, [])

    def test_no_fc_link_means_no_action(self):
        f = Flight()
        f.take_off()
        f.core.status_at -= 10               # vehicle_status stopped
        self.assertEqual(f.core.step(f.t), [])
        self.assertEqual(f.core.state, g.NO_FC)
        self.assertFalse(f.core.request(f.t, 'RTL')[0])

    def test_gcs_request_works_with_as2_alive_and_is_not_doubled(self):
        f = Flight()
        f.take_off()
        accepted, _ = f.core.request(f.t, 'RTL', 'gcs')
        self.assertTrue(accepted)
        f.tick(0.1)
        self.assertEqual([c['command'] for c in f.sent], [20])
        accepted, reason = f.core.request(f.t, 'RTL', 'gcs')   # radio repeat
        self.assertTrue(accepted)
        self.assertIn('already', reason)
        f.tick(0.5)
        self.assertEqual(len(f.sent), 1)
        f.px4_follows()
        self.assertEqual(f.tick(0.2), g.EMERGENCY_ACTIVE)

    def test_gcs_escalates_rtl_to_land(self):
        f = Flight()
        f.take_off()
        f.core.request(f.t, 'RTL')
        f.tick(0.1)
        f.px4_follows()
        f.tick(0.2)
        f.core.request(f.t, 'LAND')
        f.tick(0.1)
        self.assertEqual([c['command'] for c in f.sent], [20, 21])
        f.px4_follows()
        self.assertEqual(f.tick(0.2), g.EMERGENCY_ACTIVE)
        self.assertEqual(f.core.emergency['action'], 'LAND')

    def test_gcs_request_with_as2_dead(self):
        f = Flight()
        f.take_off()
        f.nav = POSCTL                       # even a pilot mode: explicit operator emergency
        f.as2_alive = False
        f.tick(5)
        self.assertTrue(f.core.request(f.t, 'LAND')[0])
        f.tick(0.1)
        self.assertEqual(f.sent[-1]['command'], 21)

    def test_no_new_action_after_stand_down_until_as2_returns(self):
        f = Flight(action='RTL')
        f.take_off()
        f.as2_alive = False
        f.tick(3.5)
        f.px4_follows()
        f.tick(0.2)
        f.nav = OFFBOARD                     # something switched back to OFFBOARD
        self.assertEqual(f.tick(0.2), g.STOOD_DOWN)
        f.tick(5)
        self.assertEqual(len(f.sent), 1, 'stays out of the way while AS2 is still silent')
        f.as2_alive = True                   # AS2 restarted and took over
        self.assertEqual(f.tick(0.3), g.MONITORING)
        f.as2_alive = False                  # and died again: a new emergency
        f.tick(3.5)
        self.assertEqual(len(f.sent), 2)

    def test_disarm_after_emergency_resets(self):
        f = Flight(action='LAND')
        f.take_off()
        f.as2_alive = False
        f.tick(3.5)
        f.px4_follows()
        f.tick(0.2)
        f.height = 0.0                        # touched down, still armed
        self.assertEqual(f.tick(0.5), g.EMERGENCY_ACTIVE)
        f.armed, f.takeoff_time = False, 0
        self.assertEqual(f.tick(0.5), g.ON_GROUND)
        self.assertIsNone(f.core.emergency)

    def test_every_state_change_is_logged(self):
        f = Flight()
        f.take_off()
        f.as2_alive = False
        f.tick(3.5)
        texts = [text for _, text, _ in f.core.take_logs()]
        self.assertTrue(any('ON_GROUND -> MONITORING' in t for t in texts))
        self.assertTrue(any(t.startswith('EMERGENCY RTL (as2_loss)') for t in texts))
        self.assertEqual(f.core.take_logs(), [])

    def test_snapshot_is_json(self):
        f = Flight()
        f.take_off()
        data = json.loads(json.dumps(f.core.snapshot(f.t)))
        self.assertEqual(data['state'], g.MONITORING)
        self.assertTrue(data['as2']['alive'])
        self.assertTrue(data['px4']['airborne'])
        self.assertEqual(data['px4']['nav'], 'OFFBOARD')


def setUpModule():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/guardian_test'])


def tearDownModule():
    if rclpy.ok():
        rclpy.shutdown()


class NodeTests(unittest.TestCase):
    """The ROS node publishes PX4 VehicleCommand on AS2 loss and answers GCS requests."""

    def setUp(self):
        from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus
        from as2_msgs.msg import PlatformInfo
        from as2_rescue_runtime.fc_guardian import FcGuardian
        self.types = VehicleStatus, VehicleLocalPosition, PlatformInfo
        self.guardian = FcGuardian()
        self.guardian.set_parameters([rclpy.parameter.Parameter('as2_timeout_s', value=1.0)])
        self.probe = rclpy.create_node('guardian_probe', namespace='/guardian_test')
        self.commands, self.states = [], []
        from rclpy.qos import qos_profile_sensor_data
        self.probe.create_subscription(VehicleCommand, '/fmu/in/vehicle_command', self.commands.append,
                                       qos_profile_sensor_data)
        self.probe.create_subscription(String, 'fc_guardian/state', lambda m: self.states.append(json.loads(m.data)), 10)
        self.status_pub = self.probe.create_publisher(VehicleStatus, '/fmu/out/vehicle_status_v1', qos_profile_sensor_data)
        self.position_pub = self.probe.create_publisher(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                                        qos_profile_sensor_data)
        self.platform_pub = self.probe.create_publisher(PlatformInfo, 'platform/info', 10)
        self.rescue_pub = self.probe.create_publisher(String, 'rescue/state', 10)
        self.request_pub = self.probe.create_publisher(String, 'fc_guardian/request', 10)
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        self.action_pub = self.probe.create_publisher(String, 'fc_guardian/set_action', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def tearDown(self):
        self.guardian.destroy_node()
        self.probe.destroy_node()

    def run_for(self, seconds, armed=True, nav=OFFBOARD, as2=True, height=10.0):
        VehicleStatus, VehicleLocalPosition, PlatformInfo = self.types
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            status = VehicleStatus()
            status.arming_state = 2 if armed else 1
            status.nav_state = nav
            status.takeoff_time = 1 if armed else 0
            self.status_pub.publish(status)
            position = VehicleLocalPosition()
            position.z, position.z_valid = -height, True
            self.position_pub.publish(position)
            if as2:
                info = PlatformInfo()
                info.status.state = 3
                self.platform_pub.publish(info)
                self.rescue_pub.publish(String(data='{}'))
            for _ in range(5):
                rclpy.spin_once(self.guardian, timeout_sec=0.01)
                rclpy.spin_once(self.probe, timeout_sec=0.01)

    def test_as2_loss_publishes_vehicle_command(self):
        self.run_for(0.6, armed=False, height=0.0)
        self.run_for(1.0)
        self.assertEqual(self.guardian.core.state, g.MONITORING)
        self.assertEqual(self.commands, [])
        self.run_for(1.6, as2=False)
        self.assertTrue(self.commands, 'guardian must command PX4 when AS2 stops')
        command = self.commands[0]
        self.assertEqual(command.command, 20)
        self.assertEqual((command.target_system, command.target_component, command.from_external), (1, 1, True))
        self.run_for(0.5, nav=RTL, as2=False)
        sent = len(self.commands)
        self.run_for(1.5, nav=RTL, as2=False)
        self.assertEqual(len(self.commands), sent)
        self.assertEqual(self.states[-1]['state'], g.EMERGENCY_ACTIVE)

    def test_gcs_request_and_latched_action(self):
        self.action_pub.publish(String(data='LAND'))
        self.run_for(0.6, armed=False, height=0.0)
        self.assertEqual(self.guardian.get_parameter('emergency_action').value, 'LAND')
        self.run_for(0.5)
        self.request_pub.publish(String(data=json.dumps(dict(action='LAND', source='gcs', stamp=time.time()))))
        self.run_for(0.3)
        self.assertEqual([c.command for c in self.commands][:1], [21])
        self.request_pub.publish(String(data=json.dumps(dict(action='RTL', source='gcs', stamp=time.time() - 60))))
        self.run_for(0.3, nav=LAND)
        self.assertNotIn(20, [c.command for c in self.commands], 'stale request must be ignored')


if __name__ == '__main__':
    unittest.main()
