"""Shared telemetry radio: a gateway acts only on packets for its own system ID; target_system 0
(all aircraft) is honoured only for the fleet-wide emergency RTL/LAND."""
from types import SimpleNamespace
import unittest

from as2_mavlink_gateway.protocol import ROUTE_AS2, ROUTE_EMERGENCY, addressed_to, command_route

P = (0.0,) * 7
OWN = 3
ARM, TAKEOFF, LAND, RTL, SET_MODE, PAUSE = 400, 22, 21, 20, 176, 193


class CommandRouteTest(unittest.TestCase):
    def test_own_system_reaches_as2(self):
        for command in (ARM, TAKEOFF, PAUSE):
            self.assertEqual(command_route(command, OWN, 190, P, OWN), (ROUTE_AS2, None))
        self.assertEqual(command_route(LAND, OWN, 190, P, OWN), (ROUTE_AS2, None))  # AS2 land
        self.assertEqual(command_route(SET_MODE, OWN, 190, (1.0, 6.0, 0, 0, 0, 0, 0), OWN), (ROUTE_AS2, None))

    def test_foreign_system_is_ignored(self):
        for command in (ARM, TAKEOFF, LAND, PAUSE, RTL):
            for target in (1, 2, 4, 255):
                self.assertEqual(command_route(command, target, 190, P, OWN), (None, None), (command, target))
        self.assertEqual(command_route(SET_MODE, 2, 190, (1.0, 4.0, 5.0, 0, 0, 0, 0), OWN), (None, None))

    def test_broadcast_only_for_fleet_wide_emergency(self):
        for command in (ARM, TAKEOFF, PAUSE):
            self.assertEqual(command_route(command, 0, 0, P, OWN), (None, None), command)
        self.assertEqual(command_route(LAND, 0, 190, P, OWN), (None, None))  # AS2 land is not fleet-wide
        self.assertEqual(command_route(SET_MODE, 0, 0, (1.0, 6.0, 0, 0, 0, 0, 0), OWN), (None, None))  # OFFBOARD
        self.assertEqual(command_route(RTL, 0, 0, P, OWN), (ROUTE_EMERGENCY, 'RTL'))
        self.assertEqual(command_route(LAND, 0, 1, P, OWN), (ROUTE_EMERGENCY, 'LAND'))
        self.assertEqual(command_route(SET_MODE, 0, 0, (1.0, 4.0, 6.0, 0, 0, 0, 0), OWN), (ROUTE_EMERGENCY, 'LAND'))

    def test_own_emergency(self):
        self.assertEqual(command_route(RTL, OWN, 1, P, OWN), (ROUTE_EMERGENCY, 'RTL'))
        self.assertEqual(command_route(LAND, OWN, 1, P, OWN), (ROUTE_EMERGENCY, 'LAND'))

    def test_addressed_to(self):
        self.assertTrue(addressed_to(OWN, OWN))
        self.assertFalse(addressed_to(0, OWN))
        self.assertFalse(addressed_to(1, OWN))
        self.assertFalse(addressed_to(None, OWN))


class Message(SimpleNamespace):
    def get_type(self):
        return self.kind

    def get_srcSystem(self):
        return 255

    def get_srcComponent(self):
        return 190


class FakeGateway:
    """Just enough of MavlinkGateway for handle_message: records what would have happened."""

    def __init__(self, own):
        self.own = own
        self.calls = []
        self.acks = []
        self.fleet_relay = None
        self.mission_running = False
        self.gcs_last_heartbeat_time = 0.0
        self.recent_ros_logs = []
        self.mavlink = SimpleNamespace(MAV_RESULT_UNSUPPORTED=3, MAV_MISSION_ACCEPTED=0)
        self.connection = SimpleNamespace(target_system=255, target_component=190,
                                          mav=SimpleNamespace(command_ack_send=lambda c, r: self.acks.append((c, r))))

    def get_parameter(self, name):
        assert name == 'system_id'
        return SimpleNamespace(value=self.own)

    def emergency_command(self, action, command):
        self.calls.append(('emergency', action, command))

    def command_long(self, message, *_):
        self.calls.append(('as2', message.command))

    def begin_mission_upload(self, message, *_):
        self.calls.append(('mission_count', message.count))

    def receive_mission_item(self, message, *_):
        self.calls.append(('mission_item', message.seq))

    def stop_mission(self):
        self.calls.append(('stop_mission',))

    def _mission_ack(self, *args):
        self.calls.append(('mission_ack',) + args)

    def manual_control(self, message):
        self.calls.append(('manual', message.x))


def command_long(command, target, component=190, params=P):
    return Message(kind='COMMAND_LONG', command=command, target_system=target, target_component=component,
                   param1=params[0], param2=params[1], param3=params[2], param4=params[3],
                   param5=params[4], param6=params[5], param7=params[6])


class HandleMessageTest(unittest.TestCase):
    """The real MavlinkGateway.handle_message against a fake node."""

    @classmethod
    def setUpClass(cls):
        try:
            from as2_mavlink_gateway.gateway_node import MavlinkGateway
        except ImportError as exc:  # ROS / AS2 Python API not sourced
            raise unittest.SkipTest(str(exc))
        cls.handle = staticmethod(MavlinkGateway.handle_message)

    def run_messages(self, *messages):
        gateway = FakeGateway(OWN)
        for message in messages:
            self.handle(gateway, message)
        return gateway

    def test_foreign_and_broadcast_commands_do_nothing(self):
        gateway = self.run_messages(command_long(ARM, 1, params=(1.0,) + P[1:]), command_long(TAKEOFF, 2),
                                    command_long(ARM, 0), command_long(TAKEOFF, 0), command_long(LAND, 0, 190),
                                    command_long(RTL, 2, 1), command_long(PAUSE, 4))
        self.assertEqual(gateway.calls, [])
        self.assertEqual(gateway.acks, [])
        self.assertEqual(gateway.connection.target_system, 255)  # reply path untouched

    def test_own_commands_reach_as2(self):
        gateway = self.run_messages(command_long(ARM, OWN), command_long(TAKEOFF, OWN))
        self.assertEqual(gateway.calls, [('as2', ARM), ('as2', TAKEOFF)])

    def test_fleet_wide_and_own_emergency(self):
        gateway = self.run_messages(command_long(RTL, 0, 0), command_long(LAND, OWN, 1))
        self.assertEqual(gateway.calls, [('emergency', 'RTL', RTL), ('emergency', 'LAND', LAND)])

    def test_command_int_emergency_only(self):
        def command_int(command, target, component=0):
            return Message(kind='COMMAND_INT', command=command, target_system=target, target_component=component,
                           param1=0.0, param2=0.0, param3=0.0, param4=0.0, x=0, y=0, z=0.0)
        gateway = self.run_messages(command_int(RTL, 0), command_int(ARM, 0), command_int(ARM, 2),
                                    command_int(ARM, OWN))
        self.assertEqual(gateway.calls, [('emergency', 'RTL', RTL)])
        self.assertEqual(gateway.acks, [(ARM, 3)])  # own, not supported: refused, never executed

    def test_mission_upload_clear_and_sticks_need_own_system(self):
        foreign = [Message(kind='MISSION_COUNT', target_system=t, count=3, mission_type=0) for t in (0, 1)]
        foreign += [Message(kind='MISSION_ITEM_INT', target_system=1, seq=0),
                    Message(kind='MISSION_CLEAR_ALL', target_system=0),
                    Message(kind='MANUAL_CONTROL', target=1, x=500, y=0, z=500, r=0),
                    Message(kind='MANUAL_CONTROL', target=0, x=500, y=0, z=500, r=0)]
        self.assertEqual(self.run_messages(*foreign).calls, [])
        own = self.run_messages(Message(kind='MISSION_COUNT', target_system=OWN, count=3, mission_type=0),
                                Message(kind='MISSION_ITEM_INT', target_system=OWN, seq=0),
                                Message(kind='MANUAL_CONTROL', target=OWN, x=500, y=0, z=500, r=0),
                                Message(kind='MISSION_CLEAR_ALL', target_system=OWN))
        self.assertEqual(own.calls, [('mission_count', 3), ('mission_item', 0), ('manual', 500),
                                     ('mission_ack', 255, 190, 0)])


if __name__ == '__main__':
    unittest.main()


class GcsDisarmTest(unittest.TestCase):
    """AERIXIS core force-disarms PX4, so a GCS DISARM reaches it only for a landed aircraft."""

    @classmethod
    def setUpClass(cls):
        try:
            from as2_mavlink_gateway import gateway_node
            from as2_msgs.msg import PlatformStatus
        except ImportError as exc:  # ROS / AS2 Python API not sourced
            raise unittest.SkipTest(str(exc))
        cls.node, cls.status = gateway_node, PlatformStatus

    def gateway(self, state, age=0.1):
        import time
        node = self.node.MavlinkGateway
        acks, disarms = [], []
        gateway = SimpleNamespace(
            mavlink=SimpleNamespace(MAV_CMD_COMPONENT_ARM_DISARM=ARM, MAV_CMD_NAV_TAKEOFF=TAKEOFF, MAV_CMD_NAV_LAND=LAND,
                                    MAV_CMD_DO_SET_MODE=SET_MODE, MAV_CMD_DO_PAUSE_CONTINUE=PAUSE,
                                    MAV_RESULT_UNSUPPORTED=3, MAV_RESULT_DENIED=2),
            drone=SimpleNamespace(disarm=lambda: disarms.append(1) or True),
            platform=SimpleNamespace(status=SimpleNamespace(state=state)), last_platform_time=time.monotonic() - age,
            connection=SimpleNamespace(mav=SimpleNamespace(command_ack_send=lambda c, r: acks.append(r))),
            get_logger=lambda: SimpleNamespace(warning=lambda *_: None, info=lambda *_: None, error=lambda *_: None),
            mission_running=False, mission_total=0)
        gateway._on_ground = lambda: node._on_ground(gateway)
        gateway._command_result = node._command_result
        node.command_long(gateway, command_long(ARM, OWN), OWN, 1)
        return acks, disarms

    def test_disarm_refused_unless_landed(self):
        S = self.status
        for state in (S.FLYING, S.TAKING_OFF, S.LANDING, S.EMERGENCY):
            self.assertEqual(self.gateway(state), ([2], []), state)          # DENIED, motors untouched
        self.assertEqual(self.gateway(S.LANDED), ([0], [1]))
        self.assertEqual(self.gateway(S.LANDED, age=5.0), ([2], []))         # stale platform info
