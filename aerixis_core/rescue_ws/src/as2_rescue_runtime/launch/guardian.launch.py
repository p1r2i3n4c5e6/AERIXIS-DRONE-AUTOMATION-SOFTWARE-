"""FC guardian with its own rate gate: one supervised link (rescue_ws/scripts/onboard_links.py).

The guardian sends emergency RTL/LAND straight to PX4 when the AERIXIS core stops, so it must not
depend on the core. It needs PX4's local position only for "airborne above 1 m"; PX4 publishes it
at ~100 Hz, which cost the Python guardian ~25 % of a Pi 5 core (2026-10-25). This link starts a
C++ rate gate for that one topic (20 Hz). PX4 status, land detector, command acks and the
emergency commands stay direct. If either process exits, the whole launch ends and the
supervisor restarts both.

  ros2 launch as2_rescue_runtime guardian.launch.py drone_id:=drone0 target_system:=1
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    drone = value('drone_id')
    prefix = value('fmu_prefix').strip().rstrip('/')
    fmu = (prefix + '/fmu') if prefix else '/fmu'      # same rule as fc_guardian.py
    position = fmu + '/out/vehicle_local_position'
    gated = 'rescue/guardian/vehicle_local_position'
    gate = Node(package='as2_rescue_rate_gate', executable='rate_gate', name='guardian_rate_gate',
                namespace=drone, output='screen',
                parameters=[{'topics': ['{} {} px4_msgs/msg/VehicleLocalPosition 20'.format(position, gated)]}])
    parameters = {'target_system': int(value('target_system')), 'emergency_action': value('emergency_action'),
                  'as2_timeout_s': float(value('as2_timeout_s'))}
    if prefix:
        parameters['fmu_prefix'] = prefix
    guardian = Node(package='as2_rescue_runtime', executable='fc_guardian', namespace=drone, output='screen',
                    parameters=[parameters], remappings=[(position, gated)])
    stop = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[EmitEvent(event=Shutdown(
        reason='guardian link: a process exited'))])) for node in (gate, guardian)]
    return [gate, guardian] + stop


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone0'),
        DeclareLaunchArgument('target_system', default_value='1', description='PX4 MAV_SYS_ID'),
        DeclareLaunchArgument('emergency_action', default_value='RTL'),
        DeclareLaunchArgument('as2_timeout_s', default_value='3.0'),
        DeclareLaunchArgument('fmu_prefix', default_value='',
                              description="PX4 topic prefix ('' = /fmu/... on a real FC, '/drone0' = SITL)"),
        OpaqueFunction(function=setup)])
