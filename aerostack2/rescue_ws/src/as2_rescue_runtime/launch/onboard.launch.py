"""Everything one aircraft runs onboard, except the radios and the camera tracker.

  AS2 core     platform (PX4 via Micro XRCE-DDS), GNSS state estimator, PID speed
               controller, takeoff/land/go-to behaviours
  Rescue       rescue_runtime, route approval + plan origin, surface, geolocator,
               payload AUX adapter, swarm agent, delivery zone, track logger,
               fc_health (GPS fix/satellites/HDOP and PX4 sensor faults for the GCS card)

The Micro XRCE-DDS agent, GCS telemetry gateway, mesh bridge, mission manager and the A8
tracker are started separately (see rescue_ws/scripts/as2_pi_launcher.py) so each link can
be connected and restarted on its own.

  ros2 launch as2_rescue_runtime onboard.launch.py drone_id:=drone0 \
      override:=$HOME/.config/as2/drone0_runtime.yaml
"""
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, launch_file, arguments):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=list(arguments.items()))


def as2_config(drone_id, fmu_prefix, target_system_id):
    """Per-drone copy of as2_onboard.yaml with the platform's PX4 topic prefix and system ID."""
    share = Path(get_package_share_directory('as2_rescue_runtime'))
    documents = list(yaml.safe_load_all((share / 'config' / 'as2_onboard.yaml').read_text()))
    merged = {}
    for document in documents:
        for key, value in (document or {}).items():
            merged.setdefault(key, {}).update(value)
    platform = merged['/**']['platform']['ros__parameters']
    # An empty prefix must not be written: as2_core reads this file as text and `fmu_prefix: ''`
    # becomes the two characters '' (topics ''/fmu/..., platform dies). Left out, the platform's
    # own default applies, overridden by the fmu_prefix launch argument passed below.
    if fmu_prefix:
        platform['fmu_prefix'] = fmu_prefix
    else:
        platform.pop('fmu_prefix', None)
    platform['target_system_id'] = int(target_system_id)
    folder = Path.home() / '.cache' / 'as2'
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / ('%s_as2.yaml' % drone_id)
    path.write_text(yaml.safe_dump(merged, sort_keys=False))
    return str(path)


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    drone = value('drone_id')
    sim_time = value('use_sim_time')
    config = as2_config(drone, value('fmu_prefix'), value('target_system_id'))
    controller = dict(namespace=drone, config_file=config, plugin_name='pid_speed_controller',
                      use_sim_time=sim_time)
    # The plugin's packaged controller_default.yaml has all gains at zero (no mode can be
    # served); ship the px4_vision tuning unless a measured per-aircraft file is given.
    controller['plugin_config_file'] = value('controller_config') or os.path.join(
        get_package_share_directory('as2_rescue_runtime'), 'config', 'pid_speed_controller.yaml')
    actions = []
    if value('start_as2') == 'true':
        actions += [
            # fmu_prefix passed explicitly: otherwise as2_core takes the default from the platform's
            # YAML text, where `fmu_prefix: ''` becomes the two characters '' and every PX4 topic is
            # ''/fmu/... (InvalidTopicNameError, platform dies; real Pi 5, 2026-10-24).
            include('as2_platform_pixhawk', 'pixhawk_launch.py',
                    dict(namespace=drone, platform_config_file=config, fmu_prefix=value('fmu_prefix'))),
            include('as2_state_estimator', 'state_estimator_launch.py',
                    dict(namespace=drone, config_file=config, plugin_name='raw_odometry',
                         use_sim_time=sim_time)),
            include('as2_motion_controller', 'controller_launch.py', controller),
            include('as2_behaviors_motion', 'motion_behaviors_launch.py',
                    dict(namespace=drone, config_file=config, use_sim_time=sim_time)),
        ]
    profile = value('profile')
    override = value('override')
    rescue = dict(drone_id=drone, profile=profile, start_gimbal='false', start_route='true',
                  start_surface='true', start_camera_tf='false', target_system_id=value('target_system_id'),
                  fmu_prefix=value('fmu_prefix'), rate_gate=value('rate_gate'))
    if override:
        rescue['override'] = override
    actions.append(include('as2_rescue_runtime', 'rescue.launch.py', rescue))
    actions.append(Node(package='as2_rescue_runtime', executable='fc_health', namespace=drone, output='screen',
                        parameters=[{'fmu_prefix': value('fmu_prefix'), 'mavlink_url': value('fc_mavlink_url'),
                                     'mavlink_baud': int(value('fc_mavlink_baud'))}]))
    if value('start_logger') == 'true':
        # Camera frames at 10 Hz through the rate gate (rescue.launch.py): the logger records
        # changes, and 23 Hz cost it ~10 % of a Pi 5 core (2026-10-25).
        gated = [('vision/frame', 'rescue/in/vision_frame')] if value('rate_gate') == 'true' else []
        actions.append(Node(package='as2_rescue_runtime', executable='track_logger', namespace=drone,
                            output='screen', parameters=[{'use_sim_time': sim_time == 'true'}],
                            remappings=gated))
    return actions


def generate_launch_description():
    share = get_package_share_directory('as2_rescue_runtime')
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone0'),
        DeclareLaunchArgument('fmu_prefix', default_value='',
                              description="PX4 uXRCE-DDS topic prefix ('' = /fmu/..., '/drone0' = /drone0/fmu/...)"),
        DeclareLaunchArgument('target_system_id', default_value='1', description='PX4 MAV_SYS_ID'),
        DeclareLaunchArgument('profile', default_value=os.path.join(share, 'config', 'aircraft.yaml')),
        DeclareLaunchArgument('override', default_value='',
                              description='Per-aircraft parameter file written by the Pi launcher'),
        DeclareLaunchArgument('controller_config', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_as2', default_value='true'),
        DeclareLaunchArgument('start_logger', default_value='true'),
        DeclareLaunchArgument('rate_gate', default_value='true',
                              description='Python nodes read high-rate topics through the C++ rate gate'),
        DeclareLaunchArgument('fc_mavlink_url', default_value='',
                              description="Read-only PX4 MAVLink tap for sensor health and STATUSTEXT "
                                          "('/dev/ttyACM0' = FC USB on the Pi, 'udpin:0.0.0.0:14540' = SITL instance 0)"),
        DeclareLaunchArgument('fc_mavlink_baud', default_value='115200'),
        OpaqueFunction(function=setup)])
