"""Start the AERIXIS core PX4 platform stack and MAVLink gateway in one terminal."""

from pathlib import Path
import re
import socket

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction,
    GroupAction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _workspace_root():
    prefix = Path(get_package_prefix('as2_mavlink_gateway')).resolve()
    candidates = [*prefix.parents, Path.cwd(), *Path.cwd().parents]
    for candidate in candidates:
        if (candidate / 'px4_sitl' / 'run_px4_sitl.sh').is_file():
            return candidate
    raise RuntimeError(
        'Cannot locate the aerostack2 workspace containing px4_sitl/run_px4_sitl.sh.')


def _include(package, relative_launch_path, launch_args):
    path = Path(get_package_share_directory(package)) / 'launch' / relative_launch_path
    return GroupAction(scoped=True, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(path)), launch_arguments=launch_args.items())
    ])


def generate_launch_description():
    workspace = _workspace_root()
    config_dir = str(workspace / 'projects' / 'px4_vision' / 'config')
    namespace = LaunchConfiguration('drone_namespace')
    config = LaunchConfiguration('config_dir')
    common = {'namespace': namespace, 'config_file': [config, '/config.yaml']}

    actions = [
        DeclareLaunchArgument('drone_namespace', default_value='drone0'),
        DeclareLaunchArgument('config_dir', default_value=config_dir),
        DeclareLaunchArgument('gcs_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('gcs_port', default_value='14551'),
        DeclareLaunchArgument('udp_port', default_value='14550'),
        DeclareLaunchArgument('xrce_agent_port', default_value='8888'),
        DeclareLaunchArgument(
            'gateway_start_delay', default_value='8.0',
            description='Seconds to let AERIXIS core behaviors and services initialize before gateway startup.'),
        DeclareLaunchArgument(
            'start_xrce_agent', default_value='true',
            description='Set false only when a Micro XRCE-DDS Agent is already running.'),
        DeclareLaunchArgument('home_latitude', default_value='47.397742'),
        DeclareLaunchArgument('home_longitude', default_value='8.545594'),
        DeclareLaunchArgument('home_altitude', default_value='488.0'),
        Node(
            package='as2_mavlink_gateway', executable='mavlink_gateway',
            name='mavlink_gateway', output='screen', emulate_tty=True,
            parameters=[{
                'drone_namespace': namespace,
                'mission_topic_prefix': '/',
                'use_sim_time': False,
                'bind_address': '0.0.0.0',
                'udp_port': LaunchConfiguration('udp_port'),
                'gcs_host': LaunchConfiguration('gcs_host'),
                'gcs_port': LaunchConfiguration('gcs_port'),
                'home_latitude': LaunchConfiguration('home_latitude'),
                'home_longitude': LaunchConfiguration('home_longitude'),
                'home_altitude': LaunchConfiguration('home_altitude'),
            }]),
    ]

    # DroneInterfaceTeleop waits briefly for each behavior's action/service. Let
    # the platform, behavior servers, and trajectory generator come up first.
    gateway_action = actions.pop()
    actions.append(TimerAction(
        period=LaunchConfiguration('gateway_start_delay'), actions=[gateway_action]))

    agent = ExecuteProcess(
        cmd=[str(workspace / 'install' / 'px4_microxrce_agent' / 'bin' /
                 'MicroXRCEAgent'), 'udp4', '-p', LaunchConfiguration('xrce_agent_port')],
        cwd=str(workspace), output='screen', emulate_tty=True,
        name='micro_xrce_dds_agent',
        env={
            'LD_LIBRARY_PATH': [
                str(workspace / 'install' / 'px4_microxrce_agent' / 'lib'), ':',
                '/opt/ros/humble/lib', ':', EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
            ],
        })
    sender = None
    external_log_target = ('127.0.0.1', 14552)
    severity_pattern = re.compile(
        r'\b(error|fatal|failed|failure|exception|warning|warn)\b', re.IGNORECASE)

    def relay_process_output(event):
        nonlocal sender
        if sender is None:
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        output = event.text.decode('utf-8', errors='replace')
        for line in output.splitlines():
            match = severity_pattern.search(line)
            if not match:
                continue
            severity = 'WARN' if match.group(1).lower() in ('warning', 'warn') else 'ERROR'
            source = getattr(event, 'process_name', 'AERIXIS core process')
            packet = f'{severity}|{source}|{line.strip()}'.encode(
                'utf-8', errors='replace')[:3500]
            try:
                sender.sendto(packet, external_log_target)
            except OSError:
                pass
        return []

    def report_process_exit(event, _context):
        nonlocal sender
        code = int(event.returncode)
        if code not in (0, 130, -2, -15, 143):
            if sender is None:
                sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            source = getattr(event, 'process_name', 'AERIXIS core process')
            packet = f'ERROR|{source}|Process exited with code {code}'.encode()
            try:
                sender.sendto(packet, external_log_target)
            except OSError:
                pass
        return []

    actions[0:0] = [
        RegisterEventHandler(OnProcessIO(
            on_stdout=relay_process_output, on_stderr=relay_process_output)),
        RegisterEventHandler(OnProcessExit(on_exit=report_process_exit)),
    ]
    actions.extend([
        # Existing agent installations can be reused without binding UDP 8888 twice.
        GroupAction(
            scoped=True, actions=[agent],
            condition=IfCondition(LaunchConfiguration('start_xrce_agent'))),
        _include('as2_platform_pixhawk', 'pixhawk_launch.py', {
            'namespace': namespace,
            'platform_config_file': [config, '/config.yaml'],
        }),
        _include('as2_state_estimator', 'state_estimator_launch.py', {
            **common, 'plugin_name': 'raw_odometry',
        }),
        _include('as2_motion_controller', 'controller_launch.py', {
            **common,
            'plugin_name': 'pid_speed_controller',
            'plugin_config_file': [config, '/pid_speed_controller.yaml'],
        }),
        _include('as2_behaviors_motion', 'motion_behaviors_launch.py', {
            **common, 'use_sim_time': 'false',
        }),
        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(
                    Path(get_package_share_directory('as2_behaviors_trajectory_generation')) /
                    'generate_polynomial_trajectory_behavior' / 'launch' /
                    'generate_polynomial_trajectory_behavior_launch.py')),
                launch_arguments={**common, 'use_sim_time': 'false'}.items()),
        ]),
        ExecuteProcess(
            cmd=[str(Path(get_package_prefix('as2_python_api')) / 'lib' /
                     'as2_python_api' / 'mission_executor'), '--n', namespace],
            output='screen', emulate_tty=True, name='mission_executor'),
    ])
    return LaunchDescription(actions)
