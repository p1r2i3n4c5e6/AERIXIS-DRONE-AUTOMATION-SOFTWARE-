"""A8 mini person tracking for one aircraft: tracker process + ROS bridge (+ optional sim).

  vision:=preview  detection and selection, gimbal not commanded (default, safe on bench)
  vision:=track    detection, selection and gimbal tracking (props off for first test)
  vision:=sim      SITL: simulated camera/gimbal, no tracker process, no camera needed
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    namespace = LaunchConfiguration('drone_id').perform(context)
    vision = LaunchConfiguration('vision').perform(context)
    if vision not in ('preview', 'track', 'sim'):
        raise RuntimeError('vision must be preview, track or sim')
    actions = []
    if vision == 'sim':
        actions.append(Node(package='as2_a8_tracker', executable='a8_sim_vision', namespace=namespace,
                            output='screen', parameters=[{
                                'persons': LaunchConfiguration('sim_persons').perform(context),
                                'lose_after_s': float(LaunchConfiguration('sim_lose_after_s').perform(context)),
                                'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true'}]))
        return actions
    cores = LaunchConfiguration('cores').perform(context).split()
    detector_cores = LaunchConfiguration('detector_cores').perform(context).split()
    actions.append(ExecuteProcess(
        cmd=['ros2', 'run', 'as2_a8_tracker', 'a8_tracker', '--mode', vision,
             '--camera-ip', LaunchConfiguration('camera_ip').perform(context),
             '--stream', LaunchConfiguration('stream_port').perform(context),
             '--threads', LaunchConfiguration('threads').perform(context),
             '--gimbal-speed', LaunchConfiguration('gimbal_speed').perform(context), '--cores'] + cores
            + (['--detector-cores'] + detector_cores if detector_cores else []),
        output='screen', name='a8_tracker',
        # The tracker exits on purpose when it cannot start safely (e.g. gimbal centre not verified).
        # End the whole launch then, so the onboard service restarts the camera link (5-60 s backoff)
        # and shows it as restarting; before (2026-10-23, Pi 5) the launch kept running without a
        # tracker and the panel reported the link as running.
        on_exit=Shutdown(reason='a8_tracker exited')))
    actions.append(Node(package='as2_a8_tracker', executable='a8_tracker_bridge', namespace=namespace,
                        output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone0'),
        DeclareLaunchArgument('vision', default_value='preview'),
        DeclareLaunchArgument('camera_ip', default_value='192.168.144.25'),
        DeclareLaunchArgument('stream_port', default_value='8080'),
        DeclareLaunchArgument('threads', default_value='2'),
        # Gimbal response: normal (original tuning), fast, max (see tracker_process.GIMBAL_PROFILES).
        DeclareLaunchArgument('gimbal_speed', default_value='normal'),
        # Pi 5: tracker loop and video decoding on 0-1, the person detector alone on 2-3 (measured
        # 240 ms per inference vs 450 ms when everything shared 2-3). AS2 / flight nodes are not pinned.
        DeclareLaunchArgument('cores', default_value='0 1'),
        DeclareLaunchArgument('detector_cores', default_value='2 3'),
        DeclareLaunchArgument('sim_persons', default_value='20.0,15.0'),
        DeclareLaunchArgument('sim_lose_after_s', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=setup)])
