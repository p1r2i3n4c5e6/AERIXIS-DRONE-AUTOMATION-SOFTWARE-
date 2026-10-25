from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    parameters = {
        'bind_address': LaunchConfiguration('bind_address'),
        'udp_port': LaunchConfiguration('udp_port'),
        'gcs_host': LaunchConfiguration('gcs_host'),
        'gcs_port': LaunchConfiguration('gcs_port'),
        'drone_namespace': LaunchConfiguration('drone_namespace'),
        'mission_topic_prefix': LaunchConfiguration('mission_topic_prefix'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'home_latitude': LaunchConfiguration('home_latitude'),
        'home_longitude': LaunchConfiguration('home_longitude'),
        'home_altitude': LaunchConfiguration('home_altitude'),
        'field_width_m': LaunchConfiguration('field_width_m'),
        'field_length_m': LaunchConfiguration('field_length_m'),
        'manual_timeout_sec': LaunchConfiguration('manual_timeout_sec'),
        'px4_status_topic': LaunchConfiguration('px4_status_topic'),
    }
    args = [
        DeclareLaunchArgument('bind_address', default_value='0.0.0.0'),
        DeclareLaunchArgument('udp_port', default_value='14550'),
        DeclareLaunchArgument('gcs_host', default_value=''),
        DeclareLaunchArgument('gcs_port', default_value='14550'),
        DeclareLaunchArgument('drone_namespace', default_value='drone0'),
        DeclareLaunchArgument('mission_topic_prefix', default_value='/'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('home_latitude', default_value='47.397742'),
        DeclareLaunchArgument('home_longitude', default_value='8.545594'),
        DeclareLaunchArgument('home_altitude', default_value='488.0'),
        DeclareLaunchArgument('field_width_m', default_value='60.0'),
        DeclareLaunchArgument('field_length_m', default_value='40.0'),
        DeclareLaunchArgument('manual_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument(
            'px4_status_topic', default_value='/drone0/fmu/out/vehicle_status_v1'),
        Node(
            package='as2_mavlink_gateway',
            executable='mavlink_gateway',
            name='mavlink_gateway',
            output='screen',
            parameters=[parameters],
        ),
    ]
    return LaunchDescription(args)
