"""Camera and NCNN detection, isolated to selected vision cores; no flight behavior."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone0'),
        DeclareLaunchArgument('deployment_dir', description='Absolute path to RASPI5_DEPLOYMENT_PACKAGE'),
        DeclareLaunchArgument('vision_cores', default_value='2,3'),
        DeclareLaunchArgument('rtsp_url', default_value='rtsp://192.168.144.25:8554/main.264'),
        DeclareLaunchArgument('calibration_file', default_value=''),
        Node(package='as2_rescue_perception', executable='a8_camera',
             namespace=LaunchConfiguration('drone_id'), output='screen',
             prefix=['taskset -c ', LaunchConfiguration('vision_cores')],
             parameters=[{'rtsp_url': LaunchConfiguration('rtsp_url'),
                          'calibration_file': LaunchConfiguration('calibration_file'),
                          'frame_id': [LaunchConfiguration('drone_id'), '/camera_optical_frame']}]),
        Node(package='as2_rescue_perception', executable='person_detector',
             namespace=LaunchConfiguration('drone_id'), output='screen',
             prefix=['taskset -c ', LaunchConfiguration('vision_cores')],
             additional_env={'OMP_NUM_THREADS': '2', 'OPENBLAS_NUM_THREADS': '1'},
             parameters=[{'deployment_dir': LaunchConfiguration('deployment_dir'),
                          'ncnn_threads': 2}]),
    ])
