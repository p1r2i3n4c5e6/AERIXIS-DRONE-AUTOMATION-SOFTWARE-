"""Per-aircraft rescue services; camera and radio are separately configured."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

# What the C++ rate gate forwards to the Python nodes (in the drone namespace), and the remaps
# that point them at it.
GATED_TOPICS = ['self_localization/pose rescue/in/pose geometry_msgs/msg/PoseStamped 25',
                'self_localization/twist rescue/in/twist geometry_msgs/msg/TwistStamped 25',
                'sensor_measurements/imu rescue/in/imu sensor_msgs/msg/Imu 50',   # vibration: >= 20 samples in 0.5 s
                # Camera frames for the track logger (it records changes; the runtime keeps 23 Hz).
                'vision/frame rescue/in/vision_frame std_msgs/msg/String 10',
                # Pose/twist for nodes that keep the latest value for their own 5-10 Hz loop
                # (swarm agent, delivery zone, route): 12.5 Hz is <= 80 ms old against their 0.5 s
                # freshness limit, and each skipped wake-up saves Python executor time (2026-10-25).
                'self_localization/pose rescue/in/pose_slow geometry_msgs/msg/PoseStamped 12.5',
                'self_localization/twist rescue/in/twist_slow geometry_msgs/msg/TwistStamped 12.5']
GATE_REMAPS = [('self_localization/pose', 'rescue/in/pose'), ('self_localization/twist', 'rescue/in/twist'),
               ('sensor_measurements/imu', 'rescue/in/imu'), ('/tf', 'rescue/in/tf')]
SLOW_POSE_NODES = ('swarm_agent', 'delivery_zone', 'rescue_route')
SLOW_REMAPS = [('self_localization/pose', 'rescue/in/pose_slow'), ('self_localization/twist', 'rescue/in/twist_slow'),
               ('sensor_measurements/imu', 'rescue/in/imu'), ('/tf', 'rescue/in/tf')]


def generate_launch_description():
    namespace = LaunchConfiguration('drone_id')
    profile = LaunchConfiguration('profile')
    actions = [DeclareLaunchArgument('drone_id', default_value='drone0'),
        DeclareLaunchArgument('profile', default_value=PathJoinSubstitution([
            FindPackageShare('as2_rescue_runtime'), 'config', 'aircraft.yaml'])),
        DeclareLaunchArgument('start_gimbal', default_value='false',
            description='Starts UDP attitude queries to the configured A8 camera'),
        DeclareLaunchArgument('start_route', default_value='true',
            description='Publishes rescue/route from a validated planner bundle'),
        DeclareLaunchArgument('start_surface', default_value='true',
            description='Publishes rescue/surface; default source is unverified'),
        DeclareLaunchArgument('start_camera_tf', default_value='false',
            description='Publishes the measured world->camera_optical_frame transform'),
        DeclareLaunchArgument('override', default_value='',
            description='Optional per-aircraft parameter file applied after the profile'),
        DeclareLaunchArgument('target_system_id', default_value='0',
            description='PX4 MAV_SYS_ID: picks this aircraft\'s own return altitude (0 = from drone_id)'),
        DeclareLaunchArgument('fmu_prefix', default_value='',
            description="PX4 topic prefix ('' = /fmu/... on a real FC, '/drone0' = namespaced SITL)"),
        DeclareLaunchArgument('rate_gate', default_value='true',
            description='Python nodes read pose/twist/IMU/tf through the C++ rate gate (25/25/50/30 Hz)')]
    profiles = [profile, ParameterFile(LaunchConfiguration('override'), allow_substs=False)]
    actions.append(OpaqueFunction(function=lambda context: [] if LaunchConfiguration('override').perform(context)
        else [SetLaunchConfiguration('override', profile.perform(context))]))
    system = {'system_id': ParameterValue(LaunchConfiguration('target_system_id'), value_type=int)}
    prefix = {'fmu_prefix': ParameterValue(LaunchConfiguration('fmu_prefix'), value_type=str)}
    # Rate gate (Pi 5, 2026-10-25): AS2 publishes pose/twist/IMU/tf at the PX4 odometry rate
    # (~100 Hz). Humble's Python executor costs a few ms per message on a Pi, so every Python node
    # listening at 100 Hz burned 20-50 % of a core. The C++ gate takes them once and the Python
    # nodes read rate-limited copies (remapped, code unchanged). AS2's own C++ nodes and the FC
    # guardian's PX4 status keep the full rate. rate_gate:=false restores the direct topics.
    actions.append(Node(package='as2_rescue_rate_gate', executable='rate_gate', name='rescue_rate_gate',
        namespace=namespace, output='screen', condition=IfCondition(LaunchConfiguration('rate_gate')),
        parameters=[{'topics': GATED_TOPICS, 'tf_in': '/tf', 'tf_out': 'rescue/in/tf', 'tf_hz': 30.0}]))

    def python_nodes(context):
        gated = LaunchConfiguration('rate_gate').perform(context) == 'true'
        remap = lambda executable: (SLOW_REMAPS if executable in SLOW_POSE_NODES else GATE_REMAPS) if gated else []
        node = lambda executable, parameters, condition=None: Node(
            package='as2_rescue_runtime', executable=executable, namespace=namespace,
            parameters=parameters, remappings=remap(executable), output='screen', condition=condition)
        nodes = [node(executable, ([system] if executable == 'rescue_runtime' else [])
                      + ([prefix] if executable == 'payload_aux' else []) + profiles)
                 for executable in ('rescue_runtime', 'geolocator', 'payload_aux', 'swarm_agent', 'delivery_zone')]
        nodes.append(node('a8_gimbal', profiles, IfCondition(LaunchConfiguration('start_gimbal'))))
        # The surface estimator is safe to run unverified: geolocation rejects an
        # unverified datum, so publishing it cannot authorise a wrong fix.
        nodes.append(node('rescue_surface', profiles, IfCondition(LaunchConfiguration('start_surface'))))
        # Route and camera transform stay off until a validated bundle and measured
        # mounting offset exist; enabling them without those would be unsafe.
        nodes.append(node('rescue_route', profiles, IfCondition(LaunchConfiguration('start_route'))))
        return nodes
    actions.append(OpaqueFunction(function=python_nodes))
    actions.append(Node(package='as2_rescue_runtime', executable='a8_camera_transform',
        namespace=namespace, parameters=profiles, output='screen',
        condition=IfCondition(LaunchConfiguration('start_camera_tf'))))
    return LaunchDescription(actions)
