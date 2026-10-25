from glob import glob
from setuptools import setup
name='as2_rescue_runtime'
setup(name=name,version='0.2.0',packages=[name],
 data_files=[('share/ament_index/resource_index/packages',['resource/'+name]),
 ('share/'+name,['package.xml']),('share/'+name+'/config',glob('config/*')),
 ('share/'+name+'/launch',glob('launch/*.launch.py'))],
 install_requires=['setuptools'],entry_points={'console_scripts':[
 'rescue_runtime = as2_rescue_runtime.runtime_node:main',
 'delivery_zone = as2_rescue_runtime.delivery_zone_node:main',
 'a8_gimbal = as2_rescue_runtime.gimbal_node:main',
 'geolocator = as2_rescue_runtime.geolocator_node:main',
 'payload_aux = as2_rescue_runtime.payload_node:main',
 'mesh_bridge = as2_rescue_runtime.mesh_bridge:main',
 'swarm_agent = as2_rescue_runtime.swarm_agent:main',
 'swarm_authority = as2_rescue_runtime.swarm_node:main',
 'rescue_surface = as2_rescue_runtime.surface_node:main',
 'track_logger = as2_rescue_runtime.track_logger:main',
 'rescue_route = as2_rescue_runtime.route_node:main',
 'a8_camera_transform = as2_rescue_runtime.camera_tf_node:main',
 'fc_health = as2_rescue_runtime.fc_health_node:main',
 'fc_guardian = as2_rescue_runtime.fc_guardian:main']})
