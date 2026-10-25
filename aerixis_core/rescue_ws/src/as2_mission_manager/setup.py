from setuptools import setup
name = 'as2_mission_manager'
setup(name=name,version='0.1.0',packages=[name],
      data_files=[('share/ament_index/resource_index/packages',['resource/'+name]),
                  ('share/'+name,['package.xml'])],
      install_requires=['setuptools'],
      entry_points={'console_scripts':['mission_manager = as2_mission_manager.server:main']})
