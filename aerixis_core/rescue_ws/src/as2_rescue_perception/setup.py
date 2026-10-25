from glob import glob
from setuptools import setup

name = 'as2_rescue_perception'
setup(name=name, version='0.1.0', packages=[name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                  ('share/' + name, ['package.xml']),
                  ('share/' + name + '/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': [
          'a8_camera = as2_rescue_perception.camera_node:main',
          'person_detector = as2_rescue_perception.detector_node:main']})
