import os
from glob import glob
from setuptools import setup

name = 'as2_a8_tracker'


def tree(source, target):
    """Install the vendored tracker (code, OC-SORT, NCNN and NanoTrack models) unchanged."""
    entries = []
    for folder, directories, files in os.walk(source):
        directories[:] = [d for d in directories if d not in ('__pycache__', 'logs')]
        paths = [os.path.join(folder, f) for f in files if not f.endswith('.pyc')]
        if paths:
            entries.append((os.path.join(target, os.path.relpath(folder, source)), paths))
    return entries


setup(name=name, version='0.1.0', packages=[name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                  ('share/' + name, ['package.xml']),
                  ('share/' + name + '/launch', glob('launch/*.launch.py')),
                  ('share/' + name + '/config', glob('config/*'))]
      + tree('tracker', 'share/' + name + '/tracker'),
      install_requires=['setuptools'], zip_safe=False,
      entry_points={'console_scripts': [
          'a8_tracker = as2_a8_tracker.tracker_process:main',
          'a8_tracker_bridge = as2_a8_tracker.bridge_node:main',
          'a8_sim_vision = as2_a8_tracker.sim_vision_node:main']})
