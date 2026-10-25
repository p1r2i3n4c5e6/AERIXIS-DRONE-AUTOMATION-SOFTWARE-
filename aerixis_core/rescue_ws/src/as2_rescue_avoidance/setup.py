from setuptools import setup

name = 'as2_rescue_avoidance'

setup(name=name, version='0.1.0', packages=[name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                  ('share/' + name, ['package.xml']),
                  # The trained avoidance brain (stage 2); flown onboard in stage 3.
                  ('share/' + name + '/models', ['models/brain.npz'])],
      install_requires=['setuptools'], zip_safe=False,
      entry_points={'console_scripts': [
          'avoidance_eval = as2_rescue_avoidance.evaluate:main']})
