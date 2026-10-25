from glob import glob
from setuptools import setup

package_name = 'as2_ground_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CVAR-UPM',
    maintainer_email='cvar.upm3@gmail.com',
    description='Browser-based MAVLink ground station for AERIXIS simulation.',
    license='BSD-3-Clause',
    entry_points={'console_scripts': ['ground_control = as2_ground_control.server:main']},
)
