from glob import glob
from setuptools import setup

package_name = 'as2_mavlink_gateway'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aerostack2 contributors',
    maintainer_email='maintainers@example.com',
    description='Routes standard MAVLink GCS missions and controls through Aerostack2.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'mavlink_gateway = as2_mavlink_gateway.gateway_node:main',
        ],
    },
)
