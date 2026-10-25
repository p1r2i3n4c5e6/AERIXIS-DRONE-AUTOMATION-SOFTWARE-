from setuptools import setup

package_name = 'as2_rescue_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AS2 rescue project',
    maintainer_email='maintainer@example.invalid',
    description='Validated KML geometry, coverage partition and per-drone mission bundles',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plan_fleet = as2_rescue_planner.cli:main',
        ],
    },
)
