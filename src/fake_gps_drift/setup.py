#!/usr/bin/env python3
from setuptools import setup, find_packages

package_name = 'fake_gps_drift'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/fake_gps_drift']),
        ('share/fake_gps_drift', ['package.xml']),
        ('share/fake_gps_drift/launch', ['launch/default.launch.py']),
    ],
    install_requires=[    
        'setuptools',
        'rclpy',
        'geometry_msgs',
    ],
    zip_safe=True,
    maintainer='Eric Price',
    maintainer_email='eric.price@tuebingen.mpg.de',
    description='Simulate GPS drift in ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_gps_drift_node = fake_gps_drift.fake_gps_drift_node:main',
        ],
    },
)