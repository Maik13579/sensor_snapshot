# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
from setuptools import find_packages, setup

package_name = 'sensor_snapshot_cli'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Maik Knof',
    maintainer_email='maik.knof@gmx.de',
    description='Discover, inspect and preview sensor snapshots from ros2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'sensor_snapshot = sensor_snapshot_cli.command:SensorSnapshotCommand',
        ],
    },
)
