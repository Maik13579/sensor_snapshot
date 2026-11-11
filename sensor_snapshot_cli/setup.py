# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
import shlex
import subprocess

from setuptools import Extension, find_packages, setup

package_name = 'sensor_snapshot_cli'

# pkg-config supplies the versioned PCL/Eigen include directories on each LTS.
# Link just the I/O and common modules; runtime dependencies are in package.xml.
pcl_cflags = shlex.split(subprocess.check_output(
    ['pkg-config', '--cflags', 'pcl_io'], text=True))
pcl_library_dirs = shlex.split(subprocess.check_output(
    ['pkg-config', '--libs-only-L', 'pcl_io'], text=True))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    ext_modules=[Extension(
        package_name + '._pcl_io', sources=['src/pcl_io.cpp'], language='c++',
        extra_compile_args=pcl_cflags + ['-std=c++17'],
        extra_link_args=pcl_library_dirs, libraries=['pcl_io', 'pcl_common'],
    )],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Maik Knof',
    maintainer_email='maik.knof@gmx.de',
    description='Discover, inspect, export and preview sensor snapshots from ros2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'sensor_snapshot = sensor_snapshot_cli.command:SensorSnapshotCommand',
        ],
    },
)
