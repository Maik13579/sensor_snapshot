# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('sensor_snapshot_recorder'), 'config', 'recorder.yaml'])),
        ComposableNodeContainer(
            name='snapshot_recorder_container', namespace='', package='rclcpp_components',
            executable='component_container_mt', output='screen',
            composable_node_descriptions=[ComposableNode(
                package='sensor_snapshot_recorder',
                plugin='sensor_snapshot_recorder::SnapshotRecorder',
                parameters=[LaunchConfiguration('params_file')])]),
    ])
