# Copyright 2026 Maik Knof
# Maintainer: Maik Knof <maik.knof@gmx.de>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Load only the snapshots whose topic arguments are set."""

import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def _load_components(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    parameters = {
        'input.reliability': LaunchConfiguration('reliability').perform(context),
        'cache.history_size': int(LaunchConfiguration('history_size').perform(context)),
        'cache.max_age_seconds': float(LaunchConfiguration('max_age_seconds').perform(context)),
        'service.qos_depth': int(LaunchConfiguration('service_qos_depth').perform(context)),
    }
    components = []
    topics = []
    for argument, plugin in (
        ('image_topic', 'sensor_snapshot::ImageSnapshot'),
        ('cloud_topic', 'sensor_snapshot::PointCloudSnapshot'),
        ('imu_topic', 'sensor_snapshot::ImuSnapshot'),
    ):
        topic = LaunchConfiguration(argument).perform(context).strip()
        if not topic:
            continue
        topics.append(topic)
        components.append(ComposableNode(
            package='sensor_snapshot',
            plugin=plugin,
            # Let the component derive its snapshot_<topic> name from input.topic.
            namespace=namespace,
            parameters=[{**parameters, 'input.topic': topic}],
            extra_arguments=[{'use_intra_process_comms': True}],
        ))

    if not components:
        return []

    container = LaunchConfiguration('container').perform(context)
    if container:
        # This is the preferred deployment: the sensor publisher shares this process.
        return [LoadComposableNodes(
            target_container=container, composable_node_descriptions=components)]
    # Separate launch invocations also need distinct container node names.
    topic_name = re.sub(r'[^A-Za-z0-9_]', '_', topics[0].lstrip('/'))
    return [ComposableNodeContainer(
        name=f'snapshot_{topic_name}_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=components,
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('container', default_value='', description=(
            'Existing multithreaded sensor container; empty starts a new container when needed')),
        DeclareLaunchArgument('namespace', default_value='', description='Component namespace'),
        DeclareLaunchArgument('image_topic', default_value='', description=(
            'Image input topic; empty disables the image component')),
        DeclareLaunchArgument('cloud_topic', default_value='', description=(
            'Cloud input topic; empty disables the cloud component')),
        DeclareLaunchArgument('imu_topic', default_value='', description=(
            'IMU input topic; empty disables the IMU component')),
        DeclareLaunchArgument('reliability', default_value='best_effort', description=(
            'Input reliability: best_effort or reliable')),
        DeclareLaunchArgument('history_size', default_value='1', description=(
            'Number of received messages retained per component')),
        DeclareLaunchArgument('max_age_seconds', default_value='0.0', description=(
            'Default maximum reception age; zero disables expiry')),
        DeclareLaunchArgument('service_qos_depth', default_value='1', description=(
            'Service request/response history depth')),
        OpaqueFunction(function=_load_components),
    ])
