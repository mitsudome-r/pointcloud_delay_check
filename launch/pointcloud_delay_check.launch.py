# Copyright 2020 Tier IV, Inc. All rights reserved.
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

import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.substitutions import EnvironmentVariable


def generate_launch_description():
    components = []

    components.append(ComposableNode(
        package='pointcloud_delay_check',
        plugin='PointCloudPublisher',
        name='pointcloud_publisher',
        namespace='',
        parameters=[{
            'width': 1000,
            'length': 1000,
            'rate': 10.0,
            'use_sim_time': False
        }],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process', default='True')
        }],
    ))

    components.append(ComposableNode(
        package='pointcloud_delay_check',
        plugin='PointCloudDelayChecker',
        name='delay_checker',
        namespace='',
        remappings=[('input_topic', 'pointcloud')],
        parameters=[{
            'use_sim_time': False
        }],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process', default='True')
        }],
    ))

    container = ComposableNodeContainer(
        name='delay_check_container',
        package='rclcpp_components',
        namespace='',
        executable=LaunchConfiguration('container_executable'),
        composable_node_descriptions=components,
        output='screen',
        parameters=[{
            'use_sim_time': False
        }],
    )

    set_container_executable = SetLaunchConfiguration(
        'container_executable',
        'component_container',
        condition=UnlessCondition(LaunchConfiguration('use_multithread'))
    )

    set_container_mt_executable = SetLaunchConfiguration(
        'container_executable',
        'component_container_mt',
        condition=IfCondition(LaunchConfiguration('use_multithread'))
    )

    return launch.LaunchDescription([set_container_executable,
                                     set_container_mt_executable] +
                                    [container])
