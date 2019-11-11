#!/usr/bin/env python3
#
# Copyright 2019 Matt Richard
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

import os

from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    param_dir = launch.substitutions.LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('johnny5_bringup'),
            'param',
            'johnny5.yaml'))

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='ssc32u_driver', node_executable='ssc32u_node', output='screen', parameters=[param_dir]),

        launch_ros.actions.Node(
            package='ssc32u_controllers', node_executable='servo_controller_node', output='screen',
            parameters=[param_dir], remappings=[('command', 'torso/command'), ('joint_states', 'torso/joint_states')],
            node_name='torso_controller'),

        launch_ros.actions.Node(
            package='ssc32u_controllers', node_executable='servo_controller_node', output='screen',
            parameters=[param_dir], remappings=[('command', 'right_arm/command'), ('joint_states', 'right_arm/joint_states')],
            node_name='right_arm_controller'),

        launch_ros.actions.Node(
            package='ssc32u_controllers', node_executable='servo_controller_node', output='screen',
            parameters=[param_dir], remappings=[('command', 'left_arm/command'), ('joint_states', 'left_arm/joint_states')],
            node_name='left_arm_controller'),

        launch_ros.actions.Node(
            package='ssc32u_controllers', node_executable='sabertooth_2x5_controller_node', output='screen', parameters=[param_dir])
        )
    ])

    # TODO: Figure out why parameters not applying with components
    # container = ComposableNodeContainer(
    #         node_name='johnny5_container',
    #         node_namespace='',
    #         package='rclcpp_components',
    #         node_executable='component_container',
    #         parameters=[param_dir],
    #         composable_node_descriptions=[
    #             ComposableNode(
    #                 package='ssc32u_driver',
    #                 node_plugin='ssc32u_driver::SSC32UDriver',
    #                 node_name='ssc32u',
    #                 parameters=[param_dir]),

    #             ComposableNode(
    #                 package='ssc32u_controllers',
    #                 node_plugin='ssc32u_controllers::ServoController',
    #                 node_name='torso_controller',
    #                 parameters=[param_dir],
    #                 remappings=[('command', 'torso/command'), ('joint_states', 'torso/joint_states')]),

    #             ComposableNode(
    #                 package='ssc32u_controllers',
    #                 node_plugin='ssc32u_controllers::ServoController',
    #                 node_name='right_arm_controller',
    #                 parameters=[param_dir],
    #                 remappings=[('command', 'right_arm/command'), ('joint_states', 'right_arm/joint_states')]),

    #             ComposableNode(
    #                 package='ssc32u_controllers',
    #                 node_plugin='ssc32u_controllers::ServoController',
    #                 node_name='left_arm_controller',
    #                 parameters=[param_dir],
    #                 remappings=[('command', 'left_arm/command'), ('joint_states', 'left_arm/joint_states')])
    #         ],
    #         output='screen',
    # )

    # return launch.LaunchDescription([container])
