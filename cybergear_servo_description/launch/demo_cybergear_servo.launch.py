#!/usr/bin/env -S python3

# MIT License
#
# Copyright (c) 2024 Naoki Takahashi
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros


def generate_launch_description():
    return launch.LaunchDescription(
        generate_declare_launch_arguments()
        + generate_launch_nodes()
    )


def generate_declare_launch_arguments():
    this_pkg_share_dir = get_package_share_directory('cybergear_servo_description')
    return [
        launch.actions.DeclareLaunchArgument(
            'robot_model_path',
            default_value=[os.path.join(this_pkg_share_dir, 'urdf')],
        ),
        launch.actions.DeclareLaunchArgument(
            'robot_model_file',
            default_value=['cybergear_servo.urdf.xacro'],
        ),
        launch.actions.DeclareLaunchArgument(
            'use_joint_state_publisher_gui',
            default_value=['false'],
        ),
        launch.actions.DeclareLaunchArgument(
            'use_rviz',
            default_value=['true'],
        ),
        launch.actions.DeclareLaunchArgument(
            'rviz_config_file',
            default_value=[os.path.join(this_pkg_share_dir, 'rviz', 'demo.rviz')],
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('use_rviz')
            )
        ),
    ]


def generate_launch_nodes():
    output = 'screen'
    urdf_file = launch.substitutions.PathJoinSubstitution([
        launch.substitutions.LaunchConfiguration('robot_model_path'),
        launch.substitutions.LaunchConfiguration('robot_model_file')
    ])
    robot_description = {
        'robot_description': launch.substitutions.Command([
            'xacro',
            ' ',
            urdf_file
        ])
    }
    return [
        launch.actions.GroupAction(actions=[
            launch_ros.actions.Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='cybergear_servo_state_publisher',
                output=output,
                parameters=[robot_description],
            ),
            launch_ros.actions.Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='cybergear_servo_joint_state_publisher',
                output=output,
                condition=launch.conditions.UnlessCondition(
                    launch.substitutions.LaunchConfiguration('use_joint_state_publisher_gui')
                )
            ),
            launch_ros.actions.Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='cybergear_servo_joint_state_publisher_gui',
                output=output,
                condition=launch.conditions.IfCondition(
                    launch.substitutions.LaunchConfiguration('use_joint_state_publisher_gui')
                )
            ),
            launch_ros.actions.Node(
                package='rviz2',
                executable='rviz2',
                name='cybergear_servo_rviz2',
                output=output,
                arguments=[
                    '-d', launch.substitutions.LaunchConfiguration('rviz_config_file')
                ],
                condition=launch.conditions.IfCondition(
                    launch.substitutions.LaunchConfiguration('use_rviz')
                )
            )
        ])
    ]
