'''
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: Ligcox
Date: 2022-05-14 20:44:31
FilePath: /bubble/src/bubble_detector/bubble_rune/launch/visual_launch.py
LastEditors: HarryWen
LastEditTime: 2022-07-08 13:38:26
E-mail: robomaster@birdiebot.top
'''

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]


def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('bubble_rune'),
        'config',
        'config.yaml'
    )
    if (os.getenv('ROS_DISTRO') == "dashing") or (os.getenv('ROS_DISTRO') == "eloquent"):
        return LaunchDescription([
            Node(
                package='bubble_rune',
                node_name='Visual',
                node_executable='Visual',
                parameters=[config],
                output='screen',
            ),
        ])
    else:
        return LaunchDescription([
            Node(
                package='bubble_rune',
                name='Visual',
                executable='VisualRune',
                emulate_tty=True,
                parameters=[config],
                output='screen',
            )
        ])
