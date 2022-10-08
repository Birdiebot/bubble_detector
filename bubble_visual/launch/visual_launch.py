'''
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: Ligcox
Date: 2022-05-14 20:44:31
FilePath: /bubble_bringup/home/nvidia/Desktop/bubble/src/bubble_contrib/bubble_visual/launch/visual_launch.py
LastEditors: Ligcox
LastEditTime: 2022-05-16 21:41:34
E-mail: robomaster@birdiebot.top
'''

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

config = os.path.join(
    get_package_share_directory('bubble_visual'),
    'config',
    'config.yaml'
)


def generate_launch_description():
    if (os.getenv('ROS_DISTRO') == "dashing") or (os.getenv('ROS_DISTRO') == "eloquent"):
        return LaunchDescription([
            Node(
                package='bubble_visual_sjtu',
                node_name='Visual',
                node_executable='Visual',
                parameters=[config],
                output='screen',
            )
        ])
    else:
        return LaunchDescription([
            Node(
                package='bubble_visual_sjtu',
                name='Visual',
                executable='Visual',
                emulate_tty=True,
                parameters=[config],
                output='screen',
            )
        ])
