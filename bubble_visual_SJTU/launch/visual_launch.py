'''
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: Ligcox
Date: 2022-05-14 20:44:31
FilePath: /bubble/src/bubble_detector/bubble_visual_SJTU/launch/visual_launch.py
LastEditors: Ligcox
LastEditTime: 2022-07-03 16:38:53
E-mail: robomaster@birdiebot.top
'''

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

arg_list = [
    DeclareLaunchArgument(
        "model_path",
        default_value="/home/nvidia/Desktop/bubble/src/bubble_resources/module/autoaming_SJTU2021_base.onnx",
        description="yolox onnx model path."
    ),

    DeclareLaunchArgument(
        "debug_mode",
        default_value="True",
        description="Enable debug mode"
    )
]


def generate_launch_description():
    if (os.getenv('ROS_DISTRO') == "dashing") or (os.getenv('ROS_DISTRO') == "eloquent"):
        return LaunchDescription(arg_list + [
            Node(
                package='bubble_visual_sjtu',
                node_name='Visual',
                node_executable='Visual',
                parameters=[{
                        "model_path": LaunchConfiguration("model_path"),
                        "debug_mode": LaunchConfiguration("debug_mode"),
                }],
                output='screen',
            )
        ])
    else:
        return LaunchDescription(arg_list + [
            Node(
                package='bubble_visual_sjtu',
                name='Visual',
                executable='Visual',
                emulate_tty=True,
                parameters=[{
                        "model_path": LaunchConfiguration("model_path"),
                        "debug_mode": LaunchConfiguration("debug_mode"),
                }],
                output='screen',
            )
        ])
