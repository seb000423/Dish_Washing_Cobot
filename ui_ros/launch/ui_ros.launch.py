#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output = LaunchConfiguration("output")

    return LaunchDescription([
        DeclareLaunchArgument(
            "output",
            default_value="screen",
            description="ROS2 node output destination",
        ),

        Node(
            package="ui_ros",
            executable="gripper_safety",
            name="gripper_safety",
            output=output,
            emulate_tty=True,
        ),

        Node(
            package="ui_ros",
            executable="force_gripper_monitor",
            name="force_gripper_monitor",
            output=output,
            emulate_tty=True,
        ),

        Node(
            package="ui_ros",
            executable="home_gripper_control",
            name="home_gripper_control",
            output=output,
            emulate_tty=True,
        ),
    ])
