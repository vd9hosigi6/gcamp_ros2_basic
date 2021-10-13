#!/usr/bin/env python3

# 모듈 불러오기
import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

        # turtlesim_node Node
        turtlesim_node = Node(
                package="turtlesim",
                executable="turtlesim_node",
                parameters=[],
                arguments=[],
                output="screen",
        )

        # turtle_teleop_key_node
        turtle_teleop_key_node = Node(
                package="turtlesim",
                executable="turtle_teleop_key",
                parameters=[],
                arguments=[],
                prefix=["xterm -e"],
                output="screen",
        )

        return LaunchDescription([
                turtlesim_node,
                turtle_teleop_key_node,
        ])


