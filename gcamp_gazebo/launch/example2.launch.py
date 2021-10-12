#! /usr/bin/env python3

# 모듈 불러오기
from launch import LaunchDescription
from launch_ros.actions import Node

# turtlesim_node Node
turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        parameters=[],
        arguments=[],
        output="screen",
)

# draw_square_node Node
draw_square_node = Node(
        package="turtlesim",
        executable="draw_square",
        parameters=[],
        arguments=[],
        output="screen",
)

def generate_launch_description():
        return LaunchDescription([
                turtlesim_node,
                draw_square_node,
        ])
