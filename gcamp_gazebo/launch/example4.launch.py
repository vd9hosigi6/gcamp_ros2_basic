#! /usr/bin/env python3

# 모듈 불러오기
from launch import LaunchDescription
from launch.actions import ExecuteProcess

#
def generate_launch_description():
        return LaunchDescription([
                ExecuteProcess(
                        cmd=["ros2", "run", "turtlesim", "turtlesim_node"],
                        output="screen"
                ),
                ExecuteProcess(
                        cmd=["xterm", "-e", "ros2", "run", "turtlesim", "turtle_teleop_key"],
                        output="screen"
                ),
        ])

