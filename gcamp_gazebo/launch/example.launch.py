#! /usr/bin/env python3

import os

# 모듈 불러오기
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# skidbot.rviz 절대경로 얻기
rviz_arg = os.path.join(get_package_share_directory("gcamp_gazebo"), "rviz", "skidbot.rviz")
print("rviz_arg : {}" .format(rviz_arg))

# 함수 정의하기
def generate_launch_description():
        return LaunchDescription([
                ExecuteProcess(
                        cmd=["ros2", "run", "rviz2", "rviz2", "-d", rviz_arg],
                        output="screen"
                ),
        ])
