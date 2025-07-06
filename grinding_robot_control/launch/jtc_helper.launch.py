#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
 
    
    jtc_helper_node = Node(
        package='grinding_robot_control',
        executable='JTC_helper.py',
        name='jtc_helper_node',
        output='screen',
        prefix="xterm -e",
    )

    return LaunchDescription([
        jtc_helper_node
    ])