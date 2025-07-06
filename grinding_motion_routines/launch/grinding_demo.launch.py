import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share_motion = get_package_share_directory('grinding_motion_routines')
    motion_config = os.path.join(pkg_share_motion, 'config', 'motion_config.yaml')
    pkg_share_scene = get_package_share_directory('grinding_scene_description')
    planning_scene_config = os.path.join(pkg_share_scene, 'config', 'planning_scene_config.yaml')

    grinding_demo_node = Node(
        package='grinding_motion_routines',
        executable='grinding_demo.py',
        name='grinding_demo_node', 
        output='screen',
        prefix='terminator -x',
        parameters=[motion_config,planning_scene_config],
    )

    return LaunchDescription([
        grinding_demo_node
    ])
