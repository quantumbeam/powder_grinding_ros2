from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='grinding_force_torque',
            executable='wrench_statistics.py',
            name='wrench_statistics',
            output='screen',
            parameters=[
                {'wrench_topic': '/wrench_filtered'},
                {'wrench_statistics_service': '/wrench_statistics'}
            ]
        )
    ])
