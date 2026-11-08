# rqt_joint_trajectory_controller.launch.py
#
# Start rqt_joint_trajectory_controller to jog the robot joint by joint through
# the joint trajectory controller (e.g. to bring a twisted wrist back near the
# initial pose before running the grinding demo).

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rqt_joint_trajectory_controller',
            executable='rqt_joint_trajectory_controller',
            name='rqt_joint_trajectory_controller',
            output='screen',
        ),
    ])
