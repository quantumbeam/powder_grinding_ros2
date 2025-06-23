from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    planning_scene_config_arg = DeclareLaunchArgument(
        'planning_scene_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('grinding_motion_routines'),
            'config', 'ur3e_grinding_demo', 'planning_scene_config.yaml'
        ]),
        description='Path to planning scene config file'
    )
    
    motion_config_arg = DeclareLaunchArgument(
        'motion_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('grinding_motion_routines'),
            'config', 'ur3e_grinding_demo', 'motion_config.yaml'
        ]),
        description='Path to motion config file'
    )

    abs_force_arg = DeclareLaunchArgument(
        'abs_force',
        default_value='false',
        description='Enable absolute force mode'
    )

    rosbag_record_arg = DeclareLaunchArgument(
        'rosbag_record',
        default_value='false',
        description='Enable rosbag recording'
    )

    rosbag_name_arg = DeclareLaunchArgument(
        'rosbag_name',
        default_value=PathJoinSubstitution([
            FindPackageShare('grinding_motion_routines'),
            'bag', ''
        ]),
        description='Rosbag output path'
    )

    # Force/Torque filter node
    ur_ft_filter_node = Node(
        package='grinding_force_torque',
        executable='wrench_filter.py',
        name='ur_ft_filter',
        output='screen',
        arguments=['-z', '-t', 'wrench']
    )

    # Main grinding demo node
    ur3e_grinding_demo_node = Node(
        package='grinding_motion_routines',
        executable='grinding_demo.py',
        name='mechano_grinding',
        output='screen',
        parameters=[
            LaunchConfiguration('planning_scene_config'),
            LaunchConfiguration('motion_config')
        ]
    )

    # Rosbag recording (conditional)
    rosbag_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('rosbag_record')),
        actions=[
            Node(
                package='rosbag2',
                executable='record',
                name='rosbag_record',
                arguments=[
                    'record', '-a', 
                    '-o', LaunchConfiguration('rosbag_name'),
                    '-x', '/camera/depth/(.*)|/camera/color/image_raw/(.*)'
                ]
            )
        ]
    )

    return LaunchDescription([
        planning_scene_config_arg,
        motion_config_arg,
        abs_force_arg,
        rosbag_record_arg,
        rosbag_name_arg,
        ur_ft_filter_node,
        ur3e_grinding_demo_node,
        rosbag_group
    ])