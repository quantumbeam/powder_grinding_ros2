# ur5e_with_pestle_bringup.launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition # Potentially needed if 'use_moveit' controls inclusion
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # --- Get Launch Configuration values ---
    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    use_moveit = LaunchConfiguration('use_moveit') # Evaluate the condition
    launch_moveit_rviz = LaunchConfiguration('launch_moveit_rviz') # Unified RViz argument
    planning_scene_config = LaunchConfiguration('planning_scene_config')
    initial_joint_controller = LaunchConfiguration('initial_joint_controller')

    # --- Get Package Share Directories ---
    grinding_robot_bringup_share = get_package_share_directory('grinding_robot_bringup')
    grinding_moveit_config_share = get_package_share_directory('grinding_moveit_config')
    grinding_scene_description_share = get_package_share_directory('grinding_scene_description')

    # --- Include ur_control launch file ---
    # This provides the basic robot driver and controllers.
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([grinding_robot_bringup_share, 'launch', 'include', 'ur_control.launch.py'])
        ),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'use_fake_hardware': use_mock_hardware,
            'fake_sensor_commands': mock_sensor_commands,
            # Always set launch_rviz to 'false' for ur_control,
            # as we want to use the MoveIt RViz configuration if RViz is launched at all.
            'launch_rviz': 'false',
            'initial_joint_controller': initial_joint_controller,
        }.items()
    )

    # --- Include ur_moveit launch file ---
    # This sets up the MoveIt planning capabilities.
    # We include it based on the 'use_moveit' argument.
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([grinding_moveit_config_share, 'launch', 'ur_moveit.launch.py'])
        ),
        launch_arguments={
            'ur_type': ur_type,
            # Pass the unified 'launch_rviz' argument.
            # ur_moveit.launch.py will handle launching RViz with its config if this is 'true'.
            'launch_rviz': launch_moveit_rviz,
        }.items(),
        # Only include this if 'use_moveit' is true
        condition=IfCondition(use_moveit)
    )

    # --- Include load_planning_scene launch file (XML) ---
    # This likely publishes the static scene elements (mortar, etc.) to the planning scene.
    load_planning_scene_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource( # Use AnyLaunchDescriptionSource for XML files
            PathJoinSubstitution([grinding_scene_description_share, 'launch', 'load_planning_scene.launch.xml'])
        ),
        launch_arguments={
            'planning_scene_config': planning_scene_config,
        }.items(),
        # Typically, the planning scene is only relevant when using MoveIt.
        condition=IfCondition(use_moveit)
    )



    # --- Launch ft_filter node ---
    ft_filter_node = Node(
        package='grinding_ft_filter',
        executable='ft_filter.py',
        name='ft_filter',
        output='screen',
        parameters=[
            {'input_topic': '/wrench_raw'},
            {'output_topic': '/wrench_filtered'},
            {'sampling_frequency': 500.0},
            {'cutoff_frequency': 2.5},
            {'filter_order': 3},
            {'data_window': 100},
            {'initial_zero': True},
            {'disable_filtering': False}
        ]
    )

    # --- List of actions to execute ---
    actions_to_start = [
        ur_control_launch,
        ur_moveit_launch,
        load_planning_scene_launch,
        ft_filter_node
    ]

    return actions_to_start

def generate_launch_description():
    declared_arguments = []

    # --- Declare Launch Arguments ---
    declared_arguments.append(DeclareLaunchArgument(
        'ur_type',
        default_value='ur5e',
        description='Type of UR robot (e.g., ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20).'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.58.42',
        description='IP address of the robot controller.'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='true',
        description='Use mock hardware interface for testing without a physical robot.'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'mock_sensor_commands',
        default_value='true',
        description='Enable mock sensor commands for testing.'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'use_moveit',
        default_value='true',
        description='Whether to launch MoveIt related nodes and load the planning scene.'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'initial_joint_controller',
        default_value='scaled_joint_trajectory_controller',
        description='Initial robot controller.'
    ))
    # --- Unified RViz Argument ---
    declared_arguments.append(DeclareLaunchArgument(
        'launch_moveit_rviz',
        default_value='true', # Default to launching RViz (with MoveIt config if use_moveit is true)
        description='Whether to launch RViz. If use_moveit is true, uses MoveIt configuration.'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'planning_scene_config',
        # Use PathJoinSubstitution to construct the default path dynamically
        default_value=PathJoinSubstitution([
            get_package_share_directory('grinding_scene_description'),
            'config',
            'planning_scene_config.yaml'
        ]),
        description='Path to the planning scene configuration file (used if use_moveit is true).'
    ))

    # --- Create Launch Description ---
    # Combine declared arguments with the OpaqueFunction that executes the launch setup logic
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

