
import os
from launch import LaunchDescription
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        
    # planning_scene_config = PathJoinSubstitution(
    # [FindPackageShare("grinding_scene_description"),"config","planning_scene_config.yaml"]
    # )
    planning_scene_config = os.path.join(get_package_share_directory('grinding_scene_description'), 'config', 'planning_scene_config.yaml')
    print(planning_scene_config)
    
    load_scene_container=ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='load_scene_container',
        namespace='',
        composable_node_descriptions=[
            ComposableNode(
                package='grinding_scene_description',
                plugin='grinding_scene_description::PlanningSceneLoader',
                name='scene_load_node',
                parameters=[planning_scene_config]
            )
        ],
        output='screen'
    )
    load_scene_node=Node(
        package='grinding_scene_description',
        executable='load_planning_scene_node',
        name='planning_scene_loader',
        parameters=[planning_scene_config],
        output='screen',
    )
        
    return LaunchDescription([
        # load_scene_container,
        load_scene_node
    ])