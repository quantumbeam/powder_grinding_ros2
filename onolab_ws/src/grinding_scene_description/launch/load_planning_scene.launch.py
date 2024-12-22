
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
        
    config = PathJoinSubstitution(
    [FindPackageShare("grinding_scene_description"),'config','planning_scene_config.yaml']
    )
    
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
                parameters=[config]
            )
        ],
        output='screen'
    )
        
    return LaunchDescription([
        load_scene_container,
    ])