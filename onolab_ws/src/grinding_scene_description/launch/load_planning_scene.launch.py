
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return launch.LaunchDescription([
        ComposableNodeContainer(
            package='rclcpp_components',
            executable='component_container',
            name='load_scene_container',
            namespace='',
            composable_node_descriptions=[
                ComposableNode(
                    package='grinding_scene_description',
                    plugin='grinding_scene_description::PlanningSceneLoader',
                    name='scene_load_node'
                )
            ],
            output='screen'
        )
    ])