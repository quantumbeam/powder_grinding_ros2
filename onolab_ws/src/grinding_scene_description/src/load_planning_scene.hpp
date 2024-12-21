#ifndef PLANNING_SCENE_LOADER_HPP
#define PLANNING_SCENE_LOADER_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

/**
 * @brief A class to handle loading planning scenes in ROS2.
 */
class PlanningSceneLoader : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for PlanningSceneLoader.
   * @param name_space The namespace for the node.
   * @param options Options for the rclcpp::Node.
   */
  PlanningSceneLoader(const std::string& name_space="", 
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
};
#endif // PLANNING_SCENE_LOADER_HPP
