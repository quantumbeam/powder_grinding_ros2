// grinding_scene_description.hpp
#ifndef GRINDING_SCENE_DESCRIPTION_HPP
#define GRINDING_SCENE_DESCRIPTION_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>


#include "grinding_scene_description/visibility_control.h"


namespace grinding_scene_description
{
class PlanningSceneLoader : public rclcpp::Node
{
public:
  // Macro for multi-platform exporting
  GRINDING_SCENE_DESCRIPTION_PUBLIC
  explicit PlanningSceneLoader(const rclcpp::NodeOptions & options);
  virtual ~PlanningSceneLoader();
  void load_scene();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr planning_scene_diff_client_;
  static const rclcpp::Logger LOGGER;

};

} // namespace grinding_scene_description

#endif // GRINDING_SCENE_DESCRIPTION_HPP
