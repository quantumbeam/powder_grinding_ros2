#include <rclcpp/rclcpp.hpp>
#include "load_planning_scene.hpp"

PlanningSceneLoader::PlanningSceneLoader(const std::string& name_space, const rclcpp::NodeOptions& options)
    : Node("planning_scene_loader", name_space, options)
{
  RCLCPP_INFO(this->get_logger(),"minimal_node_test");
}
