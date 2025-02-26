
#include <rclcpp/rclcpp.hpp>
#include <grinding_scene_description/load_planning_scene.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("load_planning_scene");

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<grinding_scene_description::PlanningSceneLoader>(rclcpp::NodeOptions());
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}
  