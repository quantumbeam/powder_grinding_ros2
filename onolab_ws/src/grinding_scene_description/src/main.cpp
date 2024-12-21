
#include <rclcpp/rclcpp.hpp>
#include "load_planning_scene.hpp"

int main(int argc, char **argv)
{
  // ROS2ノードの初期化
  rclcpp::init(argc, argv);

  // PlanningSceneLoaderノードを作成
  auto node = std::make_shared<PlanningSceneLoader>("", rclcpp::NodeOptions());

  // ノードの動作確認ログ
  RCLCPP_INFO(node->get_logger(), "PlanningSceneLoader node has been started.");

  // ノードのスピン
  rclcpp::spin(node);

  // ROS2ノードの終了
  rclcpp::shutdown();

  return 0;
}
