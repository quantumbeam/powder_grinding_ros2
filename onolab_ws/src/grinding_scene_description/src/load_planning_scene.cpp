// grinding_scene_description.cpp
#include "grinding_scene_description/load_planning_scene.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace grinding_scene_description
{
const rclcpp::Logger PlanningSceneLoader::LOGGER = rclcpp::get_logger("grinding_scene_description::PlanningSceneLoader");


PlanningSceneLoader::PlanningSceneLoader(const rclcpp::NodeOptions & options)
:rclcpp::Node("load_planning_scene", options)
{
  planning_scene_diff_client_ = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client_->wait_for_service();
  load_scene();
}

void PlanningSceneLoader::load_scene()
{
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "base_link";
  attached_object.object.header.frame_id = "base_link";
  attached_object.object.id = "box";

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = 0.0;
  pose.position.z = 0.2;
  pose.orientation.w = 1.0;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.075;
  primitive.dimensions[1] = 0.075;
  primitive.dimensions[2] = 0.075;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);
  attached_object.object.operation = attached_object.object.ADD;

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;

  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene = planning_scene;

  auto response_future = planning_scene_diff_client_->async_send_request(request).future.share();

  std::chrono::seconds wait_time(1);
  std::future_status fs = response_future.wait_for(wait_time);
  if (fs == std::future_status::timeout)
  {
    RCLCPP_ERROR(LOGGER, "Service timed out.");
  }
  else
  {
    auto planning_response = response_future.get();
    if (planning_response->success)
    {
      RCLCPP_INFO(LOGGER, "Service successfully added object.");
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Service failed to add object.");
    }
  }
}



// デストラクタ
PlanningSceneLoader::~PlanningSceneLoader() {}

}  // namespace grinding_scene_description

// クラスをコンポーネントとして登録
RCLCPP_COMPONENTS_REGISTER_NODE(grinding_scene_description::PlanningSceneLoader)
