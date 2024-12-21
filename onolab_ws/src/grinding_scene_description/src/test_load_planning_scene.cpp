
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

// MoveIt
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("load_planning_scene");

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("load_planning_scene", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]()
              { executor.spin(); })
      .detach();

  // // You can use topic to unsynchronized load of the planning scene (requires enough waiting time to load in the script)
  // rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher =
  //       node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
  // while (planning_scene_diff_publisher->get_subscription_count() < 1)
  // {
  //   rclcpp::sleep_for(std::chrono::seconds(1));
  // }
  // You can use service to synchronized load of the planning scene
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr planning_scene_diff_client =
      node->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client->wait_for_service();

  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "base_link";
  attached_object.object.header.frame_id = "base_link";
  attached_object.object.id = "box";

  /* A default pose */
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
  // planning_scene_diff_publisher->publish(planning_scene);
  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene = planning_scene;
  std::shared_future<std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene_Response>> response_future;
  response_future = planning_scene_diff_client->async_send_request(request).future.share();

  std::chrono::seconds wait_time(1);
  std::future_status fs = response_future.wait_for(wait_time);
  if (fs == std::future_status::timeout)
  {
    RCLCPP_ERROR(LOGGER, "Service timed out.");
  }
  else
  {
    std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene_Response> planning_response;
    planning_response = response_future.get();
    if (planning_response->success)
    {
      RCLCPP_INFO(LOGGER, "Service successfully added object.");
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Service failed to add object.");
    }
  }

  rclcpp::shutdown();
  return 0;
}