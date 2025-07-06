// grinding_scene_description.cpp
#include "grinding_scene_description/load_planning_scene.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <geometric_shapes/mesh_operations.h> 
#include <geometric_shapes/shape_operations.h>

namespace grinding_scene_description
{
const rclcpp::Logger PlanningSceneLoader::LOGGER = rclcpp::get_logger("grinding_scene_description::PlanningSceneLoader");


PlanningSceneLoader::PlanningSceneLoader(const rclcpp::NodeOptions & options)
: rclcpp::Node("load_planning_scene", options)
{
    // Declare parameters
  this->declare_parameter<std::string>("move_group_name", "default_group");
  this->declare_parameter<std::vector<double>>("table_position", {0.0, 0.0, 0.0});
  this->declare_parameter<std::vector<double>>("table_scale", {1.0, 1.0, 1.0});
  this->declare_parameter<std::vector<double>>("mortar_top_position", {0.0, 0.0, 0.0});
  this->declare_parameter<std::vector<double>>("mortar_inner_scale", {1.0, 1.0, 1.0});
  this->declare_parameter<std::string>("mortar_mesh_file_name", "asone_agate_mortar_80x100x36.stl");

  // Get parameters
  std::string move_group_name;
  std::vector<double> table_position;
  std::vector<double> table_scale;
  std::vector<double> mortar_top_position;
  std::vector<double> mortar_inner_scale;
  std::string mortar_mesh_file_name;

  this->get_parameter("move_group_name", move_group_name);
  this->get_parameter("table_position", table_position);
  this->get_parameter("table_scale", table_scale);
  
  this->get_parameter("mortar_top_position", mortar_top_position);
  this->get_parameter("mortar_inner_scale", mortar_inner_scale);
  this->get_parameter("mortar_mesh_file_name", mortar_mesh_file_name);

  RCLCPP_INFO(this->get_logger(), "Move Group Name: %s", move_group_name.c_str());
  RCLCPP_INFO(this->get_logger(), "Table Position: [%f, %f, %f]", table_position[0], table_position[1], table_position[2]);
  RCLCPP_INFO(this->get_logger(), "Table Scale: [%f, %f, %f]", table_scale[0], table_scale[1], table_scale[2]);

  RCLCPP_INFO(this->get_logger(), "Mortar Top Position: [%f, %f, %f]", mortar_top_position[0], mortar_top_position[1], mortar_top_position[2]);
  RCLCPP_INFO(this->get_logger(), "Mortar Inner Scale: [%f, %f, %f]", mortar_inner_scale[0], mortar_inner_scale[1], mortar_inner_scale[2]);
  RCLCPP_INFO(this->get_logger(), "Mortar Mesh File Name: %s", mortar_mesh_file_name.c_str());

  planning_scene_diff_client_ = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client_->wait_for_service();
  
  // 初期化時に既存のオブジェクトを削除
  clear_all_objects();
  
  load_scene();
}

void PlanningSceneLoader::clear_all_objects()
{
  // 既存のオブジェクトを削除
  moveit_msgs::msg::CollisionObject remove_object;
  remove_object.header.frame_id = "base_link";
  remove_object.id = "table";
  remove_object.operation = remove_object.REMOVE;

  moveit_msgs::msg::CollisionObject remove_mortar;
  remove_mortar.header.frame_id = "base_link";
  remove_mortar.id = "mortar";
  remove_mortar.operation = remove_mortar.REMOVE;

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene.world.collision_objects.push_back(remove_mortar);
  planning_scene.is_diff = true;

  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene = planning_scene;

  auto response_future = planning_scene_diff_client_->async_send_request(request);
  std::chrono::seconds wait_time(5);
  if (response_future.wait_for(wait_time) == std::future_status::timeout)
  {
    RCLCPP_WARN(LOGGER, "Service timed out when clearing objects.");
  }
  else
  {
    auto planning_response = response_future.get();
    if (planning_response->success)
    {
      RCLCPP_INFO(LOGGER, "Successfully cleared existing objects.");
    }
    else
    {
      RCLCPP_WARN(LOGGER, "Failed to clear existing objects (they may not exist).");
    }
  }
}

void PlanningSceneLoader::load_scene()
{
  _add_table(this->get_parameter("table_scale").as_double_array(), this->get_parameter("table_position").as_double_array());
  // Use box instead of mesh to avoid timeout issues
  // _add_mortar_box(this->get_parameter("mortar_top_position").as_double_array(), this->get_parameter("mortar_inner_scale").as_double_array());
}


void PlanningSceneLoader::_add_table(const std::vector<double>& table_scale, const std::vector<double>& table_pos)
{
 // Add table
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = "base_link";
  table.id = "table";

  shape_msgs::msg::SolidPrimitive table_primitive;
  table_primitive.type = table_primitive.BOX;
  table_primitive.dimensions.resize(3);
  table_primitive.dimensions[0] = table_scale[0];
  table_primitive.dimensions[1] = table_scale[1];
  table_primitive.dimensions[2] = table_scale[2];

  geometry_msgs::msg::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x = table_pos[0];
  table_pose.position.y = table_pos[1];
  table_pose.position.z = table_pos[2] - table_scale[2] / 2.0 - 0.001;

  table.primitives.push_back(table_primitive);
  table.primitive_poses.push_back(table_pose);
  table.operation = table.ADD;

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(table);
  planning_scene.is_diff = true;

  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene = planning_scene;

  auto response_future = planning_scene_diff_client_->async_send_request(request);
  std::chrono::seconds wait_time(5);
  if (response_future.wait_for(wait_time) == std::future_status::timeout)
  {
    RCLCPP_ERROR(LOGGER, "Service timed out.");
  }
  else
  {
    auto planning_response = response_future.get();
    if (planning_response->success)
    {
      RCLCPP_INFO(LOGGER, "Service successfully added table.");
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Service failed to add table.");
    }
  }
}

void PlanningSceneLoader::_add_mortar(const std::string& file_path, const std::vector<double>& mortar_pos)
{
  // Add mortar
  moveit_msgs::msg::CollisionObject mortar;
  mortar.header.frame_id = "base_link";
  mortar.id = "mortar";

  shape_msgs::msg::Mesh mortar_mesh;
  shapes::Mesh* m = shapes::createMeshFromResource(file_path);
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  mortar_mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  geometry_msgs::msg::Pose mortar_pose;
  mortar_pose.position.x = mortar_pos[0];
  mortar_pose.position.y = mortar_pos[1];
  mortar_pose.position.z = mortar_pos[2];

  mortar.meshes.push_back(mortar_mesh);
  mortar.mesh_poses.push_back(mortar_pose);
  mortar.operation = mortar.ADD;

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(mortar);
  planning_scene.is_diff = true;

  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene = planning_scene;

  auto response_future = planning_scene_diff_client_->async_send_request(request);
  std::chrono::seconds wait_time(5);
  if (response_future.wait_for(wait_time) == std::future_status::timeout)
  {
    RCLCPP_ERROR(LOGGER, "Service timed out.");
  }
  else
  {
    auto planning_response = response_future.get();
    if (planning_response->success)
    {
      RCLCPP_INFO(LOGGER, "Service successfully added mortar.");
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Service failed to add mortar.");
    }
  }
}

void PlanningSceneLoader::_add_mortar_box(const std::vector<double>& mortar_pos, const std::vector<double>& mortar_scale)
{
  // Add mortar as a box instead of mesh to avoid timeout issues
  moveit_msgs::msg::CollisionObject mortar;
  mortar.header.frame_id = "base_link";
  mortar.id = "mortar";

  shape_msgs::msg::SolidPrimitive mortar_primitive;
  mortar_primitive.type = mortar_primitive.BOX;
  mortar_primitive.dimensions.resize(3);
  mortar_primitive.dimensions[0] = mortar_scale[0];
  mortar_primitive.dimensions[1] = mortar_scale[1];
  mortar_primitive.dimensions[2] = mortar_scale[2];

  geometry_msgs::msg::Pose mortar_pose;
  mortar_pose.orientation.w = 1.0;
  mortar_pose.position.x = mortar_pos[0];
  mortar_pose.position.y = mortar_pos[1];
  mortar_pose.position.z = mortar_pos[2];

  mortar.primitives.push_back(mortar_primitive);
  mortar.primitive_poses.push_back(mortar_pose);
  mortar.operation = mortar.ADD;

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(mortar);
  planning_scene.is_diff = true;

  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene = planning_scene;

  auto response_future = planning_scene_diff_client_->async_send_request(request);
  std::chrono::seconds wait_time(5);
  if (response_future.wait_for(wait_time) == std::future_status::timeout)
  {
    RCLCPP_ERROR(LOGGER, "Service timed out adding mortar box.");
  }
  else
  {
    auto planning_response = response_future.get();
    if (planning_response->success)
    {
      RCLCPP_INFO(LOGGER, "Service successfully added mortar box.");
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Service failed to add mortar box.");
    }
  }
}

// デストラクタ
PlanningSceneLoader::~PlanningSceneLoader() {}

}  // namespace grinding_scene_description

// クラスをコンポーネントとして登録
RCLCPP_COMPONENTS_REGISTER_NODE(grinding_scene_description::PlanningSceneLoader)
