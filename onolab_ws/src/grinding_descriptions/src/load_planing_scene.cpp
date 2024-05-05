#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>


// class PlanningScene
// {
// public:
//   PlanningScene(const std::shared_ptr<rclcpp::Node> &node, const std::string &move_group_name)
//       : node_(node)
//   {
//     move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, move_group_name);
//     planning_frame_ = move_group_->getPlanningFrame();
//     planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>(node);

//     // node -> declare_parameter("table_position", std::vector<double>(3, 0));
//     // node -> declare_parameter("table_scale", std::vector<double>(3, 0));
//     // node -> declare_parameter("mortar_top_position", std::vector<double>(3, 0));
//     // node -> declare_parameter("mortar_inner_scale", std::vector<double>(3, 0));

//     table_pos = node->get_parameter("table_position").as_double_array();
//     table_scale = node->get_parameter("table_scale").as_double_array();
//     mortar_top_pos = node->get_parameter("mortar_top_position").as_double_array();
//     mortar_inner_scale = node->get_parameter("mortar_inner_scale").as_double_array();
//   }

//   void initPlanningScene()
//   {
//     std::string mortar_mesh_file_path = "package://grinding_descriptions/mesh/moveit_scene_object/mortar_40mm.stl";
//     rclcpp::Parameter param;

//     addTable(table_pos, table_scale);
//     // addMortar(mortar_mesh_file_path, mortar_pos);
  
//   }

//   void addTable(std::vector<double> &table_pos, std::vector<double> &table_scale)
//     {
//       moveit_msgs::msg::CollisionObject table;
//       table.id = "Table";
//       table.header.frame_id = planning_frame_;

//       shape_msgs::msg::SolidPrimitive primitive;
//       primitive.type = primitive.BOX;
//       primitive.dimensions.resize(3);
//       std::cout << "table_scale[0]: " << table_scale[0] << std::endl;
//       primitive.dimensions[primitive.BOX_X] = table_scale[0];
//       primitive.dimensions[primitive.BOX_Y] = table_scale[1];
//       primitive.dimensions[primitive.BOX_Z] = table_scale[2];
//       geometry_msgs::msg::Pose box_pose;
//       box_pose.orientation.w = 1.0;
//       box_pose.position.x = table_pos[0];
//       box_pose.position.y = table_pos[1];
//       box_pose.position.z = table_pos[2];
//       box_pose.position.z -= table_scale[2] / 2.0;

//       table.primitives.push_back(primitive);
//       table.primitive_poses.push_back(box_pose);
//       table.operation = table.ADD;

//       // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//       planning_scene_interface_->applyCollisionObject(table);
//     }

// private:
//   std::shared_ptr<rclcpp::Node> node_;
//   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
//   std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
//   std::string planning_frame_;
//   std::vector<double> table_pos;
//   std::vector<double> table_scale;
//   std::vector<double> mortar_top_pos;
//   std::vector<double> mortar_inner_scale;
//   std::string mortar_mesh_file_path;

// };

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<TestNode> node = nullptr;
  node = std::make_shared<LoadingPlanningScene>("test_topic", "test_node", node_options);

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  planning_frame = move_group.getPlanningFrame();
  table_pos = node->get_parameter("table_position");
  table_scale = node->get_parameter("table_scale");
  mortar_top_pos = node->get_parameter("mortar_top_position");
  mortar_inner_scale = node->get_parameter("mortar_inner_scale");

  moveit_msgs::msg::CollisionObject table;
  table.id = "Table";
  table.header.frame_id = planning_frame;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  std::cout << "table_scale[0]: " << table_scale[0] << std::endl;
  primitive.dimensions[primitive.BOX_X] = table_scale[0];
  primitive.dimensions[primitive.BOX_Y] = table_scale[1];
  primitive.dimensions[primitive.BOX_Z] = table_scale[2];
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = table_pos[0];
  box_pose.position.y = table_pos[1];
  box_pose.position.z = table_pos[2];
  box_pose.position.z -= table_scale[2] / 2.0;

  table.primitives.push_back(primitive);
  table.primitive_poses.push_back(box_pose);
  table.operation = table.ADD;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(table);


  // PlanningScene planning_scene(node, "ur_manipulator");
  // planning_scene.initPlanningScene();

  // // Shutdown ROS
  rclcpp::shutdown();

  return 0;
}
