#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#if 0
class PlanningScene
{
public:
  PlanningScene(const std::shared_ptr<rclcpp::Node>& node, const std::string& move_group_name)
    : node_(node)
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(&node,&move_group_name);
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    planning_frame_ = move_group_->getPlanningFrame();

  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::string planning_frame_;

//   void initPlanningScene()
//   {
//     std::string mortar_mesh_file_path = "package://grinding_descriptions/mesh/moveit_scene_object/mortar_40mm.stl";

//     rclcpp::Parameter param;
//     node_->get_parameter("table_position", param);
//     geometry_msgs::msg::Point table_pos = param.get_value<geometry_msgs::msg::Point>();
//     node_->get_parameter("table_scale", param);
//     geometry_msgs::msg::Vector3 table_scale = param.get_value<geometry_msgs::msg::Vector3>();
//     node_->get_parameter("mortar_top_position", param);
//     geometry_msgs::msg::Point mortar_pos = param.get_value<geometry_msgs::msg::Point>();
//     node_->get_parameter("mortar_inner_scale", param);
//     geometry_msgs::msg::Vector3 mortar_inner_scale = param.get_value<geometry_msgs::msg::Vector3>();
//     node_->get_parameter("funnel_position", param);
//     geometry_msgs::msg::Point funnel_pos = param.get_value<geometry_msgs::msg::Point>();
//     node_->get_parameter("funnel_scale", param);
//     geometry_msgs::msg::Vector3 funnel_scale = param.get_value<geometry_msgs::msg::Vector3>();
//     node_->get_parameter("MasterSizer_position", param);
//     geometry_msgs::msg::Point MasterSizer_pos = param.get_value<geometry_msgs::msg::Point>();
//     node_->get_parameter("MasterSizer_scale", param);
//     geometry_msgs::msg::Vector3 MasterSizer_scale = param.get_value<geometry_msgs::msg::Vector3>();

//     // Add table if parameters are provided
//     if (!table_pos.x && !table_pos.y && !table_pos.z && !table_scale.x && !table_scale.y && !table_scale.z)
//     {
//       addTable(table_scale, table_pos);
//     }

//     // Add mortar if parameters are provided
//     if (!mortar_pos.x && !mortar_pos.y && !mortar_pos.z)
//     {
//       addMortar(mortar_mesh_file_path, mortar_pos);
//     }

//     // Add funnel if parameters are provided
//     if (!funnel_pos.x && !funnel_pos.y && !funnel_scale.z)
//     {
//       addFunnel(funnel_pos, funnel_scale);
//     }

//     // Add MasterSizer if parameters are provided
//     if (!MasterSizer_pos.x && !MasterSizer_pos.y && !MasterSizer_scale.z)
//     {
//       addMasterSizer(MasterSizer_pos, MasterSizer_scale);
//     }
//   }

// private:
//   void addTable(const geometry_msgs::msg::Vector3& table_scale, const geometry_msgs::msg::Point& table_pos)
//   {
//     moveit_msgs::msg::CollisionObject table;
//     table.id = "Table";
//     table.header.frame_id = planning_frame_;
//     table.primitives.resize(1);
//     table.primitives[0].type = table.primitives[0].BOX;
//     table.primitives[0].dimensions.resize(3);
//     table.primitives[0].dimensions[0] = table_scale.x;
//     table.primitives[0].dimensions[1] = table_scale.y;
//     table.primitives[0].dimensions[2] = table_scale.z;
//     table.primitive_poses.resize(1);
//     table.primitive_poses[0].position = table_pos;
//     table.primitive_poses[0].position.z -= table_scale.z / 2.0;
//     table.operation = table.ADD;

//     planning_scene_interface_->applyCollisionObject(table);
//   }

//   void addMortar(const std::string& file_path, const geometry_msgs::msg::Point& mortar_pos)
//   {
//     moveit_msgs::msg::CollisionObject mortar;
//     mortar.id = "Mortar";
//     mortar.header.frame_id = planning_frame_;
//     mortar.meshes.resize(1);
//     mortar.meshes[0].triangles.clear();
//     mortar.meshes[0].vertices.clear();
//     // Load your STL mesh here and populate mortar.meshes[0].triangles and mortar.meshes[0].vertices
//     // Example code for loading mesh: https://github.com/ros-planning/moveit/blob/master/moveit_planners/ompl/ompl_interface/src/ompl_interface/ompl_interface.cpp#L1160
//     mortar.mesh_poses.resize(1);
//     mortar.mesh_poses[0].position = mortar_pos;
//     mortar.operation = mortar.ADD;

//     planning_scene_interface_->applyCollisionObject(mortar);
//   }

//   void addFunnel(const geometry_msgs::msg::Point& funnel_pos, const geometry_msgs::msg::Vector3& funnel_scale)
//   {
//     moveit_msgs::msg::CollisionObject funnel;
//     funnel.id = "Funnel";
//     funnel.header.frame_id = planning_frame_;
//     funnel.primitives.resize(1);
//     funnel.primitives[0].type = funnel.primitives[0].CYLINDER;
//     funnel.primitives[0].dimensions.resize(2);
//     funnel.primitives[0].dimensions[0] = funnel_scale.x;
//     funnel.primitives[0].dimensions[1] = funnel_scale.z;
//     funnel.primitive_poses.resize(1);
//     funnel.primitive_poses[0].position = funnel_pos;
//     funnel.operation = funnel.ADD;

//     planning_scene_interface_->applyCollisionObject(funnel);
//   }

//   void addMasterSizer(const geometry_msgs::msg::Point& MasterSizer_pos, const geometry_msgs::msg::Vector3& MasterSizer_scale)
//   {
//     moveit_msgs::msg::CollisionObject MasterSizer;
//     MasterSizer.id = "MasterSizer";
//     MasterSizer.header.frame_id = planning_frame_;
//     MasterSizer.primitives.resize(1);
//     MasterSizer.primitives[0].type = MasterSizer.primitives[0].CYLINDER;
//     MasterSizer.primitives[0].dimensions.resize(2);
//     MasterSizer.primitives[0].dimensions[0] = MasterSizer_scale.x;
//     MasterSizer.primitives[0].dimensions[1] = MasterSizer_scale.z;
//     MasterSizer.primitive_poses.resize(1);
//     MasterSizer.primitive_poses[0].position = MasterSizer_pos;
//     MasterSizer.operation = MasterSizer.ADD;

//     planning_scene_interface_->applyCollisionObject(MasterSizer);
//   }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "load_planning_scene", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  PlanningScene planning_scene(node, "ur_manipulator");
//   planning_scene.initPlanningScene();

  rclcpp::shutdown();
  return 0;
}
#endif

int main(int argc, char* argv[])
{
   // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

}