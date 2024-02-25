#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>


class PlanningScene
{
public:
  PlanningScene(const std::shared_ptr<rclcpp::Node> &node, const std::string &move_group_name)
      : node_(node)
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, move_group_name);
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    planning_frame_ = move_group_->getPlanningFrame();

    // パラメータを宣言
    node -> declare_parameter("table_position", std::vector<double>(3, 0));
    node -> declare_parameter("table_scale", std::vector<double>(3, 0));
    node -> declare_parameter("mortar_top_position", std::vector<double>(3, 0));
    node -> declare_parameter("mortar_inner_scale", std::vector<double>(3, 0));
    node -> declare_parameter("funnel_position", std::vector<double>(3, 0));
    node -> declare_parameter("funnel_scale", std::vector<double>(3, 0));
    node -> declare_parameter("MasterSizer_position", std::vector<double>(3, 0));
    node -> declare_parameter("MasterSizer_scale", std::vector<double>(3, 0));

    auto table_pos = node->get_parameter("table_position").as_double_array();
    auto table_scale = node->get_parameter("table_scale").as_double_array();
    auto mortar_top_pos = node->get_parameter("mortar_top_position").as_double_array();
    auto mortar_inner_scale = node->get_parameter("mortar_inner_scale").as_double_array();
    auto funnel_pos = node->get_parameter("funnel_position").as_double_array();
    auto funnel_scale = node->get_parameter("funnel_scale").as_double_array();
    auto MasterSizer_pos = node->get_parameter("MasterSizer_position").as_double_array();
    auto MasterSizer_scale = node->get_parameter("MasterSizer_scale").as_double_array();
    std::cout << "table_pos: ";
    for (auto value : table_pos) {
        std::cout << value << " ";
    }
    std::cout << std::endl;
  }

  void initPlanningScene()
  {
    std::string mortar_mesh_file_path = "package://grinding_descriptions/mesh/moveit_scene_object/mortar_40mm.stl";
    rclcpp::Parameter param;

    // Add table if parameters are provided
    // if (!table_pos.x && !table_pos.y && !table_pos.z && !table_scale.x && !table_scale.y && !table_scale.z)
    // {

    //   addTable(table_pos, table_scale);
    // }

    // Add mortar if parameters are provided
    // if (!mortar_pos.x && !mortar_pos.y && !mortar_pos.z)
    // {
    //   addMortar(mortar_mesh_file_path, mortar_pos);
    // }

    // Add funnel if parameters are provided
    // if (!funnel_pos.x && !funnel_pos.y && !funnel_pos.z && !funnel_scale.x && !funnel_scale.y && !funnel_scale.z)
    // {
    //   addFunnel(funnel_pos, funnel_scale);
    // }

    // Add MasterSizer if parameters are provided
    // if (!MasterSizer_pos.x && !MasterSizer_pos.y && !MasterSizer_pos.z && !MasterSizer_scale.x && !MasterSizer_scale.y && !MasterSizer_scale.z)
    // {
    //   addMasterSizer(MasterSizer_pos, MasterSizer_scale);
    // }
  }

  // private:
  //   void addTable(const geometry_msgs::msg::Point &table_pos, const geometry_msgs::msg::Vector3 &table_scale)
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

  //   void addMortar(const std::string &file_path, const geometry_msgs::msg::Point &mortar_pos)
  //   {
  //     moveit_msgs::msg::CollisionObject mortar;
  //     mortar.id = "Mortar";
  //     mortar.header.frame_id = planning_frame_;
  //     mortar.meshes.resize(1);
  //     mortar.meshes[0].triangles.clear();
  //     mortar.meshes[0].vertices.clear();
  //     mortar.mesh_poses.resize(1);
  //     mortar.mesh_poses[0].position = mortar_pos;
  //     mortar.operation = mortar.ADD;

  //     planning_scene_interface_->applyCollisionObject(mortar);
  //   }

  //   void addFunnel(const geometry_msgs::msg::Point &funnel_pos, const geometry_msgs::msg::Vector3 &funnel_scale)
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

  //   void addMasterSizer(const geometry_msgs::msg::Point &MasterSizer_pos, const geometry_msgs::msg::Vector3 &MasterSizer_scale)
  // {
  //   moveit_msgs::msg::CollisionObject MasterSizer;
  //   MasterSizer.id = "MasterSizer";
  //   MasterSizer.header.frame_id = planning_frame_;
  //   MasterSizer.primitives.resize(1);
  //   MasterSizer.primitives[0].type = MasterSizer.primitives[0].CYLINDER;
  //   MasterSizer.primitives[0].dimensions.resize(2);
  //   MasterSizer.primitives[0].dimensions[0] = MasterSizer_scale.x;
  //   MasterSizer.primitives[0].dimensions[1] = MasterSizer_scale.z;
  //   MasterSizer.primitive_poses.resize(1);
  //   MasterSizer.primitive_poses[0].position = MasterSizer_pos;
  //   MasterSizer.operation = MasterSizer.ADD;

  //   planning_scene_interface_->applyCollisionObject(MasterSizer);
  // }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::string planning_frame_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "load_planning_scene", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]()
                             { executor.spin(); });
  PlanningScene planning_scene(node, "ur_manipulator");
  planning_scene.initPlanningScene();

  // // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
