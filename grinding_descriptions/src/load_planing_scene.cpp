#if 1
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
    declare_parameter("table_position", std::vector<double>(0, 0, 0));
    declare_parameter("table_scale", std::vector<double>(0, 0, 0));
    declare_parameter("mortar_top_position", std::vector<double>(0, 0, 0));
    declare_parameter("mortar_inner_scale", std::vector<double>(0, 0, 0));
    declare_parameter("funnel_position", std::vector<double>(0, 0, 0));
    declare_parameter("funnel_scale", std::vector<double>(0, 0, 0));
    declare_parameter("MasterSizer_position", std::vector<double>(0, 0, 0));
    declare_parameter("MasterSizer_scale", std::vector<double>(0, 0, 0));

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
#endif

#if 0
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char *argv[])
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
  auto spinner = std::thread([&executor]()
                             { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  // std::string manipulator_group_name = "panda_arm";
  // std::string reference_link = "panda_link0";
  std::string manipulator_group_name = "ur_manipulator";
  std::string reference_link = "base_link";

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, manipulator_group_name);
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{node, reference_link, rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                             move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closure for updating the text in rviz
  auto const draw_title = [&moveit_visual_tools](auto text)
  {
    auto const text_pose = []
    {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text)
  { moveit_visual_tools.prompt(text); };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup(manipulator_group_name)](
          auto const trajectory)
  { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };

  // Set a target Pose
  auto const target_pose = []
  {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = 0.4; // <---- This value was changed
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create collision object for the robot to avoid
  auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()]
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.2;
    box_pose.position.y = 0.2;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  // Create a plan to that target pose
  prompt("Press 'next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  }
  else
  {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}

#endif