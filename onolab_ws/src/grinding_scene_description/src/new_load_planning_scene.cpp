#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("load_planning_scene");

class PlanningSceneLoader : public rclcpp::Node
{
public:
    PlanningSceneLoader(const std::string &node_name, const std::string &move_group_name)
        : Node(node_name)
    {
        // move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), move_group_name);
        // planning_frame_ = move_group_->getPlanningFrame();
        // planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        init();
    }

    void init()
    {
        planning_scene_diff_publisher_ = node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
        while (planning_scene_diff_publisher_->get_subscription_count() < 1)
        {
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }

    // ボックスオブジェクトを追加するメソッド
    void addBox()
    {
        moveit_msgs::msg::AttachedCollisionObject attached_object;
        attached_object.link_name = "base_link";
        attached_object.object.header.frame_id = "base_link";
        attached_object.object.id = "box";

        /* デフォルトの位置と姿勢 */
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

        // オブジェクトをベクターに追加
        attached_objects_.push_back(attached_object);
    }

    // 新たにオブジェクトを追加するためのメソッド
    void addObject(const moveit_msgs::msg::AttachedCollisionObject &object)
    {
        attached_objects_.push_back(object);
    }

    // プランニングシーンを初期化し、追加されたオブジェクトをパブリッシュするメソッド
    void init_planning_scene()
    {
        moveit_msgs::msg::PlanningScene planning_scene;
        for (const auto &attached_object : attached_objects_)
        {
            planning_scene.world.collision_objects.push_back(attached_object.object);
        }
        planning_scene.is_diff = true;
        planning_scene_diff_publisher_->publish(planning_scene);
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::string planning_frame_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;
    std::vector<moveit_msgs::msg::AttachedCollisionObject> attached_objects_;
};

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
    auto node = std::make_shared<PlanningSceneLoader>("planning_scene_loader", "ur_manipulator");

    // ボックスを追加し、シーンをパブリッシュ
    node->addBox();
    node->init_planning_scene();

    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
