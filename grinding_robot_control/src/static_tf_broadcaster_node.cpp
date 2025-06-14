#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <string>
#include <vector>
#include <memory>

namespace grinding_robot_control {

/**
 * @class StaticTFBroadcaster
 * @brief パラメータに基づいて静的なTFを一度だけブロードキャストするROS 2ノード
 */
class StaticTFBroadcaster : public rclcpp::Node
{ // NOLINT (misc-misplaced-const)
public:
    explicit StaticTFBroadcaster(const rclcpp::NodeOptions & options)
    : Node("static_tf_broadcaster_node", options)
    {
        // ROSパラメータの宣言
        this->declare_parameter<std::string>("parent_frame_id", "base_link");
        this->declare_parameter<std::string>("child_frame_id", "ft_sensor_base");
        this->declare_parameter<std::vector<double>>("target_position", {0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("translation_offset", {0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("rotation_rpy_offset", {0.0, 0.0, 0.0});

        // パラメータの取得
        auto parent_frame_id = this->get_parameter("parent_frame_id").as_string();
        auto child_frame_id = this->get_parameter("child_frame_id").as_string();
        auto target_position = this->get_parameter("target_position").as_double_array();
        auto translation_offset = this->get_parameter("translation_offset").as_double_array();
        auto rotation_rpy_offset = this->get_parameter("rotation_rpy_offset").as_double_array();

        // パラメータの検証
        if (target_position.size() != 3 || translation_offset.size() != 3 || rotation_rpy_offset.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "Parameters 'target_position', 'translation_offset', and 'rotation_rpy_offset' must be double arrays of size 3.");
            // エラーが発生した場合、ノードを終了させる
            throw std::invalid_argument("Invalid parameter size");
        }
        
        // StaticTransformBroadcasterを初期化
        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // TransformStampedメッセージを作成
        geometry_msgs::msg::TransformStamped t;

        // ヘッダー情報を設定
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = parent_frame_id;
        t.child_frame_id = child_frame_id;

        // 並進ベクトルを設定（ベース位置＋オフセット）
        t.transform.translation.x = target_position[0] + translation_offset[0];
        t.transform.translation.y = target_position[1] + translation_offset[1];
        t.transform.translation.z = target_position[2] + translation_offset[2];

        // 回転を設定（オイラー角からクォータニオンへ変換）
        tf2::Quaternion q;
        q.setRPY(
            rotation_rpy_offset[0], // Roll
            rotation_rpy_offset[1], // Pitch
            rotation_rpy_offset[2]  // Yaw
        );
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // 静的TFをブロードキャスト
        broadcaster_->sendTransform(t);
        
        RCLCPP_INFO(this->get_logger(), "Successfully broadcasted static transform from '%s' to '%s'.", parent_frame_id.c_str(), child_frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "  Translation: [%.3f, %.3f, %.3f]", t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
        RCLCPP_INFO(this->get_logger(), "  Rotation (RPY): [%.3f, %.3f, %.3f]", rotation_rpy_offset[0], rotation_rpy_offset[1], rotation_rpy_offset[2]);
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};
} // namespace grinding_robot_control


RCLCPP_COMPONENTS_REGISTER_NODE(grinding_robot_control::StaticTFBroadcaster)
