#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <string>
#include <vector>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

namespace grinding_robot_control {

/**
 * @class DynamicTFBroadcaster
 * @brief パラメータに基づいて動的なTFを定期的にブロードキャストするROS 2ノード
 */
class DynamicTFBroadcaster : public rclcpp::Node
{ // NOLINT (misc-misplaced-const)
public:
    explicit DynamicTFBroadcaster(const rclcpp::NodeOptions & options)
    : Node("dynamic_tf_broadcaster_node", options)
    {
        // ROSパラメータの宣言
        this->declare_parameter<std::string>("parent_frame_id", "base_link");
        this->declare_parameter<std::string>("child_frame_id", "dynamic_frame");
        this->declare_parameter<std::vector<double>>("target_position", {0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("translation_offset", {0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("rotation_rpy_offset", {0.0, 0.0, 0.0});
        this->declare_parameter<double>("broadcast_rate", 10.0); // ブロードキャスト周期 (Hz)

        // パラメータの取得
        parent_frame_id_ = this->get_parameter("parent_frame_id").as_string();
        child_frame_id_ = this->get_parameter("child_frame_id").as_string();
        auto target_position = this->get_parameter("target_position").as_double_array();
        auto translation_offset = this->get_parameter("translation_offset").as_double_array();
        auto rotation_rpy_offset = this->get_parameter("rotation_rpy_offset").as_double_array();
        auto broadcast_rate = this->get_parameter("broadcast_rate").as_double();

        // パラメータの検証
        if (target_position.size() != 3 || translation_offset.size() != 3 || rotation_rpy_offset.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "Parameters 'target_position', 'translation_offset', and 'rotation_rpy_offset' must be double arrays of size 3.");
            throw std::invalid_argument("Invalid parameter size");
        }
        
        // TransformBroadcasterを初期化
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Transformメッセージを事前に計算してメンバ変数に保存
        // 並進ベクトルを設定（ベース位置＋オフセット）
        transform_to_broadcast_.translation.x = target_position[0] + translation_offset[0];
        transform_to_broadcast_.translation.y = target_position[1] + translation_offset[1];
        transform_to_broadcast_.translation.z = target_position[2] + translation_offset[2];

        // 回転を設定（オイラー角からクォータニオンへ変換）
        tf2::Quaternion q;
        q.setRPY(
            rotation_rpy_offset[0], // Roll
            rotation_rpy_offset[1], // Pitch
            rotation_rpy_offset[2]  // Yaw
        );
        transform_to_broadcast_.rotation.x = q.x();
        transform_to_broadcast_.rotation.y = q.y();
        transform_to_broadcast_.rotation.z = q.z();
        transform_to_broadcast_.rotation.w = q.w();

        // タイマーを作成して、指定した周期でコールバック関数を実行
        auto period = std::chrono::duration<double>(1.0 / broadcast_rate);
        timer_ = this->create_wall_timer(period, std::bind(&DynamicTFBroadcaster::broadcast_timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Initialized dynamic TF broadcaster from '%s' to '%s' at %.1f Hz.",
            parent_frame_id_.c_str(), child_frame_id_.c_str(), broadcast_rate);
    }

private:
    /**
     * @brief タイマーによって定期的に呼ばれ、TFをブロードキャストするコールバック関数
     */
    void broadcast_timer_callback()
    {
        geometry_msgs::msg::TransformStamped t;

        // ヘッダー情報を設定（タイムスタンプは毎回更新）
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = parent_frame_id_;
        t.child_frame_id = child_frame_id_;

        // 保存しておいた変換情報をセット
        t.transform = transform_to_broadcast_;

        // TFをブロードキャスト
        broadcaster_->sendTransform(t);
    }

    // メンバ変数
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Transform transform_to_broadcast_;
    std::string parent_frame_id_;
    std::string child_frame_id_;
};
} // namespace grinding_robot_control


RCLCPP_COMPONENTS_REGISTER_NODE(grinding_robot_control::DynamicTFBroadcaster)
