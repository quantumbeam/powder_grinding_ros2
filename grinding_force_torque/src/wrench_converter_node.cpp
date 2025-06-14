#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // tf2::doTransformのために必要

#include <memory>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

namespace grinding_force_torque {

/**
 * @class WrenchConverter
 * @brief Subscribes to a WrenchStamped topic, transforms it to a target frame, and republishes.
 */
class WrenchConverter : public rclcpp::Node
{ // NOLINT (misc-misplaced-const)
public:
    explicit WrenchConverter(const rclcpp::NodeOptions & options)
    : Node("force_torque_converter", options)
    {
        // ROSパラメータの宣言とデフォルト値の設定
        this->declare_parameter<std::string>("input_topic", "/wrench");
        this->declare_parameter<std::string>("output_topic", "/wrench_on_target");
        this->declare_parameter<std::string>("ft_sensor_frame", "sensor_frame");
        this->declare_parameter<std::string>("target_frame", "target_frame");
        this->declare_parameter<bool>("invert_wrench", false);

        // パラメータの取得
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        ft_sensor_frame_ = this->get_parameter("ft_sensor_frame").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();
        invert_wrench_ = this->get_parameter("invert_wrench").as_bool();

        // TF2のバッファとリスナーを初期化
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // SubscriberとPublisherを初期化
        subscription_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            input_topic_, 10, std::bind(&WrenchConverter::wrench_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(output_topic_, 10);
        
        RCLCPP_INFO(this->get_logger(), "Force/Torque Converter initialized:");
        RCLCPP_INFO(this->get_logger(), "  Input Topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output Topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  FT Sensor Frame: %s", ft_sensor_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Target Frame: %s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Invert Wrench: %s", invert_wrench_ ? "true" : "false");
    }

private:
    /**
     * @brief Callback function for the wrench subscriber.
     * @param msg The incoming WrenchStamped message.
     */
    void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        // 入力メッセージのフレームが設定と一致するか確認
        if (msg->header.frame_id != ft_sensor_frame_) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Incoming wrench frame '%s' does not match configured sensor frame '%s'. Using incoming frame as source.", 
                msg->header.frame_id.c_str(), ft_sensor_frame_.c_str());
        }

        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            // センサーフレームからターゲットフレームへの座標変換を取得
            transform_stamped = tf_buffer_->lookupTransform(
                target_frame_, msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(1.0));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
            return;
        }

        // 変換前のWrenchデータを作成
        geometry_msgs::msg::Wrench wrench_in = msg->wrench;

        // パラメータに応じて力の向きを反転
        if (invert_wrench_) {
            wrench_in.force.x *= -1;
            wrench_in.force.y *= -1;
            wrench_in.force.z *= -1;
            wrench_in.torque.x *= -1;
            wrench_in.torque.y *= -1;
            wrench_in.torque.z *= -1;
        }

        // tf2のヘルパー関数を使ってWrenchを変換
        geometry_msgs::msg::Wrench wrench_out;
        tf2::doTransform(wrench_in, wrench_out, transform_stamped);

        // 変換後のWrenchStampedメッセージを作成
        auto output_msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
        output_msg->header.stamp = msg->header.stamp;
        output_msg->header.frame_id = target_frame_;
        output_msg->wrench = wrench_out;

        // メッセージをパブリッシュ
        publisher_->publish(std::move(output_msg));
    }

    // ROS通信関連
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // パラメータ
    std::string input_topic_;
    std::string output_topic_;
    std::string ft_sensor_frame_;
    std::string target_frame_;
    bool invert_wrench_;
};
} // namespace grinding_force_torque


RCLCPP_COMPONENTS_REGISTER_NODE(grinding_force_torque::WrenchConverter)
