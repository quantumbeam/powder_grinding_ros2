#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_srvs/srv/empty.hpp>
#include <Eigen/Dense>
#include <deque>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

// 6軸力覚データ用のEigenベクトル型エイリアス
using WrenchVector = Eigen::Vector<double, 6>;

namespace grinding_force_torque {

/**
 * @brief Butterworthローパスフィルタをサンプルごとに適用するクラス
 * Direct Form II Transposed を使用
 */
class ButterworthLowPass {
public:
    // コンストラクタ: フィルタ係数を設定
    ButterworthLowPass(const std::vector<double>& b, const std::vector<double>& a)
        : b_(b), a_(a) {
        if (a_.empty() || std::abs(a_[0]) < 1e-9) {
            throw std::invalid_argument("First 'a' coefficient cannot be zero.");
        }
        // a[0]で正規化
        if (std::abs(a_[0] - 1.0) > 1e-9) {
            double a0 = a_[0];
            for (auto& val : b_) val /= a0;
            for (auto& val : a_) val /= a0;
        }
        // 状態ベクトルのサイズを決定 (order = a.size() - 1)
        z_.assign(a_.size() - 1, 0.0);
    }

    // フィルタをリセットして初期状態に戻す
    void reset() {
        std::fill(z_.begin(), z_.end(), 0.0);
    }

    // 新しいサンプル値にフィルタを適用し、結果を返す
    double filter(double x) {
        double y = b_[0] * x + z_[0];
        for (size_t i = 0; i < z_.size() - 1; ++i) {
            z_[i] = b_[i + 1] * x + z_[i + 1] - a_[i + 1] * y;
        }
        z_.back() = b_.back() * x - a_.back() * y;
        return y;
    }

private:
    std::vector<double> b_; // 分子(numerator)係数
    std::vector<double> a_; // 分母(denominator)係数
    std::vector<double> z_; // 状態ベクトル
};

/**
 * @brief ROS 2ノード本体
 */
class FTFilterNode : public rclcpp::Node { // NOLINT (misc-misplaced-const)
public:
    explicit FTFilterNode(const rclcpp::NodeOptions & options)
    : Node("wrench_filter", options) {
        // ROSパラメータの宣言
        this->declare_parameter<std::string>("input_topic", "/wrench_raw");
        this->declare_parameter<std::string>("output_topic", "/wrench_filtered");
        this->declare_parameter<int>("data_window", 100);
        this->declare_parameter<bool>("initial_zero", true);
        this->declare_parameter<bool>("disable_filtering", false);

        // 注意: フィルタ係数はパラメータとして与える必要があります
        // Python版のscipy.signal.butter(order, cutoff / (fs/2))に相当
        this->declare_parameter<std::vector<double>>("filter_b", {0.0001, 0.0003, 0.0003, 0.0001}); // 例
        this->declare_parameter<std::vector<double>>("filter_a", {1.0, -2.847, 2.706, -0.858});   // 例

        // パラメータの取得
        auto input_topic = this->get_parameter("input_topic").as_string();
        auto output_topic = this->get_parameter("output_topic").as_string();
        data_window_ = this->get_parameter("data_window").as_int();
        initial_zero_ = this->get_parameter("initial_zero").as_bool();
        disable_filtering_ = this->get_parameter("disable_filtering").as_bool();
        auto b_coeffs = this->get_parameter("filter_b").as_double_array();
        auto a_coeffs = this->get_parameter("filter_a").as_double_array();

        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());

        // 6軸それぞれにフィルタを初期化
        for (int i = 0; i < 6; ++i) {
            filters_.emplace_back(b_coeffs, a_coeffs);
        }
        wrench_offset_.setZero();
        latest_filtered_wrench_.setZero();

        // ROS通信の設定
        subscription_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            input_topic, 10, std::bind(&FTFilterNode::wrench_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(output_topic, 10);
        
        zero_service_ = this->create_service<std_srvs::srv::Empty>(
            output_topic + "/zero_ftsensor",
            std::bind(&FTFilterNode::handle_zero, this, std::placeholders::_1, std::placeholders::_2));

        // 自動ゼロ初期化タイマー
        if (initial_zero_) {
            zero_initialized_ = false;
            timer_ = this->create_wall_timer(1s, std::bind(&FTFilterNode::check_and_zero_init, this));
        } else {
            zero_initialized_ = true;
        }
    }

private:
    // WrenchStampedメッセージを受信したときのコールバック関数
    void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        WrenchVector current_wrench = from_wrench(msg->wrench);
        
        data_queue_.push_back(current_wrench);
        if (data_queue_.size() > static_cast<size_t>(data_window_)) {
            data_queue_.pop_front();
        }

        // フィルタリング無効の場合
        if (disable_filtering_) {
            latest_filtered_wrench_ = current_wrench;
        } else {
            // 6軸それぞれにフィルタを適用
            for (int i = 0; i < 6; ++i) {
                latest_filtered_wrench_(i) = filters_[i].filter(current_wrench(i));
            }
        }
        
        // オフセットを引いた最終的な力を計算
        WrenchVector final_wrench = latest_filtered_wrench_ - wrench_offset_;

        // メッセージを組み立ててパブリッシュ
        auto filtered_msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
        filtered_msg->header = msg->header;
        filtered_msg->wrench = to_wrench(final_wrench);
        publisher_->publish(std::move(filtered_msg));
    }

    // ゼロ点調整サービスのコールバック関数
    void handle_zero(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                     std::shared_ptr<std_srvs::srv::Empty::Response>) {
        if (data_queue_.size() < static_cast<size_t>(data_window_)) {
            RCLCPP_WARN(this->get_logger(), "Not enough data to compute offset. Need %d, have %zu.", data_window_, data_queue_.size());
            return;
        }
        wrench_offset_ = latest_filtered_wrench_;
        RCLCPP_INFO(this->get_logger(), "FT sensor zeroed (offset updated).");
    }

    // 起動時の自動ゼロ点調整
    void check_and_zero_init() {
        if (!zero_initialized_ && data_queue_.size() >= static_cast<size_t>(data_window_)) {
            wrench_offset_ = latest_filtered_wrench_;
            zero_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Auto zeroing complete.");
            timer_->cancel(); // タイマーを停止
        }
    }

    // geometry_msgs::msg::Wrench から Eigen::Vector への変換
    WrenchVector from_wrench(const geometry_msgs::msg::Wrench& msg) {
        WrenchVector array;
        array << msg.force.x, msg.force.y, msg.force.z,
                 msg.torque.x, msg.torque.y, msg.torque.z;
        return array;
    }

    // Eigen::Vector から geometry_msgs::msg::Wrench への変換
    geometry_msgs::msg::Wrench to_wrench(const WrenchVector& array) {
        geometry_msgs::msg::Wrench msg;
        msg.force.x = array(0);
        msg.force.y = array(1);
        msg.force.z = array(2);
        msg.torque.x = array(3);
        msg.torque.y = array(4);
        msg.torque.z = array(5);
        return msg;
    }

    // ROS通信関連
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr zero_service_;
    rclcpp::TimerBase::SharedPtr timer_;

    // メンバ変数
    std::deque<WrenchVector> data_queue_;
    std::vector<ButterworthLowPass> filters_;
    WrenchVector wrench_offset_;
    WrenchVector latest_filtered_wrench_;
    
    int data_window_;
    bool initial_zero_;
    bool disable_filtering_;
    bool zero_initialized_ = false;
};

} // namespace grinding_force_torque

RCLCPP_COMPONENTS_REGISTER_NODE(grinding_force_torque::FTFilterNode)