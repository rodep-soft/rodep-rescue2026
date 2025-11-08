#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <memory>
#include <string>

#include "crawler_driver/roboclaw_driver.hpp"

#include <tf2_ros/transform_broadcaster.h>

using namespace crawler_driver::roboclaw;

/**
 * @brief ロボット物理パラメータ
 *
 * YAMLファイルやURDFから読み込むべき情報
 */
struct RobotParameters {
  // 機械的パラメータ
  double wheel_base{0.5};      // 左右のクローラー間の距離 [m]
  double wheel_radius{0.062};  // クローラーの有効半径 [m]

  // エンコーダパラメータ
  int encoder_cpr{256};   // Counts Per Revolution
  int gearbox_ratio{66};  // ギアボックスの減速比
  int pulley_ratio{2};    // プーリーの減速比

  // 計算済みパラメータ
  int total_counts_per_rev() const { return encoder_cpr * gearbox_ratio * pulley_ratio; }

  double meters_per_count() const { return (2.0 * M_PI * wheel_radius) / total_counts_per_rev(); }

  double counts_per_meter() const { return total_counts_per_rev() / (2.0 * M_PI * wheel_radius); }
};

/**
 * @brief Differential Drive Controller
 *
 * Twist (linear.x, angular.z) を左右の車輪速度に変換
 */
class DifferentialDriveController {
public:
  explicit DifferentialDriveController(const RobotParameters& params) : params_(params) {}

  /**
   * @brief Twistを左右車輪速度に変換
   *
   * @param linear_x 前進速度 [m/s]
   * @param angular_z 回転速度 [rad/s]
   * @return {left_vel, right_vel} [m/s]
   */
  std::pair<double, double> twistToWheelVelocities(double linear_x, double angular_z) const {
    // 差動駆動の運動学
    const double v_left = linear_x - (angular_z * params_.wheel_base / 2.0);
    const double v_right = linear_x + (angular_z * params_.wheel_base / 2.0);

    return {v_left, v_right};
  }

  /**
   * @brief 車輪速度 [m/s] をエンコーダカウント速度に変換
   */
  double velocityToCounts(double velocity_m_s) const {
    return velocity_m_s * params_.counts_per_meter();
  }

  /**
   * @brief エンコーダカウント速度を車輪速度 [m/s] に変換
   */
  double countsToVelocity(int32_t counts_per_sec) const {
    return counts_per_sec * params_.meters_per_count();
  }

private:
  const RobotParameters& params_;
};

/**
 * @brief オドメトリ計算
 */
class OdometryCalculator {
public:
  explicit OdometryCalculator(const RobotParameters& params)
    : params_(params), x_(0.0), y_(0.0), theta_(0.0) {}

  void update(int32_t left_counts, int32_t right_counts, double dt) {
    // エンコーダカウントを距離に変換
    const double left_distance = left_counts * params_.meters_per_count();
    const double right_distance = right_counts * params_.meters_per_count();

    // 移動距離と回転角度
    const double distance = (left_distance + right_distance) / 2.0;
    const double delta_theta = (right_distance - left_distance) / params_.wheel_base;

    // オドメトリ更新（簡易版）
    theta_ += delta_theta;
    x_ += distance * std::cos(theta_);
    y_ += distance * std::sin(theta_);
  }

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getTheta() const { return theta_; }

private:
  const RobotParameters& params_;
  double x_, y_, theta_;
};

/**
 * @brief ROS2 Crawler Driver V3
 *
 * より良い設計:
 * - geometry_msgs/Twist を直接受信（標準的）
 * - YAMLからパラメータ読み込み
 * - オドメトリ計算とPublish
 * - 物理単位での制御
 */
class CrawlerDriverV3 : public rclcpp::Node {
public:
  CrawlerDriverV3() : Node("crawler_driver_v3"), estop_active_(false), initialized_(false) {
    // パラメータ宣言（YAMLから読み込み推奨）
    declareParameters();
    loadParameters();

    // コントローラー初期化
    controller_ = std::make_unique<DifferentialDriveController>(robot_params_);
    odometry_ = std::make_unique<OdometryCalculator>(robot_params_);

    // Roboclaw初期化
    const std::string port = get_parameter("roboclaw_port").as_string();
    const int address = get_parameter("roboclaw_address").as_int();
    roboclaw_ = std::make_unique<RoboclawDriver>(port, static_cast<uint8_t>(address));

    // Subscriber
    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&CrawlerDriverV3::twistCallback, this, std::placeholders::_1));

    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "emergency_stop", 10,
        std::bind(&CrawlerDriverV3::estopCallback, this, std::placeholders::_1));

    // Publisher
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Timer for odometry updates
    odom_timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                    std::bind(&CrawlerDriverV3::updateOdometry, this));

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 初期化
    initialize();
  }

private:
  // ハードウェア
  std::unique_ptr<RoboclawDriver> roboclaw_;
  RobotParameters robot_params_;

  // コントローラー
  std::unique_ptr<DifferentialDriveController> controller_;
  std::unique_ptr<OdometryCalculator> odometry_;

  // PID
  PIDConstants m1_pid_, m2_pid_;

  // 状態
  bool estop_active_;
  bool initialized_;
  rclcpp::Time last_odom_time_;

  // ROS通信
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void declareParameters() {
    // Roboclaw設定
    declare_parameter("roboclaw_port", "/dev/roboclaw");
    declare_parameter("roboclaw_address", 0x80);

    // ロボット物理パラメータ
    declare_parameter("wheel_base", 0.5);
    declare_parameter("wheel_radius", 0.062);
    declare_parameter("encoder_cpr", 256);
    declare_parameter("gearbox_ratio", 66);
    declare_parameter("pulley_ratio", 2);

    // PID設定
    declare_parameter("m1_pid.p", 0.464);
    declare_parameter("m1_pid.i", 0.021);
    declare_parameter("m1_pid.d", 0.0);
    declare_parameter("m2_pid.p", 0.438);
    declare_parameter("m2_pid.i", 0.020);
    declare_parameter("m2_pid.d", 0.0);
  }

  void loadParameters() {
    // ロボットパラメータ
    robot_params_.wheel_base = get_parameter("wheel_base").as_double();
    robot_params_.wheel_radius = get_parameter("wheel_radius").as_double();
    robot_params_.encoder_cpr = get_parameter("encoder_cpr").as_int();
    robot_params_.gearbox_ratio = get_parameter("gearbox_ratio").as_int();
    robot_params_.pulley_ratio = get_parameter("pulley_ratio").as_int();

    // PID設定
    m1_pid_.p = static_cast<float>(get_parameter("m1_pid.p").as_double());
    m1_pid_.i = static_cast<float>(get_parameter("m1_pid.i").as_double());
    m1_pid_.d = static_cast<float>(get_parameter("m1_pid.d").as_double());
    m1_pid_.qpps = static_cast<int32_t>(EncoderQPPS::M1);

    m2_pid_.p = static_cast<float>(get_parameter("m2_pid.p").as_double());
    m2_pid_.i = static_cast<float>(get_parameter("m2_pid.i").as_double());
    m2_pid_.d = static_cast<float>(get_parameter("m2_pid.d").as_double());
    m2_pid_.qpps = static_cast<int32_t>(EncoderQPPS::M2);

    // パラメータ表示
    RCLCPP_INFO(get_logger(), "📐 ロボットパラメータ:");
    RCLCPP_INFO(get_logger(), "  wheel_base: %.3f m", robot_params_.wheel_base);
    RCLCPP_INFO(get_logger(), "  wheel_radius: %.3f m", robot_params_.wheel_radius);
    RCLCPP_INFO(get_logger(), "  total_reduction: %d", robot_params_.total_counts_per_rev());
    RCLCPP_INFO(get_logger(), "  counts/meter: %.2f", robot_params_.counts_per_meter());
    RCLCPP_INFO(get_logger(), "  meters/count: %.6f", robot_params_.meters_per_count());
  }

  void initialize() {
    RCLCPP_INFO(get_logger(), "🔧 Roboclaw初期化中...");

    // モーター停止
    roboclaw_->setVelocity(Motor::M1, 0.0, [this](bool success) {
      if (success)
        RCLCPP_INFO(get_logger(), "  M1: 停止 ✓");
    });

    roboclaw_->setVelocity(Motor::M2, 0.0, [this](bool success) {
      if (success)
        RCLCPP_INFO(get_logger(), "  M2: 停止 ✓");
    });

    // PID設定
    roboclaw_->setPID(Motor::M1, m1_pid_, [this](bool success) {
      if (success)
        RCLCPP_INFO(get_logger(), "  M1 PID: 設定完了 ✓");
    });

    roboclaw_->setPID(Motor::M2, m2_pid_, [this](bool success) {
      if (success)
        RCLCPP_INFO(get_logger(), "  M2 PID: 設定完了 ✓");
    });

    // エンコーダリセット
    roboclaw_->resetEncoders([this](bool success) {
      if (success) {
        initialized_ = true;
        last_odom_time_ = now();
        RCLCPP_INFO(get_logger(), "🚀 初期化完了！");
      }
    });
  }

  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (estop_active_ || !initialized_) {
      return;
    }

    // Twist → 左右車輪速度 [m/s]
    auto [left_vel, right_vel] = controller_->twistToWheelVelocities(msg->linear.x, msg->angular.z);

    // 車輪速度 [m/s] → エンコーダカウント速度 [counts/s]
    const double left_counts = controller_->velocityToCounts(left_vel);
    const double right_counts = controller_->velocityToCounts(right_vel);

    RCLCPP_DEBUG(get_logger(), "cmd_vel: linear=%.2f angular=%.2f → L=%.0f R=%.0f counts/s",
                 msg->linear.x, msg->angular.z, left_counts, right_counts);

    // Roboclawへ送信
    roboclaw_->setVelocity(Motor::M1, left_counts, [](bool) {});
    roboclaw_->setVelocity(Motor::M2, right_counts, [](bool) {});
  }

  void estopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && !estop_active_) {
      RCLCPP_WARN(get_logger(), "🚨 E-stop有効化");
      stopMotors();
    } else if (!msg->data && estop_active_) {
      RCLCPP_INFO(get_logger(), "✅ E-stop解除");
    }
    estop_active_ = msg->data;
  }

  void updateOdometry() {
    if (!initialized_)
      return;

    // エンコーダ値読み取り（実装例）
    // TODO: 実際にはRoboclawからエンコーダ値を読み取る
    // roboclaw_->readEncoder(Motor::M1, ...);
    // roboclaw_->readEncoder(Motor::M2, ...);

    // オドメトリPublish
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = odometry_->getX();
    odom_msg.pose.pose.position.y = odometry_->getY();

    tf2::Quaternion q;
    q.setRPY(0, 0, odometry_->getTheta());
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_pub_->publish(odom_msg);
  }

  void stopMotors() {
    roboclaw_->setVelocity(Motor::M1, 0.0, [](bool) {});
    roboclaw_->setVelocity(Motor::M2, 0.0, [](bool) {});
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<CrawlerDriverV3>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "ノード起動失敗: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
