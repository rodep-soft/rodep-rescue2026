#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <atomic>
#include <thread>
#include <cmath>

using std::placeholders::_1;


class JoyMoveItTeleop : public rclcpp::Node
{
public:
  JoyMoveItTeleop()
  : Node("joy_moveit_teleop"),
    speed_linear_(0.06),
    speed_angular_(0.05),
    is_moving_(false)
  {
    // QoS設定：軽量化
    rclcpp::QoS qos(1);
    qos.best_effort();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", qos, std::bind(&JoyMoveItTeleop::joyCallback, this, _1));

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "target_pose", qos);

    RCLCPP_INFO(this->get_logger(), "JoyMoveItTeleop started");
  }

  void initMoveGroup()
  {
    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), "sekirei_arm");

      
      move_group_->setMaxVelocityScalingFactor(0.5);
      move_group_->setMaxAccelerationScalingFactor(0.5);

      auto pose_stamped = move_group_->getCurrentPose();
      current_pose_ = pose_stamped.pose;
      planning_frame_ = move_group_->getPlanningFrame();

      RCLCPP_INFO(this->get_logger(),
                  "MoveGroupInterface ready. Planning frame: %s",
                  planning_frame_.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to init MoveGroup: %s", e.what());
    }
  }

private:
  // ----------- Joyコールバック -----------
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!move_group_) return;
    if (msg->axes.size() < 5) return;

  

    // --- デッドゾーン処理 ---
    const double deadzone = 0.1;
    auto dz = [deadzone](double v) { return (std::abs(v) < deadzone) ? 0.0 : v; };

    // --- 軸入力 ---
    double lx = dz(msg->axes[0]); // 左スティック左右 → Y軸（
    double ly = dz(msg->axes[1]); // 左スティック上下 → X軸
    double rz = dz(msg->axes[4]); // 右スティック上下 → Z軸
    double dx = 0.0, dy = 0.0, dz_ = 0.0;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;

    
    dx = ly * speed_linear_;
    dy = lx * speed_linear_;
    dz_ = rz * speed_linear_;


      
     
    if (msg->buttons[5]) yaw =  speed_angular_;  // R1　エンドエフェクタ上
    if (msg->buttons[4]) yaw = -speed_angular_;  // L1　エンドエフェクタ下
    // if (msg->axes[5] < -0.5) pitch =  speed_angular_;  // R2 エンドエフェクタ左
    if (msg->axes[2] < -0.5) pitch = -speed_angular_;  // L2　エンドエフェクタ右
    if (msg->buttons[1]) roll =  -speed_angular_;       // ○　エンドエフェクタ右回転
    if (msg->buttons[0]) roll = speed_angular_;       // □ エンドエフェクタ左回転

    std::vector<double> joints = move_group_->getCurrentJointValues();

  // // 安全チェック（ジョイント数）
  //   if (joints.empty()) return;

  // // 左スティック左右でベース回転
  //   double base_delta = msg->axes[5] * speed_angular_;

  // // 0番目のジョイント（arm_joint1）を更新
  //   joints[0] += base_delta;
    // move_group_->setJointValueTarget(joints);



       

    // 入力がなければ終了
    if (dx == 0 && dy == 0 && dz_ == 0 && roll == 0 && pitch == 0 && yaw == 0) return;
    if (is_moving_) return;

    // --- 現Pose更新 ---
    
      // 姿勢はエンドエフェクタ座標系基準
      tf2::Quaternion q_current;
      tf2::fromMsg(current_pose_.orientation, q_current);

      tf2::Quaternion q_delta;
      q_delta.setRPY(roll, pitch, yaw); // エンドエフェクタ基準
      q_current = q_current * q_delta;
      q_current.normalize();

      current_pose_.orientation = tf2::toMsg(q_current);

      current_pose_.position.x += dx;
      //current_pose_.position.y += dy;
      current_pose_.position.z += dz_;


    // --- Pose出力（RViz可視化用） ---
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = this->now();
      pose_msg.header.frame_id = planning_frame_;
      pose_msg.pose = current_pose_;
      pose_pub_->publish(pose_msg);

      // --- MoveItへ非同期実行 ---
      asyncMove();
  }

  // ----------- MoveIt動作実行 -----------
  void asyncMove()
  {
    
    is_moving_ = true;
    std::thread([this]() {
      move_group_->setPoseTarget(current_pose_, "arm6_link");
      
      
      auto result = move_group_->move();

      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "Move failed (error code %d)", result.val);
        current_pose_ = move_group_->getCurrentPose("arm6_link").pose;
      }
      is_moving_ = false;
    }).detach();
  }

  // ----------- メンバ変数 -----------
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  geometry_msgs::msg::Pose current_pose_;
  std::string planning_frame_;
  double speed_linear_;
  double speed_angular_;
  std::atomic<bool> is_moving_;
};

// ----------- main() -----------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyMoveItTeleop>();
  node->initMoveGroup();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
