#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <atomic>
#include <thread>
#include <mutex>
#include <cmath>
#include <map>

using std::placeholders::_1;

class JoyMoveItTeleop : public rclcpp::Node
{
public:
  JoyMoveItTeleop()
  : Node("joy_moveit_teleop"),
    speed_linear_(0.06),
    speed_angular_(0.05),
    is_moving_(false),
    have_joint_state_(false),
    pose_initialized_(false)
  {
    // Joy
    rclcpp::QoS qos_joy(1);
    qos_joy.best_effort();
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", qos_joy, std::bind(&JoyMoveItTeleop::joyCallback, this, _1));

    // joint_states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&JoyMoveItTeleop::jointStateCallback, this, _1));

    // Pose
    rclcpp::QoS qos_pose(1);
    qos_pose.reliable();
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "target_pose", qos_pose);

    RCLCPP_INFO(this->get_logger(), "JoyMoveItTeleop constructed.");
  }

  void initMoveGroup()
  {
    try {
      auto node_ptr = std::static_pointer_cast<rclcpp::Node>(shared_from_this());

      move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_ptr, "sekirei_arm");

      move_group_->setMaxVelocityScalingFactor(0.5);
      move_group_->setMaxAccelerationScalingFactor(0.5);

      planning_frame_ = move_group_->getPlanningFrame();

      RCLCPP_INFO(this->get_logger(),
                  "MoveGroupInterface ready. Planning frame: %s",
                  planning_frame_.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to init MoveGroup: %s", e.what());
    }
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    have_joint_state_ = true;

    // 関節角を保存
    {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      size_t n = std::min(msg->name.size(), msg->position.size());
      for (size_t i = 0; i < n; ++i) {
        joint_positions_[msg->name[i]] = msg->position[i];
      }
    }

    // まだ current_pose_ を MoveIt から取っていないなら、ここで一度だけ
    if (!pose_initialized_ && move_group_) {
      try {
        auto pose_stamped = move_group_->getCurrentPose("arm6_link");
        current_pose_ = pose_stamped.pose;
        if (!pose_stamped.header.frame_id.empty()) {
          planning_frame_ = pose_stamped.header.frame_id;
        }
        pose_initialized_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Initial pose initialized from MoveIt.");
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to get initial pose from MoveIt: %s", e.what());
      }
    }
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!move_group_) return;
    if (msg->axes.size() < 5) return;
    if (is_moving_) return;

    if (!have_joint_state_) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Waiting for /joint_states...");
      return;
    }
    if (!pose_initialized_) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Waiting for initial pose from MoveIt...");
      return;
    }

    const double dz_limit = 0.1;
    auto dz = [dz_limit](double v) {
      return (std::abs(v) < dz_limit) ? 0.0 : v;
    };

    // 入力取得 
    double lx = dz(msg->axes[0]);   // 左スティック左右
    double ly = dz(msg->axes[1]);   // 左スティック上下
    double rz = dz(msg->axes[4]);   // 右スティック上下

    double dx = ly * speed_linear_;
    double dy = lx * speed_linear_;
    double dz_ = rz * speed_linear_;

    double roll = 0.0, pitch = 0.0, yaw = 0.0;

    // 回転入力
    if (msg->buttons[5]) yaw =  speed_angular_;   // R1
    if (msg->buttons[4]) yaw = -speed_angular_;   // L1
    if (msg->axes[2] < -0.5) pitch = -speed_angular_;  // L2
    if (msg->buttons[1]) roll = -speed_angular_;       // ○
    if (msg->buttons[0]) roll =  speed_angular_;       // □

    // ベースの回転（十字キー左右）
    double base_axis = 0.0;
    if (msg->axes.size() > 6) {
      base_axis = dz(msg->axes[6]);
    }

    if (std::abs(base_axis) > 0.0) {
      // 現在の全関節角をコピー
      std::map<std::string, double> target;
      {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        target = joint_positions_;  // 全関節の現在値
      }

      auto it = target.find("arm_joint1");
      if (it == target.end()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "arm_joint1 not found in joint_positions.");
        return;
      }

      double base_delta = base_axis * speed_angular_;
      it->second += base_delta; 

      asyncMoveBase(target);
      return;
    }

    // 回転入力がない場合はPose制御

    if (dx == 0 && dy == 0 && dz_ == 0 &&
        roll == 0 && pitch == 0 && yaw == 0)
      return;

    // Pose 更新
    tf2::Quaternion q_current;
    tf2::fromMsg(current_pose_.orientation, q_current);

    tf2::Quaternion q_delta;
    q_delta.setRPY(roll, pitch, yaw);

    q_current = q_current * q_delta;
    q_current.normalize();

    current_pose_.orientation = tf2::toMsg(q_current);

    current_pose_.position.x += dx;
    //current_pose_.position.y += dy;  // 必要ならON
    current_pose_.position.z += dz_;

    // RViz に出力
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = planning_frame_;
    pose_msg.pose = current_pose_;
    pose_pub_->publish(pose_msg);

    // MoveIt 実行（Poseターゲット）
    asyncMovePose();
  }

  //MoveIt 非同期実行
  void asyncMovePose()
  {
    is_moving_ = true;

    std::thread([this]() {
      std::lock_guard<std::mutex> lock(move_mutex_);

      move_group_->setPoseTarget(current_pose_, "arm6_link");

      auto result = move_group_->move();

      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(),
                    "Pose move failed (error code %d)", result.val);
        try {
          auto pose_stamped = move_group_->getCurrentPose("arm6_link");
          current_pose_ = pose_stamped.pose;
        } catch (const std::exception &e) {
          RCLCPP_WARN(this->get_logger(),
                      "Failed to refresh current pose after failure: %s", e.what());
        }
      }

      is_moving_ = false;
    }).detach();
  }

  //MoveIt 非同期実行 ベース回転用
  void asyncMoveBase(const std::map<std::string, double>& target)
  {
    is_moving_ = true;

    std::thread([this, target]() {
      std::lock_guard<std::mutex> lock(move_mutex_);

      move_group_->setJointValueTarget(target);

      auto result = move_group_->move();

      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(),
                    "Base joint move failed (error code %d)", result.val);
      }

      // 実行後の姿勢を current_pose_ に反映
      try {
        auto pose_stamped = move_group_->getCurrentPose("arm6_link");
        current_pose_ = pose_stamped.pose;
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to refresh pose after base move: %s", e.what());
      }

      is_moving_ = false;
    }).detach();
  }

  // メンバ変数
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  geometry_msgs::msg::Pose current_pose_;
  std::string planning_frame_;

  double speed_linear_;
  double speed_angular_;

  std::atomic<bool> is_moving_;
  std::mutex move_mutex_;

  std::atomic<bool> have_joint_state_;
  std::atomic<bool> pose_initialized_;

  // 全関節角を保存
  std::map<std::string, double> joint_positions_;
  std::mutex joint_mutex_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<JoyMoveItTeleop>();
  node->initMoveGroup();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
