#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <atomic>
#include <thread>
#include <mutex>
#include <cmath>
#include <map>
#include <string>
#include <algorithm>

using std::placeholders::_1;

class JoyMoveItTeleop : public rclcpp::Node
{
public:
  JoyMoveItTeleop()
  : Node("joy_moveit_teleop"),
    speed_linear_(0.06),
    speed_angular_(0.05),
    is_moving_(false),
    pose_initialized_(false),
    base_vel_cmd_(0.0)
  {
    // ---- params ----
    base_vel_topic_ = this->declare_parameter<std::string>(
        "base_vel_topic", "/base_velocity_controller/commands");
    ee_link_ = this->declare_parameter<std::string>(
        "ee_link", "arm6_link");

    // Joy (DualSenseは/joyがbest_effortで来ることが多い)
    rclcpp::QoS qos_joy(1);
    qos_joy.best_effort();
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", qos_joy, std::bind(&JoyMoveItTeleop::joyCallback, this, _1));

    // joint_states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&JoyMoveItTeleop::jointStateCallback, this, _1));

    // RViz表示用 Pose
    rclcpp::QoS qos_pose(1);
    qos_pose.reliable();
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "target_pose", qos_pose);

    // base velocity controller 出力
    base_vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        base_vel_topic_, 10);

    last_joy_stamp_ = this->now();
    base_vel_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),  // 50Hz
        std::bind(&JoyMoveItTeleop::baseVelTimer, this));

    RCLCPP_INFO(this->get_logger(), "JoyMoveItTeleop constructed.");
  }

  void initMoveGroup()
  {
    try {
      auto node_ptr = std::static_pointer_cast<rclcpp::Node>(shared_from_this());

      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
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
  enum class CommandType { None, Pose, Joints };

  struct MotionCommand
  {
    CommandType type{CommandType::None};
    geometry_msgs::msg::Pose pose;
    std::map<std::string, double> joints;
  };

  // ---------- joint_states ----------
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(joint_mutex_);

      const size_t npos = std::min(msg->name.size(), msg->position.size());
      for (size_t i = 0; i < npos; ++i) {
        joint_positions_[msg->name[i]] = msg->position[i];
      }

      const size_t nvel = std::min(msg->name.size(), msg->velocity.size());
      for (size_t i = 0; i < nvel; ++i) {
        joint_velocities_[msg->name[i]] = msg->velocity[i];
      }
    }

    if (!pose_initialized_ && move_group_) {
      try {
        auto pose_stamped = move_group_->getCurrentPose(ee_link_);
        current_pose_ = pose_stamped.pose;
        if (!pose_stamped.header.frame_id.empty()) {
          planning_frame_ = pose_stamped.header.frame_id;
        }
        pose_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial pose initialized from MoveIt.");
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to get initial pose from MoveIt: %s", e.what());
      }
    }
  }

  // ---------- base vel watchdog timer ----------
  void baseVelTimer()
  {
    // Joyが止まったら勝手に回り続けないように0へ戻す
    const auto now = this->now();
    const double timeout_sec = 0.25;

    double cmd = base_vel_cmd_.load();
    if ((now - last_joy_stamp_).seconds() > timeout_sec) {
      cmd = 0.0;
      base_vel_cmd_.store(0.0);
    }

    std_msgs::msg::Float64MultiArray out;
    out.data.resize(1);
    out.data[0] = cmd;
    base_vel_pub_->publish(out);
  }

  // ---------- Joy ----------
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_stamp_ = this->now();

    if (!move_group_) return;

    // axes[7], buttons[5]まで使うので最低これだけ必要
    if (msg->axes.size() < 8) return;
    if (msg->buttons.size() < 6) return;

    // MoveIt動作中は、ベース速度は0へ（競合回避）
    if (is_moving_) {
      base_vel_cmd_.store(0.0);
      return;
    }

    if (joint_positions_.empty()) {
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

    const double deadzone_limit = 0.1;
    auto dz = [deadzone_limit](double v) {
      return (std::abs(v) < deadzone_limit) ? 0.0 : v;
    };

    // ---- pose入力（左スティック + 右スティック縦 + ボタンでroll/pitch）----
    const double lx = dz(msg->axes[0]);
    const double ly = dz(msg->axes[1]);
    const double rz = dz(msg->axes[4]);

    const double dx = ly * speed_linear_;
    const double dy = lx * speed_linear_;
    const double dz_lin = rz * speed_linear_;

    double roll = 0.0, pitch = 0.0;
    if (msg->axes[2] < -0.5) pitch = -speed_angular_;
    if (msg->axes[5] < -0.5) pitch =  speed_angular_;
    if (msg->buttons[6])     roll  = -speed_angular_;
    if (msg->buttons[7])     roll  =  speed_angular_;

    // ---- ベース回転（速度制御）----
    // 十字キー左右: axes[6] が想定（環境で違う場合あり）
    const double base_axis = dz(msg->axes[6]);
    // [rad/s] にしたいなら係数はここ（今は speed_angular_ を流用）
    const double base_vel = base_axis * speed_angular_;
    base_vel_cmd_.store(base_vel);

    // ---- 関節(例: joint2, joint5)はMoveItで位置制御（増分）----
    // 十字キー上下: axes[7]、L1/R1: buttons[4]/[5]
    const double ee_vertical_axis = -dz(msg->axes[7]);
    const double ee_horiz_axis = (double)msg->buttons[4] - (double)msg->buttons[5]; // L1 - R1

    const bool joint_control_active =
        (std::abs(ee_vertical_axis) > 0.0) || (std::abs(ee_horiz_axis) > 0.0);

    MotionCommand cmd;

    if (joint_control_active) {
      std::map<std::string, double> target;
      {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        target = joint_positions_;
      }

      // arm_joint2, arm_joint5 が存在する前提
      auto it2 = target.find("arm_joint2");
      auto it5 = target.find("arm_joint5");
      if (it2 == target.end() || it5 == target.end()) return;

      const double delta2 = ee_vertical_axis * speed_angular_;
      const double delta5 = ee_horiz_axis    * speed_angular_;

      it2->second += delta2;
      it5->second += delta5;

      cmd.type = CommandType::Joints;
      cmd.joints = std::move(target);
      startAsyncMove(cmd);
      return;
    }

    // ---- pose制御（MoveIt）----
    const bool pose_active =
        !(dx == 0 && dy == 0 && dz_lin == 0 && roll == 0 && pitch == 0);

    if (!pose_active) return;

    tf2::Quaternion q_current;
    tf2::fromMsg(current_pose_.orientation, q_current);

    tf2::Quaternion q_delta;
    q_delta.setRPY(roll, pitch, 0.0);

    q_current = q_current * q_delta;
    q_current.normalize();
    current_pose_.orientation = tf2::toMsg(q_current);

    current_pose_.position.x += dx;
    current_pose_.position.y += dy;
    current_pose_.position.z += dz_lin;

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = planning_frame_;
    pose_msg.pose = current_pose_;
    pose_pub_->publish(pose_msg);

    cmd.type = CommandType::Pose;
    cmd.pose = current_pose_;
    startAsyncMove(cmd);
  }

  // ---------- MoveIt 非同期 ----------
  void startAsyncMove(const MotionCommand& cmd)
  {
    if (is_moving_) return;
    is_moving_ = true;

    std::thread([this, cmd]() {
      // 例外でも確実に is_moving_ を戻す
      struct Guard {
        std::atomic<bool>& flag;
        ~Guard(){ flag.store(false); }
      } guard{is_moving_};

      std::lock_guard<std::mutex> lock(move_mutex_);

      try {
        moveit::core::MoveItErrorCode result(moveit::core::MoveItErrorCode::SUCCESS);

        if (cmd.type == CommandType::Pose) {
          move_group_->setPoseTarget(cmd.pose, ee_link_);
          result = move_group_->move();
        } else if (cmd.type == CommandType::Joints) {
          move_group_->setJointValueTarget(cmd.joints);
          result = move_group_->move();
        }

        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(this->get_logger(),
                      "Move failed (error code %d)", result.val);
        }

        // 実行後の姿勢更新
        auto pose_stamped = move_group_->getCurrentPose(ee_link_);
        current_pose_ = pose_stamped.pose;

      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Exception in move thread: %s", e.what());
      }
    }).detach();
  }

  // ---------- members ----------
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  // base velocity controller
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr base_vel_pub_;
  rclcpp::TimerBase::SharedPtr base_vel_timer_;
  std::atomic<double> base_vel_cmd_;
  rclcpp::Time last_joy_stamp_;
  std::string base_vel_topic_;

  geometry_msgs::msg::Pose current_pose_;
  std::string planning_frame_;
  std::string ee_link_;

  double speed_linear_;
  double speed_angular_;

  std::atomic<bool> is_moving_;
  std::mutex move_mutex_;

  bool pose_initialized_;

  std::map<std::string, double> joint_positions_;
  std::map<std::string, double> joint_velocities_;
  std::mutex joint_mutex_;
};

// ---------- main ----------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<JoyMoveItTeleop>();
  node->initMoveGroup();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}