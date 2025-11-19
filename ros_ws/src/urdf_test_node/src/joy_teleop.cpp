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
#include <string>

using std::placeholders::_1;

class JoyMoveItTeleop : public rclcpp::Node
{
public:
  JoyMoveItTeleop()
  : Node("joy_moveit_teleop"),
    speed_linear_(0.06),
    speed_angular_(0.05),
    is_moving_(false),
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

    // RViz用 Pose
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

      // move_group_->setPoseReferenceFrame("arm_base_link");

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
    geometry_msgs::msg::Pose pose;                 // type == Pose のとき有効
    std::map<std::string, double> joints;          // type == Joints のとき有効
  };

  // ---------- joint_states コールバック ----------
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      size_t n = std::min(msg->name.size(), msg->position.size());
      for (size_t i = 0; i < n; ++i) {
        joint_positions_[msg->name[i]] = msg->position[i];
      }
    }

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

  // ---------- Joy コールバック ----------
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!move_group_) return;
    if (msg->axes.size() < 7) return;
    if (is_moving_) return;

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
    auto deadzon = [deadzone_limit](double v) {
      return (std::abs(v) < deadzone_limit) ? 0.0 : v;
    };

    // ---- 入力取得 ----
    double lx = deadzon(msg->axes[0]);
    double ly = deadzon(msg->axes[1]);
    double rz = deadzon(msg->axes[4]);

    double dx = ly * speed_linear_;
    double dy = lx * speed_linear_;
    double dz = rz * speed_linear_;

    double roll = 0.0, pitch = 0.0;

    if (msg->axes[2] < -0.5) pitch = -speed_angular_;
    if (msg->axes[5] < -0.5) pitch =  speed_angular_;
    if (msg->buttons[1]) roll = -speed_angular_;
    if (msg->buttons[0]) roll =  speed_angular_;

    // ---- ベース回転入力 ----
    double base_axis = 0.0;   
    double end_effector_left =0.0;
    double end_effector_right =0.0;
    double end_effector_vartical =0.0;

    base_axis = deadzon(msg->axes[6]);//十字キー左右
    end_effector_vartical = -deadzon(msg->axes[7]);//十字キー上下
    end_effector_left =msg->buttons[4];// L1
    end_effector_right =msg->buttons[5]; // R1

    MotionCommand cmd;

    // ジョイント制御
    if (std::abs(base_axis) > 0.0 ||
        std::abs(end_effector_vartical) > 0.0 ||
        std::abs(end_effector_left) > 0.0 ||
        std::abs(end_effector_right) > 0.0)
    {
      std::map<std::string, double> target;
      {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        target = joint_positions_;
      }
    //arm_joint1:ベース回転  2は肩 3は肘 4は手首回転 5はエンドエフェクタ水平 6はエンドエフェクタ垂直 
      auto it_1 = target.find("arm_joint1");
      auto it_2 = target.find("arm_joint2");
      auto it_5 = target.find("arm_joint5");

      if (it_1 == target.end() || it_2== target.end() || it_5== target.end()) return;

      double base_delta = base_axis * speed_angular_;
      double ee_vertical    = end_effector_vartical * speed_angular_;
      double ee_horizontal   = (end_effector_left-end_effector_right) * speed_angular_;
      it_1->second += base_delta;
      it_2->second += ee_vertical;
      it_5->second += ee_horizontal;

      cmd.type   = CommandType::Joints;
      cmd.joints = std::move(target);
    }
    // ポーズ制御
    else {
      if (dx == 0 && dy == 0 && dz == 0 &&
          roll == 0 && pitch == 0) {
        return;
      }

      tf2::Quaternion q_current;
      tf2::fromMsg(current_pose_.orientation, q_current);

      tf2::Quaternion q_delta;
      q_delta.setRPY(roll, pitch, 0.0);

      q_current = q_current * q_delta;
      q_current.normalize();

      current_pose_.orientation = tf2::toMsg(q_current);

      current_pose_.position.x += dx;
      current_pose_.position.y += dy;
      current_pose_.position.z += dz;

      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = this->now();
      pose_msg.header.frame_id = planning_frame_;
      pose_msg.pose = current_pose_;
      pose_pub_->publish(pose_msg);

      cmd.type = CommandType::Pose;
      cmd.pose = current_pose_;
    }

    if (cmd.type != CommandType::None) {
      startAsyncMove(cmd);
    }
  }

  // ---------- MoveIt 非同期実行 ----------
  void startAsyncMove(const MotionCommand& cmd)
  {
    if (is_moving_) return;
    is_moving_ = true;

    std::thread([this, cmd]() {
      std::lock_guard<std::mutex> lock(move_mutex_);

      moveit::core::MoveItErrorCode result(moveit::core::MoveItErrorCode::SUCCESS);

      if (cmd.type == CommandType::Pose) {
        move_group_->setPoseTarget(cmd.pose, "arm6_link");
        result = move_group_->move();
      } else if (cmd.type == CommandType::Joints) {
        move_group_->setJointValueTarget(cmd.joints);
        result = move_group_->move();
      }

      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(),
                    "Move failed (error code %d)", result.val);
      }

      try {
        auto pose_stamped = move_group_->getCurrentPose("arm6_link");
        current_pose_ = pose_stamped.pose;
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to refresh pose after move: %s", e.what());
      }

      is_moving_ = false;
    }).detach();
  }
  // ---------- メンバ変数 ----------
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

  bool pose_initialized_;
  std::map<std::string, double> joint_positions_;
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
