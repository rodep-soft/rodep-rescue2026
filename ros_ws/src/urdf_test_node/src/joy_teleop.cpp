#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
    // QoS設定：最新1件のみ保持、軽量なbest_effort
    rclcpp::QoS qos_joy(1);
    qos_joy.best_effort();

    rclcpp::QoS qos_pose(1);
    qos_pose.best_effort();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", qos_joy, std::bind(&JoyMoveItTeleop::joyCallback, this, _1));

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "target_pose", qos_pose);

    RCLCPP_INFO(this->get_logger(), "Joy MoveIt Teleop constructed (light QoS)");
  }

  void initMoveGroup()
  {
    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), "sekirei_arm");

      move_group_->setMaxVelocityScalingFactor(0.5);
      move_group_->setMaxAccelerationScalingFactor(0.5);

      auto pose_stamped = move_group_->getCurrentPose();
      if (pose_stamped.header.frame_id.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current pose");
        return;
      }

      current_pose_ = pose_stamped.pose;
      planning_frame_ = move_group_->getPlanningFrame();

      RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized. Planning frame: %s",
                  planning_frame_.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveGroup: %s", e.what());
    }
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!move_group_) return;
    if (msg->axes.size() < 5) return;

    const double deadzone = 0.1;
    auto applyDeadzone = [deadzone](double value) {
      return (std::abs(value) < deadzone) ? 0.0 : value;
    };

    double dx = applyDeadzone(msg->axes[0]) * speed_linear_;
    double dy = applyDeadzone(msg->axes[1]) * speed_linear_;
    double dz = applyDeadzone(msg->axes[4]) * speed_linear_;
    double dyaw = applyDeadzone(msg->axes[3]) * speed_angular_;

    if (dx == 0.0 && dy == 0.0 && dz == 0.0 && dyaw == 0.0)
      return;

    if (is_moving_) {
      RCLCPP_DEBUG(this->get_logger(), "Still moving, skipping command");
      return;
    }

    current_pose_.position.x += dx;
    current_pose_.position.y += dy;
    current_pose_.position.z += dz;

    tf2::Quaternion q_current;
    tf2::fromMsg(current_pose_.orientation, q_current);
    tf2::Quaternion q_delta;
    q_delta.setRPY(0, 0, dyaw);
    q_current = q_current * q_delta;
    q_current.normalize();
    current_pose_.orientation = tf2::toMsg(q_current);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = planning_frame_;
    pose_msg.pose = current_pose_;
    pose_pub_->publish(pose_msg);

    asyncMove();
  }

  void asyncMove()
  {
    is_moving_ = true;
    std::thread([this]() {
      move_group_->setPoseTarget(current_pose_);
      auto result = move_group_->move();

      if (result == moveit::core::MoveItErrorCode::SUCCESS)
        RCLCPP_DEBUG(this->get_logger(), "Move succeeded");
      else {
        RCLCPP_WARN(this->get_logger(), "Move failed with error code: %d", result);
        current_pose_ = move_group_->getCurrentPose().pose;
      }
      is_moving_ = false;
    }).detach();
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  geometry_msgs::msg::Pose current_pose_;
  std::string planning_frame_;
  double speed_linear_;
  double speed_angular_;
  std::atomic<bool> is_moving_;
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
