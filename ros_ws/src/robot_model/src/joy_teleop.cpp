#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <moveit/move_group_interface/move_group_interface.h>

using std::placeholders::_1;

class JoyMoveItTeleop : public rclcpp::Node {
public:
  JoyMoveItTeleop() : Node("joy_moveit_teleop"), speed_linear_(0.1), speed_angular_(0.1) {
    // JoySubscription
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&JoyMoveItTeleop::joyCallback, this, _1));

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);

    RCLCPP_INFO(this->get_logger(), "Joy MoveIt Teleop constructed (waiting for initMoveGroup)");
  }

  void initMoveGroup() {
    // 無限ループしたからコメントアウト
    //  while (!this->has_parameter("robot_description"))
    //  {
    //      RCLCPP_INFO(this->get_logger(), "Waiting for robot_description parameter...");
    //      std::this_thread::sleep_for(std::chrono::seconds(1));
    //      rclcpp::spin_some(this->get_node_base_interface());
    //  }

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "sekirei_arm");
    current_pose_ = move_group_->getCurrentPose().pose;
    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized");
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (!move_group_)
      return;
    if (msg->axes.size() < 4)
      return;

    double dx = msg->axes[0] * speed_linear_;
    double dy = msg->axes[1] * speed_linear_;
    double dz = msg->axes[4] * speed_linear_;
    double dyaw = msg->axes[3] * speed_angular_;

    current_pose_.position.x += dx;
    current_pose_.position.y += dy;
    current_pose_.position.z += dz;

    double yaw = getYawFromQuaternion(current_pose_.orientation);
    yaw += dyaw;
    current_pose_.orientation = createQuaternionMsgFromYaw(yaw);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "base_link";
    pose_msg.pose = current_pose_;
    pose_pub_->publish(pose_msg);

    move_group_->setPoseTarget(current_pose_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      move_group_->execute(plan);
      RCLCPP_INFO(this->get_logger(), "Pose updated via joystick");
    } else {
      RCLCPP_WARN(this->get_logger(), "Planning failed");
    }
  }

  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = sin(yaw / 2.0);
    q.w = cos(yaw / 2.0);
    return q;
  }

  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  geometry_msgs::msg::Pose current_pose_;
  double speed_linear_;
  double speed_angular_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyMoveItTeleop>();
  node->initMoveGroup();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
