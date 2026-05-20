#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using std::placeholders::_1;
using ServoCommandType = moveit_msgs::srv::ServoCommandType;

class JoyServoTeleop : public rclcpp::Node
{
public:
  JoyServoTeleop()
  : Node("joy_teleop")
  {
    command_frame_ = declare_parameter<std::string>("command_frame", "arm6_link");

    twist_topic_ = declare_parameter<std::string>(
      "twist_topic", "/servo_node/delta_twist_cmds");
    joint_topic_ = declare_parameter<std::string>(
      "joint_topic", "/servo_node/delta_joint_cmds");
    switch_srv_ = declare_parameter<std::string>(
      "switch_command_type_srv", "/servo_node/switch_command_type");

    twist_scale_ = declare_parameter<double>("twist_scale", 0.6);
    joint_scale_ = declare_parameter<double>("joint_scale", 0.8);
    deadzone_ = declare_parameter<double>("deadzone", 0.10);
    joy_timeout_sec_ = declare_parameter<double>("joy_timeout_sec", 0.25);

    // joystick mapping
    axis_ly_ = declare_parameter<int>("axis_ly", 1);
    axis_ry_ = declare_parameter<int>("axis_ry", 4);
    axis_l2_ = declare_parameter<int>("axis_l2", 2);
    axis_r2_ = declare_parameter<int>("axis_r2", 5);
    axis_dpad_x_ = declare_parameter<int>("axis_dpad_x", 6);
    axis_dpad_y_ = declare_parameter<int>("axis_dpad_y", 7);

    button_square_ = declare_parameter<int>("button_square", 3);
    button_triangle_ = declare_parameter<int>("button_triangle", 2);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      rclcpp::SensorDataQoS(),
      std::bind(&JoyServoTeleop::joyCb, this, _1));

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_, 10);
    joint_pub_ = create_publisher<control_msgs::msg::JointJog>(joint_topic_, 10);
    switch_client_ = create_client<moveit_msgs::srv::ServoCommandType>(switch_srv_);

    timer_period_ = 1.0 / 50.0;
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timer_period_)),
      std::bind(&JoyServoTeleop::publishLoop, this));

    RCLCPP_INFO(
      get_logger(),
      "JoyServoTeleop ready. joint=%s twist=%s",
      joint_topic_.c_str(),
      twist_topic_.c_str());
  }

private:

  double getAxis(const sensor_msgs::msg::Joy& msg, int idx) const
  {
    if (idx < 0 || idx >= static_cast<int>(msg.axes.size())) {
      return 0.0;
    }
    return msg.axes[idx];
  }

  int getButton(const sensor_msgs::msg::Joy& msg, int idx) const
  {
    if (idx < 0 || idx >= static_cast<int>(msg.buttons.size())) {
      return 0;
    }
    return msg.buttons[idx];
  }

  double applyDeadzone(double value) const
  {
    return (std::abs(value) < deadzone_) ? 0.0 : value;
  }

  double triggerPress01(const sensor_msgs::msg::Joy& msg, int idx) const
  {
    // 押し込み量を 0.0 ~ 1.0 に変換
    const double a = getAxis(msg, idx);
    double p = 1.0 - a;
    p = std::clamp(p, 0.0, 1.0);
    return applyDeadzone(p);
  }

  void requestCommandType(int8_t type)
  {
    if (current_cmd_type_.load() == type) {
      return;
    }

    if (!switch_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Servo switch service not ready: %s", switch_srv_.c_str());
      return;
    }

    if (switch_in_flight_.exchange(true)) {
      return;
    }

    auto req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    req->command_type = type;

    switch_client_->async_send_request(
      req,
      [this, type](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture fut)
      {
        bool ok = false;
        try {
          ok = fut.get()->success;
        } catch (...) {
          ok = false;
        }

        if (ok) {
          current_cmd_type_.store(type);
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "Failed to set servo command_type=%d", type);
        }

        switch_in_flight_.store(false);
      });
  }

  void joyCb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joy_mtx_);
    last_joy_ = *msg;
    last_joy_time_ = now();
    have_joy_.store(true);
  }

  void publishLoop()
  {
    if (!have_joy_.load()) {
      return;
    }

    sensor_msgs::msg::Joy joy;
    rclcpp::Time stamp;

    {
      std::lock_guard<std::mutex> lock(joy_mtx_);
      joy = last_joy_;
      stamp = last_joy_time_;
    }

    const rclcpp::Time now_t = now();
    if ((now_t - stamp).seconds() > joy_timeout_sec_) {
      return;
    }

    const double joint1 =
      applyDeadzone(getAxis(joy, axis_dpad_x_)) * joint_scale_;
    const double joint4 =
      -applyDeadzone(getAxis(joy, axis_dpad_y_)) * joint_scale_;
    const double joint3 =
      (getButton(joy, button_square_) - getButton(joy, button_triangle_)) * joint_scale_;

    const double joint6 =
      std::clamp(
        (triggerPress01(joy, axis_r2_) - triggerPress01(joy, axis_l2_)) * joint_scale_,
        -1.0, 1.0);

    const double tx =
      std::clamp(applyDeadzone(getAxis(joy, axis_ly_)) * twist_scale_, -1.0, 1.0);
    const double tz =
      std::clamp(applyDeadzone(getAxis(joy, axis_ry_)) * twist_scale_, -1.0, 1.0);

    const bool joint_active =
      (joint1 != 0.0) || (joint3 != 0.0) || (joint4 != 0.0) || (joint6 != 0.0);
    const bool twist_active =
      (tx != 0.0) || (tz != 0.0);

    // 優先 joint jog
    if (joint_active) {
      requestCommandType(ServoCommandType::Request::JOINT_JOG);

      control_msgs::msg::JointJog jog;
      jog.header.stamp = now_t;

      if (joint1 != 0.0) {
        jog.joint_names.push_back("arm_joint1");
        jog.velocities.push_back(joint1);
      }
      if (joint3 != 0.0) {
        jog.joint_names.push_back("arm_joint3");
        jog.velocities.push_back(joint3);
      }
      if (joint4 != 0.0) {
        jog.joint_names.push_back("arm_joint4");
        jog.velocities.push_back(joint4);
      }
      if (joint6 != 0.0) {
        jog.joint_names.push_back("arm_joint6");
        jog.velocities.push_back(joint6);
      }

      jog.duration = timer_period_;
      joint_pub_->publish(jog);
      return;
    }

    if (twist_active) {
      requestCommandType(ServoCommandType::Request::TWIST);

      geometry_msgs::msg::TwistStamped twist;
      twist.header.stamp = now_t;
      twist.header.frame_id = command_frame_;
      twist.twist.linear.x = tx;
      twist.twist.linear.y = 0.0;
      twist.twist.linear.z = tz;
      twist.twist.angular.x = 0.0;
      twist.twist.angular.y = 0.0;
      twist.twist.angular.z = 0.0;

      twist_pub_->publish(twist);
      return;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex joy_mtx_;
  sensor_msgs::msg::Joy last_joy_;
  rclcpp::Time last_joy_time_;
  std::atomic<bool> have_joy_{false};

  std::string command_frame_;
  std::string twist_topic_;
  std::string joint_topic_;
  std::string switch_srv_;

  int axis_ly_;
  int axis_ry_;
  int axis_l2_;
  int axis_r2_;
  int axis_dpad_x_;
  int axis_dpad_y_;
  int button_square_;
  int button_triangle_;

  double twist_scale_;
  double joint_scale_;
  double deadzone_;
  double joy_timeout_sec_;
  double timer_period_;

  std::atomic<int8_t> current_cmd_type_{-1};
  std::atomic<bool> switch_in_flight_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyServoTeleop>());
  rclcpp::shutdown();
  return 0;
}