#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>

#include <moveit_msgs/srv/servo_command_type.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>
#include <fmt/format.h>

using std::placeholders::_1;

class JoyServoTeleop : public rclcpp::Node
{
public:
  JoyServoTeleop()
  : Node("joy_teleop")
  {
    servo_node_name_ = declare_parameter<std::string>("servo_node_name", "/servo_node");
    planning_frame_  = declare_parameter<std::string>("planning_frame", "base_link");
    command_frame_   = declare_parameter<std::string>("command_frame", "arm6_link");

    twist_topic_ = declare_parameter<std::string>(
      "twist_topic", servo_node_name_ + "/delta_twist_cmds");
    joint_topic_ = declare_parameter<std::string>(
      "joint_topic", servo_node_name_ + "/delta_joint_cmds");
    switch_srv_  = declare_parameter<std::string>(
      "switch_command_type_srv", servo_node_name_ + "/switch_command_type");

    twist_scale_  = declare_parameter<double>("twist_scale", 0.6);   // [-1..1]
    joint_scale_  = declare_parameter<double>("joint_scale", 0.8);   // [-1..1]
    stick_deadzone_   = declare_parameter<double>("stick_deadzone", 0.20);
    trigger_deadzone_ = declare_parameter<double>("trigger_deadzone", 0.05);

    // 十字はデジタル化する（0.5超えたら±1、それ以外0）
    dpad_threshold_ = declare_parameter<double>("dpad_threshold", 0.5);

    publish_hz_      = declare_parameter<double>("publish_hz", 50.0);
    joy_timeout_sec_ = declare_parameter<double>("joy_timeout_sec", 0.25);

    default_command_type_ = declare_parameter<int>("default_command_type", 0); // 0=JOINT_JOG

    // 十字横→joint1、十字縦→joint4、L2/R2→joint6
    jog_joint1_name_ = declare_parameter<std::string>("jog_joint1_name", "arm_joint1");
    jog_joint3_name_ = declare_parameter<std::string>("jog_joint3_name", "arm_joint3");
    jog_joint4_name_ = declare_parameter<std::string>("jog_joint4_name", "arm_joint4");
    jog_joint6_name_ = declare_parameter<std::string>("jog_joint6_name", "arm_joint6");

    // joystick mapping
    axis_ly_      = declare_parameter<int>("axis_ly", 1); // 左スティック縦（上が+）
    axis_ry_      = declare_parameter<int>("axis_ry", 4); // 右スティック縦（上が+）
    axis_l2_      = declare_parameter<int>("axis_l2", 2); // L2（離す=1, 押すと0へ）
    axis_r2_      = declare_parameter<int>("axis_r2", 5); // R2（同上）
    axis_dpad_x_  = declare_parameter<int>("axis_dpad_x", 6); // 十字横（左が+ / 右が-）
    axis_dpad_y_  = declare_parameter<int>("axis_dpad_y", 7); // 十字縦（上が+）

    button_cross_ = declare_parameter<int>("button_cross", 0); // 十字ボタン（クロス）
    button_triangle_ = declare_parameter<int>("button_triangle", 2); // 十字ボタン（三角）

    invert_dpad_x_ = declare_parameter<bool>("invert_dpad_x", true);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::SensorDataQoS(), std::bind(&JoyServoTeleop::joyCb, this, _1));

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_, 10);
    joint_pub_ = create_publisher<control_msgs::msg::JointJog>(joint_topic_, 10);

    switch_client_ = create_client<moveit_msgs::srv::ServoCommandType>(switch_srv_);

    const double hz = std::max(1.0, publish_hz_);
    timer_period_ = 1.0 / hz;
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(timer_period_)),
      std::bind(&JoyServoTeleop::publishLoop, this));

    last_joy_time_ = now();
    current_cmd_type_.store(-1);

    // 起動時に自動セット
    requestCommandType(static_cast<int8_t>(default_command_type_));

    RCLCPP_INFO(get_logger(),
      "JoyServoTeleop ready. joint=%s twist=%s (auto switch)", joint_topic_.c_str(), twist_topic_.c_str());
  }

private:
  static double apply_deadzone(double v, double dz)
  {
    return (std::abs(v) < dz) ? 0.0 : v;
  }

  static double digitalize(double v, double thr)
  {
    if (v >  thr) return  1.0;
    if (v < -thr) return -1.0;
    return 0.0;
  }

  double axis(const sensor_msgs::msg::Joy& msg, int idx) const
  {
    if (idx < 0 || static_cast<size_t>(idx) >= msg.axes.size()) return 0.0;
    return msg.axes[static_cast<size_t>(idx)];
  }

  // safe button access (avoids out-of-range reads that can freeze the servo node)
  int button(const sensor_msgs::msg::Joy& msg, int idx) const
  {
    if (idx < 0 || static_cast<size_t>(idx) >= msg.buttons.size()) return 0;
    return msg.buttons[static_cast<size_t>(idx)];
  }

  double triggerPress01(const sensor_msgs::msg::Joy& msg, int idx) const
  {
    // 離す=1.0、押すと0へ → 押し込み量
    const double v = axis(msg, idx);
    double p = 1.0 - v;
    p = std::clamp(p, 0.0, 1.0);
    if (p < 0.5) p = 0.0;
    return p;
  }

  void requestCommandType(int8_t type)
  {
    if (current_cmd_type_.load() == type) return;

    if (!switch_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Servo switch service not ready: %s", switch_srv_.c_str());
      return;
    }

    if (switch_in_flight_.exchange(true)) return;

    auto req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    req->command_type = type;

    switch_client_->async_send_request(
      req,
      [this, type](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture fut) {
        bool ok = false;
        try { ok = fut.get()->success; } catch (...) { ok = false; }

        if (ok) {
          current_cmd_type_.store(type);
          RCLCPP_INFO(this->get_logger(), "Servo command_type set to %d", static_cast<int>(type));
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to set servo command_type=%d", static_cast<int>(type));
        }
        switch_in_flight_.store(false);
      }
    );
  }

  void joyCb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(joy_mtx_);
    last_joy_ = *msg;
    last_joy_time_ = now();
  }

  void publishLoop()
  {
    sensor_msgs::msg::Joy joy;
    rclcpp::Time stamp;

    {
      std::lock_guard<std::mutex> lk(joy_mtx_);
      joy = last_joy_;
      stamp = last_joy_time_;
    }

    const auto now_t = now();
    const bool stale = (now_t - stamp).seconds() > joy_timeout_sec_;
    if (stale) return;

    // ---- 十字（デジタル化）----
    double dpad_x = -axis(joy, axis_dpad_x_);
    double dpad_y = axis(joy, axis_dpad_y_);
    if (invert_dpad_x_) dpad_x = -dpad_x;
    dpad_x = digitalize(dpad_x, dpad_threshold_);
    dpad_y = digitalize(dpad_y, dpad_threshold_);

    const double joint1 = dpad_x * joint_scale_;
    const double joint4 = -dpad_y * joint_scale_;
    // triangle/cross are used for joint3; use safe accessor to avoid bad indices
    const double joint3 = (button(joy, button_cross_) - button(joy, button_triangle_)) * joint_scale_;

    // ---- joint6（R2-L2）----
    const double l2p = triggerPress01(joy, axis_l2_);
    const double r2p = triggerPress01(joy, axis_r2_);
    const double joint6 = std::clamp((r2p - l2p) * joint_scale_, -1.0, 1.0);

    // ---- Twist（平行移動のみ）----
    const double ly = apply_deadzone(axis(joy, axis_ly_), stick_deadzone_);
    const double ry = apply_deadzone(axis(joy, axis_ry_), stick_deadzone_);

    const double tx = std::clamp(ly * twist_scale_, -1.0, 1.0);
    const double tz = std::clamp(ry * twist_scale_, -1.0, 1.0);

    const bool joint1_on = (joint1 != 0.0);
    const bool joint3_on = (joint3 != 0.0);
    const bool joint4_on = (joint4 != 0.0);
    const bool joint6_on = (joint6 != 0.0);
    const bool joint_active = joint1_on ||  joint3_on || joint4_on || joint6_on;

    const bool twist_active = (tx != 0.0) || (tz != 0.0);

    // 優先：関節ジョグ > Twist
    if (joint_active) {
      requestCommandType(0); // JOINT_JOG

      control_msgs::msg::JointJog jog;
      jog.header.stamp = now_t;
      jog.header.frame_id = command_frame_;

      // 動いてる関節だけ送る（混線防止）
      if (joint1_on) { jog.joint_names.push_back(jog_joint1_name_); jog.velocities.push_back(joint1); }
      if (joint3_on) { jog.joint_names.push_back(jog_joint3_name_); jog.velocities.push_back(joint3); }
      if (joint4_on) { jog.joint_names.push_back(jog_joint4_name_); jog.velocities.push_back(joint4); }
      if (joint6_on) { jog.joint_names.push_back(jog_joint6_name_); jog.velocities.push_back(joint6); }

      jog.duration = timer_period_;
      joint_pub_->publish(jog);
      return;
    }

    if (twist_active) {
      requestCommandType(1); // TWIST

      geometry_msgs::msg::TwistStamped twist;
      twist.header.stamp = now_t;
      twist.header.frame_id = command_frame_;
      twist.twist.linear.x = tx;
      twist.twist.linear.z = tz;
      twist.twist.angular.x = 0.0;
      twist.twist.angular.y = 0.0;
      twist.twist.angular.z = 0.0;

      twist_pub_->publish(twist);
      return;
    }
  }

  // ---- members ----
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex joy_mtx_;
  sensor_msgs::msg::Joy last_joy_;
  rclcpp::Time last_joy_time_;

  std::string servo_node_name_;
  std::string planning_frame_;
  std::string command_frame_;
  std::string twist_topic_;
  std::string joint_topic_;
  std::string switch_srv_;

  std::string jog_joint1_name_;
  std::string jog_joint3_name_;
  std::string jog_joint4_name_;
  std::string jog_joint6_name_;

  int axis_ly_, axis_ry_;
  int axis_l2_, axis_r2_;
  int axis_dpad_x_, axis_dpad_y_;
  bool invert_dpad_x_;
  int button_cross_, button_triangle_;

  double twist_scale_;
  double joint_scale_;
  double stick_deadzone_;
  double trigger_deadzone_;
  double dpad_threshold_;
  double publish_hz_;
  double joy_timeout_sec_;
  int default_command_type_;
  double timer_period_;

  std::atomic<int8_t> current_cmd_type_;
  std::atomic<bool> switch_in_flight_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyServoTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}