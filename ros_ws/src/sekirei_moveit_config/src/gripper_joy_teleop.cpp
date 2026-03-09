#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include <stdexcept>

class GripperJoyTeleop : public rclcpp::Node
{
public:
  GripperJoyTeleop() : Node("gripper_joy_teleop")
  {
    // ---- parameters ----
    joy_topic_        = declare_parameter<std::string>("joy_topic", "/joy");
    command_topic_    = declare_parameter<std::string>("command_topic", "/gripper_controller/commands");
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");

    joint_names_      = declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{"gripper_joint_27", "gripper_joint_28"});

    //rad
    open_targets_     = declare_parameter<std::vector<double>>("open_targets",  std::vector<double>{5.13, 2.34});
    close_targets_    = declare_parameter<std::vector<double>>("close_targets", std::vector<double>{4.26, 3.28});

    close_button_     = declare_parameter<int>("close_button", 0);
    open_button_      = declare_parameter<int>("open_button", 1);

    use_grasp_detection_ = declare_parameter<bool>("use_grasp_detection", true);
    effort_threshold_    = declare_parameter<double>("effort_threshold", 105.0);
    consecutive_count_   = declare_parameter<int>("consecutive_count", 3);
    loop_ms_             = declare_parameter<int>("loop_ms", 50);

    if (joint_names_.size() != 2 || open_targets_.size() != 2 || close_targets_.size() != 2) {
      RCLCPP_FATAL(get_logger(), "joint_names/open_targets/close_targets must be size 2.");
      throw std::runtime_error("Invalid parameter size");
    }

    // ---- pub/sub ----
    cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::QoS(10),
      std::bind(&GripperJoyTeleop::onJoy, this, std::placeholders::_1));

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::QoS(10),
      std::bind(&GripperJoyTeleop::onJointState, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(loop_ms_),
      std::bind(&GripperJoyTeleop::controlLoop, this));

    RCLCPP_INFO(get_logger(), "GripperJoyTeleop started. pub: %s  joy: %s",
                command_topic_.c_str(), joy_topic_.c_str());
  }

private:
  // topics/params
  std::string joy_topic_;
  std::string command_topic_;
  std::string joint_states_topic_;
  std::vector<std::string> joint_names_;
  std::vector<double> open_targets_;
  std::vector<double> close_targets_;
  int close_button_{0};
  int open_button_{1};

  bool use_grasp_detection_{true};
  double effort_threshold_{105.0};
  int consecutive_count_{3};
  int loop_ms_{50};

  // state
  std::vector<int> last_buttons_;
  bool closing_{false};
  int over_count_{0};

  bool have_pos_{false};
  bool have_eff_{false};
  double last_pos_[2]{0.0, 0.0};
  double last_eff_[2]{0.0, 0.0};

  // ros
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  static bool risingEdge(const std::vector<int>& prev, const std::vector<int>& cur, int idx)
  {
    if (idx < 0) return false;
    if ((int)cur.size() <= idx) return false;
    int p = ((int)prev.size() > idx) ? prev[idx] : 0;
    return (p == 0 && cur[idx] != 0);
  }

  void publishTargets(const std::vector<double>& targets)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = targets;  // controller側の joints 順序と targets の順序を一致させる
    cmd_pub_->publish(msg);
  }

  void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const auto& b = msg->buttons;

    // rising edge only
    bool do_close = risingEdge(last_buttons_, b, close_button_);
    bool do_open  = risingEdge(last_buttons_, b, open_button_);
    last_buttons_ = b;

    if (do_close) {
      publishTargets(close_targets_);
      closing_ = true;
      over_count_ = 0;
      RCLCPP_INFO(get_logger(), "Close command sent.");
    } else if (do_open) {
      publishTargets(open_targets_);
      closing_ = false;
      over_count_ = 0;
      RCLCPP_INFO(get_logger(), "Open command sent.");
    }
  }

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // joint_names_ の2関節の index を探して保持
    auto idx_of = [&](const std::string& name) -> int {
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == name) return (int)i;
      }
      return -1;
    };

    int i0 = idx_of(joint_names_[0]);
    int i1 = idx_of(joint_names_[1]);
    if (i0 < 0 || i1 < 0) return;

    if ((int)msg->position.size() > std::max(i0, i1)) {
      last_pos_[0] = msg->position[i0];
      last_pos_[1] = msg->position[i1];
      have_pos_ = true;
    }

    if ((int)msg->effort.size() > std::max(i0, i1)) {
      last_eff_[0] = msg->effort[i0];
      last_eff_[1] = msg->effort[i1];
      have_eff_ = true;
    }
  }

  void controlLoop()
  {
    if (!use_grasp_detection_) return;
    if (!closing_) return;

    // effort が無い環境なら把持検出できないので何もしない
    if (!have_eff_) return;

    bool over = (std::abs(last_eff_[0]) > effort_threshold_) ||
                (std::abs(last_eff_[1]) > effort_threshold_);

    over_count_ = over ? (over_count_ + 1) : 0;

    if (over_count_ >= consecutive_count_) {
      // 「その時点の位置」を再送して保持（stopAtCurrentPosition 相当）
      if (have_pos_) {
        std::vector<double> hold{last_pos_[0], last_pos_[1]};
        publishTargets(hold);
      }
      closing_ = false;
      over_count_ = 0;
      RCLCPP_INFO(get_logger(), "Grasp detected -> hold position.");
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperJoyTeleop>());
  rclcpp::shutdown();
  return 0;
}