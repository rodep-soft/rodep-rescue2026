#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

// ===== Control Table (XM series) =====
#define ADDR_OPERATING_MODE            11
#define ADDR_CURRENT_LIMIT             38
#define ADDR_TORQUE_ENABLE             64
#define ADDR_SHUTDOWN                  63
#define ADDR_HARDWARE_ERROR_STATUS     70

#define ADDR_GOAL_POSITION            116
#define ADDR_PRESENT_CURRENT          126   // CURRENT (2 byte, signed, 2.69 mA/unit)
#define ADDR_PRESENT_POSITION         132

#define ADDR_PRESENT_INPUT_VOLTAGE    144
#define ADDR_PRESENT_TEMPERATURE      146



#define PROTOCOL_VERSION 2.0

class DynamixelController : public rclcpp::Node {
public:
  DynamixelController() : Node("dynamixel_control_node") {
    RCLCPP_INFO(get_logger(), "Run dynamixel control node");

    declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
    declare_parameter<int>("baudrate", 1000000);
    declare_parameter<int>("qos_depth", 10);

    declare_parameter<int>("loop_ms", 20);
    declare_parameter<int>("step_pos", 50); //アームを閉じるときの1ステップあたりの移動量
    declare_parameter<int>("epsilon_pos", 30); //許容誤差
    declare_parameter<int>("stall_pos_delta", 100); //スタック検出のための位置変化量閾値
    declare_parameter<int>("stall_consecutive", 4); //スタック検出のための連続カウント閾値
    declare_parameter<int>("close_timeout_ms", 3000); //閉じる動作のタイムアウト時間

    get_parameter("device_name", device_name_);
    get_parameter("baudrate", baudrate_);
    get_parameter("qos_depth", qos_depth_);
    get_parameter("loop_ms", loop_ms_);
    get_parameter("step_pos", step_pos_);
    get_parameter("epsilon_pos", epsilon_pos_);
    get_parameter("stall_pos_delta", stall_pos_delta_);
    get_parameter("stall_consecutive", stall_consecutive_);
    get_parameter("close_timeout_ms", close_timeout_ms_);

    port_handler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!port_handler_->openPort()) {
      RCLCPP_ERROR(get_logger(), "Failed to open port");
      rclcpp::shutdown();
      return;
    }

    port_handler_->setBaudRate(baudrate_);

    sync_write_goal_pos_ = std::make_unique<dynamixel::GroupSyncWrite>(
      port_handler_, packet_handler_, ADDR_GOAL_POSITION, 4);

    for (uint8_t id : ids_) setupDynamixel(id);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", qos_depth_,
      [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);

        if (msg->buttons.size() > 1 && msg->buttons[1]) startCloseLocked();
        if (msg->buttons.size() > 0 && msg->buttons[0]) startOpenLocked();

      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(loop_ms_),
      [this]() { onTimer(); });

    RCLCPP_INFO(get_logger(), "Ready. O=Close, X=Open");
  }

private:
  enum class Mode { IDLE, CLOSING, HOLDING };

  const std::array<uint8_t, 2> ids_{{27, 28}};
  const std::array<int, 2> gripper_close_{{2856, 2134}};
  const std::array<int, 2> gripper_open_ {{3549, 1376}};

  dynamixel::PortHandler* port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_goal_pos_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string device_name_;
  int baudrate_{};
  int qos_depth_{};

  int loop_ms_{};
  int step_pos_{};
  int epsilon_pos_{};
  int stall_pos_delta_{};
  int stall_consecutive_{};
  int close_timeout_ms_{};

  std::mutex mtx_;
  Mode mode_{Mode::IDLE};

  std::array<int, 2> current_pos_{};
  std::array<int, 2> prev_pos_{};
  std::array<int, 2> target_pos_{};
  std::array<int, 2> hold_pos_{};

  int stall_count_{0};
  rclcpp::Time close_start_time_;
  std::vector<int32_t> last_buttons_;

  // ===== Low-level IO =====
  bool write1(uint8_t id, uint16_t addr, uint8_t val) {
    uint8_t err;
    return packet_handler_->write1ByteTxRx(
      port_handler_, id, addr, val, &err) == COMM_SUCCESS && err == 0;
  }

  bool read2(uint8_t id, uint16_t addr, uint16_t& out) {
    uint8_t err;
    return packet_handler_->read2ByteTxRx(
      port_handler_, id, addr, &out, &err) == COMM_SUCCESS && err == 0;
  }

  bool read4(uint8_t id, uint16_t addr, uint32_t& out) {
    uint8_t err;
    return packet_handler_->read4ByteTxRx(
      port_handler_, id, addr, &out, &err) == COMM_SUCCESS && err == 0;
  }

  // ===== Setup =====
  void setupDynamixel(uint8_t id) {
    write1(id, ADDR_TORQUE_ENABLE, 0);
    write1(id, ADDR_OPERATING_MODE, 5); //position current control mode
    write1(id, ADDR_CURRENT_LIMIT, 119); //360mA
    write1(id, ADDR_TORQUE_ENABLE, 1);
  }

  // ===== Read helpers =====
  void readPositions() {
    for (int i = 0; i < 2; ++i) {
      uint32_t p;
      if (read4(ids_[i], ADDR_PRESENT_POSITION, p))
        current_pos_[i] = static_cast<int>(p);
    }
  }

  void readCurrentCurrent() {
    for (uint8_t id : ids_) {
      double current_mA[2] = {0.0, 0.0};
      uint16_t raw = 0;
      if (!read2(id, ADDR_PRESENT_CURRENT, raw)) continue;

      int16_t signed_raw = static_cast<int16_t>(raw);
      current_mA[id] = signed_raw * 2.69;
      
      RCLCPP_INFO(
        get_logger(),
        "[ID:%d] PresentCurrent = %+7.1f mA (raw=%d)",
        id, current_mA[id], signed_raw
      );

      // 電流リミット（絶対値）
      if (std::abs(current_mA[0]) > 400.0 && std::abs(current_mA[1]) > 400.0) {
        RCLCPP_WARN(
          get_logger(),
          "[ID:%d] Over current detected (|I|=%.1f mA) -> HOLD",
          id, std::abs(current_mA[id])
        );
        holdHereLocked();
        return;
      }
    }
  }

//   void readAndPrintCurrentLocked() {
//   for (uint8_t id : ids_) {
//     uint16_t raw = 0;
//     if (!read2(id, ADDR_PRESENT_CURRENT, raw)) continue;

//     int16_t signed_raw = static_cast<int16_t>(raw);
//     double current_mA = signed_raw * 2.69;

//     RCLCPP_INFO(
//       get_logger(),
//       "[ID:%d] PresentCurrent = %+7.1f mA (raw=%d)",
//       id, current_mA, signed_raw
//     );

//     // 電流リミット（絶対値）
//     if (std::abs(current_mA) > 400.0) {
//       RCLCPP_WARN(
//         get_logger(),
//         "[ID:%d] Over current detected (|I|=%.1f mA) -> HOLD",
//         id, std::abs(current_mA)
//       );
//       holdHereLocked();
//       return;
//     }
//   }
// }


  void syncWriteGoalPositions(const std::array<int, 2>& goals) {
    sync_write_goal_pos_->clearParam();

    for (int i = 0; i < 2; ++i) {
      uint32_t v = goals[i];
      uint8_t p[4] = {
        DXL_LOBYTE(DXL_LOWORD(v)),
        DXL_HIBYTE(DXL_LOWORD(v)),
        DXL_LOBYTE(DXL_HIWORD(v)),
        DXL_HIBYTE(DXL_HIWORD(v))
      };
      sync_write_goal_pos_->addParam(ids_[i], p);
    }
    sync_write_goal_pos_->txPacket();
  }

  // ===== Commands =====
  void startOpenLocked() {
    target_pos_ = gripper_open_;
    syncWriteGoalPositions(target_pos_);
    mode_ = Mode::IDLE;//開く動作は途中で止めない
  }

  void startCloseLocked() {
    mode_ = Mode::CLOSING; 
    target_pos_ = gripper_close_;
    //stall_count_ = 0;
    //close_start_time_ = now();
    //readPositions();
    //prev_pos_ = current_pos_;
  }

  // ===== Timer =====
  void onTimer() {
    std::lock_guard<std::mutex> lk(mtx_);

    
    if (mode_ != Mode::CLOSING) return;

    readPositions();

    // int dp0 = std::abs(current_pos_[0] - prev_pos_[0]);
    // int dp1 = std::abs(current_pos_[1] - prev_pos_[1]);

    // if (dp0 <= stall_pos_delta_ || dp1 <= stall_pos_delta_)
    //   stall_count_++;
    // else
    //   stall_count_ = 0;
    readCurrentCurrent();

    // prev_pos_ = current_pos_;

    // if (stall_count_ >= stall_consecutive_) {
    //   holdHereLocked();
    //   return;
    // }

    std::array<int, 2> next = current_pos_;
    for (int i = 0; i < 2; ++i) {
      int diff = target_pos_[i] - current_pos_[i];
      next[i] += std::clamp(diff, -step_pos_, step_pos_);
    }
    syncWriteGoalPositions(next);
  }

  void holdHereLocked() {
    readPositions();
    hold_pos_ = current_pos_;
    syncWriteGoalPositions(hold_pos_);
    mode_ = Mode::HOLDING;
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamixelController>());
  rclcpp::shutdown();
  return 0;
}
