#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

// ===== Control Table (XM/X series想定) =====
#define ADDR_OPERATING_MODE     11
#define ADDR_CURRENT_LIM        38
#define ADDR_TORQUE_ENABLE      64
#define ADDR_GOAL_POSITION     116
#define ADDR_PRESENT_POSITION  132

#define PROTOCOL_VERSION 2.0

class DynamixelController : public rclcpp::Node {
public:
  DynamixelController() : Node("dynamixel_control_node") {
    RCLCPP_INFO(get_logger(), "Run dynamixel control node");

    // ===== ROS params =====
    declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
    declare_parameter<int>("baudrate", 1000000);
    declare_parameter<int>("qos_depth", 10);

    // 制御パラメータ（位置ベース停止）
    declare_parameter<int>("loop_ms", 20);
    declare_parameter<int>("step_pos", 20);          // 1周期で進めるtick（小さいほど優しい）
    declare_parameter<int>("epsilon_pos", 30);       // 目標到達判定
    declare_parameter<int>("stall_pos_delta", 100);   // 位置変化がこれ以下なら「動いてない」
    declare_parameter<int>("stall_consecutive", 4);  // stall連続回数で停止
    declare_parameter<int>("close_timeout_ms", 3000);// 閉じ動作タイムアウト
    declare_parameter<int>("current_limit", 160);    // 保険：最大出力上限（判定には使わない）

    get_parameter("device_name", device_name_);
    get_parameter("baudrate", baudrate_);
    get_parameter("qos_depth", qos_depth_);

    get_parameter("loop_ms", loop_ms_);
    get_parameter("step_pos", step_pos_);
    get_parameter("epsilon_pos", epsilon_pos_);
    get_parameter("stall_pos_delta", stall_pos_delta_);
    get_parameter("stall_consecutive", stall_consecutive_);
    get_parameter("close_timeout_ms", close_timeout_ms_);
    get_parameter("current_limit", current_limit_);

    // ===== Dynamixel SDK =====
    port_handler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    const auto qos_profile =
      rclcpp::QoS(rclcpp::KeepLast(qos_depth_)).reliable().durability_volatile();

    if (!port_handler_->openPort()) {
      RCLCPP_ERROR(get_logger(), "Failed to open the port: %s", device_name_.c_str());
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Succeeded to open the port: %s", device_name_.c_str());

    if (!port_handler_->setBaudRate(baudrate_)) {
      RCLCPP_ERROR(get_logger(), "Failed to set baudrate: %d", baudrate_);
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Succeeded to set baudrate: %d", baudrate_);

    // SyncWrite（両方同時にGOAL_POSITIONを送る）
    //4はデータ長(byte)
    sync_write_goal_pos_ = std::make_unique<dynamixel::GroupSyncWrite>(
      port_handler_, packet_handler_, ADDR_GOAL_POSITION, 4);

    // 各Dynamixel初期設定
    for (uint8_t id : ids_) setupDynamixel(id);

    // Joy subscriber（ボタンの立ち上がりでコマンド確定）
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", qos_profile,
      [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (!msg) return;
        std::lock_guard<std::mutex> lk(mtx_);

        // buttonsサイズ変動対策
        if (last_buttons_.size() != msg->buttons.size()) {
          last_buttons_.assign(msg->buttons.size(), 0);
        }

        auto rising = [&](size_t idx) -> bool {
          if (idx >= msg->buttons.size()) return false;
          return (msg->buttons[idx] != 0) && (last_buttons_[idx] == 0);
        };

        // あなたの割当: O=buttons[1] close / X=buttons[0] open
        if (rising(1)) {
          startCloseLocked();
        } else if (rising(0)) {
          startOpenLocked();
        }

        // 更新
        for (size_t i = 0; i < msg->buttons.size(); ++i) {
          last_buttons_[i] = msg->buttons[i];
        }
      });

    // 非同期：timerが一定周期で「閉じる処理の1ステップ」を進める
    timer_ = create_wall_timer(
      std::chrono::milliseconds(loop_ms_),
      [this]() { onTimer(); });

    RCLCPP_INFO(get_logger(), "Ready. O=Close, X=Open");
  }

  ~DynamixelController() override {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!port_handler_ || !packet_handler_) return;

    for (uint8_t id : ids_) {
      uint8_t dxl_error = 0;
      int r = packet_handler_->write1ByteTxRx(
        port_handler_, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);

      if (r != COMM_SUCCESS) {
        RCLCPP_ERROR(get_logger(), "[ID:%d] %s", id, packet_handler_->getTxRxResult(r));
      } else if (dxl_error != 0) {
        RCLCPP_ERROR(get_logger(), "[ID:%d] %s", id, packet_handler_->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(get_logger(), "Disabled torque [ID:%d]", id);
      }
    }
    port_handler_->closePort();
    RCLCPP_INFO(get_logger(), "Closed the port");
  }

private:
  enum class Mode { IDLE, CLOSING, HOLDING };

  // ===== Gripper IDs and targets =====
  const std::array<uint8_t, 2> ids_{{27, 28}};
  const std::array<int, 2> gripper_close_{{2856, 2134}};
  const std::array<int, 2> gripper_open_ {{3549, 1376}};

  // ===== Dynamixel SDK handlers =====
  dynamixel::PortHandler* port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_goal_pos_;

  // ===== ROS =====
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ===== Parameters =====
  int qos_depth_{10};
  int baudrate_{1000000};
  std::string device_name_{"/dev/ttyUSB0"};

  int loop_ms_{20};
  int step_pos_{20};
  int epsilon_pos_{30};
  int stall_pos_delta_{100};
  int stall_consecutive_{8};
  int close_timeout_ms_{3000};
  int current_limit_{160};

  // ===== State =====
  std::mutex mtx_;
  Mode mode_{Mode::IDLE};

  std::array<int, 2> current_pos_{{0, 0}};
  std::array<int, 2> prev_pos_{{0, 0}};
  std::array<int, 2> target_pos_{{0, 0}};
  std::array<int, 2> hold_pos_{{0, 0}};

  int stall_count_{0};
  rclcpp::Time close_start_time_;

  std::vector<int32_t> last_buttons_;

private:
  //write関数は1バイト、2バイト、4バイト用を用意
  bool write1(uint8_t id, uint16_t addr, uint8_t val) {
    uint8_t dxl_error = 0;
    int r = packet_handler_->write1ByteTxRx(port_handler_, id, addr, val, &dxl_error);
    if (r != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "[ID:%d] %s", id, packet_handler_->getTxRxResult(r));
      return false;
    }
    if (dxl_error != 0) {
      RCLCPP_ERROR(get_logger(), "[ID:%d] %s", id, packet_handler_->getRxPacketError(dxl_error));
      return false;
    }
    return true;
  }

  bool write2(uint8_t id, uint16_t addr, uint16_t val) {
    uint8_t dxl_error = 0;
    int r = packet_handler_->write2ByteTxRx(port_handler_, id, addr, val, &dxl_error);
    if (r != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "[ID:%d] %s", id, packet_handler_->getTxRxResult(r));
      return false;
    }
    if (dxl_error != 0) {
      RCLCPP_ERROR(get_logger(), "[ID:%d] %s", id, packet_handler_->getRxPacketError(dxl_error));
      return false;
    }
    return true;
  }

  bool read4(uint8_t id, uint16_t addr, uint32_t& out) {
    uint8_t dxl_error = 0;
    int r = packet_handler_->read4ByteTxRx(port_handler_, id, addr, &out, &dxl_error);
    if (r != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "[ID:%d] %s", id, packet_handler_->getTxRxResult(r));
      return false;
    }
    if (dxl_error != 0) {
      RCLCPP_ERROR(get_logger(), "[ID:%d] %s", id, packet_handler_->getRxPacketError(dxl_error));
      return false;
    }
    return true;
  }

  // ---------- Setup ----------
  void setupDynamixel(uint8_t id) {
    // Torque OFF
    write1(id, ADDR_TORQUE_ENABLE, 0);

    // Operating Mode: Position Control (3)
    write1(id, ADDR_OPERATING_MODE, 3);

    // 保険：最大出力上限（判定には使わない）
    write2(id, ADDR_CURRENT_LIM, static_cast<uint16_t>(current_limit_));

    // Torque ON
    write1(id, ADDR_TORQUE_ENABLE, 1);

    RCLCPP_INFO(get_logger(), "[ID:%d] setup done (mode=3, current_lim=%d)", id, current_limit_);
  }

  // ---------- Read/Write ----------
  void readPositions() {
    for (int i = 0; i < 2; ++i) {
      uint32_t p = 0;
      if (read4(ids_[i], ADDR_PRESENT_POSITION, p)) {
        current_pos_[i] = static_cast<int>(p);
      }
    }
  }

  void syncWriteGoalPositions(const std::array<int, 2>& goals) {
    if (!sync_write_goal_pos_) return;

    sync_write_goal_pos_->clearParam();

    for (int i = 0; i < 2; ++i) {
      const uint32_t v = static_cast<uint32_t>(goals[i]);
      uint8_t param[4] = {
        DXL_LOBYTE(DXL_LOWORD(v)),
        DXL_HIBYTE(DXL_LOWORD(v)),
        DXL_LOBYTE(DXL_HIWORD(v)),
        DXL_HIBYTE(DXL_HIWORD(v))
      };
      bool ok = sync_write_goal_pos_->addParam(ids_[i], param);
      if (!ok) {
        RCLCPP_ERROR(get_logger(), "GroupSyncWrite addParam failed [ID:%d]", ids_[i]);
      }
    }

    int r = sync_write_goal_pos_->txPacket();
    if (r != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "GroupSyncWrite txPacket failed: %s",
                   packet_handler_->getTxRxResult(r));
    }
    sync_write_goal_pos_->clearParam();
  }

  // ---------- Command start (locked) ----------
  void startOpenLocked() {
    // OPENは一発で送ってIDLEへ
    target_pos_ = gripper_open_;
    syncWriteGoalPositions(target_pos_);
    mode_ = Mode::IDLE;
    stall_count_ = 0;
    RCLCPP_INFO(get_logger(), "Command: OPEN");
  }

  void startCloseLocked() {
    mode_ = Mode::CLOSING;
    target_pos_ = gripper_close_;
    stall_count_ = 0;
    close_start_time_ = now();

    // 初期prev_pos
    readPositions();
    prev_pos_ = current_pos_;

    RCLCPP_INFO(get_logger(), "Command: CLOSE (async, stop on stall)");
  }

  // ---------- Timer loop ----------
  void onTimer() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (mode_ == Mode::IDLE) return;

    if (mode_ == Mode::CLOSING) {
      stepClosingLocked();
    } else if (mode_ == Mode::HOLDING) {
      // HOLDは、保持目標を送ってあるので基本何もしない
    }
  }

  void stepClosingLocked() {
    // タイムアウト
    const auto elapsed_ms = (now() - close_start_time_).nanoseconds() / 1000000;
    if (elapsed_ms > close_timeout_ms_) {
      RCLCPP_WARN(get_logger(), "Close timeout -> HOLD");
      holdHereLocked();
      return;
    }

    readPositions();

    // 目標到達判定
    const bool reached =
      (std::abs(current_pos_[0] - target_pos_[0]) <= epsilon_pos_) &&
      (std::abs(current_pos_[1] - target_pos_[1]) <= epsilon_pos_);

    if (reached) {
      RCLCPP_INFO(get_logger(), "Reached close target -> HOLD");
      holdHereLocked();
      return;
    }

    // stall 判定：まだ動かす必要があるのに位置がほぼ変わらない
    const int dp0 = std::abs(current_pos_[0] - prev_pos_[0]);
    const int dp1 = std::abs(current_pos_[1] - prev_pos_[1]);

    // 安全側：どちらかが詰まったら止める（片側だけ物体に当たるケース）
    const bool stalled_any = (dp0 <= stall_pos_delta_) || (dp1 <= stall_pos_delta_);

    if (stalled_any) stall_count_++;
    else stall_count_ = 0;

    prev_pos_ = current_pos_;

    if (stall_count_ >= stall_consecutive_) {
      RCLCPP_INFO(get_logger(), "Stall detected -> HOLD (dp0=%d, dp1=%d, cnt=%d)",
                  dp0, dp1, stall_count_);
      holdHereLocked();
      return;
    }

    // 次のgoal：両方同時に少しずつ閉じる（ターゲット超えない）
    std::array<int, 2> next_goal = current_pos_;

    for (int i = 0; i < 2; ++i) {
      const int curp = current_pos_[i];
      const int tgtp = target_pos_[i];
      const int diff = tgtp - curp;

      if (std::abs(diff) <= epsilon_pos_) {
        next_goal[i] = curp;
        continue;
      }

      int delta = std::clamp(diff, -step_pos_, step_pos_);
      int g = curp + delta;

      if (diff > 0) g = std::min(g, tgtp);
      else          g = std::max(g, tgtp);

      next_goal[i] = g;
    }

    // ★両方同時送信
    syncWriteGoalPositions(next_goal);
  }

  void holdHereLocked() {
    // 今の位置で保持（＝それ以上閉じない）
    readPositions();
    hold_pos_ = current_pos_;
    syncWriteGoalPositions(hold_pos_);
    mode_ = Mode::HOLDING;

    RCLCPP_INFO(get_logger(), "HOLD at pos=(%d,%d)", hold_pos_[0], hold_pos_[1]);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamixelController>());
  rclcpp::shutdown();
  return 0;
}
