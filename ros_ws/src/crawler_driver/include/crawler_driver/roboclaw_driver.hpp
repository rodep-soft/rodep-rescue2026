#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

// rclcpp types
#include <rclcpp/rclcpp.hpp>

namespace crawler_driver::roboclaw {

// =====================
// 定数定義
// =====================

// RoboclawMotionStudioから確認/変更ができる
inline constexpr uint8_t DEFAULT_ADDRESS = 0x80;

// PC<->Roboclawの通信速度
inline constexpr uint32_t BAUD_RATE = 38400;
inline constexpr auto TIMEOUT = std::chrono::milliseconds(1000);

// 応答コード
inline constexpr uint8_t ACK = 0xFF;

// コマンドID
enum class Command : uint8_t {
  M1_TEST = 6,
  M2_TEST = 7,
  RESET_ENCODERS = 20,
  M1_SET_PID = 28,
  M2_SET_PID = 29,
  M1_VELOCITY = 35,
  M2_VELOCITY = 36,
  M1_ENCODER = 92,
  M2_ENCODER = 93
};

// モーターの識別
enum class Motor : uint8_t { M1 = 1, M2 = 2 };

// エンコーダ定数
enum class EncoderQPPS : int32_t { M1 = 53250, M2 = 50062 };

// PIDゲイン
struct PIDConstants {
  float p{0.0f};
  float i{0.0f};
  float d{0.0f};
  int32_t qpps{0};
};

// エンコーダ値
struct EncoderValue {
  int32_t counts{0};
  uint8_t status{0};
  bool is_valid() const { return status == ACK; }
};

// =====================
// RoboclawDriver クラス
// =====================

class RoboclawDriver {
public:
  using Callback = std::function<void(bool success)>;
  using EncoderCallback = std::function<void(std::optional<EncoderValue>)>;
  using ByteBuffer = std::vector<uint8_t>;

  explicit RoboclawDriver(std::string port, uint8_t address = DEFAULT_ADDRESS);
  ~RoboclawDriver();

  // コピー・ムーブ禁止
  RoboclawDriver(const RoboclawDriver&) = delete;
  RoboclawDriver& operator=(const RoboclawDriver&) = delete;
  RoboclawDriver(RoboclawDriver&&) = delete;
  RoboclawDriver& operator=(RoboclawDriver&&) = delete;

  // モーター制御
  void setVelocity(Motor motor, double counts_per_sec, Callback callback);
  void setPID(Motor motor, const PIDConstants& pid, Callback callback);
  void resetEncoders(Callback callback);
  void readEncoder(Motor motor, EncoderCallback callback);

private:
  const uint8_t address_;
  const std::string port_;
  const rclcpp::Logger logger_;

  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_port_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
  std::thread io_thread_;

  void initializeSerialPort();
  void sendCommand(ByteBuffer command, Callback callback);
  void sendCommandWithResponse(ByteBuffer command, size_t response_size,
                               std::function<void(std::optional<ByteBuffer>)> callback);

  // コマンド生成ヘルパー
  static Command getVelocityCommand(Motor motor);
  static Command getPIDCommand(Motor motor);
  static Command getEncoderCommand(Motor motor);

  // データシリアライゼーション
  static constexpr uint16_t calculateCRC(std::span<const uint8_t> data);
  static void appendCRC(ByteBuffer& data);
  static void appendInt32BE(ByteBuffer& data, int32_t value);
  static void appendFloat32BE(ByteBuffer& data, float value);
  static int32_t extractInt32BE(std::span<const uint8_t> data, size_t offset);

  ByteBuffer buildCommand(Command cmd) const;
  ByteBuffer buildCommand(Command cmd, int32_t value) const;
  ByteBuffer buildCommand(Command cmd, const PIDConstants& pid) const;
};

}  // namespace crawler_driver::roboclaw
