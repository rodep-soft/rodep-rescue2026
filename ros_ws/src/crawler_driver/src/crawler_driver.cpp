#include <rclcpp/rclcpp.hpp>

#include <memory>

#include <vector>
#include <thread>
#include <functional>
#include <string>
#include <stdexcept>
#include <cstdint>
#include <cstring>
#include <cstdlib>

#include <boost/asio.hpp>
#include <custom_interfaces/msg/crawler_velocity.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <chrono>


namespace {

// RoboclawのアドレスはMotionStudioから確認、変更することができる
// デフォルト値は0x80
constexpr uint8_t ROBOCLAW_ADDRESS = 0x80;

enum class Command : uint8_t {
  M1_SET_PID = 28,
  M2_SET_PID = 29,
  M1_VELOCITY = 35,
  M2_VELOCITY = 36,
  M1_ENCODER = 92,
  M2_ENCODER = 93,
  RESET_ENCODERS = 20,
};

constexpr int32_t SERIAL_BAUD_RATE = 38400;
constexpr auto SERIAL_TIMEOUT_MS = std::chrono::milliseconds(1000);
constexpr int32_t M1_QPPS = 53250;
constexpr int32_t M2_QPPS = 50062;
} // namespace

// RoboclawDriver Class
class RoboclawDriver {
public:
  explicit RoboclawDriver(const std::string& port)
    : io_(), serial_(io_, port), work_(boost::asio::make_work_guard(io_)) {
    try {
      configureSerialPort();
      if (!serial_.is_open()) {
        throw std::runtime_error(std::string("Failed to open serial port: ") + port);
      }
      io_thread_ = std::thread([this]() { io_.run(); });
    } catch (const std::exception& e) {
      throw std::runtime_error(std::string("Failed to configure serial port: ") + e.what());
    }
  }

  ~RoboclawDriver() {
    // Reset work guard so io_context can stop when no pending handlers
    work_.reset();
    io_.stop();
    if (io_thread_.joinable()) {
      io_thread_.join();
    }
  }

  void asyncSendRoboclawCommand(const std::vector<uint8_t>& data, std::function<void(bool)> callback) {
    auto write_buf = std::make_shared<std::vector<uint8_t>>(data);
    boost::asio::async_write(
        serial_, boost::asio::buffer(*write_buf),
        [this, write_buf, callback](const boost::system::error_code& ec, std::size_t /*bytes_transferred*/) {
          if (ec) {
            RCLCPP_ERROR(rclcpp::get_logger("RoboclawDriver"), "Serial Write Error: %s",
                         ec.message().c_str());
            callback(false);
            return;
          }

          auto read_buf = std::make_shared<std::array<uint8_t, 1>>();
          boost::asio::async_read(serial_, boost::asio::buffer(*read_buf),
                                  [this, read_buf, callback](const boost::system::error_code& ec,
                                                             std::size_t /*bytes_transferred*/) {
                                    if (ec) {
                                      RCLCPP_ERROR(rclcpp::get_logger("RoboclawDriver"),
                                                   "Serial Read Error: %s", ec.message().c_str());
                                      callback(false);
                                      return;
                                    }

                                    uint8_t response = (*read_buf)[0];
                                    RCLCPP_DEBUG(rclcpp::get_logger("RoboclawDriver"),
                                                 "Received Response: 0x%02X", response);
                                    callback(true);
                                  });
        });
  }

  bool setMotorVelocity(int command, double /*int*/ counts_per_sec,
                        std::function<void(bool)> callback) {
    std::vector<uint8_t> data = {static_cast<uint8_t>(ROBOCLAW_ADDRESS), static_cast<uint8_t>(command)};
    // Convert counts_per_sec to int32 safely (device expects integer counts/sec)
    long rounded_counts = std::lround(counts_per_sec);
    long clamped_counts = std::min<long>(std::numeric_limits<int32_t>::max(), std::max<long>(std::numeric_limits<int32_t>::min(), rounded_counts));
    appendInt32(data, static_cast<int>(clamped_counts));
    appendCRC(data);
    asyncSendRoboclawCommand(data, callback);
    return true;
  }

  bool setPIDConstants(int command, float K_p, float K_i, float K_d, int qpps,
                       std::function<void(bool)> callback) {
    std::vector<uint8_t> data = {static_cast<uint8_t>(ROBOCLAW_ADDRESS), static_cast<uint8_t>(command)};
    appendFloat32(data, K_d);
    appendFloat32(data, K_p);
    appendFloat32(data, K_i);
    appendInt32(data, qpps);
    appendCRC(data);
    asyncSendRoboclawCommand(data, callback);
    return true;
  }

  bool resetEncoders(std::function<void(bool)> callback) {
    std::vector<uint8_t> data = {static_cast<uint8_t>(ROBOCLAW_ADDRESS), static_cast<uint8_t>(Command::RESET_ENCODERS)};
    appendCRC(data);
    asyncSendRoboclawCommand(data, callback);
    return true;
  }

private:
  boost::asio::io_context io_;
  boost::asio::serial_port serial_;
  std::thread io_thread_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_;

  void configureSerialPort() {
    serial_.set_option(boost::asio::serial_port_base::baud_rate(SERIAL_BAUD_RATE));
    serial_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  }

  uint16_t calculateCRC(const std::vector<uint8_t>& data) {
    uint16_t crc = 0;
    for (auto byte : data) {
      crc ^= static_cast<uint16_t>(byte) << 8;
      for (int i = 0; i < 8; i++) {
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
      }
    }
    return crc;
  }

  void appendCRC(std::vector<uint8_t>& data) {
    uint16_t crc = calculateCRC(data);
    data.push_back(static_cast<uint8_t>(crc >> 8));
    data.push_back(static_cast<uint8_t>(crc & 0xFF));
  }

  void appendInt32(std::vector<uint8_t>& data, int value) {
    for (int i = 3; i >= 0; --i) {
      data.push_back(static_cast<uint8_t>((value >> (8 * i)) & 0xFF));
    }
  }

  void appendFloat32(std::vector<uint8_t>& data, float value) {
    // Reinterpret float bits safely into uint32_t
    uint32_t as_int = 0;
    static_assert(sizeof(float) == sizeof(uint32_t), "Unexpected float size");
    std::memcpy(&as_int, &value, sizeof(float));
    for (int i = 3; i >= 0; --i) {
      data.push_back(static_cast<uint8_t>((as_int >> (8 * i)) & 0xFF));
    }
  }
};

// ROS2 Driver Node
class CrawlerDriver : public rclcpp::Node {
public:
  CrawlerDriver() : Node("crawler_driver"), roboclaw_("/dev/roboclaw") {
    // 実機に合わせて変更。Yamlから指定できると良い
    declare_parameter("crawler_circumference", 0.39);
    declare_parameter("counts_per_rev", 256);
    declare_parameter("gearhead_ratio", 66);
    declare_parameter("pulley_ratio", 2);

    // Initialize parameters
    initParams();

    subscription_ = create_subscription<custom_interfaces::msg::CrawlerVelocity>(
        "/crawler_driver", 10, std::bind(&CrawlerDriver::driver_callback, this, std::placeholders::_1));

    init();
  }

private:
  RoboclawDriver roboclaw_;
  double crawler_circumference_;
  int counts_per_rev_;
  int gearhead_ratio_;
  int pulley_ratio_;
  double counts_per_meter_;

  rclcpp::Subscription<custom_interfaces::msg::CrawlerVelocity>::SharedPtr subscription_;

  inline double velocity_to_counts_per_sec(double velocity) const {
    return velocity * counts_per_meter_;
  }

  void initParams() {
    crawler_circumference_ = get_parameter("crawler_circumference").as_double();
    counts_per_rev_ = get_parameter("counts_per_rev").as_int();
    gearhead_ratio_ = get_parameter("gearhead_ratio").as_int();
    pulley_ratio_ = get_parameter("pulley_ratio").as_int();
    counts_per_meter_ =
        (counts_per_rev_ * gearhead_ratio_ * pulley_ratio_) / crawler_circumference_;
  }

  void init() {
    roboclaw_.setMotorVelocity(static_cast<int>(Command::M1_VELOCITY), 0,
                              [this](bool success) { handleMotorInitResult(success, "M1"); });
    roboclaw_.setMotorVelocity(static_cast<int>(Command::M2_VELOCITY), 0,
                              [this](bool success) { handleMotorInitResult(success, "M2"); });
    roboclaw_.setPIDConstants(static_cast<int>(Command::M1_SET_PID), 0.464f, 0.021f, 0.0f, M1_QPPS,
                             [this](bool success) { handlePIDInitResult(success, "M1"); });
    roboclaw_.setPIDConstants(static_cast<int>(Command::M2_SET_PID), 0.438f, 0.020f, 0.0f, M2_QPPS,
                             [this](bool success) { handlePIDInitResult(success, "M2"); });
    roboclaw_.resetEncoders([this](bool success) {
      if (!success) {
        RCLCPP_ERROR(get_logger(), "Failed to reset encoders");
      }
    });
  }

  void handleMotorInitResult(bool success, const std::string& motor_name) {
    if (!success) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize %s motor", motor_name.c_str());
    } else if (success) {
      RCLCPP_INFO(get_logger(), "success!");
    }
  }

  void handlePIDInitResult(bool success, const std::string& motor_name) {
    if (!success) {
      RCLCPP_ERROR(get_logger(), "Failed to set PID constants for %s motor", motor_name.c_str());
    }
  }

  void driver_callback(const custom_interfaces::msg::CrawlerVelocity::SharedPtr msg) {
    double M1_counts_per_sec = velocity_to_counts_per_sec(msg->m1_vel);
    double M2_counts_per_sec = velocity_to_counts_per_sec(msg->m2_vel);

    // Send commands
    roboclaw_.setMotorVelocity(static_cast<int>(Command::M1_VELOCITY), M1_counts_per_sec, [this](bool success) {
      if (!success) {
        RCLCPP_ERROR(get_logger(), "Failed to send command to M1 motor");
      }
    });

    roboclaw_.setMotorVelocity(static_cast<int>(Command::M2_VELOCITY), M2_counts_per_sec, [this](bool success) {
      if (!success) {
        RCLCPP_ERROR(get_logger(), "Failed to send command to M2 motor");
      }
    });
  }

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrawlerDriver>());
  rclcpp::shutdown();
  return 0;
}
