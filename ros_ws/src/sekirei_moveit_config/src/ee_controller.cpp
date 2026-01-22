#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#define ADDR_OPERATING_MODE     11
#define ADDR_CURRENT_LIMIT      38
#define ADDR_TORQUE_ENABLE      64
#define ADDR_GOAL_CURRENT       102
#define ADDR_GOAL_POSITION      116
#define ADDR_PRESENT_CURRENT    126
#define ADDR_PRESENT_POSITION   132

#define PROTOCOL_VERSION 2.0

class EEController : public rclcpp::Node
{
public:
  EEController() : Node("ee_controller")
  {
    RCLCPP_INFO(this->get_logger(), "EEController started");

    declare_parameter<std::string>("device", "/dev/ttyUSB0");
    declare_parameter<int>("baud_rate", 1000000);
    declare_parameter<int>("qos_depth", 10);

    get_parameter("device", device_);
    get_parameter("baud_rate", baud_rate_);
    get_parameter("qos_depth", qos_depth_);

    port_handler_ = dynamixel::PortHandler::getPortHandler(device_.c_str());
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    port_handler_->openPort();
    port_handler_->setBaudRate(baud_rate_);

    sync_write_goal_position_ =
      std::make_unique<dynamixel::GroupSyncWrite>(
        port_handler_, packet_handler_, ADDR_GOAL_POSITION, 4);

    for (auto id : ids_) {
      write1(id, ADDR_TORQUE_ENABLE, 0);
      write1(id, ADDR_OPERATING_MODE, 5);     // Current-based Position
      write2(id, ADDR_CURRENT_LIMIT, 300);
      write2(id, ADDR_GOAL_CURRENT, 110);
      write1(id, ADDR_TORQUE_ENABLE, 1);
    }

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::QoS(qos_depth_),
      std::bind(&EEController::joyCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&EEController::controlLoop, this));
  }

  ~EEController()
  {
    for (auto id : ids_) {
      write1(id, ADDR_TORQUE_ENABLE, 0);
    }
    port_handler_->closePort();
    RCLCPP_INFO(this->get_logger(), "EEController stopped");
  }

private:
  std::vector<uint8_t> ids_{27, 28};
  std::vector<int32_t> close_positions_{2776, 2139};
  std::vector<int32_t> open_positions_{3345, 1525};

  std::string device_;
  int baud_rate_;
  int qos_depth_;

  bool closing_ = false;
  int over_current_count_ = 0;

  dynamixel::PortHandler *port_handler_;
  dynamixel::PacketHandler *packet_handler_;
  std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_goal_position_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool write1(uint8_t id, uint16_t addr, uint8_t data)
  {
    uint8_t err;
    return packet_handler_->write1ByteTxRx(
      port_handler_, id, addr, data, &err) == COMM_SUCCESS && err == 0;
  }

  bool write2(uint8_t id, uint16_t addr, uint16_t data)
  {
    uint8_t err;
    return packet_handler_->write2ByteTxRx(
      port_handler_, id, addr, data, &err) == COMM_SUCCESS && err == 0;
  }

  bool read4(uint8_t id, uint16_t addr, int32_t &out)
  {
    uint8_t err;
    return packet_handler_->read4ByteTxRx(
      port_handler_, id, addr,
      reinterpret_cast<uint32_t*>(&out), &err) == COMM_SUCCESS && err == 0;
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->buttons.size() < 2) return;

    const std::vector<int32_t>* target = nullptr;

    if (msg->buttons[0]) {
      target = &close_positions_;
      closing_ = true;
    }
    else if (msg->buttons[1]) {
      target = &open_positions_;
      closing_ = false;
    }
    else {
      return;
    }

    sync_write_goal_position_->clearParam();

    for (size_t i = 0; i < ids_.size(); ++i) {
      uint8_t p[4];
      p[0] = DXL_LOBYTE(DXL_LOWORD((*target)[i]));
      p[1] = DXL_HIBYTE(DXL_LOWORD((*target)[i]));
      p[2] = DXL_LOBYTE(DXL_HIWORD((*target)[i]));
      p[3] = DXL_HIBYTE(DXL_HIWORD((*target)[i]));
      sync_write_goal_position_->addParam(ids_[i], p);
    }

    sync_write_goal_position_->txPacket();
  }

  /*把持検出 */
  void controlLoop()
  {
    if (!closing_) return;

    int16_t current;
    uint8_t err;

    packet_handler_->read2ByteTxRx(
      port_handler_, ids_[0], ADDR_PRESENT_CURRENT,
      reinterpret_cast<uint16_t*>(&current), &err);

    if (std::abs(current) > 105) {
      over_current_count_++;
    } else {
      over_current_count_ = 0;
    }

    if (over_current_count_ >= 3) {
      stopAtCurrentPosition();
      closing_ = false;
      over_current_count_ = 0;
      RCLCPP_INFO(this->get_logger(), "Grasp detected  stopping motors.");
    }
  }

  void stopAtCurrentPosition()
  {
    sync_write_goal_position_->clearParam();

    for (auto id : ids_) {
      int32_t pos;
      read4(id, ADDR_PRESENT_POSITION, pos);

      uint8_t p[4];
      p[0] = DXL_LOBYTE(DXL_LOWORD(pos));
      p[1] = DXL_HIBYTE(DXL_LOWORD(pos));
      p[2] = DXL_LOBYTE(DXL_HIWORD(pos));
      p[3] = DXL_HIBYTE(DXL_HIWORD(pos));
      sync_write_goal_position_->addParam(id, p);
    }

    sync_write_goal_position_->txPacket();
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EEController>());
  rclcpp::shutdown();
  return 0;
}
