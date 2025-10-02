#include "dynamixel_sdk/dynamixel_sdk.h"

#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_POSITION 132

#define PROTOCOL_VERSION 2.0

#define BAUDRATE 115200
#define DEVICE_NAME "/dev/ttyUSB0"

dynamixel::PortHandler* portHandler;
dynamixel::PacketHandler* packetHandler;

uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;  // Communication result

class DynamixelController : public rclcpp::Node {
   public:
    DynamixelController() : Node("dynamixel_control_node") {
        RCLCPP_INFO(this->get_logger(), "Run dynamixel control node");

        this->declare_parameter("qos_depth", 10);
        int qos_depth = this->get_parameter("qos_depth").as_int();

        const auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(qos_depth))
                                     .reliable()
                                     .durability_volatile();

        // Subscriber for joystick
        set_position_subscriber_ =
            this->create_subscription<sensor_msgs::msg::Joy>(
                "joy", qos_profile,
                [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
                    if (msg->axes.empty()) return;

                    float raw = msg->axes[1];  // left stick vertical axis
                    int target_velocity = static_cast<int>(200 * raw);

                    dxl_comm_result = packetHandler->write4ByteTxRx(
                        portHandler,
                        BROADCAST_ID,  // dynamixel ID
                        ADDR_GOAL_VELOCITY, target_velocity, &dxl_error);

                    if (dxl_comm_result != COMM_SUCCESS) {
                        RCLCPP_ERROR(
                            this->get_logger(), "%s",
                            packetHandler->getTxRxResult(
                                dxl_comm_result));  // transmit or receive error
                    } else if (dxl_error != 0) {
                        RCLCPP_ERROR(this->get_logger(), "%s",
                                     packetHandler->getRxPacketError(
                                         dxl_error));  // dynamixel error
                    } else {
                        RCLCPP_INFO(this->get_logger(),
                                    "Set [Goal Velocity: %d]", target_velocity);
                    }
                });
    }

   private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
        set_position_subscriber_;
};

void setupDynamixel(uint8_t dxl_id) {
    // Use Velocity Control Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler, dxl_id, ADDR_OPERATING_MODE, BROADCAST_ID, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("dynamixel_setup"),
                     "Failed to set Velocity Control Mode.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("dynamixel_setup"),
                    "Succeeded to set Velocity Control Mode.");
    }

    // Enable Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("dynamixel_setup"),
                     "Failed to enable torque.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("dynamixel_setup"),
                    "Succeeded to enable torque.");
    }
}

int main(int argc, char* argv[]) {
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler =
        dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    bool result = portHandler->openPort();
    if (!result) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to open the port!");
        return -1;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("main"), "Succeeded to open the port.");
    }

    result = portHandler->setBaudRate(BAUDRATE);
    if (!result) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to set the baudrate!");
        return -1;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "Succeeded to set the baudrate.");
    }

    setupDynamixel(BROADCAST_ID);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelController>());
    rclcpp::shutdown();

    // Disable Torque
    packetHandler->write1ByteTxRx(portHandler, 1, ADDR_TORQUE_ENABLE, 0,
                                  &dxl_error);

    return 0;
}
