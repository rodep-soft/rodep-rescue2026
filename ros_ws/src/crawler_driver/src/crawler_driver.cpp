#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_POSITION 132

#define PROTOCOL_VERSION 2.0

uint8_t ids[] = {1, 2, 3, 4};
uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;  // Communication result

class DynamixelController : public rclcpp::Node {
   public:
    DynamixelController() : Node("dynamixel_control_node") {
        RCLCPP_INFO(this->get_logger(), "Run dynamixel control node");

        this->declare_parameter("device_name", "/dev/ttyUSB0");
        this->declare_parameter("baudrate", 115200);
        this->declare_parameter("qos_depth", 10);

        this->get_parameter("qos_depth", qos_depth);
        this->get_parameter("device_name", device_name);
        this->get_parameter("baudrate", baudrate);

        portHandler =
            dynamixel::PortHandler::getPortHandler(device_name.c_str());
        packetHandler =
            dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        const auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(qos_depth))
                                     .reliable()
                                     .durability_volatile();

        if (!portHandler->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Succeeded to open the port.");

        if (!portHandler->setBaudRate(baudrate)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate!");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Succeeded to set the baudrate.");

        // Setup each Dynamixel
        for (uint8_t id : ids) {
            setupDynamixel(id);
        }

        // Subscriber for joystick
        set_velocity_subscriber_ =
            this->create_subscription<sensor_msgs::msg::Joy>(
                "joy", qos_profile,
                [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
                    if (msg->axes.empty()) return;

                    float raw = msg->axes[1];  // left stick vertical axis
                    int target_velocity = static_cast<int>(200 * raw);

                    // Send goal velocity to each Dynamixel
                    for (uint8_t id : ids) {
                        sendGoalVelocity(id, target_velocity);
                    }
                });
    }

    ~DynamixelController() {
        // Disable Torque and Close port
        for (uint8_t id : ids) {
            dxl_comm_result = packetHandler->write1ByteTxRx(
                portHandler, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "%s",
                             packetHandler->getTxRxResult(dxl_comm_result));
            } else if (dxl_error != 0) {
                RCLCPP_ERROR(this->get_logger(), "%s",
                             packetHandler->getRxPacketError(dxl_error));
            } else {
                RCLCPP_INFO(this->get_logger(), "Disabled torque [ID: %d]", id);
            }
        }
        portHandler->closePort();
        RCLCPP_INFO(this->get_logger(), "Closed the port");
    }

   private:
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
    int qos_depth;
    int baudrate;
    std::string device_name;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
        set_velocity_subscriber_;

    void setupDynamixel(uint8_t dxl_id) {
        // Use Velocity Control Mode
        dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, dxl_id, ADDR_OPERATING_MODE,
            1,  // Velocity Control Mode
            &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to set Velocity Control Mode.");
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "Succeeded to set Velocity Control Mode.");
        }

        // Enable Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable torque.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Succeeded to enable torque.");
        }
    }

    void sendGoalVelocity(uint8_t id, int velocity) {
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler, id, ADDR_GOAL_VELOCITY, velocity, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "%s",
                         packetHandler->getTxRxResult(dxl_comm_result));
        } else if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "%s",
                         packetHandler->getRxPacketError(dxl_error));
        } else {
            RCLCPP_INFO(this->get_logger(), "Set [ID: %d Goal Velocity: %d]",
                        id, velocity);
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelController>());
    rclcpp::shutdown();
    return 0;
}
