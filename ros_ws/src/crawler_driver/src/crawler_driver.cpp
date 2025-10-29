#include <boost/asio.hpp>
#include <custom_interfaces/msg/crawler_velocity.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>

using namespace std;
using namespace boost::asio;  // Serial communication
using std::placeholders::_1;  // std::bind

// Constants
constexpr int ROBOCLAW_ADDRESS = 0x80;
// constexpr int M1_MOTOR_COMMAND = 6;
// constexpr int M2_MOTOR_COMMAND = 7;
constexpr int M1_MOTOR_COMMAND = 35;
constexpr int M2_MOTOR_COMMAND = 36;
constexpr int M1_ENCODER_COMMAND = 92;
constexpr int M2_ENCODER_COMMAND = 93;
constexpr int M1_SET_PID_CONSTANTS_COMMAND = 28;
constexpr int M2_SET_PID_CONSTANTS_COMMAND = 29;
constexpr int SERIAL_BAUD_RATE = 38400;
constexpr int SERIAL_TIMEOUT_MS = 1000;
constexpr int RESET_QUAD_ENCODER = 20;
constexpr int M1_QPPS = 53250;
constexpr int M2_QPPS = 50062;

// RoboclawDriver Class
class RoboclawDriver {
   public:
    explicit RoboclawDriver(const string& port)
        : io(), serial(io, port), work(boost::asio::make_work_guard(io)) {
        try {
            configureSerialPort();
            io_thread_ = thread([this]() { io.run(); });
        } catch (const boost::system::system_error& e) {
            throw runtime_error("Failed to configure serial port: " +
                                string(e.what()));
        }
    }

    ~RoboclawDriver() {
        io.stop();
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
    }

    void asyncSendRoboclawCommand(const vector<uint8_t>& data,
                                  std::function<void(bool)> callback) {
        boost::asio::async_write(
            serial, boost::asio::buffer(data),
            [this, callback](const boost::system::error_code& ec,
                             std::size_t /*bytes_transferred*/) {
                if (ec) {
                    RCLCPP_ERROR(rclcpp::get_logger("RoboclawDriver"),
                                 "Serial Write Error: %s",
                                 ec.message().c_str());
                    callback(false);
                    return;
                }

                uint8_t response;
                boost::asio::async_read(
                    serial, boost::asio::buffer(&response, 1),
                    [this, callback, response](
                        const boost::system::error_code& ec,
                        std::size_t /*bytes_transferred*/) {
                        if (ec) {
                            RCLCPP_ERROR(rclcpp::get_logger("RoboclawDriver"),
                                         "Serial Read Error: %s",
                                         ec.message().c_str());
                            callback(false);
                            return;
                        }

                        RCLCPP_DEBUG(rclcpp::get_logger("RoboclawDriver"),
                                     "Received Response: 0x%02X", response);
                        callback(true);
                    });
            });
    }

    bool setMotorVelocity(int command, double /*int*/ counts_per_sec,
                          std::function<void(bool)> callback) {
        vector<uint8_t> data = {ROBOCLAW_ADDRESS,
                                static_cast<uint8_t>(command)};
        // test
        appendInt32(data, static_cast<int>(counts_per_sec));
        // data.push_back(counts_per_sec);
        appendCRC(data);
        asyncSendRoboclawCommand(data, callback);
        return true;
    }

    bool setPIDConstants(int command, float K_p, float K_i, float K_d, int qpps,
                         std::function<void(bool)> callback) {
        vector<uint8_t> data = {ROBOCLAW_ADDRESS,
                                static_cast<uint8_t>(command)};
        appendFloat32(data, K_d);
        appendFloat32(data, K_p);
        appendFloat32(data, K_i);
        appendInt32(data, qpps);
        appendCRC(data);
        asyncSendRoboclawCommand(data, callback);
        return true;
    }

    bool resetEncoders(std::function<void(bool)> callback) {
        vector<uint8_t> data = {ROBOCLAW_ADDRESS, RESET_QUAD_ENCODER};
        appendCRC(data);
        asyncSendRoboclawCommand(data, callback);
        return true;
    }

   private:
    io_context io;
    serial_port serial;
    thread io_thread_;
    // io_context::work work;  // Keeps io_context running
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type>
        work;

    void configureSerialPort() {
        serial.set_option(serial_port_base::baud_rate(SERIAL_BAUD_RATE));
        serial.set_option(serial_port_base::character_size(8));
        serial.set_option(
            serial_port_base::parity(serial_port_base::parity::none));
        serial.set_option(
            serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        serial.set_option(serial_port_base::flow_control(
            serial_port_base::flow_control::none));
    }

    uint16_t calculateCRC(const vector<uint8_t>& data) {
        uint16_t crc = 0;
        for (auto byte : data) {
            crc ^= static_cast<uint16_t>(byte) << 8;
            for (int i = 0; i < 8; i++) {
                crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
            }
        }
        return crc;
    }

    void appendCRC(vector<uint8_t>& data) {
        uint16_t crc = calculateCRC(data);
        data.push_back(static_cast<uint8_t>(crc >> 8));
        data.push_back(static_cast<uint8_t>(crc & 0xFF));
    }

    void appendInt32(vector<uint8_t>& data, int value) {
        for (int i = 3; i >= 0; --i) {
            data.push_back(static_cast<uint8_t>((value >> (8 * i)) & 0xFF));
        }
    }

    void appendFloat32(vector<uint8_t>& data, float value) {
        // uint8_t* bytes = reinterpret_cast<uint8_t*>(&value);
        // for (size_t i = 0; i < sizeof(float); ++i) {
        //	data.push_back(bytes[i]);
        //}
        for (int i = 3; i >= 0; --i) {
            data.push_back(static_cast<uint8_t>(
                (static_cast<int>(value) >> (8 * i)) & 0xFF));
        }
    }
};

// ROS2 Driver Node
class CrawlerDriver : public rclcpp::Node {
   public:
    CrawlerDriver() : Node("crawler_driver"), roboclaw("/dev/roboclaw") {
        declare_parameter("crawler_circumference", 0.39);
        declare_parameter("counts_per_rev", 256);
        declare_parameter("gearhead_ratio", 66);
        declare_parameter("pulley_ratio", 2);

        // Initialize parameters
        initParams();

        subscription_ =
            create_subscription<custom_interfaces::msg::CrawlerVelocity>(
                "/crawler_driver", 10,
                bind(&CrawlerDriver::driver_callback, this, _1));

        estop_subscription_ = create_subscription<std_msgs::msg::Bool>(
            "/emergency_stop", 10,
            bind(&CrawlerDriver::estop_callback, this, _1));

        init();
    }

   private:
    RoboclawDriver roboclaw;
    double crawler_circumference_;
    int counts_per_rev_;
    int gearhead_ratio_;
    int pulley_ratio_;
    double counts_per_meter_;
    bool estop_active_ = false;  // E-stop state

    rclcpp::Subscription<custom_interfaces::msg::CrawlerVelocity>::SharedPtr
        subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscription_;

    inline double velocity_to_counts_per_sec(double velocity) const {
        return velocity * counts_per_meter_;
    }

    void initParams() {
        crawler_circumference_ =
            get_parameter("crawler_circumference").as_double();
        counts_per_rev_ = get_parameter("counts_per_rev").as_int();
        gearhead_ratio_ = get_parameter("gearhead_ratio").as_int();
        pulley_ratio_ = get_parameter("pulley_ratio").as_int();
        counts_per_meter_ =
            (counts_per_rev_ * gearhead_ratio_ * pulley_ratio_) /
            crawler_circumference_;
    }

    void init() {
        roboclaw.setMotorVelocity(M1_MOTOR_COMMAND, 0, [this](bool success) {
            handleMotorInitResult(success, "M1");
        });
        roboclaw.setMotorVelocity(M2_MOTOR_COMMAND, 0, [this](bool success) {
            handleMotorInitResult(success, "M2");
        });
        roboclaw.setPIDConstants(
            M1_SET_PID_CONSTANTS_COMMAND, 0.464f, 0.021f, 0.0f, M1_QPPS,
            [this](bool success) { handlePIDInitResult(success, "M1"); });
        roboclaw.setPIDConstants(
            M2_SET_PID_CONSTANTS_COMMAND, 0.438f, 0.020f, 0.0f, M2_QPPS,
            [this](bool success) { handlePIDInitResult(success, "M2"); });
        roboclaw.resetEncoders([this](bool success) {
            if (!success) {
                RCLCPP_ERROR(get_logger(), "Failed to reset encoders");
            }
        });
    }

    void handleMotorInitResult(bool success, const string& motor_name) {
        if (!success) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize %s motor",
                         motor_name.c_str());
        } else if (success) {
            RCLCPP_INFO(get_logger(), "success!");
        }
    }

    void handlePIDInitResult(bool success, const string& motor_name) {
        if (!success) {
            RCLCPP_ERROR(get_logger(),
                         "Failed to set PID constants for %s motor",
                         motor_name.c_str());
        }
    }

    void driver_callback(const custom_interfaces::msg::CrawlerVelocity& msg) {
        if (estop_active_) {
            RCLCPP_WARN(get_logger(),
                        "E-stop is active. Ignoring motor commands.");
            return;
        }

        double M1_counts_per_sec = velocity_to_counts_per_sec(msg.m1_vel);
        double M2_counts_per_sec = velocity_to_counts_per_sec(msg.m2_vel);

        // test
        roboclaw.setMotorVelocity(
            M1_MOTOR_COMMAND, M1_counts_per_sec, [this](bool success) {
                if (!success) {
                    RCLCPP_ERROR(get_logger(),
                                 "Failed to send command to M1 motor");
                }
            });

        roboclaw.setMotorVelocity(
            M2_MOTOR_COMMAND, M2_counts_per_sec, [this](bool success) {
                if (!success) {
                    RCLCPP_ERROR(get_logger(),
                                 "Failed to send command to M2 motor");
                }
            });
    }

    void estop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        estop_active_ = msg->data;

        if (estop_active_) {
            RCLCPP_WARN(get_logger(), "E-stop activated. Stopping all motors.");
            stopMotors();
        } else {
            RCLCPP_INFO(get_logger(),
                        "E-stop deactivated. Resuming motor control.");
        }
    }

    void stopMotors() {
        roboclaw.setMotorVelocity(M1_MOTOR_COMMAND, 0, [](bool success) {
            if (!success) {
                RCLCPP_ERROR(rclcpp::get_logger("RoboclawDriver"),
                             "Failed to stop M1 motor");
            }
        });
        roboclaw.setMotorVelocity(M2_MOTOR_COMMAND, 0, [](bool success) {
            if (!success) {
                RCLCPP_ERROR(rclcpp::get_logger("RoboclawDriver"),
                             "Failed to stop M2 motor");
            }
        });
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<CrawlerDriver>());
    rclcpp::shutdown();
    return 0;
}
