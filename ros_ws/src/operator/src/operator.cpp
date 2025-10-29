#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "custom_interfaces/msg/crawler_velocity.hpp"
#include "custom_interfaces/msg/flipper_velocity.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class Operator : public rclcpp::Node {
   public:
    enum class Mode { STOP, DRIVE };

    Operator()
        : Node("operator"), m1_vel(0.0f), m2_vel(0.0f), mode_(Mode::STOP) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&Operator::joy_callback, this, _1));

        crawler_publisher_ =
            this->create_publisher<custom_interfaces::msg::CrawlerVelocity>(
                "/crawler_driver", 10);
        flipper_publisher_ =
            this->create_publisher<custom_interfaces::msg::FlipperVelocity>(
                "/flipper_driver", 10);

    }  // Constructor

   private:
    static constexpr float WIDTH = 0.29f;
    static constexpr float MAX_SPEED = 0.7f;
    static constexpr float DEADZONE = 0.1f;
    static constexpr int FLIPPER_SPEED = 1000;

    float m1_vel, m2_vel;
    Mode mode_;

    static float applyDeadzone(float val, float threshold) {
        return (std::abs(val) < threshold) ? 0.0f : val;
    }

    void joy_callback(const sensor_msgs::msg::Joy& msg) {
        // ボタンをチェックしてモードを切り替える
        if (msg.buttons.size() > 0) {
            // テスト
            if (msg.buttons[8] == 1)  // ボタン0が押された場合
            {
                mode_ = Mode::STOP;
                RCLCPP_INFO(this->get_logger(), "Mode switched to STOP");
            } else if (msg.buttons[9] == 1)  // ボタン1が押された場合
            {
                mode_ = Mode::DRIVE;
                RCLCPP_INFO(this->get_logger(), "Mode switched to DRIVE");
            }
        }

        // モードがSTOPの場合は速度を0に設定
        if (mode_ == Mode::STOP) {
            m1_vel = 0.0f;
            m2_vel = 0.0f;
        } else if (mode_ == Mode::DRIVE) {
            if (msg.axes.size() < 4) {
                RCLCPP_WARN(this->get_logger(),
                            "Joystick axes array is too small!");
                return;
            }

            float m1_axis_y = applyDeadzone(
                std::clamp<float>(msg.axes[1], -0.95f, 0.95f), DEADZONE);
            float m2_axis_y = applyDeadzone(
                std::clamp<float>(msg.axes[4], -0.95f, 0.95f), DEADZONE);

            m1_vel = -1.0 * std::clamp<float>(m1_axis_y * MAX_SPEED, -MAX_SPEED,
                                              MAX_SPEED);
            m2_vel =
                std::clamp<float>(m2_axis_y * MAX_SPEED, -MAX_SPEED, MAX_SPEED);

            // RCLCPP_INFO(this->get_logger(), "%f", m1_vel);
            // RCLCPP_INFO(this->get_logger(), "%f", m2_vel);
        }

        auto crawler_message = custom_interfaces::msg::CrawlerVelocity();
        crawler_message.m1_vel = m1_vel;
        crawler_message.m2_vel = m2_vel;


        auto flipper_message = custom_interfaces::msg::FlipperVelocity();


        int flipper1_sign = 0;
        int flipper2_sign = 0;
        int flipper3_sign = 0;
        int flipper4_sign = 0;
        int flipper1_speed = 0;
        int flipper2_speed = 0;
        int flipper3_speed = 0;
        int flipper4_speed = 0;

        // left backward
        if (msg.axes[7] == 1) {
            flipper1_sign = -1;
        } else if (msg.axes[7] == -1) {
            flipper1_sign = 1;
        }
        flipper1_speed = FLIPPER_SPEED * flipper1_sign;

        // right backward
        if (msg.buttons[2] == 1) {
            flipper2_sign = -1;
        } else if (msg.buttons[0] == 1) {
            flipper2_sign = 1;
        }
        flipper2_speed = FLIPPER_SPEED * flipper2_sign;

        // left forward
        if (msg.axes[2] == -1) {
            flipper3_sign = -1;
        } else if (msg.buttons[4] == 1) {
            flipper3_sign = 1;
        }
        flipper3_speed = FLIPPER_SPEED * flipper3_sign;

        // right forward
        if (msg.buttons[5] == 1) {
            flipper4_sign = 1;
        } else if (msg.axes[5] == -1) {
            flipper4_sign = -1;
        }
        flipper4_speed = FLIPPER_SPEED * flipper4_sign;

        flipper_message.flipper_vel.resize(4);
        flipper_message.flipper_vel[0] = flipper1_speed;
        flipper_message.flipper_vel[1] = flipper2_speed;
        flipper_message.flipper_vel[2] = flipper3_speed;
        flipper_message.flipper_vel[3] = flipper4_speed;

        crawler_publisher_->publish(crawler_message);
        flipper_publisher_->publish(flipper_message);
    }


    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<custom_interfaces::msg::CrawlerVelocity>::SharedPtr
        crawler_publisher_;
    rclcpp::Publisher<custom_interfaces::msg::FlipperVelocity>::SharedPtr
        flipper_publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Operator>());
    rclcpp::shutdown();
    return 0;
}
