#include "crawler_driver/roboclaw_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/msg/crawler_velocity.hpp>
#include <std_msgs/msg/Bool.hpp>
#include <memory>
#include <string>

using namespace crawler_driver::roboclaw;
using std::placeholders::_1;

/**
 * @brief ROS2 Crawler Driver Node (Roboclaw V2)
 * 
 * 新しいRoboclawDriverライブラリを使用した改善版
 * - モダンなC++20機能
 * - 型安全性の向上
 * - エラーハンドリングの改善
 */
class CrawlerDriverV2 : public rclcpp::Node {
public:
    CrawlerDriverV2() 
        : Node("crawler_driver_v2")
        , roboclaw_(std::make_unique<RoboclawDriver>("/dev/roboclaw"))
        , estop_active_(false)
    {
        // パラメータ宣言
        declare_parameter("crawler_circumference", 0.39);
        declare_parameter("counts_per_rev", 256);
        declare_parameter("gearhead_ratio", 66);
        declare_parameter("pulley_ratio", 2);
        declare_parameter("m1_pid_p", 0.464);
        declare_parameter("m1_pid_i", 0.021);
        declare_parameter("m1_pid_d", 0.0);
        declare_parameter("m2_pid_p", 0.438);
        declare_parameter("m2_pid_i", 0.020);
        declare_parameter("m2_pid_d", 0.0);

        // パラメータ初期化
        initParams();

        // サブスクライバー
        velocity_sub_ = create_subscription<custom_interfaces::msg::CrawlerVelocity>(
            "/crawler_driver", 10,
            std::bind(&CrawlerDriverV2::velocityCallback, this, _1));

        estop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/emergency_stop", 10,
            std::bind(&CrawlerDriverV2::estopCallback, this, _1));

        // 初期化
        initialize();
    }

private:
    // Roboclawドライバー
    std::unique_ptr<RoboclawDriver> roboclaw_;
    
    // パラメータ
    double crawler_circumference_;
    int counts_per_rev_;
    int gearhead_ratio_;
    int pulley_ratio_;
    double counts_per_meter_;
    
    // PID定数
    PIDConstants m1_pid_;
    PIDConstants m2_pid_;
    
    // 状態
    bool estop_active_;
    bool initialized_;

    // サブスクライバー
    rclcpp::Subscription<custom_interfaces::msg::CrawlerVelocity>::SharedPtr velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;

    /**
     * @brief パラメータ初期化
     */
    void initParams() {
        crawler_circumference_ = get_parameter("crawler_circumference").as_double();
        counts_per_rev_ = get_parameter("counts_per_rev").as_int();
        gearhead_ratio_ = get_parameter("gearhead_ratio").as_int();
        pulley_ratio_ = get_parameter("pulley_ratio").as_int();
        
        // counts/meterの計算
        counts_per_meter_ = (counts_per_rev_ * gearhead_ratio_ * pulley_ratio_) / 
                           crawler_circumference_;

        // PID定数の設定
        m1_pid_.p = static_cast<float>(get_parameter("m1_pid_p").as_double());
        m1_pid_.i = static_cast<float>(get_parameter("m1_pid_i").as_double());
        m1_pid_.d = static_cast<float>(get_parameter("m1_pid_d").as_double());
        m1_pid_.qpps = static_cast<int32_t>(EncoderQPPS::M1);

        m2_pid_.p = static_cast<float>(get_parameter("m2_pid_p").as_double());
        m2_pid_.i = static_cast<float>(get_parameter("m2_pid_i").as_double());
        m2_pid_.d = static_cast<float>(get_parameter("m2_pid_d").as_double());
        m2_pid_.qpps = static_cast<int32_t>(EncoderQPPS::M2);

        RCLCPP_INFO(get_logger(), "パラメータ初期化完了");
        RCLCPP_INFO(get_logger(), "  counts/meter: %.2f", counts_per_meter_);
        RCLCPP_INFO(get_logger(), "  M1 PID: P=%.3f I=%.3f D=%.3f QPPS=%d",
                    m1_pid_.p, m1_pid_.i, m1_pid_.d, m1_pid_.qpps);
        RCLCPP_INFO(get_logger(), "  M2 PID: P=%.3f I=%.3f D=%.3f QPPS=%d",
                    m2_pid_.p, m2_pid_.i, m2_pid_.d, m2_pid_.qpps);
    }

    /**
     * @brief Roboclaw初期化
     */
    void initialize() {
        RCLCPP_INFO(get_logger(), "Roboclaw初期化中...");
        
        initialized_ = false;

        // モーター停止
        roboclaw_->setVelocity(Motor::M1, 0.0, [this](bool success) {
            if (success) {
                RCLCPP_INFO(get_logger(), "M1 初期化: 停止 ✓");
            } else {
                RCLCPP_ERROR(get_logger(), "M1 初期化失敗");
            }
        });

        roboclaw_->setVelocity(Motor::M2, 0.0, [this](bool success) {
            if (success) {
                RCLCPP_INFO(get_logger(), "M2 初期化: 停止 ✓");
            } else {
                RCLCPP_ERROR(get_logger(), "M2 初期化失敗");
            }
        });

        // PID設定
        roboclaw_->setPID(Motor::M1, m1_pid_, [this](bool success) {
            if (success) {
                RCLCPP_INFO(get_logger(), "M1 PID設定完了 ✓");
            } else {
                RCLCPP_ERROR(get_logger(), "M1 PID設定失敗");
            }
        });

        roboclaw_->setPID(Motor::M2, m2_pid_, [this](bool success) {
            if (success) {
                RCLCPP_INFO(get_logger(), "M2 PID設定完了 ✓");
            } else {
                RCLCPP_ERROR(get_logger(), "M2 PID設定失敗");
            }
        });

        // エンコーダリセット
        roboclaw_->resetEncoders([this](bool success) {
            if (success) {
                RCLCPP_INFO(get_logger(), "エンコーダリセット完了 ✓");
                initialized_ = true;
                RCLCPP_INFO(get_logger(), "🚀 Roboclaw初期化完了！");
            } else {
                RCLCPP_ERROR(get_logger(), "エンコーダリセット失敗");
            }
        });
    }

    /**
     * @brief 速度をcounts/secに変換
     */
    inline double velocityToCountsPerSec(double velocity_m_s) const {
        return velocity_m_s * counts_per_meter_;
    }

    /**
     * @brief 速度コマンドコールバック
     */
    void velocityCallback(const custom_interfaces::msg::CrawlerVelocity::SharedPtr msg) {
        // E-stop チェック
        if (estop_active_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                "⚠️ E-stop有効: モーターコマンド無視");
            return;
        }

        // 初期化チェック
        if (!initialized_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                "⚠️ 初期化未完了: モーターコマンド無視");
            return;
        }

        // 速度変換
        const double m1_counts = velocityToCountsPerSec(msg->m1_vel);
        const double m2_counts = velocityToCountsPerSec(msg->m2_vel);

        RCLCPP_DEBUG(get_logger(), "速度指令: M1=%.2f m/s (%.0f counts/s), M2=%.2f m/s (%.0f counts/s)",
                    msg->m1_vel, m1_counts, msg->m2_vel, m2_counts);

        // M1速度設定
        roboclaw_->setVelocity(Motor::M1, m1_counts, [this](bool success) {
            if (!success) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "M1 速度設定失敗");
            }
        });

        // M2速度設定
        roboclaw_->setVelocity(Motor::M2, m2_counts, [this](bool success) {
            if (!success) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "M2 速度設定失敗");
            }
        });
    }

    /**
     * @brief 緊急停止コールバック
     */
    void estopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        const bool prev_state = estop_active_;
        estop_active_ = msg->data;

        if (estop_active_ && !prev_state) {
            RCLCPP_WARN(get_logger(), "🚨 E-stop有効化: 全モーター停止");
            stopMotors();
        } else if (!estop_active_ && prev_state) {
            RCLCPP_INFO(get_logger(), "✅ E-stop解除: モーター制御再開");
        }
    }

    /**
     * @brief 全モーター停止
     */
    void stopMotors() {
        roboclaw_->setVelocity(Motor::M1, 0.0, [this](bool success) {
            if (!success) {
                RCLCPP_ERROR(get_logger(), "M1 停止失敗");
            }
        });

        roboclaw_->setVelocity(Motor::M2, 0.0, [this](bool success) {
            if (!success) {
                RCLCPP_ERROR(get_logger(), "M2 停止失敗");
            }
        });
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<CrawlerDriverV2>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), 
                     "ノード起動失敗: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
