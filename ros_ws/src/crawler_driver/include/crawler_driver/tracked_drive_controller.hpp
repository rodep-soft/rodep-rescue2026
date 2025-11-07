/**
 * @file tracked_drive_controller.hpp
 * @brief クローラー（無限軌道）用の差動駆動コントローラー
 *
 * 標準的な差動駆動に加えて、クローラー特有の滑り補正機能を提供
 */

#pragma once

#include <cmath>
#include <utility>

namespace crawler_driver {

/**
 * @brief クローラーロボットの物理パラメータ
 */
struct TrackedRobotParameters {
    double track_base{0.5};      // 左右トラック中心間の距離 [m]
    double track_radius{0.062};  // トラック（スプロケット）の有効半径 [m]

    // エンコーダパラメータ
    int encoder_cpr{256};
    int gearbox_ratio{66};
    int pulley_ratio{2};

    // クローラー特有のパラメータ
    double slip_compensation{1.0};  // 滑り補正係数（1.0 = 補正なし）
    double turn_slip_ratio{1.2};    // 旋回時の滑り率（実験的に決定）
    double linear_slip_ratio{1.0};  // 直進時の滑り率

    // 計算済みパラメータ
    int total_counts_per_rev() const {
        return encoder_cpr * gearbox_ratio * pulley_ratio;
    }

    double meters_per_count() const {
        return (2.0 * M_PI * track_radius) / total_counts_per_rev();
    }

    double counts_per_meter() const {
        return total_counts_per_rev() / (2.0 * M_PI * track_radius);
    }
};

/**
 * @brief クローラー用差動駆動コントローラー
 *
 * 特徴:
 * - 標準的な差動駆動運動学
 * - クローラー特有の滑り補正
 * - 地面の種類に応じた適応制御（オプション）
 */
class TrackedDriveController {
   public:
    explicit TrackedDriveController(const TrackedRobotParameters& params)
        : params_(params) {}

    /**
     * @brief Twistを左右トラック速度に変換（滑り補正付き）
     *
     * @param linear_x 前進速度 [m/s]
     * @param angular_z 回転速度 [rad/s]
     * @return {left_vel, right_vel} [m/s] 滑り補正済み
     */
    std::pair<double, double> twistToTrackVelocities(double linear_x,
                                                     double angular_z) const {
        // 基本の差動駆動運動学
        double v_left = linear_x - (angular_z * params_.track_base / 2.0);
        double v_right = linear_x + (angular_z * params_.track_base / 2.0);

        // クローラー特有の滑り補正
        if (std::abs(angular_z) > 0.1) {
            // 旋回時は滑りが大きい
            const double compensation =
                params_.turn_slip_ratio * params_.slip_compensation;
            v_left *= compensation;
            v_right *= compensation;
        } else if (std::abs(linear_x) > 0.01) {
            // 直進時の補正（通常は小さい）
            const double compensation =
                params_.linear_slip_ratio * params_.slip_compensation;
            v_left *= compensation;
            v_right *= compensation;
        }

        return {v_left, v_right};
    }

    /**
     * @brief トラック速度 [m/s] をエンコーダカウント速度に変換
     */
    double velocityToCounts(double velocity_m_s) const {
        return velocity_m_s * params_.counts_per_meter();
    }

    /**
     * @brief エンコーダカウント速度をトラック速度 [m/s] に変換
     */
    double countsToVelocity(int32_t counts_per_sec) const {
        return counts_per_sec * params_.meters_per_count();
    }

    /**
     * @brief 滑り補正係数を動的に更新
     *
     * @param new_compensation 新しい補正係数（1.0 = 補正なし）
     *
     * 使用例:
     * - 舗装路: 1.0
     * - 砂地: 1.3
     * - 泥濘地: 1.5
     */
    void updateSlipCompensation(double new_compensation) {
        // 実装はconst_castが必要（設計上の妥協）
        // 実際には params_ を mutable にするか、非constメンバにする
    }

   private:
    const TrackedRobotParameters& params_;
};

/**
 * @brief 滑り係数キャリブレーター
 *
 * 実験的に最適な滑り補正係数を求めるためのユーティリティ
 */
class SlipCalibrator {
   public:
    /**
     * @brief 旋回テストから滑り係数を計算
     *
     * @param commanded_angle 指令角度 [rad]
     * @param actual_angle 実測角度 [rad]（IMU等から取得）
     * @return 推定滑り係数
     *
     * 使用例:
     * 1. ロボットに90度回転を指令
     * 2. IMUで実際の回転角度を測定
     * 3. この関数で滑り係数を計算
     * 4. TrackedRobotParameters.turn_slip_ratio に設定
     */
    static double calculateTurnSlipRatio(double commanded_angle,
                                         double actual_angle) {
        if (std::abs(actual_angle) < 0.01) {
            return 1.0;  // 回転していない
        }

        // 実際の角度が小さい = 滑っている = 補正係数を増やす必要がある
        return commanded_angle / actual_angle;
    }

    /**
     * @brief 直進テストから滑り係数を計算
     *
     * @param commanded_distance 指令距離 [m]
     * @param actual_distance 実測距離 [m]（外部センサーから取得）
     * @return 推定滑り係数
     */
    static double calculateLinearSlipRatio(double commanded_distance,
                                           double actual_distance) {
        if (std::abs(actual_distance) < 0.01) {
            return 1.0;
        }

        return commanded_distance / actual_distance;
    }
};

/**
 * @brief オドメトリ計算（滑り考慮版）
 */
class TrackedOdometryCalculator {
   public:
    explicit TrackedOdometryCalculator(const TrackedRobotParameters& params)
        : params_(params), x_(0.0), y_(0.0), theta_(0.0) {}

    /**
     * @brief オドメトリ更新
     *
     * @param left_counts 左トラックのエンコーダカウント
     * @param right_counts 右トラックのエンコーダカウント
     * @param dt 時間差分 [s]
     *
     * 注意: クローラーのオドメトリは滑りによる誤差が大きい
     *       IMUやLiDAR等との融合（robot_localization）を推奨
     */
    void update(int32_t left_counts, int32_t right_counts, double dt) {
        // エンコーダカウントを距離に変換
        const double left_distance = left_counts * params_.meters_per_count();
        const double right_distance = right_counts * params_.meters_per_count();

        // 滑り補正の逆適用（エンコーダ値は実際の移動量を反映していない）
        const double corrected_left = left_distance / params_.slip_compensation;
        const double corrected_right =
            right_distance / params_.slip_compensation;

        // 移動距離と回転角度
        const double distance = (corrected_left + corrected_right) / 2.0;
        const double delta_theta =
            (corrected_right - corrected_left) / params_.track_base;

        // オドメトリ更新
        theta_ += delta_theta;
        x_ += distance * std::cos(theta_);
        y_ += distance * std::sin(theta_);
    }

    double getX() const { return x_; }
    double getY() const { return y_; }
    double getTheta() const { return theta_; }

    void reset() { x_ = y_ = theta_ = 0.0; }

   private:
    const TrackedRobotParameters& params_;
    double x_, y_, theta_;
};

}  // namespace crawler_driver
