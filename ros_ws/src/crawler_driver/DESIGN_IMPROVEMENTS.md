# Crawler Driver V3: 設計の改善点

## 🎯 V2からV3への改善点

### V2の問題点
```cpp
// ❌ 問題：パラメータが散らばっている
double crawler_circumference_ = 0.39;
double counts_per_rev_ = 256.0;
double gearhead_ratio_ = 66.0;
double pulley_ratio_ = 2.0;

// ❌ 問題：変換ロジックがノードに直接書かれている
double velocityToCountsPerSec(double velocity) {
    return velocity * counts_per_meter_;
}
```

### V3の改善
```cpp
// ✅ 改善：物理パラメータを構造体にまとめる
struct RobotParameters {
    double wheel_base;
    double wheel_radius;
    int encoder_cpr;
    int gearbox_ratio;
    int pulley_ratio;
};

// ✅ 改善：変換ロジックをコントローラークラスに分離
class DifferentialDriveController {
    double velocityToCounts(double velocity_m_s) const;
    std::pair<double, double> twistToWheelVelocities(...);
};
```

## 🏗️ アーキテクチャの違い

### V2: フラットな設計
```
cmd_vel → 変換計算 → Roboclaw
    └─ 全部crawler_driver_v2に実装
```

### V3: 階層化設計
```
cmd_vel (Twist)
    ↓
DifferentialDriveController （運動学）
    ↓
Motor velocities [m/s]
    ↓
RoboclawDriver （ハードウェア抽象化）
    ↓
Serial commands
```

## 📊 設計の比較

| 観点 | V2 | V3 | 理由 |
|------|----|----|------|
| **可読性** | ⭐⭐ | ⭐⭐⭐⭐⭐ | 責務分離でコードが明確 |
| **テスト** | ⭐⭐ | ⭐⭐⭐⭐⭐ | 各クラスを独立テスト可能 |
| **再利用性** | ⭐ | ⭐⭐⭐⭐⭐ | Controllerを他のロボットで再利用可 |
| **保守性** | ⭐⭐ | ⭐⭐⭐⭐⭐ | 変更箇所が明確 |
| **設定** | ⭐⭐ | ⭐⭐⭐⭐⭐ | YAMLで外部化 |

## 🔑 キーポイント

### 1. 物理パラメータの抽象化
```cpp
// ❌ V2: 直接計算
counts_per_meter_ = (counts_per_rev_ * gearhead_ratio_ * pulley_ratio_) 
                    / crawler_circumference_;

// ✅ V3: 意味のあるメソッド
double counts_per_meter() const {
    return total_counts_per_rev() / (2.0 * M_PI * wheel_radius);
}
```

**利点:**
- 公式の意味が明確
- 車輪周長は `2πr` という物理的意味が分かる
- `crawler_circumference` という曖昧な名前を排除

### 2. 差動駆動の運動学を分離
```cpp
// ✅ 標準的な差動駆動計算
std::pair<double, double> twistToWheelVelocities(
    double linear_x, double angular_z) const {
    
    const double v_left = linear_x - (angular_z * wheel_base / 2.0);
    const double v_right = linear_x + (angular_z * wheel_base / 2.0);
    return {v_left, v_right};
}
```

**利点:**
- 教科書通りの公式
- 他のロボットでも再利用可能
- ユニットテストが簡単

### 3. YAMLによる設定外部化
```yaml
# ❌ V2: C++コードにハードコード
# crawler_driver_v2.cpp内に直接書かれている

# ✅ V3: YAMLファイル
wheel_base: 0.5
wheel_radius: 0.062
encoder_cpr: 256
```

**利点:**
- ハードウェア変更時に再ビルド不要
- 複数のロボット設定を管理可能
- ROS2の標準的な方法

### 4. オドメトリ計算の追加
```cpp
class OdometryCalculator {
    void update(int32_t left_counts, int32_t right_counts, double dt);
    double getX() const;
    double getY() const;
    double getTheta() const;
};
```

**利点:**
- Navigation2に必要
- エンコーダフィードバックを活用
- ロボットの位置推定が可能

## 🚀 使い方

### ビルド
```bash
cd /home/rodep/working/rodep-rescue2026/ros_ws
colcon build --packages-select crawler_driver
```

### 実行
```bash
ros2 run crawler_driver crawler_driver_v3 \
    --ros-args --params-file src/crawler_driver/config/crawler_params.yaml
```

### Teleop制御
```bash
# 別のターミナルで
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/crawler_driver_v3/cmd_vel
```

## 🧪 テスト方法

### 単体テスト例
```cpp
TEST(DifferentialDriveController, TwistToWheelVelocities) {
    RobotParameters params{
        .wheel_base = 0.5,
        .wheel_radius = 0.062
    };
    DifferentialDriveController controller(params);
    
    // 前進のみ
    auto [left, right] = controller.twistToWheelVelocities(1.0, 0.0);
    EXPECT_DOUBLE_EQ(left, 1.0);
    EXPECT_DOUBLE_EQ(right, 1.0);
    
    // 回転のみ
    auto [left2, right2] = controller.twistToWheelVelocities(0.0, 1.0);
    EXPECT_DOUBLE_EQ(left2, -0.25);  // -0.5 * 0.5 / 2
    EXPECT_DOUBLE_EQ(right2, 0.25);  // 0.5 * 0.5 / 2
}
```

## 📚 さらなる改善案

### 案A: URDF統合
```xml
<!-- robot.urdf.xacro -->
<xacro:property name="wheel_base" value="0.5"/>
<xacro:property name="wheel_radius" value="0.062"/>
```
YAMLとURDFを統一することで、シミュレーションと実機で同じパラメータを使用。

### 案B: dynamic_reconfigure
```cpp
// PIDパラメータをランタイムで調整可能に
add_on_set_parameters_callback(...);
```

### 案C: Hardware Interfaceへの移行
```cpp
// ros2_control フレームワークへ移行
class RoboclawHardwareInterface : public hardware_interface::SystemInterface {
    // ...
};
```
これがROS2の最終形態（最も標準的）。

## 🎓 結論

### いつV2を使うべきか
- 🏃 とにかく素早くプロトタイプを作りたい
- 🔧 一度だけ使う使い捨てコード
- 👤 1人で開発、他人に触らせない

### いつV3を使うべきか
- 🏭 **プロダクション環境** ← 推奨
- 👥 チーム開発
- 🔄 長期保守が必要
- 🧪 自動テストを書きたい
- 📦 複数のロボットで再利用

### ベストプラクティス
**crawler_circumferenceのような曖昧なパラメータは避け、物理的に意味のある`wheel_radius`を使いましょう。**

- ✅ `wheel_radius`: 明確（車輪の半径）
- ✅ `wheel_base`: 明確（左右の距離）
- ❌ `crawler_circumference`: 曖昧（何の周長？）
- ❌ `counts_per_meter`: 導出値（計算で求まる）

**原則: 基本的なパラメータのみをYAMLに書き、導出値はコードで計算する。**
