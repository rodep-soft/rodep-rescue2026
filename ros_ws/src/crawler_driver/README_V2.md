# Crawler Driver V2 - 使用方法

## 📦 新しいRoboclawライブラリを使用した改善版

`crawler_driver_v2`は、モダンなC++20とRoboclawDriverライブラリを使用した改善版のクローラードライバーです。

## ✨ 主な改善点

### 従来版 (crawler_driver.cpp) との違い

| 項目 | 従来版 | V2 |
|-----|--------|-----|
| C++バージョン | C++14 | C++20 |
| 型安全性 | 低い（生のint） | 高い（enum class, 構造体） |
| エラーハンドリング | 基本的 | 詳細（タイムアウト、CRC検証） |
| コード可読性 | 中 | 高（意図が明確） |
| ロギング | 限定的 | 詳細（絵文字付き） |
| ヘッダー分離 | なし | あり（roboclaw_driver.hpp） |

### モダンな機能

```cpp
// ✅ 型安全なモーター指定
roboclaw_->setVelocity(Motor::M1, speed, callback);
// ❌ 従来: roboclaw.setMotorVelocity(35, speed, callback);

// ✅ PID定数を構造体で管理
PIDConstants pid{.p=0.464f, .i=0.021f, .d=0.0f, .qpps=53250};
roboclaw_->setPID(Motor::M1, pid, callback);
// ❌ 従来: roboclaw.setPIDConstants(28, 0.464f, 0.021f, 0.0f, 53250, callback);

// ✅ エンコーダ読み取り（新機能）
roboclaw_->readEncoder(Motor::M1, [](std::optional<EncoderValue> value) {
    if (value && value->is_valid()) {
        std::cout << "Counts: " << value->counts << std::endl;
    }
});
```

## 🚀 ビルド方法

```bash
cd /home/rodep/working/rodep-rescue2026/ros_ws
colcon build --packages-select crawler_driver
source install/setup.bash
```

## 📋 実行方法

### V2ドライバーの起動

```bash
ros2 run crawler_driver crawler_driver_v2_node
```

### パラメータ設定付き起動

```bash
ros2 run crawler_driver crawler_driver_v2_node \
  --ros-args \
  -p crawler_circumference:=0.39 \
  -p counts_per_rev:=256 \
  -p gearhead_ratio:=66 \
  -p pulley_ratio:=2 \
  -p m1_pid_p:=0.464 \
  -p m1_pid_i:=0.021 \
  -p m1_pid_d:=0.0 \
  -p m2_pid_p:=0.438 \
  -p m2_pid_i:=0.020 \
  -p m2_pid_d:=0.0
```

### Launchファイルで起動（推奨）

`launch/crawler_driver_v2.launch.py`を作成:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='crawler_driver',
            executable='crawler_driver_v2_node',
            name='crawler_driver_v2',
            output='screen',
            parameters=[{
                'crawler_circumference': 0.39,
                'counts_per_rev': 256,
                'gearhead_ratio': 66,
                'pulley_ratio': 2,
                'm1_pid_p': 0.464,
                'm1_pid_i': 0.021,
                'm1_pid_d': 0.0,
                'm2_pid_p': 0.438,
                'm2_pid_i': 0.020,
                'm2_pid_d': 0.0,
            }]
        )
    ])
```

起動:
```bash
ros2 launch crawler_driver crawler_driver_v2.launch.py
```

## 📡 トピック

### Subscribe
- `/crawler_driver` (`custom_interfaces/msg/CrawlerVelocity`)
  - `m1_vel`: M1モーターの速度 [m/s]
  - `m2_vel`: M2モーターの速度 [m/s]

- `/emergency_stop` (`std_msgs/msg/Bool`)
  - `true`: 緊急停止有効（モーター停止）
  - `false`: 緊急停止解除

## 🧪 テスト方法

### 1. モーター動作テスト

```bash
# ターミナル1: ドライバー起動
ros2 run crawler_driver crawler_driver_v2_node

# ターミナル2: 速度指令送信
ros2 topic pub /crawler_driver custom_interfaces/msg/CrawlerVelocity \
  "{m1_vel: 0.1, m2_vel: 0.1}" --once
```

### 2. 緊急停止テスト

```bash
# E-stop有効化
ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: true" --once

# E-stop解除
ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: false" --once
```

### 3. ログ確認

```bash
# INFO以上のログ
ros2 run crawler_driver crawler_driver_v2_node --ros-args --log-level info

# DEBUGログも表示
ros2 run crawler_driver crawler_driver_v2_node --ros-args --log-level debug
```

## 🔧 トラブルシューティング

### シリアルポートが見つからない

```bash
# デバイス確認
ls -l /dev/roboclaw
ls -l /dev/ttyACM*

# 権限追加
sudo usermod -a -G dialout $USER
# ログアウト→ログイン
```

### 初期化失敗

```log
[ERROR] [crawler_driver_v2]: M1 初期化失敗
```

**対処法**:
1. Roboclawの電源を確認
2. シリアル接続を確認
3. ボーレート設定を確認（38400 bps）
4. RoboClaw Motion Studioでアドレス確認（0x80）

### PID設定失敗

**対処法**:
1. パラメータの範囲を確認（負の値は不可）
2. QPPSが正しいか確認
   - M1: 53250
   - M2: 50062

## 📚 関連ファイル

- `include/crawler_driver/roboclaw_driver.hpp` - Roboclawドライバーヘッダー
- `src/roboclaw_driver.cpp` - Roboclawドライバー実装
- `src/crawler_driver_v2.cpp` - ROS2ノード実装
- `ROBOCLAW_SPEC.md` - Roboclaw通信仕様書

## 🎯 今後の機能追加案

- [ ] エンコーダ値のPublish
- [ ] オドメトリ計算
- [ ] 電流モニタリング
- [ ] バッテリー電圧監視
- [ ] 診断情報のPublish (`diagnostic_msgs`)
- [ ] サービスによるPID動的調整

## 📝 ライセンス

同プロジェクトのライセンスに従います。
