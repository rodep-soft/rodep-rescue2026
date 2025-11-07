# Operator ノード

このNodeは、Joystickの入力を受け取り、ロボットの駆動速度を制御するためのmsg(custom_interfaces/msg/DriverVelocity)を送信します。また、Serviceを介して動作Modeを変更することができます。

## 必要な依存関係

- ROS2 Humble
- `sensor_msgs` package
- `custom_interfaces` package（`DriverVelocity` msg及び `SetMode` srvを含む）

## ビルド方法

```sh
colcon build --packages-select operator
source install/setup.bash
```

## 使用方法

### ノードの実行

```sh
ros2 run operator operator
```

### ジョイスティック操作

- **ボタン 4 (Share)** を押すと `STOP` Modeに変更
- **ボタン 6 (Options)** を押すと `DRIVE` Modeに変更
- **左スティック (axes[1])** と **右スティック (axes[3])** でロボットの速度を制御(左スティックが左側モーター、右スティックが右側モーターに対応)

### サービスを使用してモードを変更

```sh
ros2 service call /set_mode custom_interfaces/srv/SetMode "{mode: 'STOP'}"
ros2 service call /set_mode custom_interfaces/srv/SetMode "{mode: 'DRIVE'}"
```

## トピック

| トピック名  | 型 | 説明 |
|-------------|----------------------------|------------------------------|
| `/joy`      | `sensor_msgs/msg/Joy`      | Joystickの入力をSubscribe |
| `/driver`   | `custom_interfaces/msg/DriverVelocity` | モータの速度をPublish |

## サービス

| サービス名  | 型 | 説明 |
|-------------|----------------------------|------------------------------|
| `/set_mode` | `custom_interfaces/srv/SetMode` | Mode (`STOP` / `DRIVE`) を設定 |

## 実装の概要

- `joy_callback`: Joystickの入力を処理し、Modeの切り替えや速度計算を行う。
- `set_mode_callback`: `/set_mode` Serviceのリクエストを処理し、Modeを変更。
- `applyDeadzone`: スティックのDeadzoneを適用。
- `main`: ROS2 Nodeを初期化し、`Operator` Nodeを起動。

## ライセンス

MIT License
