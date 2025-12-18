# Roboclaw Driver for ROS2

## 概要
このパッケージは、Roboclaw モータードライバを ROS2 で制御するためのノードを提供します。シリアル通信を用いてモーターの速度制御やエンコーダーのリセットを行います。

## 必要な依存関係
このパッケージを動作させるためには、以下のパッケージとライブラリが必要です。

- ROS2 Humble 以降
- Boost Asio (`boost::asio`)
- `rclcpp`
- `std_msgs` (`std_msgs/msg/bool.hpp`)
- `custom_interfaces` (`custom_interfaces/msg/driver_velocity.hpp`)

## インストール方法
1. ワークスペースを作成し、`src` ディレクトリに移動します。

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. このリポジトリをクローンします。

```sh
git clone <リポジトリのURL>
```

3. 依存関係をインストールします。

```sh
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. パッケージをビルドします。

```sh
colcon build --packages-select <パッケージ名>
```

## 使用方法
1. ROS2 をセットアップします。

```sh
source ~/ros2_ws/install/setup.bash
```

2. ドライバーノードを起動します。

```sh
ros2 run <パッケージ名> driver
```

3. `/operator` トピックに速度コマンドを送信すると、モーターが動作します。

```sh
ros2 topic pub /operator custom_interfaces/msg/DriverVelocity "{m1_vel: 1.0, m2_vel: 1.0}"
```

4. `/emergency_stop` トピックに `true` を送信すると、モーターが停止します。

```sh
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: true}"
```

## パラメータ設定
ノードは以下のパラメータをサポートしています。

| パラメータ名            | デフォルト値 | 説明 |
|----------------|---------|--------------------------------------|
| `crawler_circumference` | 0.39    | クローラーの円周（m） |
| `pulse_per_rev`        | 256     | 1回転あたりのパルス数（エンコーダー） |
| `gearhead_ratio`       | 66      | 減速機の比率 |
| `pulley_ratio`         | 2       | プーリーの比率 |

パラメータを変更する場合は、起動時に指定することができます。

```sh
ros2 run <パッケージ名> driver --ros-args -p crawler_circumference:=0.5
```

## ライセンス
このプロジェクトは [MIT License](LICENSE) のもとで提供されます。
