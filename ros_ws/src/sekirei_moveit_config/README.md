# Sekirei MoveIt Configuration

Sekirei 6-DOF ロボットアームの MoveIt2 設定パッケージです。ROS2 Jazzy + ros2_control を使用して、シミュレーションと実機の両方に対応しています。

## 概要

このパッケージは以下の機能を提供します：
- OMPL による動作計画
- RViz2 での可視化と操作
- ros2_control による制御
- Time-Optimal Trajectory Generation による軌道の時間パラメータ化
- Mock hardware とDynamixel実機の切り替え

## 必要なパッケージ

- ROS2 Jazzy
- MoveIt2
- ros2_control
- ros2_controllers
- urdf_test_node (URDF定義)
- dynamixel_hardware (実機制御用、オプション)

## 使い方

### 1. シミュレーション（Mock Hardware）で動かす

Mock hardware を使用すると、実機なしで動作確認ができます。

```bash
# ワークスペースをビルド
cd /root/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sekirei_moveit_config

# セットアップ
source install/setup.bash

# デモ起動（自動的にmock hardwareを使用）
ros2 launch sekirei_moveit_config demo.launch.py
```

**動作確認：**
1. RViz2 が起動します
2. Motion Planning パネルで Planning Group が "sekirei_arm" になっていることを確認
3. Interactive Marker（青い球）をドラッグして目標位置を設定
4. "Plan & Execute" ボタンをクリック
5. アームが動作すれば成功！

### 2. 実機（Dynamixel Hardware）で動かす

実機を使用する場合は、`dynamixel_hardware` パッケージが必要です。

#### 前提条件

1. **ハードウェア接続：**
   - Dynamixel モーターが正しく接続されている
   - USB-to-Serial アダプタが接続されている（通常 `/dev/ttyUSB0`）

2. **dynamixel_hardware パッケージがインストール済み：**
   ```bash
   # パッケージの存在を確認
   ros2 pkg list | grep dynamixel_hardware
   ```

3. **デバイスのパーミッション設定：**
   ```bash
   # USB デバイスへのアクセス権限を付与
   sudo chmod 666 /dev/ttyUSB0

   # または udev ルールで恒久的に設定
   sudo usermod -aG dialout $USER
   # ログアウト/ログインが必要
   ```

#### 起動手順

```bash
# ワークスペースをビルド
cd /root/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sekirei_moveit_config dynamixel_hardware

# セットアップ
source install/setup.bash

# デモ起動（自動的にdynamixel hardwareを使用）
ros2 launch sekirei_moveit_config demo.launch.py
```

**launch.py は自動的に以下を判断します：**
- `dynamixel_hardware` パッケージが存在する → `sekirei_moveit.urdf` (実機用) を使用
- `dynamixel_hardware` パッケージが存在しない → `sekirei_moveit_dummy.urdf` (Mock用) を使用

#### 実機での注意事項

1. **初回起動時：**
   - モーターのトルクがOFFの状態で起動することを確認
   - 可動範囲に障害物がないことを確認

2. **動作速度の調整：**
   - RViz2 の Motion Planning パネルで "Max Velocity" と "Max Acceleration" のスライダーを調整
   - 初回は低速（10-20%）から始めることを推奨

3. **緊急停止：**
   - 何か問題が発生したら、すぐに電源を切断してください
   - または別ターミナルで: `docker exec ros2_working_container pkill -9 -f ros2`

## アーキテクチャ

### Hardware Interface の切り替え

`demo.launch.py` は起動時に自動的に適切な URDF を選択します：

```python
try:
    _ = get_package_share_directory('dynamixel_hardware')
    urdf_file = 'sekirei_moveit.urdf'  # 実機用
    print("Using sekirei_moveit.urdf with dynamixel_hardware plugin")
except PackageNotFoundError:
    urdf_file = 'sekirei_moveit_dummy.urdf'  # Mock用
    print("Using sekirei_moveit_dummy.urdf with mock_components")
```

### ros2_control の構成

**Mock Hardware (`sekirei_moveit_dummy.urdf`):**
```xml
<ros2_control name="sekirei_dynamixel_hw" type="system">
  <hardware>
    <plugin>mock_components/GenericSystem</plugin>
  </hardware>
  <joint name="arm_joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  ...
</ros2_control>
```

**実機 Hardware (`sekirei_moveit.urdf`):**
```xml
<ros2_control name="sekirei_dynamixel_hw" type="system">
  <hardware>
    <plugin>dynamixel_hardware/DynamixelHardware</plugin>
    <param name="usb_port">/dev/ttyUSB0</param>
    <param name="baud_rate">1000000</param>
  </hardware>
  <joint name="arm_joint1">
    <param name="id">1</param>
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  ...
</ros2_control>
```

### Controller 構成

`ros2_controllers.yaml` で定義されたコントローラー：

1. **joint_state_broadcaster**: ジョイント状態をパブリッシュ
2. **sekirei_arm_controller**: JointTrajectoryController で軌道追従

```yaml
sekirei_arm_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - arm_joint1
      - arm_joint2
      - arm_joint3
      - arm_joint4
      - arm_joint5
      - arm_joint6

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity
```

## Dynamixel Hardware の詳細設定

### モーターと関節の対応

実機では以下のマッピングが使用されます：

| 関節名 | Transmission | Dynamixel ID | GPIO名 |
|--------|--------------|--------------|--------|
| arm_joint1 | transmission[0] | 1 | dxl1 |
| arm_joint2 | transmission[1] | 2 | dxl2 |
| arm_joint3 | transmission[2] | 3 | dxl3 |
| arm_joint4 | transmission[3] | 4 | dxl4 |
| arm_joint5 | transmission[4] | 5 | dxl5 |
| arm_joint6 | transmission[5] | 6 | dxl6 |

### Transmission Matrix（変換行列）

1対1マッピングのため、6x6の単位行列を使用：

```xml
<param name="number_of_transmissions">6</param>
<param name="joint_to_transmission_matrix">1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1</param>
<param name="transmission_to_joint_matrix">1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1</param>
```

**Transmissionとは？**
- 物理的なモーター（Dynamixel）と論理的な関節（joint）の間の変換を定義
- 複雑な機構（例：2つのモーターで1つの関節、ギア比など）に対応可能
- 単純な1対1の場合は単位行列

**Matrix形式：**
```
joint_to_transmission (6x6):
[1, 0, 0, 0, 0, 0]   arm_joint1 → transmission[0]
[0, 1, 0, 0, 0, 0]   arm_joint2 → transmission[1]
[0, 0, 1, 0, 0, 0]   arm_joint3 → transmission[2]
[0, 0, 0, 1, 0, 0]   arm_joint4 → transmission[3]
[0, 0, 0, 0, 1, 0]   arm_joint5 → transmission[4]
[0, 0, 0, 0, 0, 1]   arm_joint6 → transmission[5]
```

### GPIO設定（Dynamixel固有パラメータ）

各モーター（dxl1〜dxl6）に対して以下を設定：

```xml
<gpio name="dxl1">
    <param name="type">dxl</param>
    <param name="ID">1</param>  <!-- Dynamixelモーターに設定したID -->

    <!-- 制御インターフェース -->
    <command_interface name="Goal Position"/>

    <!-- 状態インターフェース -->
    <state_interface name="Present Position"/>
    <state_interface name="Present Velocity"/>
    <state_interface name="Present Current"/>  <!-- 電流モニタリング -->

    <!-- PID制御パラメータ（XM540-W270のデフォルト値） -->
    <param name="Position P Gain">800</param>   <!-- 比例ゲイン -->
    <param name="Position I Gain">0</param>     <!-- 積分ゲイン -->
    <param name="Position D Gain">4000</param>  <!-- 微分ゲイン -->

    <!-- 回転方向 -->
    <param name="Drive Mode">0</param>  <!-- 0=時計回り, 1=反時計回り -->
</gpio>
```

**なぜ`<gpio>`タグ？**
- `<joint>`タグは標準的な制御インターフェース（position/velocity/effort）のみ
- `<gpio>`タグはDynamixel固有の設定（モーターID、PIDゲイン、Drive Modeなど）を定義
- dynamixel_hardware_interfaceが`<gpio>`を読み込み、USB経由でモーターに設定を書き込む

**通信経路：**
```
ROS2 MoveIt
  ↓
JointTrajectoryController (ros2_control)
  ↓
dynamixel_hardware_interface plugin
  ↓
/dev/ttyUSB0 (USB)
  ↓
U2D2/USB2Dynamixel アダプタ
  ↓
Dynamixel XM540-W270 モーター (ID: 1〜6)
```

### Dynamixelモーターの事前設定

実機を使用する前に、各モーターを以下のように設定してください：

1. **Dynamixel Wizard を使用：**
   ```bash
   # ROBOTIS公式ツール
   # https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/
   ```

2. **必須設定項目：**
   - **ID**: 1〜6 に設定（上記マッピング表参照）
   - **Baud Rate**: 1000000 (1Mbps)
   - **Operating Mode**: Position Control Mode (3)
   - **Torque**: 初期はOFF、起動後に自動でONになります

3. **推奨設定項目：**
   - **Position P/I/D Gain**: URDFのデフォルト値（P=800, I=0, D=4000）
   - **Drive Mode**: 回転方向を確認（必要に応じて反転）
   - **Velocity/Acceleration Profile**: 適切な速度制限を設定

### PIDゲインの調整

ロボットの動作が不安定な場合、PIDゲインを調整できます：

1. **URDFを編集：**
   ```bash
   vim ros_ws/src/urdf_test_node/urdf/sekirei_moveit.urdf
   ```

2. **調整の指針：**
   - **P Gain ↑**: 応答速度が速くなるが、振動しやすい（500〜1500推奨）
   - **I Gain**: 通常は0のまま（定常偏差がある場合のみ増やす）
   - **D Gain ↑**: 振動を抑制（3000〜5000推奨）

3. **再ビルド＆起動：**
   ```bash
   colcon build --packages-select sekirei_moveit_config
   source install/setup.bash
   ros2 launch sekirei_moveit_config demo.launch.py
   ```

### Drive Mode（回転方向）の調整

モーターの回転方向が逆の場合：

```xml
<!-- URDFで該当モーターのDrive Modeを変更 -->
<param name="Drive Mode">1</param>  <!-- 0→1で反転 -->
```

## MoveIt Planning Pipeline

### Planning Request Adapters (計画前処理)

`ompl_planning.yaml` で定義：
- `ResolveConstraintFrames`: 制約フレームの解決
- `ValidateWorkspaceBounds`: ワークスペース境界の検証
- `CheckStartStateBounds`: 開始状態の境界チェック
- `CheckStartStateCollision`: 開始状態の衝突チェック

### Planning Response Adapters (計画後処理)

**重要**: これらのアダプターが軌道に時間情報を追加します

```yaml
response_adapters:
  - default_planning_response_adapters/AddTimeOptimalParameterization
  - default_planning_response_adapters/ValidateSolution
  - default_planning_response_adapters/DisplayMotionPath
```

**AddTimeOptimalParameterization** について：
- OMPL が生成する kinematic trajectory には時間情報がありません
- このアダプターが各waypoint に `time_from_start` と速度/加速度を計算
- `joint_limits.yaml` で定義された速度・加速度制限を使用
- これがないと "Time between points is not strictly increasing" エラーが発生

## Joint Limits 設定

`joint_limits.yaml` でジョイントの動的制限を定義：

```yaml
joint_limits:
  arm_joint1:
    has_velocity_limits: true
    max_velocity: 3.14159  # rad/s
    has_acceleration_limits: true
    max_acceleration: 6.28318  # rad/s^2
  # ... 他のジョイントも同様
```

**重要ポイント：**
- Time-Optimal Parameterization には加速度制限が必須
- 値は実機の仕様に合わせて調整してください
- より保守的な値から始めて、徐々に上げることを推奨

## トラブルシューティング

### 問題1: "Time between points is not strictly increasing" エラー

**原因**: 軌道に時間情報が設定されていない

**解決策**:
1. `ompl_planning.yaml` に `response_adapters` が設定されているか確認
2. `joint_limits.yaml` に全ジョイントの加速度制限が定義されているか確認
3. `demo.launch.py` で `joint_limits.yaml` が正しくロードされているか確認

### 問題2: 計画は成功するが実行が ABORTED

**原因候補**:
1. コントローラーが起動していない
2. 開始位置と現在位置の差が大きすぎる
3. 軌道の時間パラメータが不適切

**確認コマンド**:
```bash
# コントローラーの状態確認
ros2 control list_controllers

# 期待される出力:
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# sekirei_arm_controller[joint_trajectory_controller/JointTrajectoryController] active

# トピックの確認
ros2 topic list | grep sekirei_arm_controller
```

### 問題3: 実機が動かない（dynamixel_hardware使用時）

**チェックリスト**:
1. USB デバイスが認識されているか: `ls -l /dev/ttyUSB*`
2. パーミッション設定: `sudo chmod 666 /dev/ttyUSB0`
3. Dynamixel モーターの電源が入っているか
4. モーターIDとボーレートが正しいか（URDF設定と一致）

**デバッグ方法**:
```bash
# ros2_control のログを確認
ros2 run tf2_ros tf2_echo world base_link

# ジョイント状態の確認
ros2 topic echo /joint_states
```

### 問題4: RViz2 が起動しない

**原因**: Docker環境でのディスプレイ設定

**解決策**:
```bash
# Dockerコンテナ起動時に --env DISPLAY=$DISPLAY を追加
# または demo.launch.py で use_rviz:=false として起動し、
# ホスト側から RViz2 を起動
```

## 高度な使い方

### プログラムから制御する

MoveIt の C++ API または Python API を使用：

```python
# Python example
from moveit_msgs.action import MoveGroup
import rclpy
from rclpy.action import ActionClient

# MoveGroupInterface を使用して制御
# 詳細は MoveIt2 tutorials を参照
```

### カスタム制約の追加

`ompl_planning.yaml` でプランナーのパラメータを調整可能：

```yaml
sekirei_arm:
  default_planner_config: RRTConnect
  longest_valid_segment_fraction: 0.005
```

### 速度・加速度のスケーリング

実行時に調整可能：
- RViz2 の UI から: Max Velocity / Max Acceleration スライダー
- コードから: `MotionPlanRequest.max_velocity_scaling_factor`

## 参考資料

- [MoveIt2 Documentation](https://moveit.picknik.ai/main/index.html)
- [ros2_control Documentation](https://control.ros.org/)
- [Time Parameterization Tutorial](https://moveit.picknik.ai/main/doc/examples/time_parameterization/time_parameterization_tutorial.html)

## よくある質問

**Q: Mock と実機を切り替えるには？**

A: `dynamixel_hardware` パッケージのインストール有無で自動的に切り替わります。手動で切り替える場合は、対応する URDF ファイルを指定してください。

**Q: 動作が遅い/速すぎる場合は？**

A: `joint_limits.yaml` の `max_velocity` と `max_acceleration` を調整するか、RViz2 の UI でスケーリングファクターを変更してください。

**Q: 計画された軌道を保存したい**

A: MoveIt の `robot_trajectory` を ROS bag に記録するか、`moveit_msgs/RobotTrajectory` をファイルに保存してください。

## ライセンス

このパッケージのライセンスについては、プロジェクトのルートディレクトリの LICENSE ファイルを参照してください。
