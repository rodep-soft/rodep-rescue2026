# Flutter UIによるロボットアーム制御

## 概要

このドキュメントでは、Flutter UIを使用してROS2 MoveItで制御されるロボットアームを操作する方法について説明します。

## 実装した機能

### 1. Named Poses (プリセットポーズ)

8つのプリセットポーズを定義し、ワンクリックでロボットアームを所定の姿勢に移動できます。

- **Home** - ホームポジション (全関節0度)
- **Ready** - 待機姿勢
- **Up** - 上向き姿勢
- **Forward** - 前方伸展姿勢
- **Compact** - コンパクト姿勢
- **Left** - 左向き姿勢
- **Right** - 右向き姿勢
- **Back** - 後方姿勢

### 2. システムアーキテクチャ

```
Flutter UI (localhost)
    ↓ WebSocket (ws://localhost:9090)
rosbridge_server
    ↓ ROS2 Topics
moveit_bridge.py
    ↓ MoveGroup Action
MoveIt2 + ros2_control
    ↓ Joint Commands
Robot Hardware (Dynamixel or Mock)
```

### 3. 主要コンポーネント

#### moveit_bridge.py
- **役割**: FlutterからのコマンドをMoveItのアクションに変換
- **Subscribe**: `/move_to_pose` (std_msgs/String)
- **Publish**: `/arm_status` (std_msgs/String)
- **機能**: Named Poseへの移動、状態フィードバック

#### rosbridge_server
- **役割**: WebSocketプロトコルでROS2トピックをJSON APIとして公開
- **ポート**: 9090
- **アドレス**: 127.0.0.1 (localhost)

#### Flutter UI
- **プラットフォーム**: Linux Desktop (Docker内)
- **WebSocket**: web_socket_channel パッケージ使用
- **機能**: Named Poses制御、カメラ映像表示、カスタムトピック購読

## 起動方法

### 前提条件

- Docker と docker-compose がインストールされていること
- X11 サーバーが動作していること（GUI表示用）

### 1. ROS2システムの起動

```bash
# Docker Composeでコンテナを起動
docker-compose up -d ros2_container

# ROS2システムを起動（rosbridge有効）
docker-compose exec ros2_container bash -c '
  cd /root/ros_ws &&
  source /opt/ros/jazzy/setup.bash &&
  source install/setup.bash &&
  ros2 launch sekirei_moveit_config demo.launch.py use_rosbridge:=true
'
```

このコマンドで以下が起動します:
- RViz2（可視化）
- MoveIt2 move_group（モーションプランニング）
- ros2_control（コントローラー管理）
- joint_state_broadcaster（関節状態配信）
- sekirei_arm_controller（アームコントローラー）
- rosbridge_websocket（Flutter連携）

### 2. moveit_bridgeの起動

```bash
docker-compose exec -d ros2_container bash -c '
  cd /root/ros_ws &&
  source /opt/ros/jazzy/setup.bash &&
  source install/setup.bash &&
  ros2 run sekirei_moveit_config moveit_bridge.py
'
```

### 3. Flutter UIの起動

```bash
# Flutterコンテナを起動
docker-compose up -d flutter

# Flutter UIを実行
docker-compose exec flutter bash -c '
  cd robot_ui &&
  flutter run -d linux
'
```

### 起動確認

ROS2ノードが正常に起動しているか確認:

```bash
docker-compose exec ros2_container bash -c '
  source /opt/ros/jazzy/setup.bash &&
  ros2 node list
'
```

以下のノードが表示されればOK:
- `/move_group`
- `/controller_manager`
- `/rosbridge_websocket`
- `/moveit_bridge`
- `/rviz2`

## 使用方法

### Flutter UIでのNamed Poses制御

1. Flutter UIが起動すると、メインウィンドウが表示されます
2. "Named Poses"セクションに8つのボタンが表示されます
3. 任意のボタンをクリックすると、ロボットアームがそのポーズに移動します
4. RViz2でロボットの動きをリアルタイムで確認できます
5. `/arm_status`トピックでステータスがフィードバックされます

### コマンドラインでのテスト

Flutter UIを使わずにコマンドラインでテストする場合:

```bash
# "home"ポーズに移動
docker-compose exec ros2_container bash -c '
  source /opt/ros/jazzy/setup.bash &&
  ros2 topic pub --once /move_to_pose std_msgs/msg/String "{data: home}"
'

# "ready"ポーズに移動
docker-compose exec ros2_container bash -c '
  source /opt/ros/jazzy/setup.bash &&
  ros2 topic pub --once /move_to_pose std_msgs/msg/String "{data: ready}"
'
```

## ハードウェア自動検出

システムは起動時に自動的にハードウェアを検出します:

### 実機モード（Dynamixel接続時）
- `/dev/ttyUSB*` デバイスが検出された場合
- `dynamixel_hardware_interface` パッケージが存在する場合
- → `sekirei_moveit.urdf` を使用（実機制御）

### モックモード（デバイス未接続時）
- USBデバイスが検出されない場合
- → `sekirei_moveit_dummy.urdf` を使用（シミュレーション）
- → `mock_components/GenericSystem` で動作

## トラブルシューティング

### rosbridgeに接続できない

**症状**: Flutter UIで "Disconnected" と表示される

**確認事項**:
```bash
# rosbridgeが起動しているか確認
docker-compose exec ros2_container bash -c '
  source /opt/ros/jazzy/setup.bash &&
  ros2 node list | grep rosbridge
'

# ポート9090がリスンしているか確認（ホストマシンで実行）
ss -tlnp | grep 9090
# → 127.0.0.1:9090 でリスンしていることを確認
```

**解決方法**:
- rosbridge_websocketノードを再起動
- Flutter UIのWebSocketアドレスが `localhost:9090` になっているか確認

### moveit_bridgeが動作しない

**症状**: Named Posesボタンを押してもロボットが動かない

**確認事項**:
```bash
# moveit_bridgeが起動しているか
docker-compose exec ros2_container bash -c '
  source /opt/ros/jazzy/setup.bash &&
  ros2 node list | grep moveit_bridge
'

# トピックが存在するか
docker-compose exec ros2_container bash -c '
  source /opt/ros/jazzy/setup.bash &&
  ros2 topic list | grep -E "move_to_pose|arm_status"
'
```

**解決方法**:
- moveit_bridge.pyを再起動
- move_groupが正常に動作しているか確認

### RViz2が表示されない

**症状**: RViz2ウィンドウが表示されない

**解決方法**:
```bash
# DISPLAY環境変数を確認（ホストマシンで）
echo $DISPLAY

# X11転送が有効か確認
xhost +local:docker

# docker-compose.ymlでDISPLAY環境変数が設定されているか確認
```

### Flutter UIでレンダリングエラー

**症状**: 大量のエラーメッセージが表示される

これらのエラーは通常は無視できます（デバッグモードの警告）。UIは正常に動作します。

## 設定ファイル

### demo.launch.py
- **パス**: `ros_ws/src/sekirei_moveit_config/launch/demo.launch.py`
- **設定項目**:
  - `use_rosbridge`: rosbridgeを有効化 (default: false)
  - rosbridge address: `127.0.0.1`
  - rosbridge port: `9090`

### moveit_bridge.py
- **パス**: `ros_ws/src/sekirei_moveit_config/scripts/moveit_bridge.py`
- **設定項目**:
  - Named Posesの定義（joint値）
  - MoveGroup設定
  - 実行タイムアウト

### main.dart
- **パス**: `flutter/robot_ui/lib/main.dart`
- **設定項目**:
  - WebSocket address: `localhost`
  - WebSocket port: `9090`

## 開発履歴

### 実装内容
1. Named Poses機能の追加（sekirei.srdf）
2. moveit_bridgeノードの作成
3. rosbridge統合（demo.launch.py）
4. Flutter UI拡張（Named Posesボタン）
5. localhost接続への修正（Tailscale IP問題の解決）
6. USB自動検出機能の実装

### 技術的な課題と解決

#### 課題1: rosbridgeがTailscale IPにバインドされる
**問題**: rosbridge_serverがホストのTailscale IP (100.109.100.122) にバインドされ、localhostからアクセスできない

**解決**: demo.launch.pyのrosbridge設定で明示的に `address: '127.0.0.1'` を指定

#### 課題2: WebSocket接続がリセットされる
**問題**: addressパラメータが効かず、WebSocketハンドシェイクに失敗

**解決**: addressを文字列 `'127.0.0.1'` として明示的に設定することで解決

#### 課題3: Docker内でのハードウェア検出
**問題**: 実機とモックを手動で切り替える必要がある

**解決**: `/dev/ttyUSB*` デバイスの存在を自動検出し、適切なURDFを選択

## 今後の拡張案

- [ ] Joint単位での個別制御
- [ ] カスタムポーズの保存・呼び出し機能
- [ ] 軌道の記録と再生
- [ ] カメラ映像と同期した制御
- [ ] 複数ロボットの同時制御
- [ ] モバイルアプリ版の実装

## 参考資料

- [rosbridge_suite Documentation](http://wiki.ros.org/rosbridge_suite)
- [MoveIt2 Documentation](https://moveit.picknik.ai/main/index.html)
- [Flutter WebSocket](https://pub.dev/packages/web_socket_channel)
- [ros2_control Documentation](https://control.ros.org/)
