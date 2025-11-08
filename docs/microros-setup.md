# micro-ROS Setup Guide

## 概要

micro-ROSは専用コンテナで動作します。マイコン（STM32など）との通信を担当するmicro-ROS agentが含まれています。

## アーキテクチャ

```
┌─────────────────┐     ┌──────────────────┐     ┌──────────────┐
│  ros2_container │────▶│ microros_agent   │────▶│   Micro-     │
│   (ROS2 Jazzy)  │     │  (Agent running) │     │   controller │
└─────────────────┘     └──────────────────┘     └──────────────┘
        ▲                        │                      (STM32)
        │                        │
        │                   USB/Serial
        └────────────────────────┘
              DDS network
```

## コンテナ構成

### microros_agent コンテナ
- **ベースイメージ**: osrf/ros:jazzy-desktop-full
- **役割**: micro-ROS agentの実行
- **ネットワーク**: host mode（ROS2コンテナと同じDDSドメイン）
- **デバイス**: USB/Serial デバイスへのアクセス

## セットアップ

### 1. コンテナのビルド

```bash
# 全コンテナをビルド
make build

# または、micro-ROSのみ
docker compose build microros_agent
```

### 2. コンテナの起動

```bash
# 全コンテナを起動
make up

# または、特定のコンテナのみ
docker compose up -d microros_agent
```

### 3. コンテナに接続

```bash
# micro-ROSコンテナに入る
make shell-microros

# または
docker compose exec microros_agent bash
```

## micro-ROS Agent の起動

### 基本的な使い方

```bash
# コンテナに入る
make shell-microros

# STM32 Nucleo F446RE用（推奨）
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

# 他のSTM32ボード（ST-Link経由）
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

# USB-Serialアダプタ経由
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

# シリアルデバイス名を確認する場合
ls /dev/serial/by-id/*

# UDP経由（WiFi/Ethernet接続のマイコン用）
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# TCPサーバーモード
ros2 run micro_ros_agent micro_ros_agent tcp4 --port 8888

# 詳細ログを表示（デバッグ用）
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
```

### よく使うコマンド

```bash
# USBデバイスの確認
lsusb

# シリアルポートの確認
ls -l /dev/tty*

# 接続されているmicro-ROSノードの確認
ros2 node list

# micro-ROSからパブリッシュされているトピックの確認
ros2 topic list

# トピックの内容をモニタリング
ros2 topic echo /microROS/ping
```

### デバイスの確認

```bash
# USBデバイス一覧
lsusb

# シリアルポート確認
ls -l /dev/tty*

# 権限確認
ls -l /dev/ttyUSB0
ls -l /dev/ttyACM0
```

## トラブルシューティング

### 問題: デバイスが見つからない

**原因**: デバイスが接続されていない、または`docker-compose.yml`で指定されていない

**解決策**:
```bash
# ホストでデバイスを確認
ls -l /dev/ttyUSB* /dev/ttyACM*

# docker-compose.ymlのdevicesセクションを確認・修正
# 例: /dev/ttyUSB1 を使用する場合
devices:
  - /dev/ttyUSB1:/dev/ttyUSB1
```

### 問題: Permission denied

**原因**: デバイスへのアクセス権限がない

**解決策**:
```bash
# ホストでユーザーをdialoutグループに追加
sudo usermod -aG dialout $USER

# または、デバイスの権限を変更（一時的）
sudo chmod 666 /dev/ttyUSB0

# udevルールで永続化（推奨）
# /etc/udev/rules.d/99-microros.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666"

# udevをリロード
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 問題: コンテナ間通信ができない

**原因**: ROS_DOMAIN_IDが異なる、またはネットワーク設定の問題

**解決策**:
```bash
# 両方のコンテナでROS_DOMAIN_IDを確認
echo $ROS_DOMAIN_ID  # 両方とも0であることを確認

# トピック一覧を確認（両方のコンテナで）
ros2 topic list

# ノード一覧を確認
ros2 node list
```

### 問題: micro-ROS agentがクラッシュする

**原因**: マイコン側のプロトコルバージョン不一致、通信エラー

**解決策**:
```bash
# 詳細ログを有効化
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6

# マイコン側のmicro-ROSライブラリのバージョンを確認
# ROS2とmicro-ROSのバージョンを合わせる（両方Jazzy推奨）
```

## カスタマイズ

### docker-compose.ymlの調整

異なるデバイスを使用する場合：

```yaml
microros_agent:
  devices:
    - /dev/ttyUSB0:/dev/ttyUSB0
    - /dev/ttyUSB1:/dev/ttyUSB1  # 複数デバイス
    # または
    - /dev/serial/by-id/usb-STMicroelectronics_...:dev/stm32
```

### Dockerfileのカスタマイズ

追加パッケージが必要な場合、`microros_ws/Dockerfile`に追加：

```dockerfile
RUN apt-get update && apt-get install -y \
    your-package \
    && rm -rf /var/lib/apt/lists/*
```

## 開発ワークフロー

### 1. マイコン側の開発

```bash
# 1. マイコンのコードをビルド（別ターミナル）
cd your_mcu_project
make flash

# 2. micro-ROS agentを起動
make shell-microros
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

### 2. ROS2側との連携

```bash
# ROS2コンテナでトピックをサブスクライブ
make exec
ros2 topic echo /micro_ros_topic

# または、ROS2からマイコンにパブリッシュ
ros2 topic pub /micro_ros_cmd std_msgs/msg/String "data: 'test'"
```

## ベストプラクティス

1. **udevルールの設定**: デバイス名を固定して信頼性向上
2. **ログの確認**: `-v6`オプションで詳細ログを確認
3. **バージョン管理**: ROS2とmicro-ROSのバージョンを統一
4. **エラーハンドリング**: マイコン側で接続エラーを適切に処理
5. **テスト**: 単純なメッセージから始めて段階的に複雑化

## 参考リンク

- [micro-ROS公式ドキュメント](https://micro.ros.org/)
- [micro-ROS GitHub](https://github.com/micro-ROS)
- [micro-ROS Tutorials](https://micro.ros.org/docs/tutorials/)
