# micro-ROS Ping Pong Test

## 概要

micro-ROSのサンプルアプリケーション「Ping Pong」を使って、マイコンとROS2の通信をテストします。

## Ping Pongアプリの動作

```
┌─────────────────────────────────────────────────────────────┐
│                    micro-ROS Node (MCU)                      │
│                                                              │
│  ┌──────────────┐              ┌──────────────┐            │
│  │ Ping Publisher│─────────────▶│ Ping Topic   │            │
│  │ (5秒ごと)     │              │ /microROS/ping│           │
│  └──────────────┘              └──────────────┘            │
│                                                              │
│  ┌──────────────┐              ┌──────────────┐            │
│  │Ping Subscriber│◀─────────────│  Fake Ping   │            │
│  │               │              │              │            │
│  └───────┬──────┘              └──────────────┘            │
│          │                                                   │
│          │ 受信したら自動的にPongを返す                         │
│          ▼                                                   │
│  ┌──────────────┐              ┌──────────────┐            │
│  │ Pong Publisher│─────────────▶│ Pong Topic   │            │
│  │               │              │ /microROS/pong│           │
│  └──────────────┘              └──────────────┘            │
└─────────────────────────────────────────────────────────────┘
```

## テスト手順

### 1. micro-ROS Agentの起動

```bash
# ターミナル1: micro-ROSコンテナに入る
make shell-microros

# Agentを起動（シリアルポートは環境に合わせて変更）
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

成功すると以下のようなログが表示されます：
```
[1731899999.123456] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3
[1731899999.234567] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
```

### 2. マイコンからのPingを確認

```bash
# ターミナル2: ROS2コンテナに入る
make exec

# Pingトピックをサブスクライブ
ros2 topic echo /microROS/ping
```

5秒ごとにマイコンからのPingメッセージが表示されます：
```
stamp:
  sec: 20
  nanosec: 867000000
frame_id: '1344887256_1085377743'
---
stamp:
  sec: 25
  nanosec: 942000000
frame_id: '730417256_1085377743'
---
```

### 3. Fake Pingを送信してPongをテスト

```bash
# ターミナル3: Pongトピックをサブスクライブ
make exec
ros2 topic echo /microROS/pong
```

```bash
# ターミナル4: Fake Pingを送信
make exec
ros2 topic pub --once /microROS/ping std_msgs/msg/Header '{frame_id: "fake_ping"}'
```

成功すると、ターミナル3でPongが表示されます：
```
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
---
```

## 期待される動作

### 正常な場合
1. **定期的なPing送信**: マイコンが5秒ごとにPingを送信
2. **Fake Pingへの応答**: ROS2から送信したFake Pingに対してPongで応答
3. **フレームIDの保持**: 送信したframe_idがPongでそのまま返される

### 問題がある場合のトラブルシューティング

#### Pingが表示されない
```bash
# Agent側で詳細ログを確認
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6

# マイコン側の接続を確認
# - ボーレートの確認（デフォルト: 115200）
# - シリアルポートの権限確認
ls -l /dev/ttyUSB0
```

#### Fake Pingに応答しない
- マイコン側のPing Subscriberが正しく動作しているか確認
- トピック名が一致しているか確認: `/microROS/ping`
- ROS_DOMAIN_IDが一致しているか確認（デフォルト: 0）

#### パフォーマンスの問題
- QoS設定を確認（BEST_EFFORT vs RELIABLE）
- ネットワーク帯域を確認
- シリアル通信のボーレートを確認

## 複数のPing Pongノード

複数のマイコンを接続した場合、それぞれがお互いのPingに応答します：

```
Ping send seq 1711620172_1742614911                      # 自分のPingを送信
Pong for seq 1711620172_1742614911 (1)                   # 1台目が応答
Pong for seq 1711620172_1742614911 (2)                   # 2台目が応答
Pong for seq 1711620172_1742614911 (3)                   # 3台目が応答
Ping received with seq 1845948271_546591567. Answering.  # 他のノードのPingを受信して応答
```

## 次のステップ

1. **カスタムメッセージの送信**: `std_msgs`以外のメッセージタイプを試す
2. **QoS設定の変更**: 信頼性とパフォーマンスのトレードオフを調整
3. **独自アプリケーションの作成**: Ping Pongを参考に独自のノードを作成

## 参考

- [micro-ROS公式チュートリアル](https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/)
- Ping Pongアプリのソースコード: `firmware/freertos_apps/apps/ping_pong/app.c`
