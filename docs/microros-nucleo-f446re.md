# micro-ROS Setup for STM32 Nucleo F446RE

## ハードウェア情報

- **ボード**: STM32 Nucleo F446RE
- **マイコン**: STM32F446RET6 (ARM Cortex-M4, 180MHz)
- **接続方法**: USB経由（ST-Link V2-1統合デバッガ）
- **シリアルポート**: `/dev/ttyACM0`（通常）

## 特徴

- USB 1本でデバッグとシリアル通信が可能
- Arduino互換ピン配置
- 豊富なペリフェラル（UART, SPI, I2C, CAN, ADC, DAC等）
- STM32CubeMXでの設定が容易

## セットアップ手順

### 前提条件

✅ **ファームウェアは既にDockerイメージにビルド済み**です！

Dockerfileで以下が自動実行されています：
- ファームウェアワークスペースの作成（Nucleo F446RE用）
- Ping Pongアプリの設定（Serial通信）
- ファームウェアのビルド

### 1. Dockerコンテナのビルド・起動

```bash
# コンテナをビルド（初回のみ、時間がかかります）
make build

# コンテナを起動
make up
```

### 2. Nucleoボードの接続

USBケーブルでNucleo F446REをPCに接続します。

### 3. ファームウェアのフラッシュ

```bash
# micro-ROSコンテナに入る
make shell-microros

# フラッシュ実行（OpenOCDは既にインストール済み）
ros2 run micro_ros_setup flash_firmware.sh
```

成功すると以下のようなログが表示されます：
```
Open On-Chip Debugger 0.11.0
...
target halted due to debug-request, current mode: Thread
...
** Programming Finished **
** Verify Started **
** Verified OK **
```

### 4. micro-ROS Agentの起動

ファームウェアがフラッシュされたら、Dockerコンテナでagentを起動：

```bash
# micro-ROSコンテナに入る
make shell-microros

# Agentを起動（Nucleoは通常 /dev/ttyACM0）
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

**デバイス名の確認方法**:
```bash
# ホスト側で確認
ls -l /dev/ttyACM*

# または詳細情報
ls /dev/serial/by-id/
# 出力例: usb-STMicroelectronics_STM32_STLink_...
```

### 5. 動作確認

別のターミナルでROS 2からトピックを確認：

```bash
# ROS2コンテナに入る
make exec

# Pingトピックを確認（5秒ごとにメッセージが来る）
ros2 topic echo /microROS/ping

# Fake Pingを送信してPongが返ることを確認
ros2 topic pub --once /microROS/ping std_msgs/msg/Header '{frame_id: "fake_ping"}'

# Pongトピックを確認
ros2 topic echo /microROS/pong
```

## よくある問題と解決策

### 問題1: `/dev/ttyACM0`が見つからない

**原因**: ボードが認識されていない、またはドライバの問題

**解決策**:
```bash
# ホスト側でデバイスを確認
lsusb | grep -i stm
# 出力例: Bus 001 Device 012: ID 0483:374b STMicroelectronics ST-LINK/V2.1

# カーネルログを確認
dmesg | tail | grep tty
# 出力例: cdc_acm 1-1:1.2: ttyACM0: USB ACM device

# 権限を確認
ls -l /dev/ttyACM0
```

### 問題2: Permission denied

**解決策**:
```bash
# ユーザーをdialoutグループに追加
sudo usermod -aG dialout $USER

# ログアウト・ログインして反映

# または一時的に権限変更
sudo chmod 666 /dev/ttyACM0
```

### 問題3: フラッシュに失敗する

**原因**: OpenOCDの設定やST-Linkドライバの問題

**解決策**:
```bash
# OpenOCDのバージョン確認
openocd --version

# 手動でフラッシュを試す
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program firmware/freertos_apps/microros_nucleo_f446re_extensions/build/micro-ROS.elf verify reset exit"
```

### 問題4: Agentに接続できない

**原因**: ボーレートの不一致、ファームウェアの問題

**解決策**:
```bash
# 詳細ログでデバッグ
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6

# ボーレートを明示的に指定（デフォルトは115200）
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200

# ファームウェアを再ビルド・再フラッシュ
```

## Nucleo F446REのピン配置

### UART通信（オプション）

ST-Link経由のUSB Serialではなく、外部UARTを使う場合：

| ピン | 機能 | 説明 |
|-----|------|------|
| PA2 | USART2_TX | Arduino D1 |
| PA3 | USART2_RX | Arduino D0 |
| GND | Ground | - |

設定変更が必要な場合は`configure_firmware.sh`実行前に：
```bash
# 環境変数でUARTを指定（詳細は micro_ros_setup ドキュメント参照）
export MICRO_ROS_UART=USART2
```

## カスタムアプリケーションの作成

Ping Pong以外のアプリを作成する場合：

```bash
# firmware/freertos_apps/apps/ にカスタムアプリを配置
cd firmware/freertos_apps/apps/
mkdir my_app
cd my_app

# app.c と app-colcon.meta を作成
# （詳細は公式ドキュメント参照）

# ビルド前に設定
cd ~/microros_ws
ros2 run micro_ros_setup configure_firmware.sh my_app --transport serial
ros2 run micro_ros_setup build_firmware.sh
```

## パフォーマンスチューニング

### メモリ最適化

STM32F446REは512KBのFlashと128KBのRAMを搭載。micro-ROSはメモリを消費するため：

- 不要なROS 2機能を無効化
- メッセージサイズを最小化
- QoSをBEST_EFFORTに設定（信頼性よりパフォーマンス優先）

### FreeRTOSの設定

`firmware/freertos_apps/`のFreeRTOSConfig.hで調整：
```c
#define configTOTAL_HEAP_SIZE        ( ( size_t ) ( 64 * 1024 ) )  // ヒープサイズ
#define configMINIMAL_STACK_SIZE     ( ( unsigned short ) 256 )    // スタックサイズ
```

## 参考リンク

- [STM32 Nucleo F446RE仕様](https://www.st.com/en/evaluation-tools/nucleo-f446re.html)
- [micro-ROS公式ドキュメント](https://micro.ros.org/)
- [FreeRTOS公式サイト](https://www.freertos.org/)
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
