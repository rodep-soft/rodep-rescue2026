# Clang-Tidy Quick Start

## TL;DR

clang-tidyは**compile_commands.json**が必要なので、pre-commitでは無効化しています。

手動で実行する場合のみ使用してください。

## セットアップ（1回のみ）

```bash
# 1. Dockerコンテナに入る
make exec

# 2. ROS2ワークスペースをビルド（compile_commands.json生成）
cd ros_ws
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## 使い方

### ホストから実行

```bash
# 静的解析のみ
make tidy

# 自動修正も適用
make tidy-fix
```

### コンテナ内から実行

```bash
make exec
cd /root/ros_ws

# 単一パッケージのみ
clang-tidy -p build --config-file=src/.clang-tidy src/your_package/src/your_file.cpp
```

## pre-commitでの動作

clang-tidyは**pre-commitで無効化**されています。

理由：
- compile_commands.jsonの依存関係
- 実行時間が長い（数十秒〜数分）
- 膨大な警告が出る可能性

**推奨**: コミット前に手動で`make tidy`を実行

## 現在有効なチェック（最小限）

最も重要なチェックのみ有効化：

- **bugprone-\***: バグ検出
- **performance-\***: パフォーマンス改善
- **modernize-use-nullptr**: NULL → nullptr
- **modernize-use-override**: override明示
- **modernize-use-auto**: auto使用推奨

命名規則チェックなど、警告が多すぎるものは無効化しています。

## カスタマイズ

より厳格なチェックが必要な場合は、`ros_ws/src/.clang-tidy`を編集してください。

詳細は`docs/clang-tidy-guide.md`を参照。
