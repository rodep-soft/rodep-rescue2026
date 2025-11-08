# Clang-Format Configuration Guide

## 概要

このプロジェクトでは、ROS2のC++ファイルにのみclang-formatを適用します。

## ファイル構成

- **`ros_ws/src/.clang-format`** - ROS2 C++コード用のフォーマット設定
- **ルートの`.clang-format`** - 削除またはバックアップ推奨（混乱を避けるため）

## ROS2 C++コードのフォーマット設定

### 主な特徴

- **スタイル**: Google C++ Style Guideベース（ROS2標準）
- **インデント**: 2スペース
- **行の長さ**: 100文字
- **ポインタ**: 左寄せ (`int* ptr` not `int *ptr`)
- **C++標準**: C++17

### ヘッダーインクルードの順序

1. ROS2コアヘッダー (`rclcpp`, `rcl`, など)
2. ROS2メッセージ (`std_msgs`, `sensor_msgs`, など)
3. その他のROS2パッケージ (`tf2`, など)
4. C++標準ライブラリ
5. プロジェクト固有ヘッダー

## 使い方

### VS Codeでの自動フォーマット

`.vscode/settings.json`に以下を追加：

```json
{
  "[cpp]": {
    "editor.defaultFormatter": "xaver.clang-format",
    "editor.formatOnSave": true
  },
  "clang-format.executable": "/usr/bin/clang-format",
  "clang-format.style": "file"
}
```

### コマンドラインでフォーマット

```bash
# 単一ファイル
clang-format -i ros_ws/src/your_package/src/your_file.cpp

# パッケージ全体
find ros_ws/src/your_package -name '*.cpp' -o -name '*.hpp' | xargs clang-format -i

# ros_ws/src配下すべて
find ros_ws/src -name '*.cpp' -o -name '*.hpp' | xargs clang-format -i
```

### Makefileコマンド（追加推奨）

`Makefile`に以下を追加すると便利です：

```makefile
format-cpp:
	@echo "Formatting ROS2 C++ files..."
	@find ros_ws/src -name '*.cpp' -o -name '*.hpp' | xargs clang-format -i
	@echo "Done!"

check-format:
	@echo "Checking C++ formatting..."
	@find ros_ws/src -name '*.cpp' -o -name '*.hpp' | xargs clang-format --dry-run --Werror
```

## Git統合（pre-commit）

このプロジェクトでは既に**pre-commit**を使用しています。

### 設定を確認・更新

`.pre-commit-config.yaml`を確認：

```yaml
-   repo: https://github.com/pre-commit/mirrors-clang-format
    rev: v21.1.2
    hooks:
    -   id: clang-format
        # ROS2 C++ files only (in ros_ws/src/)
        files: ^ros_ws/src/.*\.(cpp|hpp|h|cc|cxx)$
        exclude: |
          (?x)^(
              ros_ws/src/eProsima/.*|
              ros_ws/src/ros2/.*|
              ros_ws/src/micro_ros_setup/.*|
              # ... other excludes
          )$
```

### pre-commitのインストール・更新

```bash
# pre-commitがインストールされていない場合
pip install pre-commit

# pre-commitフックをインストール（初回のみ）
pre-commit install

# 設定を更新した場合
pre-commit autoupdate

# 全ファイルに対して手動実行
pre-commit run --all-files

# 特定のフックのみ実行
pre-commit run clang-format --all-files
```

### コミット時の動作

コミット時に自動的にclang-formatが実行されます：

```bash
git add ros_ws/src/your_package/src/your_file.cpp
git commit -m "Add new feature"
# → clang-formatが自動実行され、フォーマットが修正される
```

### pre-commitをスキップする（緊急時のみ）

```bash
git commit --no-verify -m "Emergency fix"
```

## 特定パッケージを除外する場合

パッケージのルートに空の`.clang-format`を作成すると、そのパッケージではフォーマットが無効化されます：

```bash
# 例: eProsima（外部ライブラリ）を除外
touch ros_ws/src/eProsima/.clang-format
echo "DisableFormat: true" > ros_ws/src/eProsima/.clang-format
```

## トラブルシューティング

### clang-formatがインストールされていない場合

```bash
# Ubuntu/Debian
sudo apt-get install clang-format

# 特定バージョン（推奨: 14以上）
sudo apt-get install clang-format-14
```

### VS Code拡張機能

以下の拡張機能をインストール：
- **C/C++** (ms-vscode.cpptools)
- **Clang-Format** (xaver.clang-format)

## 参考リンク

- [ROS2 C++ Style Guide](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- [Clang-Format Documentation](https://clang.llvm.org/docs/ClangFormat.html)
- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
