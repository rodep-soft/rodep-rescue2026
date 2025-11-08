# Clang-Tidy Setup Guide

## 概要

clang-tidyは、C++コードの静的解析ツールです。バグの可能性、パフォーマンス問題、モダンC++への改善提案などを検出します。

## 設定ファイル

`ros_ws/src/.clang-tidy` - ROS2 C++プロジェクト用の設定

### 有効なチェック

#### バグ検出（bugprone）
- 一般的なバグパターンの検出
- 例外処理の問題
- メモリリークの可能性

#### C++ Core Guidelines（cppcoreguidelines）
- モダンC++のベストプラクティス
- リソース管理
- 型安全性

#### Google Style（google）
- Google C++ Style Guideに準拠
- 命名規則のチェック

#### モダン化（modernize）
- C++11/14/17の新機能の使用を推奨
- 古いコードパターンの置き換え

#### パフォーマンス（performance）
- 不要なコピーの検出
- 効率的なアルゴリズムの提案

#### 可読性（readability）
- コードの可読性向上
- 命名規則の統一

### 命名規則

ROS2標準に準拠：
- **クラス/構造体**: `CamelCase`
- **関数**: `camelBack`
- **変数/パラメータ**: `lower_case`
- **プライベートメンバー**: `lower_case_`（末尾アンダースコア）
- **定数**: `UPPER_CASE`

## 使い方

### ⚠️ 重要: 初回セットアップ

clang-tidyを使用する前に、必ず**compile_commands.json**を生成してください：

```bash
# Dockerコンテナ内で実行
make exec
cd ros_ws
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# または、ホストで実行
cd ros_ws
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Makefileコマンド

```bash
# 静的解析を実行（修正なし）
make tidy

# 自動修正を適用
make tidy-fix
```

**注意**: compile_commands.jsonがない場合、自動的にエラーメッセージが表示されます。

### コマンドライン

```bash
# 単一ファイル
clang-tidy --config-file=ros_ws/src/.clang-tidy ros_ws/src/your_package/src/your_file.cpp

# 自動修正付き
clang-tidy --config-file=ros_ws/src/.clang-tidy --fix ros_ws/src/your_package/src/your_file.cpp

# 複数ファイル
find ros_ws/src/your_package -name '*.cpp' -exec clang-tidy --config-file=ros_ws/src/.clang-tidy {} +
```

### pre-commitフック

コミット時に自動実行されます：

```bash
git add ros_ws/src/my_package/src/my_file.cpp
git commit -m "Add feature"
# → clang-tidyが自動実行される
```

一時的にスキップする場合：
```bash
SKIP=clang-tidy git commit -m "Skip tidy check"
```

## VS Code統合

### 拡張機能

以下をインストール：
- **clangd** (llvm-vs-code-extensions.vscode-clangd)

### 設定

`.vscode/settings.json`に追加：

```json
{
  "clangd.arguments": [
    "--clang-tidy",
    "--compile-commands-dir=${workspaceFolder}/ros_ws/build",
    "--background-index",
    "--completion-style=detailed"
  ],
  "clangd.path": "/usr/bin/clangd"
}
```

### compile_commands.jsonの生成

clang-tidyとclangdが正しく動作するには、compile_commands.jsonが必要です：

```bash
# ROS2パッケージをビルド
cd ros_ws
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# または、Dockerコンテナ内で
make exec
cd ros_ws
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## よくある警告と対処法

### 1. modernize-use-auto
**警告**: 型が明示的すぎる

```cpp
// Before
std::vector<int>::iterator it = vec.begin();

// After
auto it = vec.begin();
```

### 2. readability-identifier-naming
**警告**: 命名規則違反

```cpp
// Before (NG)
class my_class {
  int MyVariable;
};

// After (OK)
class MyClass {
  int my_variable_;  // プライベートメンバーは末尾_
};
```

### 3. performance-unnecessary-copy-initialization
**警告**: 不要なコピー

```cpp
// Before
void process(const std::string& str) {
  std::string copy = str;  // 不要なコピー
  // ...
}

// After
void process(const std::string& str) {
  const auto& ref = str;  // 参照を使用
  // ...
}
```

### 4. cppcoreguidelines-avoid-const-or-ref-data-members
**警告**: constまたは参照メンバー変数

```cpp
// Before (NG)
class Node {
  const std::string& name_;
};

// After (OK)
class Node {
  std::string name_;
};
```

### 5. bugprone-unused-return-value
**警告**: 戻り値を使用していない

```cpp
// Before
vec.empty();  // 戻り値を無視

// After
if (vec.empty()) {
  // ...
}
```

## チェックのカスタマイズ

### 特定のチェックを無効化

`.clang-tidy`の`Checks:`に追加：

```yaml
Checks: >
  -*,
  bugprone-*,
  -bugprone-easily-swappable-parameters,  # このチェックを無効化
```

### コード内で一時的に無効化

```cpp
// NOLINTBEGIN
// この間のコードはclang-tidyチェックをスキップ
void legacy_code() {
  // ...
}
// NOLINTEND

// 特定の行のみ
int magic = 42;  // NOLINT(readability-magic-numbers)
```

## トラブルシューティング

### 問題: clang-tidyが遅い

**解決策**: 並列実行を有効化

```bash
# parallel-clang-tidyを使用（インストールが必要）
sudo apt-get install clang-tidy-14 python3-pip
pip3 install run-clang-tidy

run-clang-tidy -p ros_ws/build -j$(nproc)
```

### 問題: compile_commands.jsonが見つからない

**解決策**: ROS2パッケージをビルド

```bash
cd ros_ws
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### 問題: 誤検出が多い

**解決策**: `.clang-tidy`で特定のチェックを無効化

```yaml
Checks: >
  -*,
  bugprone-*,
  -bugprone-easily-swappable-parameters,  # 誤検出が多い
```

### 問題: pre-commitでタイムアウト

**解決策**: `.pre-commit-config.yaml`でタイムアウトを延長

```yaml
-   id: clang-tidy
    args: [--config-file=ros_ws/src/.clang-tidy]
    # タイムアウトを延長（デフォルト: 60秒）
    timeout: 300
```

## ベストプラクティス

1. **段階的導入**: 既存コードベースには徐々に適用
2. **警告の優先順位**: bugprone > performance > readability
3. **チームで共有**: `.clang-tidy`をリポジトリにコミット
4. **CI統合**: プルリクエスト時に自動チェック
5. **定期実行**: `make tidy`を定期的に実行

## CI/CD統合例

### GitHub Actions

```yaml
- name: Run clang-tidy
  run: |
    cd ros_ws
    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    cd ..
    make tidy
```

## 参考リンク

- [clang-tidy公式ドキュメント](https://clang.llvm.org/extra/clang-tidy/)
- [チェック一覧](https://clang.llvm.org/extra/clang-tidy/checks/list.html)
- [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Developer-Guide.html)
