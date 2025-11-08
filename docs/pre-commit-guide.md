# Pre-commit Setup Guide

## 概要

このプロジェクトではpre-commitを使用してコード品質を保っています。

## インストール済みフック

### 1. 基本チェック（pre-commit-hooks）
- **check-yaml** - YAMLファイルの構文チェック
- **end-of-file-fixer** - ファイル末尾に改行を追加
- **trailing-whitespace** - 行末の空白を削除

### 2. clang-format
- **対象**: `ros_ws/src/`配下のC++ファイル（`.cpp`, `.hpp`, `.h`, `.cc`, `.cxx`）
- **除外**:
  - 外部ライブラリ（eProsima, ros2, micro_ros_setup）
  - ビルド成果物（build/, install/, log/）
  - その他の一時ファイル

## セットアップ手順

### 1. pre-commitのインストール

```bash
# pipでインストール
pip install pre-commit

# または、aptでインストール（Ubuntu/Debian）
sudo apt-get install pre-commit
```

### 2. フックのインストール

```bash
# プロジェクトルートで実行
cd /home/tatsv/working/rodep-rescue2026
pre-commit install

# 成功メッセージ：
# pre-commit installed at .git/hooks/pre-commit
```

### 3. 既存ファイルをフォーマット（初回のみ推奨）

```bash
# 全ファイルに対してフックを実行
pre-commit run --all-files

# または、clang-formatのみ実行
pre-commit run clang-format --all-files
```

## 使い方

### 通常のコミット

pre-commitが自動的に実行されます：

```bash
git add .
git commit -m "Your commit message"

# → pre-commitフックが自動実行
# → フォーマットエラーがあれば自動修正
# → 修正された場合はコミットが中断されるので、再度addして再コミット
```

**フォーマット修正があった場合の例：**
```bash
$ git commit -m "Add feature"
check yaml...............................................................Passed
Trim Trailing Whitespace.................................................Passed
Fix End of Files.........................................................Failed
- hook id: end-of-file-fixer
- exit code: 1
- files were modified by this hook

Fixing ros_ws/src/my_package/src/my_file.cpp

clang-format.............................................................Failed
- hook id: clang-format
- files were modified by this hook

# ファイルが修正されたので、再度addしてコミット
$ git add .
$ git commit -m "Add feature"
# → 今度は全てPassedになる
```

### 特定のファイルのみチェック

```bash
# ステージングされているファイルのみ
pre-commit run

# 特定のファイルを指定
pre-commit run --files ros_ws/src/my_package/src/my_file.cpp
```

### フックをスキップ（緊急時のみ）

```bash
# 全フックをスキップ
git commit --no-verify -m "Emergency fix"

# 環境変数でスキップ
SKIP=clang-format git commit -m "Skip only clang-format"
```

## メンテナンス

### フックの更新

```bash
# 最新版に自動更新
pre-commit autoupdate

# 手動で.pre-commit-config.yamlのrevを変更してもOK
```

### フックの一覧表示

```bash
pre-commit run --all-files --verbose
```

### キャッシュのクリア

```bash
pre-commit clean
```

### フックのアンインストール

```bash
pre-commit uninstall
```

## トラブルシューティング

### 問題: pre-commitが実行されない

**解決策:**
```bash
# フックが正しくインストールされているか確認
ls -la .git/hooks/pre-commit

# 再インストール
pre-commit uninstall
pre-commit install
```

### 問題: clang-formatが見つからない

**解決策:**
```bash
# clang-formatをインストール
sudo apt-get install clang-format

# または特定バージョン
sudo apt-get install clang-format-14
```

### 問題: フォーマットが期待通りにならない

**解決策:**
```bash
# .clang-formatファイルが正しい場所にあるか確認
ls -la ros_ws/src/.clang-format

# 手動でテスト
clang-format --style=file --dry-run ros_ws/src/your_package/src/your_file.cpp
```

### 問題: 外部ライブラリがフォーマットされる

**解決策:**
`.pre-commit-config.yaml`のexcludeパターンを確認・追加：
```yaml
exclude: |
  (?x)^(
      ros_ws/src/your_external_lib/.*|
      ...
  )$
```

## CI/CD統合

### GitHub Actions

`.github/workflows/pre-commit.yml`を作成：

```yaml
name: Pre-commit

on: [push, pull_request]

jobs:
  pre-commit:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v3
    - uses: actions/setup-python@v4
      with:
        python-version: '3.x'
    - name: Install clang-format
      run: sudo apt-get install -y clang-format
    - uses: pre-commit/action@v3.0.0
```

### GitLab CI

`.gitlab-ci.yml`に追加：

```yaml
pre-commit:
  image: python:3.11
  before_script:
    - apt-get update && apt-get install -y clang-format
    - pip install pre-commit
  script:
    - pre-commit run --all-files
```

## カスタマイズ

### 新しいフックの追加

`.pre-commit-config.yaml`に追加：

```yaml
# Python用（Black）
-   repo: https://github.com/psf/black
    rev: 24.10.0
    hooks:
    -   id: black
        files: ^ros_ws/launch/.*\.py$

# CMake用
-   repo: https://github.com/cheshirekow/cmake-format-precommit
    rev: v0.6.13
    hooks:
    -   id: cmake-format
        files: ^ros_ws/src/.*/CMakeLists\.txt$

# Markdownリンター
-   repo: https://github.com/igorshubovych/markdownlint-cli
    rev: v0.41.0
    hooks:
    -   id: markdownlint
```

## ベストプラクティス

1. **コミット前に手動実行**: 大きな変更の前に`pre-commit run --all-files`を実行
2. **定期的な更新**: `pre-commit autoupdate`を月1回実行
3. **チーム共有**: 全メンバーが`pre-commit install`を実行することを確認
4. **CI統合**: CIでもpre-commitを実行してダブルチェック
5. **除外パターンの文書化**: 特定のファイルを除外する理由をコメントで記載

## 参考リンク

- [pre-commit公式ドキュメント](https://pre-commit.com/)
- [サポートされているフック一覧](https://pre-commit.com/hooks.html)
- [clang-format mirror](https://github.com/pre-commit/mirrors-clang-format)
