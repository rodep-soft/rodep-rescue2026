# Git & Docker 効率化ガイド

このドキュメントでは、プロジェクトのgitとdocker管理を効率化するために追加されたツールの使い方を説明します。

## 📋 追加されたファイル

1. **`.gitignore`** - 拡張版（ROS2、Flutter、Docker関連を追加）
2. **`.dockerignore`** - Dockerビルドコンテキストの最適化
3. **`Makefile`** - 便利なコマンドショートカット
4. **`.git-aliases.sh`** - Gitエイリアス集

---

## 🐳 Docker管理（Makefile）

### 基本コマンド

```bash
# ヘルプを表示
make help

# イメージをビルド
make build

# コンテナを起動（バックグラウンド）
make up

# コンテナを停止
make down

# コンテナを再起動
make restart

# 完全リビルド（キャッシュなし）
make rebuild
```

### シェルアクセス

```bash
# ROS2コンテナに入る
make exec

# Flutterコンテナに入る
make shell-flutter
```

### ログ確認

```bash
# 全コンテナのログを表示（最新100行）
make logs

# 特定コンテナのログを表示
make logs SERVICE=ros2_container

# ログをリアルタイムで監視
make logs-f

# 特定コンテナのログをリアルタイム監視
make logs-f SERVICE=flutter
```

### ステータス確認

```bash
# Dockerデーモンの状態確認
make status

# 実行中のコンテナ一覧
make ps
```

### クリーンアップ

```bash
# コンテナ停止 + ボリューム削除
make clean

# 未使用のDocker全リソース削除（要確認）
make prune
```

### Git操作（Makefileから）

```bash
# 詳細なgitステータス表示
make git-status

# Git履歴グラフ表示
make git-graph
```

---

## 🔧 Git エイリアス（.git-aliases.sh）

### 使い方

Fish shellの場合、`.bashrc`や`.config/fish/config.fish`に以下を追加：

```fish
# Bash構文のエイリアスをFishで使えるようにする
source /home/tatsv/working/rodep-rescue2026/.git-aliases.sh
```

または、セッションごとに読み込む：

```bash
source .git-aliases.sh
```

### ステータス＆情報

- `gs` - コンパクトなステータス表示
- `gss` - 詳細なステータス
- `gl` - 最新20件のログをグラフ表示
- `gla` - 全ブランチのログをグラフ表示
- `glf` - 全履歴のグラフ表示

### ブランチ操作

- `gb` - ローカルブランチ一覧（詳細）
- `gba` - 全ブランチ一覧
- `gco <branch>` - ブランチ切り替え
- `gcb <branch>` - 新規ブランチ作成＆切り替え
- `gbd <branch>` - ブランチ削除
- `gbD <branch>` - 強制ブランチ削除

### コミット＆ステージング

- `ga <file>` - ファイルをステージング
- `gaa` - 全変更をステージング
- `gap` - インタラクティブステージング
- `gc` - コミット（エディタ起動）
- `gcm "message"` - メッセージ付きコミット
- `gca` - 最後のコミットを修正
- `gcan` - 最後のコミットを修正（メッセージそのまま）

### Diff

- `gd` - 作業ディレクトリの差分
- `gdc` - ステージングエリアの差分
- `gds` - ステージングエリアの差分（別名）

### プル＆プッシュ

- `gp` - プル
- `gpr` - リベース付きプル
- `gpu` - プッシュ
- `gpuf` - 安全な強制プッシュ
- `gpsu` - 現在のブランチをアップストリームに設定してプッシュ

### スタッシュ

- `gst` - 変更を一時保存
- `gsta` - スタッシュを適用
- `gstl` - スタッシュ一覧
- `gstp` - スタッシュを適用して削除
- `gstd` - スタッシュを削除

### 便利な関数

- `gch` - 変更されたファイル一覧
- `gundo` - 最後のコミットを取り消し（変更は保持）
- `gcq "message"` - 全変更をステージング＆コミット
- `gcnb <branch>` - mainから新ブランチ作成
- `gbclean` - マージ済みブランチを削除
- `glast` - 最後のコミットのファイル一覧
- `gstats` - リポジトリ統計情報

---

## 📁 .gitignore の改善点

以下のファイルタイプが無視されるようになりました：

- **ROS2**: `build/`, `install/`, `log/`, `*.bag`, `*.db3`
- **Python**: `__pycache__/`, `*.pyc`, `.egg-info/`
- **C/C++**: `*.o`, `*.a`, `*.so`
- **Flutter**: `.dart_tool/`, `build/`, `.flutter-plugins`
- **Docker**: `*.log`, `docker-compose.override.yml`
- **IDE**: `.vscode/`, `.idea/`, `*.swp`
- **その他**: `.DS_Store`, `.ccache/`, `node_modules/`

---

## 📦 .dockerignore の効果

Dockerビルド時に以下が除外され、ビルドが高速化されます：

- Gitファイル（`.git/`, `.gitignore`）
- ドキュメント（`*.md`, `docs/`）
- ビルド成果物（`build/`, `install/`, `log/`）
- キャッシュ（`.ccache/`, `__pycache__/`）
- IDE設定（`.vscode/`, `.idea/`）

---

## 🚀 クイックスタート例

### 開発開始

```bash
# コンテナをビルド＆起動
make build
make up

# ROS2コンテナに入る
make exec
```

### 作業中のログ確認

```bash
# ターミナルを別に開いて
make logs-f SERVICE=ros2_container
```

### Git作業（エイリアス使用）

```bash
# エイリアスを読み込み
source .git-aliases.sh

# 新機能のブランチを作成
gcnb feature/new-sensor

# コード変更後
gs                          # ステータス確認
gd                          # 差分確認
gaa                         # 全変更をステージング
gcm "Add new sensor support"  # コミット
gpu                         # プッシュ
```

### クリーンアップ

```bash
# コンテナ停止＆削除
make down

# 完全クリーンアップ（要注意）
make prune
```

---

## 💡 Tips

1. **`make help`** を実行すると全コマンドが表示されます
2. Gitエイリアスは **`alias | grep git`** で確認できます
3. `make logs SERVICE=<name>` で特定コンテナのログが見れます
4. `.git-aliases.sh` は自由にカスタマイズできます

---

## 🔄 今後の拡張案

- Docker Composeでの複数サービス管理の自動化
- CI/CDパイプライン用のMakeターゲット追加
- Git hooksの設定（pre-commit, pre-push）
- 開発環境のヘルスチェックスクリプト
