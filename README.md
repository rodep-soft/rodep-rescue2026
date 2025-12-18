# What's this?

RobocupRescueの制御用repoです。

## 開発者用

### 作業時の注意

- mainにはrulesetで直接pushできないようになっています
- 必ずFlutter, microros, ros2の３つのCIをすべてpassしたことを確認してからmergeすること。即ち、PRを出さずに勝手にmergeしてはいけない。
- MainでCIに失敗したらhotfix/ブランチを切り早急に対処すること

### 作業環境

推奨の環境
- Host: Ubuntu24.04 (WSL2でも化だが、Device/Network周り注意)
- Virtual Env: Docker/Docker-Compose (Podman一部可/podman-composeはバグる)
- Shell: Bash
- CPU: x86_64 (armだとdocker compose buildでコケる)

Yano's setup:
- Host: ArchLinux(rolling) / WSL2 (less common)
- Virtual Env: Docker/Docker-Compose
- Shell: Fish (bass)
- CPU: x86_64




### Quick Start (Setup)

```bash
# 1. リポジトリをclone
git clone https://github.com/rodep-soft/rodep-rescue2026.git
cd rodep-rescue2026

# 2. 初回セットアップ（submodule取得 + 全コンテナをビルド + micro-ROS firmwareを展開）
make setup
```

### micro-ROS開発の場合
- **micro_ros_setup の編集**: `microros_ws/src/micro_ros_setup/` (submodule)
- **FreeRTOSアプリ編集**: `microros_ws/src/micro_ros_setup/freertos_apps/apps/`
- **コンテナアクセス**: `make shell-microros`
- **変更をforkに反映**:
  ```bash
  cd microros_ws/src/micro_ros_setup
  git add .
  git commit -m "update freertos app"
  git push origin jazzy
  ```
- 詳細は [docs/microros-nucleo-f446re.md](docs/microros-nucleo-f446re.md) を参照

### 
