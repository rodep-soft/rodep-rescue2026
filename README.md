# rodep-rescue2026

## Quick Start (初回セットアップ)

```bash
# 1. リポジトリをclone
git clone https://github.com/rodep-soft/rodep-rescue2026.git
cd rodep-rescue2026

# 2. 初回セットアップ（submodule取得 + 全コンテナをビルド + micro-ROS firmwareを展開）
make setup

# 3. コンテナ起動
make up

# 4. ROS2コンテナにアクセス
make exec
```

## 🤖 MoveIt2 & RViz2

```bash
# MoveIt2 + RViz2でシミュレーション
make moveit

# 実機制御（Dynamixelハードウェア）
make moveit-hw

# ジョイスティック制御
make moveit-joy

# RViz2のみ起動
make rviz
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

## Branch rules

- mainへのpushは不可能
- 作業時は必ずbranchを切って、PRを出すこと
- ciが通っていないのにmergeしない

## Directory explanation

- ros_ws -> メインのROS2 workspace
  - src -> nodeのpkgを配置
  - sandbox -> 試験的なコード(特にルールはなし)
- microros_ws -> micro-ROS開発環境（STM32 Nucleo F446RE用）
  - src/micro_ros_setup -> micro-ROS setup（submodule、forkを編集可能）
  - firmware -> ビルド成果物（make setupで展開、.gitignoreで除外）
- scripts -> モーターの単体テストやbash script
- docs -> Linuxのsetupや書き留
