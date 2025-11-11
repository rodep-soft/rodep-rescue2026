# GDBデバッグガイド

このドキュメントでは、ROS2プロジェクトでGDBを使用したデバッグ方法について説明します。

## 前提条件

GDB (GNU Debugger) がインストールされていること：

```bash
sudo apt-get install gdb
```

GUI付きデバッグを行う場合は、xtermもインストール：

```bash
sudo apt-get install xterm
```

## デバッグシンボル付きビルド

GDBでデバッグするには、デバッグシンボル付きでビルドする必要があります。

### 方法1: Makefileを使用（推奨）

```bash
cd ros_ws

# デバッグビルド（最適化なし、デバッグシンボル付き）
make build-debug

# CMakeキャッシュをクリアしてデバッグビルド
make clean-debug
```

### 方法2: colconコマンド直接実行

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### その他のビルドオプション

```bash
# AddressSanitizer付き（メモリリーク検出）
make build-debug-asan

# 最適化あり + デバッグシンボル（パフォーマンステスト時）
make build-relwithdebinfo

# リリースビルド（最適化のみ）
make build-release
```

## デバッグ方法

### 1. 単一ノードのデバッグ

#### 方法A: Makefileを使用（簡単）

```bash
# 基本的な使い方
make debug-node PKG=sekirei_moveit_config NODE=moveit_bridge.py

# GUI付き（別ウィンドウでGDB起動）
make debug-node-gui PKG=sekirei_moveit_config NODE=moveit_bridge.py
```

#### 方法B: ros2 runで直接実行

```bash
# ターミナル内でGDB実行
ros2 run --prefix 'gdb -ex run --args' sekirei_moveit_config moveit_bridge.py

# xterm（別ウィンドウ）でGDB実行
ros2 run --prefix 'xterm -e gdb -ex run --args' sekirei_moveit_config moveit_bridge.py
```

### 2. Launch fileからのデバッグ

launch fileの中で特定のノードをデバッグする場合、`Node`の`prefix`パラメータを使用します。

#### GUI環境がある場合

```python
from launch_ros.actions import Node

start_moveit_bridge = Node(
    package='sekirei_moveit_config',
    executable='moveit_bridge.py',
    name='moveit_bridge',
    prefix=['xterm -e gdb -ex run --args'],  # GUIウィンドウで起動
    output='screen'
)
```

#### GUI環境がない場合（リモートSSHなど）

```python
start_moveit_bridge = Node(
    package='sekirei_moveit_config',
    executable='moveit_bridge.py',
    name='moveit_bridge',
    prefix=['gdb -ex run --args'],  # 同じターミナルで起動
    output='screen'
)
```

### 3. テストのデバッグ

```bash
# Makefileを使用
make debug-test TEST=./build/my_package/test/test_my_code

# または直接実行
source install/setup.bash
gdb -ex run ./build/my_package/test/test_my_code
```

例外をキャッチする場合：

```bash
gdb ./build/my_package/test/test_my_code
(gdb) catch throw
(gdb) run
```

## GDBの基本的な使い方

### プログラムがクラッシュした後

```gdb
(gdb) backtrace          # スタックトレースを表示
(gdb) bt                 # 上記の短縮形
(gdb) frame 5            # フレーム5に移動
(gdb) list               # 現在のコードを表示
(gdb) print variable     # 変数の値を表示
(gdb) quit               # GDBを終了
```

### ブレークポイントを使用

```bash
# GDBを起動（自動実行なし）
ros2 run --prefix 'gdb --args' sekirei_moveit_config moveit_bridge.py
```

GDB内で：

```gdb
(gdb) break moveit_bridge.py:42    # 42行目にブレークポイント
(gdb) break MyClass::myMethod      # 特定のメソッドにブレークポイント
(gdb) run                          # プログラムを実行
(gdb) continue                     # 次のブレークポイントまで実行
(gdb) step                         # 1行ずつ実行（関数内に入る）
(gdb) next                         # 1行ずつ実行（関数をスキップ）
(gdb) finish                       # 現在の関数が終わるまで実行
```

### 変数の監視

```gdb
(gdb) print my_variable            # 変数の値を表示
(gdb) print *my_pointer            # ポインタが指す値を表示
(gdb) display my_variable          # ステップごとに自動表示
(gdb) info locals                  # すべてのローカル変数を表示
(gdb) info args                    # 関数の引数を表示
```

## バックトレースの読み方

バックトレースは**下から上**に読みます：

```
#0  __GI_raise (sig=sig@entry=6) at ../sysdeps/unix/sysv/linux/raise.c:50
#1  0x00007ffff79cc859 in __GI_abort () at abort.c:79
#2  0x00007ffff7c52951 in ?? () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
#3  0x000055555555936c in std::vector<int>::_M_range_check (this=0x5555555cfdb0, __n=100)
    at /usr/include/c++/9/bits/stl_vector.h:1070
#4  0x0000555555558e1d in std::vector<int>::at (this=0x5555555cfdb0, __n=100)
    at /usr/include/c++/9/bits/stl_vector.h:1091
#5  0x000055555555828b in MyClass::VectorCrash (this=0x5555555cfb40)
    at /path/to/my_code.cpp:44
#6  0x0000555555559cfc in main (argc=1, argv=0x7fffffffc108)
    at /path/to/main.cpp:25
```

**読み方：**

1. **#6**: `main()` 関数の25行目から開始
2. **#5**: `MyClass::VectorCrash()` が44行目で呼ばれた
3. **#4**: `vector::at()` がインデックス100でアクセスを試みた
4. **#3**: 範囲チェックで失敗
5. **#0-#2**: システムがクラッシュを処理

**結論**: `my_code.cpp`の44行目で、vectorのサイズを超えたインデックス100にアクセスしようとしてクラッシュ。

## Docker環境でのデバッグ

### X11転送の設定

```bash
# ホストマシンで
xhost +local:docker
```

docker-compose.ymlに以下を追加：

```yaml
services:
  ros2_container:
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
```

### コンテナ内でのデバッグ

```bash
# コンテナに入る
docker-compose exec ros2_container bash

# デバッグビルド
cd /root/ros_ws
make build-debug

# ノードをデバッグ
make debug-node PKG=sekirei_moveit_config NODE=moveit_bridge.py
```

## よくある問題と解決方法

### 問題1: デバッグシンボルが見つからない

**症状**: 行番号が表示されず、`??` が多い

**解決**:
```bash
# CMakeキャッシュをクリアして再ビルド
make clean-debug
```

### 問題2: GDBが "No such file or directory"

**症状**: `ros2` コマンドが見つからない

**解決**: `--prefix` を使用する（上記の例を参照）

### 問題3: 複数ノードのlaunch fileでログが見づらい

**解決**: デバッグしたいノードだけを別ターミナルで起動：

```bash
# ターミナル1: 他のノードを起動
ros2 launch my_package main.launch.py

# ターミナル2: デバッグ対象ノードを起動
make debug-node PKG=my_package NODE=problem_node
```

### 問題4: Pythonノードのデバッグ

C++ノードと同じ方法が使えますが、Pythonの場合は `pdb` の方が適している場合があります：

```bash
# Python デバッガー
python3 -m pdb /path/to/script.py
```

## 高度な機能

### AddressSanitizer（メモリリーク検出）

```bash
# ASan付きビルド
make build-debug-asan

# 通常通り実行（クラッシュ時に詳細なレポート表示）
ros2 run my_package my_node
```

### Automatic Backtrace（backward-cpp）

`package.xml`に依存を追加：

```xml
<depend>backward_ros</depend>
```

`CMakeLists.txt`に追加：

```cmake
find_package(backward_ros REQUIRED)
```

これでクラッシュ時に自動的に美しいスタックトレースが表示されます。

## 参考資料

- [ROS 2公式: Getting Backtraces](https://docs.ros.org/en/humble/How-To-Guides/Getting-Backtraces-in-ROS-2.html)
- [GDB Documentation](https://sourceware.org/gdb/documentation/)
- [backward-cpp GitHub](https://github.com/bombela/backward-cpp)

## 便利なGDBコマンドチートシート

```gdb
# 実行制御
run                  # プログラム開始
continue (c)         # 実行継続
next (n)            # 次の行（関数をスキップ）
step (s)            # 次の行（関数内に入る）
finish              # 現在の関数から抜ける
quit (q)            # GDB終了

# ブレークポイント
break file.cpp:42   # 42行目にブレーク
break functionName  # 関数にブレーク
info breakpoints    # ブレークポイント一覧
delete 1            # ブレークポイント1を削除
clear               # すべてのブレークポイントをクリア

# 変数表示
print var           # 変数を表示
display var         # 自動表示
info locals         # ローカル変数すべて
info args           # 関数引数すべて
ptype var           # 変数の型を表示

# スタック
backtrace (bt)      # スタックトレース
frame 5             # フレーム5に移動
up                  # 上のフレームへ
down                # 下のフレームへ

# その他
list                # ソースコード表示
info threads        # スレッド一覧
thread 2            # スレッド2に切り替え
```
