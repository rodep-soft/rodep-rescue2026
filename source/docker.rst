Docker
======

DESCRIPTION
-----------

基本的に開発はDocker-composeの使用を想定。
依存関係はrosdepで入らないものは書くDockerfileに追記していくことになる。

環境はx86_64 Ubuntu24.04で想定。yanoはArchlinux(rolling)で作業しているため、他のLinuxでも基本は問題ない。
Windowsを使う場合はWSL2を用いてdockerをCLIで使うことを勧める。Device/Networkの設定に注意。

CAUTION
--------

WSL2での動作は保証しない。デバイスを用いるときはusbipd等でセットアップすること。Waylandまわりも注意。



WORKFLOW
---------

.. code-block:: bash

   # プロジェクトルートにいると仮定
   
   # dockerイメージのビルド
   # 一部イメージだけビルドしたい場合は後述
   $ docker compose build

   # コンテナを立ち上げる
   $ docker compose up -d

   # コンテナが立ち上がっているか確認(マストではない)
   $ docker compose ps

   # ros2コンテナに入る。別のコンテナの場合は名前を変える。シェルの指定は必須。
   $ docker compose exec ros2_container bash

   # コンテナ内で出るときはexet
   $ exit

Dockerコンテナ内でバイナリインストール(apt install/pip install, etc.)したものはDockerfileにも記述すること。

その他の使うコマンド
--------------------

.. code-block:: bash

   # イメージの確認
   $ docker compose images

   # imageをDockerHubからpull
   $ docker pull <image>

   # ログを確認
   $ docker logs

Tips
------

コマンドが比較的長いので、makeやエイリアスをうまくつかう。
