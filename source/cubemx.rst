STM32CubeMX
===========

Description
-----------

MXはSTM32(マイコン)のピンやクロックの設定等ができるツール。

CubeIDEを使えば統合開発環境が使えるが、yanoは使ってない。

Installation
------------

STMicroの公式サイトからダウンロードできる。結構重い。

`STM32CubeMX <https://www.st.com/en/development-tools/stm32cubemx.html>`_

ArchLinuxを使っていればAURで入る(多分)

`AUR repo <https://aur.archlinux.org/packages/stm32cubemx>`_

.. code-block:: bash

   # 例
   $ paru -S stm32cubemx


どこからでも起動できるようにする
-------------

普通に入れると/home/user/STM32CubeMXに配置されるが、いちいち移動するのも不便。
しかし、ln -sで実行ファイルのsymlinkを/usr/local/binに張るだけでは動かないので、別の方法をとる。
以下は一例。

.. code-block:: bash

   $ sudo mv ~/STM32CubeMX /opt/
   $ sudo vim /usr/local/bin/mx.sh

   # mx.shの中身
   # #!/usr/bin/bash
   # cd /opt/STM32CubeMX || exit 1
   # exec ./STM32CubeMX "$@"

   $ sudo chmod +x /usr/local/bin/mx.sh
   
   # テスト
   $ mx.sh 
    



