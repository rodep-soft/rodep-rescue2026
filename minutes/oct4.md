# 議事録　Oct 4

## 先週の進捗

### やの
- USBカメラ/imageに変換してrqtで表示
- 画像の圧縮解凍
- githubのrepoのsetup

### 山田
- Flipperのノードを書いた
- Moveitをバイナリでインストールして、tutorialをすすめられる状況にした


## 問題点
- エンドエフェクタのカメラで試すとこう確率でうまくいかない
    - docker-composeのマウントがちゃんとできてない、権限？

- Gazebo, moveit2!のrziv2でのシミュレーションがあまりにも重い
    - 部室ではなるべくUbuntuのPCを使うようにするかな


## 来週のタスク

### やの

- USBカメラの圧縮、表示、qrコード読み取り
- Image_tools やめて自作ノード
- Robot_localizationの確認、導入
- Ros2振り返り
- 環境周りの話をどっかにかく
- 前回の足回りmaxonのコードを移植

### 山田
- Moveitのチュートリアルを進める
- docker/composeの動作確認と、moveitを上にいれてその動作確認もする
    - 確認できたらDockerfileに追記
- Ros2_control, urdf, tf

#### 備考
- GUIは11月入ったら手をつけるつもり
- Moveitのc++編10月中

