#!/bin/bash
# Flutter リリースモードで実行（デバッグモードより速い）
# 画像表示してるときとか特にdebugだと厳しい
# テストだったら--profileでもいいかも

cd robot_ui
flutter run -d linux --release
