import 'dart:async';
import 'package:gamepads/gamepads.dart';

class JoyMessage {
  List<double> axes;
  List<int> buttons;

  // 引数を必須にする
  JoyMessage({required this.axes, required this.buttons});

  @override
  String toString() => 'axes: $axes, buttons: $buttons';
}

StreamSubscription<GamepadEvent> startJoyPublisher(
  void Function(JoyMessage) onJoyUpdate,
) {
  final axes = List.filled(8, 0.0);
  final buttons = List.filled(14, 0);

  return Gamepads.events.listen((event) {
    final key = event.key;
    final value = event.value;

    final aIndex = _axisIndex(key);
    if (aIndex != null) {
      axes[aIndex] = value;
    }

    final bIndex = _buttonIndex(key);
    if (bIndex != null) {
      buttons[bIndex] = value > 0 ? 1 : 0;
    }

    onJoyUpdate(JoyMessage(
      axes: List.from(axes),
      buttons: List.from(buttons),
    ));
  });
}

// 名前の保証はない
// 軸マッピング (DualShock 4)
int? _axisIndex(String key) {
  const map = {
    'left_stick_x': 0,   // 左スティック X
    'left_stick_y': 1,   // 左スティック Y
    'right_stick_x': 2,  // 右スティック X
    'right_stick_y': 3,  // 右スティック Y
    'l2': 4,              // L2 トリガー
    'r2': 5,              // R2 トリガー
    'dpad_x': 6,          // 十字キー 左(-1)/右(1)
    'dpad_y': 7,          // 十字キー 下(-1)/上(1)
  };
  return map[key];
}

// ボタンマッピング (DualShock 4)
int? _buttonIndex(String key) {
  const map = {
    'cross': 0,   // ×
    'circle': 1,  // ○
    'square': 2,  // □
    'triangle': 3, // △
    'l1': 4,
    'r1': 5,
    'l2_button': 6,
    'r2_button': 7,
    'share': 8,
    'options': 9,
    'l3': 10,
    'r3': 11,
    'ps': 12,       // PSボタン
    'touchpad': 13, // タッチパッド押し込み
  };
  return map[key];
}
