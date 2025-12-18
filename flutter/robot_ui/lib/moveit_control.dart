import 'dart:convert';
import 'package:web_socket_channel/web_socket_channel.dart';

/// MoveIt control helper for sending commands via rosbridge
class MoveItController {
  final WebSocketChannel channel;
  String _actionGoalId = '';

  MoveItController(this.channel);

  /// Generate a unique ID for action goals
  String _generateGoalId() {
    return 'goal_${DateTime.now().millisecondsSinceEpoch}';
  }

  /// Move to a named pose (e.g., "home", "ready", "up", etc.)
  void moveToNamedPose(String poseName) {
    _actionGoalId = _generateGoalId();

    // Call the /move_action action with a named target
    final goalMsg = jsonEncode({
      'op': 'call_service',
      'service': '/move_group/set_named_target',
      'args': {
        'name': poseName,
      },
    });

    print('Moving to named pose: $poseName');
    channel.sink.add(goalMsg);
  }

  /// Alternative: Use MoveGroup action directly
  /// This sends a goal to /move_action action server
  void moveToNamedPoseViaAction(String poseName) {
    _actionGoalId = _generateGoalId();

    final goalMsg = jsonEncode({
      'op': 'send_goal',
      'action': '/move_action',
      'action_type': 'moveit_msgs/MoveGroup',
      'goal': {
        'request': {
          'group_name': 'sekirei_arm',
          'num_planning_attempts': 10,
          'allowed_planning_time': 5.0,
          'max_velocity_scaling_factor': 0.1,
          'max_acceleration_scaling_factor': 0.1,
          'goal_constraints': [
            {
              'name': poseName,
              'joint_constraints': [],
            }
          ],
        },
        'planning_options': {
          'plan_only': false,
          'planning_scene_diff': {
            'is_diff': true,
          },
        },
      },
    });

    print('Sending MoveGroup action goal for: $poseName');
    channel.sink.add(goalMsg);
  }

  /// Simpler approach: Publish to a custom topic that a ROS node listens to
  void requestNamedPose(String poseName) {
    final msg = jsonEncode({
      'op': 'publish',
      'topic': '/move_to_pose',
      'type': 'std_msgs/String',
      'msg': {
        'data': poseName,
      },
    });

    print('Requesting pose: $poseName');
    channel.sink.add(msg);
  }

  /// Move to specific joint angles (in radians)
  void moveToJointAngles(List<double> angles) {
    if (angles.length != 6) {
      print('Error: Expected 6 joint angles, got ${angles.length}');
      return;
    }

    final msg = jsonEncode({
      'op': 'publish',
      'topic': '/joint_command',
      'type': 'std_msgs/Float64MultiArray',
      'msg': {
        'data': angles,
      },
    });

    print('Moving to joint angles: $angles');
    channel.sink.add(msg);
  }

  /// Stop the robot (cancel current goal)
  void stopMotion() {
    final msg = jsonEncode({
      'op': 'call_service',
      'service': '/move_group/stop',
      'args': {},
    });

    print('Stopping motion');
    channel.sink.add(msg);
  }

  /// Get current joint states
  void subscribeToJointStates(Function(Map<String, dynamic>) callback) {
    // This would need to be handled in the main message listener
    final msg = jsonEncode({
      'op': 'subscribe',
      'topic': '/joint_states',
      'type': 'sensor_msgs/JointState',
    });

    channel.sink.add(msg);
  }
}

/// Available named poses for Sekirei arm
class NamedPoses {
  static const String home = 'home';
  static const String ready = 'ready';
  static const String up = 'up';
  static const String forward = 'forward';
  static const String compact = 'compact';
  static const String left = 'left';
  static const String right = 'right';
  static const String back = 'back';

  static const List<String> all = [
    home,
    ready,
    up,
    forward,
    compact,
    left,
    right,
    back,
  ];

  static const Map<String, String> descriptions = {
    home: 'All joints at 0° (neutral position)',
    ready: 'Ready position for operation',
    up: 'Arm pointing upward',
    forward: 'Arm extended forward',
    compact: 'Compact folded position',
    left: 'Arm pointing left',
    right: 'Arm pointing right',
    back: 'Arm pointing backward',
  };
}
