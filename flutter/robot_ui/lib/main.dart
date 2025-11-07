import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'ROS2 Robot UI',
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.deepPurple),
        useMaterial3: true,
      ),
      home: const ROS2HomePage(title: 'ROS2 Robot Control'),
    );
  }
}

class ROS2HomePage extends StatefulWidget {
  const ROS2HomePage({super.key, required this.title});

  final String title;

  @override
  State<ROS2HomePage> createState() => _ROS2HomePageState();
}

// Separate widget for image display to reduce flickering
class CameraImageWidget extends StatefulWidget {
  final Stream<Uint8List?> imageStream;

  const CameraImageWidget({super.key, required this.imageStream});

  @override
  State<CameraImageWidget> createState() => _CameraImageWidgetState();
}

class _CameraImageWidgetState extends State<CameraImageWidget> {
  Uint8List? _currentImage;
  String _imageStatus = 'No image received';

  @override
  void initState() {
    super.initState();
    widget.imageStream.listen((imageData) {
      if (mounted && imageData != null) {
        setState(() {
          _currentImage = imageData;
          _imageStatus = 'Image received (${imageData.length} bytes)';
        });
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    return Card(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Padding(
            padding: const EdgeInsets.all(8.0),
            child: Row(
              children: [
                Icon(Icons.camera_alt, color: Colors.green.shade700),
                const SizedBox(width: 8),
                const Text(
                  'Camera Feed (/image_raw/compressed):',
                  style: TextStyle(
                    fontSize: 16,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ],
            ),
          ),
          Padding(
            padding: const EdgeInsets.all(8.0),
            child: Text(
              _imageStatus,
              style: TextStyle(
                fontSize: 12,
                color: Colors.grey.shade700,
              ),
            ),
          ),
          if (_currentImage != null)
            Padding(
              padding: const EdgeInsets.all(8.0),
              child: Center(
                child: Image.memory(
                  _currentImage!,
                  fit: BoxFit.contain,
                  gaplessPlayback: true, // Prevents flickering during updates
                  errorBuilder: (context, error, stackTrace) {
                    return Text('Error displaying image: $error');
                  },
                ),
              ),
            )
          else
            const Padding(
              padding: EdgeInsets.all(32.0),
              child: Center(
                child: Column(
                  children: [
                    Icon(Icons.image_not_supported, size: 48, color: Colors.grey),
                    SizedBox(height: 8),
                    Text('Waiting for image...'),
                  ],
                ),
              ),
            ),
        ],
      ),
    );
  }
}

class _ROS2HomePageState extends State<ROS2HomePage> {
  WebSocketChannel? _channel;
  String _rosStatus = 'Disconnected';
  String _lastMessage = 'No data received';
  String _talkerMessage = 'Waiting for talker...';
  bool _isConnected = false;
  final List<String> _messageHistory = [];
  final Map<String, List<String>> _customTopicMessages = {};
  final List<String> _subscribedTopics = [];

  // WebSocket address configuration
  String _wsAddress = 'localhost';
  String _wsPort = '9090';

  // Controllers for custom topic subscription
  final TextEditingController _topicController = TextEditingController();
  final TextEditingController _messageTypeController = TextEditingController();
  final TextEditingController _wsAddressController = TextEditingController();
  final TextEditingController _wsPortController = TextEditingController();

  // Stream controller for image data
  final _imageStreamController = StreamController<Uint8List?>.broadcast();

  @override
  void initState() {
    super.initState();
    _wsAddressController.text = _wsAddress;
    _wsPortController.text = _wsPort;
    _connectToROS();
  }

  void _connectToROS() {
    try {
      // Connect to rosbridge_server with configurable address
      final wsUrl = 'ws://$_wsAddress:$_wsPort';
      _channel = WebSocketChannel.connect(
        Uri.parse(wsUrl),
      );

      setState(() {
        _rosStatus = 'Connected';
        _isConnected = true;
      });

      // Subscribe to the talker topic
      Future.delayed(const Duration(milliseconds: 500), () {
        _subscribeToTopic('/chatter', 'std_msgs/String');
        // Subscribe to compressed image topic
        _subscribeToTopic('/image_raw/compressed', 'sensor_msgs/CompressedImage');
      });

      // Listen to incoming messages
      _channel!.stream.listen(
        (message) {
          setState(() {
            _lastMessage = message;
          });
          _handleROSMessage(message);
        },
        onError: (error) {
          setState(() {
            _rosStatus = 'Error: $error';
            _isConnected = false;
          });
        },
        onDone: () {
          setState(() {
            _rosStatus = 'Disconnected';
            _isConnected = false;
          });
        },
      );
    } catch (e) {
      setState(() {
        _rosStatus = 'Connection failed: $e';
        _isConnected = false;
      });
    }
  }

  void _handleROSMessage(dynamic message) {
    try {
      final data = jsonDecode(message);

      // Check if it's a message from the /chatter topic
      if (data['topic'] == '/chatter' && data['msg'] != null) {
        final talkerData = data['msg']['data'];
        setState(() {
          _talkerMessage = talkerData;
          _messageHistory.insert(0, talkerData);
          if (_messageHistory.length > 10) {
            _messageHistory.removeLast();
          }
        });
      }

      // Check if it's a compressed image message
      if (data['topic'] == '/image_raw/compressed' && data['msg'] != null) {
        try {
          final compressedData = data['msg']['data'];
          if (compressedData != null && compressedData is String) {
            // Decode base64 string to bytes
            final bytes = base64Decode(compressedData);
            // Send image to stream instead of setState
            _imageStreamController.add(bytes);
          }
        } catch (e) {
          print('Error decoding image: $e');
        }
      }

      // Handle custom subscribed topics
      if (data['topic'] != null && _subscribedTopics.contains(data['topic'])) {
        final topic = data['topic'] as String;
        final msgData = data['msg'];
        if (msgData != null) {
          setState(() {
            if (!_customTopicMessages.containsKey(topic)) {
              _customTopicMessages[topic] = [];
            }
            _customTopicMessages[topic]!.insert(0, jsonEncode(msgData));
            if (_customTopicMessages[topic]!.length > 20) {
              _customTopicMessages[topic]!.removeLast();
            }
          });
        }
      }

      print('Received ROS message: $data');
    } catch (e) {
      print('Error parsing message: $e');
    }
  }

  void _subscribeToTopic(String topic, String messageType) {
    if (_channel == null) return;

    final subscribeMsg = jsonEncode({
      'op': 'subscribe',
      'topic': topic,
      'type': messageType,
    });

    _channel!.sink.add(subscribeMsg);
  }

  void _publishToTopic(String topic, String messageType, Map<String, dynamic> msg) {
    if (_channel == null) return;

    final publishMsg = jsonEncode({
      'op': 'publish',
      'topic': topic,
      'type': messageType,
      'msg': msg,
    });

    _channel!.sink.add(publishMsg);
  }

  void _advertiseService(String service, String serviceType) {
    if (_channel == null) return;

    final advertiseMsg = jsonEncode({
      'op': 'advertise_service',
      'service': service,
      'type': serviceType,
    });

    _channel!.sink.add(advertiseMsg);
  }

  // Example: Subscribe to a topic
  void _subscribeToCmdVel() {
    _subscribeToTopic('/cmd_vel', 'geometry_msgs/Twist');
  }

  // Subscribe to custom topic
  void _subscribeToCustomTopic() {
    final topic = _topicController.text.trim();
    final messageType = _messageTypeController.text.trim();

    if (topic.isEmpty || messageType.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('Please enter topic name and message type')),
      );
      return;
    }

    if (_subscribedTopics.contains(topic)) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Already subscribed to $topic')),
      );
      return;
    }

    _subscribeToTopic(topic, messageType);
    setState(() {
      _subscribedTopics.add(topic);
    });

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(content: Text('Subscribed to $topic ($messageType)')),
    );
  }

  // Unsubscribe from custom topic
  void _unsubscribeFromTopic(String topic) {
    if (_channel == null) return;

    final unsubscribeMsg = jsonEncode({
      'op': 'unsubscribe',
      'topic': topic,
    });

    _channel!.sink.add(unsubscribeMsg);

    setState(() {
      _subscribedTopics.remove(topic);
      _customTopicMessages.remove(topic);
    });

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(content: Text('Unsubscribed from $topic')),
    );
  }

  // Example: Publish a message
  void _publishTestMessage() {
    _publishToTopic(
      '/test_topic',
      'std_msgs/String',
      {'data': 'Hello from Flutter!'},
    );
  }

  @override
  void dispose() {
    _topicController.dispose();
    _messageTypeController.dispose();
    _wsAddressController.dispose();
    _wsPortController.dispose();
    _imageStreamController.close();
    _channel?.sink.close();
    super.dispose();
  }

  void _showSettingsDialog() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('WebSocket Settings'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            TextField(
              controller: _wsAddressController,
              decoration: const InputDecoration(
                labelText: 'WebSocket Address',
                hintText: 'localhost or IP address',
                prefixIcon: Icon(Icons.computer),
              ),
            ),
            const SizedBox(height: 12),
            TextField(
              controller: _wsPortController,
              decoration: const InputDecoration(
                labelText: 'WebSocket Port',
                hintText: '9090',
                prefixIcon: Icon(Icons.settings_ethernet),
              ),
              keyboardType: TextInputType.number,
            ),
            const SizedBox(height: 16),
            Text(
              'Current: ws://$_wsAddress:$_wsPort',
              style: TextStyle(
                fontSize: 12,
                color: Colors.grey.shade600,
              ),
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              setState(() {
                _wsAddress = _wsAddressController.text.trim();
                _wsPort = _wsPortController.text.trim();
                _isConnected = false;
                _rosStatus = 'Disconnected';
              });
              _channel?.sink.close();
              Navigator.pop(context);
              _connectToROS();
            },
            child: const Text('Connect'),
          ),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Text(widget.title),
        actions: [
          IconButton(
            icon: const Icon(Icons.settings),
            onPressed: _showSettingsDialog,
            tooltip: 'WebSocket Settings',
          ),
        ],
      ),
      body: SingleChildScrollView(
        child: Padding(
          padding: const EdgeInsets.all(16.0),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
            // Connection Status
            Card(
              child: ListTile(
                leading: Icon(
                  _isConnected ? Icons.check_circle : Icons.error,
                  color: _isConnected ? Colors.green : Colors.red,
                ),
                title: const Text('ROS2 Bridge Status'),
                subtitle: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(_rosStatus),
                    const SizedBox(height: 4),
                    Text(
                      'ws://$_wsAddress:$_wsPort',
                      style: TextStyle(
                        fontSize: 12,
                        color: Colors.grey.shade600,
                      ),
                    ),
                  ],
                ),
                trailing: IconButton(
                  icon: const Icon(Icons.refresh),
                  onPressed: !_isConnected ? _connectToROS : null,
                  tooltip: 'Reconnect',
                ),
              ),
            ),
            const SizedBox(height: 20),

            // Camera Image
            CameraImageWidget(imageStream: _imageStreamController.stream),
            const SizedBox(height: 20),

            // Talker Message
            Card(
              color: Colors.blue.shade50,
              child: Padding(
                padding: const EdgeInsets.all(16.0),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Row(
                      children: [
                        Icon(Icons.message, color: Colors.blue.shade700),
                        const SizedBox(width: 8),
                        const Text(
                          'Talker Message (/chatter):',
                          style: TextStyle(
                            fontSize: 16,
                            fontWeight: FontWeight.bold,
                          ),
                        ),
                      ],
                    ),
                    const SizedBox(height: 8),
                    Text(
                      _talkerMessage,
                      style: const TextStyle(
                        fontSize: 18,
                        fontWeight: FontWeight.w500,
                      ),
                    ),
                  ],
                ),
              ),
            ),
            const SizedBox(height: 20),

            // Message History
            const Text(
              'Message History:',
              style: TextStyle(
                fontSize: 16,
                fontWeight: FontWeight.bold,
              ),
            ),
            const SizedBox(height: 8),
            SizedBox(
              height: 300,
              child: Card(
                child: _messageHistory.isEmpty
                    ? const Center(child: Text('No messages yet'))
                    : ListView.builder(
                        itemCount: _messageHistory.length,
                        itemBuilder: (context, index) {
                          return ListTile(
                            leading: CircleAvatar(
                              child: Text('${index + 1}'),
                            ),
                            title: Text(_messageHistory[index]),
                            dense: true,
                          );
                        },
                      ),
              ),
            ),
            const SizedBox(height: 20),

            // Custom Topic Subscription
            Card(
              color: Colors.green.shade50,
              child: Padding(
                padding: const EdgeInsets.all(16.0),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Row(
                      children: [
                        Icon(Icons.subscriptions, color: Colors.green.shade700),
                        const SizedBox(width: 8),
                        const Text(
                          'Custom Topic Subscription:',
                          style: TextStyle(
                            fontSize: 16,
                            fontWeight: FontWeight.bold,
                          ),
                        ),
                      ],
                    ),
                    const SizedBox(height: 12),
                    TextField(
                      controller: _topicController,
                      decoration: const InputDecoration(
                        labelText: 'Topic Name',
                        hintText: '/my_topic',
                        border: OutlineInputBorder(),
                        prefixIcon: Icon(Icons.topic),
                      ),
                    ),
                    const SizedBox(height: 8),
                    TextField(
                      controller: _messageTypeController,
                      decoration: const InputDecoration(
                        labelText: 'Message Type',
                        hintText: 'std_msgs/String',
                        border: OutlineInputBorder(),
                        prefixIcon: Icon(Icons.code),
                      ),
                    ),
                    const SizedBox(height: 8),
                    ElevatedButton.icon(
                      onPressed: _isConnected ? _subscribeToCustomTopic : null,
                      icon: const Icon(Icons.add),
                      label: const Text('Subscribe'),
                      style: ElevatedButton.styleFrom(
                        minimumSize: const Size.fromHeight(40),
                      ),
                    ),
                  ],
                ),
              ),
            ),
            const SizedBox(height: 20),

            // Subscribed Topics List
            if (_subscribedTopics.isNotEmpty) ...[
              const Text(
                'Subscribed Topics:',
                style: TextStyle(
                  fontSize: 16,
                  fontWeight: FontWeight.bold,
                ),
              ),
              const SizedBox(height: 8),
              ...(_subscribedTopics.map((topic) => Card(
                    child: ExpansionTile(
                      title: Text(topic),
                      leading: const Icon(Icons.topic),
                      trailing: IconButton(
                        icon: const Icon(Icons.close),
                        onPressed: () => _unsubscribeFromTopic(topic),
                      ),
                      children: [
                        Container(
                          constraints: const BoxConstraints(maxHeight: 200),
                          child: _customTopicMessages[topic]?.isEmpty ?? true
                              ? const Padding(
                                  padding: EdgeInsets.all(16.0),
                                  child: Text('No messages yet'),
                                )
                              : ListView.builder(
                                  shrinkWrap: true,
                                  itemCount: _customTopicMessages[topic]!.length,
                                  itemBuilder: (context, index) {
                                    return ListTile(
                                      dense: true,
                                      title: Text(
                                        _customTopicMessages[topic]![index],
                                        style: const TextStyle(fontSize: 12),
                                        maxLines: 3,
                                        overflow: TextOverflow.ellipsis,
                                      ),
                                    );
                                  },
                                ),
                        ),
                      ],
                    ),
                  ))),
              const SizedBox(height: 20),
            ],

            // Action Buttons
            Wrap(
              spacing: 10,
              runSpacing: 10,
              children: [
                ElevatedButton.icon(
                  onPressed: _isConnected
                      ? () => _subscribeToTopic('/chatter', 'std_msgs/String')
                      : null,
                  icon: const Icon(Icons.play_arrow),
                  label: const Text('Subscribe Talker'),
                ),
                ElevatedButton.icon(
                  onPressed: _isConnected
                      ? () => _subscribeToTopic('/image_raw/compressed', 'sensor_msgs/CompressedImage')
                      : null,
                  icon: const Icon(Icons.camera),
                  label: const Text('Subscribe Image'),
                ),
                ElevatedButton.icon(
                  onPressed: _isConnected ? _publishTestMessage : null,
                  icon: const Icon(Icons.send),
                  label: const Text('Publish Test'),
                ),
                ElevatedButton.icon(
                  onPressed: !_isConnected ? _connectToROS : null,
                  icon: const Icon(Icons.refresh),
                  label: const Text('Reconnect'),
                ),
                ElevatedButton.icon(
                  onPressed: () {
                    setState(() {
                      _messageHistory.clear();
                    });
                  },
                  icon: const Icon(Icons.clear),
                  label: const Text('Clear History'),
                ),
              ],
            ),
          ],
        ),
        ),
      ),
    );
  }
}
